"""Live cameras as camera streams: ``xblick cam``. docs/guides/cam.md has the use.

Any camera FFmpeg opens, through PyAV (the extra ``xblick[cam]``): a V4L2 device (``/dev/video0``: webcams, capture
cards), a URL (``rtsp://``, ``http://``), or another FFmpeg input format named with ``--format`` (``lavfi``: a test
source). It becomes one stream as a camera publishes it: ``<base>/image_raw/...`` with ``camera_info``, diagnostics
on ``<camera>/diagnostics``, the info record with source ``cam``, its options as the parameters of the node
``<camera>``.

The size, rate and pixel format asked for go to FFmpeg as the device options ``video_size``, ``framerate`` and
(V4L2) ``input_format``. A device may change them, and a URL or a filter graph ignores them: what the first frame
and the stream deliver is checked against what was asked, and a difference ends the program (CamError). A frame's
stamp is the arrival of its packet from FFmpeg on this PC (wall clock). A camera that delivers no frame for
``timeout`` seconds, or whose stream ends, ends the program with an error: there is no reconnect.

FFmpeg reads and decodes in a daemon thread of its own. The main thread waits, reports and stops: Ctrl-C (SIGTERM)
closes the publisher at once (its ring goes with it), also while the thread is blocked in FFmpeg.
"""

from __future__ import annotations

import argparse
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import zenoh

from .camera import CameraPublisher
from .codecs import JPEG, RVL
from .node import Reporter, state
from .video import VideoError, camera_info_for, camera_of, parse_size

SOURCE = "cam"  # the info record's source
V4L2 = "video4linux2"
URL_SCHEMES = ("rtsp", "rtsps", "rtmp", "http", "https", "udp", "tcp", "srt")
RATE_TOLERANCE = 0.01  # relative: a rate asked for and the one delivered agree within this
# decoded frame format -> (format to convert to, the stream's encoding): as video.VideoFile; everything else rgb24
FORMATS = {"gray16le": ("gray16le", "16UC1"), "gray16be": ("gray16le", "16UC1"), "gray": ("gray", "mono8"),
           "gray8": ("gray", "mono8")}


class CamError(ValueError):
    """A camera that cannot be opened, that delivers another mode than the one asked for, or that stopped."""


def import_av():
    try:
        import av
    except ImportError:
        raise ImportError("xblick cam needs PyAV: install xblick[cam] (pip install 'xblick[cam]', "
                          "or uv sync --extra cam)") from None
    return av


def input_format(source: str, given: str | None) -> str | None:
    """The FFmpeg input format of ``source``: ``given`` (``--format``), V4L2 for a path below /dev, None for a URL
    (FFmpeg takes its protocol)."""
    if given:
        return given
    if source.startswith("/dev/"):
        return V4L2
    scheme, sep, _ = source.partition("://")
    if sep and scheme.lower() in URL_SCHEMES:
        return None
    if Path(source).is_file():
        raise CamError(f"{source}: a file. xblick play plays video files")
    raise CamError(f"{source}: a V4L2 device (/dev/video0), a URL ({', '.join(s + '://' for s in URL_SCHEMES)}), "
                   "or another FFmpeg input with --format (e.g. --format lavfi testsrc)")


def v4l2_devices() -> list[str]:
    return sorted(str(p) for p in Path("/dev").glob("video*"))


def device_options(fmt: str | None, size: tuple[int, int] | None, fps: float | None,
                   pixel_format: str | None) -> dict:
    """FFmpeg's options for the mode asked for (none given: none, the device's own mode)."""
    options = {}
    if size is not None:
        options["video_size"] = f"{size[0]}x{size[1]}"
    if fps is not None:
        if not fps > 0:
            raise CamError(f"--fps {fps:g}: above 0")
        options["framerate"] = f"{fps:g}"
    if pixel_format is not None:
        if fmt != V4L2:
            raise CamError(f"--pixel-format {pixel_format}: for V4L2 devices (FFmpeg's input_format); "
                           f"this source is {fmt or 'a URL'}")
        options["input_format"] = pixel_format
    return options


@dataclass
class Mode:
    """What a camera delivers, or what is asked of it (None: anything)."""

    size: tuple[int, int] | None = None
    fps: float | None = None
    pixel_format: str | None = None  # V4L2's input format: a codec (mjpeg, h264) or a raw pixel format (yuyv422)

    def text(self) -> str:
        parts = [f"{self.size[0]}x{self.size[1]}" if self.size else "",
                 self.pixel_format or "", f"at {self.fps:g} frames/s" if self.fps else "at an unknown rate"]
        return " ".join(p for p in parts if p)


def check_mode(source: str, asked: Mode, delivered: Mode):
    """CamError naming what was asked and what the camera delivers, unless it delivers what was asked."""
    wrong = []
    if asked.size is not None and asked.size != delivered.size:
        wrong.append(f"{asked.size[0]}x{asked.size[1]}")
    if asked.fps is not None and (delivered.fps is None
                                  or abs(delivered.fps - asked.fps) > RATE_TOLERANCE * asked.fps):
        wrong.append(f"{asked.fps:g} frames/s")
    if asked.pixel_format is not None and asked.pixel_format != delivered.pixel_format:
        wrong.append(asked.pixel_format)
    if wrong:
        raise CamError(f"{source}: asked for {' '.join(wrong)}, the camera delivers {delivered.text()}")


class LiveCamera:
    """One live camera published as the stream ``base`` (module docstring). :meth:`run` blocks until :meth:`stop`
    or an error (CamError, raised in the calling thread)."""

    def __init__(self, session: zenoh.Session, source: str, base: str, *, fmt: str | None = None,
                 size: tuple[int, int] | None = None, fps: float | None = None, pixel_format: str | None = None,
                 camera_info: str | None = None, hfov: float | None = None, vfov: float | None = None,
                 frame_id: str | None = None, timeout: float = 5.0, prog: str = "xblick cam"):
        self.av = import_av()
        self.session, self.source, self.base, self.prog = session, source, base, prog
        self.camera = camera_of(base)
        self.fmt = input_format(source, fmt)
        self.asked = Mode(size, fps, pixel_format)
        self.options = device_options(self.fmt, size, fps, pixel_format)
        if not timeout > 0:
            raise CamError(f"--timeout {timeout:g}: above 0")
        self.timeout = timeout
        self.frame_id = frame_id or base.replace("/", "_")
        self.calibration, self.hfov, self.vfov = camera_info, hfov, vfov
        if self.fmt == V4L2 and not Path(source).exists():
            devices = v4l2_devices()
            raise CamError(f"{source}: no such device (V4L2 devices on this PC: {', '.join(devices) or 'none'})")
        self.publisher: CameraPublisher | None = None
        self.delivered: Mode | None = None
        self.error: BaseException | None = None
        self.frames = 0
        self.decode_s = 0.0  # summed over the frames: from the packet's arrival to the array
        self._last_frame = time.monotonic()
        self._lock = threading.Lock()  # publish against close
        self._stopping = False
        self._done = threading.Event()
        self._ready = threading.Event()
        self._thread = threading.Thread(target=self._capture, name=f"cam {base}", daemon=True)

    # -------------------------------------------------------------------------------------------- the thread

    def _open(self):
        try:
            return self.av.open(self.source, format=self.fmt, container_options=self.options)  # not the decoder
        except self.av.FFmpegError as error:
            asked = f" with {self.asked.text()}" if self.options else ""
            raise CamError(f"{self.source}: FFmpeg cannot open it{asked} ({error})") from None

    def _capture(self):
        """Opens the camera, checks its mode, publishes every frame until stopped; an error ends it (``error``)."""
        container = None
        try:
            container = self._open()
            if not container.streams.video:
                raise CamError(f"{self.source}: no video stream")
            stream = container.streams.video[0]
            context = stream.codec_context
            rate = stream.average_rate or stream.guessed_rate
            raw = context.name in ("rawvideo", "wrapped_avframe")  # no codec: the pixel format is the mode
            for packet in container.demux(stream):
                arrived = time.time_ns()
                started = time.monotonic()
                for frame in packet.decode():
                    if self.delivered is None:  # the first frame: the camera's mode, checked; then the publisher
                        self.delivered = Mode((frame.width, frame.height), float(rate) if rate else None,
                                              context.pix_fmt if raw else context.name)
                        check_mode(self.source, self.asked, self.delivered)
                        self._start_publisher(frame.format.name)
                    elif (frame.width, frame.height) != self.delivered.size:
                        raise CamError(f"{self.source}: the camera changed its frame size from "
                                       f"{self.delivered.size[0]}x{self.delivered.size[1]} to "
                                       f"{frame.width}x{frame.height}")
                    image = frame.to_ndarray(format=self._to)
                    with self._lock:
                        if self._stopping:
                            return
                        self.publisher.publish(image, stamp_ns=arrived)
                        self.frames += 1
                        self.decode_s += time.monotonic() - started
                        self._last_frame = time.monotonic()
            raise CamError(f"{self.source}: the stream ended")
        except self.av.FFmpegError as error:  # unplugged, a connection lost
            self.error = CamError(f"{self.source}: the camera stopped delivering ({error})")
        except BaseException as error:  # noqa: BLE001 - raised by run() in the main thread
            self.error = error
        finally:
            self._done.set()
            if container is not None:
                container.close()

    def _start_publisher(self, pixel_format: str):
        self._to, encoding = FORMATS.get(pixel_format, ("rgb24", "rgb8"))
        info, said = camera_info_for(self.delivered.size[0], self.delivered.size[1], self.frame_id,
                                     self.calibration, self.hfov, self.vfov)
        with self._lock:
            if self._stopping:
                return
            self.publisher = CameraPublisher(self.session, self.base, encoding=encoding, frame_id=self.frame_id,
                                             camera_info=info, source=SOURCE,
                                             codec=RVL if encoding == "16UC1" else JPEG)
        self.encoding = encoding
        print(f"{self.prog}: {self.source} -> {self.base}: {self.delivered.text()}, published as {encoding}; "
              f"{said}", flush=True)
        self._ready.set()

    # ------------------------------------------------------------------------------------------ the main thread

    def run(self):
        """Starts the camera and blocks, its diagnostics once a second, until KeyboardInterrupt (which stops the
        camera and propagates) or an error: the camera's, or CamError when it delivers no frame for ``timeout``
        s."""
        reporter = Reporter(self.session, self.camera, SOURCE, self.prog)
        self._last_frame = time.monotonic()
        self._thread.start()
        sent = 0
        try:
            while not self._done.wait(0.1):
                if time.monotonic() - self._last_frame > self.timeout:
                    what = "a frame" if self._ready.is_set() else "its first frame"
                    raise CamError(f"{self.source}: no frame for {self.timeout:g} s (waiting for {what}; "
                                   "the camera stopped, or raise --timeout)")
                if self._ready.is_set() and reporter.due():
                    frames, decode_s = self.frames, self.decode_s
                    _, seconds = reporter.take()
                    hz = (frames - sent) / seconds
                    level, message = state(f"{hz:.1f} frames/s")
                    reporter.publish(level, message, {"frames/s": hz, "mode": self.delivered.text(),
                                                      "decode ms": f"{decode_s / max(frames, 1) * 1e3:.2f}"})
                    sent = frames
            if self.error is not None:
                raise self.error
        finally:
            self.stop()
            reporter.close()

    def stop(self):
        """Closes the publisher (its ring goes with it); the thread ends at its next frame, or with the process."""
        with self._lock:
            self._stopping = True
            if self.publisher is not None:
                self.publisher.close()
                self.publisher = None
        self._thread.join(timeout=0.5)


def add_arguments(parser: argparse.ArgumentParser):
    """``xblick cam``'s own options (cli.py adds the parameters' and the Zenoh session's)."""
    def size(text: str) -> tuple[int, int]:
        try:
            return parse_size(text)
        except VideoError as error:
            raise argparse.ArgumentTypeError(str(error)) from None

    parser.add_argument("source", help="a V4L2 device (/dev/video0), a URL (rtsp://..., http://...), or with --format "
                                       "another FFmpeg input (e.g. --format lavfi testsrc=size=640x480:rate=30)")
    parser.add_argument("--stream", required=True, metavar="BASE",
                        help="the stream base, <camera>/<stream>, e.g. robot/webcam/color")
    parser.add_argument("--format", default=None, metavar="FORMAT",
                        help="FFmpeg's input format (default: video4linux2 for /dev/..., the protocol for a URL)")
    parser.add_argument("--size", type=size, default=None, metavar="WxH",
                        help="ask the camera for this size; it must deliver it (default: its own)")
    parser.add_argument("--fps", type=float, default=None,
                        help="ask the camera for this rate; it must deliver it (default: its own)")
    parser.add_argument("--pixel-format", default=None, metavar="FORMAT",
                        help="V4L2: ask the device for this input format, e.g. mjpeg or yuyv422 (default: its own)")
    parser.add_argument("--camera-info", default=None, metavar="FILE",
                        help="a ROS calibration file of the delivered size (default: a pinhole, --hfov)")
    parser.add_argument("--hfov", type=float, default=None, metavar="DEG",
                        help="without --camera-info: a pinhole of this horizontal field of view (default 60)")
    parser.add_argument("--vfov", type=float, default=None, metavar="DEG", help="... or of this vertical one")
    parser.add_argument("--frame-id", default=None, metavar="ID", help="the frame id (default: the base, / as _)")
    parser.add_argument("--timeout", type=float, default=5.0, metavar="S",
                        help="end with an error when the camera delivers no frame for this long (default 5)")
