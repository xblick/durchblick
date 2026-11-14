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
``timeout`` seconds, or whose stream ends, ends the program with an error; with ``--reconnect`` it is opened again
until it delivers, in the mode of its first frame. ``--option KEY=VALUE`` passes FFmpeg input options, checked to
exist (ffopts.py).

FFmpeg reads and decodes in a daemon thread of its own. The main thread waits, reports and stops: Ctrl-C (SIGTERM)
closes the publisher at once (its ring goes with it), also while the thread is blocked in FFmpeg.
"""

from __future__ import annotations

import argparse
import os
import select
import sys
import termios
import threading
import time
import tty
from dataclasses import dataclass
from pathlib import Path

import zenoh

from .camera import CameraPublisher
from . import ffopts, topics, v4l2
from .codecs import JPEG, RVL
from .diagnostics import DiagnosticsPublisher
from .node import Reporter, state
from .video import VideoError, camera_info_for, camera_of, parse_size

SOURCE = "cam"  # the info record's source
V4L2 = "video4linux2"
URL_SCHEMES = ("rtsp", "rtsps", "rtmp", "http", "https", "udp", "tcp", "srt")
RECONNECT_S = 2.0  # --reconnect without a value: seconds between attempts
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


MONO = ("mono8", "16UC1")  # published as these: the stream is <camera>/mono, else <camera>/color


def camera_for(source: str, fmt: str | None, stream: str | None) -> str:
    """The camera (``<camera>``, the node of the parameters and the diagnostics): that of ``--stream``, else
    ``cam/<device node>`` for a V4L2 device (``/dev/video0``: ``cam/video0``), else ``cam/camera``."""
    if stream is not None:
        return camera_of(stream)
    if fmt != V4L2:
        return "cam/camera"
    node = Path(source).name
    try:
        return topics.check_name(f"cam/{node}")
    except topics.TopicError:
        raise CamError(f"{source}: {node!r} is no valid name part ([A-Za-z_][A-Za-z0-9_]*): give --stream") from None


def stream_for(camera: str, encoding: str) -> str:
    """The default stream of a camera: ``<camera>/mono`` for a grey one (mono8, 16UC1), else ``<camera>/color``."""
    return f"{camera}/{'mono' if encoding in MONO else 'color'}"


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


class _Attempt:
    """One opening of the camera, read by a thread of its own: what came of it."""

    def __init__(self, number: int):
        self.number = number
        self.started = time.monotonic()
        self.frames = 0
        self.ended = threading.Event()
        self.error: BaseException | None = None  # why it ended
        self.fatal = False  # an error no reconnect helps: a mode not asked for, or another one than before


class LiveCamera:
    """One live camera published as the stream ``base`` (module docstring); None: ``camera_for``'s camera and
    ``stream_for``'s stream, named at the first frame (colour or grey). ``reconnect``: seconds between attempts to
    open the camera again after it stopped (None: a camera that stops ends it). ``options``: FFmpeg's input options
    (``--option``), checked to exist (ffopts.py). :meth:`run` blocks until KeyboardInterrupt or an error (CamError,
    raised in the calling thread)."""

    def __init__(self, session: zenoh.Session, source: str, base: str | None = None, *, fmt: str | None = None,
                 size: tuple[int, int] | None = None, fps: float | None = None, pixel_format: str | None = None,
                 camera_info: str | None = None, hfov: float | None = None, vfov: float | None = None,
                 frame_id: str | None = None, timeout: float = 5.0, reconnect: float | None = None,
                 options: dict | None = None, prog: str = "xblick cam"):
        self.av = import_av()
        self.session, self.source, self.base, self.prog = session, source, base, prog
        self.fmt = input_format(source, fmt)
        self.camera = camera_for(source, self.fmt, base)
        self.asked = Mode(size, fps, pixel_format)
        self.options = device_options(self.fmt, size, fps, pixel_format)
        options = dict(options or {})
        own = sorted(set(options) & set(self.options))
        if own:
            raise CamError(f"--option {', '.join(own)}: give it with --size, --fps or --pixel-format")
        ffopts.check(options, self.fmt, source)
        self.options.update(options)
        if not timeout > 0:
            raise CamError(f"--timeout {timeout:g}: above 0")
        if reconnect is not None and not reconnect > 0:
            raise CamError(f"--reconnect {reconnect:g}: above 0")
        self.timeout, self.reconnect = timeout, reconnect
        self.frame_id = frame_id  # None: the stream's name, "/" as "_", once it is known
        self.calibration, self.hfov, self.vfov = camera_info, hfov, vfov
        if self.fmt == V4L2 and not Path(source).exists() and reconnect is None:
            devices = v4l2_devices()
            raise CamError(f"{source}: no such device (V4L2 devices on this PC: {', '.join(devices) or 'none'})")
        self.publisher: CameraPublisher | None = None
        self.delivered: Mode | None = None
        self.encoding: str | None = None
        self.error: BaseException | None = None
        self.frames = 0
        self.losses = 0
        self.returns = 0  # times it came back after a loss
        self.away: str | None = None  # why the camera is away (reconnecting), None while it delivers
        self.decode_s = 0.0  # summed over the frames: from the packet's arrival to the array
        self._last_frame = time.monotonic()
        self._lock = threading.Lock()  # publish against close; the attempt that may publish
        self._stopping = False
        self._ready = threading.Event()
        self._attempt: _Attempt | None = None
        self._thread: threading.Thread | None = None

    # ---------------------------------------------------------------------------------------- an attempt's thread

    def _open(self):
        network = self.fmt is None  # a URL: FFmpeg's interrupt callback ends a hung connect or read (PyAV timeout)
        try:
            return self.av.open(self.source, format=self.fmt, container_options=self.options,  # not the decoder
                                timeout=self.timeout if network else None)
        except self.av.FFmpegError as error:
            asked = f" with {self.asked.text()}" if self.asked != Mode() else ""
            raise CamError(f"{self.source}: FFmpeg cannot open it{asked} ({error})") from None

    def _capture(self, attempt: _Attempt):
        """One attempt: opens the camera, checks its mode, publishes its frames until it stops (``attempt.error``)
        or this attempt is no longer the current one (``_attempt``: abandoned, or stopped)."""
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
                    if attempt.frames == 0:
                        self._first_frame(attempt, Mode((frame.width, frame.height), float(rate) if rate else None,
                                                        context.pix_fmt if raw else context.name),
                                          frame.format.name)
                    elif (frame.width, frame.height) != self.delivered.size:
                        attempt.fatal = True
                        raise CamError(f"{self.source}: the camera changed its frame size from "
                                       f"{self.delivered.size[0]}x{self.delivered.size[1]} to "
                                       f"{frame.width}x{frame.height}")
                    image = frame.to_ndarray(format=self._to)
                    with self._lock:
                        if self._stopping or self._attempt is not attempt:
                            return
                        self.publisher.publish(image, stamp_ns=arrived)
                        attempt.frames += 1
                        self.frames += 1
                        self.decode_s += time.monotonic() - started
                        self._last_frame = time.monotonic()
            raise CamError(f"{self.source}: the stream ended")
        except self.av.FFmpegError as error:  # unplugged, a connection lost
            attempt.error = CamError(f"{self.source}: the camera stopped delivering ({error})")
        except BaseException as error:  # noqa: BLE001 - raised or reconnected by run() in the main thread
            attempt.error = error
        finally:
            attempt.ended.set()
            if container is not None:
                container.close()

    def _first_frame(self, attempt: _Attempt, mode: Mode, pixel_format: str):
        """The first frame of an attempt: of the first one checked against what was asked, and the publisher made;
        of a later one (a reconnect) checked against the first's mode, which subscribers and the ring were made for."""
        to, encoding = FORMATS.get(pixel_format, ("rgb24", "rgb8"))
        if self.delivered is None:
            try:
                check_mode(self.source, self.asked, mode)
            except CamError:
                attempt.fatal = True
                raise
            self.delivered, self._to = mode, to
            self._start_publisher(encoding)
            return
        if (mode.size, mode.pixel_format, encoding) != (self.delivered.size, self.delivered.pixel_format,
                                                         self.encoding):
            attempt.fatal = True
            raise CamError(f"{self.source}: back with {mode.text()} ({encoding}), before {self.delivered.text()} "
                           f"({self.encoding}): the stream's subscribers and ring are set up for the first")

    def _start_publisher(self, encoding: str):
        self.base = self.base or stream_for(self.camera, encoding)
        self.frame_id = self.frame_id or self.base.replace("/", "_")
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

    def _start_attempt(self, number: int):
        attempt = _Attempt(number)
        with self._lock:
            self._attempt = attempt
        self._last_frame = time.monotonic()
        self._thread = threading.Thread(target=self._capture, args=(attempt,), name=f"cam {self.source}",
                                        daemon=True)
        self._thread.start()

    def _lost(self, why: str):
        """The camera stopped: with reconnect, logged once; the next attempt follows after ``reconnect`` s."""
        if self.away is None:
            self.losses += 1
            self.away_since = time.monotonic()
            print(f"{self.prog}: {self.source}: lost ({why}); trying again every {self.reconnect:g} s", flush=True)
        self.away = why

    def run(self):
        """Starts the camera and blocks, its diagnostics once a second, until :meth:`stop`, KeyboardInterrupt (which
        stops the camera and propagates) or an error: the camera's, or CamError when it delivers no frame for
        ``timeout`` s. With ``reconnect``, a camera that stops or cannot be opened is opened again every
        ``reconnect`` s (the first opening too); an attempt still blocked in FFmpeg is waited for (a V4L2 device
        cannot be opened twice). Its mode must be the first one's."""
        reporter = Reporter(self.session, self.camera, SOURCE, self.prog)
        number, next_try = 1, None
        self._start_attempt(number)
        sent = 0
        try:
            while not self._stopping:  # stop() from another thread ends it too
                time.sleep(0.05)
                attempt = self._attempt
                if self.away is not None and attempt is not None and attempt.frames > 0:  # before its end, if any
                    print(f"{self.prog}: {self.source}: back after {time.monotonic() - self.away_since:.1f} s",
                          flush=True)
                    self.away = None
                    self.returns += 1
                if attempt is not None and attempt.ended.is_set():  # it stopped by itself
                    if attempt.fatal or self.reconnect is None or not isinstance(attempt.error, CamError):
                        raise attempt.error
                    self._lost(str(attempt.error).removeprefix(f"{self.source}: "))
                    with self._lock:
                        self._attempt = None
                    next_try = time.monotonic() + self.reconnect
                elif attempt is not None and time.monotonic() - self._last_frame > self.timeout:
                    first = "its first frame" if attempt.frames == 0 else "a frame"
                    if self.reconnect is None:
                        what = "a frame" if self._ready.is_set() else "its first frame"
                        raise CamError(f"{self.source}: no frame for {self.timeout:g} s (waiting for {what}; "
                                       "the camera stopped, or raise --timeout)")
                    self._lost(f"no frame for {self.timeout:g} s, waiting for {first}")
                    with self._lock:
                        self._attempt = None  # abandoned: its thread publishes nothing more
                    next_try = time.monotonic() + self.reconnect
                if (self._attempt is None and next_try is not None and time.monotonic() >= next_try
                        and not (self._thread is not None and self._thread.is_alive())):
                    number += 1
                    self._start_attempt(number)
                    next_try = None
                if reporter.due():
                    frames, decode_s = self.frames, self.decode_s
                    _, seconds = reporter.take()
                    hz = (frames - sent) / seconds
                    sent = frames
                    values = {"frames/s": hz, "losses": self.losses}
                    if self.delivered is not None:
                        values.update({"mode": self.delivered.text(),
                                       "decode ms": f"{decode_s / max(frames, 1) * 1e3:.2f}"})
                    if self.away is not None:
                        reporter.publish(DiagnosticsPublisher.WARN, f"camera away for "
                                         f"{time.monotonic() - self.away_since:.0f} s ({self.away}), trying again "
                                         f"every {self.reconnect:g} s", values)
                    elif self._ready.is_set():
                        level, message = state(f"{hz:.1f} frames/s")
                        reporter.publish(level, message, values)
        finally:
            self.stop()
            reporter.close()

    def stop(self):
        """Closes the publisher (its ring goes with it); the thread ends at its next frame, or with the process."""
        with self._lock:
            self._stopping = True
            self._attempt = None
            if self.publisher is not None:
                self.publisher.close()
                self.publisher = None
        if self._thread is not None:
            self._thread.join(timeout=0.5)


# --------------------------------------------------------------------------------- choosing a camera without a source

UP, DOWN = (b"\x1b[A", b"\x1bOA", b"k"), (b"\x1b[B", b"\x1bOB", b"j")
ENTER, LEAVE = (b"\r", b"\n"), (b"\x1b", b"q", b"Q", b"\x03")


def _key(fd: int) -> bytes:
    """One key from the terminal: an arrow's escape sequence comes in one piece; Esc alone is followed by nothing
    within 50 ms."""
    data = os.read(fd, 16)
    if data == b"\x1b" and select.select([fd], [], [], 0.05)[0]:
        data += os.read(fd, 16)
    return data


def choose(devices: list[v4l2.Device], fd: int, out, title: str) -> v4l2.Device | None:
    """A list of ``devices`` on the terminal (input ``fd``, output ``out``) moved through with the arrow keys (or j,
    k): Enter returns the marked one; Esc, q, Ctrl-C, SIGTERM (KeyboardInterrupt) or the end of the input return
    None. The terminal's mode and cursor are restored, and the list erased, on every way out, an exception too."""
    saved = termios.tcgetattr(fd)
    index, drawn = 0, False
    height = len(devices) + 1

    def draw():
        nonlocal drawn
        out.write((f"\x1b[{height}A" if drawn else "") + f"\x1b[2K{title}\n")
        for i, device in enumerate(devices):
            out.write("\x1b[2K" + (f"\x1b[7m> {device.text()}\x1b[0m" if i == index else f"  {device.text()}") + "\n")
        out.flush()
        drawn = True

    try:
        tty.setcbreak(fd)  # keys one by one, no echo; Ctrl-C stays a signal
        out.write("\x1b[?25l")  # no cursor while choosing
        draw()
        while True:
            key = _key(fd)
            if key in ENTER:
                return devices[index]
            if key in LEAVE or key == b"":
                return None
            if key in UP:
                index = (index - 1) % len(devices)
            elif key in DOWN:
                index = (index + 1) % len(devices)
            draw()
    except KeyboardInterrupt:
        return None
    finally:
        try:
            out.write((f"\x1b[{height}A\x1b[J" if drawn else "") + "\x1b[?25h")
            out.flush()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, saved)


def select_source(stdin=None, stdout=None, prog: str = "xblick cam") -> str:
    """The source of ``xblick cam`` without one: a V4L2 capture device of this PC, chosen in a list on the terminal
    (``choose``). CamError when there is none, when stdin or stdout is no terminal (never waits for keys in a pipe),
    or when nothing was chosen."""
    stdin, stdout = stdin or sys.stdin, stdout or sys.stdout
    listed = v4l2.devices()
    usable = v4l2.capture_devices(listed)
    if not usable:
        others = f" ({', '.join(d.path for d in listed)} capture no video)" if listed else ""
        raise CamError(f"no source given and no V4L2 capture device on this PC{others}: give a source, a URL "
                       "(rtsp://..., http://...) or another FFmpeg input with --format")
    if not (stdin.isatty() and stdout.isatty()):
        raise CamError("no source given, and no terminal to choose one in: give the source, one of "
                       + "; ".join(d.text() for d in usable))
    chosen = choose(usable, stdin.fileno(), stdout, f"{prog}: choose a camera (arrow keys, Enter; Esc or q: none)")
    if chosen is None:
        raise CamError("no camera chosen")
    print(f"{prog}: {chosen.text()}", flush=True)
    return chosen.path


def parse_options(given: list[str] | None) -> dict:
    """``--option KEY=VALUE`` (repeated) as a dict; CamError for one without ``=`` or given twice."""
    options = {}
    for item in given or []:
        key, sep, value = item.partition("=")
        if not sep or not key:
            raise CamError(f"--option {item!r}: KEY=VALUE, e.g. rtsp_transport=tcp")
        if key in options:
            raise CamError(f"--option {key}: given twice")
        options[key] = value
    return options


def add_arguments(parser: argparse.ArgumentParser):
    """``xblick cam``'s own options (cli.py adds the parameters' and the Zenoh session's)."""
    def size(text: str) -> tuple[int, int]:
        try:
            return parse_size(text)
        except VideoError as error:
            raise argparse.ArgumentTypeError(str(error)) from None

    parser.add_argument("source", nargs="?", default=None,
                        help="a V4L2 device (/dev/video0), a URL (rtsp://..., http://...), or with --format another "
                             "FFmpeg input (e.g. --format lavfi testsrc=size=640x480:rate=30); none: choose one of "
                             "this PC's V4L2 cameras in a list")
    parser.add_argument("--stream", default=None, metavar="BASE",
                        help="the stream base, <camera>/<stream> (default: cam/<device node>/color, e.g. "
                             "cam/video0/color, or cam/camera/color for a URL or --format source; /mono instead of "
                             "/color for a grey camera)")
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
                        help="end with an error when the camera delivers no frame for this long (default 5); with "
                             "--reconnect: open it again")
    parser.add_argument("--reconnect", type=float, nargs="?", const=RECONNECT_S, default=None, metavar="SECONDS",
                        help="a camera that stops, or cannot be opened (also at the start), is opened again every "
                             f"SECONDS (default {RECONNECT_S:g}); it must come back with the same mode (default: a "
                             "camera that stops ends the command)")
    parser.add_argument("--option", action="append", default=None, metavar="KEY=VALUE",
                        help="an FFmpeg input option, e.g. rtsp_transport=tcp (repeat for more); one FFmpeg does not "
                             "have for this source is refused")
