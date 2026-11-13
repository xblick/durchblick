"""Videos played as camera streams: ``blick play`` with video files (or image sequences) instead of MCAP
recordings; ``recording.Recorder.play`` hands them to :class:`VideoPlayer`. docs/guides/video.md has the use.

Every video is one stream (``--stream <base>``, its frames on ``<base>/image_raw/...`` with ``camera_info``, as a
camera publishes them, source ``video``). Several videos play at once on one clock, a decoding and a sending thread
each: a frame's stamp is the playback's start (wall time) plus its time in the video (its PTS less ``--start``)
divided by the rate, plus its stream's offset (``--offset-ms``), and it is sent at that stamp. Frames of equal PTS
(and offset) get the same stamp, to the nanosecond, so that stereo and RGB-D nodes pair them exactly. Looped, all
streams start again together once the longest has ended, the stamps moved on by that length: they keep growing.
Pausing holds the clock (the stamps after it go on from the wall time of the resume).

Decoding is PyAV's (FFmpeg inside its wheels), the extra ``blick[video]``; image sequences (a directory or a
glob of PNG/JPEG files) are read with Pillow. A colour video becomes ``rgb8``, a grey one ``mono8``, a 16-bit grey one
(e.g. FFV1 ``gray16le`` in Matroska: depth in mm) ``16UC1``.
"""

from __future__ import annotations

import collections
import glob
import math
import queue
import threading
import time
from pathlib import Path

import numpy as np
import zenoh

from .camera import CameraPublisher
from .codecs import JPEG, RVL
from .diagnostics import DiagnosticsPublisher
from .msg import CameraInfo
from .recording import Paced, Seeked

MCAP_MAGIC = b"\x89MCAP0\r\n"
IMAGE_SUFFIXES = (".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff", ".ppm", ".pgm")
DEFAULT_HFOV = 60.0  # degrees: the pinhole assumed without --camera-info, --hfov or --vfov
LATE_S = 0.005  # a frame sent this much after its stamp counts as late
QUEUE = 8  # decoded frames a stream keeps ready


class VideoError(ValueError):
    """A video, an image sequence or an option for them that does not fit."""


def import_av():
    try:
        import av
    except ImportError:
        raise ImportError("playing videos needs PyAV: install blick[video] (pip install 'blick[video]', "
                          "or uv sync --extra video)") from None
    return av


def is_mcap(path: Path) -> bool:
    """An MCAP file, by its magic bytes (not its name)."""
    with open(path, "rb") as stream:
        return stream.read(len(MCAP_MAGIC)) == MCAP_MAGIC


def _is_glob(text: str) -> bool:
    return any(c in text for c in "*?[")


def is_video_input(given: str | Path) -> bool:
    """A video input: an image directory, a glob, or a file that is not MCAP."""
    path = Path(given)
    if path.is_dir() or (_is_glob(str(given)) and not path.exists()):
        return True
    return path.is_file() and not is_mcap(path)


def parse_size(text: str) -> tuple[int, int]:
    """``WxH`` -> (W, H)."""
    w, sep, h = text.lower().partition("x")
    if not sep or not w.isdigit() or not h.isdigit() or int(w) < 1 or int(h) < 1:
        raise VideoError(f"size {text!r}: WIDTHxHEIGHT, e.g. 640x480")
    return int(w), int(h)


def _out_size(width: int, height: int, size: tuple[int, int] | None, scale: float | None) -> tuple[int, int]:
    if size is not None and scale is not None:
        raise VideoError("size or scale, not both")
    if size is not None:
        return size
    if scale is not None:
        if not scale > 0:
            raise VideoError(f"scale {scale}: > 0")
        return max(1, round(width * scale)), max(1, round(height * scale))
    return width, height


class VideoFile:
    """A video file's first video stream: its frames as arrays, ``(seconds from start, array)``, from ``start`` to
    ``end`` (seconds of the file), each pass (``frames()``) from the beginning."""

    FORMATS = {"gray16le": ("gray16le", "16UC1"), "gray16be": ("gray16le", "16UC1"), "gray": ("gray", "mono8"),
               "gray8": ("gray", "mono8")}

    def __init__(self, path: Path, start: float = 0.0, end: float | None = None, size=None, scale=None,
                 fps: float | None = None):
        self.av = import_av()
        self.path, self.start, self.end, self.fps_given = Path(path), start, end, fps
        with self._open() as container:
            stream = container.streams.video[0]
            context = stream.codec_context
            self.format, self.encoding = self.FORMATS.get(context.pix_fmt or "", ("rgb24", "rgb8"))
            self.width, self.height = _out_size(context.width, context.height, size, scale)
            rate = fps or (float(stream.average_rate) if stream.average_rate else None)
            if not rate:
                raise VideoError(f"{path}: no frame rate in the file: give --fps")
            self.fps = rate
            length = (float(stream.duration * stream.time_base) if stream.duration else
                      container.duration / 1e6 if container.duration else None)
        # the seconds it plays (0: not known: no seeking)
        self.duration = max(0.0, min(length, end) - start if end is not None else length - start) if length else 0.0

    def _open(self):
        try:
            container = self.av.open(str(self.path))
        except self.av.FFmpegError as error:
            raise VideoError(f"{self.path}: not a video FFmpeg reads ({error})") from None
        if not container.streams.video:
            container.close()
            raise VideoError(f"{self.path}: no video stream")
        return container

    def frames(self, from_s: float = 0.0):
        """From ``from_s`` seconds after the start on: the key frame before it found, decoded on to the frame."""
        begin = self.start + from_s
        with self._open() as container:
            stream = container.streams.video[0]
            stream.thread_type = "AUTO"
            if begin > 0 and self.fps_given is None:
                container.seek(int(begin / stream.time_base), stream=stream)  # the key frame before it
            index = 0
            for frame in container.decode(stream):
                if self.fps_given is not None:  # the frames' times from the rate given
                    t = index / self.fps_given
                    index += 1
                    if t < begin - 1e-9:
                        continue
                else:
                    if frame.pts is None:
                        raise VideoError(f"{self.path}: a frame without a time stamp (PTS): give --fps")
                    t = float(frame.pts * stream.time_base)
                    if t < begin - 1e-6:
                        continue
                if self.end is not None and t >= self.end:
                    return
                image = frame.reformat(width=self.width, height=self.height, format=self.format).to_ndarray()
                yield t - self.start, image


class ImageSequence:
    """Image files (a directory's, sorted by name, or a glob's) at ``fps`` frames a second."""

    def __init__(self, given: str, start: float = 0.0, end: float | None = None, size=None, scale=None,
                 fps: float | None = None):
        from PIL import Image

        self.Image = Image
        path = Path(given)
        names = (sorted(p for p in path.iterdir() if p.suffix.lower() in IMAGE_SUFFIXES) if path.is_dir()
                 else sorted(Path(p) for p in glob.glob(str(given))))
        if not names:
            raise VideoError(f"{given}: no images ({', '.join(IMAGE_SUFFIXES)})")
        if not fps:
            raise VideoError(f"{given}: images have no frame rate: give --fps")
        self.fps, self.start, self.end, self.names = fps, start, end, names
        first = self._read(names[0], None)
        self.encoding = {"I;16": "16UC1", "L": "mono8"}.get(self._mode, "rgb8")
        h, w = first.shape[:2]
        self.width, self.height = _out_size(w, h, size, scale)
        self.path = path
        length = len(names) / fps
        self.duration = max(0.0, (min(length, end) if end is not None else length) - start)

    def _read(self, name: Path, size):
        image = self.Image.open(name)
        mode = image.mode if image.mode in ("I;16", "L") else "RGB"
        if not hasattr(self, "_mode"):
            self._mode = mode
        elif mode != self._mode:
            raise VideoError(f"{name}: {image.mode}, the images before {self._mode}: one kind in a sequence")
        image = image.convert(mode) if image.mode != mode else image
        if size is not None and image.size != size:
            image = image.resize(size)
        return np.asarray(image)

    def frames(self, from_s: float = 0.0):
        for i, name in enumerate(self.names):
            t = i / self.fps
            if t < self.start + from_s - 1e-9:
                continue
            if self.end is not None and t >= self.end:
                return
            yield t - self.start, self._read(name, (self.width, self.height))


def open_source(given: str, **options):
    path = Path(given)
    if path.is_dir() or (_is_glob(given) and not path.exists()):
        return ImageSequence(given, **options)
    if path.suffix.lower() in IMAGE_SUFFIXES:
        raise VideoError(f"{given}: a single image; a sequence is a directory or a glob ('frames/*.png')")
    return VideoFile(path, **options)


def camera_info_for(width: int, height: int, frame_id: str, calibration: Path | None, hfov: float | None,
                    vfov: float | None) -> tuple[CameraInfo, str]:
    """The stream's CameraInfo: from a ROS calibration file (its size the stream's), else a pinhole of ``hfov`` or
    ``vfov`` degrees (default DEFAULT_HFOV); and how it came about, for the log."""
    if calibration is not None:
        if hfov is not None or vfov is not None:
            raise VideoError("--camera-info or --hfov/--vfov, not both")
        info = CameraInfo.from_yaml(Path(calibration).read_text())
        if (info.width, info.height) != (width, height):
            raise VideoError(f"{calibration}: a calibration of {info.width}x{info.height}, the stream is "
                             f"{width}x{height} (--size {info.width}x{info.height}, or a calibration of that size)")
        info.header.frame_id = frame_id
        return info, f"camera_info from {calibration}"
    if hfov is not None and vfov is not None:
        raise VideoError("--hfov or --vfov, not both")
    if vfov is not None:
        f, said = 0.5 * height / math.tan(math.radians(vfov) / 2), f"{vfov:g} deg vertical"
    else:
        degrees = hfov if hfov is not None else DEFAULT_HFOV
        f, said = 0.5 * width / math.tan(math.radians(degrees) / 2), f"{degrees:g} deg horizontal"
    if not f > 0 or not math.isfinite(f):
        raise VideoError("a field of view between 0 and 180 degrees")
    info = CameraInfo.pinhole(width, height, f, f, (width - 1) / 2, (height - 1) / 2, frame_id=frame_id)
    return info, f"camera_info ASSUMED: a pinhole of {said} field of view (give --camera-info for a calibration)"


def camera_of(base: str) -> str:
    """The camera a stream belongs to: its base less the last part (its diagnostics go to <camera>/diagnostics)."""
    camera, sep, _ = base.rpartition("/")
    if not sep:
        raise VideoError(f"stream {base!r}: <camera>/<stream>, e.g. robot/video/color")
    return camera


class _Stream:
    """One video's stream: its source, publisher and counts."""

    def __init__(self, source, base: str, offset_ns: int, publisher: CameraPublisher, said: str):
        self.source, self.base, self.offset_ns, self.publisher, self.said = source, base, offset_ns, publisher, said
        self.sent = 0
        self.late = 0
        self.late_max_s = 0.0
        self.late_window = (0, 0.0)  # late frames and the latest by s since the last report
        self.position_s = 0.0
        self.decode_s = collections.deque(maxlen=100)
        self.lateness = collections.deque(maxlen=100)  # send time less the stamp, s, of the last frames
        self.queue: queue.Queue = queue.Queue(maxsize=QUEUE)
        self.next_s, self.next_epoch = 0.0, -1  # where its next frame would come (s of the videos), in which epoch
        self.reported = (0, time.monotonic())


class VideoPlayer(Paced):
    """Plays videos at once, one stream each (``streams``: their bases, in the order of ``files``), on one clock
    (module docstring). ``offsets_ms``, ``camera_infos``, ``frame_ids``: per stream, in file order (or none of them);
    ``size``/``scale``, ``start``/``end`` (s), ``fps`` (instead of the files' times), ``hfov``/``vfov``: for all.
    :meth:`run` blocks until the end (with ``loop`` until :meth:`end`)."""

    def __init__(self, session: zenoh.Session, files: list[str], streams: list[str], rate: float = 1.0,
                 loop: bool = False, cuda_device: int | None = None, offsets_ms=(), camera_infos=(), frame_ids=(),
                 size: tuple[int, int] | None = None, scale: float | None = None, start: float | None = None,
                 end: float | None = None, fps: float | None = None, hfov: float | None = None,
                 vfov: float | None = None):
        super().__init__(rate, "wall")
        n = len(files)
        if len(streams) != n:
            raise VideoError(f"{n} video(s) and {len(streams)} --stream: one stream base per video, in their order")
        if len(set(streams)) != n:
            raise VideoError(f"streams {', '.join(streams)}: each video a stream of its own")
        for name, given in (("--offset-ms", offsets_ms), ("--camera-info", camera_infos), ("--frame-id", frame_ids)):
            if given and len(given) != n:
                raise VideoError(f"{len(given)} {name} for {n} video(s): one per video, in their order, or none")
        start = start or 0.0
        if start < 0 or (end is not None and end <= start):
            raise VideoError(f"--start {start:g} --end {end}: 0 <= start < end")
        if fps is not None and not fps > 0:
            raise VideoError(f"--fps {fps}: > 0")
        self.session, self.files, self.loop = session, list(files), loop
        self.cuda_device = cuda_device
        self.passes = 0
        self.error: BaseException | None = None
        self._streams: list[_Stream] = []
        self._threads: list[threading.Thread] = []
        self._stamps: collections.OrderedDict = collections.OrderedDict()  # log time -> its stamp (equal for all)
        self._wall_offset_ns = 0
        # per epoch (0, then one per seek or loop): the seconds of the videos it starts at and its stamps' base
        self._epoch_start: dict[int, float] = {0: 0.0}
        self._epoch_base: dict[int, int] = {0: 0}
        self._ended_streams: dict[int, set] = {}  # epoch -> streams at their end in it
        self._all_ended = False  # not looped: every stream at its end
        self._diagnostics: dict[str, DiagnosticsPublisher] = {}
        try:
            for i, (given, base) in enumerate(zip(files, streams)):
                source = open_source(str(given), start=start, end=end, size=size, scale=scale, fps=fps)
                frame_id = frame_ids[i] if frame_ids else base.replace("/", "_")
                info, said = camera_info_for(source.width, source.height, frame_id,
                                             camera_infos[i] if camera_infos else None, hfov, vfov)
                camera = camera_of(base)
                publisher = CameraPublisher(session, base, encoding=source.encoding, frame_id=frame_id,
                                            camera_info=info, source="video",
                                            codec=RVL if source.encoding == "16UC1" else JPEG,
                                            cuda_device=cuda_device)
                offset_ns = round((offsets_ms[i] if offsets_ms else 0.0) * 1e6)
                self._streams.append(_Stream(source, base, offset_ns, publisher, said))
                if camera not in self._diagnostics:
                    self._diagnostics[camera] = DiagnosticsPublisher(session, camera, "play")
        except BaseException:
            self.close()
            raise
        for s in self._streams:
            print(f"blick play: {s.source.path} -> {s.base}: {s.source.width}x{s.source.height} "
                  f"{s.source.encoding} at {s.source.fps:g} frames/s" + (f", offset {s.offset_ns / 1e6:g} ms"
                                                                       if s.offset_ns else "") + f"; {s.said}",
                  flush=True)

    # ----------------------------------------------------------------------------------------------- state

    @property
    def topics(self) -> list[str]:
        return [s.base for s in self._streams]

    @property
    def sent(self) -> int:
        return sum(s.sent for s in self._streams)

    @property
    def position_s(self) -> float:
        return max((s.position_s for s in self._streams), default=0.0)

    def skew_ms(self) -> float:
        """How far apart the streams go out: the spread of their recent mean send times less stamps, ms."""
        means = [sum(s.lateness) / len(s.lateness) for s in self._streams if s.lateness]
        return (max(means) - min(means)) * 1e3 if len(means) > 1 else 0.0

    def status(self) -> dict:
        return {"kind": "video", "files": [str(f) for f in self.files], "position_s": round(self.position_s, 3),
                "sent": self.sent, "passes": self.passes, "rate": self.rate, "loop": self.loop,
                "cuda_device": self.cuda_device, "pace": self.pace, "finished": self.finished,
                "skew_ms": round(self.skew_ms(), 3),
                "streams": [{"base": s.base, "file": str(s.source.path), "sent": s.sent, "late": s.late,
                             "position_s": round(s.position_s, 3), "offset_ms": s.offset_ns / 1e6,
                             "size": [s.source.width, s.source.height], "encoding": s.source.encoding,
                             "fps": s.source.fps} for s in self._streams]}

    def step(self, frames=None, until_ns=None, key=None, timeout: float = 60.0) -> dict:
        raise RuntimeError("step: videos play at the wall clock's pace only (pace wall)")

    # ------------------------------------------------------------------------------------------------ playing

    def _stamp(self, t: int, due: float) -> int:
        """The stamp of log time ``t``, the same for every stream (equal PTS and offset: equal stamps)."""
        with self._grant:
            stamp = self._stamps.get(t)
            if stamp is None:
                stamp = self._stamps[t] = round(due * 1e9) + self._wall_offset_ns
                while len(self._stamps) > 1024:
                    self._stamps.popitem(last=False)
            return stamp

    @property
    def duration_s(self) -> float:
        return max(s.source.duration for s in self._streams)

    @property
    def file(self) -> str:
        return ", ".join(str(s.source.path) for s in self._streams)

    def _period_ns(self) -> int:
        """The gap in the stamps at a loop or a seek: a frame period of the slowest video."""
        return round(1e9 / min(s.source.fps for s in self._streams))

    def seek(self, position_s: float):
        if not 0 <= position_s <= self.duration_s:
            raise ValueError(f"position {position_s:g} s: between 0 and the length, {self.duration_s:g} s")
        with self._grant:
            self._new_epoch(float(position_s))

    def _new_epoch(self, start_s: float):
        """From ``start_s`` seconds of the videos on (a seek, or 0 for a loop): the frame there comes where the
        furthest stream's next frame would have come (its last one's time and its period), so the stamps grow, and
        a loop moves them on by the longest video's length. Holding ``_grant``."""
        ahead = max((s.next_s for s in self._streams if s.next_epoch == self._epoch), default=None)
        base = self._epoch_base[self._epoch]
        epoch = self._epoch + 1
        self._epoch_start[epoch] = start_s
        if ahead is not None:
            next_ns = base + round(ahead * 1e9)
        else:  # nothing sent in this epoch (a seek right after another): after the last frame sent at all
            next_ns = (self.log_time_ns if self.log_time_ns is not None else 0) + self._period_ns()
        self._epoch_base[epoch] = next_ns - round(start_s * 1e9)
        self._epoch = epoch
        for s in self._streams:
            s.position_s = start_s
        self._grant.notify_all()

    def _decode(self, s: _Stream):
        """Decodes from each epoch's start into the stream's queue, ``(epoch, t, image)``, image None at its end;
        a new epoch (a seek, a loop) starts it again; ends with the playback."""
        try:
            while not self._ended.is_set():
                with self._grant:
                    epoch, start_s = self._epoch, self._epoch_start[self._epoch]
                began = time.monotonic()
                for t, image in s.source.frames(start_s):
                    s.decode_s.append(time.monotonic() - began)
                    if not self._put(s, (epoch, t, image), epoch):
                        break
                    began = time.monotonic()
                else:
                    self._put(s, (epoch, None, None), epoch)
                with self._grant:  # the next epoch (or the end)
                    while self._epoch == epoch and not self._ended.is_set() and not self._all_ended:
                        self._grant.wait(0.2)
                    if self._all_ended:
                        return
        except BaseException as error:  # noqa: BLE001 - ends the playback, raised by Recorder.check()
            self.error = self.error or error
            self.end()

    def _put(self, s: _Stream, item, epoch: int) -> bool:
        """Into the stream's queue, unless the playback ended or a new epoch came (False)."""
        while not self._ended.is_set() and self._epoch == epoch:
            try:
                s.queue.put(item, timeout=0.2)
                return True
            except queue.Full:
                continue
        return False

    def _at_end(self, i: int, epoch: int) -> bool:
        """Stream ``i`` sent the last frame of ``epoch``: once all have, the next pass (looped) or the end (True)."""
        with self._grant:
            if epoch != self._epoch:
                return False
            done = self._ended_streams.setdefault(epoch, set())
            done.add(i)
            if len(done) == len(self._streams):
                if self.loop:
                    self.passes += 1
                    self._new_epoch(0.0)
                else:
                    self._all_ended = True
                    self._grant.notify_all()
            while self._epoch == epoch and not self._ended.is_set() and not self._all_ended:
                self._grant.wait(0.2)
            return self._all_ended or self._ended.is_set()

    def _send(self, i: int, s: _Stream):
        try:
            while not self._ended.is_set():
                try:
                    epoch, t_s, image = s.queue.get(timeout=0.2)
                except queue.Empty:
                    continue
                if epoch != self._epoch:
                    continue  # decoded before a seek
                if image is None:
                    if self._at_end(i, epoch):
                        return
                    continue
                t = self._epoch_base[epoch] + round(t_s * 1e9) + s.offset_ns
                try:
                    due = self._wait_for(t, epoch)
                except Seeked:
                    continue
                if due is None:
                    return
                stamp = self._stamp(t, due)
                late = time.monotonic() - due
                s.publisher.publish(image, stamp_ns=stamp)
                s.lateness.append(late)
                if late > LATE_S:
                    s.late += 1
                    s.late_max_s = max(s.late_max_s, late)
                    n, worst = s.late_window
                    s.late_window = (n + 1, max(worst, late))
                s.sent += 1
                s.position_s = t_s
                s.next_s, s.next_epoch = t_s + 1.0 / s.source.fps, epoch
                with self._grant:
                    self.log_time_ns = t if self.log_time_ns is None else max(self.log_time_ns, t)
        except BaseException as error:  # noqa: BLE001
            self.error = self.error or error
            self.end()

    def _report(self):
        while not self._ended.wait(1.0):
            self.report()

    def report(self):
        """A diagnostics status ``play`` on each camera's diagnostics: per stream frames/s, decode ms, position,
        late frames; the passes and the streams' send skew."""
        skew = self.skew_ms()
        for camera, diagnostics in self._diagnostics.items():
            values, problems = {"passes": self.passes, "send skew ms": f"{skew:.2f}", "rate": self.rate}, []
            mine = [s for s in self._streams if camera_of(s.base) == camera]
            for s in mine:
                now = time.monotonic()
                sent, at = s.reported
                s.reported = (s.sent, now)
                name = s.base.rpartition("/")[2]
                decode = sorted(s.decode_s)
                values.update({f"{name} frames/s": f"{(s.sent - sent) / max(now - at, 1e-9):.1f}",
                               f"{name} decode ms": f"{decode[len(decode) // 2] * 1e3:.1f}" if decode else "-",
                               f"{name} position s": f"{s.position_s:.2f}", f"{name} late frames": s.late,
                               f"{name} late max ms": f"{s.late_max_s * 1e3:.1f}"})
                late, worst = s.late_window
                s.late_window = (0, 0.0)
                if late:
                    problems.append(f"{name}: {late} frames sent late in the last {now - at:.0f} s (decoding or "
                                    f"publishing behind, up to {worst * 1e3:.0f} ms)")
            level = DiagnosticsPublisher.WARN if problems else DiagnosticsPublisher.OK
            message = "; ".join(problems) if problems else (
                f"playing {len(mine)} video stream(s) at {self.rate:g}x, pass {self.passes + 1}")
            diagnostics.publish(level, message, values)

    def run(self):
        """Play until the end (with ``loop`` until :meth:`end`). An error ends it and stays in ``error``."""
        try:
            with self._grant:
                self._wall_offset_ns = time.time_ns() - round(time.monotonic() * 1e9)
                self._anchor = (min(s.offset_ns for s in self._streams), time.monotonic())
            threads = [threading.Thread(target=self._decode, args=(s,), name=f"decode {s.base}", daemon=True)
                       for s in self._streams]
            threads += [threading.Thread(target=self._send, args=(i, s), name=f"send {s.base}", daemon=True)
                        for i, s in enumerate(self._streams)]
            reporter = threading.Thread(target=self._report, name="video report", daemon=True)
            for thread in threads + [reporter]:
                thread.start()
            self._threads = threads + [reporter]
            for thread in threads[len(self._streams):]:  # the senders
                thread.join()
        finally:
            self.end()
            self._finish()
            if self.error is None:
                self.report()

    def close(self):
        self.end()
        for thread in self._threads:
            thread.join(5.0)
        for s in self._streams:
            s.publisher.close()
        for diagnostics in self._diagnostics.values():
            diagnostics.close()
