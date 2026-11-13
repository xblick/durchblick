"""Recordings of Durchblick's topics as MCAP files, and playing them back: ``blick record``, ``blick play``.

MCAP (mcap.dev) is rosbag2's default storage: a recording opens in ``ros2 bag``, Foxglove and every MCAP reader. Each
Zenoh sample is written as it came, CDR with its ROS type's definition as the channel's schema (``ros2msg``,
msgdefs.py); JSON (bodies) as JSON; anything else unchanged under its Zenoh encoding. The topic is ``/<key>``; each
channel's metadata keeps the Zenoh key and encoding and, for streams and detections, the publisher's info record, so
that ``play`` serves them as their publishers did.

**What is recorded.** What ``blick rerun`` finds (``find_streams``, looked for again every few seconds): every
image stream's compressed form (``image_raw/compressed``, JPEG or lossless PNG) and ``camera_info``; label images the
same; detections; point clouds (``cloud``) only with ``points``: they are large and come again from depth and colour.
Streams without a compressed form (float disparity, feature maps) are left out. And, by key pattern below each
namespace: a robot's ``joint_states``, ``tf``, ``odom`` and ``wrench/*``, each camera's ``tf``, ``tf_static`` (also
asked for at the start of each file, as it is published once), ``imu``, ``diagnostics`` and bodies, and a node's
``tf`` and ``odom`` (``<camera>/<node>/...``, e.g. cuVSLAM's). The recorder reads what goes over the network, never
shared memory, so it runs on any PC that reaches the robot's routers.

**One node records and plays** (``Recorder``; ``blick record`` starts it recording, ``blick play`` playing,
both take every command later). On ``durchblick/<name>/control`` (plain text or JSON ``{"command": ...}``) it takes

* ``record``: a new file ``<dir>/<name>_<UTC time>.mcap``, and after ``split`` seconds the next one; refused while it
  plays (it would record its own playback);
* ``play`` with ``files`` (relative to ``<dir>``, or absolute; played one after another) and optionally ``rate``,
  ``loop``, ``prefix``, ``keys`` and ``cuda_device`` (``Player``); ends a playback before, refused while it records;
* ``pause`` and ``resume`` (a playback), ``stop`` (what it does: the file is complete, or the playback ends).

* ``step`` (a playback in lockstep, below) with ``frames``, ``until_ns`` or ``key``.

It publishes its state (JSON: ``state`` idle, starting (record: until ready), recording, playing or paused, with the
file's or the playback's numbers) on ``durchblick/<name>/status`` every second and answers a query there; a refused
command's reason is its ``refused``.
While it does not record it subscribes to nothing (the publishers encode nothing for it). The control key is a
queryable too: a command sent as a query (``request``) is answered once it took effect, with its result (JSON
``{"v":1,"ok":true,"command":...,"result":{...}}``, or ``"ok":false`` with ``refused`` or ``error``).

**Recording a simulation.** ``record`` with ``keys`` (a list of key expressions) subscribes to exactly those (no
discovery), looks for their publishers and answers *ready* once it is subscribed to all of them, with the keys it
found a publisher for (``publishers``: blocking publishers' counts, and streams' info records) and the literal ones it
did not (``missing``); ``file`` names the file (one, not split). ``log_time``: ``receive`` (the default: when it came)
or ``header`` (the message's own stamp: header, the first transform of a TFMessage, a Clock, a JSON sample's
``stamp_ns``; the simulated time, so that a recording plays back on the simulation's time line). ``blocking``: for
the keys of blocking publishers (those that asked for it, ``blocking=True``; they answer ``<key>/sent`` with their
counts) the recorder blocks instead of dropping (its queue full, Zenoh holds them up, they block too), refuses to start
if one does not yet know of its subscription, and at ``stop`` checks each: every sample put after *ready* must be in
the file (by the attachment's seq; a key whose blocking publisher appears later, from its first sample on). A mismatch
is an error (RecordingError; ``ok`` false over Zenoh), and the file's metadata record ``durchblick_recording`` says
what was checked. Every other key (dropping publishers', the default, e.g. a robot's) it records as they come and
drops when behind, never blocking for them nor checking them: ready lists them as ``dropping``, stop reports their
received and dropped samples. The defaults never block: nothing is held up by a recording unless it asked for it. ``Recording`` runs it in-process on a private
session.

**Playback.** ``Player`` publishes a recording's samples on their keys (below ``prefix``, by default none: as the
robot would), at the recorded pace, with the recorded stamps: a node reading them cannot tell them from the robot's.
Looped, each pass moves every stamp (headers, transforms, a JSON sample's ``stamp_ns``; not ``tf_static``) on by the
recording's length and one period of its slowest channel, so that time goes on as a camera's does: nodes that take
only newer frames go on as well.
Each image stream goes out through a ``CameraPublisher`` as a camera's does: the recorded JPEG or PNG as it is to
compressed subscribers, decoded once into the shared-memory ring (and with ``cuda_device`` into a CUDA ring) for the
nodes on this PC, and raw; with its recorded ``camera_info``. It answers ``<base>/info`` for streams and detections,
and ``tf_static`` queries. ``pace``: ``wall`` (the default: the recorded pace, times ``rate``), or in lockstep:
``step`` (a frame, the messages of one log time, only when told: :meth:`Player.step`, the command ``step``) or
``clock`` (up to the time of each rosgraph_msgs/Clock sample on the key ``clock``, e.g. a policy's clock); in lockstep
every key blocks instead of dropping, so that a reader gets every frame.
"""

from __future__ import annotations

import array
import bisect
import dataclasses
import datetime
import fnmatch
import json
import queue
import threading
import time
from pathlib import Path

import zenoh

from . import _core, topics
from .camera import (FIND_PERIOD, SENT_VERSION, CameraPublisher, ProtocolError, _attachment, _check_encoding,
                     congestion, find_streams, parse_attachment)
from .codecs import parse_format
from .msg import CameraInfo, Clock, CompressedImage, Message, Time
from .msgdefs import MSGDEFS
from .parameters import Parameters
from .session import replies
from .stamps import header_stamp  # recording.header_stamp: its old home

STATUS_VERSION = 3
REPLY_VERSION = 1  # of a request's reply
COMMANDS = ("record", "stop", "play", "pause", "resume", "step")
RECORD_OPTIONS = ("keys", "blocking", "log_time", "file", "ready_timeout")
PLAY_OPTIONS = ("files", "rate", "loop", "prefix", "keys", "cuda_device", "pace", "clock")
STEP_OPTIONS = ("frames", "until_ns", "key")
OPTIONS = {"record": RECORD_OPTIONS, "play": PLAY_OPTIONS, "step": STEP_OPTIONS}
LOG_TIMES = ("receive", "header")
PACES = ("wall", "step", "clock")
# keys of the topics that are not streams, below a namespace (the robot's, e.g. robot; its cameras' are <ns>/<camera>;
# a simulation's world, e.g. sim, has tf, clock and events; a robot's controllers report below {ns}/controller)
PATTERNS = ("{ns}/joint_states", "{ns}/tf", "{ns}/odom", "{ns}/wrench/*", "{ns}/command/*", "{ns}/controller/**",
            "{ns}/clock", "{ns}/events", "{ns}/*/tf", "{ns}/*/tf_static", "{ns}/*/imu", "{ns}/*/diagnostics",
            "{ns}/*/bodies", "{ns}/*/*/bodies", "{ns}/*/*/tf", "{ns}/*/*/odom")
CDR = "application/cdr;"
JSON = "application/json"
READY_TIMEOUT = 2.0  # s a record with keys looks for their publishers at most
STOP_TIMEOUT = 5.0  # s a blocking stop waits at most for the publishers' counts and the samples still under way
METADATA = "durchblick_recording"  # the MCAP metadata record of a recording with keys: what was recorded and checked


class RecordingError(RuntimeError):
    """A recording that is not what was asked for: a blocking recording that lost samples, or could not be checked.
    ``result`` has the numbers (Recorder.stop's)."""

    def __init__(self, message: str, result: dict | None = None):
        super().__init__(message)
        self.result = result


def import_mcap():
    """The mcap module, or a message on how to install it."""
    try:
        import mcap.reader
        import mcap.writer
    except ModuleNotFoundError as error:
        if error.name != "mcap":
            raise
        raise SystemExit("blick record and play need the mcap package: pip install 'blick[record]'") from error
    return mcap


def control_key(name: str) -> str:
    return topics.check_name(f"durchblick/{name}/control")


def status_key(name: str) -> str:
    return topics.check_name(f"durchblick/{name}/status")


def parse_command(payload: bytes) -> tuple[str, dict]:
    """(command, its options) of a control sample: plain text, or JSON ``{"command": ..., <options>}`` (OPTIONS:
    ``record``'s, ``play``'s with ``files`` required, ``step``'s)."""
    text = payload.decode().strip()
    request = json.loads(text) if text.startswith("{") else {"command": text}
    command = request.pop("command", None)
    if command not in COMMANDS:
        raise ValueError(f"control: {command!r}, one of {', '.join(COMMANDS)} expected")
    allowed = OPTIONS.get(command, ())
    unknown = set(request) - set(allowed)
    if unknown:
        raise ValueError(f"control: {command} takes {', '.join(allowed) or 'no options'}, not {sorted(unknown)}")
    if command == "play" and not request.get("files"):
        raise ValueError(f"control: play takes {', '.join(PLAY_OPTIONS)} (files required)")
    return command, request


def check_keys(keys) -> list[str]:
    """Key expressions to record: ROS name tokens, or the wildcards ``*`` and ``**``; at least one."""
    if isinstance(keys, str) or not keys:
        raise ValueError(f"keys {keys!r}: a list of key expressions")
    checked = []
    for key in keys:
        tokens = str(key).strip("/").split("/")
        topics.check_name("/".join(t for t in tokens if t not in ("*", "**")) or "x")
        checked.append("/".join(tokens))
    if len(set(checked)) != len(checked):
        raise ValueError(f"keys {keys}: one of them twice")
    return checked


def is_literal(key: str) -> bool:
    return "*" not in key




class _File:
    """One MCAP file being written: its channels and schemas. Used by the writer thread only."""

    def __init__(self, path: Path, compression: str):
        mcap = import_mcap()
        self.path = path
        self._stream = open(path, "wb")  # noqa: SIM115 - closed by finish()
        self._writer = mcap.writer.Writer(self._stream, compression=mcap.writer.CompressionType[compression.upper()])
        self._writer.start(profile="ros2", library=f"durchblick {_core.__version__}")
        self._schemas: dict[str, int] = {}
        self._channels: dict[tuple[str, str], int] = {}
        self.messages = 0
        self.bytes = 0

    def _channel(self, key: str, encoding: str, info: dict | None) -> int:
        channel = self._channels.get((key, encoding))
        if channel is not None:
            return channel
        schema, message_encoding = 0, encoding  # unknown types: unchanged, under their Zenoh encoding
        if encoding.startswith(CDR):
            name = encoding[len(CDR):]
            if name in MSGDEFS:
                if name not in self._schemas:
                    self._schemas[name] = self._writer.register_schema(name, "ros2msg", MSGDEFS[name].encode())
                schema, message_encoding = self._schemas[name], "cdr"
        elif encoding == JSON:
            message_encoding = "json"
        metadata = {"zenoh_key": key, "zenoh_encoding": encoding}
        if info is not None:
            metadata["durchblick_info"] = json.dumps(info, separators=(",", ":"))
        channel = self._writer.register_channel(f"/{key}", message_encoding, schema, metadata)
        self._channels[(key, encoding)] = channel
        return channel

    def write(self, key: str, encoding: str, payload: bytes, log_ns: int, received_ns: int, info: dict | None):
        """One sample, logged at ``log_ns``, published (MCAP's publish time) at ``received_ns``, when it came."""
        channel = self._channel(key, encoding, info)
        self._writer.add_message(channel, log_ns, payload, received_ns)
        self.messages += 1
        self.bytes += len(payload)

    def metadata(self, name: str, values: dict):
        """An MCAP metadata record: string values (others as JSON)."""
        self._writer.add_metadata(name, {k: v if isinstance(v, str) else json.dumps(v, separators=(",", ":"))
                                         for k, v in values.items()})

    def finish(self):
        self._writer.finish()
        self._stream.close()


def glob_any(pattern: str) -> bool:
    """Something matches the glob ``pattern``."""
    import glob

    return bool(glob.glob(pattern))


class Recorder:
    """The record and play node (module docstring): records the topics into ``directory`` (:meth:`record`), plays
    recordings (:meth:`play`); controlled by its methods or over Zenoh. Errors of its threads are raised by
    :meth:`check`."""

    def __init__(self, session: zenoh.Session, directory: Path, name: str = "recorder", namespace: str = "*",
                 prefix: str = "", points: bool = False, split: float = 600.0, compression: str = "lz4",
                 look_every: float = 5.0, queue_size: int = 20000):
        self.session = session
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.name = name
        self.patterns = [p.format(ns=namespace) for p in PATTERNS]
        self.prefix = prefix
        self.points = points
        self.split = split
        self.compression = compression
        self.look_every = look_every
        self._queue: queue.Queue = queue.Queue(maxsize=queue_size)
        self._lock = threading.Lock()
        self._subscribers: dict[str, zenoh.Subscriber] = {}
        self._info: dict[str, dict] = {}  # key -> the info record of its stream or detections
        self._skipped: set[str] = set()  # streams without a compressed form, said once
        self._file: _File | None = None
        self._recording = threading.Event()
        self._stopping = threading.Event()  # no new subscriptions, the discovery ends
        self._drained = threading.Event()  # the subscriptions are gone: the writer ends once the queue is empty
        self._error: BaseException | None = None
        self._write_error: BaseException | None = None  # the writer's, of the current recording
        self._player: Player | None = None
        self._player_thread: threading.Thread | None = None
        self._play_lock = threading.Lock()  # one play or stop at a time
        self._player_lock = threading.Lock()  # the swap of _player (also by its thread when it ends by itself)
        self.refused: str | None = None  # why the last command was refused
        self.failed: str | None = None  # why the last recording is not what was asked for (RecordingError)
        self.dropped = 0
        self.files: list[Path] = []
        self._threads: list[threading.Thread] = []
        # the recording's settings (record) and what it got
        self.keys: list[str] | None = None  # None: what it finds (find_streams and PATTERNS)
        self.blocking = False
        self.log_time = "receive"
        self._path: Path | None = None  # the file asked for (keys), else <name>_<UTC time>.mcap, split
        self._ready: dict | None = None
        self._recorded: list[Path] = []  # this recording's files
        self._counts_lock = threading.Lock()
        self._received: dict[str, int] = {}  # key -> samples queued for the file in this recording
        self._seqs: dict[str, array.array] = {}  # blocking: key -> their attachment seqs, in the order they came
        self._unsequenced: dict[str, str] = {}  # blocking: key -> why a sample of it has no seq
        self._ready_reports: dict[str, dict] = {}  # blocking publishers' counts at ready
        self._blocking_keys: set[str] = set()  # keys of blocking publishers: the only ones it blocks for and checks
        self._dropped_by_key: dict[str, int] = {}  # samples it dropped (queue full), by key
        self._summary: dict | None = None  # a recording with keys: what was recorded and checked (file metadata)
        self._status = session.declare_publisher(status_key(name), encoding=zenoh.Encoding.APPLICATION_JSON)
        self._status_queryable = session.declare_queryable(status_key(name), self._answer_status)
        self._control = session.declare_subscriber(control_key(name), self._on_control)
        self._control_queryable = session.declare_queryable(control_key(name), self._on_request)
        self._closed = threading.Event()
        self._status_thread = threading.Thread(target=self._publish_status, name="durchblick-record status",
                                               daemon=True)
        self._status_thread.start()
        # the playback's rate, changed while it plays (the terminal interface's Parameters tab; blick param)
        # the playback's controls (the terminal interface's Parameters tab; blick param), MCAP and video alike;
        # the values the playback changes itself (its position, its end) announced by refresh() once a second
        self._last_play: dict | None = None  # the last play's arguments: position_s set after its end plays it again
        p = self.parameters = Parameters(session, name, "durchblick")
        p.declare("rate", 1.0, range=(0.01, 100.0), description="the playback's rate: times the recorded (or the "
                  "video's) pace; applies to the playback now",
                  read=lambda: self._player.rate if self._player is not None else 1.0, write=self._set_rate)
        p.declare("paused", False, description="true pauses the playback, false resumes it",
                  read=lambda: self._player is not None and self._player.paused, write=self._set_paused)
        p.declare("position_s", 0.0, unit="s", description="where the playback is in its files; set: it goes on "
                  "from there (0: from the start; after the end: plays them again), the stamps growing",
                  read=lambda: round(self._player.position_s, 3) if self._player is not None else 0.0,
                  write=self._set_position)
        p.declare("loop", False, description="from the beginning again at the end",
                  read=lambda: self._player is not None and self._player.loop, write=self._set_loop)
        p.declare("duration_s", 0.0, unit="s", read_only=True, description="the length of the files playing",
                  read=lambda: round(self._player.duration_s, 3) if self._player is not None else 0.0)
        p.declare("file", "", read_only=True, description="the file playing now",
                  read=lambda: self._player.file if self._player is not None else "")
        p.check_overrides()

    def _playing(self, what: str):
        player = self._player
        if player is None:
            raise RuntimeError(f"nothing is playing ({what} is a playback's: blick recorder play ...)")
        return player

    @staticmethod
    def _refused(control) -> str | None:
        """A control's writer: None when done, else why not (the parameter's refusal)."""
        try:
            control()
            return None
        except (ValueError, RuntimeError, OSError) as error:
            return str(error)

    def _set_rate(self, value):
        return self._refused(lambda: self._playing("the rate").set_rate(float(value)))

    def _set_paused(self, value):
        def control():
            player = self._playing("pausing")
            player.pause() if value else player.resume()
        return self._refused(control)

    def _set_loop(self, value):
        return self._refused(lambda: self._playing("looping").set_loop(bool(value)))

    def _set_position(self, value):
        def control():
            if self._player is None and self._last_play is not None:  # played to its end: from the start again
                self.play(**self._last_play)
                if value:
                    self._player.seek(float(value))
                return
            self._playing("a position").seek(float(value))
        return self._refused(control)

    # ------------------------------------------------------------------------------------------- state

    @property
    def state(self) -> str:
        player = self._player
        if player is not None:
            return "paused" if player.paused else "playing"
        if not self._recording.is_set():
            return "idle"
        # subscribed, looking for the publishers, the file not open yet
        return "starting" if self._file is None and not self._stopping.is_set() else "recording"

    def status(self) -> dict:
        player = self._player
        with self._lock:
            f = self._file
            return {"v": STATUS_VERSION, "name": self.name, "state": self.state,
                    "file": str(f.path) if f else None, "messages": f.messages if f else 0,
                    "bytes": f.bytes if f else 0, "topics": len(self._subscribers), "dropped": self.dropped,
                    "keys": self.keys, "blocking": self.blocking, "log_time": self.log_time,
                    "files": [str(p) for p in self.files],
                    "play": player.status() if player is not None else None, "refused": self.refused,
                    "failed": self.failed, "error": repr(self._error) if self._error else None}

    def _answer_status(self, query: zenoh.Query):
        query.reply(status_key(self.name), json.dumps(self.status()), encoding=zenoh.Encoding.APPLICATION_JSON)

    def _publish_status(self):
        while not self._closed.wait(1.0):
            self._status.put(json.dumps(self.status()))
            if hasattr(self, "parameters"):
                self.parameters.refresh()

    def _on_control(self, sample: zenoh.Sample):
        threading.Thread(target=self._command, args=(sample.payload.to_bytes(),), daemon=True).start()

    def _on_request(self, query: zenoh.Query):
        payload = query.payload.to_bytes() if query.payload is not None else b""
        threading.Thread(target=self._answer_request, args=(query, payload), daemon=True).start()

    def _answer_request(self, query: zenoh.Query, payload: bytes):
        """A command sent as a query: answered once it took effect (its result) or was refused."""
        try:
            reply = self._execute(payload)
            query.reply(control_key(self.name), json.dumps(reply), encoding=zenoh.Encoding.APPLICATION_JSON)
        finally:
            query.drop()

    def _command(self, payload: bytes):
        """A control sample: done, or refused (its reason in the status and the log; the node goes on)."""
        self._execute(payload)

    def _execute(self, payload: bytes) -> dict:
        """Do a command: the reply of a request (REPLY_VERSION): ``ok`` and its ``result``, or why it was refused or
        failed (also in the status and the log)."""
        self.refused = None  # the reason of this command's refusal, if it is refused
        command = None
        try:
            command, options = parse_command(payload)
            result = {"record": lambda: self.record(**options), "stop": self.stop,
                      "pause": lambda: (self.pause(), self.status())[1],
                      "resume": lambda: (self.resume(), self.status())[1],
                      "play": lambda: self.play(**options).status(), "step": lambda: self.step(**options)}[command]()
            return {"v": REPLY_VERSION, "ok": True, "command": command, "result": result}
        except RecordingError as error:
            print(f"blick record: {command} failed: {error}", flush=True)
            return {"v": REPLY_VERSION, "ok": False, "command": command, "error": str(error), "result": error.result}
        except (ValueError, RuntimeError, OSError, json.JSONDecodeError, TypeError) as error:
            self.refused = f"{type(error).__name__}: {error}"
            print(f"blick record: refused: {self.refused}", flush=True)
            return {"v": REPLY_VERSION, "ok": False, "command": command, "refused": self.refused}

    def check(self):
        if self._error is not None:
            raise self._error

    # ------------------------------------------------------------------------------------------- start, stop

    def record(self, keys: list[str] | None = None, blocking: bool = False, log_time: str = "receive",
               file: str | None = None, ready_timeout: float = READY_TIMEOUT) -> dict:
        """Start recording (module docstring): what it finds, or exactly ``keys``, into a new file (open when this
        returns; ``file``: that one, relative to the directory or absolute, never overwritten). Returns *ready*: the
        file, the keys, the publishers found (``publishers``: blocking ones' counts; ``streams``: keys with an info
        record) and the literal keys without one (``missing``). Nothing new if it records already (and nothing
        else is asked for). Refused (RuntimeError) while it plays; RecordingError if a blocking publisher does not
        know of the subscription after ``ready_timeout`` s."""
        if self._player is not None:
            raise RuntimeError("it plays: stop the playback before recording (it would record it)")
        import_mcap()
        if log_time not in LOG_TIMES:
            raise ValueError(f"log_time {log_time!r}: one of {', '.join(LOG_TIMES)}")
        keys = check_keys(keys) if keys is not None else None
        if blocking and keys is None:
            raise ValueError("blocking needs keys: each key's count is checked against its publisher's")
        path = None
        if file is not None:
            path = Path(file) if Path(file).is_absolute() else self.directory / file
            if path.suffix != ".mcap":
                raise ValueError(f"file {file}: an .mcap file")
            if path.exists():
                raise FileExistsError(f"{path}: exists (a recording is never overwritten)")
            path.parent.mkdir(parents=True, exist_ok=True)
        with self._lock:
            if self._recording.is_set():
                if keys is None and not blocking and file is None and log_time == "receive":
                    return self._ready
                raise RuntimeError("it records already: stop that recording first")
            self._stopping.clear()
            self._drained.clear()
            self.keys, self.blocking, self.log_time, self._path = keys, blocking, log_time, path
            self._recorded, self._summary, self._ready_reports, self.failed = [], None, {}, None
            self._write_error = None
            self._blocking_keys = set()
            with self._counts_lock:
                self._received, self._seqs, self._unsequenced, self._dropped_by_key = {}, {}, {}, {}
            self._recording.set()
        try:
            for pattern in keys or self.patterns:
                self._subscribe(pattern, None)
            ready = self._look_for_publishers(ready_timeout) if keys is not None else {
                "publishers": {}, "streams": [], "missing": [], "unmatched": []}
            if blocking and ready["unmatched"]:
                raise RecordingError(f"blocking publishers of {', '.join(ready['unmatched'])} do not know of the "
                                     f"recorder's subscription after {ready_timeout} s (their samples would be lost); "
                                     "not recording", ready)
            first = self._open()
        except BaseException:
            self._abort()
            raise
        self._threads = [threading.Thread(target=self._write, args=(first,), name="durchblick-record writer",
                                          daemon=True)]
        if keys is None:
            self._threads.append(threading.Thread(target=self._look, name="durchblick-record discovery", daemon=True))
        elif blocking and not all(is_literal(k) for k in keys):
            self._threads.append(threading.Thread(target=self._watch_blocking, name="durchblick-record blocking",
                                                  daemon=True))
        for thread in self._threads:
            thread.start()
        self._ready = {"state": "recording", "file": str(first.path), "keys": keys, "blocking": blocking,
                       "log_time": log_time, **ready}
        return self._ready

    def stop(self) -> dict:
        """Stop what it does: recording (the file is complete when this returns; ``recording``: its numbers and,
        with keys, the check) or playing (``played``: the playback's status). A blocking recording that lost samples
        or could not be checked raises RecordingError (the file is complete all the same)."""
        played = self._stop_playing()
        recorded = self._stop_recording()
        return {"state": self.state, "recording": recorded, "played": played}

    def _abort(self):
        """A record refused after it subscribed: undone, no file."""
        with self._lock:
            self._stopping.set()
            subscribers, self._subscribers = list(self._subscribers.values()), {}
        self._drained.set()  # blocked deliveries give up
        for subscriber in subscribers:
            subscriber.undeclare()
        while True:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                break
        with self._lock:
            self._recording.clear()

    def _stop_recording(self) -> dict | None:
        with self._lock:
            if not self._recording.is_set() or self._stopping.is_set():
                return None
            self._stopping.set()
        reports = self._final_reports() if self.blocking else None
        with self._lock:
            subscribers, self._subscribers = list(self._subscribers.values()), {}
        for subscriber in subscribers:
            subscriber.undeclare()
        if self.keys is not None:
            with self._counts_lock:
                self._summary = (self._check_counts(reports) if self.blocking else
                                 {"v": 1, "keys": self.keys, "blocking": False, "log_time": self.log_time,
                                  "received": dict(self._received), "dropped": dict(self._dropped_by_key)})
        self._drained.set()
        for thread in self._threads:
            thread.join()
        self._threads = []
        with self._lock:
            self._recording.clear()
        with self._counts_lock:
            result = {"files": [str(p) for p in self._recorded], "received": dict(self._received),
                      "dropped": self.dropped, "check": self._summary}
        if self._write_error is not None:  # the cause of what is missing
            self.failed = f"not written completely: {self._write_error!r}"
            raise RecordingError(f"{self._recorded[-1]}: {self.failed}", result) from self._write_error
        if self._summary is not None and self._summary.get("ok") is False:
            self.failed = "; ".join(self._summary["errors"])
            raise RecordingError(f"{self._recorded[-1]}: {self.failed}", result)
        return result

    # ------------------------------------------------------------------------------------------- publishers' counts

    def _ask_sent(self) -> dict[str, list[dict]]:
        """The blocking publishers' counts of the recorded keys (``<key>/sent``), by key; several on one key: all."""
        found: dict[str, list[dict]] = {}
        for pattern in self.keys:
            if pattern.endswith(topics.TF_STATIC):
                continue
            for reply in replies(self.session, f"{pattern}/{topics.SENT}", 1.0):
                if reply.ok is None:
                    continue
                record = json.loads(reply.ok.payload.to_bytes())
                if not isinstance(record, dict) or record.get("v") != SENT_VERSION or "key" not in record:
                    raise ProtocolError(f"{reply.ok.key_expr}: {record!r} is no count of version {SENT_VERSION}")
                found.setdefault(record["key"], []).append(record)
        return found

    @staticmethod
    def _single(found: dict[str, list[dict]]) -> dict[str, dict]:
        several = sorted(k for k, records in found.items() if len(records) > 1)
        if several:
            raise RecordingError(f"several blocking publishers on {', '.join(several)}: their counts cannot be told "
                                 "apart (one publisher per key)")
        return {k: records[0] for k, records in found.items()}

    def _ask_info(self, pattern: str):
        """The info records of the streams and detections a key expression names, into _info by recorded key."""
        for suffix in ("/" + topics.COMPRESSED, "/" + topics.CLOUD, ""):
            if pattern.endswith(suffix):
                base = pattern[: len(pattern) - len(suffix)]
                break
        for reply in replies(self.session, f"{base}/{topics.INFO}", 1.0):
            if reply.ok is None:
                continue
            try:
                record = json.loads(reply.ok.payload.to_bytes())
            except json.JSONDecodeError:
                continue  # no info record (find_streams skips those too)
            if not isinstance(record, dict):
                continue
            found = str(reply.ok.key_expr)[: -len(topics.INFO) - 1]
            kind = record.get("kind")
            key = {"image": topics.compressed(found) if "compressed" in record.get("transports", ()) else None,
                   "points": f"{found}/{topics.CLOUD}", "detections": found}.get(kind)
            if key is not None:
                self._info[key] = record

    def _look_for_publishers(self, timeout: float) -> dict:
        """Look for the keys' publishers until every literal key has one (blocking ones knowing of the subscription,
        image streams with their info record) or ``timeout`` s passed: the ready reply's publishers."""
        deadline = time.monotonic() + timeout
        literal = [k for k in self.keys if is_literal(k) and not k.endswith(topics.TF_STATIC)]
        first = True
        while True:
            reports = self._single(self._ask_sent())
            for pattern in self.keys:
                if pattern.endswith(topics.TF_STATIC):
                    continue
                if first or (pattern.endswith("/" + topics.COMPRESSED) and pattern not in self._info):
                    self._ask_info(pattern)
            first = False
            missing = [k for k in literal if k not in reports and k not in self._info]
            unmatched = sorted(k for k, r in reports.items() if not r.get("matched"))
            no_info = [k for k in literal if k.endswith("/" + topics.COMPRESSED) and k not in self._info]
            if (not missing and not unmatched and not no_info) or time.monotonic() >= deadline:
                break
            time.sleep(FIND_PERIOD)
        self._ready_reports = reports
        if self.blocking:
            with self._counts_lock:
                self._blocking_keys = set(reports)
        return {"publishers": {k: {f: r[f] for f in ("sent", "seq", "matched")} for k, r in sorted(reports.items())},
                # recorded as they come, best effort: no blocking publisher (the robot's), never blocked for nor
                # checked (a wildcard: those of its keys without a blocking publisher)
                "dropping": [k for k in self.keys if k not in reports and not k.endswith(topics.TF_STATIC)],
                "streams": sorted(k for k in self._info if any(fnmatch.fnmatchcase(k, p.replace("**", "*"))
                                                               for p in self.keys)),
                "missing": missing, "unmatched": unmatched}

    def _watch_blocking(self):
        """Thread (blocking, wildcard keys): blocking publishers that appear after ready, so that it blocks for them
        and checks them (from their first sample on); the others stay dropping."""
        try:
            while not self._stopping.wait(1.0):
                found = self._single(self._ask_sent())
                with self._counts_lock:
                    self._blocking_keys |= set(found)
        except BaseException as error:  # noqa: BLE001 - raised in the caller's thread by check()
            self._error = error

    def _arrived(self, reports: dict[str, dict]) -> bool:
        """Every sample the publishers reported is here (the last seq of each key)."""
        with self._counts_lock:
            for key, report in reports.items():
                ready = self._ready_reports.get(key)
                if ready is not None and report["sent"] == ready["sent"]:
                    continue  # nothing after ready
                if report["sent"] and (key not in self._seqs or self._seqs[key][-1] < report["seq"]):
                    return False
        return True

    def _final_reports(self) -> dict[str, dict]:
        """The blocking publishers' counts at stop, asked until every key that had a count at ready or samples has one
        and its samples arrived, at most STOP_TIMEOUT s."""
        deadline = time.monotonic() + STOP_TIMEOUT
        while True:
            reports = self._single(self._ask_sent())
            with self._counts_lock:
                wanted = set(self._ready_reports) | self._blocking_keys
            if (wanted <= set(reports) and self._arrived(reports)) or time.monotonic() >= deadline:
                return reports
            time.sleep(FIND_PERIOD)

    def _check_counts(self, reports: dict[str, dict]) -> dict:
        """Each key's samples against its publisher's count (module docstring): the summary, ``ok`` and ``errors``.
        Called with _counts_lock held, after the subscriptions ended."""
        errors, counts, dropping = [], {}, {}
        blocking = set(self._ready_reports) | self._blocking_keys
        keys = set(reports) | set(self._received) | blocking | set(self._dropped_by_key)
        for key in sorted(k for k in keys if not k.endswith(topics.TF_STATIC)):
            got = self._received.get(key, 0)
            if key not in blocking:  # a dropping publisher's: best effort, not checked
                dropping[key] = {"received": got, "dropped": self._dropped_by_key.get(key, 0)}
                continue
            counts[key] = entry = {"received": got}
            report = reports.get(key)
            if key in self._unsequenced:
                errors.append(f"{key}: {self._unsequenced[key]}")
                continue
            if report is None:
                errors.append(f"{key}: its blocking publisher did not report its count at stop (gone before stop?)")
                continue
            seqs = self._seqs.get(key, array.array("q"))
            if any(b <= a for a, b in zip(seqs, seqs[1:])):
                errors.append(f"{key}: samples out of order or twice (several publishers on the key?)")
                continue
            ready = self._ready_reports.get(key)
            if ready is not None:  # what was put after ready must all be here
                expected = report["sent"] - ready["sent"]
                after = len(seqs) - bisect.bisect_right(seqs, ready["seq"])  # seqs: from when it blocked for the key
                lost = expected - after
            elif seqs:  # found after ready: from its first sample on, none between and none after the last
                lost = (seqs[-1] - seqs[0] + 1 - len(seqs)) + (report["seq"] - seqs[-1])
                expected = len(seqs) + lost
            else:
                expected, lost = report["sent"], max(0, report["sent"] - got)
            entry.update(sent=report["sent"], expected=expected, lost=lost)
            if lost:
                errors.append(f"{key}: {lost} of {expected} samples published after ready are not in the recording")
        return {"v": 1, "keys": self.keys, "blocking": True, "log_time": self.log_time, "ok": not errors,
                "errors": errors, "counts": counts, "dropping": dropping}

    # ------------------------------------------------------------------------------------------- playback

    def play(self, files: list, rate: float = 1.0, loop: bool = False, prefix: str = "", keys: list = (),
             cuda_device: int | None = None, pace: str = "wall", clock: str | None = None,
             streams: list | None = None, **video) -> Player:
        """Play ``files`` (relative to the directory, or absolute) in a thread of its own; a playback before ends.
        MCAP recordings (told by their content) play one after another (Player); videos and image sequences at once,
        one stream each (``streams``; the other options for them in ``video``: video.VideoPlayer). Refused
        (RuntimeError) while it records; a missing file is a FileNotFoundError; options of the other kind, or both
        kinds of files, a ValueError."""
        from . import video as videos

        if self._recording.is_set():
            raise RuntimeError("it records: stop the recording before playing (it would record the playback)")
        if not files:
            raise ValueError("play: no files")
        given = [str(f) if Path(f).is_absolute() else str(self.directory / f) for f in files]
        for path in given:
            if not Path(path).exists() and not (videos._is_glob(path) and glob_any(path)):
                raise FileNotFoundError(f"{path}: no such recording or video")
        kinds = {videos.is_video_input(path) for path in given}
        if kinds == {True, False}:
            raise ValueError("MCAP recordings and videos together: play one kind at a time (recordings play one "
                             "after another, videos at once)")
        is_video = kinds == {True}
        video_given = sorted(k for k, v in video.items() if v not in (None, (), [])) + (["streams"] if streams else [])
        if not is_video and video_given:
            raise ValueError(f"{', '.join(video_given)}: options for videos, these are MCAP recordings")
        if is_video:
            mcap_given = [name for name, used in (("prefix", prefix), ("keys", list(keys)), ("clock", clock),
                                                  (f"pace {pace}", pace != "wall")) if used]
            if mcap_given:
                raise ValueError(f"{', '.join(mcap_given)}: for MCAP recordings, these are videos")
            if not streams:
                raise ValueError("videos need their stream bases (streams; --stream, or --camera for one video)")
        with self._play_lock:
            self._stop_playing_locked()
            if is_video:
                player = videos.VideoPlayer(self.session, given, list(streams), rate=rate, loop=loop,
                                            cuda_device=cuda_device, **video)
            else:
                player = Player(self.session, [Path(p) for p in given], prefix=prefix, rate=rate, loop=loop,
                                keys=list(keys), cuda_device=cuda_device, pace=pace, clock=clock)
            self._last_play = dict(files=list(files), rate=rate, loop=loop, prefix=prefix, keys=list(keys),
                                   cuda_device=cuda_device, pace=pace, clock=clock, streams=streams, **video)
            thread = threading.Thread(target=self._run_player, args=(player,), name="durchblick-play", daemon=True)
            with self._player_lock:
                self._player, self._player_thread = player, thread
            thread.start()
        print(f"blick record: playing {', '.join(given)}" + (" (loop)" if loop else "")
              + (f", paced by {pace}" + (f" {clock}" if clock else "") if pace != "wall" else ""), flush=True)
        return player

    def _take_player(self, player: Player | None = None) -> Player | None:
        """The current player, which is none from now on (only if it is ``player``, when given)."""
        with self._player_lock:
            current = self._player
            if current is None or (player is not None and current is not player):
                return None
            self._player = None
            return current

    def _run_player(self, player: Player):
        player.run()
        if player.error is not None:
            self._error = player.error  # raised by check()
        if self._take_player(player) is player:  # ended by itself: the node is idle again
            player.close()
            print(f"blick record: played {player.sent} messages", flush=True)
        self.parameters.refresh()  # its end in a ParameterEvent at once

    def wait_played(self, timeout: float | None = None) -> bool:
        """Wait for the playback to end (True; at once without one)."""
        thread = self._player_thread
        if thread is not None:
            thread.join(timeout)
            return not thread.is_alive()
        return True

    def pause(self):
        self._playback("pause").pause()

    def resume(self):
        self._playback("resume").resume()

    def step(self, frames: int | None = None, until_ns: int | None = None, key: str | None = None) -> dict:
        """A playback paced by steps one step on (Player.step)."""
        return self._playback("step").step(frames=frames, until_ns=until_ns, key=key)

    def _playback(self, command: str) -> Player:
        player = self._player
        if player is None:
            raise RuntimeError(f"{command}: nothing is playing")
        return player

    def _stop_playing(self) -> dict | None:
        with self._play_lock:
            return self._stop_playing_locked()

    def _stop_playing_locked(self) -> dict | None:
        thread = self._player_thread
        player = self._take_player()
        if player is None:
            return None
        player.end()
        thread.join()
        player.close()
        print(f"blick record: playback stopped, {player.sent} messages", flush=True)
        return player.status()

    def close(self):
        try:
            self.stop()
        finally:
            self._closed.set()
            self._status_thread.join()
            self._control.undeclare()
            self._control_queryable.undeclare()
            self._status_queryable.undeclare()
            self._status.undeclare()
            self.parameters.close()

    # ------------------------------------------------------------------------------------------- threads

    def _subscribe(self, key: str, info: dict | None):
        with self._lock:
            if key in self._subscribers or not self._recording.is_set() or self._stopping.is_set():
                return
            if info is not None:
                self._info[key] = info
            self._subscribers[key] = self.session.declare_subscriber(key, self._on_sample)

    def _on_sample(self, sample: zenoh.Sample):
        key = str(sample.key_expr)
        item = (key, str(sample.encoding), sample.payload.to_bytes(), time.time_ns())
        with self._counts_lock:
            blocking = key in self._blocking_keys
        if blocking:
            # blocks: Zenoh holds the blocking publishers up meanwhile (a simulation's: they block too); never for a
            # dropping publisher's samples (the robot's), which must not be held up
            while True:
                try:
                    self._queue.put(item, timeout=0.5)
                    break
                except queue.Full:
                    if self._error is not None or self._drained.is_set():
                        self._drop(key)  # the writer failed or the recording ended: the check says so
                        return
        else:
            try:
                self._queue.put_nowait(item)
            except queue.Full:
                self._drop(key)  # the disk is behind: shown in the status
                return
        with self._counts_lock:
            self._received[key] = self._received.get(key, 0) + 1
            if blocking:
                self._sequence(key, sample.attachment)

    def _drop(self, key: str):
        with self._counts_lock:
            self.dropped += 1
            self._dropped_by_key[key] = self._dropped_by_key.get(key, 0) + 1

    def _sequence(self, key: str, attachment):
        """The attachment's seq of a blocking recording's sample (with _counts_lock)."""
        if key in self._unsequenced:
            return
        if attachment is None:
            self._unsequenced[key] = "a sample without attachment (publish it with blocking=True)"
            return
        try:
            seq = parse_attachment(attachment.to_bytes()).get("seq")
        except ValueError as error:
            self._unsequenced[key] = str(error)
            return
        if not isinstance(seq, int):
            self._unsequenced[key] = "a sample without seq in its attachment"
            return
        self._seqs.setdefault(key, array.array("q")).append(seq)

    def _look(self):
        """Thread: the streams and detections, looked for every look_every seconds."""
        try:
            while not self._stopping.is_set():
                for base, record in find_streams(self.session, self.prefix, min(2.0, self.look_every)):
                    kind = record.get("kind")
                    if kind == "image":
                        if "compressed" not in record.get("transports", ("compressed",)):
                            if base not in self._skipped:
                                self._skipped.add(base)
                                print(f"blick record: {base} has no compressed form: not recorded", flush=True)
                            continue
                        self._subscribe(topics.compressed(base), record)
                        self._subscribe(topics.camera_info(base), None)
                    elif kind == "detections":
                        self._subscribe(base, record)
                    elif kind == "points" and self.points:
                        self._subscribe(topics.cloud(base), record)
                self._stopping.wait(self.look_every)
        except BaseException as error:  # noqa: BLE001 - raised in the caller's thread by check()
            self._error = error

    def _log_ns(self, key: str, encoding: str, payload: bytes, received_ns: int) -> int:
        return received_ns if self.log_time == "receive" else header_stamp(key, encoding, payload)

    def _open(self) -> _File:
        if self._path is not None and not self._recorded:
            path = self._path
        else:
            stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H-%M-%S.%fZ")
            path = self.directory / f"{self.name}_{stamp}.mcap"
        f = _File(path, self.compression)
        with self._lock:
            self._file = f
            self.files.append(f.path)
            self._recorded.append(f.path)
        print(f"blick record: recording to {f.path}", flush=True)
        # what is published once: the cameras' fixed transforms, asked for so that every file has them
        for pattern in self.keys or self.patterns:
            if pattern.endswith(topics.TF_STATIC):
                for reply in replies(self.session, pattern, 2.0):
                    if reply.ok is not None:
                        key, encoding = str(reply.ok.key_expr), str(reply.ok.encoding)
                        payload, received = reply.ok.payload.to_bytes(), time.time_ns()
                        f.write(key, encoding, payload, self._log_ns(key, encoding, payload, received), received, None)
        return f

    def _close(self, f: _File):
        f.finish()
        print(f"blick record: {f.path} complete, {f.messages} messages, {f.bytes / 1e6:.1f} MB", flush=True)

    def _write(self, f: _File):
        """Thread: the queue into the files (``f`` the first), a new one after split seconds (not into a file asked
        for); at stop the rest of the queue, and the summary as the last file's metadata."""
        try:
            opened = time.monotonic()
            while True:
                try:
                    key, encoding, payload, received = self._queue.get(timeout=0.2)
                except queue.Empty:
                    if self._drained.is_set():
                        break
                    continue
                if self.split and self._path is None and time.monotonic() - opened > self.split:
                    self._close(f)
                    f = self._open()
                    opened = time.monotonic()
                f.write(key, encoding, payload, self._log_ns(key, encoding, payload, received), received,
                        self._info.get(key))
        except BaseException as error:  # noqa: BLE001 - raised in the caller's thread by check() and stop()
            self._error = self._write_error = error
        finally:
            if self._summary is not None:
                f.metadata(METADATA, self._summary)
            self._close(f)
            with self._lock:
                self._file = None


STATES = {"record": "recording", "stop": "idle", "play": "playing", "pause": "paused", "resume": "playing",
          "step": "playing"}


def send_command(session: zenoh.Session, name: str, command: str, **options) -> dict:
    """Send a command (COMMANDS; ``play``'s options PLAY_OPTIONS) to a record and play node and return its status once
    it took effect; a refusal is a RuntimeError with its reason, no effect within 10 s a TimeoutError. ``request``
    returns the command's own result instead (and its errors)."""
    payload = json.dumps({"command": command, **options})
    parse_command(payload.encode())  # bad ones fail here, not in the node
    before = query_status(session, name)
    session.put(control_key(name), payload)
    end = time.monotonic() + 10.0
    while time.monotonic() < end:
        status = query_status(session, name)
        if status["state"] == STATES[command] and (command != "play" or status["play"] != before["play"]):
            return status
        if status["refused"] is not None and status["refused"] != before["refused"]:
            raise RuntimeError(f"{name}: {command} refused: {status['refused']}")
        time.sleep(0.2)
    raise TimeoutError(f"{name!r} did not {command} within 10 s (its status: {status})")


def request(session: zenoh.Session, name: str, command: str, timeout: float = 30.0, **options) -> dict:
    """Send a command to a record and play node as a query and return its result once it took effect: ``record``'s
    ready, ``stop``'s numbers, ``play``'s and ``step``'s playback status. A refusal is a RuntimeError, a recording
    that failed its check a RecordingError (with the numbers in ``result``), no answer within ``timeout`` s a
    TimeoutError. The node is found first (query_status), so that the command is sent once."""
    payload = json.dumps({"command": command, **options})
    parse_command(payload.encode())  # bad ones fail here, not in the node
    query_status(session, name, timeout=min(timeout, 10.0))
    answers = [r for r in replies(session, control_key(name), timeout, payload=payload) if r.ok is not None]
    if not answers:
        raise TimeoutError(f"{name!r} did not answer {command} within {timeout} s (a node of Durchblick 0.3.28 or "
                           "newer answers commands sent as queries)")
    reply = json.loads(answers[0].ok.payload.to_bytes())
    if not isinstance(reply, dict) or reply.get("v") != REPLY_VERSION:
        raise ProtocolError(f"{name}: {reply!r} is no reply of version {REPLY_VERSION}")
    if reply["ok"]:
        return reply["result"]
    if "refused" in reply:
        raise RuntimeError(f"{name}: {command} refused: {reply['refused']}")
    raise RecordingError(f"{name}: {command}: {reply['error']}", reply.get("result"))


def query_status(session: zenoh.Session, name: str, timeout: float = 2.0) -> dict:
    """The node's status. Asked again every FIND_PERIOD seconds until ``timeout``: a session just opened knows no
    queryable yet (find_streams)."""
    end = time.monotonic() + timeout
    while (remaining := end - time.monotonic()) > 0:
        answers = [r for r in replies(session, status_key(name), min(remaining, 1.0)) if r.ok is not None]
        if answers:
            return json.loads(answers[0].ok.payload.to_string())
        time.sleep(min(FIND_PERIOD, max(0.0, end - time.monotonic())))
    raise TimeoutError(f"no record and play node {name!r} answered on {status_key(name)}")


class Recording:
    """An in-process recording of one file, e.g. an offline batch's episode, without a node of its own: a Recorder on
    ``session`` (give the publishers the same session; a private one, ``open_session(local_router=False)``, reaches
    nobody else and needs no router) records ``keys`` into ``path`` from :meth:`start` (entering a with block) to
    :meth:`stop` (leaving it), which raises RecordingError if a blocking recording lost samples. Leaving the block
    on an exception stops without raising a RecordingError over it (the file's metadata says what is missing).

        session = blick.open_session(local_router=False)
        sim = SimPublisher(session, "robot", blocking=True)        # before start: ready finds them
        with Recording(session, "episodes/0001.mcap", sim.keys) as recording:
            ...                                                     # the episode
        recording.result["check"]["counts"]
    """

    def __init__(self, session: zenoh.Session, path, keys: list[str], blocking: bool = True,
                 log_time: str = "header", ready_timeout: float = READY_TIMEOUT, name: str = "episode",
                 compression: str = "lz4"):
        self.path = Path(path).absolute()
        self.keys = check_keys(keys)
        self.blocking = blocking
        self.log_time = log_time
        self.ready_timeout = ready_timeout
        self.recorder = Recorder(session, self.path.parent, name=name, split=0, compression=compression)
        self.ready: dict | None = None
        self.result: dict | None = None

    def start(self) -> dict:
        try:
            self.ready = self.recorder.record(keys=self.keys, blocking=self.blocking, log_time=self.log_time,
                                              file=str(self.path), ready_timeout=self.ready_timeout)
        except BaseException:
            self.recorder.close()
            raise
        return self.ready

    def stop(self) -> dict:
        """The file complete; the recording's numbers (Recorder.stop's ``recording``)."""
        try:
            self.result = self.recorder.stop()["recording"]
            self.recorder.check()
        except RecordingError as error:
            self.result = error.result
            raise
        finally:
            self.recorder.close()
        return self.result

    def __enter__(self) -> Recording:
        self.start()
        return self

    def __exit__(self, kind, error, traceback):
        if kind is None:
            self.stop()
            return
        try:
            self.stop()
        except RecordingError:
            pass  # the exception that ended the block goes on


# ----------------------------------------------------------------------------------------------- playback


def message_types() -> dict[str, type]:
    """The message classes by their ROS name."""
    found, todo = {}, [Message]
    while todo:
        cls = todo.pop()
        todo.extend(cls.__subclasses__())
        if cls.TYPE:
            found[cls.TYPE] = cls
    return found


def shift_stamps(message, ns: int):
    """Every set (non-zero) Time in ``message`` moved on by ``ns``, in place, through nested messages and lists."""
    for f in dataclasses.fields(message):
        value = getattr(message, f.name)
        if isinstance(value, Time):
            if value.to_ns():
                setattr(message, f.name, Time.from_ns(value.to_ns() + ns))
        elif dataclasses.is_dataclass(value):
            shift_stamps(value, ns)
        elif isinstance(value, (list, tuple)):
            for item in value:
                if dataclasses.is_dataclass(item):
                    shift_stamps(item, ns)


FIRST_INFO_S = 2.0  # recorded seconds after the streams' first frames a stream's first camera_info is looked for


class Paced:
    """What a playback's pace needs, MCAP's (Player) and a video's (video.VideoPlayer): ``pace`` ``wall`` at ``rate``
    times the file's pace (``set_rate`` while it plays), :meth:`pause` and :meth:`resume` (the pace goes on from where
    it paused), :meth:`end`; ``step`` in lockstep, a frame (the messages of one log time) per :meth:`step`."""

    def __init__(self, rate: float, pace: str):
        if not rate > 0:
            raise ValueError(f"rate {rate}: > 0")
        if pace not in PACES:
            raise ValueError(f"pace {pace!r}: one of {', '.join(PACES)}")
        self.pace = pace
        self.rate = rate
        self.log_time_ns: int | None = None  # the log time of the last frame played (moved on with the stamps, looped)
        self.finished = False  # played to the end (or ended)
        self._grant = threading.Condition()  # lockstep: what may be played
        self._frames = 0  # frames still allowed by the current step
        self._until: int | None = None  # log time up to which frames are allowed (a step's until_ns, the clock)
        self._key: str | None = None  # a step up to the next message of this key (fnmatch) and the rest of its frame
        self._key_seen = False
        self._requested = 0  # steps asked for
        self._completed = 0  # steps done
        self._ended = threading.Event()
        self._playing = threading.Event()
        self._playing.set()
        self._shift = 0.0  # s paused so far
        self._paused_at = 0.0
        self._anchor: tuple[int, float] | None = None  # (log time, monotonic s less the pauses) the pace counts from
        self._epoch = 0  # counts the seeks: a frame waited for in an older one is not sent
        self._seek_s: float | None = None  # a seek asked for, not yet done

    @property
    def paused(self) -> bool:
        return not self._playing.is_set()

    def set_rate(self, rate: float):
        """A new rate, from the frame played last on (the next one is due its recorded period at this rate later)."""
        if not rate > 0:
            raise ValueError(f"rate {rate}: > 0")
        with self._grant:
            if self._anchor is not None and self.log_time_ns is not None:
                self._anchor = (self.log_time_ns, time.monotonic() - self._shift)
            self.rate = rate
            self._grant.notify_all()

    def step(self, frames: int | None = None, until_ns: int | None = None, key: str | None = None,
             timeout: float = 60.0) -> dict:
        """Play on (pace ``step``): ``frames`` frames (default 1), the frames up to log time ``until_ns``, or up to
        the next message on ``key`` (fnmatch) and the rest of its frame; returns once they went out: the log time of
        the last frame played, the messages sent, and whether the recording ended."""
        if self.pace != "step":
            raise RuntimeError(f"step: the playback is paced by {self.pace} (play it with pace step)")
        if sum(x is not None for x in (frames, until_ns, key)) > 1:
            raise ValueError("step: frames, until_ns or key, one of them")
        if frames is None and until_ns is None and key is None:
            frames = 1
        if frames is not None and (not isinstance(frames, int) or frames < 1):
            raise ValueError(f"step: frames {frames!r}, 1 or more")
        with self._grant:
            if not self.finished and not self._ended.is_set():
                self._frames, self._until, self._key, self._key_seen = frames or 0, until_ns, key, False
                self._requested += 1
                mine = self._requested
                self._grant.notify_all()
                if not self._grant.wait_for(lambda: self._completed >= mine or self.finished
                                            or self._ended.is_set(), timeout):
                    raise TimeoutError(f"step: not done within {timeout} s")
            return {"log_time_ns": self.log_time_ns, "sent": self.sent, "end": self.finished}

    def _admit(self, t: int) -> bool:
        """Lockstep: wait until the frame at log time ``t`` may go (True), or the end (False). A frame not allowed
        completes the step that came before."""
        with self._grant:
            while True:
                if self._ended.is_set():
                    return False
                if self._frames > 0:
                    self._frames -= 1
                    return True
                if self._until is not None and t <= self._until:
                    return True
                if self._key is not None and not self._key_seen:
                    return True
                if self.pace == "step" and self._completed < self._requested:
                    self._until, self._key = None, None
                    self._completed = self._requested
                    self._grant.notify_all()
                self._grant.wait(0.2)

    def _finish(self):
        with self._grant:
            self.finished = True
            self._grant.notify_all()

    def pause(self):
        with self._grant:
            if self._playing.is_set():
                self._playing.clear()
                self._paused_at = time.monotonic()
            self._grant.notify_all()

    def resume(self):
        with self._grant:
            if not self._playing.is_set():
                self._shift += time.monotonic() - self._paused_at  # the pace goes on from where it paused
                self._playing.set()
            self._grant.notify_all()

    def end(self):
        """Make :meth:`run` return."""
        self._ended.set()
        with self._grant:
            self._grant.notify_all()

    @property
    def duration_s(self) -> float:
        """The files' length in seconds (0: not known)."""
        return 0.0

    @property
    def file(self) -> str:
        """The file playing now."""
        return ""

    def set_loop(self, loop: bool):
        self.loop = bool(loop)

    def seek(self, position_s: float):
        """Go on from ``position_s`` seconds of the files (0: their start); the stamps go on growing (as a loop's).
        ValueError outside the files' length."""
        if not 0 <= position_s <= self.duration_s:
            raise ValueError(f"position {position_s:g} s: between 0 and the length, {self.duration_s:g} s")
        with self._grant:
            self._seek_s = float(position_s)
            self._epoch += 1
            self._grant.notify_all()

    def _wait_for(self, t: int, epoch: int | None = None) -> float | None:
        """Pace wall: wait until log time ``t`` is due at the rate (counted from the first log time waited for, or
        from ``_anchor``; paused time does not count): the monotonic time it was due, None at the end; Seeked if a
        seek came since ``epoch``. Several threads may wait at once (a video's streams)."""
        with self._grant:
            if self._anchor is None:
                self._anchor = (t, time.monotonic() - self._shift)
            while True:
                if self._ended.is_set():
                    return None
                if epoch is not None and epoch != self._epoch:
                    raise Seeked()
                if not self._playing.is_set():
                    self._grant.wait(0.2)
                    continue
                (t0, m0) = self._anchor
                due = m0 + (t - t0) / 1e9 / self.rate + self._shift
                wait = due - time.monotonic()
                if wait <= 0:
                    return due
                self._grant.wait(wait)


class Seeked(Exception):
    """Raised by Paced._wait_for when a seek came while waiting: the frame waited for is not sent."""


class Player(Paced):
    """Plays MCAP recordings (``record``'s; other publishers' channels need its metadata) back onto Zenoh, below
    ``prefix``, at ``rate`` times the recorded pace, ``loop``-ing if asked; ``keys``: only channels whose Zenoh key
    matches one of these patterns (fnmatch); ``cuda_device``: image streams also in a CUDA ring on that device.
    :meth:`run` blocks until the end (or :meth:`end`); :meth:`pause` and :meth:`resume` hold it (the recorded pace
    goes on from where it paused). ``pace`` ``step``: a frame (the messages of one log time) only when :meth:`step`
    allows it; ``clock``: up to the time of each rosgraph_msgs/Clock sample on the key ``clock``; in both every key
    blocks instead of dropping (pause and rate are for the wall clock's pace). The record and play node
    (``Recorder``) runs it."""

    def __init__(self, session: zenoh.Session, files: list[Path], prefix: str = "", rate: float = 1.0,
                 loop: bool = False, keys: list[str] = (), cuda_device: int | None = None, pace: str = "wall",
                 clock: str | None = None):
        self.mcap = import_mcap()
        super().__init__(rate, pace)
        if (pace == "clock") != (clock is not None):
            raise ValueError("pace clock needs the key of its clock (clock), and only it takes one")
        self.clock = topics.check_name(clock) if clock is not None else None
        self.session = session
        self.files = [Path(f) for f in files]
        self.prefix = topics.check_name(prefix) if prefix else ""
        self.loop = loop
        self.keys = list(keys)
        self.cuda_device = cuda_device
        self.sent = 0
        self.error: BaseException | None = None
        self.position_s = 0.0  # recorded seconds since the start of the files played
        self.passes = 0  # looped: the passes completed
        self._types: dict[str, type | None] = {}  # key -> its message class (None: JSON) for moving its stamps on
        self._unmovable: dict[str, str] = {}  # key -> why its stamps cannot be moved on (no loop, no seek then)
        self._pass_ns = 0  # looped: what each pass moves the stamps on by
        self._file = ""
        self._start_ns, self._end_ns = self._span()
        self._publishers: dict[str, zenoh.Publisher] = {}
        self._cameras: dict[str, CameraPublisher] = {}  # recorded stream base -> its publisher
        self._queryables: list = []
        self._encodings: dict[str, str] = {}  # key -> its Zenoh encoding
        self._static: dict[str, bytes] = {}  # tf_static key -> its last payload
        self._seq: dict[str, int] = {}
        self._host = _core.host_id()
        self._congestion = congestion(pace != "wall")  # lockstep: every frame reaches its readers
        self._serve()
        self._clock_subscriber = (session.declare_subscriber(self.clock, self._on_clock) if self.clock is not None
                                  else None)

    @property
    def topics(self) -> list[str]:
        """The recorded keys it plays."""
        return list(self._publishers) + [topics.compressed(b) for b in self._cameras]

    def status(self) -> dict:
        return {"kind": "mcap", "files": [str(f) for f in self.files], "position_s": round(self.position_s, 3),
                "duration_s": round(self.duration_s, 3), "file": self.file, "sent": self.sent,
                "passes": self.passes, "rate": self.rate, "loop": self.loop, "prefix": self.prefix,
                "cuda_device": self.cuda_device, "pace": self.pace, "clock": self.clock,
                "log_time_ns": self.log_time_ns, "finished": self.finished}

    # ------------------------------------------------------------------------------------------- lockstep

    def _on_clock(self, sample: zenoh.Sample):
        """A clock sample (pace clock): frames up to its time may go."""
        try:
            _check_encoding(self.clock, str(sample.encoding), Clock)
            t = Clock.deserialize(sample.payload.to_bytes()).clock.to_ns()
        except BaseException as error:  # noqa: BLE001 - ends the playback, raised by Recorder.check()
            self.error = error
            self.end()
            return
        with self._grant:
            self._until = t if self._until is None else max(self._until, t)
            self._grant.notify_all()

    # ------------------------------------------------------------------------------------------- serving

    def _out(self, key: str) -> str:
        return f"{self.prefix}/{key}" if self.prefix else key

    def _summaries(self):
        for path in self.files:
            with open(path, "rb") as stream:
                summary = self.mcap.reader.make_reader(stream).get_summary()
            if summary is None:
                raise ValueError(f"{path}: no summary (an unfinished file: mcap recover)")
            yield path, summary

    def _span(self) -> tuple[int | None, int | None]:
        """The first and the last message's log time of the files (None without statistics)."""
        start = end = None
        for _, summary in self._summaries():
            stats = summary.statistics
            if stats is None or not stats.message_count:
                return None, None
            start = stats.message_start_time if start is None else min(start, stats.message_start_time)
            end = stats.message_end_time if end is None else max(end, stats.message_end_time)
        return start, end

    @property
    def duration_s(self) -> float:
        return (self._end_ns - self._start_ns) / 1e9 if self._start_ns is not None else 0.0

    @property
    def file(self) -> str:
        return self._file

    def _movable(self, what: str):
        """ValueError unless every channel's stamps can be moved on (for a loop or a seek)."""
        if self._unmovable:
            key, why = next(iter(self._unmovable.items()))
            raise ValueError(f"{why}: it cannot be {what} (leave it out with keys)")

    def set_loop(self, loop: bool):
        if loop:
            self._movable("looped")
            if not self._pass_ns:
                self._pass_ns = self._loop_length()
        self.loop = bool(loop)

    def seek(self, position_s: float):
        self._movable("sought")
        super().seek(position_s)

    def _period_ns(self) -> int:
        """The mean period of the slowest channel (a pass's or a seek's gap in the stamps), 1 ms if not known."""
        if self._start_ns is None:
            return 1_000_000
        try:
            return self._loop_length() - (self._end_ns - self._start_ns)
        except ValueError:
            return 1_000_000

    def _loop_length(self) -> int:
        """ns a pass moves the stamps on by: from the first message to the last, and the mean period of the slowest
        channel (one message: none), so that each channel keeps its pace from one pass to the next."""
        start = end = None
        counts: dict[str, int] = {}
        for path, summary in self._summaries():
            stats = summary.statistics
            if stats is None or not stats.message_count:
                raise ValueError(f"{path}: no messages or no statistics, cannot loop it")
            start = stats.message_start_time if start is None else min(start, stats.message_start_time)
            end = stats.message_end_time if end is None else max(end, stats.message_end_time)
            for channel_id, n in stats.channel_message_counts.items():
                key = summary.channels[channel_id].metadata["zenoh_key"]
                if self._wanted(key):
                    counts[key] = counts.get(key, 0) + n
        span = end - start
        periods = [span // (n - 1) for n in counts.values() if n > 1]
        if span <= 0 or not periods:
            raise ValueError(f"{', '.join(map(str, self.files))}: shorter than two messages of a channel, cannot loop")
        return span + max(periods)

    def _wanted(self, key: str) -> bool:
        return not self.keys or any(fnmatch.fnmatchcase(key, k) for k in self.keys)

    def _first_frames(self, bases: set[str]) -> dict[str, tuple[CompressedImage | None, CameraInfo | None]]:
        """The first compressed frame (its codec) and camera_info of each image stream, from the files; a stream without
        camera_info (label images) none, looked for until FIRST_INFO_S recorded seconds after the last first frame."""
        first = {b: [None, None] for b in bases}
        framed_at = None  # log time all streams had a frame
        for log_time, key, payload in self._messages():
            if framed_at is not None and log_time - framed_at > FIRST_INFO_S * 1e9:
                break
            if key.endswith("/" + topics.COMPRESSED):
                base, i, kind = key[: -len(topics.COMPRESSED) - 1], 0, CompressedImage
            elif key.endswith("/" + topics.CAMERA_INFO):
                base, i, kind = key[: -len(topics.CAMERA_INFO) - 1], 1, CameraInfo
            else:
                continue
            if base in first and first[base][i] is None:
                first[base][i] = kind.deserialize(payload)
            if framed_at is None and all(f[0] is not None for f in first.values()):
                framed_at = log_time
            if all(f[0] is not None and f[1] is not None for f in first.values()):
                break
        return {b: tuple(f) for b, f in first.items()}

    def _serve(self):
        """Publishers and info queryables for the recordings' channels; a CameraPublisher per image stream."""
        streams = {}  # image stream base -> its recorded info record
        types = message_types()
        channels = [(c, summary.schemas.get(c.schema_id)) for _, summary in self._summaries()
                    for c in summary.channels.values()]
        for channel, schema in channels:
            key, encoding = channel.metadata.get("zenoh_key"), channel.metadata.get("zenoh_encoding")
            if key is None or encoding is None:
                raise ValueError(f"channel {channel.topic}: no zenoh_key/zenoh_encoding (not blick record's)")
            if not self._wanted(key) or key in self._publishers or key in self._types:
                continue
            if not key.endswith(topics.TF_STATIC):
                if channel.message_encoding == "json":
                    self._types[key] = None
                elif channel.message_encoding == "cdr" and schema is not None and schema.name in types:
                    self._types[key] = types[schema.name]
                else:
                    self._unmovable[key] = (f"channel {channel.topic} ({channel.message_encoding}, "
                                            f"{schema.name if schema else 'no schema'}): its stamps cannot be moved on")
            info = channel.metadata.get("durchblick_info")
            if key.endswith("/" + topics.COMPRESSED) and info is not None:
                streams[key[: -len(topics.COMPRESSED) - 1]] = json.loads(info)
                continue
            self._encodings[key] = encoding
            out = self._out(key)
            if key.endswith(topics.TF_STATIC):
                self._queryables.append(self.session.declare_queryable(out, lambda q, k=key: self._answer_static(k, q)))
            if not key.endswith("/" + topics.CAMERA_INFO):  # those go with their stream's frames
                self._publishers[key] = self.session.declare_publisher(out, encoding=zenoh.Encoding(encoding),
                                                                       congestion_control=self._congestion)
            if info is None:
                continue
            record = dict(json.loads(info), host=self._host, cuda_device_uuid=None)
            if key.endswith("/" + topics.CLOUD):
                base = key[: -len(topics.CLOUD) - 1]
                record["transports"] = ["raw"]
            else:
                base = key  # detections
            payload = json.dumps(record, separators=(",", ":"))
            self._queryables.append(self.session.declare_queryable(
                self._out(topics.info(base)), lambda q, p=payload, b=base: q.reply(self._out(topics.info(b)), p)))
        if self.loop:
            self._movable("looped")
            self._pass_ns = self._loop_length()
        for base, (frame, camera_info) in self._first_frames(set(streams)).items():
            if frame is None:
                continue  # no frame recorded
            record = streams[base]
            _, codec = parse_format(frame.format)
            self._cameras[base] = CameraPublisher(
                self.session, self._out(base), encoding=record["encoding"], frame_id=frame.header.frame_id,
                camera_info=camera_info, source=record.get("source", ""), codec=codec, cuda_device=self.cuda_device,
                labels=bool(record.get("labels")), blocking=self.pace != "wall")
        for key in [k for k in self._encodings if k.endswith("/" + topics.CAMERA_INFO)]:
            if key[: -len(topics.CAMERA_INFO) - 1] not in self._cameras:  # camera_info of a stream not played
                self._publishers[key] = self.session.declare_publisher(
                    self._out(key), encoding=zenoh.Encoding(self._encodings[key]), congestion_control=self._congestion)

    def _answer_static(self, key: str, query: zenoh.Query):
        if key in self._static:
            query.reply(self._out(key), self._static[key], encoding=zenoh.Encoding(self._encodings[key]))

    def _moved(self, key: str, payload: bytes, shift: int):
        """The sample with its stamps moved on by ``shift`` ns: a message, or JSON bytes."""
        kind = self._types[key]
        if kind is None:
            sample = json.loads(payload)
            if isinstance(sample, dict) and "stamp_ns" in sample:
                sample["stamp_ns"] += shift
            return json.dumps(sample, separators=(",", ":")).encode()
        message = kind.deserialize(payload)
        shift_stamps(message, shift)
        return message

    def _put(self, key: str, payload: bytes, shift: int = 0):
        """Publish a recorded sample, its stamps moved on by ``shift`` ns (a looped pass)."""
        self.sent += 1
        moved = self._moved(key, payload, shift) if shift and key in self._types else None
        if key.endswith("/" + topics.COMPRESSED) and key[: -len(topics.COMPRESSED) - 1] in self._cameras:
            message = moved if moved is not None else CompressedImage.deserialize(payload)
            self._cameras[key[: -len(topics.COMPRESSED) - 1]].publish_compressed(message)
            return
        if key.endswith("/" + topics.CAMERA_INFO) and key[: -len(topics.CAMERA_INFO) - 1] in self._cameras:
            message = moved if moved is not None else CameraInfo.deserialize(payload)
            self._cameras[key[: -len(topics.CAMERA_INFO) - 1]].camera_info = message
            return  # goes with the stream's next frame
        if isinstance(moved, Message):
            payload = bytes(moved.serialize())
        elif moved is not None:
            payload = moved
        seq = self._seq[key] = self._seq.get(key, 0) + 1
        self._publishers[key].put(payload, attachment=_attachment(seq, "play"))
        if key.endswith(topics.TF_STATIC):
            self._static[key] = payload

    def _messages(self, from_ns: int | None = None):
        """(log time ns, key, payload) of all files in log-time order (from log time ``from_ns`` on)."""
        for path in self.files:
            self._file = str(path)
            with open(path, "rb") as stream:
                for _, channel, message in self.mcap.reader.make_reader(stream).iter_messages(
                        log_time_order=True, **({"starting_at": from_ns} if from_ns is not None else {})):
                    key = channel.metadata["zenoh_key"]
                    if self._wanted(key):
                        yield message.log_time, key, message.data

    def run(self):
        """Play until the end of the files (with ``loop`` until :meth:`end`). An error ends it and stays in
        ``error``."""
        try:
            shift = 0  # ns the stamps of this pass are moved on by (looped passes, seeks)
            frame = None  # the log time of the frame being played (lockstep)
            from_ns = None  # a seek's log time to go on from
            while not self._ended.is_set():
                start = self._start_ns
                epoch = self._epoch
                try:
                    for log_time, key, payload in self._messages(from_ns):
                        if start is None:
                            start = log_time
                        if epoch != self._epoch:
                            raise Seeked()
                        t = log_time + shift
                        if self.pace == "wall":
                            if self._wait_for(t, epoch) is None:
                                return
                        elif t != frame:
                            if not self._admit(t):
                                return
                            frame = t
                        self._put(key, payload, shift)
                        self.position_s = (log_time - start) / 1e9
                        self.log_time_ns = t
                        if self._key is not None and fnmatch.fnmatchcase(key, self._key):
                            with self._grant:
                                self._key_seen = True
                except Seeked:
                    with self._grant:
                        position, self._seek_s = self._seek_s, None
                    if position is None:
                        continue
                    from_ns = self._start_ns + round(position * 1e9)
                    # the message at the position comes one period after the last one played: the stamps grow
                    last = self.log_time_ns if self.log_time_ns is not None else from_ns + shift
                    shift = last - from_ns + self._period_ns()
                    self.position_s = position
                    continue
                from_ns = None
                if not self.loop:
                    return
                self.passes += 1
                shift += self._pass_ns
        except BaseException as error:  # noqa: BLE001 - raised in the node's thread by Recorder.check()
            self.error = error
        finally:
            self._finish()

    def close(self):
        self.end()
        if self._clock_subscriber is not None:
            self._clock_subscriber.undeclare()
        for closable in self._queryables:
            closable.undeclare()
        for publisher in self._publishers.values():
            publisher.undeclare()
        for camera in self._cameras.values():
            camera.close()
