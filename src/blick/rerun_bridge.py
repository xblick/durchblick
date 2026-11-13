"""Camera streams in the Rerun viewer (rerun.io, open source): ``blick rerun``.

Each stream's ``image_raw/compressed`` goes to the viewer on the timeline ``capture`` (the frame's header stamp), at
the entity of the stream's base, e.g. ``robot/camera/color``: a JPEG or 8-bit PNG as it is (not decoded and
encoded again); 16-bit images decoded, ``16UC1`` as a depth image in millimetres (REP 118: in 3D views a point cloud),
``mono16`` (infrared) as a grey image. Its ``camera_info`` becomes a pinhole camera at the same entity (logged when it
changes), so the image also shows in 3D views.

Body tracking results on ``<camera>/bodies`` (JSON of durchblick-azure-kinect's ``durchblick-k4abt``, model
``K4ABT_BODY_32``; ``<camera>`` = a bridged stream's base without its last part) and on ``<camera>/<node>/bodies``
(the same JSON from a node, durchblick-gem-x's ``gem_x``) are drawn as skeletons at that topic's entity, each in its
own frame, in 3D and projected into the image of every bridged stream of that camera whose frame the camera's transforms
relate to the bodies' ``frame_id`` (or that is the same frame). The bridge sends the viewer its layout for this: a 3D
view and a 2D view per stream. The bodies' publisher is reached like every other program, through the PCs' Zenoh
routers (docs/concepts/sessions.md, "Zenoh routers").

A camera's IMU on ``<camera>/imu`` goes to time series (linear acceleration in m/s^2, angular velocity in rad/s,
x y z each), its ``<camera>/diagnostics`` to a text log (each publisher's level and message) and time series (their
numeric values), at ``<camera>/imu`` and ``<camera>/diagnostics/<publisher>``; the layout gets their views once they
arrive.

Point cloud streams (``clouds``, e.g. ``robot/camera/points`` of durchblick-points) go to the viewer as
coloured 3D points at the stream's entity, placed by the cloud's frame like a camera stream, at most ``cloud_rate``
clouds per second (a viewer keeps everything it was sent; 30 clouds of 6 MB a second fill its memory fast). On the
publisher's host they come from shared memory (the bridge needs the points themselves, so a compressed cloud would
only cost an encoding and a decoding); elsewhere compressed (``cloud/compressed``, points.py) where the publisher has
rays, and as sensor_msgs/PointCloud2, the whole cloud each time, where it has none, which a network link may not carry.
A node's cloud (``<camera>/<node>/<name>``, e.g. durchblick-tapnext's ``robot/camera/tapnext/tracks``) belongs
to that camera; with an ``object_id`` field its points are coloured by it, and with ``u, v`` (pixels of the image of
the cloud's frame) its points are also drawn in the 2D view of the bridged stream of that frame, at
``<stream>/<node>/<name>``, as 2D detections are (only those with ``visible`` 1, where the cloud has that field).

Results of vision nodes (``detections``, keys ``<camera>/<node>/<name>`` of vision_msgs, e.g.
``robot/camera/boxer/boxes_3d``): a Detection3DArray as 3D boxes at that key's entity, placed by its header's
frame like a stream (3D views; and projected into the 2D views of that camera's streams, as bodies are: the object
poses of durchblick-foundationpose and durchblick-nemo over the images); a Detection2DArray as rectangles (rotated by
their theta) in the image of the bridged stream of that camera whose frame is the header's (its 2D view), at
``<stream>/<node>/<name>``, and only once that stream's camera_info has told its frame. Each box is labelled with
its best hypothesis (``class_id score``) and its tracking id, and coloured by its class; an empty message clears them.
A Classification (classes without a place, e.g. durchblick-vjepa2's actions of a clip) as a line of a text log at that
key's entity, its hypotheses best first (``class_id score``), the layout getting a view of it once it arrives.

Label images (``segmentations``, image streams ``<camera>/<node>/<name>`` of mono8 or mono16 whose pixels are ids,
0 for none, e.g. durchblick-sam3's ``robot/camera/sam3/instances``): a segmentation image (a colour per id,
drawn over the image) in the 2D view of the bridged stream of that camera whose frame is the label image's
(its header's), at ``<stream>/<node>/<name>``, like 2D detections; the ids of an instance image are those of the
node's detections of the same stamp.

Of what it bridges, per topic on the capture timeline: the age at send (stamps.py: from the capture to the put of the
sample, on the publisher's clock alone) in ms at ``ages/<topic>``, the age at receive (from the capture to its
arrival here, which needs synchronised clocks across PCs) in ms at ``ages_received/<topic>``, and the rate (samples
arriving here a second, over the last second) in Hz at ``rates/<topic>``; the newest of each, with the samples a
second the publisher sent that never arrived (gaps in the attachment's seq), as a Markdown table at ``ages_now``
(about once a second). ``ages=False`` (``--no-ages``) leaves all of them out.

``blick rerun`` without arguments bridges everything that answers ``**/info``: image and point cloud streams,
label images (their info record says ``labels``) and detections (kind ``detections``); and looks again every
``--report`` seconds, so that a node started later appears too.

A camera's fixed transforms on ``<camera>/tf_static`` (tf2_msgs/TFMessage, e.g. the Azure Kinect's depth camera ->
colour camera) place its streams and its bodies: each at its frame's pose relative to the root of the camera's
transform tree, which is the camera's entity. Without them, or for a frame they do not name, a stream is drawn at
the camera's origin. Everything is drawn in camera axes (x right, y down, z forward: the scene's up is -y), unless a
camera's pose in the world comes on ``<camera>/tf`` (tf2_msgs/TFMessage, parent frame ``world``, e.g. from the
simulation): then that camera's entity moves through the world with it, the root of its transform tree placed
where the pose of the named frame puts it, on the capture timeline, and the scene's up is the world's z.

A node's poses of a camera in the world (``trajectories``, any key of tf2_msgs/TFMessage with parent frame ``world``,
e.g. durchblick-vslam's ``robot/camera/vslam/tf``, or ``<camera>/tf`` to compare with) go to the entity
``trajectories/<key>``, outside the moving camera: its path so far as a line, and ``pose`` with axes at the newest pose.

The viewer has no way to tell the bridge which streams it shows: the bridge subscribes to every stream it bridges for
as long as it runs, and their publishers render and encode for it all the time (a publisher that renders only while
somebody subscribes, like the teleoperation server's head camera, renders while the bridge runs). Needs the extra
``rerun`` (``pip install 'blick[rerun]'``).
"""

from __future__ import annotations

import collections
import colorsys
import json
import os
import shutil
import sys
import threading
import time
import zlib
from collections import Counter
from functools import partial
from pathlib import Path

import numpy as np
import zenoh

from . import topics
from . import stamps
from .camera import ProtocolError, StaticTransformSubscriber, TransformSubscriber, _check_encoding, parse_attachment
from .codecs import JPEG, decompress, parse_format
from .detections import DetectionSubscriber
from .msg import (CameraInfo, Classification, CompressedImage, Detection2DArray, DiagnosticArray, DiagnosticStatus, Imu,
                  TFMessage)
from .points import PointCloudSubscriber, unpack_rgb
from .poses import _TfSubscriber, rotation_from_quaternion


def rotation_of(q) -> np.ndarray:
    """The rotation of a geometry_msgs/Quaternion."""
    return rotation_from_quaternion(q.x, q.y, q.z, q.w)

_MEDIA_TYPES = {JPEG: "image/jpeg", "png": "image/png"}
_SIXTEEN_BIT = ("mono16", "16UC1")
TIMELINE = "capture"
BODIES = "bodies"
BODIES_VERSION = 1  # durchblick-azure-kinect's kBodiesVersion
FRUSTUM_M = 0.15  # a camera's image in 3D views this far in front of it (Rerun's default covered the scene)

# joint -> parent joint, in the SDK's joint order (Azure Kinect Body Tracking SDK, k4abt_joint_id_t and its documented
# joint hierarchy)
K4ABT_BODY_32 = {
    "Pelvis": None, "SpineNaval": "Pelvis", "SpineChest": "SpineNaval", "Neck": "SpineChest",
    "ClavicleLeft": "SpineChest", "ShoulderLeft": "ClavicleLeft", "ElbowLeft": "ShoulderLeft",
    "WristLeft": "ElbowLeft", "HandLeft": "WristLeft", "HandtipLeft": "HandLeft", "ThumbLeft": "WristLeft",
    "ClavicleRight": "SpineChest", "ShoulderRight": "ClavicleRight", "ElbowRight": "ShoulderRight",
    "WristRight": "ElbowRight", "HandRight": "WristRight", "HandtipRight": "HandRight", "ThumbRight": "WristRight",
    "HipLeft": "Pelvis", "KneeLeft": "HipLeft", "AnkleLeft": "KneeLeft", "FootLeft": "AnkleLeft",
    "HipRight": "Pelvis", "KneeRight": "HipRight", "AnkleRight": "KneeRight", "FootRight": "AnkleRight",
    "Head": "Neck", "Nose": "Head", "EyeLeft": "Head", "EarLeft": "Head", "EyeRight": "Head", "EarRight": "Head",
}
SKELETONS = {"K4ABT_BODY_32": K4ABT_BODY_32}


def import_rerun():
    """The rerun module, or a message on how to install it."""
    try:
        import rerun
    except ModuleNotFoundError as error:
        if error.name != "rerun":
            raise
        raise SystemExit("blick rerun needs the Rerun SDK: pip install 'blick[rerun]'") from error
    return rerun


def viewer_executable() -> str:
    """The Rerun viewer to spawn: rerun-sdk's ``rerun`` next to this Python (a venv not activated has it there, not
    on PATH), else the one on PATH; SystemExit naming both places without one."""
    beside = Path(sys.executable).parent / "rerun"
    if beside.is_file() and os.access(beside, os.X_OK):
        return str(beside)
    found = shutil.which("rerun")
    if found:
        return found
    raise SystemExit(f"blick rerun: no Rerun viewer, neither {beside} (rerun-sdk installs it next to Python: "
                     "blick[rerun]) nor 'rerun' on PATH")


AGES_LEGEND = ("age at send: capture to sent, on the sender's clock; age at receive: capture to arrival here, "
               "which needs synchronised clocks across PCs; dropped: samples the publisher sent that did not arrive "
               "(gaps in the attachment's seq); rate: samples arriving here a second")


def _seq(sample: zenoh.Sample) -> int | None:
    """The sample's attachment seq (None without one, or one this version does not read)."""
    try:
        return parse_attachment(sample.attachment.to_bytes() if sample.attachment is not None else None).get("seq")
    except ValueError:
        return None


def ages_table(rows: dict[str, dict]) -> str:
    """The newest rate, ages and drops per topic as a Markdown table with its legend."""
    def cell(row, key, form):
        return form.format(row[key]) if row.get(key) is not None else "-"

    lines = ["| topic | rate Hz | age at send ms | age at receive ms | dropped/s |", "| --- | --: | --: | --: | --: |"]
    lines += [f"| {topic} | {cell(r, 'rate', '{:.1f}')} | {cell(r, 'send', '{:.1f}')} | {cell(r, 'receive', '{:.1f}')} "
              f"| {cell(r, 'dropped', '{:d}')} |" for topic, r in sorted(rows.items())]
    return "\n".join(lines) + "\n\n" + AGES_LEGEND + "\n"


def label_color(label: str) -> tuple[int, int, int]:
    """A colour of its own for each class (the same in every run)."""
    r, g, b = colorsys.hsv_to_rgb((zlib.crc32(label.encode()) % 360) / 360.0, 0.75, 0.95)
    return int(255 * r), int(255 * g), int(255 * b)


def detection_label(detection) -> tuple[str, str]:
    """(class of the best hypothesis, its label with score and tracking id) of a Detection2D or Detection3D."""
    best = max(detection.results, key=lambda h: h.hypothesis.score, default=None)
    name = best.hypothesis.class_id if best is not None else ""
    text = f"{name} {best.hypothesis.score:.2f}" if best is not None else ""
    if detection.id:
        text = f"{text} #{detection.id}".strip()
    return name, text


def cloud_camera(key: str) -> str:
    """The camera a point cloud stream belongs to: a camera's own (``robot/camera/points``) or a node's
    (``robot/camera/tapnext/tracks``); cameras are ``<namespace>/<camera>`` (topics.py)."""
    return camera_of(key) if key.count("/") <= 2 else camera_of(camera_of(key))


def camera_of(base: str) -> str:
    """The camera a stream belongs to: its base without the last part (``robot/camera/depth`` ->
    ``robot/camera``)."""
    camera, _, _ = base.rpartition("/")
    if not camera:
        raise ValueError(f"stream {base!r} has no camera part")
    return camera


TRAIL_POINTS = 2000  # a trajectory's line keeps this many points, its oldest going first
TRAIL_STEP_M = 0.005  # a new point of the line once the pose moved this far
TRAIL_PERIOD_S = 0.2  # the line goes to the viewer this often
TRAIL_RADIUS_M = 0.005
TRAIL_AXES_M = 0.2


class RerunBridge:
    """Logs the frames and calibrations of camera streams, the point clouds of point cloud streams (``clouds``), and the
    body tracking results of their cameras, to a Rerun recording. ``rr``: the rerun module, initialised and connected
    to a viewer or file by the caller. ``frames`` counts what was logged per stream base and per bodies topic. Errors
    in Zenoh's callbacks are kept and raised by :meth:`check`. :meth:`add` bridges more of them later (a node that
    started after the bridge)."""

    def __init__(self, session: zenoh.Session, rr, bases: list[str], clouds: list[str] = (), cloud_rate: float = 5.0,
                 detections: list[str] = (), segmentations: list[str] = (), stale_after: float = 0.0,
                 ages: bool = True, trajectories: list[str] = ()):
        if not bases and not clouds:
            raise ValueError("no streams to bridge")
        self.rr = rr
        self._session = session
        self._cloud_rate = cloud_rate
        self.bases, self.clouds, self.detections, self.segmentations, self.cameras = [], [], [], [], []
        self.trajectories = []
        self._trails: dict[str, collections.deque] = {}  # trajectory key -> the points of its path
        self._trail_logged_at: dict[str, float] = {}  # trajectory key -> when its line went to the viewer (monotonic s)
        self._detections_frame: dict[str, str] = {}  # 3D detections key -> frame of its boxes
        self.frames = Counter()
        self._calibration: dict[str, tuple] = {}
        self._frame_id: dict[str, str] = {}  # stream base -> its camera's frame (camera_info's header)
        self._sixteen_bit: set[str] = set()  # streams of depth or IR images (not where 2D results are drawn)
        self._segmentation_at: set[str] = set()  # entities of label images (their annotation context logged)
        self._bodies_at: dict[str, set] = {}  # bodies entity -> ids of the bodies drawn there
        self._bodies_frame: dict[str, tuple[str, str]] = {}  # bodies topic (entity) -> its camera, its bodies' frame
        self._overlays: dict[str, list[str]] | None = None  # stream -> the bodies and 3D boxes its 2D view shows
        self._tf: dict[str, dict] = {}  # camera -> child frame -> (parent frame, rotation, translation)
        self._placed: dict[str, tuple] = {}  # entity -> the pose logged for it
        self._with_imu: set[str] = set()  # cameras whose IMU came
        self._with_diagnostics: set[str] = set()  # cameras whose diagnostics came
        self._classifications: set[str] = set()  # keys of Classifications that came
        self._ages = ages
        self._age_now: dict[str, dict] = {}  # topic -> its newest rate, ages and dropped a second
        self._arrivals: dict[str, collections.deque] = {}  # topic -> its arrival times (monotonic s), the last 1 s
        self._gaps: dict[str, collections.deque] = {}  # topic -> (arrival, samples missing before it), the last 1 s
        self._last_seq: dict[str, int] = {}
        self._age_text_at = 0.0  # when ages_now was logged last
        self._detection_by_key: dict[str, DetectionSubscriber] = {}
        self._layout = None  # what the last blueprint was made from
        self._results: dict[str, tuple[float, int]] = {}  # entity of a result on screen -> when it came, its stamp
        self._lock = threading.Lock()
        self._error: BaseException | None = None
        self._subscribers = []
        self._transforms = []
        self._detection_readers = []
        self._world_readers = []
        self._cloud_threads = []
        self._in_world = False  # a camera's pose in the world came: the scene is drawn z up
        self._closed = threading.Event()
        # every stream is drawn in its camera's axes; without this, 3D views put +y up and show the cameras upside down
        rr.log("/", rr.ViewCoordinates.RDF, static=True)
        self.add(bases, clouds, detections, segmentations, trajectories)
        self._stale_thread = None
        if stale_after > 0:
            self._stale_thread = threading.Thread(target=self._clear_stale, args=(stale_after,),
                                                  name="stale results", daemon=True)
            self._stale_thread.start()

    def _clear_stale(self, stale_after: float):
        """Bodies, boxes and masks stay in the viewer until something clears them, and a node that ended or waits (for
        the robot's poses, say) sends no empty result that would: clear what its topic did not renew for this long."""
        rr = self.rr
        try:
            while not self._closed.wait(min(0.25, stale_after)):
                with self._lock:
                    now = time.monotonic()
                    for entity, (came, stamp) in list(self._results.items()):
                        if now - came < stale_after:
                            continue
                        del self._results[entity]
                        rr.set_time(TIMELINE, timestamp=np.datetime64(stamp + int((now - came) * 1e9), "ns"))
                        bodies = self._bodies_at.get(entity)
                        if bodies is None:
                            rr.log(entity, rr.Clear(recursive=False))
                        else:
                            for body in bodies:
                                rr.log(f"{entity}/{body}", rr.Clear(recursive=False))
                            bodies.clear()
        except BaseException as error:  # noqa: BLE001 - raised in the caller's thread by check()
            self._error = error

    def add(self, bases: list[str] = (), clouds: list[str] = (), detections: list[str] = (),
            segmentations: list[str] = (), trajectories: list[str] = ()) -> list[str]:
        """Bridge these as well; those already bridged are skipped. Returns the new ones."""
        bases = [b for b in map(topics.check_name, bases) if b not in self.bases]
        clouds = [c for c in map(topics.check_name, clouds) if c not in self.clouds]
        detections = [d for d in map(topics.check_name, detections) if d not in self.detections]
        segmentations = [s for s in map(topics.check_name, segmentations) if s not in self.segmentations]
        trajectories = [t for t in map(topics.check_name, trajectories) if t not in self.trajectories]
        for key in detections + segmentations:
            if key.count("/") < 2:
                raise ValueError(f"detections or segmentation {key!r}: <camera>/<node>/<name> expected")
        both = (set(segmentations) | set(self.segmentations)) & (set(bases) | set(self.bases))
        if both:
            raise ValueError(f"{sorted(both)}: a stream and a segmentation")
        cameras = [c for c in dict.fromkeys([camera_of(b) for b in bases] + [cloud_camera(c) for c in clouds]
                                            + [camera_of(camera_of(d)) for d in detections + segmentations])
                   if c not in self.cameras]
        session = self._session
        with self._lock:
            self.bases += bases
            self.clouds += clouds
            self.detections += detections
            self.segmentations += segmentations
            self.trajectories += trajectories
            self.cameras += cameras
            self._update_blueprint()
        for base in bases:
            self._subscribers.append(session.declare_subscriber(topics.camera_info(base), partial(self._on_info, base)))
            self._subscribers.append(session.declare_subscriber(topics.compressed(base), partial(self._on_image, base)))
        for key in segmentations:
            self._subscribers.append(session.declare_subscriber(topics.compressed(key),
                                                                partial(self._on_segmentation, key)))
        for camera in cameras:
            # the Azure Kinect tracker's <camera>/bodies, and a node's below the camera (durchblick-gem-x's
            # <camera>/gem_x/bodies), each drawn as its own entity
            for key in (f"{camera}/{BODIES}", f"{camera}/*/{BODIES}"):
                self._subscribers.append(session.declare_subscriber(key, partial(self._on_bodies, camera)))
            self._subscribers.append(session.declare_subscriber(topics.imu(camera), partial(self._on_imu, camera)))
            self._subscribers.append(session.declare_subscriber(topics.diagnostics(camera),
                                                                partial(self._on_diagnostics, camera)))
            self._transforms.append(StaticTransformSubscriber(session, camera, partial(self._on_transforms, camera)))
            self._world_readers.append(TransformSubscriber(session, camera, partial(self._on_world, camera)))
        for key in trajectories:
            self._trails[key] = collections.deque(maxlen=TRAIL_POINTS)
            self._world_readers.append(_TfSubscriber(session, key, partial(self._on_trajectory, key)))
        for key in detections:
            reader = DetectionSubscriber(session, key, partial(self._on_detections, key))
            self._detection_by_key[key] = reader
            self._detection_readers.append(reader)
        for cloud in clouds:
            thread = threading.Thread(target=self._bridge_cloud, args=(session, cloud, self._cloud_rate),
                                      name=f"durchblick-rerun {cloud}", daemon=True)
            self._cloud_threads.append(thread)
            thread.start()
        return bases + clouds + detections + segmentations

    def _log_age(self, topic: str, age_ns: int | None, stamp_ns: int | None = None, seq: int | None = None):
        """A sample of ``topic`` arrived (the time set by the caller: its stamp): its rate, age at send (``age_ns``)
        and age at receive (now less ``stamp_ns``) as time series, the samples missing before it by its ``seq``;
        the newest of all at ``ages_now`` about once a second. Called with the lock held."""
        if not self._ages:
            return
        rr = self.rr
        now = time.monotonic()
        arrivals = self._arrivals.setdefault(topic, collections.deque())
        arrivals.append(now)
        while arrivals[0] < now - 1.0:
            arrivals.popleft()
        rate = (len(arrivals) - 1) / (arrivals[-1] - arrivals[0]) if len(arrivals) > 1 and arrivals[-1] > arrivals[0] \
            else 0.0
        gaps = self._gaps.setdefault(topic, collections.deque())
        if seq is not None:
            last = self._last_seq.get(topic)
            if last is not None and seq > last + 1:
                gaps.append((now, seq - last - 1))
            self._last_seq[topic] = seq  # a seq not above the last: the publisher started again
        while gaps and gaps[0][0] < now - 1.0:
            gaps.popleft()
        row = self._age_now.setdefault(topic, {})
        row["rate"], row["dropped"] = rate, sum(n for _, n in gaps)
        if len(arrivals) > 1:
            rr.log(f"rates/{topic}", rr.Scalars([rate]))
        if age_ns is not None:
            row["send"] = age_ns / 1e6
            rr.log(f"ages/{topic}", rr.Scalars([row["send"]]))
        if stamp_ns:
            row["receive"] = (time.time_ns() - stamp_ns) / 1e6
            rr.log(f"ages_received/{topic}", rr.Scalars([row["receive"]]))
        first = len(self._age_now) == 1 and self._age_text_at == 0.0
        if first or now - self._age_text_at >= 1.0:
            self._age_text_at = now
            rr.log("ages_now", rr.TextDocument(ages_table(self._age_now), media_type="text/markdown"))
        if first:
            self._update_blueprint()  # the ages' views

    def _bridge_cloud(self, session: zenoh.Session, base: str, rate: float):
        """Thread of one point cloud stream: the newest cloud, at most ``rate`` a second."""
        try:
            subscriber = PointCloudSubscriber(session, base)
            try:
                while not self._closed.is_set():
                    started = time.monotonic()
                    got = subscriber.latest(timeout=0.5)
                    if got is not None:
                        self._log_cloud(base, got)
                        self._closed.wait(max(0.0, 1.0 / rate - (time.monotonic() - started)))
            finally:
                subscriber.close()
        except BaseException as error:  # noqa: BLE001 - raised in the caller's thread by check()
            self._error = error

    def _log_cloud(self, base: str, cloud):
        points = cloud.points
        names = {f.name for f in cloud.fields}
        if not {"x", "y", "z"} <= names:
            raise ProtocolError(f"{base}: a cloud without x, y, z ({', '.join(sorted(names))})")
        xyz = np.stack([points["x"], points["y"], points["z"]], -1).reshape(-1, 3)
        valid = np.isfinite(xyz).all(axis=1)
        colors = None
        if "rgb" in names:
            colors = unpack_rgb(points["rgb"]).reshape(-1, 3)
        elif "object_id" in names:  # a colour per object, as label_color gives each class one
            ids = points["object_id"].reshape(-1)
            palette = {int(o): label_color(str(int(o))) for o in np.unique(ids)}
            colors = np.array([palette[int(o)] for o in ids], np.uint8).reshape(-1, 3)
        rr = self.rr
        with self._lock:
            entity = base
            if cloud.header.frame_id == topics.WORLD:
                # a cloud in the world (e.g. a reconstruction): beside the cameras at the root, not below the camera
                # whose pose moves; the scene is drawn z up as with the cameras' poses
                entity = f"{topics.WORLD}/{base}"
                if not self._in_world:
                    rr.log("/", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
                    self._in_world = True
            elif self._frame_id.get(base) != cloud.header.frame_id:
                self._frame_id[base] = cloud.header.frame_id
                self._place(cloud_camera(base))
            rr.set_time(TIMELINE, timestamp=np.datetime64(cloud.stamp_ns, "ns"))
            rr.log(entity, rr.Points3D(xyz[valid], colors=None if colors is None else colors[valid]))
            if {"u", "v"} <= names and base.count("/") > 2:
                self._log_cloud_2d(base, cloud, colors)
            rr.set_time(TIMELINE, timestamp=np.datetime64(cloud.stamp_ns, "ns"))
            self._log_age(base, cloud.age_at_send_ns, cloud.stamp_ns, cloud.seq)
            self.frames[base] += 1

    def _log_cloud_2d(self, base: str, cloud, colors):
        """A node's cloud with pixels (``u, v``) over the image of its frame. Called with the lock held."""
        image = self._image_of(base, cloud.header.frame_id)
        if image is None:
            self.frames[f"{base} (no image of frame {cloud.header.frame_id!r} yet)"] += 1
            return
        points = cloud.points
        uv = np.stack([points["u"], points["v"]], -1).reshape(-1, 2)
        shown = np.isfinite(uv).all(axis=1)
        if "visible" in {f.name for f in cloud.fields}:
            shown &= points["visible"].reshape(-1) == 1
        camera = cloud_camera(base)
        self.rr.log(f"{image}/{base[len(camera) + 1:]}",
                    self.rr.Points2D(uv[shown], colors=None if colors is None else colors[shown], radii=3.0))

    def _on_world(self, camera: str, message: TFMessage):
        if self._error is not None:
            return
        try:
            self._log_world(camera, message)
        except BaseException as error:  # noqa: BLE001 - raised in the caller's thread by check()
            self._error = error

    def _log_world(self, camera: str, message: TFMessage):
        rr = self.rr
        with self._lock:
            for t in message.transforms:
                if t.header.frame_id != topics.WORLD:
                    raise ProtocolError(f"{topics.tf(camera)}: a pose in {t.header.frame_id!r}, not in "
                                        f"{topics.WORLD!r}")
                # the named frame in the world, and in the root of the camera's static transforms
                r_wc = rotation_of(t.transform.rotation)
                t_wc = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
                _, r_rc, t_rc = self._pose(camera, t.child_frame_id)
                rotation = r_wc @ r_rc.T
                translation = t_wc - rotation @ t_rc
                if not self._in_world:
                    rr.log("/", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
                    self._in_world = True
                rr.set_time(TIMELINE, timestamp=np.datetime64(t.header.stamp.to_ns(), "ns"))
                rr.log(camera, rr.Transform3D(translation=translation, mat3x3=rotation))
            self.frames[topics.tf(camera)] += 1

    def _on_trajectory(self, key: str, message: TFMessage):
        if self._error is not None:
            return
        try:
            self._log_trajectory(key, message)
        except BaseException as error:  # noqa: BLE001 - raised in the caller's thread by check()
            self._error = error

    def _log_trajectory(self, key: str, message: TFMessage):
        rr = self.rr
        entity = f"trajectories/{key}"
        with self._lock:
            for t in message.transforms:
                if t.header.frame_id != topics.WORLD:
                    raise ProtocolError(f"{key}: a pose in {t.header.frame_id!r}, not in {topics.WORLD!r}")
                position = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
                if not self._in_world:
                    rr.log("/", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
                    self._in_world = True
                rr.set_time(TIMELINE, timestamp=np.datetime64(t.header.stamp.to_ns(), "ns"))
                rr.log(f"{entity}/pose", rr.Transform3D(translation=position,
                                                        mat3x3=rotation_of(t.transform.rotation)))
                rr.log(f"{entity}/pose", rr.TransformAxes3D(axis_length=TRAIL_AXES_M), static=True)
                trail = self._trails[key]
                if not trail or np.linalg.norm(position - trail[-1]) >= TRAIL_STEP_M:
                    trail.append(position)
                # the whole line each time, so not at the poses' rate: a viewer keeps everything it was sent
                now = time.monotonic()
                if len(trail) >= 2 and now - self._trail_logged_at.get(key, 0.0) >= TRAIL_PERIOD_S:
                    self._trail_logged_at[key] = now
                    rr.log(entity, rr.LineStrips3D([np.stack(trail)], colors=[label_color(key)], radii=TRAIL_RADIUS_M))
            self.frames[entity] += 1

    def _on_detections(self, key: str, message):
        if self._error is not None:
            return
        try:
            if isinstance(message, Detection2DArray):
                self._log_detections_2d(key, message)
            elif isinstance(message, Classification):
                self._log_classification(key, message)
            else:
                self._log_detections_3d(key, message)
            reader = self._detection_by_key.get(key)
            if reader is not None:
                with self._lock:
                    self.rr.set_time(TIMELINE, timestamp=np.datetime64(message.header.stamp.to_ns(), "ns"))
                    self._log_age(key, reader.age_at_send_ns, message.header.stamp.to_ns(), reader.seq)
        except BaseException as error:  # noqa: BLE001 - raised in the caller's thread by check()
            self._error = error

    def _log_detections_3d(self, key: str, message):
        rr = self.rr
        boxes = [d.bbox for d in message.detections]
        labels = [detection_label(d) for d in message.detections]
        with self._lock:
            if self._detections_frame.get(key) != message.header.frame_id:
                self._detections_frame[key] = message.header.frame_id
                self._place(camera_of(camera_of(key)))
                self._update_blueprint()
            rr.set_time(TIMELINE, timestamp=np.datetime64(message.header.stamp.to_ns(), "ns"))
            self._results[key] = (time.monotonic(), message.header.stamp.to_ns())
            if not boxes:
                rr.log(key, rr.Clear(recursive=False))
            else:
                rr.log(key, rr.Boxes3D(
                    centers=[(b.center.position.x, b.center.position.y, b.center.position.z) for b in boxes],
                    half_sizes=[(b.size.x / 2, b.size.y / 2, b.size.z / 2) for b in boxes],
                    quaternions=[rr.Quaternion(xyzw=[b.center.orientation.x, b.center.orientation.y,
                                                     b.center.orientation.z, b.center.orientation.w]) for b in boxes],
                    labels=[text for _, text in labels], colors=[label_color(name) for name, _ in labels]))
            self.frames[key] += 1

    def _log_classification(self, key: str, message: Classification):
        rr = self.rr
        best = sorted(message.results, key=lambda h: h.score, reverse=True)
        with self._lock:
            if key not in self._classifications:
                self._classifications.add(key)
                self._update_blueprint()
            rr.set_time(TIMELINE, timestamp=np.datetime64(message.header.stamp.to_ns(), "ns"))
            rr.log(key, rr.TextLog(" | ".join(f"{h.class_id} {h.score:.2f}" for h in best) or "(none)", level="INFO"))
            self.frames[key] += 1

    def _log_detections_2d(self, key: str, message):
        camera = camera_of(camera_of(key))
        rr = self.rr
        with self._lock:
            image = self._image_of(key, message.header.frame_id)
            if image is None:
                self.frames[f"{key} (no image of frame {message.header.frame_id!r} yet)"] += 1
                return
            entity = f"{image}/{key[len(camera) + 1:]}"
            rr.set_time(TIMELINE, timestamp=np.datetime64(message.header.stamp.to_ns(), "ns"))
            self._results[entity] = (time.monotonic(), message.header.stamp.to_ns())
            if not message.detections:
                rr.log(entity, rr.Clear(recursive=False))
                self.frames[key] += 1
                return
            strips, texts, colors = [], [], []
            for d in message.detections:
                c, s = np.cos(d.bbox.center.theta), np.sin(d.bbox.center.theta)
                hx, hy = d.bbox.size_x / 2, d.bbox.size_y / 2
                corners = [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy), (-hx, -hy)]
                strips.append([(d.bbox.center.position.x + c * x - s * y, d.bbox.center.position.y + s * x + c * y)
                               for x, y in corners])
                name, text = detection_label(d)
                texts.append(text)
                colors.append(label_color(name))
            rr.log(entity, rr.LineStrips2D(strips, labels=texts, colors=colors))
            self.frames[key] += 1

    def _image_of(self, key: str, frame_id: str) -> str | None:
        """The bridged stream of the camera of ``key`` (``<camera>/<node>/<name>``) in frame ``frame_id``, if its
        camera_info told it yet; a colour one before a 16-bit one (depth, IR) of the same frame. Called with the lock
        held."""
        camera = camera_of(camera_of(key))
        streams = [b for b in self.bases if camera_of(b) == camera and self._frame_id.get(b) == frame_id]
        return min(streams, key=lambda b: b in self._sixteen_bit, default=None)

    def _on_segmentation(self, key: str, sample: zenoh.Sample):
        self._guarded(self._log_segmentation, key, sample)

    def _log_segmentation(self, key: str, sample: zenoh.Sample):
        _check_encoding(str(sample.key_expr), str(sample.encoding), CompressedImage)
        message = CompressedImage.deserialize(sample.payload.to_bytes())
        ids, encoding = decompress(message)
        if encoding not in ("mono8", "mono16"):
            raise ProtocolError(f"{key}: a label image of encoding {encoding}, mono8 or mono16 expected")
        rr = self.rr
        with self._lock:
            image = self._image_of(key, message.header.frame_id)
            if image is None:
                self.frames[f"{key} (no image of frame {message.header.frame_id!r} yet)"] += 1
                return
            camera = camera_of(camera_of(key))
            entity = f"{image}/{key[len(camera) + 1:]}"
            if entity not in self._segmentation_at:
                # id 0 is "none": transparent (Rerun colours every id, 0 too, which tinted the whole image)
                rr.log(entity, rr.AnnotationContext([rr.ClassDescription(info=(0, "none", (0, 0, 0, 0)))]),
                       static=True)
                self._segmentation_at.add(entity)
            rr.set_time(TIMELINE, timestamp=np.datetime64(message.header.stamp.to_ns(), "ns"))
            self._results[entity] = (time.monotonic(), message.header.stamp.to_ns())
            rr.log(entity, rr.SegmentationImage(ids, opacity=0.5))
            self._log_age(key, stamps.age_at_send_ns(sample), message.header.stamp.to_ns(), _seq(sample))
            self.frames[key] += 1

    def _guarded(self, handle, key: str, sample: zenoh.Sample):
        if self._error is not None:
            return
        try:
            handle(key, sample)
        except BaseException as error:  # noqa: BLE001 - raised in the caller's thread by check()
            self._error = error

    def _on_info(self, base: str, sample: zenoh.Sample):
        self._guarded(self._log_info, base, sample)

    def _on_image(self, base: str, sample: zenoh.Sample):
        self._guarded(self._log_image, base, sample)

    def _on_bodies(self, camera: str, sample: zenoh.Sample):
        self._guarded(self._log_bodies, camera, sample)

    def _on_imu(self, camera: str, sample: zenoh.Sample):
        self._guarded(self._log_imu, camera, sample)

    def _on_diagnostics(self, camera: str, sample: zenoh.Sample):
        self._guarded(self._log_diagnostics, camera, sample)

    def _log_imu(self, camera: str, sample: zenoh.Sample):
        key = str(sample.key_expr)
        _check_encoding(key, str(sample.encoding), Imu)
        imu = Imu.deserialize(sample.payload.to_bytes())
        rr = self.rr
        with self._lock:
            if camera not in self._with_imu:
                for name in ("linear_acceleration", "angular_velocity"):
                    rr.log(f"{camera}/imu/{name}", rr.SeriesLines(names=["x", "y", "z"]), static=True)
                self._with_imu.add(camera)
                self._update_blueprint()
            rr.set_time(TIMELINE, timestamp=np.datetime64(imu.header.stamp.to_ns(), "ns"))
            a, w = imu.linear_acceleration, imu.angular_velocity
            rr.log(f"{camera}/imu/linear_acceleration", rr.Scalars([a.x, a.y, a.z]))
            rr.log(f"{camera}/imu/angular_velocity", rr.Scalars([w.x, w.y, w.z]))
            self.frames[key] += 1

    _LEVELS = {DiagnosticStatus.OK: ("OK", "INFO"), DiagnosticStatus.WARN: ("WARN", "WARN"),
               DiagnosticStatus.ERROR: ("ERROR", "ERROR"), DiagnosticStatus.STALE: ("STALE", "DEBUG")}

    def _log_diagnostics(self, camera: str, sample: zenoh.Sample):
        key = str(sample.key_expr)
        _check_encoding(key, str(sample.encoding), DiagnosticArray)
        diagnostics = DiagnosticArray.deserialize(sample.payload.to_bytes())
        rr = self.rr
        with self._lock:
            if camera not in self._with_diagnostics:
                self._with_diagnostics.add(camera)
                self._update_blueprint()
            rr.set_time(TIMELINE, timestamp=np.datetime64(diagnostics.header.stamp.to_ns(), "ns"))
            for status in diagnostics.status:
                entity = f"{camera}/diagnostics/{rr.escape_entity_path_part(status.name or 'unnamed')}"
                word, level = self._LEVELS.get(status.level, (str(status.level), "WARN"))
                rr.log(entity, rr.TextLog(f"{status.name} {word}: {status.message}", level=level))
                for kv in status.values:
                    try:
                        value = float(kv.value)
                    except ValueError:
                        continue  # serial, firmware, processing, ...: in the text, not a curve
                    rr.log(f"{entity}/{rr.escape_entity_path_part(kv.key)}", rr.Scalars([value]))
            self.frames[key] += 1

    def _log_info(self, base: str, sample: zenoh.Sample):
        _check_encoding(str(sample.key_expr), str(sample.encoding), CameraInfo)
        info = CameraInfo.deserialize(sample.payload.to_bytes())
        calibration = (info.width, info.height, tuple(np.asarray(info.k, float)))
        with self._lock:
            if self._frame_id.get(base) != info.header.frame_id:
                self._frame_id[base] = info.header.frame_id
                self._place(camera_of(base))
                self._update_blueprint()
            if self._calibration.get(base) == calibration:
                return
            self._calibration[base] = calibration
            rr = self.rr
            # static, as the streams' poses: logged once per calibration, it must reach a viewer that connects later
            # (--serve keeps a bounded buffer, from which images push older messages; without the pinhole a viewer
            # shows no 2D stream in the 3D view) and hold for every frame, whatever its stamp
            rr.log(base, rr.Pinhole(image_from_camera=np.asarray(info.k, float).reshape(3, 3),
                                    resolution=[info.width, info.height], camera_xyz=rr.ViewCoordinates.RDF,
                                    image_plane_distance=FRUSTUM_M), static=True)

    def _log_image(self, base: str, sample: zenoh.Sample):
        _check_encoding(str(sample.key_expr), str(sample.encoding), CompressedImage)
        message = CompressedImage.deserialize(sample.payload.to_bytes())
        original, codec = parse_format(message.format)
        rr = self.rr
        if original in _SIXTEEN_BIT:
            pixels, encoding = decompress(message)
            if pixels.dtype != np.uint16:
                raise ProtocolError(f"{base}: {original} decoded to {pixels.dtype}, not 16-bit")
            archetype = rr.DepthImage(pixels, meter=1000.0) if encoding == "16UC1" else rr.Image(pixels)
        else:
            archetype = rr.EncodedImage(contents=bytes(message.data), media_type=_MEDIA_TYPES[codec])
        with self._lock:
            if original in _SIXTEEN_BIT and base not in self._sixteen_bit:
                self._sixteen_bit.add(base)
                self._update_blueprint()  # depth and IR views go last
            rr.set_time(TIMELINE, timestamp=np.datetime64(message.header.stamp.to_ns(), "ns"))
            rr.log(base, archetype)
            self._log_age(base, stamps.age_at_send_ns(sample), message.header.stamp.to_ns(), _seq(sample))
            self.frames[base] += 1

    def _log_bodies(self, camera: str, sample: zenoh.Sample):
        key = str(sample.key_expr)
        result = json.loads(sample.payload.to_bytes())
        if result.get("v") != BODIES_VERSION:
            raise ProtocolError(f"{key}: bodies version {result.get('v')}, this bridge reads {BODIES_VERSION}")
        skeleton = SKELETONS.get(result["model"])
        if skeleton is None:
            raise NotImplementedError(f"{key}: body model {result['model']!r} (known: {', '.join(SKELETONS)})")
        joints = list(skeleton)
        entity = key
        rr = self.rr
        with self._lock:
            if self._bodies_frame.get(entity) != (camera, result["frame_id"]):
                self._bodies_frame[entity] = (camera, result["frame_id"])
                self._place(camera)
                self._update_blueprint()
            if entity not in self._bodies_at:
                rr.log(entity, rr.AnnotationContext([rr.ClassDescription(
                    info=(0, result["model"]),
                    keypoint_annotations=[(k, name) for k, name in enumerate(joints)],
                    keypoint_connections=[(joints.index(parent), k) for k, parent in enumerate(skeleton.values())
                                          if parent is not None])]), static=True)
                self._bodies_at[entity] = set()
            rr.set_time(TIMELINE, timestamp=np.datetime64(result["stamp_ns"], "ns"))
            self._results[entity] = (time.monotonic(), result["stamp_ns"])
            drawn = set()
            for body in result["bodies"]:
                positions = np.array([body["joints"][name]["position_mm"] for name in joints], np.float32) / 1000
                rr.log(f"{entity}/{body['id']}",
                       rr.Points3D(positions, class_ids=0, keypoint_ids=np.arange(len(joints))))
                drawn.add(body["id"])
            for gone in self._bodies_at[entity] - drawn:
                rr.log(f"{entity}/{gone}", rr.Clear(recursive=False))
            self._bodies_at[entity] = drawn
            self._log_age(key, stamps.age_at_send_ns(sample), result["stamp_ns"], _seq(sample))
            self.frames[key] += 1

    def _on_transforms(self, camera: str, message: TFMessage):
        tree = {t.child_frame_id: (t.header.frame_id, rotation_of(t.transform.rotation),
                                   np.array([t.transform.translation.x, t.transform.translation.y,
                                             t.transform.translation.z]))
                for t in message.transforms}
        with self._lock:
            self._tf[camera] = tree
            self._place(camera)
            self._update_blueprint()

    def _pose(self, camera: str, frame: str) -> tuple[str, np.ndarray, np.ndarray]:
        """(root frame, rotation, translation) of a frame in the root of the camera's transform tree."""
        tree = self._tf.get(camera, {})
        rotation, translation, seen = np.eye(3), np.zeros(3), {frame}
        while frame in tree:
            parent, r, t = tree[frame]
            if parent in seen:
                raise ProtocolError(f"{topics.tf_static(camera)}: the transforms form a cycle through {parent!r}")
            seen.add(parent)
            rotation, translation, frame = r @ rotation, r @ translation + t, parent
        return frame, rotation, translation

    def _place(self, camera: str):
        """Log the pose of every stream and of the bodies of a camera whose frame is known, where it changed (Rerun:
        parent from child, as tf2). Called with the lock held."""
        rr = self.rr
        entities = {b: self._frame_id.get(b) for b in self.bases if camera_of(b) == camera}
        entities.update({c: self._frame_id.get(c) for c in self.clouds if cloud_camera(c) == camera})
        entities.update({e: f for e, (c, f) in self._bodies_frame.items() if c == camera})
        for key, frame in self._detections_frame.items():
            if camera_of(camera_of(key)) == camera:
                entities[key] = frame
        for entity, frame in entities.items():
            if frame is None:
                continue
            _, rotation, translation = self._pose(camera, frame)
            pose = (tuple(rotation.reshape(-1)), tuple(translation))
            if self._placed.get(entity) != pose:
                self._placed[entity] = pose
                rr.log(entity, rr.Transform3D(translation=translation, mat3x3=rotation), static=True)

    def _update_blueprint(self):
        """The layout: one 3D view, one 2D view per stream (time series beneath); a stream in the frame tree of a
        camera's bodies or 3D boxes shows them projected into its image (Rerun draws no 3D content below a pinhole, so
        they sit beside the streams and the view names them). Sent again when that changes; called with the lock
        held."""
        def root(frame: str, camera: str) -> str:
            return self._pose(camera, frame)[0]

        drawn_in_3d = dict(self._bodies_frame)  # entity -> (camera, frame)
        drawn_in_3d.update({k: (camera_of(camera_of(k)), f) for k, f in self._detections_frame.items()})
        overlays = {b: [e for e, (c, f) in sorted(drawn_in_3d.items())
                        if c == camera_of(b) and root(self._frame_id[b], c) == root(f, c)]
                    for b in self.bases if self._frame_id.get(b) is not None}
        overlays = {b: es for b, es in overlays.items() if es}
        # the images: colour before depth and IR, those with a node's results in them first
        order = sorted(self.bases, key=lambda b: (b in self._sixteen_bit, b not in overlays, b))
        layout = (tuple(order), sorted((b, tuple(es)) for b, es in overlays.items()), sorted(self._with_imu),
                  sorted(self._with_diagnostics), sorted(self._classifications), bool(self._age_now))
        if layout == self._layout:
            return
        self._layout = layout
        self._overlays = overlays
        rrb = self.rr.blueprint
        # the 3D view on the left; on the right the images, below them classifications, IMU and diagnostics as tabs
        images = [rrb.Spatial2DView(origin=b, name=b,
                                    contents=["$origin/**"] + [f"{e}/**" for e in overlays.get(b, [])])
                  for b in order]
        series = [rrb.TextLogView(origin=key, name=key) for key in sorted(self._classifications)]
        series += [rrb.TimeSeriesView(origin=f"{camera}/imu", name=f"{camera}/imu")
                   for camera in sorted(self._with_imu)]
        for camera in sorted(self._with_diagnostics):
            series += [rrb.TextLogView(origin=f"{camera}/diagnostics", name=f"{camera}/diagnostics"),
                       rrb.TimeSeriesView(origin=f"{camera}/diagnostics", name=f"{camera}/diagnostics values")]
        if self._age_now:
            series += [rrb.TextDocumentView(origin="ages_now", name="rates and ages now"),
                       rrb.TimeSeriesView(origin="rates", name="rate Hz"),
                       rrb.TimeSeriesView(origin="ages", name="age at send ms"),
                       rrb.TimeSeriesView(origin="ages_received", name="age at receive ms")]
        side = ([rrb.Grid(*images)] if images else []) + ([rrb.Tabs(*series)] if series else [])
        if len(side) == 2:
            side = [rrb.Vertical(*side, row_shares=[3, 1])]
        self.rr.send_blueprint(rrb.Blueprint(rrb.Horizontal(rrb.Spatial3DView(origin="/", name="3D"), *side)))

    def check(self):
        """Raise the error a callback met, if any."""
        if self._error is not None:
            raise self._error
        for transforms in self._transforms:
            transforms.check()
        for reader in self._detection_readers + self._world_readers:
            reader.check()

    def close(self):
        self._closed.set()
        if self._stale_thread is not None:
            self._stale_thread.join()
            self._stale_thread = None
        for thread in self._cloud_threads:
            thread.join()
        self._cloud_threads = []
        for subscriber in self._subscribers:
            subscriber.undeclare()
        self._subscribers = []
        for transforms in self._transforms:
            transforms.close()
        self._transforms = []
        for reader in self._detection_readers + self._world_readers:
            reader.close()
        self._detection_readers = []
        self._world_readers = []
