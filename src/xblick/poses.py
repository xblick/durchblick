"""Poses at a stamp, from TFMessages: a camera's pose in the world for an image (``<camera>/tf``), a robot's frames at a
depth frame's time (``<robot>/tf``).

The simulation stamps a camera's pose with each image; a robot's poses come with its state, at its own rate, so an
image's stamp falls between two of them. :class:`PoseBuffer` gives the pose of the stamp where there is one, else the
pose interpolated between the two around it (translation linearly, rotation along the shorter arc), if they are at
most ``max_gap`` apart; it never extrapolates.

    camera = CameraPoses(session, "robot/camera")      # <camera>/tf, frames in the world
    world_from_camera = camera.wait(image.header.stamp.to_ns(), seconds=0.2)   # 4x4 or None
"""

from __future__ import annotations

import threading
import time

import numpy as np
import zenoh

from . import topics
from .camera import TransformSubscriber
from .msg import Point, Pose, Quaternion, TFMessage, Transform, TransformStamped, Vector3


def rotation_from_quaternion(x: float, y: float, z: float, w: float) -> np.ndarray:
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def quaternion_from_rotation(r: np.ndarray) -> tuple[float, float, float, float]:
    """(x, y, z, w) of a rotation matrix, w >= 0 (Shepperd's method: the largest of w, x, y, z first)."""
    trace = np.trace(r)
    k = int(np.argmax([trace, r[0, 0], r[1, 1], r[2, 2]]))
    if k == 0:
        s = 2.0 * np.sqrt(1.0 + trace)
        q = (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, s / 4.0
    elif k == 1:
        s = 2.0 * np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
        q = s / 4.0, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s, (r[2, 1] - r[1, 2]) / s
    elif k == 2:
        s = 2.0 * np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2])
        q = (r[0, 1] + r[1, 0]) / s, s / 4.0, (r[1, 2] + r[2, 1]) / s, (r[0, 2] - r[2, 0]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1])
        q = (r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, s / 4.0, (r[1, 0] - r[0, 1]) / s
    q = np.array(q) if q[3] >= 0 else -np.array(q)
    return tuple(float(v) for v in q)


def matrix_of(t: TransformStamped) -> np.ndarray:
    """The 4x4 pose of a TransformStamped's child frame in its parent frame."""
    q, p = t.transform.rotation, t.transform.translation
    out = np.eye(4)
    out[:3, :3] = rotation_from_quaternion(q.x, q.y, q.z, q.w)
    out[:3, 3] = (p.x, p.y, p.z)
    return out


def transform_of(matrix: np.ndarray) -> Transform:
    """A 4x4 pose as a geometry_msgs/Transform (for a TransformStamped)."""
    x, y, z, w = quaternion_from_rotation(matrix[:3, :3])
    return Transform(Vector3(*map(float, matrix[:3, 3])), Quaternion(x, y, z, w))


def pose_of(matrix: np.ndarray) -> Pose:
    """A 4x4 pose as a geometry_msgs/Pose (e.g. a Detection3D's box centre)."""
    x, y, z, w = quaternion_from_rotation(matrix[:3, :3])
    return Pose(Point(*map(float, matrix[:3, 3])), Quaternion(x, y, z, w))


def interpolate(a: np.ndarray, b: np.ndarray, s: float) -> np.ndarray:
    """The pose a share ``s`` (0..1) of the way from ``a`` to ``b`` (4x4 each, or stacks of them): translation
    linearly, rotation along the shorter arc (slerp)."""
    if a.ndim == 3:
        return np.stack([interpolate(x, y, s) for x, y in zip(a, b, strict=True)])
    qa, qb = np.array(quaternion_from_rotation(a[:3, :3])), np.array(quaternion_from_rotation(b[:3, :3]))
    if np.dot(qa, qb) < 0.0:
        qb = -qb
    angle = np.arccos(np.clip(np.dot(qa, qb), -1.0, 1.0))
    q = qa + s * (qb - qa) if angle < 1e-6 else (np.sin((1 - s) * angle) * qa + np.sin(s * angle) * qb) / np.sin(angle)
    q /= np.linalg.norm(q)
    out = np.eye(4)
    out[:3, :3] = rotation_from_quaternion(*q)
    out[:3, 3] = a[:3, 3] + s * (b[:3, 3] - a[:3, 3])
    return out


class PoseBuffer:
    """Poses (4x4, or stacks of them) by stamp, the newest ``keep``; looked up at a stamp: exact, or interpolated
    between the neighbours if at most ``max_gap`` seconds apart."""

    def __init__(self, keep: int = 600, max_gap: float = 0.1):
        self.values: dict[int, np.ndarray] = {}
        self.keep = keep
        self.max_gap_ns = max_gap * 1e9
        self._lock = threading.Lock()

    def add(self, stamp_ns: int, value: np.ndarray):
        with self._lock:
            self.values[stamp_ns] = value
            while len(self.values) > self.keep:
                self.values.pop(next(iter(self.values)))

    def at(self, stamp_ns: int) -> tuple[np.ndarray | None, bool]:
        """(pose or None, final): final is False while a later pose may still make it available."""
        with self._lock:
            if stamp_ns in self.values:
                return self.values[stamp_ns], True
            before = max((t for t in self.values if t < stamp_ns), default=None)
            after = min((t for t in self.values if t > stamp_ns), default=None)
            if after is None:
                return None, False
            if before is None or after - before > self.max_gap_ns:
                return None, True
            a, b = self.values[before], self.values[after]
        return interpolate(a, b, (stamp_ns - before) / (after - before)), True

    def wait(self, stamp_ns: int, seconds: float) -> np.ndarray | None:
        """The pose at this stamp, waiting up to ``seconds`` for the poses around it; None if they do not come or are
        too far apart."""
        end = time.monotonic() + seconds
        while True:
            value, final = self.at(stamp_ns)
            if final or time.monotonic() >= end:
                return value
            time.sleep(0.003)


class CameraPoses:
    """A camera's poses in the world from ``<camera>/tf`` (or another key of the same messages, e.g. xblick-vslam's
    ``<camera>/vslam/tf``): the transforms whose parent is the world. ``frame`` is the child frame they are of."""

    def __init__(self, session: zenoh.Session, camera: str, key: str | None = None, max_gap: float = 0.1):
        self.buffer = PoseBuffer(600, max_gap)
        self.frame: str | None = None
        self._subscriber = _TfSubscriber(session, key or topics.tf(camera), self._on_message)

    def _on_message(self, message: TFMessage):
        for t in message.transforms:
            if t.header.frame_id == topics.WORLD:
                self.frame = t.child_frame_id
                self.buffer.add(t.header.stamp.to_ns(), matrix_of(t))

    def wait(self, stamp_ns: int, seconds: float) -> np.ndarray | None:
        self._subscriber.check()
        return self.buffer.wait(stamp_ns, seconds)

    def close(self):
        self._subscriber.close()


class RobotPoses:
    """A robot's frames in the world from ``<robot>/tf`` (topics.py: world -> map -> odom -> root, the robot's frames
    relative to the root): per message the world pose of every frame in it; ``frames`` their names."""

    def __init__(self, session: zenoh.Session, robot: str, max_gap: float = 0.1):
        self.robot = topics.check_name(robot)
        self.frames: list[str] | None = None
        self.buffer = PoseBuffer(200, max_gap)
        self._subscriber = _TfSubscriber(session, topics.tf(self.robot), self._on_message)

    def _on_message(self, message: TFMessage):
        links = {t.child_frame_id: (t.header.frame_id, matrix_of(t)) for t in message.transforms}
        world_from = {topics.WORLD: np.eye(4)}

        def resolve(frame: str) -> np.ndarray:
            if frame not in world_from:
                if frame not in links:
                    raise ValueError(f"{topics.tf(self.robot)}: frame {frame} is not linked to {topics.WORLD}")
                parent, pose = links[frame]
                world_from[frame] = resolve(parent) @ pose
            return world_from[frame]

        names = [t.child_frame_id for t in message.transforms]
        if self.frames is None:
            self.frames = names
        elif names != self.frames:
            raise ValueError(f"{topics.tf(self.robot)}: the frames changed")
        self.buffer.add(message.transforms[0].header.stamp.to_ns(), np.stack([resolve(n) for n in names]))

    def wait(self, stamp_ns: int, seconds: float) -> dict[str, np.ndarray] | None:
        """Every frame's pose in the world at this stamp; None as :meth:`PoseBuffer.wait`."""
        self._subscriber.check()
        stack = self.buffer.wait(stamp_ns, seconds)
        return None if stack is None else dict(zip(self.frames, stack, strict=True))

    def close(self):
        self._subscriber.close()


class _TfSubscriber(TransformSubscriber):
    """TransformSubscriber on any key of TFMessages (not only <camera>/tf)."""

    def __init__(self, session: zenoh.Session, key: str, callback):
        self.key = topics.check_name(key)
        self._callback = callback
        self._error: BaseException | None = None
        self._subscriber = session.declare_subscriber(self.key, self._on_sample)
