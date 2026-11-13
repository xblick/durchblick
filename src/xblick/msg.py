"""ROS 2 message types, field for field: builtin_interfaces/Time, std_msgs/Header, sensor_msgs/Image,
sensor_msgs/CompressedImage, sensor_msgs/RegionOfInterest, sensor_msgs/CameraInfo, sensor_msgs/Imu, tf2_msgs/TFMessage
(with geometry_msgs' Vector3, Quaternion, Transform, TransformStamped), diagnostic_msgs/DiagnosticArray (with
DiagnosticStatus, KeyValue), sensor_msgs/PointCloud2 (with PointField), vision_msgs/Detection2DArray,
Detection3DArray and Classification (with their parts and geometry_msgs' Point, Pose, PoseWithCovariance), and a robot's state:
sensor_msgs/JointState, nav_msgs/Odometry (with geometry_msgs' Twist, TwistWithCovariance) and
geometry_msgs/WrenchStamped (with Wrench); std_msgs/Float64MultiArray (with MultiArrayLayout, MultiArrayDimension), a
controller's records; moveit_msgs/RobotState (with sensor_msgs/MultiDOFJointState), a controller's command; a
simulation's rosgraph_msgs/Clock.

Names, field order and meaning are those of the ROS 2 definitions, so a message here maps 1:1 to the ROS one and
serializes to the same CDR bytes (cdr.py). Only the types Durchblick uses are defined; new ones follow the same
pattern.

Durchblick's own types (package xblick_msgs, written as ROS messages would be, so that a .msg file can be made
from them): ShmImage and CudaImage, the descriptors of a frame in a ring, carry the fields of sensor_msgs/Image
except ``data``, then where the pixels are; ShmPointCloud2 and CudaPointCloud2 do the same for a point cloud.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from ._core import protocol as _protocol
from .cdr import Reader, Writer

NS_PER_S = 1_000_000_000


@dataclass
class Time:
    """builtin_interfaces/msg/Time"""

    sec: int = 0
    nanosec: int = 0

    @classmethod
    def from_ns(cls, ns: int) -> "Time":
        ns = int(ns)
        return cls(sec=ns // NS_PER_S, nanosec=ns % NS_PER_S)

    def to_ns(self) -> int:
        return self.sec * NS_PER_S + self.nanosec

    def write(self, w: Writer):
        w.i32(self.sec)
        w.u32(self.nanosec)

    @classmethod
    def read(cls, r: Reader) -> "Time":
        return cls(sec=r.i32(), nanosec=r.u32())


@dataclass
class Header:
    """std_msgs/msg/Header"""

    stamp: Time = field(default_factory=Time)
    frame_id: str = ""

    def write(self, w: Writer):
        self.stamp.write(w)
        w.string(self.frame_id)

    @classmethod
    def read(cls, r: Reader) -> "Header":
        return cls(stamp=Time.read(r), frame_id=r.string())


class Message:
    """A message type. ``TYPE`` is its name; ``SCHEMA`` and ``ENCODING`` (``application/cdr;<schema>``) come from the
    C++ core's protocol table, where Durchblick's own types carry their version (``name@version``)."""

    TYPE: str = ""

    def __init_subclass__(cls, **kwargs):
        super().__init_subclass__(**kwargs)
        if cls.TYPE:
            if cls.TYPE not in _protocol.TYPES:
                raise TypeError(f"{cls.TYPE} is not in the protocol table (cpp/include/xblick/protocol.hpp)")
            cls.SCHEMA, cls.ENCODING = _protocol.TYPES[cls.TYPE]

    def write(self, w: Writer):
        raise NotImplementedError

    @classmethod
    def read(cls, r: Reader):
        raise NotImplementedError

    def serialize(self) -> bytearray:
        w = Writer()
        self.write(w)
        return w.getvalue()

    @classmethod
    def deserialize(cls, data):
        r = Reader(data)
        message = cls.read(r)
        r.expect_end()
        return message


@dataclass
class Image(Message):
    """sensor_msgs/msg/Image. ``data`` is any bytes-like object (bytes, memoryview, contiguous uint8 array);
    after deserialize() it is a view into the received payload."""

    TYPE = "sensor_msgs/msg/Image"

    header: Header = field(default_factory=Header)
    height: int = 0
    width: int = 0
    encoding: str = ""
    is_bigendian: int = 0
    step: int = 0
    data: object = b""

    def write(self, w: Writer):
        self.header.write(w)
        w.u32(self.height)
        w.u32(self.width)
        w.string(self.encoding)
        w.u8(self.is_bigendian)
        w.u32(self.step)
        w.u8_sequence(self.data)

    @classmethod
    def read(cls, r: Reader) -> "Image":
        return cls(header=Header.read(r), height=r.u32(), width=r.u32(), encoding=r.string(),
                   is_bigendian=r.u8(), step=r.u32(), data=r.u8_sequence())


@dataclass
class CompressedImage(Message):
    """sensor_msgs/msg/CompressedImage. ``format`` follows ROS image_transport, e.g. ``"rgb8; jpeg compressed bgr8"``
    (codecs.py)."""

    TYPE = "sensor_msgs/msg/CompressedImage"

    header: Header = field(default_factory=Header)
    format: str = ""
    data: object = b""

    def write(self, w: Writer):
        self.header.write(w)
        w.string(self.format)
        w.u8_sequence(self.data)

    @classmethod
    def read(cls, r: Reader) -> "CompressedImage":
        return cls(header=Header.read(r), format=r.string(), data=r.u8_sequence())


@dataclass
class RegionOfInterest:
    """sensor_msgs/msg/RegionOfInterest"""

    x_offset: int = 0
    y_offset: int = 0
    height: int = 0
    width: int = 0
    do_rectify: bool = False

    def write(self, w: Writer):
        w.u32(self.x_offset)
        w.u32(self.y_offset)
        w.u32(self.height)
        w.u32(self.width)
        w.bool(self.do_rectify)

    @classmethod
    def read(cls, r: Reader) -> "RegionOfInterest":
        return cls(x_offset=r.u32(), y_offset=r.u32(), height=r.u32(), width=r.u32(), do_rectify=r.bool())


@dataclass
class _RingImage(Message):
    """Fields of sensor_msgs/Image without data; ``host`` is the boot id of the publisher's host."""

    header: Header = field(default_factory=Header)
    height: int = 0
    width: int = 0
    encoding: str = ""
    is_bigendian: int = 0
    step: int = 0
    host: str = ""
    slot: int = 0
    seq: int = 0

    def _write_image(self, w: Writer):
        self.header.write(w)
        w.u32(self.height)
        w.u32(self.width)
        w.string(self.encoding)
        w.u8(self.is_bigendian)
        w.u32(self.step)
        w.string(self.host)

    @staticmethod
    def _read_image(r: Reader) -> dict:
        return dict(header=Header.read(r), height=r.u32(), width=r.u32(), encoding=r.string(), is_bigendian=r.u8(),
                    step=r.u32(), host=r.string())


@dataclass
class ShmImage(_RingImage):
    """xblick_msgs/msg/ShmImage: Header header, uint32 height, uint32 width, string encoding, uint8 is_bigendian,
    uint32 step, string host, string segment, uint32 slot, uint64 seq. ``segment`` is the POSIX shared memory name
    of the host ring."""

    TYPE = "xblick_msgs/msg/ShmImage"

    segment: str = ""

    def write(self, w: Writer):
        self._write_image(w)
        w.string(self.segment)
        w.u32(self.slot)
        w.u64(self.seq)

    @classmethod
    def read(cls, r: Reader) -> "ShmImage":
        fields = cls._read_image(r)
        return cls(**fields, segment=r.string(), slot=r.u32(), seq=r.u64())


@dataclass
class CudaImage(_RingImage):
    """xblick_msgs/msg/CudaImage: Header header, uint32 height, uint32 width, string encoding, uint8 is_bigendian,
    uint32 step, string host, string endpoint, string device_uuid, uint32 slot, uint64 seq. ``endpoint`` is the
    abstract Unix socket of the CUDA ring, ``device_uuid`` the GPU that holds it (32 hex digits)."""

    TYPE = "xblick_msgs/msg/CudaImage"

    endpoint: str = ""
    device_uuid: str = ""

    def write(self, w: Writer):
        self._write_image(w)
        w.string(self.endpoint)
        w.string(self.device_uuid)
        w.u32(self.slot)
        w.u64(self.seq)

    @classmethod
    def read(cls, r: Reader) -> "CudaImage":
        fields = cls._read_image(r)
        return cls(**fields, endpoint=r.string(), device_uuid=r.string(), slot=r.u32(), seq=r.u64())


def _zeros(n):
    return lambda: np.zeros(n)


@dataclass
class CameraInfo(Message):
    """sensor_msgs/msg/CameraInfo. ``k`` (3x3), ``r`` (3x3) and ``p`` (3x4) are row-major as in ROS; ``d`` has the
    length the distortion model needs (plumb_bob: 5, rational_polynomial: 8, equidistant: 4)."""

    TYPE = "sensor_msgs/msg/CameraInfo"

    header: Header = field(default_factory=Header)
    height: int = 0
    width: int = 0
    distortion_model: str = ""
    d: np.ndarray = field(default_factory=_zeros(0))
    k: np.ndarray = field(default_factory=_zeros(9))
    r: np.ndarray = field(default_factory=_zeros(9))
    p: np.ndarray = field(default_factory=_zeros(12))
    binning_x: int = 0
    binning_y: int = 0
    roi: RegionOfInterest = field(default_factory=RegionOfInterest)

    def write(self, w: Writer):
        self.header.write(w)
        w.u32(self.height)
        w.u32(self.width)
        w.string(self.distortion_model)
        w.f64_sequence(self.d)
        w.f64_array(self.k, 9)
        w.f64_array(self.r, 9)
        w.f64_array(self.p, 12)
        w.u32(self.binning_x)
        w.u32(self.binning_y)
        self.roi.write(w)

    @classmethod
    def read(cls, r: Reader) -> "CameraInfo":
        return cls(header=Header.read(r), height=r.u32(), width=r.u32(), distortion_model=r.string(),
                   d=r.f64_sequence(), k=r.f64_array(9), r=r.f64_array(9), p=r.f64_array(12),
                   binning_x=r.u32(), binning_y=r.u32(), roi=RegionOfInterest.read(r))

    @classmethod
    def pinhole(cls, width: int, height: int, fx: float, fy: float, cx: float, cy: float,
                frame_id: str = "") -> "CameraInfo":
        """An ideal camera without distortion (simulation, rectified images): plumb_bob with zero coefficients,
        R = identity, P = [K | 0]."""
        k = np.array([fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0])
        p = np.array([fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0])
        return cls(header=Header(frame_id=frame_id), height=height, width=width, distortion_model="plumb_bob",
                   d=np.zeros(5), k=k, r=np.eye(3).reshape(-1), p=p)

    @classmethod
    def from_yaml(cls, text: str) -> "CameraInfo":
        """ROS's calibration file: the YAML of camera_calibration_parsers, as ROS's camera_calibration writes it and a
        ROS driver's ``camera_info_url`` names (``image_width``, ``image_height``, ``camera_name``, ``camera_matrix``,
        ``distortion_model``, ``distortion_coefficients``, ``rectification_matrix``, ``projection_matrix``; a matrix
        as ``rows``, ``cols``, ``data: [...]``). Exactly that format, as C++'s CameraInfo::from_yaml reads it;
        anything else raises ValueError. The header (frame) stays empty."""
        scalars: dict[str, str] = {}
        matrices: dict[str, dict] = {}
        current = None
        pending = None  # (matrix, text so far, line) of a list not closed yet

        def fail(line: int, what: str):
            raise ValueError(f"calibration file, line {line}: {what}")

        def count(text: str, line: int) -> int:
            if not text.strip().isdigit():
                fail(line, f"'{text.strip()}' is not a count")
            return int(text)

        def finish(matrix: dict, text: str, line: int, end_line: int):
            inner, _, rest = text.partition("]")
            if rest.strip():
                fail(end_line, "text after ']'")
            values = []
            for item in inner.split(",") if inner.strip() else []:
                try:
                    if "_" in item:  # Python's float() takes 1_000, C++ does not
                        raise ValueError
                    values.append(float(item.strip()))
                except ValueError:
                    fail(line, f"'{item.strip()}' is not a number")
            matrix["data"] = values

        for line, raw in enumerate(text.splitlines(), 1):
            content = raw.split("#", 1)[0]
            if pending is not None:
                matrix, so_far, start = pending
                if content.strip() and content[0] not in " \t":
                    fail(start, "list without ']'")  # the next key came first
                so_far += " " + content
                if "]" in so_far:
                    finish(matrix, so_far, start, line)
                    pending = None
                else:
                    pending = (matrix, so_far, start)
                continue
            if not content.strip() or content.strip() == "---":
                continue
            if ":" not in content:
                fail(line, "'key: value' expected")
            key, _, value = content.partition(":")
            key, value = key.strip(), value.strip()
            if content[0] not in " \t":
                current = None
                if key in ("image_width", "image_height", "camera_name", "distortion_model"):
                    if key in scalars:
                        fail(line, f"{key} twice")
                    if len(value) >= 2 and value[0] in "\"'" and value[-1] == value[0]:
                        value = value[1:-1]
                    scalars[key] = value
                elif key in ("camera_matrix", "distortion_coefficients", "rectification_matrix", "projection_matrix"):
                    if value or key in matrices:
                        fail(line, f"{key}: rows, cols and data on the lines below, once")
                    current = matrices[key] = {}
                else:
                    fail(line, f"unknown key '{key}'")
                continue
            if current is None:
                fail(line, "indented, but not below a matrix")
            if key in ("rows", "cols") and key not in current:
                current[key] = count(value, line)
            elif key == "data" and "data" not in current:
                if not value.startswith("["):
                    fail(line, "data: [ ... ] expected")
                current["data"] = None
                if "]" in value:
                    finish(current, value[1:], line, line)
                else:
                    pending = (current, value[1:], line)
            else:
                fail(line, f"'{key}' unknown or twice")
        last = len(text.splitlines())
        if pending is not None:
            fail(pending[2], "list without ']'")
        for key in ("image_width", "image_height", "distortion_model"):
            if key not in scalars:
                fail(last, f"'{key}' missing")
        for key, (rows, cols) in (("camera_matrix", (3, 3)), ("distortion_coefficients", (1, None)),
                                  ("rectification_matrix", (3, 3)), ("projection_matrix", (3, 4))):
            m = matrices.get(key)
            if m is None:
                fail(last, f"'{key}' missing")
            if "rows" not in m or "cols" not in m or "data" not in m:
                fail(last, f"{key} needs rows, cols and data")
            if m["rows"] != rows or (cols is not None and m["cols"] != cols) or len(m["data"]) != m["rows"] * m["cols"]:
                needed = f", {rows} x {cols} needed" if cols is not None else ""
                fail(last, f"{key}: {m['rows']} x {m['cols']} with {len(m['data'])} values{needed}")
        return cls(height=count(scalars["image_height"], last), width=count(scalars["image_width"], last),
                   distortion_model=scalars["distortion_model"],
                   d=np.array(matrices["distortion_coefficients"]["data"], dtype=np.float64),
                   k=np.array(matrices["camera_matrix"]["data"], dtype=np.float64),
                   r=np.array(matrices["rectification_matrix"]["data"], dtype=np.float64),
                   p=np.array(matrices["projection_matrix"]["data"], dtype=np.float64))

    @classmethod
    def read_file(cls, path) -> "CameraInfo":
        """from_yaml() of a file; its errors name the file."""
        try:
            return cls.from_yaml(Path(path).read_text())
        except ValueError as e:
            raise ValueError(f"{path}: {e}") from None

    def to_yaml(self, camera_name: str) -> str:
        """The calibration file of this CameraInfo (from_yaml() reads it back exactly)."""
        def matrix(name: str, rows: int, cols: int, data) -> str:
            values = ", ".join(repr(float(v)) for v in np.asarray(data, dtype=np.float64).reshape(-1))
            return f"{name}:\n  rows: {rows}\n  cols: {cols}\n  data: [{values}]\n"

        return (f"image_width: {self.width}\nimage_height: {self.height}\ncamera_name: {camera_name}\n"
                + matrix("camera_matrix", 3, 3, self.k) + f"distortion_model: {self.distortion_model}\n"
                + matrix("distortion_coefficients", 1, len(self.d), self.d)
                + matrix("rectification_matrix", 3, 3, self.r) + matrix("projection_matrix", 3, 4, self.p))

    def write_file(self, path, camera_name: str):
        Path(path).write_text(self.to_yaml(camera_name))


@dataclass
class Vector3:
    """geometry_msgs/msg/Vector3"""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class Quaternion:
    """geometry_msgs/msg/Quaternion"""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0


@dataclass
class Transform:
    """geometry_msgs/msg/Transform"""

    translation: Vector3 = field(default_factory=Vector3)
    rotation: Quaternion = field(default_factory=Quaternion)


@dataclass
class TransformStamped:
    """geometry_msgs/msg/TransformStamped: the pose of ``child_frame_id`` in ``header.frame_id`` (it maps a point from
    the child frame to the parent frame), as in ROS's tf2."""

    header: Header = field(default_factory=Header)
    child_frame_id: str = ""
    transform: Transform = field(default_factory=Transform)


@dataclass
class TFMessage(Message):
    """tf2_msgs/msg/TFMessage: a camera's fixed transforms on ``<camera>/tf_static`` (topics.py)."""

    TYPE = "tf2_msgs/msg/TFMessage"

    transforms: list = field(default_factory=list)

    def write(self, w: Writer):
        w.u32(len(self.transforms))
        for t in self.transforms:
            t.header.write(w)
            w.string(t.child_frame_id)
            f = t.transform
            for v in (f.translation.x, f.translation.y, f.translation.z,
                      f.rotation.x, f.rotation.y, f.rotation.z, f.rotation.w):
                w.f64(v)

    @classmethod
    def read(cls, r: Reader) -> "TFMessage":
        transforms = []
        for _ in range(r.u32()):
            header, child = Header.read(r), r.string()
            translation = Vector3(r.f64(), r.f64(), r.f64())
            rotation = Quaternion(r.f64(), r.f64(), r.f64(), r.f64())
            transforms.append(TransformStamped(header, child, Transform(translation, rotation)))
        return cls(transforms=transforms)


def _no_orientation():
    covariance = np.zeros(9)
    covariance[0] = -1.0  # ROS: no orientation
    return covariance


@dataclass
class Imu(Message):
    """sensor_msgs/msg/Imu: one sample on ``<camera>/imu``. ``orientation_covariance[0] == -1``: no orientation (ROS's
    convention); covariances of zero: unknown. Angular velocity in rad/s, linear acceleration in m/s^2."""

    TYPE = "sensor_msgs/msg/Imu"

    header: Header = field(default_factory=Header)
    orientation: Quaternion = field(default_factory=Quaternion)
    orientation_covariance: np.ndarray = field(default_factory=_no_orientation)
    angular_velocity: Vector3 = field(default_factory=Vector3)
    angular_velocity_covariance: np.ndarray = field(default_factory=_zeros(9))
    linear_acceleration: Vector3 = field(default_factory=Vector3)
    linear_acceleration_covariance: np.ndarray = field(default_factory=_zeros(9))

    def write(self, w: Writer):
        self.header.write(w)
        q = self.orientation
        for v in (q.x, q.y, q.z, q.w):
            w.f64(v)
        w.f64_array(self.orientation_covariance, 9)
        for v in (self.angular_velocity.x, self.angular_velocity.y, self.angular_velocity.z):
            w.f64(v)
        w.f64_array(self.angular_velocity_covariance, 9)
        for v in (self.linear_acceleration.x, self.linear_acceleration.y, self.linear_acceleration.z):
            w.f64(v)
        w.f64_array(self.linear_acceleration_covariance, 9)

    @classmethod
    def read(cls, r: Reader) -> "Imu":
        header = Header.read(r)
        orientation = Quaternion(r.f64(), r.f64(), r.f64(), r.f64())
        orientation_covariance = r.f64_array(9)
        angular_velocity = Vector3(r.f64(), r.f64(), r.f64())
        angular_velocity_covariance = r.f64_array(9)
        linear_acceleration = Vector3(r.f64(), r.f64(), r.f64())
        return cls(header, orientation, orientation_covariance, angular_velocity, angular_velocity_covariance,
                   linear_acceleration, r.f64_array(9))

    def __eq__(self, other):
        return (isinstance(other, Imu) and self.header == other.header and self.orientation == other.orientation
                and self.angular_velocity == other.angular_velocity
                and self.linear_acceleration == other.linear_acceleration
                and all(np.array_equal(getattr(self, f), getattr(other, f))
                        for f in ("orientation_covariance", "angular_velocity_covariance",
                                  "linear_acceleration_covariance")))


@dataclass
class KeyValue:
    """diagnostic_msgs/msg/KeyValue"""

    key: str = ""
    value: str = ""


@dataclass
class DiagnosticStatus:
    """diagnostic_msgs/msg/DiagnosticStatus. ``level``: OK, WARN, ERROR or STALE."""

    OK, WARN, ERROR, STALE = 0, 1, 2, 3

    level: int = 0
    name: str = ""
    message: str = ""
    hardware_id: str = ""
    values: list = field(default_factory=list)

    def get(self, key: str) -> str | None:
        """The value of ``key``, or None."""
        return next((kv.value for kv in self.values if kv.key == key), None)


@dataclass
class DiagnosticArray(Message):
    """diagnostic_msgs/msg/DiagnosticArray: how a camera's publishers are doing, on ``<camera>/diagnostics``."""

    TYPE = "diagnostic_msgs/msg/DiagnosticArray"

    header: Header = field(default_factory=Header)
    status: list = field(default_factory=list)

    def write(self, w: Writer):
        self.header.write(w)
        w.u32(len(self.status))
        for s in self.status:
            w.u8(s.level)
            w.string(s.name)
            w.string(s.message)
            w.string(s.hardware_id)
            w.u32(len(s.values))
            for kv in s.values:
                w.string(kv.key)
                w.string(kv.value)

    @classmethod
    def read(cls, r: Reader) -> "DiagnosticArray":
        header, status = Header.read(r), []
        for _ in range(r.u32()):
            level, name, message, hardware_id = r.u8(), r.string(), r.string(), r.string()
            values = [KeyValue(r.string(), r.string()) for _ in range(r.u32())]
            status.append(DiagnosticStatus(level, name, message, hardware_id, values))
        return cls(header=header, status=status)


@dataclass
class PointField:
    """sensor_msgs/msg/PointField: one field of a point (byte ``offset`` in the point, ``datatype``, ``count``
    elements)."""

    INT8, UINT8, INT16, UINT16, INT32, UINT32, FLOAT32, FLOAT64 = range(1, 9)

    name: str = ""
    offset: int = 0
    datatype: int = 0
    count: int = 1


POINT_FIELD_DTYPES = {PointField.INT8: np.dtype("<i1"), PointField.UINT8: np.dtype("<u1"),
                      PointField.INT16: np.dtype("<i2"), PointField.UINT16: np.dtype("<u2"),
                      PointField.INT32: np.dtype("<i4"), PointField.UINT32: np.dtype("<u4"),
                      PointField.FLOAT32: np.dtype("<f4"), PointField.FLOAT64: np.dtype("<f8")}


def _write_fields(w: Writer, fields: list):
    w.u32(len(fields))
    for f in fields:
        w.string(f.name)
        w.u32(f.offset)
        w.u8(f.datatype)
        w.u32(f.count)


def _read_fields(r: Reader) -> list:
    return [PointField(name=r.string(), offset=r.u32(), datatype=r.u8(), count=r.u32()) for _ in range(r.u32())]


@dataclass
class PointCloud2(Message):
    """sensor_msgs/msg/PointCloud2. An organized cloud has ``height`` > 1 (row and column of the image it came from);
    ``is_dense`` is False if it holds invalid (NaN) points. ``data`` as Image's."""

    TYPE = "sensor_msgs/msg/PointCloud2"

    header: Header = field(default_factory=Header)
    height: int = 0
    width: int = 0
    fields: list = field(default_factory=list)
    is_bigendian: bool = False
    point_step: int = 0
    row_step: int = 0
    data: object = b""
    is_dense: bool = False

    def write(self, w: Writer):
        self.header.write(w)
        w.u32(self.height)
        w.u32(self.width)
        _write_fields(w, self.fields)
        w.bool(self.is_bigendian)
        w.u32(self.point_step)
        w.u32(self.row_step)
        w.u8_sequence(self.data)
        w.bool(self.is_dense)

    @classmethod
    def read(cls, r: Reader) -> "PointCloud2":
        return cls(header=Header.read(r), height=r.u32(), width=r.u32(), fields=_read_fields(r),
                   is_bigendian=r.bool(), point_step=r.u32(), row_step=r.u32(), data=r.u8_sequence(),
                   is_dense=r.bool())


@dataclass
class _RingPointCloud2(Message):
    """Fields of sensor_msgs/PointCloud2 without data; ``host`` is the boot id of the publisher's host."""

    header: Header = field(default_factory=Header)
    height: int = 0
    width: int = 0
    fields: list = field(default_factory=list)
    is_bigendian: bool = False
    point_step: int = 0
    row_step: int = 0
    is_dense: bool = False
    host: str = ""
    slot: int = 0
    seq: int = 0

    def _write_cloud(self, w: Writer):
        self.header.write(w)
        w.u32(self.height)
        w.u32(self.width)
        _write_fields(w, self.fields)
        w.bool(self.is_bigendian)
        w.u32(self.point_step)
        w.u32(self.row_step)
        w.bool(self.is_dense)
        w.string(self.host)

    @staticmethod
    def _read_cloud(r: Reader) -> dict:
        return dict(header=Header.read(r), height=r.u32(), width=r.u32(), fields=_read_fields(r),
                    is_bigendian=r.bool(), point_step=r.u32(), row_step=r.u32(), is_dense=r.bool(), host=r.string())


@dataclass
class ShmPointCloud2(_RingPointCloud2):
    """xblick_msgs/msg/ShmPointCloud2: Header header, uint32 height, uint32 width, PointField[] fields,
    bool is_bigendian, uint32 point_step, uint32 row_step, bool is_dense, string host, string segment, uint32 slot,
    uint64 seq."""

    TYPE = "xblick_msgs/msg/ShmPointCloud2"

    segment: str = ""

    def write(self, w: Writer):
        self._write_cloud(w)
        w.string(self.segment)
        w.u32(self.slot)
        w.u64(self.seq)

    @classmethod
    def read(cls, r: Reader) -> "ShmPointCloud2":
        fields = cls._read_cloud(r)
        return cls(**fields, segment=r.string(), slot=r.u32(), seq=r.u64())


@dataclass
class CudaPointCloud2(_RingPointCloud2):
    """xblick_msgs/msg/CudaPointCloud2: the fields of ShmPointCloud2 up to host, then string endpoint,
    string device_uuid, uint32 slot, uint64 seq (as CudaImage)."""

    TYPE = "xblick_msgs/msg/CudaPointCloud2"

    endpoint: str = ""
    device_uuid: str = ""

    def write(self, w: Writer):
        self._write_cloud(w)
        w.string(self.endpoint)
        w.string(self.device_uuid)
        w.u32(self.slot)
        w.u64(self.seq)

    @classmethod
    def read(cls, r: Reader) -> "CudaPointCloud2":
        fields = cls._read_cloud(r)
        return cls(**fields, endpoint=r.string(), device_uuid=r.string(), slot=r.u32(), seq=r.u64())


@dataclass
class CompressedPointCloud2(Message):
    """xblick_msgs/msg/CompressedPointCloud2: a cloud whose points lie on fixed rays (points.py). Header header,
    uint32 height, uint32 width, PointField[] fields, uint32 point_step, bool is_dense, string rays, float64
    range_step, string range_format, uint8[] range, string color_format, uint8[] color.

    A point is its ray's direction times its range. ``rays`` names the table of directions (msg.Rays, answered on
    ``cloud/rays``). ``range``: a 16-bit image of the ranges in steps of ``range_step`` metres, 0 for no point, as a
    CompressedImage's ``format`` and ``data`` (codecs.py: RVL). ``color``: the points' colours as an rgb8 image the
    same way (JPEG or PNG); empty for a cloud without the field ``rgb``."""

    TYPE = "xblick_msgs/msg/CompressedPointCloud2"

    header: Header = field(default_factory=Header)
    height: int = 0
    width: int = 0
    fields: list = field(default_factory=list)
    point_step: int = 0
    is_dense: bool = False
    rays: str = ""
    range_step: float = 0.0
    range_format: str = ""
    range: object = b""
    color_format: str = ""
    color: object = b""

    def write(self, w: Writer):
        self.header.write(w)
        w.u32(self.height)
        w.u32(self.width)
        _write_fields(w, self.fields)
        w.u32(self.point_step)
        w.bool(self.is_dense)
        w.string(self.rays)
        w.f64(self.range_step)
        w.string(self.range_format)
        w.u8_sequence(self.range)
        w.string(self.color_format)
        w.u8_sequence(self.color)

    @classmethod
    def read(cls, r: Reader) -> "CompressedPointCloud2":
        return cls(header=Header.read(r), height=r.u32(), width=r.u32(), fields=_read_fields(r), point_step=r.u32(),
                   is_dense=r.bool(), rays=r.string(), range_step=r.f64(), range_format=r.string(),
                   range=r.u8_sequence(), color_format=r.string(), color=r.u8_sequence())


@dataclass
class Rays(Message):
    """xblick_msgs/msg/Rays: the directions of a CompressedPointCloud2's points. string id, uint32 height,
    uint32 width, uint8[] directions: height * width * 3 float32 (little endian), x, y, z of each point's ray.
    ``id`` names the table (a hash of its size and bytes)."""

    TYPE = "xblick_msgs/msg/Rays"

    id: str = ""
    height: int = 0
    width: int = 0
    directions: object = b""

    def write(self, w: Writer):
        w.string(self.id)
        w.u32(self.height)
        w.u32(self.width)
        w.u8_sequence(self.directions)

    @classmethod
    def read(cls, r: Reader) -> "Rays":
        return cls(id=r.string(), height=r.u32(), width=r.u32(), directions=r.u8_sequence())




# ------------------------------------------------------------------------------------------------- vision_msgs
# Detections of vision nodes (e.g. 3D boxes of Boxer on <camera>/boxer/boxes_3d): ROS's vision_msgs 4.x (humble,
# jazzy), field for field.


@dataclass
class Point:
    """geometry_msgs/msg/Point"""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class Pose:
    """geometry_msgs/msg/Pose"""

    position: Point = field(default_factory=Point)
    orientation: Quaternion = field(default_factory=Quaternion)

    def write(self, w: Writer):
        for v in (self.position.x, self.position.y, self.position.z,
                  self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w):
            w.f64(v)

    @classmethod
    def read(cls, r: Reader) -> "Pose":
        return cls(Point(r.f64(), r.f64(), r.f64()), Quaternion(r.f64(), r.f64(), r.f64(), r.f64()))


@dataclass
class PoseWithCovariance:
    """geometry_msgs/msg/PoseWithCovariance: ``covariance`` 6x6 row-major (x, y, z, rotation about x, y, z)."""

    pose: Pose = field(default_factory=Pose)
    covariance: np.ndarray = field(default_factory=_zeros(36))

    def __eq__(self, other):
        return (isinstance(other, PoseWithCovariance) and self.pose == other.pose
                and np.array_equal(self.covariance, other.covariance))


@dataclass
class ObjectHypothesis:
    """vision_msgs/msg/ObjectHypothesis: ``class_id`` (a label, e.g. an open-vocabulary prompt) and its ``score``."""

    class_id: str = ""
    score: float = 0.0


@dataclass
class ObjectHypothesisWithPose:
    """vision_msgs/msg/ObjectHypothesisWithPose"""

    hypothesis: ObjectHypothesis = field(default_factory=ObjectHypothesis)
    pose: PoseWithCovariance = field(default_factory=PoseWithCovariance)

    def write(self, w: Writer):
        w.string(self.hypothesis.class_id)
        w.f64(self.hypothesis.score)
        self.pose.pose.write(w)
        w.f64_array(self.pose.covariance, 36)

    @classmethod
    def read(cls, r: Reader) -> "ObjectHypothesisWithPose":
        hypothesis = ObjectHypothesis(r.string(), r.f64())
        pose = Pose.read(r)
        return cls(hypothesis, PoseWithCovariance(pose, r.f64_array(36)))


@dataclass
class Point2D:
    """vision_msgs/msg/Point2D"""

    x: float = 0.0
    y: float = 0.0


@dataclass
class Pose2D:
    """vision_msgs/msg/Pose2D: ``theta`` in radians."""

    position: Point2D = field(default_factory=Point2D)
    theta: float = 0.0


@dataclass
class BoundingBox2D:
    """vision_msgs/msg/BoundingBox2D: the box's centre and size in pixels (``size_x`` along the image's x)."""

    center: Pose2D = field(default_factory=Pose2D)
    size_x: float = 0.0
    size_y: float = 0.0


@dataclass
class BoundingBox3D:
    """vision_msgs/msg/BoundingBox3D: the box's centre pose and its size along the pose's axes (metres)."""

    center: Pose = field(default_factory=Pose)
    size: Vector3 = field(default_factory=Vector3)


def _write_results(w: Writer, results: list):
    w.u32(len(results))
    for result in results:
        result.write(w)


def _read_results(r: Reader) -> list:
    return [ObjectHypothesisWithPose.read(r) for _ in range(r.u32())]


@dataclass
class Detection2D:
    """vision_msgs/msg/Detection2D: one object in an image; ``id`` a tracking id (empty: none)."""

    header: Header = field(default_factory=Header)
    results: list = field(default_factory=list)
    bbox: BoundingBox2D = field(default_factory=BoundingBox2D)
    id: str = ""


@dataclass
class Detection2DArray(Message):
    """vision_msgs/msg/Detection2DArray: the objects found in one image (``header``: that image's)."""

    TYPE = "vision_msgs/msg/Detection2DArray"

    header: Header = field(default_factory=Header)
    detections: list = field(default_factory=list)

    def write(self, w: Writer):
        self.header.write(w)
        w.u32(len(self.detections))
        for d in self.detections:
            d.header.write(w)
            _write_results(w, d.results)
            for v in (d.bbox.center.position.x, d.bbox.center.position.y, d.bbox.center.theta, d.bbox.size_x,
                      d.bbox.size_y):
                w.f64(v)
            w.string(d.id)

    @classmethod
    def read(cls, r: Reader) -> "Detection2DArray":
        header, detections = Header.read(r), []
        for _ in range(r.u32()):
            h, results = Header.read(r), _read_results(r)
            bbox = BoundingBox2D(Pose2D(Point2D(r.f64(), r.f64()), r.f64()), r.f64(), r.f64())
            detections.append(Detection2D(h, results, bbox, r.string()))
        return cls(header=header, detections=detections)


@dataclass
class Detection3D:
    """vision_msgs/msg/Detection3D: one object in 3D; ``id`` a tracking id (empty: none)."""

    header: Header = field(default_factory=Header)
    results: list = field(default_factory=list)
    bbox: BoundingBox3D = field(default_factory=BoundingBox3D)
    id: str = ""


@dataclass
class Detection3DArray(Message):
    """vision_msgs/msg/Detection3DArray: the objects found in one observation (``header``: its stamp and the frame of
    the boxes' poses)."""

    TYPE = "vision_msgs/msg/Detection3DArray"

    header: Header = field(default_factory=Header)
    detections: list = field(default_factory=list)

    def write(self, w: Writer):
        self.header.write(w)
        w.u32(len(self.detections))
        for d in self.detections:
            d.header.write(w)
            _write_results(w, d.results)
            d.bbox.center.write(w)
            for v in (d.bbox.size.x, d.bbox.size.y, d.bbox.size.z):
                w.f64(v)
            w.string(d.id)

    @classmethod
    def read(cls, r: Reader) -> "Detection3DArray":
        header, detections = Header.read(r), []
        for _ in range(r.u32()):
            h, results = Header.read(r), _read_results(r)
            center = Pose.read(r)
            bbox = BoundingBox3D(center, Vector3(r.f64(), r.f64(), r.f64()))
            detections.append(Detection3D(h, results, bbox, r.string()))
        return cls(header=header, detections=detections)


@dataclass
class Classification(Message):
    """vision_msgs/msg/Classification: classes of the whole input, without a place (e.g. the actions in a clip of
    images; ``header``: its newest image's), each an ObjectHypothesis."""

    TYPE = "vision_msgs/msg/Classification"

    header: Header = field(default_factory=Header)
    results: list = field(default_factory=list)

    def write(self, w: Writer):
        self.header.write(w)
        w.u32(len(self.results))
        for h in self.results:
            w.string(h.class_id)
            w.f64(h.score)

    @classmethod
    def read(cls, r: Reader) -> "Classification":
        header = Header.read(r)
        return cls(header=header, results=[ObjectHypothesis(r.string(), r.f64()) for _ in range(r.u32())])


# ------------------------------------------------------------------------------------------- robot state
# A robot's state below its namespace (topics.py): sensor_msgs/JointState, nav_msgs/Odometry (with geometry_msgs'
# Twist, TwistWithCovariance) and geometry_msgs/WrenchStamped (with Wrench), field for field.


def _empty():
    return np.zeros(0)


@dataclass
class JointState(Message):
    """sensor_msgs/msg/JointState: ``position`` (rad, m), ``velocity`` and ``effort`` (Nm, N) of the joints in ``name``,
    each as long as ``name`` or empty (the source does not have that quantity), as ROS allows."""

    TYPE = "sensor_msgs/msg/JointState"

    header: Header = field(default_factory=Header)
    name: list = field(default_factory=list)
    position: np.ndarray = field(default_factory=_empty)
    velocity: np.ndarray = field(default_factory=_empty)
    effort: np.ndarray = field(default_factory=_empty)

    def write(self, w: Writer):
        for values in (self.position, self.velocity, self.effort):
            if len(values) not in (0, len(self.name)):
                raise ValueError(f"JointState: {len(values)} values for {len(self.name)} joints (that many or none)")
        self.header.write(w)
        w.u32(len(self.name))
        for n in self.name:
            w.string(n)
        w.f64_sequence(self.position)
        w.f64_sequence(self.velocity)
        w.f64_sequence(self.effort)

    @classmethod
    def read(cls, r: Reader) -> "JointState":
        header = Header.read(r)
        name = [r.string() for _ in range(r.u32())]
        return cls(header, name, r.f64_sequence(), r.f64_sequence(), r.f64_sequence())

    def __eq__(self, other):
        return (isinstance(other, JointState) and self.header == other.header and self.name == other.name
                and all(np.array_equal(getattr(self, f), getattr(other, f)) for f in ("position", "velocity", "effort")))


@dataclass
class Twist:
    """geometry_msgs/msg/Twist: ``linear`` (m/s) and ``angular`` (rad/s) velocity"""

    linear: Vector3 = field(default_factory=Vector3)
    angular: Vector3 = field(default_factory=Vector3)

    def write(self, w: Writer):
        for v in (self.linear.x, self.linear.y, self.linear.z, self.angular.x, self.angular.y, self.angular.z):
            w.f64(v)

    @classmethod
    def read(cls, r: Reader) -> "Twist":
        return cls(Vector3(r.f64(), r.f64(), r.f64()), Vector3(r.f64(), r.f64(), r.f64()))


@dataclass
class TwistWithCovariance:
    """geometry_msgs/msg/TwistWithCovariance: ``covariance`` 6x6 row-major"""

    twist: Twist = field(default_factory=Twist)
    covariance: np.ndarray = field(default_factory=_zeros(36))

    def __eq__(self, other):
        return (isinstance(other, TwistWithCovariance) and self.twist == other.twist
                and np.array_equal(self.covariance, other.covariance))


@dataclass
class Odometry(Message):
    """nav_msgs/msg/Odometry: the pose of ``child_frame_id`` (the robot's root) in ``header.frame_id`` (``odom``) and
    its velocity in ``child_frame_id``."""

    TYPE = "nav_msgs/msg/Odometry"

    header: Header = field(default_factory=Header)
    child_frame_id: str = ""
    pose: PoseWithCovariance = field(default_factory=PoseWithCovariance)
    twist: TwistWithCovariance = field(default_factory=TwistWithCovariance)

    def write(self, w: Writer):
        self.header.write(w)
        w.string(self.child_frame_id)
        self.pose.pose.write(w)
        w.f64_array(self.pose.covariance, 36)
        self.twist.twist.write(w)
        w.f64_array(self.twist.covariance, 36)

    @classmethod
    def read(cls, r: Reader) -> "Odometry":
        header, child = Header.read(r), r.string()
        pose = PoseWithCovariance(Pose.read(r), r.f64_array(36))
        return cls(header, child, pose, TwistWithCovariance(Twist.read(r), r.f64_array(36)))


@dataclass
class Wrench:
    """geometry_msgs/msg/Wrench: ``force`` (N) and ``torque`` (Nm)"""

    force: Vector3 = field(default_factory=Vector3)
    torque: Vector3 = field(default_factory=Vector3)


@dataclass
class WrenchStamped(Message):
    """geometry_msgs/msg/WrenchStamped: a force-torque sensor's measurement in its frame (``header.frame_id``)."""

    TYPE = "geometry_msgs/msg/WrenchStamped"

    header: Header = field(default_factory=Header)
    wrench: Wrench = field(default_factory=Wrench)

    def write(self, w: Writer):
        self.header.write(w)
        f, t = self.wrench.force, self.wrench.torque
        for v in (f.x, f.y, f.z, t.x, t.y, t.z):
            w.f64(v)

    @classmethod
    def read(cls, r: Reader) -> "WrenchStamped":
        header = Header.read(r)
        return cls(header, Wrench(Vector3(r.f64(), r.f64(), r.f64()), Vector3(r.f64(), r.f64(), r.f64())))


@dataclass
class MultiDOFJointState:
    """sensor_msgs/msg/MultiDOFJointState: per joint in ``joint_names`` a transform, and its twist and wrench (each as
    long as ``joint_names`` or empty, as ROS allows). In a controller's command the joints are the controlled frames
    (TCPs) and ``header.frame_id`` the frame of their poses."""

    header: Header = field(default_factory=Header)
    joint_names: list = field(default_factory=list)
    transforms: list = field(default_factory=list)
    twists: list = field(default_factory=list)
    wrenches: list = field(default_factory=list)

    def check(self):
        n = len(self.joint_names)
        if len(self.transforms) != n or len(self.twists) not in (0, n) or len(self.wrenches) not in (0, n):
            raise ValueError(f"MultiDOFJointState: {len(self.transforms)} transforms, {len(self.twists)} twists, "
                             f"{len(self.wrenches)} wrenches for {n} joints (a transform each; twists and wrenches "
                             "that many or none)")


@dataclass
class RobotState(Message):
    """moveit_msgs/msg/RobotState, a controller's command in one message (topics.py): ``joint_state`` (joint targets:
    postures, hands) and ``multi_dof_joint_state`` (the controlled frames' pose targets), both stamped as the command.
    Durchblick carries no attached collision objects: none are written, and a message with some is refused."""

    TYPE = "moveit_msgs/msg/RobotState"

    joint_state: JointState = field(default_factory=JointState)
    multi_dof_joint_state: MultiDOFJointState = field(default_factory=MultiDOFJointState)
    is_diff: bool = False

    def write(self, w: Writer):
        d = self.multi_dof_joint_state
        d.check()
        self.joint_state.write(w)
        d.header.write(w)
        w.u32(len(d.joint_names))
        for name in d.joint_names:
            w.string(name)
        w.u32(len(d.transforms))
        for t in d.transforms:
            for v in (t.translation.x, t.translation.y, t.translation.z,
                      t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w):
                w.f64(v)
        w.u32(len(d.twists))
        for t in d.twists:
            for v in (t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z):
                w.f64(v)
        w.u32(len(d.wrenches))
        for t in d.wrenches:
            for v in (t.force.x, t.force.y, t.force.z, t.torque.x, t.torque.y, t.torque.z):
                w.f64(v)
        w.u32(0)  # attached_collision_objects: none
        w.u8(1 if self.is_diff else 0)

    @classmethod
    def read(cls, r: Reader) -> "RobotState":
        joint_state = JointState.read(r)
        header = Header.read(r)
        names = [r.string() for _ in range(r.u32())]
        transforms = [Transform(Vector3(r.f64(), r.f64(), r.f64()), Quaternion(r.f64(), r.f64(), r.f64(), r.f64()))
                      for _ in range(r.u32())]
        twists = [Twist(Vector3(r.f64(), r.f64(), r.f64()), Vector3(r.f64(), r.f64(), r.f64()))
                  for _ in range(r.u32())]
        wrenches = [Wrench(Vector3(r.f64(), r.f64(), r.f64()), Vector3(r.f64(), r.f64(), r.f64()))
                    for _ in range(r.u32())]
        attached = r.u32()
        if attached:
            raise ValueError(f"RobotState with {attached} attached collision objects: Durchblick carries none")
        is_diff = r.u8() != 0
        d = MultiDOFJointState(header, names, transforms, twists, wrenches)
        d.check()
        return cls(joint_state, d, is_diff)


# ------------------------------------------------------------------------------------------- arrays (std_msgs)


@dataclass
class MultiArrayDimension:
    """std_msgs/msg/MultiArrayDimension"""

    label: str = ""
    size: int = 0
    stride: int = 0


@dataclass
class MultiArrayLayout:
    """std_msgs/msg/MultiArrayLayout"""

    dim: list = field(default_factory=list)
    data_offset: int = 0


@dataclass
class Float64MultiArray(Message):
    """std_msgs/msg/Float64MultiArray: numbers of a fixed layout, e.g. a controller's state records (``layout.dim``
    "record" x "value"; the values' names elsewhere, topics.py)."""

    TYPE = "std_msgs/msg/Float64MultiArray"

    layout: MultiArrayLayout = field(default_factory=MultiArrayLayout)
    data: np.ndarray = field(default_factory=_empty)

    def write(self, w: Writer):
        w.u32(len(self.layout.dim))
        for d in self.layout.dim:
            w.string(d.label)
            w.u32(d.size)
            w.u32(d.stride)
        w.u32(self.layout.data_offset)
        w.f64_sequence(self.data)

    @classmethod
    def read(cls, r: Reader) -> "Float64MultiArray":
        dims = [MultiArrayDimension(r.string(), r.u32(), r.u32()) for _ in range(r.u32())]
        return cls(MultiArrayLayout(dims, r.u32()), r.f64_sequence())

    def __eq__(self, other):
        return (isinstance(other, Float64MultiArray) and self.layout == other.layout
                and np.array_equal(self.data, other.data))


# ------------------------------------------------------------------------------------------- simulation


@dataclass
class Clock(Message):
    """rosgraph_msgs/msg/Clock: a simulation's time (``<world>/clock``, as ROS's ``/clock``); a playback in lockstep
    takes it as the time to play up to."""

    TYPE = "rosgraph_msgs/msg/Clock"

    clock: Time = field(default_factory=Time)

    def write(self, w: Writer):
        self.clock.write(w)

    @classmethod
    def read(cls, r: Reader) -> "Clock":
        return cls(Time.read(r))


MESSAGE_TYPES = {cls.TYPE: cls for cls in (Image, CompressedImage, CameraInfo, Imu, TFMessage, DiagnosticArray,
                                           PointCloud2, ShmImage, CudaImage, ShmPointCloud2, CudaPointCloud2,
                                           Detection2DArray, Detection3DArray, JointState, Odometry, WrenchStamped,
                                           Clock)}


# ------------------------------------------------------------------------------------- parameters (rcl_interfaces)


class ParameterType:
    """rcl_interfaces/msg/ParameterType: the type numbers of ParameterValue and ParameterDescriptor."""

    (NOT_SET, BOOL, INTEGER, DOUBLE, STRING, BYTE_ARRAY, BOOL_ARRAY, INTEGER_ARRAY, DOUBLE_ARRAY,
     STRING_ARRAY) = range(10)
    NAMES = ("not set", "bool", "integer", "double", "string", "byte array", "bool array", "integer array",
             "double array", "string array")


@dataclass
class ParameterValue:
    """rcl_interfaces/msg/ParameterValue: ``type`` says which field holds the value."""

    type: int = 0
    bool_value: bool = False
    integer_value: int = 0
    double_value: float = 0.0
    string_value: str = ""
    byte_array_value: bytes = b""
    bool_array_value: list = field(default_factory=list)
    integer_array_value: list = field(default_factory=list)
    double_array_value: list = field(default_factory=list)
    string_array_value: list = field(default_factory=list)

    _FIELDS = (None, "bool_value", "integer_value", "double_value", "string_value", "byte_array_value",
               "bool_array_value", "integer_array_value", "double_array_value", "string_array_value")

    @classmethod
    def of(cls, value) -> "ParameterValue":
        """The ParameterValue of a Python value: bool, int, float, str, bytes, or a list of one of them (a list's
        type from its items; an empty list is a string array); None is NOT_SET."""
        T = ParameterType
        if value is None:
            return cls()
        if isinstance(value, bool):
            return cls(T.BOOL, bool_value=value)
        if isinstance(value, int):
            return cls(T.INTEGER, integer_value=value)
        if isinstance(value, float):
            return cls(T.DOUBLE, double_value=value)
        if isinstance(value, str):
            return cls(T.STRING, string_value=value)
        if isinstance(value, (bytes, bytearray)):
            return cls(T.BYTE_ARRAY, byte_array_value=bytes(value))
        if isinstance(value, (list, tuple)):
            items = list(value)
            if all(isinstance(v, bool) for v in items) and items:
                return cls(T.BOOL_ARRAY, bool_array_value=items)
            if all(isinstance(v, int) and not isinstance(v, bool) for v in items) and items:
                return cls(T.INTEGER_ARRAY, integer_array_value=items)
            if all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in items) and items:
                return cls(T.DOUBLE_ARRAY, double_array_value=[float(v) for v in items])
            if all(isinstance(v, str) for v in items):
                return cls(T.STRING_ARRAY, string_array_value=items)
        raise TypeError(f"{value!r}: no parameter type (bool, int, float, str, bytes or a list of one of them)")

    def value(self):
        """The Python value (None for NOT_SET)."""
        if not 0 <= self.type < len(self._FIELDS):
            raise ValueError(f"parameter type {self.type} unknown (0 to 9)")
        name = self._FIELDS[self.type]
        return None if name is None else getattr(self, name)

    def write(self, w: Writer):
        w.u8(self.type)
        w.bool(self.bool_value)
        w.i64(self.integer_value)
        w.f64(self.double_value)
        w.string(self.string_value)
        w.u8_sequence(bytes(self.byte_array_value))
        w.bool_sequence(self.bool_array_value)
        w.i64_sequence(self.integer_array_value)
        w.f64_sequence(self.double_array_value)
        w.string_sequence(self.string_array_value)

    @classmethod
    def read(cls, r: Reader) -> "ParameterValue":
        return cls(r.u8(), r.bool(), r.i64(), r.f64(), r.string(), bytes(r.u8_sequence()), r.bool_sequence(),
                   r.i64_sequence(), r.f64_sequence().tolist(), r.string_sequence())


@dataclass
class Parameter:
    """rcl_interfaces/msg/Parameter"""

    name: str = ""
    value: ParameterValue = field(default_factory=ParameterValue)

    def write(self, w: Writer):
        w.string(self.name)
        self.value.write(w)

    @classmethod
    def read(cls, r: Reader) -> "Parameter":
        return cls(r.string(), ParameterValue.read(r))


@dataclass
class FloatingPointRange:
    """rcl_interfaces/msg/FloatingPointRange: ``step`` 0 for any value between."""

    from_value: float = 0.0
    to_value: float = 0.0
    step: float = 0.0


@dataclass
class IntegerRange:
    """rcl_interfaces/msg/IntegerRange: ``step`` 0 for any value between."""

    from_value: int = 0
    to_value: int = 0
    step: int = 0


@dataclass
class ParameterDescriptor:
    """rcl_interfaces/msg/ParameterDescriptor. ``floating_point_range``, ``integer_range``: lists of at most one."""

    name: str = ""
    type: int = 0
    description: str = ""
    additional_constraints: str = ""
    read_only: bool = False
    dynamic_typing: bool = False
    floating_point_range: list = field(default_factory=list)
    integer_range: list = field(default_factory=list)

    def write(self, w: Writer):
        w.string(self.name)
        w.u8(self.type)
        w.string(self.description)
        w.string(self.additional_constraints)
        w.bool(self.read_only)
        w.bool(self.dynamic_typing)
        w.u32(len(self.floating_point_range))
        for rng in self.floating_point_range:
            w.f64(rng.from_value)
            w.f64(rng.to_value)
            w.f64(rng.step)
        w.u32(len(self.integer_range))
        for rng in self.integer_range:
            w.i64(rng.from_value)
            w.i64(rng.to_value)
            w.u64(rng.step)

    @classmethod
    def read(cls, r: Reader) -> "ParameterDescriptor":
        name, type_, description, constraints, read_only, dynamic = (r.string(), r.u8(), r.string(), r.string(),
                                                                     r.bool(), r.bool())
        floating = [FloatingPointRange(r.f64(), r.f64(), r.f64()) for _ in range(r.u32())]
        integer = [IntegerRange(r.i64(), r.i64(), r.u64()) for _ in range(r.u32())]
        return cls(name, type_, description, constraints, read_only, dynamic, floating, integer)


@dataclass
class SetParametersResult:
    """rcl_interfaces/msg/SetParametersResult"""

    successful: bool = False
    reason: str = ""

    def write(self, w: Writer):
        w.bool(self.successful)
        w.string(self.reason)

    @classmethod
    def read(cls, r: Reader) -> "SetParametersResult":
        return cls(r.bool(), r.string())


def _write_list(w: Writer, items):
    w.u32(len(items))
    for item in items:
        item.write(w)


def _read_list(r: Reader, kind) -> list:
    return [kind.read(r) for _ in range(r.u32())]


@dataclass
class ParameterEvent(Message):
    """rcl_interfaces/msg/ParameterEvent: what changed in a node's parameters (``node``: its fully qualified name)."""

    TYPE = "rcl_interfaces/msg/ParameterEvent"

    stamp: Time = field(default_factory=Time)
    node: str = ""
    new_parameters: list = field(default_factory=list)
    changed_parameters: list = field(default_factory=list)
    deleted_parameters: list = field(default_factory=list)

    def write(self, w: Writer):
        self.stamp.write(w)
        w.string(self.node)
        for items in (self.new_parameters, self.changed_parameters, self.deleted_parameters):
            _write_list(w, items)

    @classmethod
    def read(cls, r: Reader) -> "ParameterEvent":
        return cls(Time.read(r), r.string(), _read_list(r, Parameter), _read_list(r, Parameter),
                   _read_list(r, Parameter))


# the services' requests and responses (rcl_interfaces/srv/*: <Service>_Request, <Service>_Response)


@dataclass
class ListParametersRequest(Message):
    """rcl_interfaces/srv/ListParameters_Request. ``depth`` 0: every level below the prefixes."""

    TYPE = "rcl_interfaces/srv/ListParameters_Request"
    DEPTH_RECURSIVE = 0

    prefixes: list = field(default_factory=list)
    depth: int = 0

    def write(self, w: Writer):
        w.string_sequence(self.prefixes)
        w.u64(self.depth)

    @classmethod
    def read(cls, r: Reader) -> "ListParametersRequest":
        return cls(r.string_sequence(), r.u64())


@dataclass
class ListParametersResponse(Message):
    """rcl_interfaces/srv/ListParameters_Response: its ListParametersResult, ``names`` and ``prefixes``."""

    TYPE = "rcl_interfaces/srv/ListParameters_Response"

    names: list = field(default_factory=list)
    prefixes: list = field(default_factory=list)

    def write(self, w: Writer):
        w.string_sequence(self.names)
        w.string_sequence(self.prefixes)

    @classmethod
    def read(cls, r: Reader) -> "ListParametersResponse":
        return cls(r.string_sequence(), r.string_sequence())


@dataclass
class NamesRequest(Message):
    """A request of parameter names (DescribeParameters, GetParameters, GetParameterTypes); subclassed per service."""

    names: list = field(default_factory=list)

    def write(self, w: Writer):
        w.string_sequence(self.names)

    @classmethod
    def read(cls, r: Reader):
        return cls(r.string_sequence())


@dataclass
class DescribeParametersRequest(NamesRequest):
    TYPE = "rcl_interfaces/srv/DescribeParameters_Request"


@dataclass
class GetParametersRequest(NamesRequest):
    TYPE = "rcl_interfaces/srv/GetParameters_Request"


@dataclass
class GetParameterTypesRequest(NamesRequest):
    TYPE = "rcl_interfaces/srv/GetParameterTypes_Request"


@dataclass
class DescribeParametersResponse(Message):
    TYPE = "rcl_interfaces/srv/DescribeParameters_Response"

    descriptors: list = field(default_factory=list)

    def write(self, w: Writer):
        _write_list(w, self.descriptors)

    @classmethod
    def read(cls, r: Reader) -> "DescribeParametersResponse":
        return cls(_read_list(r, ParameterDescriptor))


@dataclass
class GetParametersResponse(Message):
    TYPE = "rcl_interfaces/srv/GetParameters_Response"

    values: list = field(default_factory=list)

    def write(self, w: Writer):
        _write_list(w, self.values)

    @classmethod
    def read(cls, r: Reader) -> "GetParametersResponse":
        return cls(_read_list(r, ParameterValue))


@dataclass
class GetParameterTypesResponse(Message):
    TYPE = "rcl_interfaces/srv/GetParameterTypes_Response"

    types: list = field(default_factory=list)

    def write(self, w: Writer):
        w.u8_sequence(bytes(self.types))

    @classmethod
    def read(cls, r: Reader) -> "GetParameterTypesResponse":
        return cls(list(bytes(r.u8_sequence())))


@dataclass
class ParametersRequest(Message):
    """A request of parameters to set (SetParameters, SetParametersAtomically); subclassed per service."""

    parameters: list = field(default_factory=list)

    def write(self, w: Writer):
        _write_list(w, self.parameters)

    @classmethod
    def read(cls, r: Reader):
        return cls(_read_list(r, Parameter))


@dataclass
class SetParametersRequest(ParametersRequest):
    TYPE = "rcl_interfaces/srv/SetParameters_Request"


@dataclass
class SetParametersAtomicallyRequest(ParametersRequest):
    TYPE = "rcl_interfaces/srv/SetParametersAtomically_Request"


@dataclass
class SetParametersResponse(Message):
    TYPE = "rcl_interfaces/srv/SetParameters_Response"

    results: list = field(default_factory=list)

    def write(self, w: Writer):
        _write_list(w, self.results)

    @classmethod
    def read(cls, r: Reader) -> "SetParametersResponse":
        return cls(_read_list(r, SetParametersResult))


@dataclass
class SetParametersAtomicallyResponse(Message):
    TYPE = "rcl_interfaces/srv/SetParametersAtomically_Response"

    result: SetParametersResult = field(default_factory=SetParametersResult)

    def write(self, w: Writer):
        self.result.write(w)

    @classmethod
    def read(cls, r: Reader) -> "SetParametersAtomicallyResponse":
        return cls(SetParametersResult.read(r))


# ------------------------------------------------------------------------------- a controller's torque step


@dataclass
class TorqueStepRequest(Message):
    """xblick_msgs/srv/TorqueStep_Request (protocol.hpp, ros2_cdr_msgs.hpp): ``torque`` [Nm] on ``joint`` for
    ``duration`` [s], a controller's bring-up torque step."""

    TYPE = "xblick_msgs/srv/TorqueStep_Request"

    joint: str = ""
    torque: float = 0.0
    duration: float = 0.0

    def write(self, w: Writer):
        w.string(self.joint)
        w.f64(self.torque)
        w.f64(self.duration)

    @classmethod
    def read(cls, r: Reader) -> "TorqueStepRequest":
        return cls(r.string(), r.f64(), r.f64())


@dataclass
class TorqueStepResponse(Message):
    """xblick_msgs/srv/TorqueStep_Response: taken (``ok``), else why not (``message``)."""

    TYPE = "xblick_msgs/srv/TorqueStep_Response"

    ok: bool = False
    message: str = ""

    def write(self, w: Writer):
        w.bool(self.ok)
        w.string(self.message)

    @classmethod
    def read(cls, r: Reader) -> "TorqueStepResponse":
        return cls(r.bool(), r.string())


# ------------------------------------------------------------------------------- the controller manager
# A robot's controller manager (protocol.hpp: <robot>/controller_manager/<service>): ROS 2's controller_manager_msgs
# (ros2_control Jazzy) and Durchblick's xblick_msgs services for creating a controller from a JSON config, reading a
# controller's config back and reading a config file of the robot.


@dataclass
class Duration:
    """builtin_interfaces/msg/Duration"""

    sec: int = 0
    nanosec: int = 0

    @classmethod
    def from_seconds(cls, seconds: float) -> "Duration":
        ns = round(seconds * 1e9)
        return cls(ns // 1_000_000_000, ns % 1_000_000_000)

    def write(self, w: Writer):
        w.i32(self.sec)
        w.u32(self.nanosec)

    @classmethod
    def read(cls, r: Reader) -> "Duration":
        return cls(r.i32(), r.u32())


@dataclass
class ChainConnection:
    """controller_manager_msgs/msg/ChainConnection"""

    name: str = ""
    reference_interfaces: list = field(default_factory=list)

    def write(self, w: Writer):
        w.string(self.name)
        w.string_sequence(self.reference_interfaces)

    @classmethod
    def read(cls, r: Reader) -> "ChainConnection":
        return cls(r.string(), r.string_sequence())


@dataclass
class ControllerState:
    """controller_manager_msgs/msg/ControllerState: ``state`` unconfigured, inactive, active or finalized; ``type`` the
    controller's class name on the robot."""

    name: str = ""
    state: str = ""
    type: str = ""
    is_async: bool = False
    update_rate: int = 0
    claimed_interfaces: list = field(default_factory=list)
    required_command_interfaces: list = field(default_factory=list)
    required_state_interfaces: list = field(default_factory=list)
    is_chainable: bool = False
    is_chained: bool = False
    exported_state_interfaces: list = field(default_factory=list)
    reference_interfaces: list = field(default_factory=list)
    chain_connections: list = field(default_factory=list)

    def write(self, w: Writer):
        w.string(self.name)
        w.string(self.state)
        w.string(self.type)
        w.bool(self.is_async)
        w.u16(self.update_rate)
        w.string_sequence(self.claimed_interfaces)
        w.string_sequence(self.required_command_interfaces)
        w.string_sequence(self.required_state_interfaces)
        w.bool(self.is_chainable)
        w.bool(self.is_chained)
        w.string_sequence(self.exported_state_interfaces)
        w.string_sequence(self.reference_interfaces)
        _write_list(w, self.chain_connections)

    @classmethod
    def read(cls, r: Reader) -> "ControllerState":
        return cls(r.string(), r.string(), r.string(), r.bool(), r.u16(), r.string_sequence(), r.string_sequence(),
                   r.string_sequence(), r.bool(), r.bool(), r.string_sequence(), r.string_sequence(),
                   _read_list(r, ChainConnection))


@dataclass
class ListControllersRequest(Message):
    """controller_manager_msgs/srv/ListControllers_Request: empty (ROS's one placeholder byte)."""

    TYPE = "controller_manager_msgs/srv/ListControllers_Request"

    def write(self, w: Writer):
        w.u8(0)  # rosidl's structure_needs_at_least_one_member

    @classmethod
    def read(cls, r: Reader) -> "ListControllersRequest":
        return cls()


@dataclass
class ListControllersResponse(Message):
    TYPE = "controller_manager_msgs/srv/ListControllers_Response"

    controller: list = field(default_factory=list)

    def write(self, w: Writer):
        _write_list(w, self.controller)

    @classmethod
    def read(cls, r: Reader) -> "ListControllersResponse":
        return cls(_read_list(r, ControllerState))


@dataclass
class SwitchControllerRequest(Message):
    """controller_manager_msgs/srv/SwitchController_Request. ``timeout`` zero: the manager's longest wait."""

    TYPE = "controller_manager_msgs/srv/SwitchController_Request"
    BEST_EFFORT = 1
    STRICT = 2
    AUTO = 3
    FORCE_AUTO = 4

    activate_controllers: list = field(default_factory=list)
    deactivate_controllers: list = field(default_factory=list)
    strictness: int = 2
    activate_asap: bool = False
    timeout: Duration = field(default_factory=Duration)

    def write(self, w: Writer):
        w.string_sequence(self.activate_controllers)
        w.string_sequence(self.deactivate_controllers)
        w.i32(self.strictness)
        w.bool(self.activate_asap)
        self.timeout.write(w)

    @classmethod
    def read(cls, r: Reader) -> "SwitchControllerRequest":
        return cls(r.string_sequence(), r.string_sequence(), r.i32(), r.bool(), Duration.read(r))


@dataclass
class SwitchControllerResponse(Message):
    TYPE = "controller_manager_msgs/srv/SwitchController_Response"

    ok: bool = False
    message: str = ""

    def write(self, w: Writer):
        w.bool(self.ok)
        w.string(self.message)

    @classmethod
    def read(cls, r: Reader) -> "SwitchControllerResponse":
        return cls(r.bool(), r.string())


@dataclass
class UnloadControllerRequest(Message):
    TYPE = "controller_manager_msgs/srv/UnloadController_Request"

    name: str = ""

    def write(self, w: Writer):
        w.string(self.name)

    @classmethod
    def read(cls, r: Reader) -> "UnloadControllerRequest":
        return cls(r.string())


@dataclass
class UnloadControllerResponse(Message):
    TYPE = "controller_manager_msgs/srv/UnloadController_Response"

    ok: bool = False

    def write(self, w: Writer):
        w.bool(self.ok)

    @classmethod
    def read(cls, r: Reader) -> "UnloadControllerResponse":
        return cls(r.bool())


@dataclass
class CreateControllerRequest(Message):
    """xblick_msgs/srv/CreateController_Request: a controller ``name`` of the class ``type`` from ``config``
    (JSON), created inactive."""

    TYPE = "xblick_msgs/srv/CreateController_Request"

    name: str = ""
    type: str = ""
    config: str = ""

    def write(self, w: Writer):
        w.string(self.name)
        w.string(self.type)
        w.string(self.config)

    @classmethod
    def read(cls, r: Reader) -> "CreateControllerRequest":
        return cls(r.string(), r.string(), r.string())


@dataclass
class CreateControllerResponse(Message):
    TYPE = "xblick_msgs/srv/CreateController_Response"

    ok: bool = False
    message: str = ""

    def write(self, w: Writer):
        w.bool(self.ok)
        w.string(self.message)

    @classmethod
    def read(cls, r: Reader) -> "CreateControllerResponse":
        return cls(r.bool(), r.string())


@dataclass
class GetControllerConfigRequest(Message):
    TYPE = "xblick_msgs/srv/GetControllerConfig_Request"

    name: str = ""

    def write(self, w: Writer):
        w.string(self.name)

    @classmethod
    def read(cls, r: Reader) -> "GetControllerConfigRequest":
        return cls(r.string())


@dataclass
class ConfigResponse(Message):
    """The response of GetControllerConfig and GetConfigFile: ``ok``, ``message`` (why not), ``config`` (JSON)."""

    TYPE = "xblick_msgs/srv/GetControllerConfig_Response"

    ok: bool = False
    message: str = ""
    config: str = ""

    def write(self, w: Writer):
        w.bool(self.ok)
        w.string(self.message)
        w.string(self.config)

    @classmethod
    def read(cls, r: Reader):
        return cls(r.bool(), r.string(), r.string())


@dataclass
class GetControllerConfigResponse(ConfigResponse):
    TYPE = "xblick_msgs/srv/GetControllerConfig_Response"


@dataclass
class GetConfigFileRequest(Message):
    """xblick_msgs/srv/GetConfigFile_Request: the robot's config file ``<type>/<name>``."""

    TYPE = "xblick_msgs/srv/GetConfigFile_Request"

    type: str = ""
    name: str = ""

    def write(self, w: Writer):
        w.string(self.type)
        w.string(self.name)

    @classmethod
    def read(cls, r: Reader) -> "GetConfigFileRequest":
        return cls(r.string(), r.string())


@dataclass
class GetConfigFileResponse(ConfigResponse):
    TYPE = "xblick_msgs/srv/GetConfigFile_Response"

