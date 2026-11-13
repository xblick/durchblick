"""Publishing and receiving one point cloud stream (topics.py: cloud, cloud/shm, cloud/cuda), in the manner of
camera.py's image streams, and as C++'s points.hpp.

A cloud is written once into a ring; readers on the same host get a descriptor (msg.ShmPointCloud2: host memory,
msg.CudaPointCloud2: device memory). ``cloud`` sends the whole sensor_msgs/PointCloud2 over the network: it works,
but costs the full size per cloud (a 640 x 576 cloud of 16-byte points: 5.9 MB, 177 MB/s at 30 Hz).

``cloud/compressed`` is the network form of a cloud whose points lie on fixed rays (msg.CompressedPointCloud2): a
camera's cloud, where a pixel always looks the same way, or a laser scanner's. A point is its ray's direction times
its range, so a cloud is sent as an image of ranges (16 bit, RVL) and an image of colours (JPEG); the directions are
sent once, on request (``cloud/rays``). The publisher gets the directions as ``rays`` and knows nothing else about
where the cloud comes from: a camera's node passes (x/z, y/z, 1) of each pixel (the range is then z), a scanner's
the beams' unit vectors. The ranges are rounded to ``range_step``, and JPEG changes the colours.

The layout of a point is its PointFields and ``point_step`` (bytes per point). Host clouds are NumPy structured
arrays of shape (height, width) with that layout (``points_dtype``); device clouds are gpu.CudaImage of shape
(height, width, point_step) bytes.
"""

from __future__ import annotations

import hashlib
import json
import time
from dataclasses import dataclass, field

import numpy as np
import zenoh

from . import _core, topics
from .camera import AUTO, INFO_VERSION, FrameAttachment, ProtocolError, Subscription, cdr_encoding, ring_gpu
from .codecs import JPEG, RVL, compress, decompress
from .gpu import CudaArray, CudaImage, cuda_array, is_cuda_array, write_ring
from .msg import (POINT_FIELD_DTYPES, CompressedImage, CompressedPointCloud2, CudaPointCloud2, Header, PointCloud2,
                  PointField, Rays, ShmPointCloud2, Time)
from .session import replies

XYZ = ([PointField("x", 0, PointField.FLOAT32), PointField("y", 4, PointField.FLOAT32),
        PointField("z", 8, PointField.FLOAT32)], 16)
"""x, y, z as float32 in metres, 16 bytes per point (PCL's PointXYZ, padded for aligned loads on a GPU)."""

XYZRGB = (XYZ[0] + [PointField("rgb", 12, PointField.FLOAT32)], 16)
"""x, y, z and PCL's packed colour ``rgb``: a float32 whose bits are the uint32 0x00RRGGBB (PCL's PointXYZRGB)."""

# a point cloud ring keeps 4 clouds (a reader has 3 cloud periods): clouds are large and nothing pairs them by stamp
# (image rings keep camera.DEFAULT_SLOTS)
DEFAULT_SLOTS = 4


def points_dtype(fields: list, point_step: int) -> np.dtype:
    """The NumPy structured dtype of a point (little endian). Raises ValueError for a layout that is not valid."""
    if not fields or point_step <= 0:
        raise ValueError("a point layout needs fields and a point_step")
    names, formats, offsets = [], [], []
    for f in fields:
        if f.datatype not in POINT_FIELD_DTYPES:
            raise ValueError(f"field {f.name}: unknown PointField datatype {f.datatype}")
        base = POINT_FIELD_DTYPES[f.datatype]
        if f.count < 1 or f.offset + base.itemsize * f.count > point_step:
            raise ValueError(f"field {f.name} does not fit a point of {point_step} bytes")
        if f.name in names:
            raise ValueError(f"field {f.name} is there twice")
        names.append(f.name)
        formats.append(base if f.count == 1 else (base, (f.count,)))
        offsets.append(f.offset)
    return np.dtype({"names": names, "formats": formats, "offsets": offsets, "itemsize": point_step})


def pack_rgb(rgb: np.ndarray) -> np.ndarray:
    """uint8 colours (..., 3) as PCL's packed ``rgb`` field (float32 holding 0x00RRGGBB)."""
    rgb = np.asarray(rgb, np.uint32)
    return ((rgb[..., 0] << 16) | (rgb[..., 1] << 8) | rgb[..., 2]).view(np.float32)


def unpack_rgb(packed: np.ndarray) -> np.ndarray:
    """PCL's packed ``rgb`` field as uint8 colours (..., 3)."""
    bits = np.ascontiguousarray(packed, np.float32).view(np.uint32)
    return np.stack([(bits >> 16) & 0xFF, (bits >> 8) & 0xFF, bits & 0xFF], axis=-1).astype(np.uint8)


def rays_id(rays: np.ndarray) -> str:
    """The name of a table of rays: a hash of its size and bytes."""
    digest = hashlib.sha256(np.asarray(rays.shape[:2], "<u4").tobytes())
    digest.update(np.ascontiguousarray(rays, "<f4").tobytes())
    return digest.hexdigest()[:16]


def check_rays_layout(fields: list):
    """Raise ValueError unless a point is x, y, z (float32) and at most the packed colour ``rgb``: what
    ``cloud/compressed`` holds."""
    by_name = {f.name: f for f in fields}
    if set(by_name) - {"x", "y", "z", "rgb"} or not {"x", "y", "z"} <= set(by_name):
        raise ValueError(f"cloud/compressed holds the fields x, y, z and rgb, not {', '.join(by_name)}")
    for name, f in by_name.items():
        allowed = (PointField.FLOAT32, PointField.UINT32) if name == "rgb" else (PointField.FLOAT32,)
        if f.datatype not in allowed or f.count != 1:
            raise ValueError(f"cloud/compressed: the field {name} is not a float32"
                             + (" or uint32" if name == "rgb" else ""))


def compress_cloud(points: np.ndarray, rays: np.ndarray, range_step: float, color_codec: str = JPEG,
                   jpeg_quality: int = 95) -> tuple[CompressedImage, CompressedImage | None]:
    """A cloud (structured array, height x width) on ``rays`` (height x width x 3) as its ranges and its colours
    (None without the field ``rgb``). Raises ValueError for a cloud that does not lie on the rays, or whose ranges
    do not fit 16 bits of ``range_step``."""
    if points.shape != rays.shape[:2]:
        raise ValueError(f"a cloud of {points.shape[1]} x {points.shape[0]} points, rays for {rays.shape[1]} x "
                         f"{rays.shape[0]}")
    offsets = [points.dtype.fields[name][1] for name in "xyz"]
    steps = np.empty(points.shape, np.uint16)
    valid, low, high, deviation, limit = _core.ray_cloud_ranges(
        np.ascontiguousarray(points).reshape(-1).view(np.uint8), points.dtype.itemsize, *offsets,
        np.ascontiguousarray(rays, np.float32), range_step, steps)
    if valid:
        if not (1 <= low and high <= 65535):
            raise ValueError(f"cloud/compressed: ranges of {low * range_step:.4f} to {high * range_step:.3f} m (along "
                             f"the rays) do not fit 1 to 65535 steps of {range_step} m: choose another range_step")
        if deviation > limit:  # a point is at most half a step along its ray from its rounded place
            raise ValueError(f"cloud/compressed: the cloud does not lie on its rays (a point is {deviation:.4f} m "
                             f"off, more than {limit:.4f} m): the rays are those of another camera or frame")
    ranges = compress(steps, "16UC1", RVL)
    colors = None
    if "rgb" in points.dtype.names:
        colors = compress(unpack_rgb(points["rgb"].view(np.float32)), "rgb8", color_codec, jpeg_quality)
    return ranges, colors


def decompress_cloud(message: CompressedPointCloud2, rays: np.ndarray) -> np.ndarray:
    """The cloud of a CompressedPointCloud2 on its ``rays`` (height x width x 3): a structured array."""
    shape = (message.height, message.width)
    check_rays_layout(message.fields)
    steps, _ = decompress(CompressedImage(format=message.range_format, data=message.range))
    if steps.shape != shape or steps.dtype != np.uint16 or rays.shape != shape + (3,):
        raise ProtocolError(f"a compressed cloud of {shape[1]} x {shape[0]} points with ranges of shape {steps.shape} "
                            f"and rays of shape {rays.shape}")
    points = np.zeros(shape, points_dtype(message.fields, message.point_step))
    offsets = [points.dtype.fields[name][1] for name in "xyz"]
    _core.ray_cloud_points(np.ascontiguousarray(steps), np.ascontiguousarray(rays, np.float32), message.range_step,
                           points.reshape(-1).view(np.uint8), message.point_step, *offsets)
    if "rgb" in points.dtype.names:
        colors, encoding = decompress(CompressedImage(format=message.color_format, data=message.color))
        if colors.shape != shape + (3,) or encoding != "rgb8":
            raise ProtocolError(f"a compressed cloud of {shape[1]} x {shape[0]} points with colours of shape "
                                f"{colors.shape} ({encoding})")
        points["rgb"] = pack_rgb(colors).view(points.dtype["rgb"])
    return points


class PointCloudPublisher:
    """One point cloud stream. ``publish()`` is not thread-safe (call it from one thread).

    ``cloud/shm`` is always offered, ``cloud/cuda`` only with ``cuda_device``. The rings are created at the first cloud
    a reader asks for, sized for it, and made anew for a larger one (readers follow the new ring by its name).

    ``rays``: the direction of each point's ray (height x width x 3, float32), for a cloud whose points lie on fixed
    rays; the stream then offers ``cloud/compressed`` (the module's text), with ranges in steps of ``range_step``
    metres (65.5 m at the default) and colours as ``color_codec`` (``jpeg`` of ``jpeg_quality``, or ``png``, exact
    and slower). ``set_rays()`` changes them (a camera back with another size). Every cloud is checked against its
    rays: one that does not lie on them raises ValueError."""

    def __init__(self, session: zenoh.Session, base: str, *, fields: list, point_step: int, frame_id: str,
                 source: str = "", slots: int = DEFAULT_SLOTS, cuda_device: int | None = None, rays=None,
                 range_step: float = 0.001, color_codec: str = JPEG, jpeg_quality: int = 95):
        self.base = topics.check_name(base)
        self.fields = list(fields)
        self.point_step = point_step
        self.dtype = points_dtype(self.fields, point_step)
        self.frame_id = frame_id
        self.source = source
        self.slots = slots
        self.cuda_device = cuda_device
        self.seq = 0
        self._host = _core.host_id()
        self._gpu = ring_gpu(self.base, cuda_device)
        self._host_ring: _core.HostRingWriter | None = None
        self._cuda_ring: _core.CudaRingWriter | None = None
        self.range_step = float(range_step)
        self.color_codec = color_codec
        self.jpeg_quality = jpeg_quality
        self.rays: np.ndarray | None = None
        self._rays_message = b""
        self._rays_id = ""
        self._compressed = self._rays_queryable = None
        self._with_rays = rays is not None
        if rays is not None:
            if not self.range_step > 0:
                raise ValueError(f"range_step {range_step}: a positive number of metres")
            check_rays_layout(self.fields)
            self.set_rays(rays)
            self._rays_queryable = session.declare_queryable(topics.cloud_rays(self.base), self._answer_rays)
        self._info_queryable = session.declare_queryable(topics.info(self.base), self._answer_info)
        self._alive = session.liveliness().declare_token(  # as CameraPublisher's
            topics.alive(self.base, self._host, topics.new_publisher_id()))
        drop = zenoh.CongestionControl.DROP
        self._raw = session.declare_publisher(topics.cloud(self.base), encoding=cdr_encoding(PointCloud2),
                                              congestion_control=drop)
        self._shm = session.declare_publisher(topics.cloud_shm(self.base, self._host),
                                              encoding=cdr_encoding(ShmPointCloud2), congestion_control=drop)
        self._cuda = (session.declare_publisher(topics.cloud_cuda(self.base, self._host, self._gpu),
                                                encoding=cdr_encoding(CudaPointCloud2), congestion_control=drop)
                      if cuda_device is not None else None)
        if rays is not None:
            self._compressed = session.declare_publisher(topics.cloud_compressed(self.base),
                                                         encoding=cdr_encoding(CompressedPointCloud2),
                                                         congestion_control=drop)

    def set_rays(self, rays):
        """Other rays for the clouds from now on (only for a publisher made with rays)."""
        rays = np.ascontiguousarray(rays, np.float32)
        if rays.ndim != 3 or rays.shape[2] != 3 or not np.isfinite(rays).all() or not (rays * rays).sum(-1).min() > 0:
            raise ValueError(f"rays of shape {rays.shape}: height x width x 3 finite directions, none of them 0")
        if not self._with_rays:
            raise ValueError(f"{self.base}: a publisher made without rays offers no cloud/compressed")
        self._rays_id = rays_id(rays)
        self._rays_message = Rays(id=self._rays_id, height=rays.shape[0], width=rays.shape[1],
                                  directions=rays.tobytes()).serialize()
        self.rays = rays

    def _answer_rays(self, query: zenoh.Query):
        query.reply(topics.cloud_rays(self.base), self._rays_message, encoding=cdr_encoding(Rays))

    def info_record(self) -> dict:
        """What ``<base>/info`` answers (protocol.hpp describes the fields)."""
        transports = (["raw", "shm"] + (["cuda"] if self._cuda is not None else [])
                      + (["compressed"] if self._compressed is not None else []))
        return {"v": INFO_VERSION, "kind": "points", "durchblick": _core.__version__, "host": self._host,
                "transports": transports, "fields": [f.name for f in self.fields], "frame_id": self.frame_id,
                "source": self.source,
                "cuda_device_uuid": self._gpu}

    def _answer_info(self, query: zenoh.Query):
        query.reply(topics.info(self.base), json.dumps(self.info_record(), separators=(",", ":")))

    def _publishers(self):
        return [p for p in (self._raw, self._shm, self._cuda, self._compressed) if p is not None]

    @property
    def subscribed(self) -> bool:
        """True if anybody takes clouds of this stream (any representation): compute them only then."""
        return any(p.matching_status.matching for p in self._publishers())

    def publish(self, points, stamp_ns: int | None = None, is_dense: bool = False, stream=None) -> list[str]:
        """Publish one cloud captured at ``stamp_ns`` (nanoseconds since the epoch; default: now). ``points``: a
        C-contiguous NumPy array of shape (height, width, ...) holding height * width points of this publisher's
        layout (e.g. a structured array of ``dtype``), or a device array (``__cuda_array_interface__``; needs
        ``cuda_device``; ``stream`` as CameraPublisher.publish's). ``is_dense``: there are no invalid (NaN) points.
        Returns the topics it went to."""
        gpu = is_cuda_array(points)
        array = cuda_array(points, stream) if gpu else np.ascontiguousarray(points)
        if gpu and self.cuda_device is None:
            raise ValueError("a device array needs a publisher with cuda_device")
        if len(array.shape) < 2:
            raise ValueError(f"points of shape {array.shape}: (height, width, ...) expected")
        height, width = int(array.shape[0]), int(array.shape[1])
        nbytes = height * width * self.point_step
        if array.nbytes != nbytes:
            raise ValueError(f"{array.nbytes} bytes for {height} x {width} points of {self.point_step} bytes "
                             f"({nbytes} expected)")
        header = Header(stamp=Time.from_ns(time.time_ns() if stamp_ns is None else stamp_ns), frame_id=self.frame_id)
        self.seq += 1
        attachment = FrameAttachment(self.seq, self.source)  # each put its own publish_ns
        want = {p: p.matching_status.matching for p in self._publishers()}
        fields = dict(header=header, height=height, width=width, fields=self.fields, is_bigendian=False,
                      point_step=self.point_step, row_step=width * self.point_step, is_dense=is_dense,
                      host=self._host)
        sent = []
        cuda_slot = None
        if gpu or (self._cuda is not None and want[self._cuda]):
            if self._cuda_ring is None or nbytes > self._cuda_ring.slot_size:  # first, or a larger cloud
                # an empty cloud (no points: a node with nothing to report) still gets a ring, of slots of one byte
                self._cuda_ring = _core.CudaRingWriter(self.cuda_device, self.slots, max(nbytes, 1))
            if isinstance(array, CudaArray):
                cuda_slot = write_ring(self._cuda_ring, array)
            else:  # as bytes: the rings take plain arrays, not structured ones
                cuda_slot = self._cuda_ring.write_host(array.reshape(-1).view(np.uint8))
            if self._cuda is not None and want[self._cuda]:
                message = CudaPointCloud2(**fields, endpoint=self._cuda_ring.endpoint,
                                          device_uuid=self._cuda_ring.device_uuid, slot=cuda_slot[0],
                                          seq=cuda_slot[1])
                self._cuda.put(message.serialize(), attachment=attachment.now())
                sent.append(str(self._cuda.key_expr))
        compressed = self._compressed is not None and want[self._compressed]
        if want[self._raw] or want[self._shm] or compressed:
            host = array
            if gpu:  # one download for both host representations
                host = np.empty(nbytes, np.uint8)
                self._cuda_ring.read_host(cuda_slot[0], host)
            host = host.reshape(-1).view(np.uint8)
            if want[self._shm]:
                if self._host_ring is None or nbytes > self._host_ring.slot_size:  # first, or a larger cloud
                    self._host_ring = _core.HostRingWriter(self.slots, max(nbytes, 1))  # as the CUDA ring's
                slot, seq = self._host_ring.write(host)
                message = ShmPointCloud2(**fields, segment=self._host_ring.name, slot=slot, seq=seq)
                self._shm.put(message.serialize(), attachment=attachment.now())
                sent.append(str(self._shm.key_expr))
            if want[self._raw]:
                message = PointCloud2(header=header, height=height, width=width, fields=self.fields,
                                      point_step=self.point_step, row_step=width * self.point_step, data=host,
                                      is_dense=is_dense)
                self._raw.put(message.serialize(), attachment=attachment.now())
                sent.append(str(self._raw.key_expr))
            if compressed:
                ranges, colors = compress_cloud(host.view(self.dtype).reshape(height, width), self.rays,
                                                self.range_step, self.color_codec, self.jpeg_quality)
                message = CompressedPointCloud2(
                    header=header, height=height, width=width, fields=self.fields, point_step=self.point_step,
                    is_dense=is_dense, rays=self._rays_id, range_step=self.range_step, range_format=ranges.format,
                    range=ranges.data, color_format=colors.format if colors else "",
                    color=colors.data if colors else b"")
                self._compressed.put(message.serialize(), attachment=attachment.now())
                sent.append(str(self._compressed.key_expr))
        return sent

    def close(self):
        for publisher in self._publishers():
            publisher.undeclare()
        self._alive.undeclare()
        self._info_queryable.undeclare()
        if self._rays_queryable is not None:
            self._rays_queryable.undeclare()
        self._host_ring = None  # unlinks the shared memory; readers keep their mapping until they let go
        self._cuda_ring = None


@dataclass
class PointCloudFrame:
    """A received cloud. ``points``: a NumPy structured array (height, width) of ``dtype`` (a view or a copy), or for
    the cuda transport a gpu.CudaImage of (height, width, point_step) bytes."""

    points: object
    fields: list
    point_step: int
    is_dense: bool
    header: Header
    key: str
    receive_ns: int
    seq: int | None = None
    source: str = ""
    publish_ns: int | None = None
    meta: dict = field(default_factory=dict)
    valid: object = None  # zero-copy clouds: callable telling whether the ring still holds this cloud

    @property
    def dtype(self) -> np.dtype:
        return points_dtype(self.fields, self.point_step)

    @property
    def stamp_ns(self) -> int:
        return self.header.stamp.to_ns()

    @property
    def age_at_send_ns(self) -> int | None:
        """Nanoseconds from capture to the put of this cloud's sample (its ring write, for shm and cuda: stamps.py);
        None without publish_ns."""
        if not self.stamp_ns or self.publish_ns is None:
            return None
        return self.publish_ns - self.stamp_ns

    def is_valid(self) -> bool:
        """False once the publisher has begun overwriting a zero-copy cloud. Always True for copies."""
        return True if self.valid is None else bool(self.valid())

    def numpy(self) -> np.ndarray:
        """The points as a structured host array (a copied cuda cloud is downloaded)."""
        if isinstance(self.points, CudaImage):
            return self.points.numpy().reshape(-1).view(self.dtype).reshape(self.points.shape[:2])
        return self.points


class PointCloudSubscriber(Subscription):
    """The newest cloud of one point cloud stream. ``transport``: ``"auto"`` (``local``, i.e. ``"shm"`` or ``"cuda"``,
    on the publisher's host, else ``remote``), or ``"raw"`` (sensor_msgs/PointCloud2), ``"compressed"`` (ranges on
    fixed rays, the module's text), ``"shm"`` or ``"cuda"``. ``remote``: ``"auto"`` (``"compressed"`` where every
    publisher offers it, that is where it has rays, else ``"raw"``), ``"compressed"`` or ``"raw"``.
    ``copy=False``: shm and cuda clouds are views into the ring (check ``PointCloudFrame.is_valid()``)."""

    KIND = "points"
    TRANSPORTS = {"raw": topics.cloud, "shm": topics.cloud_shm, "cuda": topics.cloud_cuda,
                  "compressed": topics.cloud_compressed}
    TYPES = {"raw": PointCloud2, "shm": ShmPointCloud2, "cuda": CudaPointCloud2, "compressed": CompressedPointCloud2}
    REMOTE = ("raw", "compressed")
    RAYS_TIMEOUT_S = 5.0

    def __init__(self, session: zenoh.Session, base: str, transport: str = AUTO, copy: bool = True,
                 local: str = "auto", max_hz: float | None = None, device=None, remote: str = "auto"):
        if remote not in ("auto",) + self.REMOTE:
            raise ValueError(f"remote {remote!r}: one of auto, {', '.join(self.REMOTE)}")
        self._remote_auto = remote == "auto"
        self._rays: tuple[str, np.ndarray] | None = None
        super().__init__(session, base, transport, local, "raw" if self._remote_auto else remote, copy, max_hz, device)

    def _choice(self, records: list[dict]):
        if self._remote_auto:  # compressed where every source has rays; a relay's record names no network form
            sources = [r for r in records if not r.get("relayed_from")] or records
            self.remote = ("compressed" if sources and all("compressed" in r.get("transports", ()) for r in sources)
                           else "raw")
        return super()._choice(records)

    def _rays_of(self, name: str) -> np.ndarray:
        """The rays of this name, asked from the publisher at the first cloud that names them."""
        if self._rays is None or self._rays[0] != name:
            for reply in replies(self._session, topics.cloud_rays(self.base), self.RAYS_TIMEOUT_S):
                if reply.ok is None:
                    continue
                rays = Rays.deserialize(reply.ok.payload.to_bytes())
                if rays.id != name:
                    continue  # another publisher of the stream, or rays that changed meanwhile
                directions = np.frombuffer(rays.directions, "<f4")
                if directions.size != rays.height * rays.width * 3:  # the name is the publisher's: not checked
                    raise ProtocolError(f"{self.base}: the rays {name} do not have their size")
                self._rays = (name, directions.reshape(rays.height, rays.width, 3))
                break
            else:
                raise ProtocolError(f"{self.base}: no publisher answered the rays {name} of its compressed clouds "
                                    f"within {self.RAYS_TIMEOUT_S:g} s ({topics.cloud_rays(self.base)})")
        return self._rays[1]

    def latest(self, consume: bool = True, timeout: float | None = None) -> PointCloudFrame | None:
        """The newest cloud, as ImageSubscriber.latest() hands out frames."""
        taken = self._take(consume, timeout)
        if taken is None:
            return None
        payload, meta, key, receive_ns, transport = taken
        message = self.TYPES[transport].deserialize(payload)
        if transport == "compressed":
            points = decompress_cloud(message, self._rays_of(message.rays))
            return PointCloudFrame(points=points, fields=message.fields, point_step=message.point_step,
                                   is_dense=message.is_dense, header=message.header, key=key, receive_ns=receive_ns,
                                   seq=meta.get("seq"), source=meta.get("source", ""),
                                   publish_ns=meta.get("publish_ns"), meta=meta)
        if message.is_bigendian:
            raise RuntimeError(f"{self.base}: big-endian clouds are not supported")
        dtype = points_dtype(message.fields, message.point_step)
        row = message.width * message.point_step
        valid = None
        if transport == "raw":
            if message.row_step < row or len(message.data) < message.row_step * message.height:
                raise RuntimeError(f"{self.base}: the cloud's data does not fit its size and row_step")
            data = np.frombuffer(message.data, np.uint8, message.row_step * message.height)
            data = data.reshape(message.height, message.row_step)[:, :row]
            points = np.ascontiguousarray(data).reshape(-1).view(dtype).reshape(message.height, message.width)
        else:
            if self._moved(message, transport):
                return None
            if message.row_step != row:
                raise RuntimeError(f"{self.base}: ring clouds have rows without padding (row_step "
                                   f"{message.row_step}, width * point_step {row})")
            got = self._read_ring(message, transport, (message.height, message.width, message.point_step),
                                  np.dtype(np.uint8), row * message.height)
            if got is None:
                self.stale += 1
                return None
            points, valid = got
            if transport == "shm":  # bytes (the rings take plain arrays) seen as points
                points = points.reshape(-1).view(dtype).reshape(message.height, message.width)
        return PointCloudFrame(points=points, fields=message.fields, point_step=message.point_step,
                               is_dense=message.is_dense, header=message.header, key=key, receive_ns=receive_ns,
                               seq=meta.get("seq"), source=meta.get("source", ""), publish_ns=meta.get("publish_ns"),
                               meta=meta, valid=valid)
