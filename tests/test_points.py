"""Point cloud streams (points.py): layouts, and publisher and subscriber over real Zenoh sessions on every
transport."""

import multiprocessing as mp
import socket
import time

import numpy as np
import pytest

import xblick
from xblick import _core
from xblick.camera import CameraPublisher, ImageSubscriber
from xblick.gpu import CudaImage
from xblick.msg import CudaPointCloud2, PointField, ShmPointCloud2
from xblick.points import (XYZ, XYZRGB, PointCloudPublisher, PointCloudSubscriber, pack_rgb, points_dtype,
                           unpack_rgb)
from xblick.session import open_session

needs_cuda = pytest.mark.skipif(_core.cuda_device_count() == 0, reason="no CUDA device")
H, W = 48, 64


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture
def sessions():
    port = free_port()
    a = open_session(listen=[f"127.0.0.1:{port}"])
    b = open_session(connect=[f"127.0.0.1:{port}"])
    yield a, b
    b.close()
    a.close()


def cloud(seed=0, h=H, w=W) -> np.ndarray:
    """An organized XYZRGB cloud: a plane at 1 m, invalid points (NaN) in the first row; blue is the seed (mod 256)."""
    seed %= 256
    points = np.zeros((h, w), points_dtype(*XYZRGB))
    y, x = np.mgrid[0:h, 0:w].astype(np.float32)
    points["x"], points["y"], points["z"] = (x - w / 2) * 0.01 + seed, (y - h / 2) * 0.01, 1.0
    points["x"][0] = points["y"][0] = points["z"][0] = np.nan
    points["rgb"] = pack_rgb(np.stack([x % 256, y % 256, np.full_like(x, seed)], -1).astype(np.uint8))
    return points


def same(a: np.ndarray, b: np.ndarray):
    assert a.dtype == b.dtype and a.shape == b.shape
    assert a.tobytes() == b.tobytes()  # NaN points included


def publish_until(pub, points, sub, timeout=10.0, **kwargs):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pub.subscribed:
            pub.publish(points, **kwargs)
        got = sub.latest(timeout=0.05)
        if got is not None:
            return got
    raise TimeoutError("no cloud")


# ---------------------------------------------------------------------------------------------- layouts


def test_layouts():
    dtype = points_dtype(*XYZRGB)
    assert dtype.itemsize == 16 and dtype.names == ("x", "y", "z", "rgb") and dtype.fields["rgb"][1] == 12
    assert points_dtype(*XYZ).names == ("x", "y", "z") and points_dtype(*XYZ).itemsize == 16
    counted = points_dtype([PointField("ring", 0, PointField.UINT16, 2), PointField("t", 8, PointField.FLOAT64)], 16)
    assert counted.fields["ring"][0].shape == (2,)
    for fields, step in (([], 16), ([PointField("x", 14, PointField.FLOAT32)], 16),
                         ([PointField("x", 0, 99)], 16), ([PointField("x", 0, 7), PointField("x", 4, 7)], 16)):
        with pytest.raises(ValueError):
            points_dtype(fields, step)


def test_rgb_packing_is_pcls():
    colours = np.array([[255, 128, 1], [0, 0, 0], [12, 34, 56]], np.uint8)
    packed = pack_rgb(colours)
    assert packed.dtype == np.float32
    assert packed[0].view(np.uint32) == 0x00FF8001  # 0x00RRGGBB
    np.testing.assert_array_equal(unpack_rgb(packed), colours)


def test_descriptor_round_trip_and_versions():
    fields = XYZRGB[0]
    shm = ShmPointCloud2(height=2, width=3, fields=fields, point_step=16, row_step=48, is_dense=True, host="h",
                         segment="/xblick-x", slot=1, seq=5)
    assert ShmPointCloud2.deserialize(shm.serialize()) == shm
    cuda = CudaPointCloud2(height=2, width=3, fields=fields, point_step=16, row_step=48, host="h", endpoint="e",
                           device_uuid="ab" * 16, slot=2, seq=6)
    assert CudaPointCloud2.deserialize(cuda.serialize()) == cuda
    assert ShmPointCloud2.ENCODING == "application/cdr;xblick_msgs/msg/ShmPointCloud2@1"
    assert xblick.PointCloud2.ENCODING == "application/cdr;sensor_msgs/msg/PointCloud2"


# ------------------------------------------------------------------------------------------ over Zenoh


def test_nothing_without_subscribers(sessions):
    a, _ = sessions
    pub = PointCloudPublisher(a, "t/p0/points", fields=XYZRGB[0], point_step=16, frame_id="depth")
    time.sleep(0.2)
    assert not pub.subscribed and pub.publish(cloud()) == []
    pub.close()


@pytest.mark.parametrize("transport", ["raw", "shm"])
def test_cloud_is_exact(sessions, transport):
    a, b = sessions
    pub = PointCloudPublisher(a, "t/p1/points", fields=XYZRGB[0], point_step=16, frame_id="depth", source="test")
    sub = PointCloudSubscriber(b, "t/p1/points", transport)
    got = publish_until(pub, cloud(3), sub, stamp_ns=123_456_789_000)
    same(got.points, cloud(3))
    assert got.header.frame_id == "depth" and got.stamp_ns == 123_456_789_000 and got.source == "test"
    assert not got.is_dense and got.key == f"t/p1/points/{'cloud' if transport == 'raw' else 'cloud/shm/' + _core.host_id()}"
    np.testing.assert_array_equal(unpack_rgb(got.points["rgb"])[..., 2], 3)
    pub.close()
    sub.close()


@pytest.mark.parametrize("transport", ["raw", "shm"])
def test_empty_cloud(sessions, transport):
    """A cloud of no points (height 1, width 0: e.g. a tracker with nothing to follow yet) goes as any other, on rings
    made for it, and a larger cloud after it as well."""
    a, b = sessions
    pub = PointCloudPublisher(a, "t/pe/points", fields=XYZRGB[0], point_step=16, frame_id="depth")
    sub = PointCloudSubscriber(b, "t/pe/points", transport)
    empty = np.zeros((1, 0), points_dtype(*XYZRGB))
    got = publish_until(pub, empty, sub)
    assert got.points.shape == (1, 0) and got.points.dtype == empty.dtype
    same(publish_until(pub, cloud(2), sub).points, cloud(2))
    pub.close()
    sub.close()


def test_any_array_of_the_layouts_size(sessions):
    """publish() takes the bytes of height x width points in any C-contiguous array, e.g. (h, w, 4) float32."""
    a, b = sessions
    pub = PointCloudPublisher(a, "t/p2/points", fields=XYZRGB[0], point_step=16, frame_id="depth")
    sub = PointCloudSubscriber(b, "t/p2/points", "raw")
    flat = cloud(1).view(np.float32).reshape(H, W, 4)
    same(publish_until(pub, flat, sub, is_dense=False).points, cloud(1))
    with pytest.raises(ValueError, match="bytes"):
        pub.publish(np.zeros((H, W, 3), np.float32))
    pub.close()


def test_shm_zero_copy_detects_overwrite(sessions):
    a, b = sessions
    pub = PointCloudPublisher(a, "t/p3/points", fields=XYZ[0], point_step=16, frame_id="depth", slots=2)
    sub = PointCloudSubscriber(b, "t/p3/points", "shm", copy=False)
    points = np.zeros((H, W), points_dtype(*XYZ))
    points["z"] = 2.0
    got = publish_until(pub, points, sub, is_dense=True)
    assert got.is_valid() and got.is_dense and float(got.points["z"][5, 5]) == 2.0
    pub.publish(points)
    pub.publish(points)  # two more clouds in a ring of two: the first slot is overwritten
    assert not got.is_valid()


def test_auto(sessions):
    a, b = sessions
    sub = PointCloudSubscriber(b, "t/p4/points")  # auto, before the publisher exists
    assert sub.transport is None and sub.latest() is None
    pub = PointCloudPublisher(a, "t/p4/points", fields=XYZRGB[0], point_step=16, frame_id="depth")
    same(publish_until(pub, cloud(4), sub).points, cloud(4))
    assert sub.transport == "shm"
    remote = PointCloudPublisher(a, "t/p5/points", fields=XYZRGB[0], point_step=16, frame_id="depth")
    remote._host = "another-host"  # as a publisher on another machine announces itself
    sub2 = PointCloudSubscriber(b, "t/p5/points")
    same(publish_until(remote, cloud(5), sub2).points, cloud(5))
    assert sub2.transport == "raw"
    for closable in (sub2, sub, remote, pub):
        closable.close()


def test_info_record_and_kinds(sessions):
    a, b = sessions
    pub = PointCloudPublisher(a, "t/p6/points", fields=XYZRGB[0], point_step=16, frame_id="depth", source="test")
    camera = CameraPublisher(a, "t/p6/color", encoding="rgb8", frame_id="cam")
    end = time.monotonic() + 10
    records = []
    while not records and time.monotonic() < end:
        records = xblick.query_info(b, "t/p6/points", timeout=0.5)
    (record,) = records
    assert record["kind"] == "points" and record["fields"] == ["x", "y", "z", "rgb"]
    assert record["transports"] == ["raw", "shm"] and record["frame_id"] == "depth" and "encoding" not in record
    # a subscriber of the other kind fails loudly instead of waiting for messages that never come
    wrong = ImageSubscriber(b, "t/p6/points", "auto")
    with pytest.raises(xblick.ProtocolError, match="kind 'points'"):
        wrong.latest(timeout=10)
    wrong2 = PointCloudSubscriber(b, "t/p6/color")
    with pytest.raises(xblick.ProtocolError, match="kind 'image'"):
        wrong2.latest(timeout=10)
    pub.close()
    camera.close()


def test_an_unknown_transport_is_refused(sessions):
    _, b = sessions
    with pytest.raises(ValueError):
        PointCloudSubscriber(b, "t/p7/points", "jpeg")


# -------------------------------------------------------------------------------------------- cuda (GPU)


def _publish_clouds(port, base, device, stop, ready):
    """Child (a CUDA ring cannot be opened by the process that made it): publish clouds on every transport; with
    ``device`` from a device array (a received cuda cloud of another publisher would be one; here a DeviceBuffer)."""
    session = open_session(listen=[f"127.0.0.1:{port}"])
    pub = PointCloudPublisher(session, base, fields=XYZRGB[0], point_step=16, frame_id="depth", cuda_device=0)
    buffer = _core.make_device_buffer(0, H * W * 16) if device else None
    ready.set()
    end = time.monotonic() + 60
    seed = 0
    while time.monotonic() < end and not stop.is_set():
        if pub.subscribed:
            seed += 1
            points = cloud(seed)
            if buffer is not None:
                buffer.upload(points.reshape(-1).view(np.uint8))
                pub.publish(CudaImage(buffer.ptr, (H, W, 16), np.uint8, 0, owner=buffer))
            else:
                pub.publish(points)
        time.sleep(0.02)
    pub.close()
    session.close()


@needs_cuda
@pytest.mark.parametrize("device", [False, True])
def test_cuda_transport(device):
    ctx = mp.get_context("spawn")
    port = free_port()
    stop, ready = ctx.Event(), ctx.Event()
    child = ctx.Process(target=_publish_clouds, args=(port, "t/pc/points", device, stop, ready), daemon=True)
    child.start()
    assert ready.wait(60)
    session = open_session(connect=[f"127.0.0.1:{port}"])
    try:
        cuda_sub = PointCloudSubscriber(session, "t/pc/points", local="cuda")  # auto, preferring the CUDA ring
        shm_sub = PointCloudSubscriber(session, "t/pc/points", "shm")  # from the publisher's download (device)
        end = time.monotonic() + 60
        got = got_shm = None
        while (got is None or got_shm is None) and time.monotonic() < end:
            got = got or cuda_sub.latest(timeout=0.1)
            got_shm = got_shm or shm_sub.latest(timeout=0.1)
        assert got is not None and got_shm is not None
        assert cuda_sub.transport == "cuda" and isinstance(got.points, CudaImage)
        assert got.points.__cuda_array_interface__["shape"] == (H, W, 16)
        host = got.numpy()
        seed = int(unpack_rgb(host["rgb"])[0, 0, 2])
        same(host, cloud(seed))
        same(got_shm.points, cloud(int(unpack_rgb(got_shm.points["rgb"])[0, 0, 2])))
    finally:
        stop.set()
        session.close()
        child.join(20)


# ---------------------------------------------------------------------------------------------- compressed clouds
# cloud/compressed: a cloud on fixed rays as its ranges (RVL) and colours (JPEG), the rays sent once on request

OTHER_HOST = "0" * 32


def camera_rays(h=H, w=W, f=60.0) -> np.ndarray:
    """A pinhole camera's rays: (x/z, y/z, 1) of each pixel, so that a point's range is its z."""
    y, x = np.mgrid[0:h, 0:w].astype(np.float32)
    return np.stack([(x - w / 2) / f, (y - h / 2) / f, np.ones_like(x)], -1)


def camera_cloud(seed=0, rays=None) -> np.ndarray:
    """What a point cloud node makes of a depth image in millimetres: z = depth / 1000, x and y along the rays; no
    point where the depth is 0 (a block and scattered pixels); smooth colours."""
    rays = camera_rays() if rays is None else rays
    h, w = rays.shape[:2]
    rng = np.random.default_rng(seed)
    y, x = np.mgrid[0:h, 0:w]
    depth = (800 + 3 * x + 5 * y + rng.integers(0, 4, (h, w)) + seed).astype(np.uint16)
    depth[5:9, 10:30] = 0
    depth[rng.random((h, w)) < 0.05] = 0
    z = depth.astype(np.float32) * np.float32(0.001)
    z[depth == 0] = np.nan
    points = np.zeros((h, w), points_dtype(*XYZRGB))
    points["x"], points["y"], points["z"] = rays[..., 0] * z, rays[..., 1] * z, z
    points["rgb"] = pack_rgb(np.stack([2 * x + seed, 3 * y, x + y], -1).astype(np.uint8))
    return points


def close_cloud(got: np.ndarray, expected: np.ndarray, metres: float, colour: float):
    """The same points within ``metres``, NaN at the same places; colours within ``colour`` levels on average."""
    assert got.dtype == expected.dtype and got.shape == expected.shape
    for name in "xyz":
        assert np.array_equal(np.isnan(got[name]), np.isnan(expected[name]))
        assert np.nanmax(np.abs(got[name] - expected[name])) <= metres, name
    if "rgb" in expected.dtype.names:
        difference = np.abs(unpack_rgb(got["rgb"]).astype(int) - unpack_rgb(expected["rgb"]))
        assert difference.mean() <= colour, difference.mean()


def test_a_camera_cloud_is_its_ranges_and_colours():
    from xblick.msg import CompressedPointCloud2
    from xblick.points import compress_cloud, decompress_cloud, rays_id
    rays, points = camera_rays(), camera_cloud()
    for codec, colour in (("png", 0), ("jpeg", 3)):
        ranges, colors = compress_cloud(points, rays, 0.001, codec)
        assert ranges.format == "16UC1; compressedDepth rvl" and codec in colors.format
        assert len(ranges.data) + len(colors.data) < points.nbytes / 4
        message = CompressedPointCloud2(height=H, width=W, fields=XYZRGB[0], point_step=16, rays=rays_id(rays),
                                        range_step=0.001, range_format=ranges.format, range=ranges.data,
                                        color_format=colors.format, color=colors.data)
        back = decompress_cloud(CompressedPointCloud2.deserialize(message.serialize()), rays)
        close_cloud(back, points, 1e-6, colour)  # millimetre depth: the ranges are exact but for float rounding
    assert rays_id(rays) != rays_id(camera_rays(f=61.0)) and rays_id(rays) != rays_id(rays.reshape(W, H, 3))


def test_a_scanner_cloud_is_rounded_to_the_range_step():
    """Rays that are no camera's: unit vectors of a scanner's beams, ranges along them, no colour."""
    from xblick.points import compress_cloud, decompress_cloud
    from xblick.msg import CompressedPointCloud2
    rng = np.random.default_rng(1)
    azimuth, elevation = np.meshgrid(np.linspace(-3, 3, 90, dtype=np.float32), np.linspace(-0.3, 0.3, 16, dtype=np.float32))
    rays = np.stack([np.cos(elevation) * np.cos(azimuth), np.cos(elevation) * np.sin(azimuth), np.sin(elevation)], -1)
    ranges = rng.uniform(0.5, 30.0, azimuth.shape).astype(np.float32)
    ranges[rng.random(azimuth.shape) < 0.1] = np.nan
    points = np.zeros(azimuth.shape, points_dtype(*XYZ))
    for i, name in enumerate("xyz"):
        points[name] = rays[..., i] * ranges
    encoded, colors = compress_cloud(points, rays, 0.002)
    assert colors is None
    message = CompressedPointCloud2(height=16, width=90, fields=XYZ[0], point_step=16, range_step=0.002,
                                    range_format=encoded.format, range=encoded.data)
    close_cloud(decompress_cloud(message, rays), points, 0.001 + 1e-5, 0)  # half a step along the ray


def test_a_cloud_that_does_not_fit_its_rays_is_refused(sessions):
    from xblick.points import compress_cloud
    rays, points = camera_rays(), camera_cloud()
    shifted = points.copy()
    shifted["x"] += 0.05  # another frame than the rays'
    with pytest.raises(ValueError, match="does not lie on its rays"):
        compress_cloud(shifted, rays, 0.001)
    with pytest.raises(ValueError, match="does not lie on its rays"):
        compress_cloud(points, camera_rays(f=70.0), 0.001)
    with pytest.raises(ValueError, match="do not fit 1 to 65535 steps"):
        compress_cloud(points, rays, 0.00001)
    with pytest.raises(ValueError, match="rays for"):
        compress_cloud(points, camera_rays(h=H + 1), 0.001)
    a, _ = sessions
    with pytest.raises(ValueError, match="x, y, z and rgb"):
        PointCloudPublisher(a, "t/c0/points", fields=XYZ[0] + [PointField("ring", 12, PointField.UINT16)],
                            point_step=16, frame_id="depth", rays=rays)
    with pytest.raises(ValueError, match="none of them 0"):
        PointCloudPublisher(a, "t/c0/points", fields=XYZ[0], point_step=16, frame_id="depth", rays=rays * 0)
    plain = PointCloudPublisher(a, "t/c0/points", fields=XYZ[0], point_step=16, frame_id="depth")
    try:
        assert "compressed" not in plain.info_record()["transports"]
        with pytest.raises(ValueError, match="made without rays"):
            plain.set_rays(rays)
    finally:
        plain.close()


def test_compressed_transport_and_rays_that_change(sessions):
    """A compressed subscriber asks the rays once and gets every cloud on them; after set_rays (a camera back with
    another size) it asks the new ones."""
    a, b = sessions
    pub = PointCloudPublisher(a, "t/c1/points", fields=XYZRGB[0], point_step=16, frame_id="depth", source="test",
                              rays=camera_rays(), color_codec="png")
    sub = PointCloudSubscriber(b, "t/c1/points", "compressed")
    asked = []
    counter = a.declare_subscriber("t/c1/points/cloud/compressed", asked.append)
    try:
        assert "compressed" in pub.info_record()["transports"]
        got = publish_until(pub, camera_cloud(3), sub, stamp_ns=123_456_789_000)
        close_cloud(got.points, camera_cloud(3), 1e-6, 0)
        assert got.key == "t/c1/points/cloud/compressed" and got.header.frame_id == "depth"
        assert got.stamp_ns == 123_456_789_000 and got.source == "test" and got.seq == pub.seq and not got.is_dense
        size = len(asked[-1].payload.to_bytes())
        assert size < camera_cloud(3).nbytes / 4, size
        first = sub._rays[0]
        close_cloud(publish_until(pub, camera_cloud(4), sub).points, camera_cloud(4), 1e-6, 0)
        assert sub._rays[0] == first
        small = camera_rays(h=24, w=32, f=30.0)
        pub.set_rays(small)
        end = time.monotonic() + 10
        while time.monotonic() < end:
            got = publish_until(pub, camera_cloud(5, small), sub)
            if got.points.shape == (24, 32):
                break
        close_cloud(got.points, camera_cloud(5, small), 1e-6, 0)
        assert sub._rays[0] != first
    finally:
        counter.undeclare()
        sub.close()
        pub.close()


@pytest.mark.parametrize("rays", [True, False])
def test_auto_takes_compressed_from_another_host_where_the_publisher_has_rays(sessions, monkeypatch, rays):
    a, b = sessions
    with monkeypatch.context() as patched:
        patched.setattr(_core, "host_id", lambda: OTHER_HOST)
        pub = PointCloudPublisher(a, "t/c2/points", fields=XYZRGB[0], point_step=16, frame_id="depth",
                                  rays=camera_rays() if rays else None)
    sub = PointCloudSubscriber(b, "t/c2/points")
    try:
        got = publish_until(pub, camera_cloud(1), sub)
        assert sub.transport == ("compressed" if rays else "raw"), sub.transport_reason
        assert got.key == "t/c2/points/cloud" + ("/compressed" if rays else "")
        if rays:
            close_cloud(got.points, camera_cloud(1), 1e-6, 3)  # JPEG of quality 95 by default
        else:
            same(got.points, camera_cloud(1))
    finally:
        sub.close()
        pub.close()


def test_auto_can_be_told_to_take_raw_from_another_host(sessions, monkeypatch):
    a, b = sessions
    with monkeypatch.context() as patched:
        patched.setattr(_core, "host_id", lambda: OTHER_HOST)
        pub = PointCloudPublisher(a, "t/c3/points", fields=XYZRGB[0], point_step=16, frame_id="depth",
                                  rays=camera_rays())
    sub = PointCloudSubscriber(b, "t/c3/points", remote="raw")
    try:
        same(publish_until(pub, camera_cloud(2), sub).points, camera_cloud(2))
        assert sub.transport == "raw"
        with pytest.raises(ValueError, match="remote"):
            PointCloudSubscriber(b, "t/c3/points", remote="shm")
    finally:
        sub.close()
        pub.close()
