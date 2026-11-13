"""The point cloud node (xblick-points) end to end: Python publishes a camera (depth, colour, their camera_info,
tf_static), the program makes the clouds, Python checks every point against the camera model, independently of how
the node computes it: projected with the depth camera's distortion, a point lands on its own pixel at that pixel's
depth; its colour is the colour pixel its projection into the colour camera meets (the colour image encodes the pixel
coordinates).

Needs the C++ build with XBLICK_POINTS (XBLICK_CPP_BUILD with xblick-points) and a GPU."""

import os
import socket
import subprocess
import threading
import time
from pathlib import Path

import numpy as np
import pytest

from xblick import _core
from xblick.camera import CameraPublisher, StaticTransformPublisher
from xblick.msg import (CameraInfo, DiagnosticArray, Header, Quaternion, Time, Transform, TransformStamped,
                        Vector3)
from xblick.points import PointCloudSubscriber, unpack_rgb
from xblick.session import open_session

BUILD = os.environ.get("XBLICK_CPP_BUILD")
PROGRAM = Path(BUILD) / "xblick-points" if BUILD else None
pytestmark = [
    pytest.mark.skipif(not BUILD, reason="XBLICK_CPP_BUILD (the C++ build directory) is not set"),
    pytest.mark.skipif(bool(BUILD) and not PROGRAM.exists(), reason="built without XBLICK_POINTS"),
    pytest.mark.skipif(_core.cuda_device_count() == 0, reason="no CUDA device"),
]

DW, DH = 80, 60  # depth camera
CW, CH = 120, 90  # colour camera
DEPTH_INFO = CameraInfo.pinhole(DW, DH, 70.0, 71.0, 39.2, 29.7, frame_id="Depth")
DEPTH_INFO.distortion_model = "rational_polynomial"
DEPTH_INFO.d = np.array([0.4, -0.1, 0.001, -0.002, 0.01, 0.75, 0.05, 0.02])
COLOR_INFO = CameraInfo.pinhole(CW, CH, 100.0, 100.5, 60.3, 44.8, frame_id="Color")
COLOR_INFO.d = np.array([0.05, -0.02, 0.0005, 0.0003, 0.001])  # plumb_bob
# the colour camera in the depth camera's frame: 32 mm to the side, turned 6 degrees about x
ANGLE = np.radians(6.0)
COLOR_POSE = TransformStamped(Header(frame_id="Depth"), "Color",
                              Transform(Vector3(-0.032, 0.002, 0.004),
                                        Quaternion(np.sin(ANGLE / 2), 0.0, 0.0, np.cos(ANGLE / 2))))


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def distort(info: CameraInfo, x, y):
    k1, k2, p1, p2, k3 = info.d[:5]
    k4, k5, k6 = info.d[5:8] if len(info.d) == 8 else (0.0, 0.0, 0.0)
    r2 = x * x + y * y
    radial = (1 + r2 * (k1 + r2 * (k2 + r2 * k3))) / (1 + r2 * (k4 + r2 * (k5 + r2 * k6)))
    return (x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x), y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y)


def project(info: CameraInfo, p: np.ndarray):
    xd, yd = distort(info, p[..., 0] / p[..., 2], p[..., 1] / p[..., 2])
    return info.k[0] * xd + info.k[2], info.k[4] * yd + info.k[5]


def depth_image() -> np.ndarray:
    """A tilted plane in millimetres, with holes (0)."""
    v, u = np.mgrid[0:DH, 0:DW]
    depth = (1000 + 4 * u - 3 * v).astype(np.uint16)
    depth[10:14, 20:30] = 0
    depth[::7, ::5] = 0
    return depth


def color_image() -> np.ndarray:
    """Every pixel says where it is: red = column, green = row, blue = 7 (0 is left for points without colour)."""
    v, u = np.mgrid[0:CH, 0:CW]
    return np.stack([u, v, np.full_like(u, 7)], -1).astype(np.uint8)


class Camera:
    """Publishes depth and colour at 30 Hz (colour 0.4 ms after depth, as two sensors' stamps differ) until closed."""

    def __init__(self, session, cuda: bool, color_offset_ns: int = 400_000, host: str | None = None,
                 one_frame: bool = False):
        """one_frame: colour in the depth camera's frame (a simulation renders both from one pose), no tf_static"""
        device = 0 if cuda else None
        self.depth = CameraPublisher(session, "t/k/depth", encoding="16UC1", frame_id="Depth", camera_info=DEPTH_INFO,
                                     codec="png", cuda_device=device)
        color_info = COLOR_INFO
        if one_frame:
            color_info = CameraInfo.from_yaml(COLOR_INFO.to_yaml("c"))
            color_info.header.frame_id = "Depth"
        self.color = CameraPublisher(session, "t/k/color", encoding="rgb8", frame_id=color_info.header.frame_id,
                                     camera_info=color_info, cuda_device=device)
        if host is not None:  # as a camera on another machine announces itself
            self.depth._host = self.color._host = host
        self.tf = None if one_frame else StaticTransformPublisher(session, "t/k", [COLOR_POSE])
        self.stamps = []
        self._stop = threading.Event()
        self._offset = color_offset_ns
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        depth, color = depth_image(), color_image()
        while not self._stop.is_set():
            stamp = time.time_ns()
            if self.depth.subscribed or self.color.subscribed:
                self.color.publish(color, stamp + self._offset)
                self.depth.publish(depth, stamp)
                self.stamps.append(stamp)
            time.sleep(1 / 30)

    def close(self):
        self._stop.set()
        self._thread.join()
        for p in (self.depth, self.color, self.tf):
            if p is not None:
                p.close()


@pytest.fixture
def node():
    started = []

    def start(port, *extra):
        proc = subprocess.Popen([str(PROGRAM), "--connect", f"127.0.0.1:{port}", "--depth", "t/k/depth",
                                 "--cuda-device", "0", *extra], stderr=subprocess.PIPE, text=True)
        started.append(proc)
        return proc

    yield start
    for proc in started:
        proc.terminate()
        _, err = proc.communicate(timeout=20)
        print(err)


def first_cloud(sub, timeout=30.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        got = sub.latest(timeout=0.1)
        if got is not None:
            return got
    raise TimeoutError(f"no cloud on {sub.base} ({sub.transport})")


def check_geometry(points: np.ndarray, max_depth_mm: int = 0):
    depth = depth_image()
    xyz = np.stack([points["x"], points["y"], points["z"]], -1).astype(np.float64)
    holes = (depth == 0) | ((depth > max_depth_mm) if max_depth_mm else False)
    assert np.isnan(xyz[holes]).all()
    valid = ~holes
    assert np.isfinite(xyz[valid]).all()
    np.testing.assert_allclose(xyz[valid][:, 2], depth[valid] / 1000.0, rtol=1e-6)
    u, v = project(DEPTH_INFO, xyz[valid])
    vv, uu = np.nonzero(valid)
    assert np.abs(u - uu).max() < 2e-3 and np.abs(v - vv).max() < 2e-3  # each point on its own pixel
    return xyz, valid


def check_colour(points: np.ndarray, xyz: np.ndarray, valid: np.ndarray, moved: bool = True):
    """moved: the colour camera at COLOR_POSE; else in the depth camera's place"""
    q = COLOR_POSE.transform.rotation
    r = np.array([[1, 0, 0], [0, np.cos(ANGLE), -np.sin(ANGLE)], [0, np.sin(ANGLE), np.cos(ANGLE)]])
    assert np.isclose(q.x, np.sin(ANGLE / 2))
    t = np.array([-0.032, 0.002, 0.004])
    if not moved:
        r, t = np.eye(3), np.zeros(3)
    in_color = (xyz[valid] - t) @ r  # p_color = R^T (p_depth - t)
    u, v = project(COLOR_INFO, in_color)
    ui, vi = np.rint(u).astype(int), np.rint(v).astype(int)
    seen = (ui >= 0) & (vi >= 0) & (ui < CW) & (vi < CH)
    rgb = unpack_rgb(points["rgb"])[valid]
    assert (rgb[~seen] == 0).all()  # not seen by the colour camera: black
    assert (rgb[seen, 2] == 7).all()  # seen: a colour pixel
    exact = (rgb[seen, 0] == ui[seen]) & (rgb[seen, 1] == vi[seen])
    near = (np.abs(rgb[seen, 0].astype(int) - ui[seen]) <= 1) & (np.abs(rgb[seen, 1].astype(int) - vi[seen]) <= 1)
    assert near.all() and exact.mean() > 0.99, exact.mean()  # float32 on the GPU: a pixel's border may round over
    assert seen.mean() > 0.5


@pytest.mark.parametrize("cuda_inputs", [True, False])
def test_coloured_cloud(node, cuda_inputs):
    port = free_port()
    session = open_session(listen=[f"127.0.0.1:{port}"])
    camera = Camera(session, cuda=cuda_inputs)
    proc = node(port, "--color", "t/k/color")
    statuses = []
    session.declare_subscriber("t/k/diagnostics", lambda s: statuses.extend(
        DiagnosticArray.deserialize(s.payload.to_bytes()).status))
    try:
        cuda_sub = PointCloudSubscriber(session, "t/k/points", local="cuda")
        got = first_cloud(cuda_sub)
        assert cuda_sub.transport == "cuda"
        assert [f.name for f in got.fields] == ["x", "y", "z", "rgb"] and got.header.frame_id == "Depth"
        assert got.stamp_ns in camera.stamps and not got.is_dense
        points = got.numpy()
        xyz, valid = check_geometry(points)
        check_colour(points, xyz, valid)
        shm = first_cloud(PointCloudSubscriber(session, "t/k/points", "shm"))  # the node's download
        check_geometry(shm.points)
        end = time.monotonic() + 10
        while not any(s.name == "point_cloud" and s.level == 0 and s.get("clouds/s") not in (None, "0.00")
                      for s in statuses) and time.monotonic() < end:
            time.sleep(0.1)
        status = [s for s in statuses if s.name == "point_cloud"][-1]
        assert status.level == 0, (status.message, status.values)
        assert status.get("depth transport") == ("cuda" if cuda_inputs else "shm")
        assert proc.poll() is None
    finally:
        camera.close()
        session.close()


def test_the_node_offers_its_cloud_compressed(node):
    """What a reader on another host takes: the node's cloud as ranges on the depth camera's rays and colours. It is
    the cloud of the ring: the same points (the depth is in millimetres), the colours through JPEG."""
    port = free_port()
    session = open_session(listen=[f"127.0.0.1:{port}"])
    camera = Camera(session, cuda=False)
    node(port, "--color", "t/k/color")
    sizes = []
    session.declare_subscriber("t/k/points/cloud/compressed", lambda s: sizes.append(len(s.payload.to_bytes())))
    try:
        compressed = PointCloudSubscriber(session, "t/k/points", "compressed")
        whole = PointCloudSubscriber(session, "t/k/points", "shm")
        by_seq, pair = {}, None
        end = time.monotonic() + 20
        while pair is None and time.monotonic() < end:
            for name, sub in (("compressed", compressed), ("whole", whole)):
                got = sub.latest(timeout=0.05)
                if got is not None:
                    by_seq.setdefault(got.seq, {})[name] = got
                    if len(by_seq[got.seq]) == 2:
                        pair = by_seq[got.seq]
        assert pair is not None, "no cloud came on both transports"
        got, expected = pair["compressed"].points, pair["whole"].points
        assert got.dtype == expected.dtype and got.shape == expected.shape and pair["compressed"].header.frame_id == "Depth"
        for name in "xyz":
            assert np.array_equal(np.isnan(got[name]), np.isnan(expected[name]))
            assert np.nanmax(np.abs(got[name] - expected[name])) <= 1e-6, name
        valid = ~np.isnan(expected["z"])
        assert valid.sum() > 100
        colour = np.abs(unpack_rgb(got["rgb"]).astype(int) - unpack_rgb(expected["rgb"]))[valid]
        assert colour.mean() < 6, colour.mean()
        assert sizes and max(sizes) < expected.nbytes / 3, (max(sizes), expected.nbytes)
        check_geometry(got)
    finally:
        camera.close()
        session.close()


def test_cloud_without_colour_and_from_another_host(node):
    """Inputs of a camera on another host come over the network (raw here, to compare exactly)."""
    port = free_port()
    session = open_session(listen=[f"127.0.0.1:{port}"])
    camera = Camera(session, cuda=False, host="another-host")
    node(port, "--remote", "raw")
    try:
        sub = PointCloudSubscriber(session, "t/k/points", "raw")
        got = first_cloud(sub)
        assert [f.name for f in got.fields] == ["x", "y", "z"] and got.point_step == 16
        check_geometry(got.points)
    finally:
        camera.close()
        session.close()


def test_points_beyond_max_depth_are_nan(node):
    """--max-depth: points farther away are NaN, the others unchanged."""
    port = free_port()
    session = open_session(listen=[f"127.0.0.1:{port}"])
    camera = Camera(session, cuda=False)
    node(port, "--max-depth", "1.1")
    try:
        got = first_cloud(PointCloudSubscriber(session, "t/k/points", "shm"))
        depth = depth_image()
        assert ((depth > 1100) & (depth > 0)).sum() > 500 and ((depth <= 1100) & (depth > 0)).sum() > 500
        check_geometry(got.points, max_depth_mm=1100)
    finally:
        camera.close()
        session.close()


def test_one_frame_needs_no_tf_static(node):
    """Depth and colour in one frame (the simulation, depth_to_color): the colour camera is the depth camera's
    place, without a tf_static."""
    port = free_port()
    session = open_session(listen=[f"127.0.0.1:{port}"])
    camera = Camera(session, cuda=False, one_frame=True)
    node(port, "--color", "t/k/color")
    try:
        got = first_cloud(PointCloudSubscriber(session, "t/k/points", "shm"))
        xyz, valid = check_geometry(got.points)
        check_colour(got.points, xyz, valid, moved=False)
    finally:
        camera.close()
        session.close()


def test_colour_of_another_time_is_not_taken(node):
    """Depth and colour 50 ms apart (more than --max-offset-ms): no clouds, and the node says why."""
    port = free_port()
    session = open_session(listen=[f"127.0.0.1:{port}"])
    camera = Camera(session, cuda=False, color_offset_ns=50_000_000)
    node(port, "--color", "t/k/color", "--max-offset-ms", "10")
    statuses = []
    session.declare_subscriber("t/k/diagnostics", lambda s: statuses.extend(
        DiagnosticArray.deserialize(s.payload.to_bytes()).status))
    try:
        sub = PointCloudSubscriber(session, "t/k/points", "shm")
        end = time.monotonic() + 15
        while not any(s.level == 1 for s in statuses) and time.monotonic() < end:
            assert sub.latest(timeout=0.1) is None
        warned = [s for s in statuses if s.level == 1]
        assert warned and "without a cloud" in warned[-1].message and int(warned[-1].get("unpaired")) > 0
    finally:
        camera.close()
        session.close()


def test_colour_stamped_a_little_earlier_pairs_at_once(node):
    """Colour stamped 2 ms before depth (a stereo depth stamped later than its colour camera), --max-offset-ms 5, 8
    frames at 15 Hz and then none: the last depth frame gets its cloud. The node takes a colour sample at most a
    quarter of the colour's period before the depth stamp at once. With the rule that waited for a later colour
    sample, the last frame never paired."""
    port = free_port()
    session = open_session(listen=[f"127.0.0.1:{port}"])
    depth = CameraPublisher(session, "t/k/depth", encoding="16UC1", frame_id="Depth", camera_info=DEPTH_INFO,
                            codec="png")
    color = CameraPublisher(session, "t/k/color", encoding="rgb8", frame_id="Color", camera_info=COLOR_INFO)
    tf = StaticTransformPublisher(session, "t/k", [COLOR_POSE])
    node(port, "--color", "t/k/color", "--max-offset-ms", "5")
    try:
        sub = PointCloudSubscriber(session, "t/k/points", "shm")
        end = time.monotonic() + 30
        while not (depth.subscribed and color.subscribed) and time.monotonic() < end:
            time.sleep(0.05)
        assert depth.subscribed and color.subscribed
        stamps = []
        for _ in range(8):
            stamp = time.time_ns()
            color.publish(color_image(), stamp - 2_000_000)
            depth.publish(depth_image(), stamp)
            stamps.append(stamp)
            time.sleep(1 / 15)
        clouds = []
        end = time.monotonic() + 5
        while stamps[-1] not in clouds and time.monotonic() < end:
            got = sub.latest(timeout=0.1)
            if got is not None:
                clouds.append(got.stamp_ns)
        assert stamps[-1] in clouds, (clouds, stamps)
    finally:
        for p in (depth, color, tf):
            p.close()
        session.close()


def test_max_hz(node):
    """--max-hz 5 of a 30 Hz camera: about 5 clouds a second, one per 200 ms period of the stamps, the newest depth
    frames; the frames between skipped (not decoded) and counted in the diagnostics."""
    port = free_port()
    session = open_session(listen=[f"127.0.0.1:{port}"])
    camera = Camera(session, cuda=False)
    node(port, "--color", "t/k/color", "--max-hz", "5")
    statuses = []
    session.declare_subscriber("t/k/diagnostics", lambda s: statuses.extend(
        DiagnosticArray.deserialize(s.payload.to_bytes()).status))
    try:
        sub = PointCloudSubscriber(session, "t/k/points", "shm")
        first_cloud(sub)
        stamps = []
        end = time.monotonic() + 3.0
        while time.monotonic() < end:
            got = sub.latest(timeout=0.1)
            if got is not None:
                stamps.append(got.stamp_ns)
        assert 10 <= len(stamps) <= 17, len(stamps)
        assert len({s // 200_000_000 for s in stamps}) == len(stamps)
        end = time.monotonic() + 5
        while not any(int(s.get("skipped for max-hz") or 0) > 0 for s in statuses) and time.monotonic() < end:
            time.sleep(0.1)
        status = [s for s in statuses if s.name == "point_cloud"][-1]
        assert int(status.get("skipped for max-hz")) > 0 and "skipped for max-hz" in status.message, status.message
        assert status.get("max-hz") == "max 5 Hz"
    finally:
        camera.close()
        session.close()


def test_every_frame_by_default_and_with_none(node):
    """Without --max-hz, and with --max-hz none (or 0): every frame it can take, and the report says so."""
    for extra in ((), ("--max-hz", "none")):
        port = free_port()
        session = open_session(listen=[f"127.0.0.1:{port}"])
        camera = Camera(session, cuda=False)
        node(port, *extra)
        statuses = []
        session.declare_subscriber("t/k/diagnostics", lambda s: statuses.extend(
            DiagnosticArray.deserialize(s.payload.to_bytes()).status))
        try:
            first_cloud(PointCloudSubscriber(session, "t/k/points", "shm"))
            end = time.monotonic() + 5
            while not any(s.get("max-hz") for s in statuses) and time.monotonic() < end:
                time.sleep(0.1)
            status = [s for s in statuses if s.name == "point_cloud"][-1]
            assert status.get("max-hz") == "every frame" and status.get("skipped for max-hz") == "0", extra
        finally:
            camera.close()
            session.close()


@pytest.mark.parametrize("offset_ms", [0, 15])
@pytest.mark.parametrize("max_hz", [30, 20])
def test_max_hz_limits_the_pair(node, max_hz, offset_ms):
    """Colour 15 ms after depth (measured on a camera: 11-17 ms) or at its stamp, a 30 Hz camera, --max-hz 30 and 20:
    only the depth is limited; each depth frame taken finds its colour frame (no pair lost to the limit), and only
    those colour frames are decoded (the diagnostics' colour/s is the clouds/s)."""
    port = free_port()
    session = open_session(listen=[f"127.0.0.1:{port}"])
    camera = Camera(session, cuda=False, color_offset_ns=offset_ms * 1_000_000 or 1)
    node(port, "--color", "t/k/color", "--max-hz", str(max_hz), "--max-offset-ms", "20")
    statuses = []
    session.declare_subscriber("t/k/diagnostics", lambda s: statuses.extend(
        s for s in DiagnosticArray.deserialize(s.payload.to_bytes()).status if s.name == "point_cloud"))
    try:
        sub = PointCloudSubscriber(session, "t/k/points", "shm")
        first_cloud(sub)
        start = len(statuses)
        time.sleep(4.5)
        reports = statuses[start + 1:]  # whole seconds of steady state
        assert len(reports) >= 3
        clouds = sum(float(s.get("clouds/s")) for s in reports)
        colours = sum(float(s.get("colour/s")) for s in reports)
        depths = sum(float(s.get("depth/s")) for s in reports)
        unpaired = sum(int(s.get("unpaired")) for s in reports)
        print(f"max_hz {max_hz}, offset {offset_ms} ms, {len(reports)} s: clouds {clouds:.0f}, depth taken "
              f"{depths:.0f}, colour decoded {colours:.0f}, unpaired {unpaired}")
        assert unpaired <= 1 and abs(colours - clouds) <= 1.5, (unpaired, colours, clouds)
        assert clouds >= 0.8 * min(max_hz, 30) * len(reports)
    finally:
        camera.close()
        session.close()
