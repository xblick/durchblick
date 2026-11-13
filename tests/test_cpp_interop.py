"""C++ and Python against each other, every transport in both directions: the C++ tools (cpp/tools) publish and
receive the test patterns that pattern() and cloud_pattern() compute here.

Needs the C++ build: XBLICK_CPP_BUILD=<CMake build directory with xblick-cpp-camera and xblick-cpp-probe>
(CI sets it; skipped without). The cuda cases need a GPU (CUDA_VISIBLE_DEVICES) and are skipped without one."""

import json
import os
import socket
import subprocess
import time
from pathlib import Path

import numpy as np
import pytest

from xblick import _core, diagnostics, stamps, topics
from xblick.camera import CameraPublisher, ImageSubscriber, query_info
from xblick.msg import CameraInfo, CompressedImage, DiagnosticArray
from xblick.points import XYZRGB, PointCloudPublisher, PointCloudSubscriber, points_dtype
from xblick.session import open_session, replies

BUILD = os.environ.get("XBLICK_CPP_BUILD")
pytestmark = pytest.mark.skipif(not BUILD, reason="XBLICK_CPP_BUILD (the C++ build directory) is not set")
needs_cuda = pytest.mark.skipif(_core.cuda_device_count() == 0, reason="no CUDA device")
W, H = 64, 48


def tool(name: str) -> str:
    path = Path(BUILD) / name
    if not path.exists():
        raise FileNotFoundError(f"{path} does not exist: build the C++ tools (cmake --build {BUILD})")
    return str(path)


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def pattern(encoding: str, seq: int) -> np.ndarray:
    """cpp/tools/tools.hpp: the same frames on both sides."""
    y, x = np.mgrid[0:H, 0:W]
    if encoding == "rgb8":
        return np.stack([(x + 2 * y + 3 * c + seq) & 0xFF for c in range(3)], axis=-1).astype(np.uint8)
    return ((7 * x + 13 * y + 5 * seq) & 0xFFFF).astype(np.uint16)


def checksum(image: np.ndarray) -> int:
    return int(np.frombuffer(np.ascontiguousarray(image).tobytes(), np.uint8).sum(dtype=np.uint64)) & 0xFFFFFFFF


# --------------------------------------------------------------------------------- C++ publishes, Python reads


@pytest.fixture
def cpp_camera():
    started = []

    def start(encoding="rgb8", codec="jpeg", cuda=False, device_frames=False, jpeg_frames=False, points=False,
              diagnostics=False, params=False, extra=()):
        port = free_port()
        start.port = port
        cmd = [tool("xblick-cpp-camera"), "--listen", f"127.0.0.1:{port}", "--base", "t/cpp/color",
               "--encoding", encoding, "--codec", codec, "--size", str(W), str(H), "--fps", "30", "--seconds", "60"]
        if cuda:
            cmd += ["--cuda-device", "0"] + (["--device-frames"] if device_frames else [])
        if jpeg_frames:
            cmd += ["--jpeg-frames"]
        if points:
            cmd += ["--points"] + (["--rays"] if points == "rays" else [])
        if diagnostics:
            cmd += ["--diagnostics"]
        if params:
            cmd += ["--params"]
        cmd += list(extra)
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, text=True)
        started.append(proc)
        assert proc.stdout.readline().strip() == "ready", "the C++ camera did not start"
        return open_session(connect=[f"127.0.0.1:{port}"])

    yield start
    for proc in started:
        proc.terminate()
        proc.wait(10)


def first_frame(sub, timeout=15.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        frame = sub.latest(timeout=0.1)
        if frame is not None:
            return frame
    raise TimeoutError(f"no frame from C++ on {sub.base} ({sub.transport})")


@pytest.mark.parametrize("transport", ["raw", "shm", "auto"])
def test_cpp_to_python_exact(cpp_camera, transport):
    session = cpp_camera()
    sub = ImageSubscriber(session, "t/cpp/color", transport, camera_info=True)
    frame = first_frame(sub)
    assert frame.encoding == "rgb8" and frame.source == "test" and frame.header.frame_id == "TestCamera"
    np.testing.assert_array_equal(frame.image, pattern("rgb8", frame.seq))
    assert 0 <= frame.age_at_send_ns < 1_000_000_000  # the C++ publisher's publish_ns and stamp
    assert sub.transport == ("shm" if transport == "auto" else transport)
    end = time.monotonic() + 5
    while sub.camera_info is None and time.monotonic() < end:
        time.sleep(0.05)
    info = sub.camera_info
    assert isinstance(info, CameraInfo) and info.width == W and info.k[0] == 500.0
    session.close()


@pytest.mark.parametrize("cuda", [False, pytest.param(True, marks=needs_cuda)])
def test_cpp_relay_publisher(cpp_camera, cuda):
    """A relay's publisher (PublisherOptions transports and relayed_from, publish with an Origin): only its rings are
    offered (no raw, compressed or camera_info key gets a sample), its info record says relayed_from, and the frames
    carry the source's seq, source and frame_id."""
    transports = ["shm", "cuda"] if cuda else ["shm"]
    session = cpp_camera(cuda=cuda, device_frames=cuda, extra=["--transports", ",".join(transports),
                                                               "--relayed-from", "the-source-host", "--origin-seq", "1000"])
    records = query_info(session, "t/cpp/color", timeout=2.0)
    assert len(records) == 1
    assert records[0]["transports"] == transports and records[0]["relayed_from"] == "the-source-host"
    leaked = []
    subs = [session.declare_subscriber(f"t/cpp/color/{suffix}", lambda s: leaked.append(str(s.key_expr)))
            for suffix in (topics.IMAGE_RAW, topics.COMPRESSED, topics.CAMERA_INFO)]
    for transport in transports:
        frame = first_frame(ImageSubscriber(session, "t/cpp/color", transport))
        assert frame.seq >= 1000 and frame.source == "origin" and frame.header.frame_id == "OriginCamera"
        image = frame.image.numpy() if transport == "cuda" else frame.image
        np.testing.assert_array_equal(image, pattern("rgb8", frame.seq - 999))  # the tool's frame n: origin n + 999
    time.sleep(0.5)
    assert leaked == []
    for s in subs:
        s.undeclare()
    session.close()


def test_cpp_publish_ns_per_transport(cpp_camera):
    """C++ takes publish_ns right before each transport's put: of one frame, the compressed sample (encoded, put after
    the raw one) is older at send than the raw one; publish_called_ns is the same for both."""
    session = cpp_camera()
    got = {}
    subs = [session.declare_subscriber(f"t/cpp/color/{suffix}", lambda s, t=t: got.setdefault(t, {}).__setitem__(
        json.loads(s.attachment.to_bytes())["seq"], s)) for t, suffix in (("raw", topics.IMAGE_RAW),
                                                                           ("compressed", topics.COMPRESSED))]
    end = time.monotonic() + 15
    while not (set(got.get("raw", {})) & set(got.get("compressed", {}))) and time.monotonic() < end:
        time.sleep(0.05)
    both = sorted(set(got["raw"]) & set(got["compressed"]))
    assert both
    for seq in both:
        raw, compressed = got["raw"][seq], got["compressed"][seq]
        meta = [json.loads(s.attachment.to_bytes()) for s in (raw, compressed)]
        assert meta[0]["publish_called_ns"] == meta[1]["publish_called_ns"] <= meta[0]["publish_ns"]
        assert stamps.age_at_send_ns(compressed) >= stamps.age_at_send_ns(raw) >= 0
    for s in subs:
        s.undeclare()
    session.close()


def test_cpp_to_python_jpeg(cpp_camera):
    session = cpp_camera()
    frame = first_frame(ImageSubscriber(session, "t/cpp/color", "compressed"))
    assert frame.encoding == "rgb8"
    error = np.abs(frame.image.astype(int) - pattern("rgb8", frame.seq)).mean()
    assert error < 4, f"JPEG differs by {error:.1f} on average"
    session.close()


@pytest.mark.parametrize("codec", ["png", "rvl"])
def test_cpp_to_python_depth(cpp_camera, codec):
    session = cpp_camera(encoding="16UC1", codec=codec)
    for transport in ("compressed", "raw", "shm"):
        frame = first_frame(ImageSubscriber(session, "t/cpp/color", transport))
        assert frame.encoding == "16UC1"
        np.testing.assert_array_equal(frame.image, pattern("16UC1", frame.seq))
    session.close()


@needs_cuda
@pytest.mark.parametrize("device_frames", [False, True])
def test_cpp_to_python_cuda(cpp_camera, device_frames):
    session = cpp_camera(cuda=True, device_frames=device_frames)
    frame = first_frame(ImageSubscriber(session, "t/cpp/color", "cuda"))
    np.testing.assert_array_equal(frame.image.numpy(), pattern("rgb8", frame.seq))
    if device_frames:  # the CPU representations come from the publisher's download
        frame = first_frame(ImageSubscriber(session, "t/cpp/color", "shm"))
        np.testing.assert_array_equal(frame.image, pattern("rgb8", frame.seq))
    session.close()


def compressed_messages(session, base: str) -> list:
    """The compressed messages as they arrive (not decoded): to see which JPEG bytes were sent."""
    received = []
    session.declare_subscriber(topics.compressed(base),
                               lambda sample: received.append(CompressedImage.deserialize(sample.payload.to_bytes())))
    return received


def wait_for(received: list, timeout=15.0):
    end = time.monotonic() + timeout
    while not received and time.monotonic() < end:
        time.sleep(0.02)
    assert received, "no compressed message"
    return received[-1]


def test_cpp_jpeg_frames_pass_through(cpp_camera):
    """An MJPEG camera's JPEG goes out as it is (the tool marks it with a comment segment), without pixel subscribers
    (publish_jpeg) and with them (publish with the JPEG; the pixels are the decoded frame, here the pattern)."""
    session = cpp_camera(jpeg_frames=True)
    received = compressed_messages(session, "t/cpp/color")
    message = wait_for(received)
    assert b"xblick-test-jpeg " in bytes(message.data) and message.format == "rgb8; jpeg compressed bgr8"
    frame = first_frame(ImageSubscriber(session, "t/cpp/color", "raw"))  # now somebody takes the pixels
    np.testing.assert_array_equal(frame.image, pattern("rgb8", frame.seq))
    received.clear()
    message = wait_for(received)
    assert b"xblick-test-jpeg " in bytes(message.data)
    sub = ImageSubscriber(session, "t/cpp/color", "compressed")
    decoded = first_frame(sub)
    assert np.abs(decoded.image.astype(int) - pattern("rgb8", decoded.seq)).mean() < 4
    session.close()


@needs_cuda
def test_cpp_jpeg_frames_with_device_frames(cpp_camera):
    session = cpp_camera(cuda=True, device_frames=True, jpeg_frames=True)
    received = compressed_messages(session, "t/cpp/color")
    frame = first_frame(ImageSubscriber(session, "t/cpp/color", "cuda"))
    np.testing.assert_array_equal(frame.image.numpy(), pattern("rgb8", frame.seq))
    assert b"xblick-test-jpeg " in bytes(wait_for(received).data)
    session.close()


def test_cpp_diagnostics_name_their_session(cpp_camera):
    """A C++ DiagnosticsPublisher's samples carry the session's zid, in Zenoh's own format (the admin space's, Python's
    str of a ZenohId); the message is the same for a reader that ignores the attachment (before 0.3.29)."""
    session = cpp_camera(diagnostics=True)
    samples = []
    subscriber = session.declare_subscriber("t/cpp/diagnostics", samples.append)
    end = time.monotonic() + 10
    while not samples and time.monotonic() < end:
        time.sleep(0.05)
    assert samples, "no diagnostics from C++"
    camera, array = diagnostics.read(samples[0])
    assert camera == "t/cpp" and [s.name for s in array.status] == ["cpp_camera"]
    assert DiagnosticArray.deserialize(samples[0].payload.to_bytes()) == array
    peers = [str(z) for z in session.info.peers_zid()]  # the C++ camera's session alone
    assert len(peers) == 1 and diagnostics.sender(samples[0]) == peers[0]
    meta = json.loads(samples[0].attachment.to_bytes())
    assert meta["v"] == 1 and meta["source"] == "" and meta["seq"] >= 1
    subscriber.undeclare()
    session.close()


# --------------------------------------------------------------------------------- Python publishes, C++ reads


def probe(port: int, transport: str, frames: int = 3, *extra, timeout: float = 20) -> list[dict]:
    proc = subprocess.Popen([tool("xblick-cpp-probe"), "--connect", f"127.0.0.1:{port}", "--base", "t/py/color",
                             "--transport", transport, "--frames", str(frames), "--timeout", f"{timeout:g}", *extra],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    return proc


def run_python_camera(port, proc, encoding="rgb8", codec="png", cuda=False):
    session = open_session(listen=[f"127.0.0.1:{port}"])
    info = CameraInfo.pinhole(W, H, 500.0, 500.0, (W - 1) / 2, (H - 1) / 2, frame_id="PyCamera")
    pub = CameraPublisher(session, "t/py/color", encoding=encoding, frame_id="PyCamera", source="py", codec=codec,
                          camera_info=info, cuda_device=0 if cuda else None)
    end = time.monotonic() + 30
    try:
        while proc.poll() is None and time.monotonic() < end:
            if pub.subscribed:
                pub.publish(pattern(encoding, pub.seq + 1))  # the frame publish() is about to number
            time.sleep(1 / 30)
        out, err = proc.communicate(timeout=10)
    finally:
        pub.close()
        session.close()
    assert proc.returncode == 0, f"C++ probe failed ({proc.returncode}): {err}"
    return [json.loads(line) for line in out.splitlines()]


@pytest.mark.parametrize("transport,expected", [("raw", "raw"), ("compressed", "compressed"), ("shm", "shm"),
                                                ("auto", "shm")])
def test_python_to_cpp(transport, expected):
    port = free_port()
    records = run_python_camera(port, probe(port, transport, 3, "--camera-info"))
    assert len(records) == 3
    for r in records:
        assert r["transport"] == expected and r["encoding"] == "rgb8" and r["source"] == "py"
        assert r["checksum"] == checksum(pattern("rgb8", r["seq"])), r
    assert any(r["camera_info"] == {"width": W, "fx": 500.0} for r in records)


@pytest.mark.parametrize("transport", ["compressed", "shm"])
def test_python_to_cpp_max_hz_and_age_at_send(transport):
    """C++ with max_hz 5 takes at most a frame per 200 ms period of the stamps, the newest; the age at send it
    reads is the Python publisher's (frames stamped 30 ms before they are published)."""
    port = free_port()
    proc = probe(port, transport, 1000, "--max-hz", "5", timeout=4)  # more than come: it ends at its timeout
    session = open_session(listen=[f"127.0.0.1:{port}"])
    pub = CameraPublisher(session, "t/py/color", encoding="rgb8", frame_id="PyCamera", source="py", codec="png")
    end = time.monotonic() + 30
    try:
        while proc.poll() is None and time.monotonic() < end:
            if pub.subscribed:
                pub.publish(pattern("rgb8", pub.seq + 1), stamp_ns=time.time_ns() - 30_000_000)
            time.sleep(1 / 30)
        out, err = proc.communicate(timeout=10)
    finally:
        pub.close()
        session.close()
    records = [json.loads(line) for line in out.splitlines()]
    assert proc.returncode == 2 and "skipped for max-hz" in err, err  # fewer frames than asked for: the limit
    stamps = [r["stamp_ns"] for r in records]
    # a frame per period of 200 ms of the stamps (periods from the epoch: the first may come early in its period)
    assert len(records) >= 5 and len({s // 200_000_000 for s in stamps}) == len(stamps), np.diff(stamps)
    assert all(30_000_000 <= r["age_at_send_ns"] < 200_000_000 for r in records), records
    skipped = int(err.rsplit("skipped for max-hz", 1)[1].split()[0])
    assert skipped >= 2 * len(records), err


@pytest.mark.parametrize("codec", ["png", "rvl"])
def test_python_to_cpp_depth(codec):
    port = free_port()
    for r in run_python_camera(port, probe(port, "compressed"), encoding="16UC1", codec=codec):
        assert r["encoding"] == "16UC1" and r["checksum"] == checksum(pattern("16UC1", r["seq"]))


def test_python_to_cpp_shm_view():
    port = free_port()
    for r in run_python_camera(port, probe(port, "shm", 3, "--view")):
        assert r["valid"] and r["checksum"] == checksum(pattern("rgb8", r["seq"]))


@needs_cuda
def test_python_to_cpp_cuda():
    port = free_port()
    for r in run_python_camera(port, probe(port, "cuda"), cuda=True):
        assert r["transport"] == "cuda" and r["checksum"] == checksum(pattern("rgb8", r["seq"]))


# ------------------------------------------------------------------------------------- point clouds, both ways


def cloud_pattern(seq: int) -> np.ndarray:
    """cpp/tools/tools.hpp cloud_pattern: point i, byte k = (7i + 3k + seq) mod 256 (layout xyzrgb, 16 bytes)."""
    i = np.arange(W * H, dtype=np.uint64)[:, None]
    k = np.arange(16, dtype=np.uint64)[None, :]
    return ((7 * i + 3 * k + seq) & 0xFF).astype(np.uint8).reshape(-1).view(points_dtype(*XYZRGB)).reshape(H, W)


@pytest.mark.parametrize("transport", ["raw", "shm", "auto"])
def test_cpp_to_python_points(cpp_camera, transport):
    session = cpp_camera(points=True)
    sub = PointCloudSubscriber(session, "t/cpp/color", transport)
    got = first_frame(sub)
    assert got.points.tobytes() == cloud_pattern(got.seq).tobytes()
    assert [f.name for f in got.fields] == ["x", "y", "z", "rgb"] and got.header.frame_id == "TestCamera"
    assert sub.transport == ("shm" if transport == "auto" else transport)
    session.close()


@needs_cuda
@pytest.mark.parametrize("device_frames", [False, True])
def test_cpp_to_python_points_cuda(cpp_camera, device_frames):
    session = cpp_camera(cuda=True, device_frames=device_frames, points=True)
    sub = PointCloudSubscriber(session, "t/cpp/color", local="cuda")
    got = first_frame(sub)
    assert sub.transport == "cuda" and got.numpy().tobytes() == cloud_pattern(got.seq).tobytes()
    if device_frames:  # the host representations come from the publisher's download
        got = first_frame(PointCloudSubscriber(session, "t/cpp/color", "shm"))
        assert got.points.tobytes() == cloud_pattern(got.seq).tobytes()
    session.close()


def ray_cloud_rays() -> np.ndarray:
    """cpp/tools/tools.hpp ray_cloud_rays: a pinhole camera's rays."""
    y, x = np.mgrid[0:H, 0:W].astype(np.float32)
    return np.stack([(x - np.float32(W) / 2) / np.float32(60), (y - np.float32(H) / 2) / np.float32(60),
                     np.ones_like(x)], -1)


def ray_cloud_pattern(seq: int) -> np.ndarray:
    """cpp/tools/tools.hpp ray_cloud_pattern: a cloud on ray_cloud_rays, computed the same way."""
    y, x = np.mgrid[0:H, 0:W]
    z = (800 + 3 * x + 5 * y + seq % 100).astype(np.float32) * np.float32(0.001)
    z[(x + 2 * y + seq) % 11 == 0] = np.nan
    points = np.zeros((H, W), points_dtype(*XYZRGB))
    rays = ray_cloud_rays()
    points["x"], points["y"], points["z"] = rays[..., 0] * z, rays[..., 1] * z, rays[..., 2] * z
    points["rgb"] = (((x + y) & 0xFF) << 16 | (y & 0xFF) << 8 | ((x + seq) & 0xFF)).astype(np.uint32).view(np.float32)
    return points


def same_cloud(got: np.ndarray, expected: np.ndarray):
    """The same points: equal numbers, NaN at the same places (whatever bits a NaN has), equal colours."""
    for name in "xyz":
        np.testing.assert_array_equal(got[name], expected[name])
    assert got["rgb"].tobytes() == expected["rgb"].tobytes()


def test_cpp_to_python_points_compressed(cpp_camera):
    """C++'s cloud/compressed, read by Python: the rays asked once, the cloud exactly the C++ one (millimetre
    ranges, PNG colours)."""
    session = cpp_camera(points="rays")
    sub = PointCloudSubscriber(session, "t/cpp/color", "compressed")
    for _ in range(3):
        got = first_frame(sub)
        same_cloud(got.points, ray_cloud_pattern(got.seq))
    assert sub.transport == "compressed" and got.key == "t/cpp/color/cloud/compressed"
    assert got.header.frame_id == "TestCamera" and not got.is_dense
    other = PointCloudSubscriber(session, "t/cpp/color", "shm")  # the same cloud whole
    whole = first_frame(other)
    same_cloud(whole.points, ray_cloud_pattern(whole.seq))
    session.close()


def test_python_to_cpp_points_compressed():
    """Python's cloud/compressed, read by C++."""
    port = free_port()
    records = run_python_points(port, probe(port, "compressed", 3, "--points"), rays=True)
    assert len(records) == 3
    for r in records:
        assert r["transport"] == "compressed" and r["fields"] == ["x", "y", "z", "rgb"] and r["point_step"] == 16
        assert (r["height"], r["width"], r["frame_id"], r["source"]) == (H, W, "PyCamera", "py")
        assert r["checksum"] == checksum(ray_cloud_pattern(r["seq"])), r


def run_python_points(port, proc, cuda=False, rays=False):
    session = open_session(listen=[f"127.0.0.1:{port}"])
    pub = PointCloudPublisher(session, "t/py/color", fields=XYZRGB[0], point_step=16, frame_id="PyCamera",
                              source="py", cuda_device=0 if cuda else None,
                              rays=ray_cloud_rays() if rays else None, color_codec="png")
    end = time.monotonic() + 30
    try:
        while proc.poll() is None and time.monotonic() < end:
            if pub.subscribed:
                pattern_of = ray_cloud_pattern if rays else cloud_pattern
                pub.publish(pattern_of(pub.seq + 1))  # the cloud publish() is about to number
            time.sleep(1 / 30)
        out, err = proc.communicate(timeout=10)
    finally:
        pub.close()
        session.close()
    assert proc.returncode == 0, f"C++ probe failed ({proc.returncode}): {err}"
    return [json.loads(line) for line in out.splitlines()]


@pytest.mark.parametrize("transport,expected,extra", [("raw", "raw", ()), ("shm", "shm", ()), ("auto", "shm", ()),
                                                      ("shm", "shm", ("--view",))])
def test_python_to_cpp_points(transport, expected, extra):
    port = free_port()
    records = run_python_points(port, probe(port, transport, 3, "--points", *extra))
    assert len(records) == 3
    for r in records:
        assert r["transport"] == expected and r["fields"] == ["x", "y", "z", "rgb"] and r["point_step"] == 16
        assert (r["height"], r["width"], r["frame_id"], r["source"]) == (H, W, "PyCamera", "py") and r["valid"]
        assert r["checksum"] == checksum(cloud_pattern(r["seq"])), r


@needs_cuda
def test_python_to_cpp_points_cuda():
    port = free_port()
    for r in run_python_points(port, probe(port, "auto", 3, "--points", "--local", "cuda"), cuda=True):
        assert r["transport"] == "cuda" and r["checksum"] == checksum(cloud_pattern(r["seq"])), r


# ------------------------------------------------------------------------------------------------- parameters


def test_cpp_parameters_from_python(cpp_camera):
    """The C++ camera's parameters (Parameters, --params) through Python's client: descriptors and values, checks in
    the same words, a writer's refusal, a write changing another parameter in the event, read-only."""
    from xblick.parameters import RESTART, ParameterClient, find_nodes

    session = cpp_camera(params=True)
    end = time.monotonic() + 10
    while "t/cpp/cpp_camera" not in find_nodes(session, 0.3) and time.monotonic() < end:
        time.sleep(0.1)
    client = ParameterClient(session, "t/cpp/cpp_camera")
    events = []
    client.on_event(events.append)
    descriptors, values = client.all()
    by = {d.name: (d, v.value()) for d, v in zip(descriptors, values)}
    assert sorted(by) == ["codec", "fps", "pattern.auto_shift", "pattern.shift"]
    assert by["fps"][1] == 30.0 and by["fps"][0].additional_constraints == "unit: Hz"
    assert by["codec"][0].read_only and by["codec"][0].additional_constraints == "choices: jpeg, png, rvl"
    assert by["pattern.shift"][0].integer_range[0].to_value == 255
    results = client.set({"fps": 47.25, "pattern.shift": 13, "codec": "png", "pattern.auto_shift": "no"})
    assert [r.reason for r in results] == ["47.25 is not 1 plus a multiple of 0.5", "the camera refused 13", RESTART,
                                           "string given, the parameter is bool"]
    assert client.set({"pattern.shift": 40})[0].successful
    end = time.monotonic() + 5
    while not any(e.changed_parameters for e in events) and time.monotonic() < end:
        time.sleep(0.05)
    changed = {p.name: p.value.value() for e in events for p in e.changed_parameters}
    assert changed == {"pattern.shift": 40, "pattern.auto_shift": False}
    assert client.set_atomically({"fps": 10.0, "pattern.shift": 300}).reason == "pattern.shift: 300 is outside 0 to 255"
    assert client.get(["fps"])[0].value() == 30.0  # atomically: nothing taken
    client.close()
    session.close()


def test_python_parameters_from_cpp():
    """Python's Parameters through the C++ client (xblick-cpp-probe --params): the same descriptors, values and
    checks."""
    from xblick.parameters import Parameters

    port = free_port()
    session = open_session(listen=[f"127.0.0.1:{port}"])
    params = Parameters(session, "boxer", "t/py")
    params.declare("threshold", 0.3, range=(0.0, 1.0, 0.05), unit="1")
    params.declare("mode", "fast", choices=["fast", "accurate"])
    params.declare("labels", ["cup", "a, b"])
    try:
        def probe_params(*extra):
            out = subprocess.run([tool("xblick-cpp-probe"), "--connect", f"127.0.0.1:{port}", "--params",
                                  "t/py/boxer", *extra], capture_output=True, text=True, timeout=30, check=True)
            return [json.loads(line) for line in out.stdout.splitlines()]

        lines = probe_params("--set", "threshold:=0.45")
        assert lines[0] == {"set": "threshold", "successful": True, "reason": ""}
        assert {p["name"]: p["value"] for p in lines[1:]} == {"labels": '[cup, "a, b"]', "mode": "fast",
                                                             "threshold": "0.45"}
        assert probe_params("--set", "threshold:=0.47")[0]["reason"] == "0.47 is not 0 plus a multiple of 0.05"
        assert probe_params("--set", "mode:=slow")[0]["reason"] == "slow is none of fast, accurate"
        assert params["threshold"] == 0.45
    finally:
        params.close()
        session.close()



def test_cpp_auto_refuses_a_remote_source_without_its_form(cpp_camera):
    """A publisher that offers only raw (as the Jetson's ecam24 node in raw mode), not on this host's rings: C++'s
    auto says so (ProtocolError) instead of waiting for compressed frames that never come, as Python does."""
    cpp_camera(extra=["--transports", "raw"])
    proc = subprocess.run([tool("xblick-cpp-probe"), "--connect", f"127.0.0.1:{cpp_camera.port}", "--base",
                           "t/cpp/color", "--transport", "auto", "--frames", "1", "--timeout", "10"],
                          capture_output=True, text=True, timeout=30)
    assert proc.returncode != 0 and "offers no compressed transport" in proc.stderr, (proc.returncode, proc.stderr)


@pytest.mark.parametrize("lag", [0, 1, 2])
def test_cpp_auto_moves_to_a_ring_that_appears_and_back(monkeypatch, lag):
    """The C++ subscriber keeps its transport up to date as the Python one does (test_camera.py): it reads a stream of
    another host as compressed, moves to the ring of a publisher that starts later on its own host (a relay), and back
    when that one stops; every frame once, also from a relay that republishes a frame ``lag`` frames late."""
    from xblick import _core

    port = free_port()
    proc = probe(port, "auto", 150, timeout=40)
    session = open_session(listen=[f"127.0.0.1:{port}"])
    with monkeypatch.context() as patched:  # as a publisher on another PC is seen from here
        patched.setattr(_core, "host_id", lambda: "0c9a1f6e-5b0d-4a52-9d3e-7f1b2c3d4e5f")
        source = CameraPublisher(session, "t/py/color", encoding="rgb8", frame_id="PyCamera", source="py", codec="png")
    relay, stamp, wanted = None, 0, {}
    end = time.monotonic() + 45
    try:
        while proc.poll() is None and time.monotonic() < end:
            if source.subscribed or relay is not None:
                stamp += 1
                source.publish(pattern("rgb8", stamp), stamp_ns=stamp * 1_000_000)
                if relay is not None and stamp - lag > 45:  # created at 45
                    relay.publish(pattern("rgb8", stamp - lag), stamp_ns=(stamp - lag) * 1_000_000)
                if stamp in (40, 100):  # the probe's want token: while no source has a ring on this host
                    wanted[stamp] = [r for r in replies(session, topics.wanted_on(_core.host_id()), 1.0,
                                                        liveliness=True) if r.ok is not None]
                if stamp == 45:  # only rings and raw, as a relay: nothing of it on the compressed key
                    relay = CameraPublisher(session, "t/py/color", encoding="rgb8", frame_id="PyCamera", source="relay",
                                            codec=None)
                elif stamp == 105:
                    relay.close()
                    relay = None
            time.sleep(1 / 30)
        out, err = proc.communicate(timeout=10)
    finally:
        if relay is not None:
            relay.close()
        source.close()
        session.close()
    assert proc.returncode == 0, f"C++ probe failed ({proc.returncode}): {err}"
    records = [json.loads(line) for line in out.splitlines()]
    stamps = [r["stamp_ns"] for r in records]
    assert stamps == sorted(set(stamps)), "a frame twice, or out of order"
    routes = [r["transport"] for i, r in enumerate(records) if i == 0 or records[i - 1]["transport"] != r["transport"]]
    assert routes == ["compressed", "shm", "compressed"], (routes, err)
    # asked this host's relay for the stream; not while a source (the stand-in here is one) had a ring on this host
    assert len(wanted[40]) == 1 and wanted[100] == []
    for r in records:
        assert r["checksum"] == checksum(pattern("rgb8", r["stamp_ns"] // 1_000_000)), r
    first_ring = next(i for i, r in enumerate(records) if r["transport"] == "shm")
    around = stamps[max(0, first_ring - 10):first_ring + 10]  # nothing lost on the way to the ring
    assert around == list(range(around[0], around[-1] + 1_000_000, 1_000_000)), around


def test_cpp_auto_closes_during_a_move(monkeypatch):
    """A C++ subscriber destroyed while it waits for the first frame of the ring it moves to (a relay that sends
    nothing) ends the handover at once instead of after its 2 s: the probe stops right after its last frame. The
    source holds back the probe's last frames until the probe has begun to move (it declared its ring subscriber)."""
    frames, last_ones = 30, 3
    port = free_port()
    proc = probe(port, "auto", frames, timeout=30)
    session = open_session(listen=[f"127.0.0.1:{port}"])
    with monkeypatch.context() as patched:  # as a publisher on another PC is seen from here
        patched.setattr(_core, "host_id", lambda: "0c9a1f6e-5b0d-4a52-9d3e-7f1b2c3d4e5f")
        source = CameraPublisher(session, "t/py/color", encoding="rgb8", frame_id="PyCamera", source="py", codec="png")
    relay, stamp, moving_since, last = None, 0, None, None
    moving_deadline = time.monotonic() + 40  # set when the relay starts
    end = time.monotonic() + 30
    try:
        while proc.poll() is None and time.monotonic() < end:
            if relay is not None and moving_since is None and relay.subscribed:
                moving_since = time.monotonic()  # the probe declared its ring subscriber: it is in the handover
            if stamp == frames - last_ones and moving_since is None:
                assert time.monotonic() < moving_deadline, "the probe did not begin to move to the relay's ring"
            elif source.subscribed and stamp < frames:
                stamp += 1
                source.publish(pattern("rgb8", stamp), stamp_ns=stamp * 1_000_000)
                if stamp == 20:
                    relay = CameraPublisher(session, "t/py/color", encoding="rgb8", frame_id="PyCamera",
                                            source="relay", codec=None)  # never publishes
                    moving_deadline = time.monotonic() + 10
                if stamp == frames:  # the probe's last frame
                    last = time.monotonic()
            time.sleep(1 / 30)
        out, err = proc.communicate(timeout=10)
        stopped = time.monotonic()
    finally:
        if relay is not None:
            relay.close()
        source.close()
        session.close()
    assert proc.returncode == 0, f"C++ probe failed ({proc.returncode}): {err}"
    assert last - moving_since < 1.0, f"the probe's last frame came {last - moving_since:.2f} s into the handover"
    assert stopped - last < 1.0, f"the probe stopped {stopped - last:.2f} s after its last frame"
    stamps = [json.loads(line)["stamp_ns"] for line in out.splitlines()]
    assert len(stamps) == frames and stamps == sorted(set(stamps)), stamps  # by the network, the ring sent nothing
