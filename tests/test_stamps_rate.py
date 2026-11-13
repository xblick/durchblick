"""The age at send (stamps.py, Frame.age_at_send_ns) on every transport and for results, and --max-hz
(camera.MaxRate, the subscriptions' max_hz, RGBDInputs, node.Reporter.watch): at most so many frames a second, the
newest, the others not decoded."""

import argparse
import threading
import time

import numpy as np
import pytest
from test_camera import pattern, sessions, wait_until  # noqa: F401 - sessions is a fixture

from blick import camera as camera_module
from blick import node, stamps
from blick.camera import CameraPublisher, ImageSubscriber, MaxRate
from blick.detections import DetectionPublisher
from blick.diagnostics import DiagnosticsPublisher
from blick.msg import (CameraInfo, DiagnosticArray, Detection2DArray, Header, ShmImage, Time)
from blick.points import XYZRGB, PointCloudPublisher, PointCloudSubscriber, points_dtype
from blick.rgbd import CameraSpec, RGBDInputs

BASE = "robot/x/color"
MS = 1_000_000


def publish_frames(pub, fps: float, seconds: float, stamp_of=time.time_ns, before_ns: int = 0, stop=None):
    """Frames at ``fps`` for ``seconds`` (or until ``stop`` is set), each stamped ``before_ns`` before it is
    published."""
    end = time.monotonic() + seconds
    seq = 0
    while time.monotonic() < end and not (stop is not None and stop.is_set()):
        seq += 1
        pub.publish(pattern(seq), stamp_ns=stamp_of() - before_ns)
        time.sleep(1.0 / fps)


# ----------------------------------------------------------------------------------------------- age at send


def _key(transport):
    """The key of BASE's transport: a ring's names this host."""
    topic = camera_module.TRANSPORTS[transport]
    return topic(BASE, camera_module._core.host_id()) if transport == "shm" else topic(BASE)


@pytest.mark.parametrize("transport", ["compressed", "raw", "shm"])
def test_age_at_send_on_every_transport(sessions, transport):  # noqa: F811
    a, b = sessions
    pub = CameraPublisher(a, BASE, encoding="rgb8", frame_id="cam")
    sub = ImageSubscriber(b, BASE, transport=transport)
    samples = []
    raw = b.declare_subscriber(_key(transport), samples.append)
    wait_until(lambda: pub.subscribed)
    frame = None
    end = time.monotonic() + 5
    while frame is None and time.monotonic() < end:
        pub.publish(pattern(1), stamp_ns=time.time_ns() - 30 * MS)  # captured 30 ms before
        frame = sub.latest(timeout=0.2)
    assert frame is not None and frame.publish_ns is not None
    assert 30 * MS <= frame.age_at_send_ns < 80 * MS
    wait_until(lambda: samples)
    # the sample alone tells the same: the descriptor of a ring carries the frame's stamp and publish_ns too
    ages = [stamps.age_at_send_ns(s) for s in samples]
    assert all(age is not None and 30 * MS <= age < 80 * MS for age in ages)
    raw.undeclare()
    sub.close()
    pub.close()


def test_ring_descriptors_carry_the_stamp():
    message = ShmImage(header=Header(Time(1_700_000_000, 5), "cam"), height=1, width=1, encoding="mono8",
                       is_bigendian=0, step=1, host="h", segment="s", slot=0, seq=1)
    payload = bytes(message.serialize())
    assert stamps.header_stamp("k", ShmImage.ENCODING, payload) == 1_700_000_000_000_000_005


def test_age_at_send_of_results_diagnostics_and_clouds(sessions):  # noqa: F811
    a, b = sessions
    got = {}
    subs = [b.declare_subscriber(k, lambda s, k=k: got.setdefault(k, s))
            for k in ("robot/x/node/boxes", "robot/x/diagnostics", f"{BASE}/plain")]
    boxes = DetectionPublisher(a, "robot/x/node/boxes", Detection2DArray)
    diagnostics = DiagnosticsPublisher(a, "robot/x", "node")
    time.sleep(0.3)
    boxes.publish(Detection2DArray(header=Header(Time.from_ns(time.time_ns() - 40 * MS), "cam"), detections=[]))
    diagnostics.publish(DiagnosticsPublisher.OK, "fine", {})
    a.put(f"{BASE}/plain", bytes(DiagnosticArray(Header(Time(1, 0), ""), []).serialize()),
          encoding=camera_module.cdr_encoding(DiagnosticArray))  # no attachment: no age
    wait_until(lambda: len(got) == 3)
    assert 40 * MS <= stamps.age_at_send_ns(got["robot/x/node/boxes"]) < 90 * MS
    assert 0 <= stamps.age_at_send_ns(got["robot/x/diagnostics"]) < 50 * MS
    assert stamps.age_at_send_ns(got[f"{BASE}/plain"]) is None
    for s in subs:
        s.undeclare()
    boxes.close()
    diagnostics.close()

    cloud_pub = PointCloudPublisher(a, "robot/x/points", fields=XYZRGB[0], point_step=16, frame_id="cam")
    cloud_sub = PointCloudSubscriber(b, "robot/x/points", transport="shm")
    wait_until(lambda: cloud_pub.subscribed)
    points = np.zeros((2, 3), points_dtype(XYZRGB[0], 16))
    cloud = None
    end = time.monotonic() + 5
    while cloud is None and time.monotonic() < end:
        cloud_pub.publish(points, stamp_ns=time.time_ns() - 20 * MS)
        cloud = cloud_sub.latest(timeout=0.2)
    assert cloud is not None and 20 * MS <= cloud.age_at_send_ns < 70 * MS
    cloud_sub.close()
    cloud_pub.close()


# ------------------------------------------------------------------------------------------------------ max-hz


def test_max_rate_by_stamps():
    period = 1_000_000_000 // 30
    stamps_30hz = [1_800_000_000_000_000_000 + i * period for i in range(60)]  # 2 s at 30 Hz
    rate = MaxRate(5)
    taken = [s for s in stamps_30hz if rate.admit(s)]
    assert len(taken) in (10, 11)
    # another stream of the same camera, starting a frame later, takes the same frames from its first on
    other = MaxRate(5)
    taken_other = [s for s in stamps_30hz[1:] if other.admit(s)]
    assert set(taken_other[1:]) <= set(taken)
    # a stamp that goes back (a restart) is taken
    assert rate.admit(stamps_30hz[0])
    with pytest.raises(ValueError, match="above 0"):
        MaxRate(0)


@pytest.mark.parametrize("transport", ["compressed", "shm"])
def test_max_hz_takes_the_newest_and_decodes_no_other(sessions, monkeypatch, transport):  # noqa: F811
    a, b = sessions
    decoded = []
    real_decompress, real_read_ring = camera_module.decompress, camera_module.Subscription._read_ring
    monkeypatch.setattr(camera_module, "decompress", lambda m: decoded.append(1) or real_decompress(m))
    monkeypatch.setattr(camera_module.Subscription, "_read_ring",
                        lambda self, *args: decoded.append(1) or real_read_ring(self, *args))
    pub = CameraPublisher(a, BASE, encoding="rgb8", frame_id="cam")
    sub = ImageSubscriber(b, BASE, transport=transport, max_hz=5)
    wait_until(lambda: pub.subscribed)
    stop = threading.Event()
    publisher = threading.Thread(target=publish_frames, args=(pub, 30, 10.0), kwargs={"stop": stop})
    publisher.start()
    frames, newest_gap = [], []
    start = time.monotonic()
    while time.monotonic() - start < 2.0:
        frame = sub.latest(timeout=0.5)
        if frame is not None:
            frames.append(frame)
            newest_gap.append(time.time_ns() - frame.publish_ns)  # taken as it came: the newest there was
    stop.set()
    publisher.join()
    assert 8 <= len(frames) <= 12, len(frames)
    assert len({f.stamp_ns // (200 * MS) for f in frames}) == len(frames)  # one per 200 ms period of the stamps
    assert sub.skipped_max_hz >= 30, sub.skipped_max_hz
    assert len(decoded) == len(frames)  # skipped frames were neither decompressed nor read from the ring
    assert np.median(newest_gap) < 40 * MS
    assert "at most 5 Hz" in sub.describe()
    sub.close()
    pub.close()


@pytest.mark.parametrize("offset_ms", [0, 15])
@pytest.mark.parametrize("max_hz", [30, 20])
@pytest.mark.parametrize("max_offset_ms", [20, None])  # None: half the depth's period, measured
def test_rgbd_inputs_limit_the_pair(sessions, max_hz, offset_ms, max_offset_ms):  # noqa: F811
    """A 30 Hz camera whose depth is stamped 15 ms after its colour (measured on a camera: 11-17 ms), or as its colour:
    with max_hz only the colour is limited, each colour frame taken finds its depth (none lost to the limit), and the
    only depth frames decoded are those of the colour frames taken."""
    a, b = sessions
    info = CameraInfo.pinhole(64, 48, 50.0, 50.0, 31.5, 23.5, frame_id="cam")
    color = CameraPublisher(a, "robot/x/color", encoding="rgb8", frame_id="cam", camera_info=info)
    depth = CameraPublisher(a, "robot/x/depth", encoding="16UC1", frame_id="cam", camera_info=info, codec="png")
    wake = threading.Event()
    inputs = RGBDInputs(b, CameraSpec("robot/x"), 30, "shm", wake, max_hz=max_hz,
                        max_offset_ns=None if max_offset_ms is None else max_offset_ms * MS)
    wait_until(lambda: color.subscribed and depth.subscribed)
    stop = threading.Event()

    def camera():
        seq = 0
        while not stop.is_set():
            seq += 1
            stamp = time.time_ns()
            color.publish(pattern(seq), stamp_ns=stamp)
            depth.publish(np.full((48, 64), seq, np.uint16), stamp_ns=stamp + offset_ms * MS)
            time.sleep(1 / 30)

    thread = threading.Thread(target=camera)
    thread.start()
    processed, last, sent = 0, None, 0
    end = time.monotonic() + 3.0
    while time.monotonic() < end:  # a node's loop: the newest complete colour frame, its depth
        wake.wait(0.05)
        wake.clear()
        inputs.check()
        stamp = inputs.newest_complete()
        if stamp is None or stamp == last:
            continue
        last = stamp
        frame = inputs.histories["depth"].at(stamp)
        assert frame is not None and abs(frame.stamp_ns - stamp - offset_ms * MS) < MS
        processed += 1
    stop.set()
    thread.join()
    time.sleep(0.2)
    color_sub, depth_sub = inputs.readers["color"].subscriber, inputs.readers["depth"].subscriber
    # each colour frame taken, paired with its depth by the node (the last may have waited for its depth at the end)
    unpaired = color_sub.decoded - processed
    print(f"max_hz {max_hz}, offset {offset_ms} ms: colour taken {color_sub.decoded}, skipped "
          f"{color_sub.skipped_max_hz}; depth received {depth_sub.received}, decoded {depth_sub.decoded}; node "
          f"paired {processed}; unpaired {unpaired}")
    assert color_sub.decoded >= 2.5 * max_hz * 0.8, color_sub.decoded
    # none lost to the limit; measuring the depth's period (auto) takes two depth frames, the first colour frame
    # comes before: one more
    assert unpaired <= (2 if max_offset_ms is None else 1), unpaired
    assert depth_sub.decoded == processed  # only the depth of the colour frames the node took
    if max_hz < 30:
        assert color_sub.skipped_max_hz > 0 and depth_sub.received > 1.3 * depth_sub.decoded
    assert depth_sub.skipped_max_hz == 0  # the partner is not limited: it is read at the colour's stamps
    inputs.close()
    color.close()
    depth.close()


def test_reports_count_the_skips(sessions):  # noqa: F811
    a, b = sessions
    color = CameraPublisher(a, "robot/x/color", encoding="rgb8", frame_id="cam")
    wake = threading.Event()
    inputs = RGBDInputs(b, CameraSpec("robot/x"), 30, "shm", wake, max_hz=5)
    wait_until(lambda: color.subscribed)
    end = time.monotonic() + 1.5
    seq = 0
    while time.monotonic() < end:
        seq += 1
        color.publish(pattern(seq))
        time.sleep(1 / 30)
    time.sleep(0.2)
    skipped = sum(s.skipped_max_hz for s in node._subscriptions(inputs))
    assert skipped >= 20
    report = node.Reporter(a, "robot/x", "node", "durchblick-node", every=0.0)
    report.watch(inputs)
    got = []
    sub = b.declare_subscriber("robot/x/diagnostics", lambda s: got.append(DiagnosticArray.deserialize(
        s.payload.to_bytes())))
    time.sleep(0.3)
    report.publish(*node.state("5 frames/s"), {"frames/s": 5.0})
    wait_until(lambda: got)
    values = {kv.key: kv.value for kv in got[0].status[0].values}
    assert int(values["skipped for max-hz"]) == skipped and "skipped for max-hz" in got[0].status[0].message
    sub.undeclare()
    report.close()
    inputs.close()
    color.close()


def test_node_arguments(capsys):
    parser = argparse.ArgumentParser()
    node.add_arguments(parser)
    assert parser.parse_args([]).max_hz is None  # without a default of the node's: every frame
    assert parser.parse_args(["--max-hz", "2.5"]).max_hz == 2.5
    # a node's own default, and always the way back to every frame it can take: 0 or none
    own = argparse.ArgumentParser()
    node.add_arguments(own, max_hz=30.0)
    assert own.parse_args([]).max_hz == 30.0
    assert own.parse_args(["--max-hz", "0"]).max_hz is None and own.parse_args(["--max-hz", "none"]).max_hz is None
    assert "0 or none: every frame it can take (default: max 30 Hz)" in " ".join(own.format_help().split())
    for wrong in ("-1", "fast", "nan"):
        with pytest.raises(SystemExit):
            own.parse_args(["--max-hz", wrong])
    with pytest.raises(ValueError, match="above 0"):
        node.add_arguments(argparse.ArgumentParser(), max_hz=0)
    assert (node.rate_text(30.0), node.rate_text(2.5), node.rate_text(None)) == ("max 30 Hz", "max 2.5 Hz",
                                                                               "every frame")
    capsys.readouterr()
    node.start("durchblick-test", max_hz=30.0)
    node.start("durchblick-test", max_hz=None)
    assert capsys.readouterr().out.splitlines()[-2:] == ["durchblick-test: max 30 Hz", "durchblick-test: every frame"]
    with pytest.raises(TypeError, match="to watch"):
        node._subscriptions(object())


def test_reports_say_the_rate(sessions):  # noqa: F811
    """The effective rate in every report: given to the Reporter, or that of the subscriptions it watches."""
    a, b = sessions
    got = []
    sub = b.declare_subscriber("robot/x/diagnostics", lambda s: got.append(DiagnosticArray.deserialize(
        s.payload.to_bytes()).status[0]))
    given = node.Reporter(a, "robot/x", "given", "durchblick-given", every=0.0, max_hz=None)
    limited = ImageSubscriber(a, BASE, transport="shm", max_hz=20)
    watching = node.Reporter(a, "robot/x", "watching", "durchblick-watching", every=0.0)
    watching.watch(limited)
    time.sleep(0.3)
    given.publish(*node.state("1 frames/s"), {})
    watching.publish(*node.state("1 frames/s"), {})
    wait_until(lambda: len(got) == 2)
    rates = {s.name: {kv.key: kv.value for kv in s.values}["max-hz"] for s in got}
    assert rates == {"given": "every frame", "watching": "max 20 Hz"}
    sub.undeclare()
    limited.close()
    given.close()
    watching.close()


def test_each_transport_has_its_own_publish_ns(sessions):  # noqa: F811
    """publish_ns is taken right before each transport's put: a compressed sample's age at send has its encoding in
    it (it goes out after the raw one, encoded); publish_called_ns, the same for all, is when publish was called."""
    a, b = sessions
    pub = CameraPublisher(a, BASE, encoding="rgb8", frame_id="cam")
    got = {}
    subs = [b.declare_subscriber(_key(t), lambda s, t=t: got.setdefault(t, []).append(s))
            for t in ("raw", "compressed", "shm")]
    wait_until(lambda: pub.subscribed)
    time.sleep(0.3)
    image = np.random.default_rng(0).integers(0, 255, (480, 640, 3), np.uint8)  # an encoding that takes a while
    for _ in range(5):
        pub.publish(image, stamp_ns=time.time_ns() - 10 * MS)
        time.sleep(0.05)
    wait_until(lambda: all(len(got.get(t, ())) == 5 for t in ("raw", "compressed", "shm")))
    for i in range(5):
        meta = {t: camera_module.parse_attachment(got[t][i].attachment.to_bytes()) for t in got}
        assert len({m["seq"] for m in meta.values()}) == 1 and len({m["publish_called_ns"] for m in meta.values()}) == 1
        called = meta["raw"]["publish_called_ns"]
        assert all(m["publish_ns"] >= called for m in meta.values())
        ages = {t: stamps.age_at_send_ns(got[t][i]) for t in got}
        assert ages["compressed"] >= ages["raw"] >= ages["shm"] >= 10 * MS, ages
        # the encoding of a 640 x 480 JPEG is in the compressed sample's age
        assert meta["compressed"]["publish_ns"] - meta["raw"]["publish_ns"] > 0
    for s in subs:
        s.undeclare()
    pub.close()
