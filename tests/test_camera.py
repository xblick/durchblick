"""Publisher and subscriber over real Zenoh sessions (two peers on localhost)."""

import socket
import struct
import threading
import time

import numpy as np
import pytest

from blick import _core, topics
from blick import camera as camera_module
from blick.camera import CameraPublisher, ImageSubscriber
from blick.encodings import EncodingError
from blick.history import PartnerHistory
from blick.msg import CameraInfo
from blick.session import close as close_session
from blick.session import open_session, replies

BASE = "robot/camera/color"


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
    close_session(b)
    close_session(a)


def wait_until(condition, timeout=5.0):
    end = time.monotonic() + timeout
    while not condition():
        if time.monotonic() > end:
            raise TimeoutError
        time.sleep(0.01)


def pattern(seed=0, h=48, w=64):
    y, x = np.mgrid[0:h, 0:w]
    return np.stack([(x + seed) % 256, y * 4 % 256, np.full_like(x, seed % 256)], -1).astype(np.uint8)


def test_nothing_is_encoded_without_subscribers(sessions, monkeypatch):
    a, _ = sessions
    calls = []
    monkeypatch.setattr(camera_module, "compress", lambda *args, **kw: calls.append(1))
    publisher = CameraPublisher(a, BASE, encoding="rgb8", frame_id="cam")
    time.sleep(0.2)
    assert not publisher.subscribed
    assert publisher.publish(pattern()) == []
    assert calls == []
    publisher.close()


def test_compressed_stream_with_camera_info(sessions):
    a, b = sessions
    info = CameraInfo.pinhole(64, 48, 50.0, 50.0, 31.5, 23.5, frame_id="cam")
    publisher = CameraPublisher(a, BASE, encoding="rgb8", frame_id="cam", camera_info=info, source="sim")
    subscriber = ImageSubscriber(b, BASE, transport="compressed", camera_info=True)
    wait_until(lambda: publisher.subscribed)
    stamp = time.time_ns()
    image = pattern(3)
    sent = publisher.publish(image, stamp_ns=stamp)
    assert sent == [f"{BASE}/image_raw/compressed", f"{BASE}/camera_info"]
    frame = subscriber.latest(timeout=5.0)
    assert frame is not None
    assert frame.encoding == "rgb8" and frame.image.shape == image.shape
    assert np.abs(frame.image.astype(int) - image).mean() < 6
    assert frame.stamp_ns == stamp and frame.header.frame_id == "cam"
    assert frame.seq == 1 and frame.source == "sim" and frame.key == f"{BASE}/image_raw/compressed"
    assert subscriber.latest() is None  # consumed
    wait_until(lambda: subscriber.camera_info is not None)
    received_info = subscriber.camera_info
    assert received_info.header.stamp.to_ns() == stamp
    assert np.array_equal(received_info.k, info.k)
    subscriber.close()
    publisher.close()


def test_raw_stream_is_exact_and_keeps_only_the_newest(sessions):
    a, b = sessions
    publisher = CameraPublisher(a, BASE, encoding="rgb8", frame_id="cam")
    subscriber = ImageSubscriber(b, BASE, transport="raw")
    wait_until(lambda: publisher.subscribed)
    for seed in range(5):
        publisher.publish(pattern(seed))
    wait_until(lambda: subscriber.received == 5)
    frame = subscriber.latest()
    assert frame.seq == 5 and np.array_equal(frame.image, pattern(4))
    assert subscriber.overwritten == 4
    subscriber.close()
    publisher.close()


def test_a_partner_counts_none_overwritten(sessions):
    """A partner subscriber (keep_recent) never takes its frames, it pairs them by stamp (at): those it keeps are not
    lost, so none counts as overwritten (a plain subscriber's replaced frames do)."""
    a, b = sessions
    publisher = CameraPublisher(a, BASE, encoding="rgb8", frame_id="cam")
    plain = ImageSubscriber(b, BASE, transport="raw")
    partner = ImageSubscriber(b, BASE, transport="raw")
    partner.keep_recent(4)
    wait_until(lambda: publisher.subscribed)
    t0 = 1_000 * 1_000_000_000
    for i in range(5):
        publisher.publish(pattern(i), stamp_ns=t0 + i * 33_000_000)
    wait_until(lambda: plain.received == 5 and partner.received == 5)
    assert plain.overwritten == 4
    assert partner.overwritten == 0 and len(partner.recent_stamps()) == 4
    frame = partner.at(t0 + 2 * 33_000_000, 1_000_000)
    assert frame is not None and frame.stamp_ns == t0 + 2 * 33_000_000 and np.array_equal(frame.image, pattern(2))
    for s in (plain, partner):
        s.close()
    publisher.close()


def test_a_partner_read_too_late_is_lost_and_logged(sessions, caplog):
    """Over shm a partner sample only names a ring slot: read after the publisher reused the slot (a ring of 4, 10
    frames later), it is lost, counted in ``lost`` and logged with the fix; a recent one reads. The offset is measured:
    half the partner's period pairs each stamp with its nearest frame, and a stamp between two missing frames has
    none."""
    a, b = sessions
    publisher = CameraPublisher(a, BASE, encoding="rgb8", frame_id="cam", slots=4)
    partner = PartnerHistory(ImageSubscriber(b, BASE, transport="shm"), 30)
    wait_until(lambda: publisher.subscribed)
    t0, period = 1_000 * 1_000_000_000, 33_000_000
    for i in range(10):
        publisher.publish(pattern(i), stamp_ns=t0 + i * period)
    wait_until(lambda: partner.subscriber.received == 10)
    assert partner.period_ns() == period and partner.offset_ns() == period // 2
    frame = partner.at(t0 + 8 * period + 10_000_000)  # nearest: frame 8, 10 ms away
    assert frame is not None and np.array_equal(frame.image, pattern(8))
    assert partner.at(t0 + 9 * period + 10_000_000) is None  # 9, the newest, is before it: a nearer one may come
    with caplog.at_level("WARNING", logger="blick"):
        assert partner.at(t0 + 2 * period) is None
    assert partner.lost == 1 and "more ring slots" in caplog.text and "take a frame and its partners" in caplog.text
    assert partner.at(t0 + 12 * period) is None and partner.lost == 1  # no partner: not lost
    partner.close()
    publisher.close()


def test_a_partner_stamped_a_little_earlier_is_taken_at_once(sessions):
    """A stereo pair whose partner camera stamps 0.24 ms before the primary one (an e-CAM24 pair at 15 Hz): the
    newest partner sample pairs at once, with the measured and with an explicit offset, because the next one comes a
    period later and cannot be nearer. A primary stamp far enough after the newest sample still waits (a nearer one
    may come)."""
    a, b = sessions
    publisher = CameraPublisher(a, BASE, encoding="rgb8", frame_id="cam")
    measured = PartnerHistory(ImageSubscriber(b, BASE, transport="shm"), 30)
    explicit = PartnerHistory(ImageSubscriber(b, BASE, transport="shm"), 30, max_offset_ns=5_000_000)
    wait_until(lambda: publisher.subscribed)
    t0, period, early = 1_000 * 1_000_000_000, 66_666_667, 240_000
    for i in range(5):
        publisher.publish(pattern(i), stamp_ns=t0 + i * period)
    wait_until(lambda: measured.subscriber.received == 5 and explicit.subscriber.received == 5)
    newest = t0 + 4 * period
    for history in (measured, explicit):
        assert history.partner_of(newest + early) == newest
        frame = history.at(newest + early)
        assert frame is not None and np.array_equal(frame.image, pattern(4))
    assert measured.partner_of(newest + 30_000_000) is None  # 30 ms after it: one up to 60 ms after it is nearer
    assert explicit.partner_of(t0 + 3 * period + early) == t0 + 3 * period  # one later than it is kept
    for history in (measured, explicit):
        history.close()
    publisher.close()


class KeptStamps:
    """A partner subscriber of which PartnerHistory only asks the stamps kept."""

    def __init__(self, stamps):
        self.stamps = list(stamps)

    def keep_recent(self, n, on_put=None):
        pass

    def recent_stamps(self):
        return self.stamps


MS = 1_000_000
STEADY = [i * 40 * MS for i in range(5)]  # a 25 Hz partner: period 40 ms, newest 160 ms
CAMERA = 33_333_333  # a 30 Hz camera
SKIPPING = [i * CAMERA for i in (0, 4, 7, 11, 15)]  # a node's depth of some of its frames: steps 100 and 133 ms
JITTER = [0, 66_900_000, 133_100_000, 200_200_000, 266_400_000]  # a 15 Hz camera's stamps, with jitter


@pytest.mark.parametrize("kept, max_offset_ns, stamp, partner", [
    (STEADY, None, 170 * MS, 160 * MS),  # 10 ms (a quarter period) before: taken at once
    (STEADY, None, 170 * MS + 1, None),  # more than a quarter period before: the next one may be nearer
    (STEADY, 5 * MS, 162 * MS, 160 * MS),  # an explicit offset: taken at once as well
    (STEADY, 15 * MS, 171 * MS, None),  # explicit, more than a quarter period before: waits
    (STEADY, 5 * MS, 166 * MS, None),  # beyond the offset
    (STEADY, 5 * MS, 158 * MS, 160 * MS),  # the nearest is after the stamp: nothing nearer can come
    (STEADY, 20 * MS, 100 * MS, 80 * MS),  # equally near 80 and 120 ms: the first kept
    ([160 * MS], 5 * MS, 162 * MS, None),  # one sample: the period unmeasured, it waits for a later one
    ([160 * MS], None, 160 * MS, None),  # one sample, measured offset: no offset yet
    ([0, 10 * MS, 20 * MS, 160 * MS], 5 * MS, 162 * MS, None),  # not steady: the next one may come at any time
    (SKIPPING, None, 16 * CAMERA, None),  # skips camera frames: frame 16's own sample may still come
    (JITTER, 5 * MS, JITTER[-1] + 240_000, JITTER[-1]),  # steady within a tenth: taken at once
    (STEADY, 0, 160 * MS + 1, None),  # 0: equal stamps only
    (STEADY, 0, 160 * MS, 160 * MS),
    (STEADY, 0, 80 * MS, 80 * MS),
])
def test_the_partner_rule(kept, max_offset_ns, stamp, partner):
    """PartnerHistory.partner_of (C++ partner_of through _core): the nearest kept sample within the offset, taken
    once no nearer one can come. From a steady partner, a sample stamped at most a quarter of its smallest step before
    the primary frame is taken at once, because the next one comes about a step after it. A partner that skips camera
    frames may send the primary frame's own sample next: it waits."""
    history = PartnerHistory(KeptStamps(kept), 30, max_offset_ns=max_offset_ns)
    assert history.partner_of(stamp) == partner


def test_the_partner_period():
    """The median step of the stamps kept, each counted once (of two middle steps the larger); None before two."""
    assert [_core.partner_period_ns(k) for k in ([], [5], [5, 5])] == [None, None, None]
    assert _core.partner_period_ns([0, 10, 20, 160]) == 10
    assert _core.partner_period_ns([30, 0, 10]) == 20
    with pytest.raises(ValueError, match="not negative"):
        _core.partner_of(STEADY, 0, -1)


def test_a_partner_waited_for(sessions):
    """``wait``: the partner of a primary frame that comes after it (depth stamped later than its colour), read as it
    comes; None after the timeout without one."""
    a, b = sessions
    publisher = CameraPublisher(a, BASE, encoding="rgb8", frame_id="cam")
    partner = PartnerHistory(ImageSubscriber(b, BASE, transport="shm"), 30, max_offset_ns=5_000_000)
    wait_until(lambda: publisher.subscribed)
    t0 = 1_000 * 1_000_000_000
    publisher.publish(pattern(0), stamp_ns=t0)
    wait_until(lambda: partner.subscriber.received == 1)
    assert partner.wait(t0 + 33_000_000, 0.1) is None
    later = threading.Timer(0.1, lambda: publisher.publish(pattern(1), stamp_ns=t0 + 36_000_000))
    later.start()
    started = time.monotonic()
    frame = partner.wait(t0 + 33_000_000, 2.0)  # 1 is 3 ms away, and the newest: nothing nearer can come after it
    assert frame is not None and np.array_equal(frame.image, pattern(1)) and time.monotonic() - started < 1.0
    later.join()
    partner.close()
    publisher.close()


def test_depth_png_is_lossless(sessions):
    a, b = sessions
    publisher = CameraPublisher(a, "robot/camera/depth", encoding="16UC1", frame_id="cam", codec="png")
    subscriber = ImageSubscriber(b, "robot/camera/depth")
    wait_until(lambda: publisher.subscribed)
    depth = (np.arange(48 * 64).reshape(48, 64) * 7 % 6000).astype(np.uint16)
    publisher.publish(depth)
    frame = subscriber.latest(timeout=5.0)
    assert frame.encoding == "16UC1" and np.array_equal(frame.image, depth)
    subscriber.close()
    publisher.close()


def test_wrong_array_is_refused(sessions):
    a, _ = sessions
    publisher = CameraPublisher(a, BASE, encoding="rgb8", frame_id="cam")
    with pytest.raises(EncodingError):
        publisher.publish(np.zeros((4, 4), np.uint8))
    publisher.close()


def test_codec_is_checked_when_made(sessions):
    """A codec that cannot compress the encoding is refused when the publisher is made, not when a compressed
    subscriber comes (nodes crashed on the first viewer); codec=None publishes without a compressed form."""
    a, b = sessions
    with pytest.raises(EncodingError, match="use rvl or png"):
        CameraPublisher(a, BASE, encoding="mono16", frame_id="cam")
    with pytest.raises(EncodingError, match="codec=None"):
        CameraPublisher(a, BASE, encoding="32FC1", frame_id="cam", codec="png")
    publisher = CameraPublisher(a, BASE, encoding="32FC1", frame_id="cam", codec=None)
    assert "compressed" not in publisher.info_record()["transports"]
    subscriber = ImageSubscriber(b, BASE, transport="raw")
    wait_until(lambda: publisher.subscribed)
    values = np.linspace(0.0, 1.0, 48 * 64, dtype=np.float32).reshape(48, 64)
    publisher.publish(values)
    assert np.array_equal(subscriber.latest(timeout=5.0).image, values)
    subscriber.close()
    publisher.close()


def test_invalid_topic_names():
    from blick.topics import TopicError, stream_base

    with pytest.raises(TopicError):
        stream_base("robot", "head-camera", "color")
    with pytest.raises(TopicError):
        stream_base("robot", "1camera", "color")


def test_static_transforms(sessions):
    from blick.camera import StaticTransformPublisher, StaticTransformSubscriber
    from blick.msg import Header, Quaternion, TFMessage, Time, Transform, TransformStamped, Vector3

    a, b = sessions
    camera = BASE.rpartition("/")[0]
    transforms = [TransformStamped(Header(Time(1, 2), "depth"), "rgb",
                                   Transform(Vector3(-0.032, 0.002, 0.004), Quaternion(0.1, 0.0, 0.0, 0.995)))]
    early = []
    first = StaticTransformSubscriber(b, camera, early.append)
    time.sleep(0.3)  # the subscriber reaches the other session
    publisher = StaticTransformPublisher(a, camera, transforms)
    wait_until(lambda: early)
    first.check()
    assert early[0] == TFMessage(transforms=transforms)
    # a subscriber that starts after the publication asks for it
    late = []
    second = StaticTransformSubscriber(b, camera, late.append)
    wait_until(lambda: late)
    second.check()
    assert late == [TFMessage(transforms=transforms)]
    first.close()
    second.close()
    publisher.close()


@pytest.mark.skipif(_core.cuda_device_count() == 0, reason="no CUDA device")
def test_local_auto_takes_the_cuda_ring_on_its_gpu(sessions, monkeypatch):  # noqa: F811
    """local "auto" (the default): the publisher's CUDA ring when it is on the subscriber's GPU (by the GPU's UUID,
    in the info record from the start) or on another GPU that the process sees, else shared memory, saying why; "shm" and "cuda" as asked. (Only the choice:
    a CUDA ring cannot be read by the process that made it; test_rings and test_video read them.)"""
    from blick import camera

    a, b = sessions
    publisher = CameraPublisher(a, BASE, encoding="rgb8", frame_id="cam", cuda_device=0)
    assert publisher.info_record()["cuda_device_uuid"] == _core.cuda_device_uuid(0)

    def chosen(**options):
        subscriber = ImageSubscriber(b, BASE, transport="auto", **options)
        wait_until(lambda: subscriber.transport is not None)
        try:
            return subscriber.transport_reason
        finally:
            subscriber.close()

    # without a device the subscriber works on the CPU: shared memory, though the ring is on this very GPU
    assert chosen() == "shm (this subscriber works on the CPU)"
    assert chosen(device="cuda:0") == "cuda (publisher on this GPU)"
    assert chosen(device=0) == "cuda (publisher on this GPU)"
    assert chosen(local="shm") == "shm (asked for)"
    assert chosen(local="cuda") == "cuda (asked for)"
    assert chosen(device="cpu") == "shm (this subscriber works on the CPU)"
    monkeypatch.setattr(camera.Subscription, "_own_uuid", lambda self: "f" * 32)  # this subscriber on another GPU
    ring = _core.cuda_device_uuid(0)[:8]  # a GPU that this process sees: copied from GPU to GPU
    assert chosen(device="cuda:0") == f"cuda (copied from the publisher's GPU {ring})"
    monkeypatch.setattr(camera._core, "cuda_device_ordinal", lambda uuid: -1)  # a GPU that this process does not see
    assert chosen(device="cuda:0") == f"shm (publisher's ring is on GPU {ring}, which this process does not see)"
    publisher.close()
    plain = CameraPublisher(a, f"{BASE}2", encoding="rgb8", frame_id="cam")
    subscriber = ImageSubscriber(b, f"{BASE}2", transport="auto", device="cuda:0")
    wait_until(lambda: subscriber.transport is not None)
    assert subscriber.transport_reason == "shm (publisher offers no CUDA ring)"
    assert subscriber.describe().startswith("shm (publisher offers no CUDA ring)")
    subscriber.close()
    plain.close()


def test_auto_on_a_third_host_ignores_a_relays_record(sessions, monkeypatch):
    """A relay (durchblick-relay) offers only its own host's rings: on a third host auto takes the source's compressed
    stream instead of refusing the stream because one record (the relay's) has no compressed form."""
    _, b = sessions
    source = {"v": 1, "kind": "image", "host": "source-host", "transports": ["raw", "compressed", "shm"],
              "encoding": "rgb8", "frame_id": "cam", "source": "robot", "cuda_device_uuid": None, "relayed_from": None}
    relay = dict(source, host="relay-host", transports=["shm", "cuda"], relayed_from="source-host")
    monkeypatch.setattr(camera_module, "query_info", lambda *args, **kw: [source, relay])
    subscriber = ImageSubscriber(b, BASE, transport="auto")
    try:
        end = time.monotonic() + 5
        while not subscriber.transport_reason and time.monotonic() < end:
            time.sleep(0.02)
        assert subscriber.transport_reason == "compressed (publisher on another host)"
        assert subscriber.latest() is None  # no error raised by the choice
    finally:
        subscriber.close()


def test_find_streams_keeps_the_source_and_notes_its_relays(monkeypatch):
    """A source and its relay answer <base>/info: one entry per stream, the source's record, the relay's host in
    "relayed_by" (in either order of the replies); a stream only a relay answers for is left out."""
    import json as _json
    from types import SimpleNamespace

    source = {"v": 2, "kind": "image", "host": "src-host", "transports": ["compressed"], "relayed_from": None}
    relay = {"v": 2, "kind": "image", "host": "relay-host", "transports": ["shm"], "relayed_from": "src-host"}
    orphan = dict(relay, host="other-relay")

    def reply(base, record):
        return SimpleNamespace(ok=SimpleNamespace(key_expr=f"{base}/info", payload=SimpleNamespace(
            to_bytes=lambda: _json.dumps(record).encode())))

    for order in ([source, relay], [relay, source]):
        monkeypatch.setattr(camera_module, "replies", lambda *a, order=order, **kw: [
            reply("cam/left", r) for r in order] + [reply("cam/gone", orphan)])
        found = camera_module.find_streams(None, "", timeout=0.05)
        assert [b for b, _ in found] == ["cam/left"]
        record = found[0][1]
        assert record["host"] == "src-host" and record["relayed_by"] == ["relay-host"]


OTHER_HOST = "0c9a1f6e-5b0d-4a52-9d3e-7f1b2c3d4e5f"


def _publisher_on_another_host(session, monkeypatch, **options):
    """A publisher as one on another PC is seen from here: its records and its rings' keys name another host."""
    with monkeypatch.context() as patched:
        patched.setattr(_core, "host_id", lambda: OTHER_HOST)
        return CameraPublisher(session, BASE, encoding="rgb8", frame_id="cam", source="source", **options)


def test_ring_keys_name_their_host(sessions, monkeypatch):  # noqa: F811
    """A ring's descriptors go to the readers of its host only: a publisher on another host and one here offer the
    host ring of one stream side by side, and a reader here gets only the frames of the ring here."""
    a, b = sessions
    far = _publisher_on_another_host(a, monkeypatch)
    near = CameraPublisher(a, BASE, encoding="rgb8", frame_id="cam", source="near")
    sub = ImageSubscriber(b, BASE, transport="shm")
    wait_until(lambda: near.subscribed)
    assert not far.subscribed  # nobody on its host reads its ring
    for i in range(1, 6):
        assert far.publish(pattern(i), stamp_ns=i) == []
        assert near.publish(pattern(i), stamp_ns=i) == [f"{BASE}/image_raw/shm/{_core.host_id()}"]
    frame = sub.latest(timeout=5.0)
    assert frame is not None and frame.source == "near" and frame.key.endswith("/shm/" + _core.host_id())
    for p in (sub, far, near):
        p.close()


@pytest.mark.parametrize("lag", [0, 1, 2])
def test_auto_moves_to_a_ring_that_appears_and_back(sessions, monkeypatch, lag):  # noqa: F811
    """The order of starting does not matter: a subscriber that reads a stream of another host over the network moves
    to the ring of a publisher that starts later on its own host (a relay), and back when that one stops. While it
    moves, it hands out every frame once: none twice, and none lost on the way to the ring. ``lag``: the relay
    republishes a frame when the source sends the one so many frames later (a relay receives a frame, then writes it
    into its ring): the ring's first frames are frames the network handed out already. Before 0.3.69 the subscriber
    handed them out again (with lag 0 only when its handover ended between the two copies of a frame)."""
    a, b = sessions
    sub = ImageSubscriber(b, BASE, transport="auto")  # before any publisher
    source = _publisher_on_another_host(a, monkeypatch, codec="png")
    relay, relay_since, relay_until = None, None, None
    got = []  # (stamp, transport)
    end = time.monotonic() + 30
    stamp = 0
    while time.monotonic() < end and (relay_until is None or stamp < relay_until + 40):
        stamp += 1
        source.publish(pattern(stamp), stamp_ns=stamp)
        if relay is not None:
            if stamp - lag > relay_since:  # the same frame, as a relay republishes it
                relay.publish(pattern(stamp - lag), stamp_ns=stamp - lag)
        frame = sub.latest(timeout=0.5)  # one frame published, one read: the store keeps only the newest
        if frame is not None:
            got.append((frame.stamp_ns, "ring" if frame.key.endswith("/shm/" + _core.host_id()) else "network"))
            np.testing.assert_array_equal(frame.image, pattern(frame.stamp_ns))
        if relay is None and relay_since is None and len(got) >= 30:
            # without a compressed form, as a relay: only the source sends on that key
            relay = CameraPublisher(a, BASE, encoding="rgb8", frame_id="cam", source="relay", codec=None)
            relay_since = stamp
        elif relay is not None and sum(1 for _, how in got if how == "ring") >= 30:
            relay.close()
            relay, relay_until = None, stamp
    assert relay_until is not None, f"never read the ring: {sub.describe()}"
    stamps = [s for s, _ in got]
    assert stamps == sorted(set(stamps)), "a frame twice, or out of order"
    routes = [how for i, (_, how) in enumerate(got) if i == 0 or got[i - 1][1] != how]
    assert routes == ["network", "ring", "network"]
    # nothing lost from the first frame until the relay stopped (after that, the frames it never passed on are)
    first_ring = next(s for s, how in got if how == "ring")
    to_ring = [s for s in stamps if stamps[0] <= s <= first_ring + 10]
    assert to_ring == list(range(stamps[0], first_ring + 11)), "a frame lost while moving to the ring"
    assert sub.transport == "compressed" and sub.transport_reason == "compressed (publisher on another host)"
    sub.close()
    source.close()


# _Latest while a subscription changes its transport, driven by hand (no Zenoh); the same cases in
# cpp/tests/test_latest.cpp

T0 = 1_000_000_000_000  # 1000 s
PERIOD = 33_333_333  # 30 Hz


def frame(n):
    return T0 + n * PERIOD


def offer(latest, route, stamp):
    """Offers a sample of ``stamp`` by ``route``; the stamp handed out then (None: none)."""
    payload = struct.pack("<iiI", 0, stamp // 1_000_000_000, stamp % 1_000_000_000) + bytes(4)
    latest.offer((payload, None, "k", 0, "e", "shm"), route)
    item = latest.take(consume=True, timeout=None)
    return None if item is None else camera_module._stamp_of(item[0])


def offer_all(latest, route, stamps, out):
    out.extend(s for s in (offer(latest, route, s) for s in stamps) if s is not None)


def test_handover_leaves_out_a_lagging_new_routes_copies_and_the_old_route():
    """The network hands out 1..10; the ring lags by two frames: its 8, 9, 10 are frames handed out already, and the
    network's 11, still delivered while its subscriber is undeclared, comes after the ring's first sample."""
    latest, out = camera_module._Latest(), []
    latest.move(1)
    offer_all(latest, 1, [frame(n) for n in range(1, 11)], out)
    latest.move(2)
    offer_all(latest, 2, [frame(8)], out)
    offer_all(latest, 1, [frame(11)], out)  # the old route, left at the new route's first sample
    offer_all(latest, 2, [frame(9), frame(10), frame(11), frame(12)], out)
    assert out == [frame(n) for n in range(1, 13)]
    assert latest.duplicates == 3


def test_handover_keeps_a_restart_far_back_at_once():
    """A restart (a simulation's stamps start again) more than HANDOVER_NS behind the newest frame is kept at once on
    the new route after the old one was left. A first version of 0.3.69 left it out for 2 s (60 frames at 30 Hz)."""
    latest, out = camera_module._Latest(), []
    latest.move(1)
    offer_all(latest, 1, [frame(n) for n in range(1, 101)], out)
    latest.move(2)
    assert offer(latest, 2, frame(100)) is None  # the ring's copy: the old route is left
    assert offer(latest, 2, 1 * PERIOD) == 1 * PERIOD
    assert offer(latest, 2, 2 * PERIOD) == 2 * PERIOD


def test_handover_restart_by_the_old_route_while_both_are_read():
    """The source restarts while both routes are read, the network first: the network's restarted frame is left out
    (its frames come again by the ring), the ring's lagging copies of 99 and 100 too, and the ring's restarted frames
    are kept. Taken as a restart on the old route, the ring's 99, 100 and 1 came twice."""
    latest, out = camera_module._Latest(), []
    latest.move(1)
    offer_all(latest, 1, [frame(n) for n in range(1, 101)], out)
    latest.move(2)
    offer_all(latest, 1, [1 * PERIOD], out)  # the network's restart, before the ring's first sample
    offer_all(latest, 2, [frame(99), frame(100), 1 * PERIOD, 2 * PERIOD], out)
    assert out == [frame(n) for n in range(1, 101)] + [1 * PERIOD, 2 * PERIOD]


def test_handover_restart_by_the_new_route_first():
    """The mirror case: the ring brings the restart as its first sample while the network still delivers frames of
    before: the restart is kept and ends the handover, and the network's later frames are left out with its route."""
    latest, out = camera_module._Latest(), []
    latest.move(1)
    offer_all(latest, 1, [frame(n) for n in range(1, 101)], out)
    latest.move(2)
    offer_all(latest, 2, [1 * PERIOD], out)
    offer_all(latest, 1, [frame(101), 1 * PERIOD], out)  # the old route, left
    offer_all(latest, 2, [2 * PERIOD], out)
    assert out == [frame(n) for n in range(1, 101)] + [1 * PERIOD, 2 * PERIOD]


@pytest.mark.parametrize("behind,kept", [(-1, False), (0, False), (1, True)])
def test_handover_restart_boundary(behind, kept):
    """On the new route a stamp up to HANDOVER_NS behind the newest one is the same frame, one ns further a restart."""
    latest = camera_module._Latest()
    latest.move(1)
    assert offer(latest, 1, frame(100)) == frame(100)
    latest.move(2)
    stamp = frame(100) - camera_module.HANDOVER_NS - behind
    assert offer(latest, 2, stamp) == (stamp if kept else None)


def test_handover_leaves_out_equal_stamps_during_the_window():
    """Frames that carry one stamp on purpose, or a stamp less than HANDOVER_S behind the newest: left out during the
    window after a move (a remaining case, named in _Latest's docstring)."""
    latest = camera_module._Latest()
    latest.move(1)
    assert offer(latest, 1, frame(5)) == frame(5)
    latest.move(2)
    assert offer(latest, 2, frame(5)) is None
    assert offer(latest, 2, frame(5)) is None
    assert offer(latest, 2, frame(5) - 1_000_000_000) is None


def test_handover_window_ends_by_time(monkeypatch):
    """The window ends after HANDOVER_S: a new route that stops longer after a first sample behind the newest one hands
    out older frames again (a remaining case, named in _Latest's docstring)."""
    monkeypatch.setattr(camera_module, "HANDOVER_S", 0.3)  # also the window in stamp time: 9 frames at 30 Hz
    latest = camera_module._Latest()
    latest.move(1)
    offer_all(latest, 1, [frame(n) for n in range(1, 11)], [])
    latest.move(2)
    assert offer(latest, 2, frame(5)) is None
    time.sleep(0.4)
    assert offer(latest, 2, frame(6)) == frame(6)


def test_handover_leaves_a_new_route_that_never_delivers():
    """leave_old (the end of HANDOVER_S, or close) leaves the old route although the new one delivered nothing."""
    latest, out = camera_module._Latest(), []
    latest.move(1)
    offer_all(latest, 1, [frame(n) for n in range(1, 6)], out)
    latest.move(2)
    assert not latest.wait_moved(0.01)
    latest.leave_old()
    assert latest.wait_moved(0.0)
    offer_all(latest, 1, [frame(6)], out)  # the old route, left
    offer_all(latest, 2, [frame(5), frame(6), frame(7)], out)
    assert out == [frame(n) for n in range(1, 8)]


def test_handover_two_moves_in_a_row():
    """The second move before the route of the first delivered: the route before is read until the newest one
    delivers, an older one not at all."""
    latest, out = camera_module._Latest(), []
    latest.move(1)
    offer_all(latest, 1, [frame(n) for n in range(1, 6)], out)
    latest.move(2)
    latest.leave_old()
    latest.move(3)
    offer_all(latest, 1, [frame(6)], out)  # left with the first move
    offer_all(latest, 2, [frame(6)], out)  # read until route 3 delivers
    offer_all(latest, 3, [frame(6)], out)  # the same frame: left out, route 2 left
    offer_all(latest, 2, [frame(7)], out)
    offer_all(latest, 3, [frame(7)], out)
    assert out == [frame(n) for n in range(1, 8)]


def test_auto_closes_during_a_move(sessions, monkeypatch):  # noqa: F811
    """close() while a subscriber waits for the first frame of the ring it moves to (a relay that sends nothing) ends
    the handover at once instead of after HANDOVER_S."""
    a, b = sessions
    sub = ImageSubscriber(b, BASE, transport="auto")
    source = _publisher_on_another_host(a, monkeypatch, codec="png")
    wait_until(lambda: sub.transport == "compressed")
    relay = CameraPublisher(a, BASE, encoding="rgb8", frame_id="cam", source="relay", codec=None)
    wait_until(lambda: sub._latest._old is not None)  # moving: the ring's subscriber declared, no frame of it yet
    start = time.monotonic()
    sub.close()
    assert time.monotonic() - start < camera_module.HANDOVER_S / 2
    assert not sub._chooser.is_alive()
    relay.close()
    source.close()


def test_auto_asks_this_hosts_relay_for_a_stream_without_a_ring_here(sessions, monkeypatch):  # noqa: F811
    """A subscriber with auto holds the liveliness token <base>/want/<host>/<id> while the stream has no source with
    a ring on its host (durchblick-relay brings the streams wanted on its host into rings there); a subscriber with a
    transport of its own choice holds none."""
    a, b = sessions

    def wanted():
        return [str(r.ok.key_expr) for r in replies(a, topics.wanted_on(_core.host_id()), 1.0, liveliness=True)
                if r.ok is not None]

    source = _publisher_on_another_host(a, monkeypatch)
    fixed = ImageSubscriber(b, BASE, transport="compressed")
    sub = ImageSubscriber(b, BASE, transport="auto")
    wait_until(lambda: len(wanted()) == 1)
    assert topics.wanted_stream(wanted()[0]) == BASE and wanted()[0].startswith(f"{BASE}/want/{_core.host_id()}/")
    near = CameraPublisher(a, BASE, encoding="rgb8", frame_id="cam", codec=None)  # a source with a ring here
    wait_until(lambda: wanted() == [])
    near.close()
    wait_until(lambda: len(wanted()) == 1)
    sub.close()
    wait_until(lambda: wanted() == [])
    fixed.close()
    source.close()
