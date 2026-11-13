"""The shm and cuda transports: the C++ rings through xblick._core, and publisher/subscriber over Zenoh.

CUDA's interprocess events cannot be opened in the process that created them, so the CUDA tests run the publishers
in processes of their own (multiprocessing, spawn). They need a GPU with virtual memory management and are skipped
without one (CUDA_VISIBLE_DEVICES selects it)."""

import ctypes
import multiprocessing as mp
import socket
import time

import numpy as np
import pytest
from test_camera import stop_child, wait_until

import xblick
from xblick import _core
from xblick.camera import CameraPublisher, ImageSubscriber
from xblick.gpu import CudaImage
from xblick.msg import CudaImage as CudaImageMsg
from xblick.msg import Header, ShmImage, Time
from xblick.session import open_session

needs_cuda = pytest.mark.skipif(_core.cuda_device_count() == 0, reason="no CUDA device")


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def pattern(i, shape=(480, 640, 3)):
    image = np.zeros(shape, np.uint8)
    image[..., 0] = i % 256
    image[:, :, 1] = np.arange(shape[1], dtype=np.uint8)[None, :]
    return image


def wait_frame(sub, timeout=10.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        frame = sub.latest(timeout=0.1)
        if frame is not None:
            return frame
    raise TimeoutError("no frame")


# ------------------------------------------------------------------------------------------------ host ring


def test_host_ring_copy_view_and_overwrite():
    writer = _core.HostRingWriter(3, 1000)
    reader = _core.HostRingReader(writer.name)
    assert (reader.slot_count, reader.slot_size) == (3, 4096)  # slots are whole pages
    slot, seq = writer.write(np.full(1000, 7, np.uint8))
    out = np.empty(1000, np.uint8)
    assert reader.read(slot, seq, out) and (out == 7).all()
    view = reader.view(slot, 1000)
    assert not view.flags.writeable and (view == 7).all()
    for _ in range(3):
        writer.write(np.full(1000, 9, np.uint8))
    assert not reader.read(slot, seq, out), "an overwritten frame must be refused"
    assert not reader.validate(slot, seq)


def test_host_ring_takes_read_only_arrays():
    writer = _core.HostRingWriter(2, 100)
    frame = np.full(100, 3, np.uint8)
    frame.flags.writeable = False  # e.g. np.asarray of a PIL image
    slot, _ = writer.write(frame)
    assert (_core.HostRingReader(writer.name).view(slot, 100) == 3).all()


def test_host_ring_loan_commit():
    writer = _core.HostRingWriter(2, 100)
    reader = _core.HostRingReader(writer.name)
    slot_buffer = writer.loan()
    slot_buffer[:100] = 5
    slot, seq = writer.commit(100)
    assert reader.nbytes(slot) == 100 and (reader.view(slot, 100) == 5).all() and reader.validate(slot, seq)


def test_host_ring_rejects_too_large_frames_and_unknown_names():
    writer = _core.HostRingWriter(2, 4096)
    with pytest.raises(_core.RingError):
        writer.write(np.zeros(4097, np.uint8))
    with pytest.raises(_core.RingError):
        _core.HostRingReader("/xblick-does-not-exist")


def test_ring_goes_with_its_writer():
    writer = _core.HostRingWriter(2, 10)
    name = writer.name
    del writer
    with pytest.raises(_core.RingError):
        _core.HostRingReader(name)


# ------------------------------------------------------------------------------------------- descriptors


def test_descriptor_round_trip():
    header = Header(stamp=Time(sec=3, nanosec=4), frame_id="cam")
    shm = ShmImage(header=header, height=2, width=3, encoding="rgb8", step=9, host="h", segment="/xblick-x",
                   slot=1, seq=2 ** 40)
    assert ShmImage.deserialize(shm.serialize()) == shm
    cuda = CudaImageMsg(header=header, height=2, width=3, encoding="16UC1", step=6, host="h", endpoint="xblick-y",
                        device_uuid="ab" * 16, slot=3, seq=8)
    assert CudaImageMsg.deserialize(cuda.serialize()) == cuda


# --------------------------------------------------------------------------------------- shm over Zenoh


@pytest.fixture
def sessions():
    port = free_port()
    a = open_session(listen=[f"127.0.0.1:{port}"])
    b = open_session(connect=[f"127.0.0.1:{port}"])
    yield a, b
    b.close()
    a.close()


def publish_until(pub, image, sub, timeout=10.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pub.subscribed:
            pub.publish(image)
        frame = sub.latest(timeout=0.05)
        if frame is not None:
            return frame
    raise TimeoutError("no frame")


def test_shm_transport_copy(sessions):
    a, b = sessions
    pub = CameraPublisher(a, "t/cam/color", encoding="rgb8", frame_id="cam", source="test")
    sub = ImageSubscriber(b, "t/cam/color", "shm")
    frame = publish_until(pub, pattern(3), sub)
    assert frame.encoding == "rgb8" and frame.source == "test" and frame.is_valid()
    np.testing.assert_array_equal(frame.image, pattern(3))


def test_a_larger_frame_gets_a_new_ring(sessions):
    """A camera back with another size: the publisher makes its ring anew for the larger frame, the reader follows."""
    a, b = sessions
    pub = CameraPublisher(a, "t/cam/grow", encoding="rgb8", frame_id="cam")
    sub = ImageSubscriber(b, "t/cam/grow", transport="shm")
    wait_until(lambda: pub.subscribed)
    rings = []
    for shape, value in (((16, 32, 3), 1), ((64, 64, 3), 2), ((16, 32, 3), 3)):  # slots are whole pages (4096 B)
        pub.publish(np.full(shape, value, np.uint8))
        frame = sub.latest(timeout=5.0)
        assert frame.image.shape == shape and frame.image[0, 0, 0] == value
        rings.append(pub._host_ring.name)
    assert rings[0] != rings[1] == rings[2]  # anew for the larger frame, kept for the smaller one
    sub.close()
    pub.close()


@needs_cuda
def test_a_larger_frame_gets_a_new_cuda_ring(sessions):
    """The CUDA ring grows as the host ring (its readers are other processes: test_cuda_transport_from_host_and_from_
    device)."""
    pub = CameraPublisher(sessions[0], "t/cam/grow_cuda", encoding="rgb8", frame_id="cam", cuda_device=0)
    small, large = (np.zeros((4, 6, 3), np.uint8), np.zeros((20, 20, 3), np.uint8))  # slots: multiples of 512 B
    pub._write_cuda(small)
    first = pub._cuda_ring.endpoint
    pub._write_cuda(small)
    assert pub._cuda_ring.endpoint == first  # the same size: the same ring
    pub._write_cuda(large)
    assert pub._cuda_ring.endpoint != first and pub._cuda_ring.slot_size >= large.nbytes
    pub.close()


def test_frame_of_a_stopped_publisher_is_skipped(sessions):
    """A descriptor that came before its publisher stopped, read after the ring is gone (a camera's restart): the
    frame is stale, not an error that ends the reader; the next publisher's frames come."""
    a, b = sessions
    pub = CameraPublisher(a, "t/cam/color", encoding="rgb8", frame_id="cam")
    sub = ImageSubscriber(b, "t/cam/color", transport="shm")
    wait_until(lambda: pub.subscribed)
    pub.publish(np.zeros((4, 6, 3), np.uint8))
    wait_until(lambda: sub.received == 1)  # the descriptor is in, unread
    pub.close()
    del pub  # the ring removed
    assert sub.latest(timeout=0.5) is None and sub.stale == 1
    again = CameraPublisher(a, "t/cam/color", encoding="rgb8", frame_id="cam")
    wait_until(lambda: again.subscribed)
    again.publish(np.full((4, 6, 3), 7, np.uint8))
    frame = sub.latest(timeout=5.0)
    assert frame is not None and frame.image[0, 0, 0] == 7
    sub.close()
    again.close()


def test_shm_transport_zero_copy_detects_overwrite(sessions):
    a, b = sessions
    pub = CameraPublisher(a, "t/cam/depth", encoding="16UC1", frame_id="cam", slots=2, codec="png")
    sub = ImageSubscriber(b, "t/cam/depth", "shm", copy=False)
    depth = (np.arange(48 * 64, dtype=np.uint16).reshape(48, 64))
    frame = publish_until(pub, depth, sub)
    np.testing.assert_array_equal(frame.image, depth)
    assert frame.is_valid()
    pub.publish(depth)
    pub.publish(depth)  # two more frames in a ring of two: the first slot is overwritten
    assert not frame.is_valid()


# -------------------------------------------------------------------------------------------- cuda (GPU)


def _publish_host_frames(port, base, n, ready):
    """Child: publish numpy frames on the cuda transport (uploaded into the CUDA ring)."""
    session = open_session(listen=[f"127.0.0.1:{port}"])
    pub = CameraPublisher(session, base, encoding="rgb8", frame_id="cam", cuda_device=0)
    ready.set()
    end = time.monotonic() + 30
    while time.monotonic() < end and n.value > 0:
        if pub.subscribed:
            pub.publish(pattern(n.value))
        time.sleep(0.02)
    pub.close()
    session.close()


def _republish_device_frames(port_in, port_out, base_in, base_out, ready, stop):
    """Child: receive frames on the cuda transport and publish the device arrays themselves (GPU input), until
    ``stop``."""
    s_in = open_session(connect=[f"127.0.0.1:{port_in}"])
    s_out = open_session(listen=[f"127.0.0.1:{port_out}"])
    sub = ImageSubscriber(s_in, base_in, "cuda")
    pub = CameraPublisher(s_out, base_out, encoding="rgb8", frame_id="cam", cuda_device=0)
    ready.set()
    end = time.monotonic() + 30
    while time.monotonic() < end and not stop.is_set():
        frame = sub.latest(timeout=0.1)
        if frame is not None and pub.subscribed:
            pub.publish(frame.image)  # a CudaImage: __cuda_array_interface__
    pub.close()
    s_out.close()
    s_in.close()


@needs_cuda
def test_cuda_transport_from_host_and_from_device():
    ctx = mp.get_context("spawn")
    port_a, port_b = free_port(), free_port()
    n = ctx.Value("i", 5)
    ready_a, ready_b, stop = ctx.Event(), ctx.Event(), ctx.Event()
    first = ctx.Process(target=_publish_host_frames, args=(port_a, "t/a/color", n, ready_a), daemon=True)
    first.start()
    assert ready_a.wait(60)
    second = ctx.Process(target=_republish_device_frames,
                         args=(port_a, port_b, "t/a/color", "t/b/color", ready_b, stop), daemon=True)
    second.start()
    assert ready_b.wait(60)
    session = open_session(connect=[f"127.0.0.1:{port_b}"])
    try:
        cuda_sub = ImageSubscriber(session, "t/b/color", "cuda")
        shm_sub = ImageSubscriber(session, "t/b/color", "shm")  # needs the publisher's download of device frames
        frame = wait_frame(cuda_sub, 60)
        assert isinstance(frame.image, CudaImage)
        iface = frame.image.__cuda_array_interface__
        assert iface["shape"] == (480, 640, 3) and iface["typestr"] == "|u1"
        assert iface["data"][1] is False  # a copy is the reader's own: writable (torch.as_tensor takes no other)
        host = frame.image.numpy()
        np.testing.assert_array_equal(host[..., 1:], pattern(0)[..., 1:])
        assert host[0, 0, 0] == host[-1, -1, 0] and 1 <= host[0, 0, 0] <= 5
        shm_frame = wait_frame(shm_sub, 30)
        np.testing.assert_array_equal(shm_frame.image[..., 1:], pattern(0)[..., 1:])
    finally:
        n.value = 0
        session.close()
        stop_child(second, stop)
        first.join(10)


needs_two_gpus = pytest.mark.skipif(_core.cuda_device_count() < 2, reason="needs two CUDA devices")


@needs_two_gpus
def test_a_reader_on_another_gpu_gets_the_frame_on_its_own_gpu():
    """The publisher's CUDA ring is on device 0, the readers work on device 1: auto takes the CUDA ring and the frames
    arrive on device 1, copied from GPU to GPU; a reader that asks for views gets copies (a view would be on device
    0)."""
    ctx = mp.get_context("spawn")
    port = free_port()
    n = ctx.Value("i", 5)
    ready = ctx.Event()
    child = ctx.Process(target=_publish_host_frames, args=(port, "t/x/color", n, ready), daemon=True)
    child.start()
    assert ready.wait(60)
    session = open_session(connect=[f"127.0.0.1:{port}"])
    try:
        assert _core.cuda_device_ordinal(_core.cuda_device_uuid(1)) == 1
        assert _core.cuda_device_ordinal("0" * 32) == -1
        for copy in (True, False):
            sub = ImageSubscriber(session, "t/x/color", "auto", device=1, copy=copy)
            frame = wait_frame(sub, 60)
            assert sub.transport == "cuda" and "copied from the publisher's GPU" in sub.transport_reason
            assert isinstance(frame.image, CudaImage) and frame.image.device == 1
            assert frame.is_valid()
            host = frame.image.numpy()
            np.testing.assert_array_equal(host[..., 1:], pattern(0)[..., 1:])
            assert host[0, 0, 0] == host[-1, -1, 0] and 1 <= host[0, 0, 0] <= 5
            sub.close()
        same = ImageSubscriber(session, "t/x/color", "auto", device=0)
        assert wait_frame(same, 60).image.device == 0 and same.transport_reason == "cuda (publisher on this GPU)"
    finally:
        n.value = 0
        session.close()
        child.join(10)


def _read_with_hidden_gpu(port, base, result):
    """Child that sees only the second GPU: what auto takes, and whether a frame came."""
    session = open_session(connect=[f"127.0.0.1:{port}"])
    sub = ImageSubscriber(session, base, "auto", device=0)  # ordinal 0 of the one device it sees
    frame = wait_frame(sub, 60)
    result.put((sub.transport_reason, isinstance(frame.image, np.ndarray)))
    session.close()


@needs_two_gpus
def test_a_reader_that_does_not_see_the_publishers_gpu_takes_the_host_ring(monkeypatch):
    ctx = mp.get_context("spawn")
    port = free_port()
    n = ctx.Value("i", 5)
    ready = ctx.Event()
    child = ctx.Process(target=_publish_host_frames, args=(port, "t/y/color", n, ready), daemon=True)
    child.start()
    assert ready.wait(60)
    ring_gpu = _core.cuda_device_uuid(0)
    result = ctx.Queue()
    monkeypatch.setenv("CUDA_VISIBLE_DEVICES", "GPU-" + _uuid_text(_core.cuda_device_uuid(1)))  # inherited by the child
    reader = ctx.Process(target=_read_with_hidden_gpu, args=(port, "t/y/color", result), daemon=True)
    reader.start()
    try:
        reason, on_host = result.get(timeout=90)
        assert reason == f"shm (publisher's ring is on GPU {ring_gpu[:8]}, which this process does not see)"
        assert on_host
    finally:
        n.value = 0
        reader.join(10)
        child.join(10)


def _uuid_text(u: str) -> str:
    """32 hex digits as nvidia-smi and CUDA_VISIBLE_DEVICES write a GPU's UUID."""
    return f"{u[:8]}-{u[8:12]}-{u[12:16]}-{u[16:20]}-{u[20:]}"


class _Driver:
    """The CUDA driver through ctypes, in a test's child process: a producer as PyTorch is one, its frames made by work
    on a non-blocking stream of its own (Durchblick depends on no array library)."""

    def __init__(self):
        self.c = ctypes
        self.lib = ctypes.CDLL("libcuda.so.1")
        dev, ctx = ctypes.c_int(), ctypes.c_void_p()
        self.check(self.lib.cuInit(0))
        self.check(self.lib.cuDeviceGet(ctypes.byref(dev), 0))
        self.check(self.lib.cuDevicePrimaryCtxRetain(ctypes.byref(ctx), dev))
        self.check(self.lib.cuCtxSetCurrent(ctx))
        stream = ctypes.c_void_p()
        self.check(self.lib.cuStreamCreate(ctypes.byref(stream), 1))  # CU_STREAM_NON_BLOCKING
        self.stream = stream.value

    def check(self, result):
        if result != 0:
            raise RuntimeError(f"CUDA driver error {result}")

    def alloc(self, nbytes: int) -> int:
        ptr = self.c.c_uint64()
        self.check(self.lib.cuMemAlloc_v2(self.c.byref(ptr), self.c.c_size_t(nbytes)))
        return ptr.value

    def memset(self, ptr: int, value: int, nbytes: int):
        """Enqueued on the producer's stream."""
        self.check(self.lib.cuMemsetD8Async(self.c.c_uint64(ptr), self.c.c_ubyte(value), self.c.c_size_t(nbytes),
                                            self.c.c_void_p(self.stream)))


class _TorchLike:
    """A device array whose interface names no stream, as PyTorch's."""

    def __init__(self, ptr: int, shape: tuple):
        self.__cuda_array_interface__ = {"shape": shape, "typestr": "|u1", "data": (ptr, False), "version": 2}


def _publish_late_device_frames(port, base, mode, ready, stop):
    """Child: frames whose value (their number, 1, 2, ...) is written on a non-blocking stream after a few ms of other
    work there, published at once from the host, and overwritten with 255 on that stream as soon as publish returns
    (PyTorch's allocator giving their memory to the next frame). mode "unknown": publish is not told the stream;
    "stream": it is; "legacy" (a control, not a test): it is told the legacy default stream, wrongly. Until ``stop``."""
    driver = _Driver()
    shape = (480, 640)
    nbytes = shape[0] * shape[1]
    image, scratch = driver.alloc(nbytes), driver.alloc(256 << 20)
    session = open_session(listen=[f"127.0.0.1:{port}"])
    pub = CameraPublisher(session, base, encoding="mono8", frame_id="cam", cuda_device=0)
    ready.set()
    end, i = time.monotonic() + 30, 0
    while time.monotonic() < end and not stop.is_set():
        if not pub.subscribed:
            time.sleep(0.02)
            continue
        i = i % 200 + 1
        for _ in range(30):  # a few ms of work before the frame's own
            driver.memset(scratch, i, 256 << 20)
        driver.memset(image, i, nbytes)
        stream = {"unknown": None, "stream": driver.stream, "legacy": 0}[mode]
        pub.publish(_TorchLike(image, shape), stream=stream)
        driver.memset(image, 255, nbytes)
        time.sleep(0.02)
    pub.close()
    session.close()


def late_device_frames(mode, frames=20):
    """The frames _publish_late_device_frames gave: (number, its pixels' values) each."""
    ctx = mp.get_context("spawn")
    port = free_port()
    ready, stop = ctx.Event(), ctx.Event()
    child = ctx.Process(target=_publish_late_device_frames, args=(port, "t/late/color", mode, ready, stop),
                        daemon=True)
    child.start()
    assert ready.wait(60)
    session = open_session(connect=[f"127.0.0.1:{port}"])
    got = []
    try:
        sub = ImageSubscriber(session, "t/late/color", "cuda")
        while len(got) < frames:
            frame = wait_frame(sub, 60)
            got.append((frame.seq, set(np.unique(frame.image.numpy()).tolist())))
    finally:
        session.close()
        stop_child(child, stop)
    return got


@needs_cuda
@pytest.mark.parametrize("mode", ["unknown", "stream"])
def test_device_frames_published_complete_and_before_reuse(mode):
    """A device frame goes out as its producer finished it, also when publish is not told the producer's stream, and
    publish has copied it before it returns: none comes out unfinished or overwritten."""
    got = late_device_frames(mode)
    wrong = [(seq, values) for seq, values in got if values != {(seq - 1) % 200 + 1}]
    assert not wrong, f"frames not as published (number, values): {wrong}"


# ------------------------------------------------------------------------ protocol versions, info, auto


def test_descriptor_encodings_carry_their_version():
    assert ShmImage.ENCODING == "application/cdr;xblick_msgs/msg/ShmImage@1"
    assert CudaImageMsg.ENCODING == "application/cdr;xblick_msgs/msg/CudaImage@1"
    assert xblick.Image.ENCODING == "application/cdr;sensor_msgs/msg/Image"  # ROS types: the plain ROS name


def test_other_protocol_version_is_refused(sessions):
    import zenoh
    a, b = sessions
    sub = ImageSubscriber(b, "t/v/color", "shm")
    pub = a.declare_publisher(f"t/v/color/image_raw/shm/{_core.host_id()}",
                              encoding=zenoh.Encoding("application/cdr;xblick_msgs/msg/ShmImage@2"))
    end = time.monotonic() + 10
    with pytest.raises(xblick.ProtocolError, match="ShmImage@2"):
        while time.monotonic() < end:
            pub.put(b"\x00\x01\x00\x00")
            sub.latest(timeout=0.05)
    pub.undeclare()


def test_info_record(sessions):
    a, b = sessions
    pub = CameraPublisher(a, "t/i/color", encoding="bgr8", frame_id="cam", source="test")
    end = time.monotonic() + 10
    records = []
    while not records and time.monotonic() < end:
        records = xblick.query_info(b, "t/i/color", timeout=0.5)
    (record,) = records
    assert record["host"] == _core.host_id() and record["encoding"] == "bgr8" and record["source"] == "test"
    assert record["transports"] == ["raw", "compressed", "shm"] and record["xblick"] == xblick.__version__
    pub.close()


def test_auto_takes_the_ring_on_the_same_host(sessions):
    a, b = sessions
    sub = ImageSubscriber(b, "t/a1/color", "auto", copy=False)  # before the publisher exists
    assert sub.transport is None and sub.latest() is None
    pub = CameraPublisher(a, "t/a1/color", encoding="rgb8", frame_id="cam")
    frame = publish_until(pub, pattern(4), sub)
    assert sub.transport == "shm" and frame.is_valid()
    np.testing.assert_array_equal(frame.image, pattern(4))


def test_auto_takes_the_network_for_another_host(sessions):
    a, b = sessions
    pub = CameraPublisher(a, "t/a2/color", encoding="rgb8", frame_id="cam")
    pub._host = "another-host"  # as a publisher on another machine announces itself
    sub = ImageSubscriber(b, "t/a2/color", "auto", remote="raw")
    frame = publish_until(pub, pattern(5), sub)
    assert sub.transport == "raw"
    np.testing.assert_array_equal(frame.image, pattern(5))
