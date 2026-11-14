"""Live cameras by xblick cam (cam.py), with FFmpeg's lavfi test source as the camera (no device needed; the
filter ``realtime`` makes it deliver at its rate, as a camera does): frames at the size and rate asked for, stamps of
their arrival, camera_info and the info record; the modes a camera does not deliver, a missing device, a stream that
ends, a missing PyAV refused in one line; the command's parameters; Ctrl-C ends it at once and leaves no ring."""

import os
import signal
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np
import pytest
from test_camera import free_port, sessions, wait_until  # noqa: F401 - sessions is a fixture

from xblick import _core, cam, topics
from xblick.camera import ImageSubscriber, query_info
from xblick.history import FrameHistory
from xblick.msg import CompressedImage, ShmImage
from xblick.parameters import ParameterClient, find_nodes

av = pytest.importorskip("av")  # the extra xblick[cam]
if "lavfi" not in av.formats_available:
    pytest.skip("PyAV without lavfi", allow_module_level=True)

MS = 1_000_000


def source(w=320, h=240, rate=30, duration=None):
    """A lavfi test pattern delivering ``rate`` frames a second, as a camera does."""
    return f"testsrc=size={w}x{h}:rate={rate}" + (f":duration={duration}" if duration else "") + ",realtime"


def run_in_thread(camera):
    """``camera.run()`` in a thread; its error in the returned list."""
    errors = []

    def run():
        try:
            camera.run()
        except BaseException as error:  # noqa: BLE001 - the test looks at it
            errors.append(error)

    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    return thread, errors


def test_frames_arrive_at_the_size_and_rate_asked_for(sessions):  # noqa: F811
    a, b = sessions
    camera = cam.LiveCamera(a, source(), "t/lav/color", fmt="lavfi", size=(320, 240), fps=30.0)
    images = ImageSubscriber(b, "t/lav/color", transport="raw", camera_info=True)
    history = FrameHistory(images, 200)
    thread, errors = run_in_thread(camera)
    try:
        wait_until(lambda: len(history.stamps()) >= 45, timeout=10.0)
        stamps = sorted(history.stamps())
        steps = np.diff(stamps[-30:])
        assert np.median(steps) == pytest.approx(33.3 * MS, rel=0.1), steps  # 30 frames/s
        frame = history.at(stamps[-1])
        assert frame.image.shape == (240, 320, 3) and frame.encoding == "rgb8"
        assert frame.meta.get("source") == "cam" and frame.header.frame_id == "t_lav_color"
        assert abs(time.time_ns() - frame.stamp_ns) < 1000 * MS  # the arrival on this PC, wall clock
        # without a calibration file: a pinhole of 60 degrees horizontal field of view (video.py's default)
        wait_until(lambda: images.camera_info is not None)
        info = images.camera_info
        f = 0.5 * 320 / np.tan(np.radians(60) / 2)
        assert (info.width, info.height) == (320, 240)
        np.testing.assert_allclose(info.k, [f, 0, 159.5, 0, f, 119.5, 0, 0, 1])
        (record,) = query_info(b, "t/lav/color", timeout=1.0)
        assert record["source"] == "cam" and record["encoding"] == "rgb8"
        assert camera.delivered == cam.Mode((320, 240), 30.0, "rgb24")
    finally:
        history.close()
        camera.stop()
        thread.join(5.0)
    assert not errors and not thread.is_alive()


def test_a_calibration_file_of_the_delivered_size(sessions, tmp_path):  # noqa: F811
    from xblick.msg import CameraInfo

    a, b = sessions
    calibration = tmp_path / "cam.yaml"
    calibration.write_text(CameraInfo.pinhole(320, 240, 300.0, 301.0, 160.0, 120.0).to_yaml("webcam"))
    camera = cam.LiveCamera(a, source(), "t/cal/color", fmt="lavfi", camera_info=str(calibration),
                            frame_id="webcam")
    images = ImageSubscriber(b, "t/cal/color", transport="raw", camera_info=True)
    thread, errors = run_in_thread(camera)
    try:
        wait_until(lambda: images.camera_info is not None, timeout=10.0)
        info = images.camera_info
        assert info.k[0] == 300.0 and info.k[4] == 301.0 and info.header.frame_id == "webcam"
    finally:
        images.close()
        camera.stop()
        thread.join(5.0)
    assert not errors


@pytest.mark.parametrize("options, message", [
    ({"size": (640, 480)}, "asked for 640x480, the camera delivers 320x240 rgb24 at 30 frames/s"),
    ({"fps": 15.0}, "asked for 15 frames/s, the camera delivers 320x240 rgb24 at 30 frames/s"),
    ({"size": (640, 480), "fps": 60.0}, "asked for 640x480 60 frames/s, the camera delivers 320x240 rgb24 at 30"),
])
def test_a_mode_the_camera_does_not_deliver_ends_it(sessions, options, message):  # noqa: F811
    a, _ = sessions
    camera = cam.LiveCamera(a, source(), "t/mode/color", fmt="lavfi", **options)
    with pytest.raises(cam.CamError, match=message):
        camera.run()
    assert camera.publisher is None  # nothing was published


def test_what_is_refused(sessions, tmp_path):  # noqa: F811
    a, _ = sessions
    with pytest.raises(cam.CamError, match="for V4L2 devices"):
        cam.LiveCamera(a, source(), "t/x/color", fmt="lavfi", pixel_format="mjpeg")
    missing = "/dev/video-xblick-test-none"
    with pytest.raises(cam.CamError, match=f"{missing}: no such device \\(V4L2 devices on this PC: "):
        cam.LiveCamera(a, missing, "t/x/color")
    video = tmp_path / "clip.mp4"
    video.write_bytes(b"\x00")
    with pytest.raises(cam.CamError, match="a file. xblick play plays video files"):
        cam.LiveCamera(a, str(video), "t/x/color")
    with pytest.raises(cam.CamError, match="a V4L2 device"):
        cam.LiveCamera(a, "camera7", "t/x/color")
    with pytest.raises(cam.CamError, match="FFmpeg cannot open it"):
        cam.LiveCamera(a, "no_such_filter", "t/x/color", fmt="lavfi").run()
    # a stream that ends (unplugged, a URL's stream over) ends the program: no reconnect
    camera = cam.LiveCamera(a, source(duration=0.5), "t/x/color", fmt="lavfi")
    with pytest.raises(cam.CamError, match="the stream ended"):
        camera.run()
    assert camera.frames > 0 and camera.publisher is None
    # the device's modes and the asked ones compared: a V4L2 pixel format too
    cam.check_mode("/dev/video0", cam.Mode(pixel_format="mjpeg"), cam.Mode((640, 480), 30.0, "mjpeg"))
    with pytest.raises(cam.CamError, match="asked for mjpeg, the camera delivers 640x480 yuyv422 at 30 frames/s"):
        cam.check_mode("/dev/video0", cam.Mode(pixel_format="mjpeg"), cam.Mode((640, 480), 30.0, "yuyv422"))
    with pytest.raises(cam.CamError, match="asked for 30 frames/s, the camera delivers 640x480 h264 at an unknown"):
        cam.check_mode("rtsp://x", cam.Mode(fps=30.0), cam.Mode((640, 480), None, "h264"))


def test_without_pyav_the_command_names_the_extra(monkeypatch, capsys):
    import builtins

    from xblick import cli

    real_import = builtins.__import__

    def no_av(name, *args, **kwargs):
        if name == "av":
            raise ImportError("No module named 'av'")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", no_av)
    with pytest.raises(SystemExit, match=r"xblick cam: xblick cam needs PyAV: install xblick\[cam\] "
                                         r"\(pip install 'xblick\[cam\]'"):
        cli.main(["cam", "/dev/video0", "--stream", "t/x/color"])


def test_the_command_its_parameters_and_ctrl_c(tmp_path):
    """xblick cam as a program: its options as the parameters of the node <camera>; Ctrl-C ends it within a second,
    exit 0, its ring removed from /dev/shm."""
    from xblick.session import open_session

    port = free_port()
    base = "t/cmd/color"
    process = subprocess.Popen([sys.executable, "-m", "xblick.cli", "cam", source(), "--format", "lavfi", "--stream",
                                base, "--fps", "30", "--listen", f"tcp/127.0.0.1:{port}"],
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    session = open_session(connect=[f"tcp/127.0.0.1:{port}"])
    segments = set()
    try:
        # the shm descriptors name the ring; subscribing to them makes the publisher write it
        subscriber = session.declare_subscriber(topics.shm(base, _core.host_id()),
                                                lambda s: segments.add(ShmImage.deserialize(s.payload.to_bytes())
                                                                       .segment))
        wait_until(lambda: segments, timeout=20.0)
        (segment,) = segments
        ring = Path("/dev/shm") / segment.lstrip("/")
        assert ring.exists()
        wait_until(lambda: "t/cmd" in find_nodes(session, 0.3), timeout=10.0)
        client = ParameterClient(session, "t/cmd")
        values = dict(zip(["fps", "format", "stream", "source"], (v.value() for v in client.get(
            ["fps", "format", "stream", "source"]))))
        assert values == {"fps": 30.0, "format": "lavfi", "stream": base, "source": source()}
        subscriber.undeclare()
        process.send_signal(signal.SIGINT)
        started = time.monotonic()
        out, _ = process.communicate(timeout=10)
        assert time.monotonic() - started < 1.0, out
        assert process.returncode == 0, out
        assert "Traceback" not in out, out
        assert not ring.exists()
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        session.close()


def test_a_camera_that_stops_delivering_ends_the_command():
    """No frame for --timeout seconds (a camera unplugged, a network stream that stalls): an error, exit 1. The
    source waits 5 s between frames (realtime pauses up to its limit, 20 s)."""
    out = subprocess.run([sys.executable, "-m", "xblick.cli", "cam", "testsrc=size=64x48:rate=0.2,realtime=limit=20",
                          "--format", "lavfi", "--stream", "t/slow/color", "--timeout", "1", "--listen",
                          f"tcp/127.0.0.1:{free_port()}"], capture_output=True, text=True, timeout=30,
                         env={**os.environ, "PYTHONUNBUFFERED": "1"})
    assert out.returncode == 1, out
    assert out.stderr.strip().endswith("no frame for 1 s (waiting for a frame; the camera stopped, or raise "
                                       "--timeout)"), out.stderr


# ------------------------------------------------------------------------------ choosing a camera without a source

# struct v4l2_capability as uvcvideo fills it for a notebook's webcam: /dev/video0 captures video, /dev/video1 is its
# metadata node (the values v4l2-ctl --info prints for such a camera: capabilities 0x84a00001, device caps 0x04200001
# and 0x04a00000)
UVC_CAPTURE = struct.pack("<16s32s32sIII12x", b"uvcvideo", b"Integrated Camera: Integrated C",
                          b"usb-0000:00:14.0-8", 0x0006080F, 0x84A00001, 0x04200001)
UVC_METADATA = struct.pack("<16s32s32sIII12x", b"uvcvideo", b"Integrated Camera: Integrated C",
                           b"usb-0000:00:14.0-8", 0x0006080F, 0x84A00001, 0x04A00000)
# an old driver without device_caps: the device's capabilities count
OLD_CAPTURE = struct.pack("<16s32s32sIII12x", b"bttv", b"BT878 video", b"PCI:0000:05:00.0", 0x00000300,
                          0x05000001, 0)


def test_capture_and_metadata_nodes_told_apart():
    from xblick import v4l2

    assert v4l2.CAPABILITY.size == 104 and v4l2.VIDIOC_QUERYCAP == 0x80685600  # as linux/videodev2.h
    capture = v4l2.parse_capability(UVC_CAPTURE)
    assert (capture.driver, capture.card, capture.bus_info) == ("uvcvideo", "Integrated Camera: Integrated C",
                                                                "usb-0000:00:14.0-8")
    assert capture.captures_video
    assert not v4l2.parse_capability(UVC_METADATA).captures_video
    assert v4l2.parse_capability(OLD_CAPTURE).captures_video
    with pytest.raises(ValueError, match="104 bytes"):
        v4l2.parse_capability(UVC_CAPTURE[:100])


def test_the_device_listing(tmp_path, monkeypatch):
    """Nodes in number order, the name from VIDIOC_QUERYCAP or else from sysfs, metadata nodes left out of the
    choice, a node that cannot be asked kept with the reason."""
    from xblick import v4l2

    dev, sysfs = tmp_path / "dev", tmp_path / "sys"
    for node, name in (("video0", "Integrated Camera: Integrated C"), ("video1", "Integrated Camera: Integrated C"),
                       ("video10", "USB Capture HDMI"), ("video2", "Loopback")):
        (dev / node).parent.mkdir(exist_ok=True)
        (dev / node).write_bytes(b"")
        (sysfs / node).mkdir(parents=True)
        (sysfs / node / "name").write_text(name + "\n")
    answers = {"video0": UVC_CAPTURE, "video1": UVC_METADATA, "video2": OLD_CAPTURE}

    def query(path):
        if path.name not in answers:
            raise PermissionError(13, "Permission denied")
        return v4l2.parse_capability(answers[path.name])

    monkeypatch.setattr(v4l2, "query", query)
    listed = v4l2.devices(dev, sysfs)
    assert [(d.path[-7:], d.name, d.captures) for d in listed] == [
        ("/video0", "Integrated Camera: Integrated C", True), ("/video1", "Integrated Camera: Integrated C", False),
        ("/video2", "BT878 video", True), ("video10", "USB Capture HDMI", None)]
    assert listed[3].why == "capabilities unknown: Permission denied"
    assert [d.path[-7:] for d in v4l2.capture_devices(listed)] == ["/video0", "/video2", "video10"]
    # a plain file answers no ioctl: kept, capabilities unknown
    monkeypatch.undo()
    assert v4l2.devices(dev, sysfs)[0].captures is None


CHOOSER = """
import sys
from xblick import cam, cli, v4l2
v4l2.devices = lambda: [v4l2.Device("/dev/video0", "Integrated Camera: Integrated C", True),
                        v4l2.Device("/dev/video1", "Integrated Camera: Integrated C", False),
                        v4l2.Device("/dev/video2", "USB Capture HDMI", True)]
class Started:
    def __init__(self, session, source, base, **options):
        print(f"STARTED {source} {base}", flush=True)
        print(f"CAMERA {cam.camera_for(source, cam.input_format(source, None), base)}", flush=True)
    def run(self):
        pass
cam.LiveCamera = Started
sys.exit(cli.main(["cam", *sys.argv[2:], "--listen", f"tcp/127.0.0.1:{sys.argv[1]}"]))
"""


def choose_in_a_pty(tmp_path, keys: bytes | None, options=("--stream", "t/sel/color")) -> tuple[int, str, bool]:
    """CHOOSER in a pseudo-terminal, ``keys`` typed once the list is shown (None: SIGTERM sent instead): (exit code,
    output, whether the terminal's mode is as before)."""
    import pty
    import select
    import termios

    script = tmp_path / "chooser.py"
    script.write_text(CHOOSER)
    master, slave = pty.openpty()
    before = termios.tcgetattr(slave)
    process = subprocess.Popen([sys.executable, str(script), str(free_port()), *options], stdin=slave,
                               stdout=slave, stderr=slave, close_fds=True)
    out = b""
    try:
        end = time.monotonic() + 30
        while b"choose a camera" not in out and time.monotonic() < end:
            if select.select([master], [], [], 0.1)[0]:
                out += os.read(master, 4096)
        assert b"choose a camera" in out, out
        if keys is None:
            time.sleep(0.2)
            process.send_signal(signal.SIGTERM)
        for key in ([keys[:3], keys[3:]] if keys.startswith(b"\x1b[") else [keys]) if keys else []:  # arrow, Enter
            time.sleep(0.2)
            os.write(master, key)
        while process.poll() is None and time.monotonic() < end:
            if select.select([master], [], [], 0.1)[0]:
                out += os.read(master, 4096)
        process.wait(timeout=10)
        while select.select([master], [], [], 0.1)[0]:
            try:
                out += os.read(master, 4096)
            except OSError:
                break
        restored = termios.tcgetattr(slave) == before
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        os.close(master)
        os.close(slave)
    return process.returncode, out.decode(errors="replace"), restored


def test_down_and_enter_chooses_the_second_camera(tmp_path):
    code, out, restored = choose_in_a_pty(tmp_path, b"\x1b[B\r")
    assert "  /dev/video2  USB Capture HDMI" in out and "/dev/video1" not in out  # the metadata node not offered
    assert "xblick cam: /dev/video2  USB Capture HDMI" in out
    assert "STARTED /dev/video2 t/sel/color" in out
    assert code == 0 and restored and out.rstrip().endswith("CAMERA t/sel"), out
    assert "\x1b[?25h" in out  # the cursor shown again


def test_plain_xblick_cam_names_the_stream_after_the_device(tmp_path):
    code, out, restored = choose_in_a_pty(tmp_path, b"\r", options=())
    assert "STARTED /dev/video0 None" in out and out.rstrip().endswith("CAMERA cam/video0"), out
    assert code == 0 and restored


@pytest.mark.parametrize("key", [b"\x1b", b"q", None])
def test_esc_q_or_sigterm_starts_nothing(tmp_path, key):
    code, out, restored = choose_in_a_pty(tmp_path, key)
    assert code == 1 and restored and "STARTED" not in out
    assert out.rstrip().endswith("xblick cam: no camera chosen"), out


def test_without_a_terminal_the_source_must_be_given(tmp_path):
    script = tmp_path / "chooser.py"
    script.write_text(CHOOSER)
    out = subprocess.run([sys.executable, str(script), str(free_port()), "--stream", "t/sel/color"],
                         stdin=subprocess.DEVNULL,
                         capture_output=True, text=True, timeout=30)
    assert out.returncode == 1 and "STARTED" not in out.stdout
    assert out.stderr == ("xblick cam: no source given, and no terminal to choose one in: give the source, one of "
                          "/dev/video0  Integrated Camera: Integrated C; /dev/video2  USB Capture HDMI\n")


def test_an_invalid_stream_is_refused_before_the_list(capsys):
    from xblick import cli

    with pytest.raises(SystemExit, match="xblick cam: stream 'color': <camera>/<stream>"):
        cli.main(["cam", "--stream", "color"])


def test_the_terminal_mode_is_restored_after_an_exception():
    import pty
    import termios

    from xblick import v4l2

    class Broken:
        def write(self, text):
            raise OSError("the terminal went away")

        def flush(self):
            pass

    master, slave = pty.openpty()
    try:
        before = termios.tcgetattr(slave)
        with pytest.raises(OSError, match="went away"):
            cam.choose([v4l2.Device("/dev/video0", "a", True)], slave, Broken(), "choose")
        assert termios.tcgetattr(slave) == before
    finally:
        os.close(master)
        os.close(slave)


def test_default_stream_names(sessions):  # noqa: F811
    """No --stream: cam/<device node>/color for a V4L2 device, cam/camera/color for other sources, /mono for a grey
    camera (known at the first frame); the parameters node is the camera."""
    a, b = sessions
    assert cam.camera_for("/dev/video0", cam.V4L2, None) == "cam/video0"
    assert cam.camera_for("rtsp://192.0.2.20/stream1", None, None) == "cam/camera"
    assert cam.camera_for("/dev/video0", cam.V4L2, "lab/door/color") == "lab/door"
    with pytest.raises(cam.CamError, match="give --stream"):
        cam.camera_for("/dev/v4l/by-id/usb-Chicony-video-index0", cam.V4L2, None)
    for graph, stream, encoding in ((source(), "cam/camera/color", "rgb8"),
                                    ("testsrc=size=320x240:rate=30,format=gray,realtime", "cam/camera/mono", "mono8")):
        camera = cam.LiveCamera(a, graph, fmt="lavfi")
        assert camera.camera == "cam/camera" and camera.base is None
        thread, errors = run_in_thread(camera)
        try:
            wait_until(lambda: camera.frames > 0, timeout=10.0)
            assert camera.base == stream and camera.frame_id == stream.replace("/", "_")
            frame = ImageSubscriber(b, stream, transport="raw").latest(timeout=5.0)
            assert frame is not None and frame.encoding == encoding
        finally:
            camera.stop()
            thread.join(5.0)
        assert not errors


# ------------------------------------------------------------------------------------- reconnect, FFmpeg options

def mjpeg_clip(path, w=160, h=120, n=20, fps=10, container="matroska"):
    """``n`` MJPEG frames (frame i: red i*10) in a file, for an HTTP server to serve as a camera's stream."""
    with av.open(str(path), "w", format=container) as out:
        s = out.add_stream("mjpeg", rate=fps)
        s.width, s.height, s.pix_fmt = w, h, "yuvj422p"
        for i in range(n):
            image = np.zeros((h, w, 3), np.uint8)
            image[..., 0] = i * 10
            for packet in s.encode(av.VideoFrame.from_ndarray(image, format="rgb24")):
                out.mux(packet)
        for packet in s.encode():
            out.mux(packet)
    return path


class HttpCamera:
    """An HTTP server of a directory in a thread: a camera that ends its stream after each file (a loss for xblick
    cam), and that can be stopped and started again on its port (away and back)."""

    def __init__(self, directory):
        import functools
        import http.server

        self.port = free_port()
        self.handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory=str(directory))
        self.handler.log_message = lambda *a: None
        self.server = None
        self.start()

    def start(self):
        import http.server

        self.server = http.server.ThreadingHTTPServer(("127.0.0.1", self.port), self.handler)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def stop(self):
        self.server.shutdown()
        self.server.server_close()

    def url(self, name):
        return f"http://127.0.0.1:{self.port}/{name}"


def test_reconnect_after_the_stream_ended_and_while_the_camera_is_away(sessions, tmp_path):  # noqa: F811
    from xblick import diagnostics

    a, b = sessions
    mjpeg_clip(tmp_path / "cam.mkv")
    server = HttpCamera(tmp_path)
    statuses = []
    sub = b.declare_subscriber("cam/camera/diagnostics", lambda s: statuses.append(diagnostics.read(s)[1].status[0]))
    camera = cam.LiveCamera(a, server.url("cam.mkv"), reconnect=0.3, timeout=2.0)
    thread, errors = run_in_thread(camera)
    try:
        wait_until(lambda: camera.returns >= 2, timeout=15.0)  # the stream ended, came back, twice
        assert camera.frames >= 40 and camera.base == "cam/camera/color"
        server.stop()  # away: every attempt is refused
        wait_until(lambda: camera.away is not None and "Connection refused" in camera.away, timeout=10.0)
        wait_until(lambda: any(s.level == diagnostics.DiagnosticsPublisher.WARN and "camera away" in s.message
                               for s in statuses), timeout=5.0)
        frames, returns = camera.frames, camera.returns
        server.start()
        wait_until(lambda: camera.frames > frames and camera.returns > returns, timeout=10.0)
        # back with another mode: the stream's subscribers and ring were set up for the first: an error
        mjpeg_clip(tmp_path / "cam.mkv", w=320, h=240)
        wait_until(lambda: errors, timeout=15.0)
        assert "back with 320x240 mjpeg" in str(errors[0]) and "before 160x120 mjpeg" in str(errors[0])
    finally:
        camera.stop()
        thread.join(5.0)
        sub.undeclare()
        server.stop()


def test_reconnect_waits_for_a_camera_that_is_not_there_yet(sessions, tmp_path):  # noqa: F811
    """The first opening is tried again too (a robot that starts before its IP camera)."""
    a, _ = sessions
    mjpeg_clip(tmp_path / "late.mkv")
    server = HttpCamera(tmp_path)
    server.stop()
    camera = cam.LiveCamera(a, server.url("late.mkv"), reconnect=0.2, timeout=2.0)
    thread, errors = run_in_thread(camera)
    try:
        wait_until(lambda: camera.away is not None, timeout=5.0)
        assert camera.publisher is None
        server.start()
        wait_until(lambda: camera.frames > 0, timeout=10.0)
    finally:
        camera.stop()
        thread.join(5.0)
        server.stop()
    assert not errors


def test_ctrl_c_while_reconnecting(tmp_path):
    port = free_port()
    process = subprocess.Popen([sys.executable, "-m", "xblick.cli", "cam", f"http://127.0.0.1:{port}/none.mkv",
                                "--reconnect", "0.5", "--listen", f"tcp/127.0.0.1:{free_port()}"],
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        time.sleep(3.0)  # failed attempts meanwhile
        process.send_signal(signal.SIGINT)
        started = time.monotonic()
        out, _ = process.communicate(timeout=10)
        assert time.monotonic() - started < 1.0 and process.returncode == 0, out
        assert out.count("lost (FFmpeg cannot open it") == 1 and "Traceback" not in out, out
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()


def test_ffmpeg_options_are_checked_and_passed(sessions):  # noqa: F811
    a, b = sessions
    with pytest.raises(cam.ffopts.OptionError, match="--option bogus: FFmpeg has no such option in FFmpeg's generic "
                                                     "input options, the input format lavfi"):
        cam.LiveCamera(a, source(), fmt="lavfi", options={"bogus": "1"})
    with pytest.raises(cam.ffopts.OptionError, match="stimeout"):  # gone from FFmpeg since 5.0
        cam.LiveCamera(a, "rtsp://192.0.2.1/x", options={"stimeout": "1000000"})
    cam.LiveCamera(a, "rtsp://192.0.2.1/x", options={"rtsp_transport": "tcp", "timeout": "1000000",
                                                     "fflags": "nobuffer"})
    with pytest.raises(cam.CamError, match="--option video_size: give it with --size"):
        cam.LiveCamera(a, "/dev/video0", size=(640, 480), options={"video_size": "1x1"}, reconnect=1.0)
    assert cam.parse_options(["rtsp_transport=tcp", "fflags=nobuffer"]) == {"rtsp_transport": "tcp",
                                                                            "fflags": "nobuffer"}
    with pytest.raises(cam.CamError, match="KEY=VALUE"):
        cam.parse_options(["tcp"])
    # an option FFmpeg has and takes: the camera runs with it
    camera = cam.LiveCamera(a, source(), fmt="lavfi", options={"fflags": "nobuffer"})
    thread, errors = run_in_thread(camera)
    try:
        wait_until(lambda: camera.frames > 5, timeout=10.0)
    finally:
        camera.stop()
        thread.join(5.0)
    assert not errors


# ----------------------------------------------------------------------------------------- MJPEG pass-through

def jpeg_of(image, subsampling=1, quality=85):
    """A JPEG as libjpeg (Pillow) writes it: the standard Huffman tables (no optimize), 4:2:2 (subsampling 1) as UVC
    cameras send."""
    import io

    from PIL import Image

    out = io.BytesIO()
    Image.fromarray(image).save(out, "JPEG", quality=quality, subsampling=subsampling, optimize=False)
    return out.getvalue()


def without_dht(jpeg: bytes) -> bytes:
    """The JPEG without its DHT segments, as a UVC camera sends a frame."""
    out, i = bytearray(jpeg[:2]), 2
    while True:
        marker, length = jpeg[i + 1], struct.unpack(">H", jpeg[i + 2:i + 4])[0]
        if marker == 0xDA:
            return bytes(out + jpeg[i:])
        if marker != 0xC4:
            out += jpeg[i:i + 2 + length]
        i += 2 + length


def decoded(jpeg: bytes) -> np.ndarray:
    import io

    from PIL import Image

    return np.asarray(Image.open(io.BytesIO(jpeg)).convert("RGB"))


def pattern_image(i=0, h=120, w=160):
    y, x = np.mgrid[0:h, 0:w]
    return np.stack([(x * 2 + i * 7) % 256, (y * 2) % 256, (x + y + i) % 256], -1).astype(np.uint8)


def test_huffman_tables_of_the_standard():
    """The inserted tables are those libjpeg writes (T.81 Annex K.3); a frame with tables passes byte-identical; a
    frame without them (a UVC camera's) decodes, with them inserted, to the pixels of the original (Pillow, an
    independent decoder); an AVI1 header stays; what is no JPEG is refused."""
    from xblick import mjpeg

    original = jpeg_of(pattern_image())
    assert mjpeg.with_huffman_tables(original) is original  # (1) unchanged, byte for byte
    stripped = without_dht(original)
    assert b"\xff\xc4" not in stripped[:mjpeg.scan_start(stripped)[0]]
    fixed = mjpeg.with_huffman_tables(stripped)
    np.testing.assert_array_equal(decoded(fixed), decoded(original))  # (2) pixel-equal
    # the DHT segment libjpeg wrote holds the same four tables as the standard one
    sos, _ = mjpeg.scan_start(original)
    dht = b"".join(original[i:i + 2 + struct.unpack(">H", original[i + 2:i + 4])[0]]
                   for i in range(2, sos) if original[i:i + 2] == b"\xff\xc4")
    def tables(segments):
        found, i = {}, 0
        while i < len(segments):
            length = struct.unpack(">H", segments[i + 2:i + 4])[0]
            body, i = segments[i + 4:i + 2 + length], i + 2 + length
            j = 0
            while j < len(body):
                n = sum(body[j + 1:j + 17])
                found[body[j]] = body[j + 1:j + 17 + n]
                j += 17 + n
        return found
    assert tables(dht) == tables(mjpeg.STANDARD_DHT) and len(tables(dht)) == 4
    # an AVI1 APP0 (Motion JPEG in AVI) in place of JFIF's: kept, the frame decodes
    app0 = 2 + struct.unpack(">H", stripped[4:6])[0] + 2
    avi1 = stripped[:2] + b"\xff\xe0\x00\x10AVI1" + bytes(10) + stripped[app0:]  # length 16: 2 + 4 + 10
    np.testing.assert_array_equal(decoded(mjpeg.with_huffman_tables(avi1)), decoded(original))
    for bad in (b"", b"\x00\x01\x02\x03", original[:40]):
        with pytest.raises(mjpeg.NotJpeg):
            mjpeg.with_huffman_tables(bad)


def test_mjpeg_passthrough_end_to_end(sessions, tmp_path):  # noqa: F811
    """(3) An MJPEG camera of frames without Huffman tables (concatenated JPEGs, FFmpeg's mjpeg format, over HTTP):
    a compressed subscriber gets each frame's JPEG exactly as with_huffman_tables makes it of the camera's frame,
    byte for byte. The check of the pixels: the received JPEG decoded as xblick cam decodes the camera's frame
    (FFmpeg, to rgb24) is the frame on the ring, exactly (same stamp). Decoded by Pillow (libjpeg-turbo) it differs
    from that by a mean below 2 levels: the two decoders upsample 4:2:2 chroma and convert colour differently (up to
    about 60 levels at sharp colour edges), as they do for a JPEG that xblick cam encodes itself."""
    from xblick import mjpeg

    frames = [without_dht(jpeg_of(pattern_image(i))) for i in range(30)]
    (tmp_path / "cam.mjpeg").write_bytes(b"".join(frames))
    expected = {mjpeg.with_huffman_tables(f) for f in frames}
    server = HttpCamera(tmp_path)
    a, b = sessions
    got, ring = [], []
    compressed = b.declare_subscriber("cam/camera/color/image_raw/compressed",
                                      lambda s: got.append(CompressedImage.deserialize(s.payload.to_bytes())))
    shm = ImageSubscriber(b, "cam/camera/color", transport="shm")
    camera = cam.LiveCamera(a, server.url("cam.mjpeg"), fmt="mjpeg", mjpeg="passthrough", reconnect=0.5,
                            options={"framerate": "10"})
    thread, errors = run_in_thread(camera)
    try:
        wait_until(lambda: len(got) >= 30, timeout=20.0)
        end = time.monotonic() + 10
        while len(ring) < 5 and time.monotonic() < end:
            frame = shm.latest(timeout=0.5)
            if frame is not None:
                ring.append(frame)
    finally:
        camera.stop()
        thread.join(5.0)
        compressed.undeclare()
        shm.close()
        server.stop()
    assert not errors
    assert all(bytes(m.data) in expected for m in got)  # the camera's JPEGs, byte for byte
    assert {m.format for m in got} == {"rgb8; jpeg compressed bgr8"}
    by_stamp = {m.header.stamp.to_ns(): m for m in got}
    pairs = [(f, by_stamp[f.stamp_ns]) for f in ring if f.stamp_ns in by_stamp]
    assert pairs
    for frame, message in pairs:
        import io

        with av.open(io.BytesIO(bytes(message.data)), format="mjpeg") as one:
            ffmpeg = next(one.decode(video=0)).to_ndarray(format="rgb24")
        np.testing.assert_array_equal(ffmpeg, frame.image)
        assert np.abs(decoded(bytes(message.data)).astype(int) - frame.image.astype(int)).mean() < 2.0


def test_passthrough_refuses_a_camera_without_mjpeg(sessions):  # noqa: F811
    a, _ = sessions
    with pytest.raises(cam.CamError, match="--mjpeg passthrough: the camera delivers 320x240 rgb24 at 30 frames/s "
                                           r"\(rgb8\), not MJPEG"):
        cam.LiveCamera(a, source(), fmt="lavfi", mjpeg="passthrough").run()


def run_for(argv, seconds, until):
    """A command as a program for at most ``seconds``, ended by SIGTERM once a line contains ``until``: its output."""
    process = subprocess.Popen([sys.executable, "-m", "xblick.cli", *argv], stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, text=True)
    lines, end = [], time.monotonic() + seconds
    try:
        while time.monotonic() < end:
            line = process.stdout.readline()
            if not line:
                break
            lines.append(line.rstrip("\n"))
            if until in line:
                break
    finally:
        process.send_signal(signal.SIGTERM)
        rest, _ = process.communicate(timeout=10)
    return lines + rest.splitlines(), process.returncode


@pytest.mark.parametrize("argv, stream, camera", [
    (["cam", "testsrc=size=64x48:rate=30,realtime", "--format", "lavfi"], "cam/camera/color", "cam/camera"),
    (["test-camera"], "test/camera/color", "test/camera"),
])
def test_the_stream_and_the_camera_for_a_node_are_printed(argv, stream, camera):
    """Both publishers say on lines of their own what they publish and what a node takes (nodes take the camera
    explicitly); a session that reached no router and no other session gets a hint once at the start."""
    lines, code = run_for([*argv, "--listen", f"tcp/127.0.0.1:{free_port()}"], 30, "start a node with")
    prog = "xblick " + argv[0]
    assert code == 0, lines
    # the camera to copy directly after the stream, so that the two are not taken for each other
    at = lines.index(f"{prog}: publishing the stream {stream}")
    assert lines[at + 1] == f"{prog}: start a node with --camera {camera} (the camera, not the stream)", lines
    hints = [line for line in lines if "nobody can receive this stream yet" in line]
    assert len(hints) == 1 and hints[0].startswith(f"{prog}: ") and \
        "start a router on this PC with xblick router" in hints[0], lines


def test_no_hint_when_the_session_reaches_another(sessions):  # noqa: F811
    """A session that reached a router or another session gets no hint (the router of test_tui)."""
    from test_tui import open_router

    port = free_port()
    router = open_router(port)
    try:
        lines, code = run_for(["test-camera", "--connect", f"tcp/127.0.0.1:{port}", "--listen", "tcp/127.0.0.1:0"],
                              30, "start a node with")
    finally:
        router.close()
    assert code == 0 and "xblick test-camera: publishing the stream test/camera/color" in lines, lines
    assert not any("nobody can receive" in line for line in lines), lines
