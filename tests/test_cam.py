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
from xblick.msg import ShmImage
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
    def run(self):
        pass
cam.LiveCamera = Started
sys.exit(cli.main(["cam", "--stream", "t/sel/color", "--listen", f"tcp/127.0.0.1:{sys.argv[1]}"]))
"""


def choose_in_a_pty(tmp_path, keys: bytes | None) -> tuple[int, str, bool]:
    """CHOOSER in a pseudo-terminal, ``keys`` typed once the list is shown (None: SIGTERM sent instead): (exit code,
    output, whether the terminal's mode is as before)."""
    import pty
    import select
    import termios

    script = tmp_path / "chooser.py"
    script.write_text(CHOOSER)
    master, slave = pty.openpty()
    before = termios.tcgetattr(slave)
    process = subprocess.Popen([sys.executable, str(script), str(free_port())], stdin=slave, stdout=slave,
                               stderr=slave, close_fds=True)
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
    assert code == 0 and restored and out.rstrip().endswith("STARTED /dev/video2 t/sel/color"), out
    assert "\x1b[?25h" in out  # the cursor shown again


@pytest.mark.parametrize("key", [b"\x1b", b"q", None])
def test_esc_q_or_sigterm_starts_nothing(tmp_path, key):
    code, out, restored = choose_in_a_pty(tmp_path, key)
    assert code == 1 and restored and "STARTED" not in out
    assert out.rstrip().endswith("xblick cam: no camera chosen"), out


def test_without_a_terminal_the_source_must_be_given(tmp_path):
    script = tmp_path / "chooser.py"
    script.write_text(CHOOSER)
    out = subprocess.run([sys.executable, str(script), str(free_port())], stdin=subprocess.DEVNULL,
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
