"""Live cameras by xblick cam (cam.py), with FFmpeg's lavfi test source as the camera (no device needed; the
filter ``realtime`` makes it deliver at its rate, as a camera does): frames at the size and rate asked for, stamps of
their arrival, camera_info and the info record; the modes a camera does not deliver, a missing device, a stream that
ends, a missing PyAV refused in one line; the command's parameters; Ctrl-C ends it at once and leaves no ring."""

import os
import signal
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
