"""Videos played by xblick play (video.py through recording.Recorder.play): two generated videos of different
frame rates (a colour one at 20 frames/s, a 16-bit depth one at 10 frames/s, 3 ms later), played at once: their size
and encoding, stamps from their PTS on one clock (equal PTS: stamps exactly the offset apart), sent at their stamps,
paired by PartnerHistory within a maximum offset, looped with the stamps growing by the longest video's length, the
send skew, the rate changed through the node's parameters, pause and resume; an image sequence; what is refused."""

import time
from pathlib import Path

import numpy as np
import pytest
from test_camera import free_port, sessions, wait_until  # noqa: F401 - sessions is a fixture

from xblick import _core, recording
from xblick.camera import ImageSubscriber
from xblick.history import FrameHistory, PartnerHistory
from xblick.parameters import ParameterClient

av = pytest.importorskip("av")  # the extra xblick[video]

W, H = 64, 48
MS = 1_000_000


def write_video(path, fps, n, depth=False):
    """``n`` frames at ``fps``: colour frame i has red i*10, depth frame i the value 1000 + i (mm)."""
    with av.open(str(path), "w") as out:
        s = out.add_stream("ffv1" if depth else "libx264", rate=fps)
        s.width, s.height, s.pix_fmt = W, H, "gray16le" if depth else "yuv420p"
        for i in range(n):
            if depth:
                frame = av.VideoFrame.from_ndarray(np.full((H, W), 1000 + i, np.uint16), format="gray16le")
            else:
                image = np.zeros((H, W, 3), np.uint8)
                image[:, :, 0] = i * 10
                frame = av.VideoFrame.from_ndarray(image, format="rgb24")
            for packet in s.encode(frame):
                out.mux(packet)
        for packet in s.encode():
            out.mux(packet)
    return path


@pytest.fixture
def videos(tmp_path):
    return (write_video(tmp_path / "color.mp4", 20, 20), write_video(tmp_path / "depth.mkv", 10, 10, depth=True))


def test_two_videos_on_one_clock(sessions, videos, tmp_path):  # noqa: F811
    a, b = sessions
    color_mp4, depth_mkv = videos
    recorder = recording.Recorder(a, tmp_path, name="video_test")
    color = FrameHistory(ImageSubscriber(b, "t/cam/color", transport="raw"), 200)
    depth = PartnerHistory(ImageSubscriber(b, "t/cam/depth", transport="raw"), 200, max_offset_ns=5 * MS)
    try:
        player = recorder.play([color_mp4, depth_mkv], streams=["t/cam/color", "t/cam/depth"], offsets_ms=[0.0, 3.0],
                               loop=True)
        assert player.topics == ["t/cam/color", "t/cam/depth"]
        # two passes (1.0 s each: both videos are 1 s long) and a little
        wait_until(lambda: len(color.stamps()) >= 30, timeout=10.0)
        status = recorder.status()
        assert status["state"] == "playing" and status["play"]["kind"] == "video"
        stamps = sorted(color.stamps())
        steps = np.diff(stamps)
        # 50 ms apart, to rounding of the PTS (x264's time base); the pass's start 1.0 s after the first
        assert np.all(np.abs(steps - 50 * MS) < MS), steps
        assert stamps[20] - stamps[0] == pytest.approx(1000 * MS, abs=MS)  # looped: on by the length, growing
        newest = color.at(stamps[-1])
        assert newest.image.shape == (H, W, 3) and newest.encoding == "rgb8"
        assert newest.meta.get("source") == "video" and newest.age_at_send_ns is not None
        assert 0 <= newest.age_at_send_ns < 20 * MS  # sent at its stamp
        # every second colour frame has its depth frame 3 ms later, to the nanosecond of the offset
        wait_until(lambda: depth.partner_of(stamps[2]) is not None, timeout=5.0)
        for i in (0, 2, 4, 20, 22):
            partner = depth.partner_of(stamps[i])
            assert partner is not None and partner - stamps[i] == pytest.approx(3 * MS, abs=MS // 2)
            frame = depth.at(stamps[i])
            assert frame.encoding == "16UC1" and frame.image.dtype == np.uint16
            assert int(frame.image[0, 0]) == 1000 + (i % 20) // 2
        assert depth.partner_of(stamps[1]) is None or abs(depth.partner_of(stamps[1]) - stamps[1]) <= 5 * MS
        assert recorder.status()["play"]["skew_ms"] < 20.0
        assert recorder.status()["play"]["passes"] >= 1

        # the rate through the node's parameters: twice the pace, the stamps 25 ms apart from then on
        client = ParameterClient(b, "xblick/video_test")
        assert client.set({"rate": 2.0})[0].successful
        time.sleep(0.6)
        later = sorted(s for s in color.stamps() if s > stamps[-1])
        quick = np.diff(later[-6:])
        assert np.all(np.abs(quick - 25 * MS) < MS), quick
        # paused: nothing goes out; resumed: on from there, the stamps growing
        recorder.pause()
        time.sleep(0.2)
        held = len(color.stamps())
        time.sleep(0.4)
        assert len(color.stamps()) == held
        last = max(color.stamps())
        recorder.resume()
        wait_until(lambda: max(color.stamps()) > last, timeout=5.0)
        assert min(s for s in color.stamps() if s > last) - last >= 25 * MS
    finally:
        color.close()
        depth.close()
        recorder.close()
    recorder.check()


def test_an_image_sequence_and_what_is_refused(sessions, videos, tmp_path):  # noqa: F811
    a, b = sessions
    color_mp4, _ = videos
    frames = tmp_path / "frames"
    frames.mkdir()
    from PIL import Image

    for i in range(5):
        Image.fromarray(np.full((H, W, 3), i * 40, np.uint8)).save(frames / f"{i:03d}.png")
    recorder = recording.Recorder(a, tmp_path, name="video_test2")
    received = FrameHistory(ImageSubscriber(b, "t/seq/color", transport="raw"), 20)
    try:
        with pytest.raises(ValueError, match="give --fps"):
            recorder.play([frames], streams=["t/seq/color"])
        with pytest.raises(ValueError, match="one stream base per video"):
            recorder.play([color_mp4, color_mp4], streams=["t/a/color"])
        with pytest.raises(ValueError, match="one per video"):
            recorder.play([color_mp4], streams=["t/a/color"], offsets_ms=[1.0, 2.0])
        with pytest.raises(ValueError, match="for MCAP recordings, these are videos"):
            recorder.play([color_mp4], streams=["t/a/color"], keys=["x/*"])
        with pytest.raises(ValueError, match="videos need their stream bases"):
            recorder.play([color_mp4])
        fake_mcap = tmp_path / "rec.bin"  # MCAP by its magic bytes, whatever its name
        fake_mcap.write_bytes(b"\x89MCAP0\r\n" + bytes(64))
        with pytest.raises(ValueError, match="MCAP recordings and videos together"):
            recorder.play([fake_mcap, color_mp4], streams=["t/a/color"])
        with pytest.raises(ValueError, match="options for videos, these are MCAP recordings"):
            recorder.play([fake_mcap], streams=["t/a/color"], fps=10.0)
        with pytest.raises(ValueError, match="of 640x480, the stream is 64x48"):
            calibration = tmp_path / "cal.yaml"
            from xblick.msg import CameraInfo

            calibration.write_text(CameraInfo.pinhole(640, 480, 500, 500, 320, 240).to_yaml("cam"))
            recorder.play([color_mp4], streams=["t/a/color"], camera_infos=[calibration])
        player = recorder.play([str(frames)], streams=["t/seq/color"], fps=10.0, size=(32, 24))
        wait_until(lambda: len(received.stamps()) == 5, timeout=5.0)
        stamps = sorted(received.stamps())
        assert np.all(np.abs(np.diff(stamps) - 100 * MS) < MS)
        frame = received.at(stamps[-1])
        assert frame.image.shape == (24, 32, 3) and int(frame.image[0, 0, 0]) == 160
        wait_until(lambda: player.finished, timeout=5.0)
    finally:
        received.close()
        recorder.close()


def test_video_controls_as_parameters(sessions, videos, tmp_path):  # noqa: F811
    """The play node's parameters on a video: duration and file, paused, a seek (to a frame: decoded on from the key
    frame before it; the stamps growing), loop switched off while it plays (it ends then), the end announced."""
    a, b = sessions
    color_mp4, _ = videos
    recorder = recording.Recorder(a, tmp_path, name="video_ctl")
    color = FrameHistory(ImageSubscriber(b, "t/ctl/color", transport="raw"), 400)
    client = ParameterClient(b, "xblick/video_ctl")
    events = []
    client.on_event(lambda e: events.append({p.name: p.value.value() for p in e.changed_parameters}))
    try:
        recorder.play([color_mp4], streams=["t/ctl/color"], loop=True)
        wait_until(lambda: len(color.stamps()) >= 3, timeout=5.0)
        values = dict(zip(client.list(), [v.value() for v in client.get(client.list())]))
        assert values["duration_s"] == pytest.approx(1.0, abs=0.06) and values["file"] == str(color_mp4)
        assert values["loop"] is True and values["paused"] is False
        assert client.set({"paused": True})[0].successful
        time.sleep(0.2)
        held = len(color.stamps())
        time.sleep(0.4)
        assert len(color.stamps()) == held
        # a seek while paused: from the frame at 0.5 s (its red 10 * 10) once resumed, after the last
        last = max(color.stamps())
        assert client.set({"position_s": 0.5})[0].successful
        assert client.set({"paused": False})[0].successful
        wait_until(lambda: max(color.stamps()) > last, timeout=5.0)
        first = min(s for s in color.stamps() if s > last)
        assert first - last >= 49 * MS  # a period after it, and the pause's wall time
        assert int(color.at(first).image[0, 0, 0]) == pytest.approx(100, abs=3)  # H.264's rounding
        # loop off while it plays: it ends at the end, and says so
        assert client.set({"loop": False})[0].successful
        wait_until(lambda: recorder.state == "idle", timeout=5.0)
        wait_until(lambda: any(e.get("file") == "" for e in events), timeout=5.0)
        stamps = sorted(color.stamps())
        assert stamps == sorted(set(stamps))  # never back
    finally:
        client.close()
        color.close()
        recorder.close()


def _play_videos_on_gpu(port, files, ready):
    """Child (a CUDA ring cannot be opened by the process that made it): two videos played with a CUDA ring each."""
    from xblick.session import open_session

    session = open_session(listen=[f"127.0.0.1:{port}"])
    node = recording.Recorder(session, Path(files[0]).parent, name="video_gpu")
    node.play(files, streams=["t/gpu/color", "t/gpu/depth"], loop=True, cuda_device=0)
    ready.set()
    time.sleep(60)


@pytest.mark.skipif(_core.cuda_device_count() == 0, reason="no CUDA device")
def test_videos_in_cuda_rings(videos, tmp_path):
    """play --cuda-device with videos: each stream offers a CUDA ring on that GPU, which a subscriber on the same GPU
    takes by default (local auto, device cuda:0); it keeps serving after a seek and across loops."""
    import multiprocessing as mp

    from xblick.gpu import CudaImage
    from xblick.session import open_session
    from test_points import free_port

    ctx = mp.get_context("spawn")
    port = free_port()
    ready = ctx.Event()
    child = ctx.Process(target=_play_videos_on_gpu, args=(port, [str(v) for v in videos], ready), daemon=True)
    child.start()
    try:
        assert ready.wait(60)
        session = open_session(connect=[f"127.0.0.1:{port}"])
        color = ImageSubscriber(session, "t/gpu/color", transport="auto", device="cuda:0")
        depth = ImageSubscriber(session, "t/gpu/depth", transport="auto", device="cuda:0")

        def frame_of(subscriber):
            end = time.monotonic() + 30
            while time.monotonic() < end:
                frame = subscriber.latest(timeout=0.5)
                if frame is not None:
                    return frame
            raise TimeoutError(subscriber.describe())

        for subscriber, shape in ((color, (H, W, 3)), (depth, (H, W))):
            frame = frame_of(subscriber)
            assert subscriber.transport == "cuda", subscriber.transport_reason
            assert isinstance(frame.image, CudaImage) and frame.image.shape == shape
            assert subscriber.transport_reason == "cuda (publisher on this GPU)"
        client = ParameterClient(session, "xblick/video_gpu")
        before = frame_of(color).stamp_ns
        assert client.set({"position_s": 0.0})[0].successful
        time.sleep(1.5)  # past a loop too (the videos are 1 s long)
        after = frame_of(color)
        assert isinstance(after.image, CudaImage) and after.stamp_ns > before
        client.close()
        for c in (color, depth, session):
            c.close()
    finally:
        child.kill()
        child.join()


def test_play_takes_a_relative_path_with_a_directory(videos, tmp_path, monkeypatch, capsys):
    """xblick play rec/color.mp4 from the directory above rec: the file relative to the working directory, not to the
    node's directory (the file's own, rec/), which would make it rec/rec/color.mp4."""
    from xblick import cli

    color_mp4, _ = videos
    (tmp_path / "rec").mkdir()
    color_mp4.rename(tmp_path / "rec" / "color.mp4")
    monkeypatch.chdir(tmp_path)
    port = free_port()
    assert cli.main(["play", "rec/color.mp4", "--camera", "t/rel", "--exit", "--report", "0.2", "--listen",
                     f"tcp/127.0.0.1:{port}"]) == 0
    out = capsys.readouterr().out
    assert f"{tmp_path / 'rec' / 'color.mp4'} -> t/rel/color" in out, out
