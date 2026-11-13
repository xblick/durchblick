import threading
import time

import numpy as np
import pytest
from test_camera import pattern, sessions, wait_until  # noqa: F401 - sessions is a fixture

from blick import node, poses, tensors
from blick.camera import CameraPublisher, Frame, ImageSubscriber
from blick.diagnostics import DiagnosticsPublisher
from blick.history import FrameHistory, StampedHistory
from blick.msg import DiagnosticArray, Header, Time


def test_frame_history_keeps_frames_by_stamp(sessions):  # noqa: F811
    a, b = sessions
    pub = CameraPublisher(a, "robot/x/color", encoding="rgb8", frame_id="cam")
    wake = threading.Event()
    history = FrameHistory(ImageSubscriber(b, "robot/x/color", transport="auto"), 3, wake.set)
    wait_until(lambda: pub.subscribed)
    for stamp in range(1, 6):
        pub.publish(pattern(stamp), stamp_ns=stamp)
        wait_until(lambda s=stamp: history.newest_stamp == s)
    assert wake.is_set() and history.stamps() == {3, 4, 5} and history.newest()[0] == 5
    assert history.at(4).header.stamp.to_ns() == 4 and history.at(1) is None
    history.close()
    pub.close()


def test_histories_can_share_a_condition():
    """Several histories a node looks at together under one lock, woken by its notify_all."""
    cond = threading.Condition()
    color, depth = StampedHistory(4, cond.notify_all, cond), StampedHistory(4, cond.notify_all, cond)
    got = []

    def reader():
        with cond:
            while not (color.items and depth.items):
                cond.wait(1.0)
            got.append((next(iter(color.items)), next(iter(depth.items))))

    t = threading.Thread(target=reader)
    t.start()
    color.put(7, "c")
    depth.put(7, "d")
    t.join(2.0)
    assert got == [(7, 7)]


def test_frame_history_error_reaches_the_reader():
    class Failing:
        base = "robot/x/color"

        def latest(self, timeout):
            raise RuntimeError("another protocol version")

        def close(self):
            pass

    wake = threading.Event()
    history = FrameHistory(Failing(), 3, wake.set)
    assert wake.wait(2.0)
    with pytest.raises(RuntimeError, match="protocol"):
        history.check()
    history.close()


def frame(image, encoding):
    return Frame(image, encoding, Header(Time(0, 0), "cam"), "robot/x/color", 0)


def test_tensors():
    torch = pytest.importorskip("torch")
    rgb = np.arange(2 * 3 * 3, dtype=np.uint8).reshape(2, 3, 3)
    assert torch.equal(tensors.rgb(frame(rgb, "rgb8"), "cpu"), torch.as_tensor(rgb))
    assert torch.equal(tensors.rgb(frame(rgb[..., ::-1].copy(), "bgr8"), "cpu"), torch.as_tensor(rgb))
    rgba = np.concatenate([rgb, np.full((2, 3, 1), 255, np.uint8)], -1)
    assert torch.equal(tensors.rgb(frame(rgba, "rgba8"), "cpu"), torch.as_tensor(rgb))
    grey = rgb[..., 0].copy()
    assert torch.equal(tensors.rgb(frame(grey, "mono8"), "cpu")[..., 2], torch.as_tensor(grey))
    assert torch.allclose(tensors.depth_metres(frame(np.full((2, 3), 1500, np.uint16), "16UC1"), "cpu"),
                          torch.full((2, 3), 1.5))
    metres = np.full((2, 3), np.nan, np.float32)
    assert torch.equal(tensors.depth_metres(frame(metres, "32FC1"), "cpu"), torch.zeros(2, 3))
    with pytest.raises(ValueError, match="one of"):
        tensors.rgb(frame(rgb, "yuv422"), "cpu")


def test_reporter(sessions, capsys):  # noqa: F811
    a, b = sessions
    got = []
    subscriber = b.declare_subscriber("robot/x/diagnostics",
                                      lambda s: got.append(DiagnosticArray.deserialize(s.payload.to_bytes())))
    report = node.Reporter(a, "robot/x", "nemo", "durchblick-nemo", log_prefix="robot/x: ", every=0.05,
                           log_every=0.0)
    time.sleep(0.3)
    report.counts["frames"] += 3
    wait_until(report.due)
    done, seconds = report.take()
    assert done["frames"] == 3 and seconds > 0 and not report.due()
    level, message = node.state(f"{done['frames'] / seconds:.0f} frames/s", ["2 unpaired"])
    assert level == DiagnosticsPublisher.WARN and message.endswith("frames/s, 2 unpaired")
    report.publish(level, message, {"frames/s": 3.0}, log_extra=", 12 ms a frame")
    wait_until(lambda: got)
    assert got[0].status[0].name == "nemo" and got[0].status[0].message == message
    assert capsys.readouterr().out == f"durchblick-nemo: robot/x: {message}, 12 ms a frame\n"
    report.counts["frames"] += 1
    assert report.take()[0]["frames"] == 1  # only those since the last take
    assert node.state("0.00 frames/s", waiting="robot/x/color: no frame yet") == (
        DiagnosticsPublisher.WARN, "0.00 frames/s, waiting: robot/x/color: no frame yet")
    assert node.state("9.9 frames/s")[0] == DiagnosticsPublisher.OK
    report.close()
    subscriber.undeclare()


def test_pose_conversions_round_trip():
    angle = 0.7
    m = np.eye(4)
    m[:3, :3] = [[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]]
    m[:3, 3] = (0.1, -0.2, 0.3)
    t = poses.transform_of(m)
    q = t.rotation
    back = np.eye(4)
    back[:3, :3] = poses.rotation_from_quaternion(q.x, q.y, q.z, q.w)
    back[:3, 3] = (t.translation.x, t.translation.y, t.translation.z)
    assert np.allclose(back, m)
    p = poses.pose_of(m)
    assert (p.position.x, p.position.y, p.position.z) == pytest.approx((0.1, -0.2, 0.3))


def test_diagnostics_name_their_session(sessions):  # noqa: F811
    """Two sessions on one camera's diagnostics key: each sample says which session sent it (the attachment's zid),
    and a sample without an attachment (a publisher before 0.3.29) has no sender."""
    from blick import diagnostics

    a, b = sessions
    got = []
    subscriber = b.declare_subscriber("robot/x/diagnostics", got.append)
    ours, theirs = DiagnosticsPublisher(a, "robot/x", "nemo"), DiagnosticsPublisher(b, "robot/x", "boxer")
    time.sleep(0.3)
    ours.publish(DiagnosticsPublisher.OK, "fine", {})
    theirs.publish(DiagnosticsPublisher.WARN, "waiting", {})
    a.put("robot/x/diagnostics", bytes(DiagnosticArray(Header(Time(1, 0), ""), []).serialize()),
          encoding=diagnostics.cdr_encoding(DiagnosticArray))
    wait_until(lambda: len(got) == 3)
    senders = {diagnostics.read(s)[1].status[0].name if diagnostics.read(s)[1].status else "old":
               diagnostics.sender(s) for s in got}
    assert senders == {"nemo": str(a.info.zid()), "boxer": str(b.info.zid()), "old": None}
    subscriber.undeclare()
    ours.close()
    theirs.close()


def test_local_argument():
    """node.add_arguments(local=...): --local with its choices, auto by default; a CPU node's refuses cuda with the
    reason; without local, no --local (a node's own does not clash)."""
    import argparse

    from blick.camera import LOCAL

    gpu = argparse.ArgumentParser()
    node.add_arguments(gpu, local=LOCAL)
    assert gpu.parse_args([]).local == "auto" and gpu.parse_args(["--local", "cuda"]).local == "cuda"
    assert "its CUDA ring when that is on this node's GPU" in gpu.format_help()
    cpu = argparse.ArgumentParser()
    node.add_arguments(cpu, local=("auto", "shm"))
    assert cpu.parse_args(["--local", "shm"]).local == "shm"
    with pytest.raises(SystemExit):
        cpu.parse_args(["--local", "cuda"])
    plain = argparse.ArgumentParser()
    node.add_arguments(plain)
    plain.add_argument("--local", default="shm")  # a node's own, as before
    assert plain.parse_args([]).local == "shm"
    with pytest.raises(ValueError):
        node.add_arguments(argparse.ArgumentParser(), local=("shm",))
