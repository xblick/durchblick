"""Recording a simulation: blocking publishers and recorder with the count check, the ready handshake, log time from the
messages' stamps, lockstep playback (steps, a clock), the simulation's topics (sim.py) and the in-process Recording."""

import json
import threading
import time

import numpy as np
import pytest
from test_camera import pattern, sessions, wait_until  # noqa: F401 - sessions is a fixture

from blick import recording, topics
from blick.camera import CameraPublisher, CountedPublisher, TransformPublisher, cdr_encoding
from blick.msg import (CameraInfo, Clock, Header, JointState, TFMessage, Time, Transform,
                            TransformStamped, Vector3)
from blick.robot import RobotStatePublisher
from blick.session import open_session, replies
from blick.sim import Rate, SimPublisher

mcap_reader = pytest.importorskip("mcap.reader")  # the extra blick[record]

ROBOT = "robot"
CAMERA = f"{ROBOT}/azurekinect"
MS = 1_000_000
T0 = 1_000 * 1_000_000_000  # simulated time starts at 1000 s


def read(path):
    """(summary, [(key, log_time, publish_time, data)], {metadata name: values})"""
    with open(path, "rb") as stream:
        reader = mcap_reader.make_reader(stream)
        summary = reader.get_summary()
        messages = [(channel.metadata["zenoh_key"], m.log_time, m.publish_time, m.data)
                    for _, channel, m in reader.iter_messages(log_time_order=True)]
        stream.seek(0)
        metadata = {m.name: m.metadata for m in mcap_reader.make_reader(stream).iter_metadata()}
    return summary, messages, metadata


def body(stamp, name, x):
    return TransformStamped(Header(Time.from_ns(stamp), "world"), name, Transform(Vector3(x, 0.0, 0.0)))


class Episode:
    """A simulation's publishers on one session: SimPublisher (robot, world, commands) and a head camera with its pose,
    all blocking unless told otherwise; ``run`` publishes ``ms`` control steps of 1 ms, a camera frame every 33 ms."""

    def __init__(self, session, blocking=True, state_hz=None):
        self.sim = SimPublisher(session, ROBOT, blocking=blocking, state_hz=state_hz)
        info = CameraInfo.pinhole(64, 48, 50.0, 50.0, 31.5, 23.5, frame_id="cam")
        self.camera = CameraPublisher(session, f"{CAMERA}/color", encoding="rgb8", frame_id="cam", camera_info=info,
                                      blocking=blocking)
        self.camera_tf = TransformPublisher(session, CAMERA, blocking=blocking)
        self.frames = []  # stamps of the camera frames

    @property
    def keys(self):
        return self.sim.keys + [topics.compressed(f"{CAMERA}/color"), topics.camera_info(f"{CAMERA}/color"),
                                topics.tf(CAMERA)]

    def run(self, ms, start=0):
        names = ["j1", "j2"]
        for k in range(start, start + ms):
            stamp = T0 + k * MS
            self.sim.clock(stamp)
            self.sim.joint_states(JointState(Header(Time.from_ns(stamp)), names, np.array([k * 1e-3, 0.0]),
                                             np.zeros(2), np.zeros(2)))
            self.sim.command_joints(JointState(Header(Time.from_ns(stamp)), ["hand_left"], np.array([0.5])))
            if k % 33 == 0:
                self.sim.bodies(TFMessage([body(stamp, "mug", k * 1e-3), body(stamp, "table", 0.0)]))
                self.camera.publish(pattern(k), stamp_ns=stamp)
                self.camera_tf.publish([body(stamp, "cam", 1.0)])
                self.frames.append(stamp)
            if k == start:
                self.sim.event(stamp, "phase", phase="reach")

    def close(self):
        self.sim.close()
        self.camera.close()
        self.camera_tf.close()


def test_header_stamps():
    stamp = 1_761_480_000_123_456_789
    js = bytes(JointState(Header(Time.from_ns(stamp)), ["a"], np.array([1.0])).serialize())
    assert recording.header_stamp("k", "application/cdr;sensor_msgs/msg/JointState", js) == stamp
    tf = bytes(TFMessage([body(stamp, "mug", 1.0)]).serialize())
    assert recording.header_stamp("k", "application/cdr;tf2_msgs/msg/TFMessage", tf) == stamp
    clock = bytes(Clock(Time.from_ns(stamp)).serialize())
    assert recording.header_stamp("k", "application/cdr;rosgraph_msgs/msg/Clock", clock) == stamp
    assert recording.header_stamp("k", "application/json", json.dumps({"stamp_ns": stamp}).encode()) == stamp
    with pytest.raises(ValueError, match="without transforms"):
        recording.header_stamp("k", "application/cdr;tf2_msgs/msg/TFMessage", bytes(TFMessage([]).serialize()))
    with pytest.raises(ValueError, match="no header stamp"):
        recording.header_stamp("k", "application/cdr;other_msgs/msg/Thing", b"\0\1\0\0" + bytes(8))
    with pytest.raises(ValueError, match="stamp_ns"):
        recording.header_stamp("k", "application/json", b'{"v": 1}')
    with pytest.raises(ValueError, match="no stamp"):
        recording.header_stamp("k", "text/plain", b"x")


def test_rates_by_simulated_time():
    every = Rate(None)
    assert all(every.due(T0 + k * MS) for k in range(10))
    camera = Rate(30.0)
    assert sum(camera.due(T0 + k * MS) for k in range(3000)) in (90, 91)  # 3 s of 1 ms steps
    with pytest.raises(ValueError, match="goes back"):
        camera.due(T0)
    with pytest.raises(ValueError, match="> 0"):
        Rate(0.0)


def test_parse_command_options():
    assert recording.parse_command(b'{"command": "record", "keys": ["a/b"], "blocking": true}') == (
        "record", {"keys": ["a/b"], "blocking": True})
    assert recording.parse_command(b'{"command": "step", "frames": 3}') == ("step", {"frames": 3})
    with pytest.raises(ValueError, match="record takes keys"):
        recording.parse_command(b'{"command": "record", "rate": 2}')
    with pytest.raises(ValueError, match="stop takes no options"):
        recording.parse_command(b'{"command": "stop", "frames": 1}')
    assert recording.check_keys(["a/*/b", "sim/**"]) == ["a/*/b", "sim/**"]
    with pytest.raises(ValueError):
        recording.check_keys(["a/b c"])
    with pytest.raises(ValueError, match="twice"):
        recording.check_keys(["a", "a"])


def test_blocking_episode_in_process(tmp_path):
    """An offline batch's episode: a private session, the publishers and the Recording on it; every sample in the
    file, logged at its simulated stamp, the counts checked and kept in the file's metadata."""
    session = open_session(local_router=False)
    episode = Episode(session)
    path = tmp_path / "episodes" / "0001.mcap"
    with recording.Recording(session, path, episode.keys) as rec:
        assert set(rec.ready["publishers"]) == {k for k in episode.keys if "*" not in k}
        assert rec.ready["missing"] == [] and rec.ready["unmatched"] == []
        assert rec.ready["dropping"] == [f"{ROBOT}/wrench/*"]  # no wrench sensor: nothing blocking found
        assert rec.ready["streams"] == [topics.compressed(f"{CAMERA}/color")]
        episode.run(500)
    result = rec.result
    assert result["check"]["ok"], result["check"]["errors"]
    counts = result["check"]["counts"]
    assert counts[topics.joint_states(ROBOT)] == {"received": 500, "sent": 500, "expected": 500, "lost": 0}
    assert counts["sim/clock"]["received"] == 500 and counts["sim/events"]["received"] == 1
    assert counts[topics.compressed(f"{CAMERA}/color")]["received"] == len(episode.frames) == 16
    summary, messages, metadata = read(path)
    by_key = {}
    for key, log_time, publish_time, data in messages:
        by_key.setdefault(key, []).append((log_time, publish_time, data))
    assert [t for t, _, _ in by_key["sim/clock"]] == [T0 + k * MS for k in range(500)]  # log time: the stamp
    assert [t for t, _, _ in by_key[topics.compressed(f"{CAMERA}/color")]] == episode.frames
    assert all(p > T0 * 1000 for _, p, _ in by_key["sim/clock"])  # publish time: when it came (wall clock)
    assert [t for t, _, _ in by_key["sim/tf"]] == episode.frames
    assert json.loads(by_key["sim/events"][0][2])["phase"] == "reach"
    channels = {c.topic: c for c in summary.channels.values()}
    assert json.loads(channels[f"/{CAMERA}/color/image_raw/compressed"].metadata["durchblick_info"])["blocking"]
    check = metadata[recording.METADATA]
    assert json.loads(check["ok"]) is True and json.loads(check["keys"]) == episode.keys
    assert check["log_time"] == "header"
    with pytest.raises(FileExistsError):  # never overwritten
        recording.Recording(session, path, episode.keys).start()
    episode.close()
    session.close()


def test_blocking_recorder_blocks_instead_of_dropping(tmp_path):
    """A queue of 4 samples: the recorder holds the publishers up instead of dropping; all 3000 samples are written."""
    session = open_session(local_router=False)
    publisher = CountedPublisher(session, "sim/clock", cdr_encoding(Clock), blocking=True)
    recorder = recording.Recorder(session, tmp_path, name="slow", split=0, queue_size=4)
    recorder.record(keys=["sim/clock"], blocking=True, log_time="header", file="slow.mcap")
    for k in range(3000):
        publisher.put(bytes(Clock(Time.from_ns(T0 + k * MS)).serialize()))
    result = recorder.stop()["recording"]
    assert result["check"]["ok"] and result["received"]["sim/clock"] == 3000 and recorder.dropped == 0
    assert len(read(tmp_path / "slow.mcap")[1]) == 3000
    recorder.close()
    publisher.close()
    session.close()


def test_a_lost_sample_is_an_error(tmp_path):
    """Samples the blocking publisher counts but the recorder never got: RecordingError at stop, the file complete with
    what was missing in its metadata. A dropping publisher's key in the same recording is reported, not checked."""
    session = open_session(local_router=False)
    episode = Episode(session)
    rec = recording.Recording(session, tmp_path / "lost.mcap", episode.keys + ["sim/plain"])
    ready = rec.start()
    assert ready["missing"] == ["sim/plain"] and ready["dropping"] == [f"{ROBOT}/wrench/*", "sim/plain"]
    episode.run(50)
    session.put("sim/plain", b'{"stamp_ns": 1}', encoding="application/json")  # dropping: best effort
    episode.sim._clock._sent.count(episode.sim._clock.seq + 1)  # as if a clock sample had been lost on the way
    with pytest.raises(recording.RecordingError) as error:
        rec.stop()
    check = error.value.result["check"]
    assert check["errors"] == ["sim/clock: 1 of 51 samples published after ready are not in the recording"]
    assert check["counts"]["sim/clock"]["lost"] == 1
    assert check["dropping"] == {"sim/plain": {"received": 1, "dropped": 0}}
    metadata = read(tmp_path / "lost.mcap")[2][recording.METADATA]
    assert json.loads(metadata["ok"]) is False
    episode.close()
    session.close()


def test_the_defaults_drop(tmp_path):
    """Nothing blocks unless it asks for it: publishers drop and answer no count by default, the recorder drops; a
    publisher that asks for blocking gets it."""
    session = open_session(local_router=False)
    publishers = [CountedPublisher(session, "a/x", cdr_encoding(JointState)), RobotStatePublisher(session, ROBOT),
                  TransformPublisher(session, CAMERA), SimPublisher(session, "robot2"),
                  CameraPublisher(session, f"{CAMERA}/color", encoding="rgb8", frame_id="cam")]
    assert not any(p.blocking for p in publishers[:2]) and not publishers[3].blocking and not publishers[4].blocking
    assert replies(session, "**/sent", 0.5) == []  # no blocking publisher, no count
    recorder = recording.Recorder(session, tmp_path, name="defaults", split=0)
    assert recorder.record(keys=["a/x"], file="d.mcap", ready_timeout=0.0)["blocking"] is False
    recorder.stop()
    blocking = CountedPublisher(session, "a/y", cdr_encoding(JointState), blocking=True)
    assert [r.ok.payload.to_bytes() for r in replies(session, "a/y/sent", 0.5)]
    for p in publishers + [blocking]:
        p.close()
    recorder.close()
    session.close()


@pytest.mark.parametrize("where", ["in-process", "tcp"])
def test_a_dropping_publisher_is_never_held_up(tmp_path, monkeypatch, where):
    """A slow blocking recorder (5 ms per written sample, a queue of 4) and a robot's dropping publisher in the same
    recording: the robot's samples go out at full speed and are dropped by the recorder, never waited for; the
    blocking key is complete and checked, the dropping one reported."""
    write = recording._File.write

    def slow(self, *args):
        time.sleep(0.005)
        write(self, *args)

    monkeypatch.setattr(recording._File, "write", slow)
    if where == "in-process":
        a = b = open_session(local_router=False)
    else:
        from test_points import free_port

        port = free_port()
        a = open_session(listen=[f"127.0.0.1:{port}"])
        b = open_session(connect=[f"127.0.0.1:{port}"])
    robot = RobotStatePublisher(a, ROBOT)  # the real robot's: drops
    clock = CountedPublisher(a, "sim/clock", cdr_encoding(Clock), blocking=True)
    recorder = recording.Recorder(b, tmp_path, name="mixed", split=0, queue_size=4)
    ready = recorder.record(keys=["sim/clock", topics.joint_states(ROBOT)], blocking=True, log_time="header",
                            file="mixed.mcap", ready_timeout=0.5)
    assert ready["dropping"] == [topics.joint_states(ROBOT)] and list(ready["publishers"]) == ["sim/clock"]
    if where == "tcp":
        wait_until(lambda: robot._joint_states.matched)
    n = 400
    began = time.monotonic()
    for k in range(n):
        robot.joint_states(JointState(Header(Time.from_ns(T0 + k * MS)), ["j1"], np.array([0.0])))
    held = time.monotonic() - began
    assert held < 0.5, f"the robot's publisher was held up: {held:.2f} s for {n} samples (0.005 s each written)"
    for k in range(30):
        clock.put(bytes(Clock(Time.from_ns(T0 + k * MS)).serialize()))
    result = recorder.stop()["recording"]
    check = result["check"]
    assert check["ok"], check["errors"]
    assert check["counts"]["sim/clock"]["lost"] == 0 and check["counts"]["sim/clock"]["received"] == 30
    joints = check["dropping"][topics.joint_states(ROBOT)]
    assert joints["received"] < n  # it could not keep up: dropped (by the recorder, or the link under DROP)
    if where == "in-process":
        assert joints["dropped"] > 0 and joints["received"] + joints["dropped"] == n
    recorder.close()
    clock.close()
    robot.close()
    if where == "tcp":
        b.close()
    a.close()


def test_mixed_recording_checks_the_blocking_keys(tmp_path):
    """A simulation's blocking keys and a robot's dropping ones in one blocking recording: the check passes for the
    blocking keys, the dropping ones are reported (received, dropped) and not checked."""
    session = open_session(local_router=False)
    episode = Episode(session)
    other = RobotStatePublisher(session, "robot2")  # dropping, as on a robot
    keys = episode.keys + [topics.joint_states("robot2")]
    with recording.Recording(session, tmp_path / "mixed.mcap", keys) as rec:
        assert topics.joint_states("robot2") in rec.ready["dropping"]
        episode.run(100)
        for k in range(10):
            other.joint_states(JointState(Header(Time.from_ns(T0 + k * MS)), ["j"], np.array([0.0])))
    check = rec.result["check"]
    assert check["ok"] and topics.joint_states("robot2") not in check["counts"]
    assert check["dropping"] == {topics.joint_states("robot2"): {"received": 10, "dropped": 0}}
    assert check["counts"][topics.joint_states(ROBOT)]["expected"] == 100
    other.close()
    episode.close()
    session.close()


def test_a_publisher_unaware_of_the_subscription_is_refused(tmp_path):
    """A blocking publisher that does not know of the recorder after ready_timeout: record is refused, no file."""
    session = open_session(local_router=False)
    unaware = session.declare_queryable("sim/late/sent", lambda q: q.reply(
        "sim/late/sent", json.dumps({"v": 1, "key": "sim/late", "sent": 0, "seq": 0, "matched": False})))
    recorder = recording.Recorder(session, tmp_path, name="refuse", split=0)
    with pytest.raises(recording.RecordingError, match="do not know of the recorder's subscription"):
        recorder.record(keys=["sim/late"], blocking=True, file="late.mcap", ready_timeout=0.3)
    assert not (tmp_path / "late.mcap").exists() and recorder.state == "idle" and recorder.status()["topics"] == 0
    with pytest.raises(ValueError, match="blocking needs keys"):
        recorder.record(blocking=True)
    recorder.close()
    unaware.undeclare()
    session.close()


def test_handshake_and_count_check_over_zenoh(sessions, tmp_path):  # noqa: F811
    """The simulation controls a recorder node on another session: record with its keys answers ready (the blocking
    publishers' counts, the literal key without a publisher), stop answers with the checked counts."""
    a, b = sessions
    node = recording.Recorder(b, tmp_path, name="simrec", split=0)
    episode = Episode(a)
    episode.run(20)  # before the recording: not in it, not counted against it
    keys = episode.keys + ["sim/nobody"]
    ready = recording.request(a, "simrec", "record", keys=keys, blocking=True, log_time="header", file="ep.mcap",
                              ready_timeout=3.0)
    assert ready["file"] == str(tmp_path / "ep.mcap") and ready["blocking"] and ready["keys"] == keys
    assert ready["missing"] == ["sim/nobody"] and "sim/nobody" in ready["dropping"]
    assert ready["publishers"][topics.joint_states(ROBOT)]["sent"] == 20
    assert all(p["matched"] for p in ready["publishers"].values())
    episode.run(300, start=20)
    result = recording.request(a, "simrec", "stop")["recording"]
    check = result["check"]
    assert check["ok"], check["errors"]
    assert check["counts"][topics.joint_states(ROBOT)]["expected"] == 300
    assert check["counts"][topics.compressed(f"{CAMERA}/color")]["lost"] == 0
    messages = read(tmp_path / "ep.mcap")[1]
    clock = [t for key, t, _, _ in messages if key == "sim/clock"]
    assert clock[-300:] == [T0 + k * MS for k in range(20, 320)]
    with pytest.raises(RuntimeError, match="refused.*exists"):
        recording.request(a, "simrec", "record", keys=keys, file="ep.mcap")
    with pytest.raises(RuntimeError, match="refused.*blocking needs keys"):
        recording.request(a, "simrec", "record", blocking=True)
    episode.close()
    node.close()


def test_discovery_records_the_simulations_topics(sessions, tmp_path):  # noqa: F811
    """Without keys (blick record), the simulation's world and commands are among the patterns."""
    a, b = sessions
    sim = SimPublisher(a, ROBOT)
    recorder = recording.Recorder(b, tmp_path, name="all", split=0, look_every=0.5)
    recorder.record()
    wait_until(lambda: sim._events.matched and sim._command_joints.matched and sim._clock.matched)
    stamp = T0
    sim.clock(stamp)
    sim.command_joints(JointState(Header(Time.from_ns(stamp)), ["hand_left"], np.array([0.5])))
    sim.bodies(TFMessage([body(stamp, "mug", 0.1)]))
    sim.event(stamp, "success")
    time.sleep(0.5)
    recorder.stop()
    keys = {key for key, *_ in read(recorder.files[0])[1]}
    assert {"sim/clock", "sim/tf", "sim/events", f"{ROBOT}/command/joints"} <= keys
    recorder.close()
    sim.close()


@pytest.fixture
def episode_file(tmp_path):
    """A blocking episode of 100 ms (frames at 0, 33, 66, 99 ms), recorded in-process with log time = stamps."""
    session = open_session(local_router=False)
    episode = Episode(session)
    path = tmp_path / "episode.mcap"
    with recording.Recording(session, path, episode.keys):
        episode.run(100)
    episode.close()
    session.close()
    return path


def collect(session, keys):
    """Subscribers collecting (key, stamp) of what is played (stamps from the payloads)."""
    got = []
    subs = [session.declare_subscriber(k, lambda s: got.append((str(s.key_expr), recording.header_stamp(
        str(s.key_expr), str(s.encoding), s.payload.to_bytes())))) for k in keys]
    return got, subs


def test_step_playback_frame_by_frame(episode_file, sessions):  # noqa: F811
    """pace step: nothing goes out until a step; a step plays one frame (one log time: every key of that simulated
    millisecond), until a time, or up to the next camera frame; the end is reported."""
    a, b = sessions
    player = recording.Player(a, [episode_file], prefix="replay", pace="step")
    got, subs = collect(b, ["replay/sim/clock", f"replay/{topics.compressed(CAMERA + '/color')}",
                            f"replay/{topics.joint_states(ROBOT)}"])
    time.sleep(0.5)
    thread = threading.Thread(target=player.run)
    thread.start()
    time.sleep(0.3)
    assert got == []  # waits for a step
    first = player.step()
    assert first["log_time_ns"] == T0 and not first["end"]
    wait_until(lambda: len(got) >= 3)
    time.sleep(0.2)
    assert sorted(got) == sorted([("replay/sim/clock", T0), (f"replay/{topics.compressed(CAMERA + '/color')}", T0),
                                  (f"replay/{topics.joint_states(ROBOT)}", T0)])
    assert player.step(frames=2)["log_time_ns"] == T0 + 2 * MS
    assert player.step(until_ns=T0 + 10 * MS)["log_time_ns"] == T0 + 10 * MS
    to_frame = player.step(key=topics.compressed(CAMERA + "/color"))
    assert to_frame["log_time_ns"] == T0 + 33 * MS  # the next camera frame and the rest of its millisecond
    wait_until(lambda: ("replay/sim/clock", T0 + 33 * MS) in got)
    clock = [t for k, t in got if k == "replay/sim/clock"]
    assert clock == [T0 + k * MS for k in range(34)]  # every simulated millisecond, in order, none twice
    last = player.step(until_ns=T0 + 10_000 * MS)
    assert last["end"] and last["log_time_ns"] == T0 + 99 * MS
    thread.join(5.0)
    assert not thread.is_alive() and player.error is None
    with pytest.raises(RuntimeError, match="paced by wall"):
        recording.Player(a, [episode_file], prefix="w").step()
    for sub in subs:
        sub.undeclare()
    player.close()


def test_clock_paced_playback(episode_file, sessions):  # noqa: F811
    """pace clock: frames go out up to the time of each Clock sample on the clock key (a policy's clock)."""
    a, b = sessions
    player = recording.Player(a, [episode_file], prefix="eval", pace="clock", clock="policy/clock")
    got, subs = collect(b, ["eval/sim/clock"])
    tick = b.declare_publisher("policy/clock", encoding=cdr_encoding(Clock))
    time.sleep(0.5)
    thread = threading.Thread(target=player.run)
    thread.start()
    time.sleep(0.3)
    assert got == []
    tick.put(bytes(Clock(Time.from_ns(T0 + 4 * MS)).serialize()))
    wait_until(lambda: len(got) >= 5)
    time.sleep(0.3)
    assert [t for _, t in got] == [T0 + k * MS for k in range(5)]
    tick.put(bytes(Clock(Time.from_ns(T0 + 20 * MS)).serialize()))
    wait_until(lambda: len(got) >= 21)
    time.sleep(0.3)
    assert len(got) == 21 and player.log_time_ns == T0 + 20 * MS
    b.put("policy/clock", b"not a clock", encoding="text/plain")  # another type: the playback ends with the error
    thread.join(5.0)
    assert not thread.is_alive() and "ProtocolError" in repr(player.error)
    with pytest.raises(ValueError, match="pace clock needs"):
        recording.Player(a, [episode_file], pace="clock")
    tick.undeclare()
    for sub in subs:
        sub.undeclare()
    player.close()


def test_step_over_zenoh(episode_file, sessions):  # noqa: F811
    """The node plays in lockstep when told (play with pace step) and steps on request."""
    a, b = sessions
    node = recording.Recorder(b, episode_file.parent, name="stepper")
    got, subs = collect(a, ["lock/sim/clock"])
    status = recording.request(a, "stepper", "play", files=[episode_file.name], prefix="lock", pace="step")
    assert status["pace"] == "step"
    time.sleep(0.5)
    assert recording.request(a, "stepper", "step", frames=3)["log_time_ns"] == T0 + 2 * MS
    wait_until(lambda: len(got) >= 3)
    assert [t for _, t in got] == [T0, T0 + MS, T0 + 2 * MS]
    with pytest.raises(RuntimeError, match="refused.*one of them"):
        recording.request(a, "stepper", "step", frames=1, until_ns=T0)
    assert recording.request(a, "stepper", "step", until_ns=T0 + 10**12)["end"]
    wait_until(lambda: node.state == "idle")
    for sub in subs:
        sub.undeclare()
    node.close()


def test_command_line_records_blockingly(tmp_path, capsys):
    """blick recorder record --key ... --blocking and stop: the node's ready and its checked counts printed."""
    from test_points import free_port

    from blick import cli

    port = free_port()
    a = open_session(listen=[f"127.0.0.1:{port}"])
    node = recording.Recorder(a, tmp_path, name="clirec", split=0)
    clock = CountedPublisher(a, "sim/clock", cdr_encoding(Clock), blocking=True)
    connect = ["--name", "clirec", "--connect", f"127.0.0.1:{port}"]
    assert cli.main(["recorder", "record", "--key", "sim/clock", "--blocking", "--log-time", "header", "--file",
                     "cli.mcap", *connect]) == 0
    out = capsys.readouterr().out  # the node prints into it too (one process)
    ready = json.loads(out[out.index("{\n"):])
    assert ready["publishers"]["sim/clock"]["sent"] == 0 and ready["log_time"] == "header"
    for k in range(10):
        clock.put(bytes(Clock(Time.from_ns(T0 + k * MS)).serialize()))
    assert cli.main(["recorder", "stop", *connect]) == 0
    out = capsys.readouterr().out
    stopped = json.loads(out[out.index("{\n"):])
    assert stopped["recording"]["check"]["counts"]["sim/clock"] == {"received": 10, "sent": 10, "expected": 10,
                                                                    "lost": 0}
    clock.close()
    node.close()
    a.close()


def test_a_sample_without_stamp_fails_a_header_recording(tmp_path):
    """log_time header: a sample without a stamp it can read ends the writing; stop names that as the cause."""
    session = open_session(local_router=False)
    plain = CountedPublisher(session, "sim/text", "text/plain", blocking=True)
    rec = recording.Recording(session, tmp_path / "text.mcap", ["sim/text"])
    rec.start()
    plain.put(b"no stamp")
    with pytest.raises(recording.RecordingError, match="not written completely.*no stamp it can read"):
        rec.stop()
    plain.close()
    session.close()


def test_a_blocking_publisher_found_after_ready(tmp_path):
    """A wildcard key's blocking publisher that appears during the recording (a force-torque sensor's first
    measurement): found within a second, blocked for and checked from then on."""
    from blick.msg import WrenchStamped

    session = open_session(local_router=False)
    sim = SimPublisher(session, ROBOT, blocking=True)
    with recording.Recording(session, tmp_path / "wrench.mcap", sim.keys) as rec:
        assert f"{ROBOT}/wrench/*" in rec.ready["dropping"]
        for k in range(5):
            sim.robot.wrench("left", WrenchStamped(Header(Time.from_ns(T0 + k * MS), "ft_left")))
        time.sleep(1.5)  # the recorder looks for new blocking publishers every second
        for k in range(5, 20):
            sim.robot.wrench("left", WrenchStamped(Header(Time.from_ns(T0 + k * MS), "ft_left")))
    check = rec.result["check"]
    key = topics.wrench(ROBOT, "left")
    assert check["ok"], check["errors"]
    assert key in check["counts"] and check["counts"][key]["received"] == 20 and check["counts"][key]["lost"] == 0
    sim.close()
    session.close()
