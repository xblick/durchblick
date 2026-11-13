"""A robot's controller over Durchblick (controller.py): its commands (one message each, addressed, stale ones refused)
and its records, typed state, hands, status and diagnostics from publisher to subscriber, over real Zenoh sessions."""

import socket
import time

import numpy as np
import pytest

from xblick import topics
from xblick.controller import (ArmFields, Command, CommandPublisher, CommandSubscriber, ControllerPublisher,
                               ControllerSubscriber, command_of)
from xblick.msg import JointState, TFMessage, WrenchStamped
from xblick.parameters import Parameters
from xblick.robot import RobotStateSubscriber
from xblick.session import open_session

ROOT = "base_link"


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def wait_until(condition, timeout=5.0):
    end = time.monotonic() + timeout
    while not condition():
        if time.monotonic() > end:
            raise TimeoutError
        time.sleep(0.01)


@pytest.fixture
def sessions():
    port = free_port()
    a = open_session(listen=[f"127.0.0.1:{port}"])
    b = open_session(connect=[f"127.0.0.1:{port}"])
    yield a, b
    b.close()
    a.close()


POSE = (np.array([0.4, 0.2, 1.0]), np.array([1.0, 0.0, 0.0, 0.0]))


def test_one_message_per_command_addressed_to_its_controller(sessions):
    a, b = sessions
    got: list[Command] = []
    other: list[Command] = []
    reader = CommandSubscriber(b, "robot", "arms", got.append)
    other_reader = CommandSubscriber(b, "robot", "arms_test", other.append)
    time.sleep(0.3)  # the subscribers reach the other session
    commands = CommandPublisher(a, "robot", "arms", ROOT)

    # pose and posture with one stamp: one command, both parts
    commands.send({"left_tcp": POSE}, {"left_shoulder_1": 0.1, "left_shoulder_2": -0.2}, stamp_ns=2_000)
    # a pose alone, joints alone, nothing (e.g. only a level change elsewhere): each one command as sent
    commands.send({"left_tcp": POSE}, stamp_ns=2_001)
    commands.send(joints={"left_shoulder_1": 0.3}, stamp_ns=2_002)
    commands.send(stamp_ns=2_003)
    wait_until(lambda: len(got) == 4)
    assert [c.stamp_ns for c in got] == [2_000, 2_001, 2_002, 2_003]
    assert got[0].frame == ROOT and set(got[0].poses) == {"left_tcp"}
    assert got[0].joints == {"left_shoulder_1": 0.1, "left_shoulder_2": -0.2}
    np.testing.assert_allclose(got[0].poses["left_tcp"][0], POSE[0])
    np.testing.assert_allclose(got[0].poses["left_tcp"][1], POSE[1])
    assert got[1].joints == {} and got[1].poses
    assert got[2].poses == {} and got[2].joints == {"left_shoulder_1": 0.3}
    assert got[3].poses == {} and got[3].joints == {}
    time.sleep(0.1)
    assert other == [] and reader.refused == 0  # another controller's commands do not reach this one
    reader.check()
    for r in (reader, other_reader):
        r.close()
    commands.close()


def test_stale_and_frameless_commands_are_refused(sessions):
    a, b = sessions
    got: list[Command] = []
    refusals: list[str] = []
    reader = CommandSubscriber(b, "robot", "arms", got.append, on_refused=refusals.append)
    time.sleep(0.3)
    first = CommandPublisher(a, "robot", "arms", ROOT)
    second = CommandPublisher(a, "robot", "arms", ROOT)  # a second client of the same controller
    first.send({"left_tcp": POSE}, stamp_ns=5_000)
    wait_until(lambda: len(got) == 1)
    second.send({"left_tcp": POSE}, stamp_ns=4_000)  # older than what the controller took: stale
    wait_until(lambda: refusals)
    assert "not newer" in refusals[0] and reader.refused == 1 and len(got) == 1
    with pytest.raises(ValueError, match="not newer"):
        first.send(stamp_ns=5_000)  # the publisher refuses its own non-increasing stamps
    frameless = CommandPublisher(a, "robot", "arms", "")
    frameless.send({"left_tcp": POSE}, stamp_ns=6_000)
    wait_until(lambda: len(refusals) == 2)
    assert "frame" in refusals[1] and len(got) == 1
    first.send(joints={"left_shoulder_1": 0.2}, stamp_ns=7_000)  # refusals do not block what follows
    wait_until(lambda: len(got) == 2)

    # a wall-clock stamp too old (a backlog) or from the future (the clocks disagree): refused
    clock = CommandPublisher(a, "robot", "arms", ROOT)
    clock.send(joints={"left_shoulder_1": 0.2}, stamp_ns=time.time_ns() - 2_000_000_000)
    wait_until(lambda: len(refusals) == 3)
    assert "from now" in refusals[2] and len(got) == 2
    clock.send(joints={"left_shoulder_1": 0.2}, stamp_ns=time.time_ns() + 60_000_000_000)
    wait_until(lambda: len(refusals) == 4)
    assert "from now" in refusals[3] and len(got) == 2
    clock.close()
    # a joint or a frame twice: refused (as the bridge does), not the last one taken
    from xblick.controller import command_message
    message = command_message(Command(8_000, ROOT, {"left_tcp": POSE}, {"left_shoulder_1": 0.1}))
    message.joint_state.name.append("left_shoulder_1")
    message.joint_state.position = np.array([0.1, 0.2])
    with pytest.raises(ValueError, match="twice"):
        command_of(message)
    reader.check()
    reader.close()
    for p in (first, second, frameless):
        p.close()


def fields_and_arms():
    joints = ["A1", "A2"]
    fields = ["record", "cycle", "time"]
    for group in ("q", "dq", "torque"):
        fields += [f"left/{group}/{j}" for j in joints]
    for pose in ("measured_pose", "reference_pose", "target_pose"):
        fields += [f"left/{pose}/{c}" for c in ("x", "y", "z", "qw", "qx", "qy", "qz")]
    fields += [f"left/commanded_wrench/{c}" for c in ("fx", "fy", "fz", "tx", "ty", "tz")]
    return fields, [ArmFields("left", "left_tcp", joints)]


def test_records_typed_state_hands_and_diagnostics(sessions):
    a, b = sessions
    fields, arms = fields_and_arms()
    parameters = Parameters(a, "arms", "robot/controller")  # the controller's node: record_fields
    parameters.declare("record_fields", fields, read_only=True)
    state = ControllerPublisher(a, "robot", "arms", fields, arms, ROOT)

    batches, hands, statuses, fast = [], [], [], []
    joints, tf, wrench = [], [], []
    key = topics.controller("robot", "arms")
    readers = [RobotStateSubscriber(b, topics.joint_states(key), JointState, joints.append),
               RobotStateSubscriber(b, topics.tf(key), TFMessage, tf.append),
               RobotStateSubscriber(b, topics.wrench(key, "left"), WrenchStamped, wrench.append)]
    client = ControllerSubscriber(b, "robot", "arms", lambda r, f: batches.append(r.copy()),
                                  on_hands=lambda n, p: hands.append((n, p)), on_diagnostics=statuses.append,
                                  on_status=fast.append)
    assert client.fields == fields
    time.sleep(0.3)

    records = np.zeros((3, len(fields)))
    for k in range(3):
        records[k, fields.index("record")] = k
        records[k, fields.index("time")] = 10.0 + 0.001 * k
        records[k, fields.index("left/q/A2")] = 0.5 + k
        records[k, fields.index("left/measured_pose/x")] = 0.4
        records[k, fields.index("left/measured_pose/qw")] = 1.0
        records[k, fields.index("left/reference_pose/qw")] = 1.0
        records[k, fields.index("left/target_pose/qw")] = 1.0
        records[k, fields.index("left/commanded_wrench/fz")] = 2.5
    state.records(records)
    state.hands({"LeftHandFingers": 0.25, "LeftHandThumb": float("nan")})
    state.diagnostics(state._diagnostics.OK, "active, normal", {"plant": "test"})
    state.status({"active": "true", "commands_rejected": 2})

    wait_until(lambda: batches and len(joints) == 3 and len(tf) == 3 and len(wrench) == 3 and hands and statuses
               and fast)
    assert {v.key: v.value for v in fast[0].values} == {"active": "true", "commands_rejected": "2"}
    assert fast[0].hardware_id == "arms"
    np.testing.assert_array_equal(batches[0], records)
    assert joints[2].name == ["A1", "A2"] and joints[2].position[1] == pytest.approx(2.5)
    assert joints[2].header.stamp.to_ns() == 10_002_000_000
    assert [t.child_frame_id for t in tf[0].transforms] == ["left_tcp/measured", "left_tcp/reference",
                                                            "left_tcp/target"]
    assert tf[0].transforms[0].header.frame_id == ROOT and tf[0].transforms[0].transform.translation.x == 0.4
    assert tf[0].transforms[0].transform.rotation.w == 1.0
    assert wrench[0].wrench.force.z == 2.5 and wrench[0].header.frame_id == ROOT
    names, positions = hands[0]
    assert names == ["LeftHandFingers", "LeftHandThumb"] and positions[0] == 0.25 and np.isnan(positions[1])
    assert statuses[0].message == "active, normal" and {v.key: v.value for v in statuses[0].values}["plant"] == "test"
    client.check()
    for r in readers:
        r.check()
        r.close()
    client.close()
    state.close()


def test_subscriber_refuses_records_of_another_layout(sessions):
    a, b = sessions
    fields, arms = fields_and_arms()
    parameters = Parameters(a, "arms", "robot/controller")
    parameters.declare("record_fields", fields[:-1], read_only=True)  # the client is told one value less
    state = ControllerPublisher(a, "robot", "arms", fields, arms, ROOT, typed=False)
    client = ControllerSubscriber(b, "robot", "arms", lambda r, f: None)
    time.sleep(0.3)
    state.records(np.zeros((1, len(fields))))
    wait_until(lambda: client._error is not None)
    with pytest.raises(ValueError, match="the fields name"):
        client.check()
    client.close()
    state.close()
