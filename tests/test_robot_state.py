"""A robot's state (robot.py): JointState, Odometry and WrenchStamped against rosbags (the same CDR bytes as ROS 2
writes them, and read back), and publisher and subscriber over real Zenoh sessions."""

import socket
import time

import numpy as np
import pytest

from xblick import topics
from xblick.camera import ProtocolError
from xblick.msg import (Header, JointState, Odometry, Point, Pose, PoseWithCovariance, Quaternion, TFMessage, Time,
                        Transform, TransformStamped, Twist, TwistWithCovariance, Vector3, Wrench, WrenchStamped)
from xblick.robot import RobotStatePublisher, RobotStateSubscriber
from xblick.session import open_session

typesys = pytest.importorskip("rosbags.typesys")
STORE = typesys.get_typestore(typesys.Stores.ROS2_LYRICAL)
T = STORE.types

FRAME_IDS = ["", "c", "root", "left_wrist"]


def ros_header(stamp_ns, frame_id):
    return T["std_msgs/msg/Header"](stamp=T["builtin_interfaces/msg/Time"](sec=stamp_ns // 10**9,
                                                                            nanosec=stamp_ns % 10**9),
                                    frame_id=frame_id)


def v3(x, y, z):
    return T["geometry_msgs/msg/Vector3"](x=x, y=y, z=z)


@pytest.mark.parametrize("frame_id", FRAME_IDS)
@pytest.mark.parametrize("count", [0, 1, 3])
@pytest.mark.parametrize("quantities", [("position",), ("position", "velocity", "effort"), ()])
def test_joint_state_matches_rosbags(frame_id, count, quantities):
    names = [f"Joint{k}" * (k + 1) for k in range(count)]
    values = {q: np.arange(count) * (0.1 + i) if q in quantities else np.zeros(0)
              for i, q in enumerate(("position", "velocity", "effort"))}
    ours = JointState(Header(Time(2, 3), frame_id), names, **values)
    theirs = STORE.serialize_cdr(T["sensor_msgs/msg/JointState"](header=ros_header(2 * 10**9 + 3, frame_id),
                                                                  name=names, **values),
                                 "sensor_msgs/msg/JointState")
    assert bytes(ours.serialize()) == bytes(theirs)
    assert JointState.deserialize(theirs) == ours


def test_joint_state_refuses_other_lengths():
    with pytest.raises(ValueError, match="2 values for 3 joints"):
        JointState(name=["a", "b", "c"], position=np.zeros(3), velocity=np.zeros(2)).serialize()


@pytest.mark.parametrize("frame_id", FRAME_IDS)
@pytest.mark.parametrize("child", ["", "root"])
def test_odometry_matches_rosbags(frame_id, child):
    cov = np.arange(36) * 0.01
    ours = Odometry(Header(Time(4, 5), frame_id), child,
                    PoseWithCovariance(Pose(Point(1.0, -2.0, 0.0), Quaternion(0.0, 0.0, 0.6, 0.8)), cov),
                    TwistWithCovariance(Twist(Vector3(0.3, 0.1, 0.0), Vector3(0.0, 0.0, -0.2)), cov * 2))
    theirs = STORE.serialize_cdr(T["nav_msgs/msg/Odometry"](
        header=ros_header(4 * 10**9 + 5, frame_id), child_frame_id=child,
        pose=T["geometry_msgs/msg/PoseWithCovariance"](
            pose=T["geometry_msgs/msg/Pose"](position=T["geometry_msgs/msg/Point"](x=1.0, y=-2.0, z=0.0),
                                             orientation=T["geometry_msgs/msg/Quaternion"](x=0.0, y=0.0, z=0.6, w=0.8)),
            covariance=cov),
        twist=T["geometry_msgs/msg/TwistWithCovariance"](
            twist=T["geometry_msgs/msg/Twist"](linear=v3(0.3, 0.1, 0.0), angular=v3(0.0, 0.0, -0.2)),
            covariance=cov * 2)), "nav_msgs/msg/Odometry")
    assert bytes(ours.serialize()) == bytes(theirs)
    assert Odometry.deserialize(theirs) == ours


@pytest.mark.parametrize("frame_id", FRAME_IDS)
def test_wrench_stamped_matches_rosbags(frame_id):
    ours = WrenchStamped(Header(Time(6, 7), frame_id), Wrench(Vector3(1.5, -2.0, 9.81), Vector3(0.1, 0.2, -0.3)))
    theirs = STORE.serialize_cdr(T["geometry_msgs/msg/WrenchStamped"](
        header=ros_header(6 * 10**9 + 7, frame_id),
        wrench=T["geometry_msgs/msg/Wrench"](force=v3(1.5, -2.0, 9.81), torque=v3(0.1, 0.2, -0.3))),
        "geometry_msgs/msg/WrenchStamped")
    assert bytes(ours.serialize()) == bytes(theirs)
    assert WrenchStamped.deserialize(theirs) == ours


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


def test_robot_state_over_zenoh(sessions):
    a, b = sessions
    got = {"joints": [], "tf": [], "odom": [], "wrench": []}
    readers = [RobotStateSubscriber(b, topics.joint_states("robot"), JointState, got["joints"].append),
               RobotStateSubscriber(b, topics.tf("robot"), TFMessage, got["tf"].append),
               RobotStateSubscriber(b, topics.odom("robot"), Odometry, got["odom"].append),
               RobotStateSubscriber(b, topics.wrench("robot", "left"), WrenchStamped, got["wrench"].append)]
    time.sleep(0.3)  # the subscribers reach the other session
    state = RobotStatePublisher(a, "robot")
    joints = JointState(Header(Time(1, 0)), ["neck_yaw", "neck_pitch"], np.array([0.1, -0.2]))
    tf = TFMessage([TransformStamped(Header(Time(1, 0), topics.WORLD), topics.MAP, Transform()),
                    TransformStamped(Header(Time(1, 0), "root"), "neck_pitch",
                                     Transform(Vector3(0.0, 0.1, 1.6), Quaternion(0.0, 0.1, 0.0, 0.995)))])
    odom = Odometry(Header(Time(1, 0), topics.ODOM_FRAME), "root")
    wrench = WrenchStamped(Header(Time(1, 0), "FT L"), Wrench(Vector3(0.0, 0.0, -4.0)))
    state.joint_states(joints)
    state.tf(tf)
    state.odom(odom)
    state.wrench("left", wrench)
    wait_until(lambda: all(got.values()))
    for reader in readers:
        reader.check()
        reader.close()
    assert (got["joints"][0], got["tf"][0], got["odom"][0], got["wrench"][0]) == (joints, tf, odom, wrench)
    with pytest.raises(TypeError, match="JointState expected"):
        state.joint_states(odom)
    state.close()


def test_subscriber_refuses_another_type(sessions):
    a, b = sessions
    got = []
    reader = RobotStateSubscriber(b, topics.odom("robot"), JointState, got.append)  # odom carries Odometry
    time.sleep(0.3)
    state = RobotStatePublisher(a, "robot")
    state.odom(Odometry())
    wait_until(lambda: reader._error is not None)
    with pytest.raises(ProtocolError):
        reader.check()
    assert not got
    reader.close()
    state.close()


def test_topics():
    assert topics.joint_states("robot") == "robot/joint_states"
    assert topics.odom("/robot") == "robot/odom"
    assert topics.wrench("robot", "left") == "robot/wrench/left"
    with pytest.raises(topics.TopicError):
        topics.wrench("robot", "FT L")
