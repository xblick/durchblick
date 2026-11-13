"""Poses at a stamp (poses.py): interpolation, the buffer's rules (exact, between two, gap, never extrapolated), and a
robot's frames in the world from <robot>/tf over Zenoh."""

import socket
import time

import numpy as np
import pytest

from blick import topics
from blick.camera import TransformPublisher
from blick.msg import Header, Quaternion, TFMessage, Time, Transform, TransformStamped, Vector3
from blick.poses import (CameraPoses, PoseBuffer, RobotPoses, interpolate, matrix_of, quaternion_from_rotation,
                              rotation_from_quaternion)
from blick.robot import RobotStatePublisher
from blick.session import open_session


def yaw(angle: float, t=(0.0, 0.0, 0.0)) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] = [[np.cos(angle), -np.sin(angle), 0.0], [np.sin(angle), np.cos(angle), 0.0], [0.0, 0.0, 1.0]]
    m[:3, 3] = t
    return m


def test_quaternions_round_trip():
    rng = np.random.default_rng(0)
    for q in rng.normal(size=(20, 4)):
        q /= np.linalg.norm(q)
        r = rotation_from_quaternion(*q)
        back = np.array(quaternion_from_rotation(r))
        assert np.allclose(back, q if q[3] >= 0 else -q)


def test_interpolate():
    assert np.allclose(interpolate(yaw(0.0), yaw(np.pi / 2, (1, 2, 0)), 0.5), yaw(np.pi / 4, (0.5, 1.0, 0.0)))
    # the shorter arc: from +170 deg to -170 deg passes 180
    assert np.allclose(interpolate(yaw(np.radians(170)), yaw(np.radians(-170)), 0.5), yaw(np.pi))
    stack = interpolate(np.stack([yaw(0.0), yaw(1.0)]), np.stack([yaw(1.0), yaw(1.0)]), 0.5)
    assert np.allclose(stack, np.stack([yaw(0.5), yaw(1.0)]))


def test_buffer():
    buffer = PoseBuffer(keep=10, max_gap=100e-9)
    buffer.add(1000, yaw(0.0))
    assert buffer.at(1000)[1] and np.allclose(buffer.at(1000)[0], yaw(0.0))
    assert buffer.at(1050) == (None, False)  # nothing after it yet: it may still come
    buffer.add(1100, yaw(1.0))
    value, final = buffer.at(1050)
    assert final and np.allclose(value, yaw(0.5))
    buffer.add(1300, yaw(1.0))
    assert buffer.at(1200) == (None, True)  # 200 ns apart: more than max_gap
    assert buffer.at(900) == (None, True)  # before the first: never extrapolated
    assert buffer.wait(2000, 0.05) is None  # waited for, nothing came


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
    b.close()
    a.close()


def stamped(parent, child, pose, stamp_ns):
    x, y, z, w = quaternion_from_rotation(pose[:3, :3])
    return TransformStamped(Header(Time.from_ns(stamp_ns), parent), child,
                            Transform(Vector3(*pose[:3, 3]), Quaternion(x, y, z, w)))


def test_robot_and_camera_poses_over_zenoh(sessions):
    a, b = sessions
    robot = RobotPoses(b, "robot")
    camera = CameraPoses(b, "robot/camera")
    time.sleep(0.3)  # the subscribers reach the other session
    state = RobotStatePublisher(a, "robot")
    camera_tf = TransformPublisher(a, "robot/camera")
    for stamp, angle in ((1_000_000_000, 0.0), (1_010_000_000, 0.2)):
        state.tf(TFMessage([stamped(topics.WORLD, topics.MAP, yaw(0.5, (2, 1, 0)), stamp),
                            stamped(topics.MAP, topics.ODOM_FRAME, np.eye(4), stamp),
                            stamped(topics.ODOM_FRAME, "root", yaw(angle), stamp),
                            stamped("root", "Neck", yaw(0.0, (0, 0, 1.6)), stamp)]))
        camera_tf.publish([stamped(topics.WORLD, "AzureKinectCamera", yaw(angle, (0, 0, 1.7)), stamp)])
    frames = robot.wait(1_005_000_000, 2.0)
    assert frames is not None and robot.frames == [topics.MAP, topics.ODOM_FRAME, "root", "Neck"]
    assert np.allclose(frames["Neck"], yaw(0.5, (2, 1, 0)) @ yaw(0.1) @ yaw(0.0, (0, 0, 1.6)))
    assert np.allclose(camera.wait(1_005_000_000, 2.0), yaw(0.1, (0, 0, 1.7)))
    assert camera.frame == "AzureKinectCamera"
    assert np.allclose(matrix_of(stamped("a", "b", yaw(0.3, (1, 2, 3)), 0)), yaw(0.3, (1, 2, 3)))
    for closable in (robot, camera, state):
        closable.close()
