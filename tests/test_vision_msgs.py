"""vision_msgs' Detection2DArray, Detection3DArray and Classification (msg.py) against rosbags, with the message definitions of
ros-perception/vision_msgs 4.1.1 (branch humble, commit 5f35e55; Apache License 2.0, tests/data/vision_msgs/LICENSE)
registered in its type store: the same CDR bytes as ROS 2 writes them, and read back."""

from pathlib import Path

import numpy as np
import pytest

from blick.msg import (BoundingBox2D, BoundingBox3D, Classification, Detection2D, Detection2DArray, Detection3D,
                            Detection3DArray, Header, ObjectHypothesis, ObjectHypothesisWithPose, Point, Point2D,
                            Pose, Pose2D, PoseWithCovariance, Quaternion, Time, Vector3)

typesys = pytest.importorskip("rosbags.typesys")
STORE = typesys.get_typestore(typesys.Stores.ROS2_LYRICAL)
DEFINITIONS = Path(__file__).parent / "data" / "vision_msgs"
for definition in sorted(DEFINITIONS.glob("*.msg")):
    STORE.register(typesys.get_types_from_msg(definition.read_text(), f"vision_msgs/msg/{definition.stem}"))
T = STORE.types

FRAME_IDS = ["", "c", "AzureKinectCamera"]
LABELS = ["", "a", "coffee mug"]


def ros_header(stamp_ns, frame_id):
    return T["std_msgs/msg/Header"](stamp=T["builtin_interfaces/msg/Time"](sec=stamp_ns // 10**9,
                                                                            nanosec=stamp_ns % 10**9),
                                    frame_id=frame_id)


def hypotheses(k, label):
    """(ours, theirs): k hypotheses with labels of different lengths and a pose with a covariance"""
    ours, theirs = [], []
    for j in range(k):
        cov = np.arange(36) * 0.01 * (j + 1)
        position, orientation = (0.1 * j, -0.2, 1.5), (0.0, 0.1 * j, 0.0, 0.99)
        ours.append(ObjectHypothesisWithPose(ObjectHypothesis(label * j, 0.25 * j),
                                             PoseWithCovariance(Pose(Point(*position), Quaternion(*orientation)), cov)))
        theirs.append(T["vision_msgs/msg/ObjectHypothesisWithPose"](
            hypothesis=T["vision_msgs/msg/ObjectHypothesis"](class_id=label * j, score=0.25 * j),
            pose=T["geometry_msgs/msg/PoseWithCovariance"](
                pose=T["geometry_msgs/msg/Pose"](
                    position=T["geometry_msgs/msg/Point"](x=position[0], y=position[1], z=position[2]),
                    orientation=T["geometry_msgs/msg/Quaternion"](x=orientation[0], y=orientation[1],
                                                                  z=orientation[2], w=orientation[3])),
                covariance=cov)))
    return ours, theirs


@pytest.mark.parametrize("frame_id", FRAME_IDS)
@pytest.mark.parametrize("label", LABELS)
@pytest.mark.parametrize("count", [0, 1, 3])
def test_detection3d_array_matches_rosbags(frame_id, label, count):
    ours_d, theirs_d = [], []
    for k in range(count):
        ours_h, theirs_h = hypotheses(k, label)
        q = (0.0, 0.0, np.sin(0.1 * k), np.cos(0.1 * k))
        ours_d.append(Detection3D(Header(Time(4, k), frame_id), ours_h,
                                  BoundingBox3D(Pose(Point(0.1 * k, 0.2, 1.0), Quaternion(*q)),
                                                Vector3(0.3, 0.2 * k, 0.1)), str(k) * k))
        theirs_d.append(T["vision_msgs/msg/Detection3D"](
            header=ros_header(4 * 10**9 + k, frame_id), results=theirs_h,
            bbox=T["vision_msgs/msg/BoundingBox3D"](
                center=T["geometry_msgs/msg/Pose"](position=T["geometry_msgs/msg/Point"](x=0.1 * k, y=0.2, z=1.0),
                                                   orientation=T["geometry_msgs/msg/Quaternion"](
                                                       x=q[0], y=q[1], z=q[2], w=q[3])),
                size=T["geometry_msgs/msg/Vector3"](x=0.3, y=0.2 * k, z=0.1)),
            id=str(k) * k))
    ours = Detection3DArray(Header(Time(3, 2), frame_id), ours_d)
    theirs = STORE.serialize_cdr(T["vision_msgs/msg/Detection3DArray"](header=ros_header(3 * 10**9 + 2, frame_id),
                                                                       detections=theirs_d),
                                 "vision_msgs/msg/Detection3DArray")
    assert bytes(ours.serialize()) == bytes(theirs)
    assert Detection3DArray.deserialize(theirs) == ours


@pytest.mark.parametrize("frame_id", FRAME_IDS)
@pytest.mark.parametrize("label", LABELS)
@pytest.mark.parametrize("count", [0, 1, 3])
def test_detection2d_array_matches_rosbags(frame_id, label, count):
    ours_d, theirs_d = [], []
    for k in range(count):
        ours_h, theirs_h = hypotheses(k, label)
        ours_d.append(Detection2D(Header(Time(6, k), frame_id), ours_h,
                                  BoundingBox2D(Pose2D(Point2D(320.5 + k, 240.25), 0.1 * k), 40.0, 30.0 + k),
                                  str(k) * k))
        theirs_d.append(T["vision_msgs/msg/Detection2D"](
            header=ros_header(6 * 10**9 + k, frame_id), results=theirs_h,
            bbox=T["vision_msgs/msg/BoundingBox2D"](
                center=T["vision_msgs/msg/Pose2D"](position=T["vision_msgs/msg/Point2D"](x=320.5 + k, y=240.25),
                                                   theta=0.1 * k),
                size_x=40.0, size_y=30.0 + k),
            id=str(k) * k))
    ours = Detection2DArray(Header(Time(5, 1), frame_id), ours_d)
    theirs = STORE.serialize_cdr(T["vision_msgs/msg/Detection2DArray"](header=ros_header(5 * 10**9 + 1, frame_id),
                                                                       detections=theirs_d),
                                 "vision_msgs/msg/Detection2DArray")
    assert bytes(ours.serialize()) == bytes(theirs)
    assert Detection2DArray.deserialize(theirs) == ours


@pytest.mark.parametrize("frame_id", FRAME_IDS)
@pytest.mark.parametrize("label", LABELS)
@pytest.mark.parametrize("count", [0, 1, 3])
def test_classification_matches_rosbags(frame_id, label, count):
    ours = Classification(Header(Time(7, 3), frame_id), [ObjectHypothesis(label * k, 0.3 * k) for k in range(count)])
    theirs = STORE.serialize_cdr(T["vision_msgs/msg/Classification"](
        header=ros_header(7 * 10**9 + 3, frame_id),
        results=[T["vision_msgs/msg/ObjectHypothesis"](class_id=label * k, score=0.3 * k) for k in range(count)]),
        "vision_msgs/msg/Classification")
    assert bytes(ours.serialize()) == bytes(theirs)
    assert Classification.deserialize(theirs) == ours


def test_definitions_are_the_registered_ones():
    """The eleven files are there and rosbags knows the three message types from them (not from somewhere else)."""
    assert {p.stem for p in DEFINITIONS.glob("*.msg")} == {
        "BoundingBox2D", "BoundingBox3D", "Classification", "Detection2D", "Detection2DArray", "Detection3D",
        "Detection3DArray", "ObjectHypothesis", "ObjectHypothesisWithPose", "Point2D", "Pose2D"}
    assert {"vision_msgs/msg/Detection3DArray", "vision_msgs/msg/Detection2DArray",
            "vision_msgs/msg/Classification"} <= set(T)
