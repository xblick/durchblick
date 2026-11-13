"""Generates ``src/xblick/msgdefs.py``: the ROS 2 message definitions (``ros2msg``, as rosbag2 writes them into
MCAP schemas) of the ROS types Durchblick carries, so that its recordings open in ``ros2 bag``, Foxglove and other MCAP
readers. Run again when a ROS type is added to the protocol table (cpp/include/xblick/protocol.hpp).

The definitions come from rosbags' type store of ROS 2 Jazzy; vision_msgs (not in it) from a checkout of
ros-perception/vision_msgs (branch ros2, version 4.x); moveit_msgs/RobotState with the types it needs
(AttachedCollisionObject, CollisionObject) from a checkout of moveit/moveit_msgs (branch ros2) and
object_recognition_msgs/ObjectType from one of wg-perception/object_recognition_msgs (branch ros2). Each type is
checked against Durchblick's own CDR: an example message serialized by ``xblick.msg`` must deserialize with the
definition.

    uv venv /tmp/gen && uv pip install -p /tmp/gen rosbags && uv pip install -p /tmp/gen -e .
    /tmp/gen/bin/python tools/gen_msgdefs.py <vision_msgs> <moveit_msgs> <object_recognition_msgs> \
        > src/xblick/msgdefs.py
"""

import sys
from pathlib import Path

import numpy as np

from rosbags.typesys import Stores, get_types_from_msg, get_typestore

from xblick import msg
from xblick._core import protocol

STORE = Stores.ROS2_JAZZY


# from the other checkouts only what RobotState needs (the rest of moveit_msgs needs packages Jazzy's store lacks)
MOVEIT_MSGS = ("RobotState", "AttachedCollisionObject", "CollisionObject")
OBJECT_RECOGNITION_MSGS = ("ObjectType",)


def typestore(vision_msgs: Path, moveit_msgs: Path, object_recognition_msgs: Path):
    store = get_typestore(STORE)
    types = {}
    for path in sorted((vision_msgs / "vision_msgs" / "msg").glob("*.msg")):
        types.update(get_types_from_msg(path.read_text(), f"vision_msgs/msg/{path.stem}"))
    for package, checkout, names in (("moveit_msgs", moveit_msgs, MOVEIT_MSGS),
                                     ("object_recognition_msgs", object_recognition_msgs, OBJECT_RECOGNITION_MSGS)):
        for name in names:
            types.update(get_types_from_msg((checkout / "msg" / f"{name}.msg").read_text(),
                                            f"{package}/msg/{name}"))
    store.register(types)
    return store


def examples():
    """One filled message per ROS type of the protocol table, from xblick.msg."""
    h = msg.Header(msg.Time(12, 345), "cam")
    pose = msg.Pose(msg.Point(0.1, 0.2, 0.3), msg.Quaternion(0.0, 0.0, 0.0, 1.0))
    hyp = msg.ObjectHypothesisWithPose(msg.ObjectHypothesis("mug", 0.5))
    tf = msg.TransformStamped(h, "child", msg.Transform(msg.Vector3(1.0, 2.0, 3.0), msg.Quaternion(0.0, 0.0, 0.0, 1.0)))
    return {
        "sensor_msgs/msg/CompressedImage": msg.CompressedImage(h, "jpeg", b"\x01\x02"),
        "sensor_msgs/msg/CameraInfo": msg.CameraInfo.pinhole(64, 48, 50.0, 50.0, 31.5, 23.5, frame_id="cam"),
        "sensor_msgs/msg/Image": msg.Image(h, 1, 2, "mono8", 0, 2, b"\x01\x02"),
        "sensor_msgs/msg/Imu": msg.Imu(h),
        "sensor_msgs/msg/JointState": msg.JointState(h, ["a"], [1.0], [], []),
        "sensor_msgs/msg/PointCloud2": msg.PointCloud2(h, 1, 1, [msg.PointField("x", 0, msg.PointField.FLOAT32, 1)],
                                                        False, 4, 4, b"\x00\x00\x80\x3f", True),
        "tf2_msgs/msg/TFMessage": msg.TFMessage([tf]),
        "nav_msgs/msg/Odometry": msg.Odometry(h, "base"),
        "geometry_msgs/msg/WrenchStamped": msg.WrenchStamped(h),
        "diagnostic_msgs/msg/DiagnosticArray": msg.DiagnosticArray(h, [msg.DiagnosticStatus(
            msg.DiagnosticStatus.OK, "cam", "ok", "", [msg.KeyValue("fps", "30")])]),
        "vision_msgs/msg/Detection2DArray": msg.Detection2DArray(h, [msg.Detection2D(
            h, [hyp], msg.BoundingBox2D(msg.Pose2D(msg.Point2D(1.0, 2.0), 0.5), 3.0, 4.0), "7")]),
        "vision_msgs/msg/Detection3DArray": msg.Detection3DArray(h, [msg.Detection3D(
            h, [hyp], msg.BoundingBox3D(pose, msg.Vector3(0.1, 0.2, 0.3)), "7")]),
        "vision_msgs/msg/Classification": msg.Classification(h, [msg.ObjectHypothesis("opening something", 0.5)]),
        "rosgraph_msgs/msg/Clock": msg.Clock(msg.Time(12, 345)),
        "std_msgs/msg/Float64MultiArray": msg.Float64MultiArray(
            msg.MultiArrayLayout([msg.MultiArrayDimension("record", 1, 2), msg.MultiArrayDimension("value", 2, 2)]),
            np.array([0.5, -1.0])),
        "moveit_msgs/msg/RobotState": msg.RobotState(
            msg.JointState(h, ["a", "b"], np.array([0.25, -0.5])),
            msg.MultiDOFJointState(msg.Header(msg.Time(12, 345), "root"), ["left_tcp"],
                                   [msg.Transform(msg.Vector3(0.1, 0.2, 0.3), msg.Quaternion(0.0, 0.0, 0.6, 0.8))])),
    }


def main():
    store = typestore(Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3]))
    # xblick_msgs: not ROS types; rcl_interfaces: a node's parameter services and events, not recorded streams
    ros_types = sorted(t for t in protocol.TYPES if not t.startswith(("xblick_msgs/", "rcl_interfaces/")))
    filled = examples()
    missing = set(ros_types) - set(filled)
    if missing:
        raise SystemExit(f"no example message for {sorted(missing)}: add one to examples()")
    lines = ["# ruff: noqa: E501 - generated", '"""ROS 2 message definitions (ros2msg) of the ROS types Durchblick carries, for the schemas of its MCAP',
             "recordings. GENERATED by tools/gen_msgdefs.py (rosbags' ROS 2 Jazzy store; vision_msgs 4.x; moveit_msgs and object_recognition_msgs, branch ros2): do not edit.",
             '"""', "", "MSGDEFS = {"]
    for name in ros_types:
        text, _ = store.generate_msgdef(name, ros_version=2)
        # the check: Durchblick's CDR of the type deserializes with this definition, to the same field values
        data = bytes(filled[name].serialize())
        decoded = store.deserialize_cdr(data, name)
        again = bytes(store.serialize_cdr(decoded, name))
        if again != data:
            raise SystemExit(f"{name}: Durchblick's CDR does not round-trip through the definition")
        lines.append(f"    {name!r}: {text!r},")
    lines += ["}", ""]
    sys.stdout.write("\n".join(lines))


if __name__ == "__main__":
    main()
