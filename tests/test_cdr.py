"""The CDR bytes of our messages against rosbags, an independent ROS 2 implementation (types of ROS 2 Lyrical)."""

import numpy as np
import pytest

from xblick.cdr import CdrError
from xblick.msg import (CameraInfo, Clock, CompressedImage, CudaPointCloud2, DiagnosticArray, DiagnosticStatus, Header,
                        Image, Imu, KeyValue, PointCloud2, PointField, Quaternion, RegionOfInterest,
                        ShmPointCloud2, TFMessage, Time, Transform, TransformStamped, Vector3)

rosbags = pytest.importorskip("rosbags.typesys")
STORE = rosbags.get_typestore(rosbags.Stores.ROS2_LYRICAL)
T = STORE.types


def ros_header(stamp_ns, frame_id):
    return T["std_msgs/msg/Header"](stamp=T["builtin_interfaces/msg/Time"](sec=stamp_ns // 10**9,
                                                                            nanosec=stamp_ns % 10**9),
                                    frame_id=frame_id)


# frame ids of different lengths move everything after them to other alignments
FRAME_IDS = ["", "c", "cam", "AzureKinectRGBCamera", "Roboception_LeftCamera"]


@pytest.mark.parametrize("frame_id", FRAME_IDS)
@pytest.mark.parametrize("encoding", ["rgb8", "16UC1", "32FC1"])
def test_image_matches_rosbags(frame_id, encoding):
    stamp = 1_761_480_000_123_456_789
    data = np.arange(2 * 3 * 4, dtype=np.uint8)
    ours = Image(header=Header(Time.from_ns(stamp), frame_id), height=2, width=3, encoding=encoding,
                 is_bigendian=0, step=12, data=data.tobytes()).serialize()
    theirs = STORE.serialize_cdr(T["sensor_msgs/msg/Image"](header=ros_header(stamp, frame_id), height=2, width=3,
                                                             encoding=encoding, is_bigendian=0, step=12, data=data),
                                 "sensor_msgs/msg/Image")
    assert bytes(ours) == bytes(theirs)
    back = Image.deserialize(theirs)
    assert back.header.stamp.to_ns() == stamp and back.header.frame_id == frame_id
    assert (back.height, back.width, back.encoding, back.step) == (2, 3, encoding, 12)
    assert bytes(back.data) == data.tobytes()


@pytest.mark.parametrize("frame_id", FRAME_IDS)
def test_compressed_image_matches_rosbags(frame_id):
    data = np.frombuffer(b"\xff\xd8 not really a jpeg \xff\xd9", np.uint8)
    ours = CompressedImage(header=Header(Time(5, 7), frame_id), format="rgb8; jpeg compressed bgr8",
                           data=data.tobytes()).serialize()
    theirs = STORE.serialize_cdr(T["sensor_msgs/msg/CompressedImage"](header=ros_header(5 * 10**9 + 7, frame_id),
                                                                       format="rgb8; jpeg compressed bgr8", data=data),
                                 "sensor_msgs/msg/CompressedImage")
    assert bytes(ours) == bytes(theirs)
    back = CompressedImage.deserialize(theirs)
    assert back.format == "rgb8; jpeg compressed bgr8" and bytes(back.data) == data.tobytes()


@pytest.mark.parametrize("frame_id", FRAME_IDS)
@pytest.mark.parametrize("d", [np.zeros(0), np.arange(5) * 0.1, np.arange(8) * 0.01])
def test_camera_info_matches_rosbags(frame_id, d):
    k = np.array([600.0, 0, 319.5, 0, 600.0, 239.5, 0, 0, 1])
    r = np.eye(3).reshape(-1)
    p = np.array([600.0, 0, 319.5, -45.0, 0, 600.0, 239.5, 0, 0, 0, 1, 0])
    model = {0: "", 5: "plumb_bob", 8: "rational_polynomial"}[d.size]
    ours = CameraInfo(header=Header(Time(1, 2), frame_id), height=480, width=640, distortion_model=model, d=d, k=k,
                      r=r, p=p, binning_x=1, binning_y=2,
                      roi=RegionOfInterest(1, 2, 3, 4, True)).serialize()
    theirs = STORE.serialize_cdr(
        T["sensor_msgs/msg/CameraInfo"](
            header=ros_header(10**9 + 2, frame_id), height=480, width=640, distortion_model=model, d=d, k=k, r=r, p=p,
            binning_x=1, binning_y=2,
            roi=T["sensor_msgs/msg/RegionOfInterest"](x_offset=1, y_offset=2, height=3, width=4, do_rectify=True)),
        "sensor_msgs/msg/CameraInfo")
    assert bytes(ours) == bytes(theirs)
    back = CameraInfo.deserialize(theirs)
    assert back.distortion_model == model and np.array_equal(back.d, d) and np.array_equal(back.p, p)
    assert back.roi == RegionOfInterest(1, 2, 3, 4, True)


@pytest.mark.parametrize("frame_id", FRAME_IDS)
@pytest.mark.parametrize("count", [0, 1, 3])
def test_tf_message_matches_rosbags(frame_id, count):
    def ours_t(k):
        return TransformStamped(Header(Time(7, k), frame_id), f"child{k}" * k,
                                Transform(Vector3(0.1 * k, -0.2, 0.3), Quaternion(0.0, 0.1 * k, 0.0, 0.99)))

    def theirs_t(k):
        return T["geometry_msgs/msg/TransformStamped"](
            header=ros_header(7 * 10**9 + k, frame_id), child_frame_id=f"child{k}" * k,
            transform=T["geometry_msgs/msg/Transform"](
                translation=T["geometry_msgs/msg/Vector3"](x=0.1 * k, y=-0.2, z=0.3),
                rotation=T["geometry_msgs/msg/Quaternion"](x=0.0, y=0.1 * k, z=0.0, w=0.99)))

    ours = TFMessage(transforms=[ours_t(k) for k in range(count)]).serialize()
    theirs = STORE.serialize_cdr(T["tf2_msgs/msg/TFMessage"](transforms=[theirs_t(k) for k in range(count)]),
                                 "tf2_msgs/msg/TFMessage")
    assert bytes(ours) == bytes(theirs)
    assert TFMessage.deserialize(theirs) == TFMessage(transforms=[ours_t(k) for k in range(count)])


@pytest.mark.parametrize("frame_id", FRAME_IDS)
def test_imu_matches_rosbags(frame_id):
    cov = np.arange(9) * 0.5
    ours = Imu(Header(Time(3, 4), frame_id), Quaternion(0.1, 0.2, 0.3, 0.9), cov, Vector3(0.01, -0.02, 0.03), cov * 2,
               Vector3(0.1, -9.81, 0.2), cov * 3)
    v3, q = T["geometry_msgs/msg/Vector3"], T["geometry_msgs/msg/Quaternion"]
    theirs = STORE.serialize_cdr(T["sensor_msgs/msg/Imu"](
        header=ros_header(3 * 10**9 + 4, frame_id), orientation=q(x=0.1, y=0.2, z=0.3, w=0.9),
        orientation_covariance=cov, angular_velocity=v3(x=0.01, y=-0.02, z=0.03), angular_velocity_covariance=cov * 2,
        linear_acceleration=v3(x=0.1, y=-9.81, z=0.2), linear_acceleration_covariance=cov * 3), "sensor_msgs/msg/Imu")
    assert bytes(ours.serialize()) == bytes(theirs)
    assert Imu.deserialize(theirs) == ours
    assert Imu().orientation_covariance[0] == -1  # ROS: no orientation


@pytest.mark.parametrize("name", ["", "a", "azure_kinect"])
@pytest.mark.parametrize("count", [0, 1, 2])
def test_diagnostic_array_matches_rosbags(name, count):
    def ours_s(k):
        return DiagnosticStatus(k % 4, name * k, "message" * k, "hw", [KeyValue(f"k{j}", "v" * j) for j in range(k)])

    def theirs_s(k):
        return T["diagnostic_msgs/msg/DiagnosticStatus"](
            level=k % 4, name=name * k, message="message" * k, hardware_id="hw",
            values=[T["diagnostic_msgs/msg/KeyValue"](key=f"k{j}", value="v" * j) for j in range(k)])

    ours = DiagnosticArray(Header(Time(5, 6), name), [ours_s(k) for k in range(count)])
    theirs = STORE.serialize_cdr(T["diagnostic_msgs/msg/DiagnosticArray"](
        header=ros_header(5 * 10**9 + 6, name), status=[theirs_s(k) for k in range(count)]),
        "diagnostic_msgs/msg/DiagnosticArray")
    assert bytes(ours.serialize()) == bytes(theirs)
    assert DiagnosticArray.deserialize(theirs) == ours


@pytest.mark.parametrize("frame_id", FRAME_IDS)
@pytest.mark.parametrize("fields", [[], [("x", 0, 7, 1)], [("x", 0, 7, 1), ("y", 4, 7, 1), ("z", 8, 7, 1),
                                                           ("rgb", 12, 7, 1)], [("ring", 0, 4, 2), ("t", 8, 8, 1)]])
@pytest.mark.parametrize("is_dense", [False, True])
def test_point_cloud2_matches_rosbags(frame_id, fields, is_dense):
    data = np.arange(2 * 3 * 16, dtype=np.uint8)
    ours = PointCloud2(header=Header(Time(8, 9), frame_id), height=2, width=3,
                       fields=[PointField(*f) for f in fields], point_step=16, row_step=48, data=data.tobytes(),
                       is_dense=is_dense).serialize()
    theirs = STORE.serialize_cdr(T["sensor_msgs/msg/PointCloud2"](
        header=ros_header(8 * 10**9 + 9, frame_id), height=2, width=3,
        fields=[T["sensor_msgs/msg/PointField"](name=n, offset=o, datatype=d, count=c) for n, o, d, c in fields],
        is_bigendian=False, point_step=16, row_step=48, data=data, is_dense=is_dense), "sensor_msgs/msg/PointCloud2")
    assert bytes(ours) == bytes(theirs)
    back = PointCloud2.deserialize(theirs)
    assert back.fields == [PointField(*f) for f in fields] and back.is_dense == is_dense
    assert (back.height, back.width, back.point_step, back.row_step) == (2, 3, 16, 48)
    assert bytes(back.data) == data.tobytes()


def test_clock_matches_rosbags():
    ours = Clock(Time(1_761_480_000, 123_456_789))
    theirs = STORE.serialize_cdr(T["rosgraph_msgs/msg/Clock"](clock=T["builtin_interfaces/msg/Time"](
        sec=1_761_480_000, nanosec=123_456_789)), "rosgraph_msgs/msg/Clock")
    assert bytes(ours.serialize()) == bytes(theirs)
    assert Clock.deserialize(theirs) == ours


@pytest.mark.parametrize("labels", [("record", "value"), ("", "a much longer label"), ()])
def test_float64_multi_array_matches_rosbags(labels):
    from xblick.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout

    sizes = [3, 2][:len(labels)]
    data = np.arange(np.prod(sizes) if sizes else 5, dtype=np.float64) * 0.5 - 1.0
    dims = [MultiArrayDimension(label, size, int(np.prod(sizes[i:]))) for i, (label, size) in
            enumerate(zip(labels, sizes))]
    ours = Float64MultiArray(MultiArrayLayout(dims, 1), data)
    theirs = STORE.serialize_cdr(T["std_msgs/msg/Float64MultiArray"](
        layout=T["std_msgs/msg/MultiArrayLayout"](
            dim=[T["std_msgs/msg/MultiArrayDimension"](label=d.label, size=d.size, stride=d.stride) for d in dims],
            data_offset=1),
        data=data), "std_msgs/msg/Float64MultiArray")
    assert bytes(ours.serialize()) == bytes(theirs)
    assert Float64MultiArray.deserialize(theirs) == ours


def test_rejects_other_encapsulation():
    data = bytearray(Image(encoding="rgb8").serialize())
    data[1] = 0x00  # CDR big endian
    with pytest.raises(CdrError):
        Image.deserialize(data)


def test_rejects_truncated_and_trailing_data():
    data = bytes(Image(encoding="rgb8", data=b"123456").serialize())
    with pytest.raises(CdrError):
        Image.deserialize(data[:-3])
    with pytest.raises(CdrError):
        Image.deserialize(data + b"\x00" * 8)


def test_time_from_ns():
    t = Time.from_ns(1_234_567_890_987_654_321)
    assert (t.sec, t.nanosec) == (1_234_567_890, 987_654_321) and t.to_ns() == 1_234_567_890_987_654_321


def test_cpp_golden_bytes():
    """cpp/tests/test_messages.cpp checks the C++ messages against these bytes; this keeps its constants equal to
    what Python (and therefore ROS, see above) writes. A failure here means the constants there need updating."""
    import re
    from pathlib import Path

    from xblick.msg import (BoundingBox2D, BoundingBox3D, CameraInfo, Classification, CudaImage, Detection2D,
                            Detection2DArray, Detection3D, Detection3DArray, Header, Image, JointState, ObjectHypothesis,
                            ObjectHypothesisWithPose, Odometry, Point, Point2D, Pose, Pose2D, PoseWithCovariance,
                            ShmImage, Time, Twist, TwistWithCovariance, Wrench, WrenchStamped)

    image = Image(header=Header(stamp=Time(sec=1, nanosec=500000000), frame_id="cam"), height=1, width=2,
                  encoding="rgb8", step=6, data=bytes([1, 2, 3, 4, 5, 6]))
    info = CameraInfo.pinhole(640, 480, 500.0, 501.0, 319.5, 239.5, frame_id="cam")
    info.header.stamp = Time(sec=3, nanosec=4)
    info.binning_x = 1
    info.roi.do_rectify = True
    shm = ShmImage(header=Header(stamp=Time(sec=5, nanosec=6), frame_id="f"), height=2, width=3, encoding="16UC1",
                   step=6, host="h", segment="/xblick-x", slot=1, seq=2 ** 40)
    cuda = CudaImage(header=Header(frame_id="g"), height=4, width=5, encoding="rgb8", step=15, host="h2",
                     endpoint="xblick-e", device_uuid="ab" * 16, slot=3, seq=8)
    tf = TFMessage(transforms=[TransformStamped(
        Header(Time(sec=9, nanosec=10), "depth"), "rgb",
        Transform(Vector3(-0.032, 0.002, 0.004), Quaternion(0.1, 0.0, 0.0, 0.995)))])
    imu = Imu(Header(Time(11, 12), "AzureKinectImu"), angular_velocity=Vector3(0.01, -0.02, 0.03),
              linear_acceleration=Vector3(0.1, -9.81, 0.2))
    diagnostics = DiagnosticArray(Header(Time(13, 14), ""), [DiagnosticStatus(
        1, "azure_kinect", "2 dropped", "000123", [KeyValue("captures/s", "30.0"), KeyValue("temperature", "35.5")])])
    xyzrgb = [PointField("x", 0, PointField.FLOAT32), PointField("y", 4, PointField.FLOAT32),
              PointField("z", 8, PointField.FLOAT32), PointField("rgb", 12, PointField.FLOAT32)]
    cloud = PointCloud2(header=Header(Time(15, 16), "depth"), height=1, width=2, fields=xyzrgb, point_step=16,
                        row_step=32, data=bytes(range(32)), is_dense=True)
    shm_cloud = ShmPointCloud2(header=Header(Time(17, 18), "d"), height=2, width=3,
                               fields=[PointField("x", 0, PointField.FLOAT32),
                                       PointField("i", 4, PointField.UINT16, 2)],
                               point_step=8, row_step=24, is_dense=False, host="h", segment="/xblick-y", slot=2,
                               seq=2 ** 33)
    cuda_cloud = CudaPointCloud2(header=Header(frame_id="e"), height=4, width=5, fields=xyzrgb[:3], point_step=16,
                                 row_step=80, is_dense=True, host="h3", endpoint="xblick-f", device_uuid="cd" * 16,
                                 slot=1, seq=9)
    cov = np.zeros(36)
    cov[0], cov[35] = 0.01, 0.5
    hypothesis = ObjectHypothesisWithPose(ObjectHypothesis("mug", 0.75), PoseWithCovariance(
        Pose(Point(0.1, -0.2, 1.5), Quaternion(0.0, 0.0, 0.6, 0.8)), cov))
    boxes = Detection3DArray(Header(Time(19, 20), "AzureKinectCamera"), [Detection3D(
        Header(Time(19, 20), "AzureKinectCamera"), [hypothesis],
        BoundingBox3D(Pose(Point(0.1, -0.2, 1.5), Quaternion(0.0, 0.0, 0.6, 0.8)), Vector3(0.08, 0.12, 0.1)), "7")])
    boxes2d = Detection2DArray(Header(Time(21, 22), "AzureKinectCamera"), [Detection2D(
        Header(Time(21, 22), "AzureKinectCamera"), [ObjectHypothesisWithPose(ObjectHypothesis("chair", 0.5))],
        BoundingBox2D(Pose2D(Point2D(320.5, 240.25), 0.0), 40.0, 30.0), "")])
    joints = JointState(Header(Time(23, 24), ""), ["neck_yaw", "left_shoulder_1"], np.array([0.1, -0.2]),
                        effort=np.array([1.5, 0.0]))
    odom_pose_cov, odom_twist_cov = np.zeros(36), np.zeros(36)
    odom_pose_cov[0], odom_twist_cov[35] = 0.01, 0.5
    odom = Odometry(Header(Time(25, 26), "odom"), "root",
                    PoseWithCovariance(Pose(Point(1.0, -2.0, 0.0), Quaternion(0.0, 0.0, 0.6, 0.8)), odom_pose_cov),
                    TwistWithCovariance(Twist(Vector3(0.3, 0.1, 0.0), Vector3(0.0, 0.0, -0.2)), odom_twist_cov))
    wrench = WrenchStamped(Header(Time(27, 28), "FT L"), Wrench(Vector3(1.5, -2.0, 9.81), Vector3(0.1, 0.2, -0.3)))
    actions = Classification(Header(Time(29, 30), "AzureKinectCamera"),
                             [ObjectHypothesis("Opening something", 0.625), ObjectHypothesis("", 0.125)])
    from xblick.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout
    records = Float64MultiArray(MultiArrayLayout([MultiArrayDimension("record", 2, 6), MultiArrayDimension("value", 3, 3)]),
                                np.array([0.5, -1.0, 2.0, 1e-3, 0.0, 7.0]))
    from xblick.msg import MultiDOFJointState, RobotState
    command = RobotState(JointState(Header(Time(31, 32), ""), ["left_shoulder_1", "LeftHandFingers"],
                                    np.array([0.1, 0.5])),
                         MultiDOFJointState(Header(Time(31, 32), "base_link"), ["left_tcp"],
                                            [Transform(Vector3(0.4, 0.2, 1.0), Quaternion(0.0, 0.0, 0.6, 0.8))]))
    source = (Path(__file__).parent.parent / "cpp" / "tests" / "test_messages.cpp").read_text()
    for name, message in (("kImage", image), ("kCameraInfo", info), ("kShm", shm), ("kCuda", cuda), ("kTf", tf),
                          ("kImu", imu), ("kDiagnostics", diagnostics), ("kCloud", cloud), ("kShmCloud", shm_cloud),
                          ("kCudaCloud", cuda_cloud), ("kDetections3D", boxes), ("kDetections2D", boxes2d),
                          ("kJointState", joints), ("kOdometry", odom), ("kWrench", wrench),
                          ("kClassification", actions), ("kRecords", records), ("kRobotState", command)):
        constant = re.search(rf'const char\* {name} = "([0-9a-f]+)";', source).group(1)
        assert constant == bytes(message.serialize()).hex(), name


# ------------------------------------------------------------------------------------------ rcl_interfaces

from xblick.msg import (DescribeParametersRequest, DescribeParametersResponse, FloatingPointRange,  # noqa: E402
                        GetParametersRequest, GetParametersResponse, GetParameterTypesRequest,
                        GetParameterTypesResponse, IntegerRange, ListParametersRequest, ListParametersResponse,
                        Parameter, ParameterDescriptor, ParameterEvent, ParameterType, ParameterValue,
                        SetParametersAtomicallyRequest, SetParametersAtomicallyResponse, SetParametersRequest,
                        SetParametersResponse, SetParametersResult)

# rcl_interfaces/srv/*, as ROS 2 defines them (the typestore has the messages, not the services)
SERVICES = {
    "rcl_interfaces/srv/ListParameters_Request": "string[] prefixes\nuint64 depth\n",
    "rcl_interfaces/srv/ListParameters_Response": "rcl_interfaces/ListParametersResult result\n",
    "rcl_interfaces/srv/DescribeParameters_Request": "string[] names\n",
    "rcl_interfaces/srv/DescribeParameters_Response": "rcl_interfaces/ParameterDescriptor[] descriptors\n",
    "rcl_interfaces/srv/GetParameters_Request": "string[] names\n",
    "rcl_interfaces/srv/GetParameters_Response": "rcl_interfaces/ParameterValue[] values\n",
    "rcl_interfaces/srv/GetParameterTypes_Request": "string[] names\n",
    "rcl_interfaces/srv/GetParameterTypes_Response": "uint8[] types\n",
    "rcl_interfaces/srv/SetParameters_Request": "rcl_interfaces/Parameter[] parameters\n",
    "rcl_interfaces/srv/SetParameters_Response": "rcl_interfaces/SetParametersResult[] results\n",
    "rcl_interfaces/srv/SetParametersAtomically_Request": "rcl_interfaces/Parameter[] parameters\n",
    "rcl_interfaces/srv/SetParametersAtomically_Response": "rcl_interfaces/SetParametersResult result\n",
}
from rosbags.typesys import get_types_from_msg  # noqa: E402

for _name, _text in SERVICES.items():
    STORE.register(get_types_from_msg(_text, _name))  # rosbags names them rcl_interfaces/srv/msg/<Service>_<Part>
T = STORE.types


def ros(name: str) -> str:
    """rosbags' name of a type (its services' types have a msg part in the name; the bytes are the same)."""
    return name.replace("/srv/", "/srv/msg/") if "/srv/" in name else name

VALUES = [None, True, -(2**40), 0.1, "texts ü", b"\x00\x01\xff", [True, False, True], [1, -2, 2**50],
          [0.5, -1e300], ["a", "", "cc"]]


def ros_value(v: ParameterValue):
    return T["rcl_interfaces/msg/ParameterValue"](
        type=v.type, bool_value=v.bool_value, integer_value=v.integer_value, double_value=v.double_value,
        string_value=v.string_value, byte_array_value=np.frombuffer(v.byte_array_value, np.uint8),
        bool_array_value=np.array(v.bool_array_value, bool), integer_array_value=np.array(v.integer_array_value,
                                                                                            np.int64),
        double_array_value=np.array(v.double_array_value, np.float64), string_array_value=list(v.string_array_value))


def ros_parameter(p: Parameter):
    return T["rcl_interfaces/msg/Parameter"](name=p.name, value=ros_value(p.value))


def same(ours, theirs, name):
    assert bytes(ours.serialize()) == bytes(STORE.serialize_cdr(theirs, ros(name)))
    back = type(ours).deserialize(STORE.serialize_cdr(theirs, ros(name)))
    assert bytes(back.serialize()) == bytes(ours.serialize())
    return back


@pytest.mark.parametrize("node", ["", "/robot/camera/boxer"])
def test_parameter_event_matches_rosbags(node):
    params = [Parameter(f"p{k}", ParameterValue.of(v)) for k, v in enumerate(VALUES)]
    ours = ParameterEvent(Time(12, 34), node, params[:4], params[4:], params[:1])
    theirs = T["rcl_interfaces/msg/ParameterEvent"](
        stamp=T["builtin_interfaces/msg/Time"](sec=12, nanosec=34), node=node,
        new_parameters=[ros_parameter(p) for p in params[:4]],
        changed_parameters=[ros_parameter(p) for p in params[4:]], deleted_parameters=[ros_parameter(params[0])])
    back = same(ours, theirs, "rcl_interfaces/msg/ParameterEvent")
    assert [p.value.value() for p in back.new_parameters + back.changed_parameters] == list(VALUES)


def ros_descriptor(d: ParameterDescriptor):
    return T["rcl_interfaces/msg/ParameterDescriptor"](
        name=d.name, type=d.type, description=d.description, additional_constraints=d.additional_constraints,
        read_only=d.read_only, dynamic_typing=d.dynamic_typing,
        floating_point_range=[T["rcl_interfaces/msg/FloatingPointRange"](from_value=r.from_value, to_value=r.to_value,
                                                                          step=r.step)
                              for r in d.floating_point_range],
        integer_range=[T["rcl_interfaces/msg/IntegerRange"](from_value=r.from_value, to_value=r.to_value, step=r.step)
                       for r in d.integer_range])


def test_parameter_services_match_rosbags():
    descriptors = [
        ParameterDescriptor("threshold", ParameterType.DOUBLE, "a box's score at least", '{"unit":"1"}',
                            floating_point_range=[FloatingPointRange(0.0, 1.0, 0.05)]),
        ParameterDescriptor("jpeg_quality", ParameterType.INTEGER, "", "", read_only=True,
                            integer_range=[IntegerRange(-5, 100, 5)]),
        ParameterDescriptor("mode", ParameterType.STRING, "x", '{"choices":["a","b"]}', dynamic_typing=True)]
    params = [Parameter(f"p{k}", ParameterValue.of(v)) for k, v in enumerate(VALUES)]
    results = [SetParametersResult(True, ""), SetParametersResult(False, "below 0.0")]
    S = "rcl_interfaces/srv/"
    same(ListParametersRequest(["a", "b.c"], 3), T[ros(S + "ListParameters_Request")](prefixes=["a", "b.c"], depth=3),
         S + "ListParameters_Request")
    same(ListParametersResponse(["a.x", "y"], ["a"]), T[ros(S + "ListParameters_Response")](
        result=T["rcl_interfaces/msg/ListParametersResult"](names=["a.x", "y"], prefixes=["a"])),
         S + "ListParameters_Response")
    for request, name in ((DescribeParametersRequest, "DescribeParameters"), (GetParametersRequest, "GetParameters"),
                          (GetParameterTypesRequest, "GetParameterTypes")):
        same(request(["x", "yy", ""]), T[ros(f"{S}{name}_Request")](names=["x", "yy", ""]), f"{S}{name}_Request")
    back = same(DescribeParametersResponse(descriptors), T[ros(S + "DescribeParameters_Response")](
        descriptors=[ros_descriptor(d) for d in descriptors]), S + "DescribeParameters_Response")
    assert back.descriptors == descriptors
    same(GetParametersResponse([p.value for p in params]), T[ros(S + "GetParameters_Response")](
        values=[ros_value(p.value) for p in params]), S + "GetParameters_Response")
    same(GetParameterTypesResponse([1, 3, 9]), T[ros(S + "GetParameterTypes_Response")](
        types=np.array([1, 3, 9], np.uint8)), S + "GetParameterTypes_Response")
    for request, name in ((SetParametersRequest, "SetParameters"),
                          (SetParametersAtomicallyRequest, "SetParametersAtomically")):
        same(request(params), T[ros(f"{S}{name}_Request")](parameters=[ros_parameter(p) for p in params]),
             f"{S}{name}_Request")
    ros_result = T["rcl_interfaces/msg/SetParametersResult"]
    same(SetParametersResponse(results), T[ros(S + "SetParameters_Response")](
        results=[ros_result(successful=r.successful, reason=r.reason) for r in results]), S + "SetParameters_Response")
    same(SetParametersAtomicallyResponse(results[1]), T[ros(S + "SetParametersAtomically_Response")](
        result=ros_result(successful=False, reason="below 0.0")), S + "SetParametersAtomically_Response")


def test_parameter_values_of_python():
    assert [ParameterValue.of(v).type for v in VALUES] == list(range(10))
    assert ParameterValue.of([]).type == ParameterType.STRING_ARRAY and ParameterValue.of([1, 2.5]).type == 8
    with pytest.raises(TypeError, match="no parameter type"):
        ParameterValue.of({"a": 1})


def test_cpp_parameter_golden_bytes():
    """cpp/tests/test_parameters.cpp checks the C++ parameter messages against these bytes (Python's, hence ROS's)."""
    import re
    from pathlib import Path

    from xblick.msg import Time

    event = ParameterEvent(Time(12, 34), "/robot/camera/camera",
                           [Parameter("exposure_ms", ParameterValue.of(8.5)),
                            Parameter("enabled", ParameterValue.of(True))],
                           [Parameter("mode", ParameterValue.of("fast")), Parameter("ids", ParameterValue.of([1, -2]))])
    describe = DescribeParametersResponse([
        ParameterDescriptor("exposure_ms", 3, "exposure time", '{"unit": "ms"}',
                            floating_point_range=[FloatingPointRange(0.5, 33.0, 0.5)]),
        ParameterDescriptor("gain", 2, read_only=True, integer_range=[IntegerRange(0, 255, 1)])])
    results = SetParametersResponse([SetParametersResult(True, ""), SetParametersResult(False, "read only")])
    source = (Path(__file__).parent.parent / "cpp" / "tests" / "test_parameters.cpp").read_text()
    for name, message in (("kParameterEvent", event), ("kDescribe", describe), ("kSetResults", results)):
        constant = re.search(rf'const char\* {name} = "([0-9a-f]+)";', source).group(1)
        assert constant == bytes(message.serialize()).hex(), name
