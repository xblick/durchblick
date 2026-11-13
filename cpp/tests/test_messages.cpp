// Tests of the C++ messages and codecs. The expected bytes were written by the Python implementation (msg.py), which
// tests/test_cdr.py compares byte for byte with rosbags (an independent ROS 2 CDR implementation), so the C++ and the
// Python side and ROS agree. tests/test_cdr.py::test_cpp_golden_bytes checks that these constants still are Python's
// bytes; update them from it when a message changes.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "durchblick/camera.hpp"
#include "durchblick/image.hpp"
#include "durchblick/msg.hpp"

namespace
{
    int failures = 0;

    void expect(bool condition, const char* what)
    {
        if (not condition)
        {
            std::fprintf(stderr, "FAILED: %s\n", what);
            ++failures;
        }
    }

    std::string hex(const std::vector<std::uint8_t>& bytes)
    {
        static const char* digits = "0123456789abcdef";
        std::string out;
        for (auto b : bytes)
        {
            out += digits[b >> 4];
            out += digits[b & 0xF];
        }
        return out;
    }

    const char* kImage = "00010000010000000065cd1d0400000063616d0001000000020000000500000072676238000000000600000006000000010203040506";
    const char* kCameraInfo = "0001000003000000040000000400000063616d00e0010000800200000a000000706c756d625f626f620000000500000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000407f4000000000000000000000000000f8734000000000000000000000000000507f400000000000f06d4000000000000000000000000000000000000000000000f03f000000000000f03f000000000000000000000000000000000000000000000000000000000000f03f000000000000000000000000000000000000000000000000000000000000f03f0000000000407f4000000000000000000000000000f87340000000000000000000000000000000000000000000507f400000000000f06d40000000000000000000000000000000000000000000000000000000000000f03f000000000000000001000000000000000000000000000000000000000000000001";
    const char* kShm = "000100000500000006000000020000006600000002000000030000000600000031365543310000000600000002000000680000000e0000002f6475726368626c69636b2d78000000010000000000000000010000";
    const char* kCuda = "000100000000000000000000020000006700000004000000050000000500000072676238000000000f00000003000000683200000d0000006475726368626c69636b2d650000000021000000616261626162616261626162616261626162616261626162616261626162616200000000030000000800000000000000";
    const char* kTf = "0001000001000000090000000a0000000600000064657074680000000400000072676200fca9f1d24d62a0bffca9f1d24d62603ffca9f1d24d62703f9a9999999999b93f00000000000000000000000000000000d7a3703d0ad7ef3f";
    const char* kImu = "000100000b0000000c0000000f000000417a7572654b696e656374496d75000000000000000000000000000000000000000000000000000000000000000000000000f03f000000000000f0bf000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000007b14ae47e17a843f7b14ae47e17a94bfb81e85eb51b89e3f0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000009a9999999999b93f1f85eb51b89e23c09a9999999999c93f000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000";
    const char* kDiagnostics = "000100000d0000000e000000010000000000000001000000010000000d000000617a7572655f6b696e656374000000000a000000322064726f70706564000000070000003030303132330000020000000b00000063617074757265732f7300000500000033302e30000000000c00000074656d7065726174757265000500000033352e3500";
    const char* kCloud = "000100000f0000001000000006000000646570746800000001000000020000000400000002000000780000000000000007000000010000000200000079000000040000000700000001000000020000007a00000008000000070000000100000004000000726762000c000000070000000100000000000000100000002000000020000000000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f01";
    const char* kShmCloud = "0001000011000000120000000200000064000000020000000300000002000000020000007800000000000000070000000100000002000000690000000400000004000000020000000000000008000000180000000000000002000000680000000e0000002f6475726368626c69636b2d7900000002000000000000000000000002000000";
    const char* kCudaCloud = "000100000000000000000000020000006500000004000000050000000300000002000000780000000000000007000000010000000200000079000000040000000700000001000000020000007a0000000800000007000000010000000000000010000000500000000100000003000000683300000d0000006475726368626c69636b2d660000000021000000636463646364636463646364636463646364636463646364636463646364636400000000010000000900000000000000";
    const char* kDetections3D = "00010000130000001400000012000000417a7572654b696e65637443616d65726100000001000000130000001400000012000000417a7572654b696e65637443616d65726100000001000000040000006d756700000000000000e83f9a9999999999b93f9a9999999999c9bf000000000000f83f00000000000000000000000000000000333333333333e33f9a9999999999e93f7b14ae47e17a843f0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000e03f9a9999999999b93f9a9999999999c9bf000000000000f83f00000000000000000000000000000000333333333333e33f9a9999999999e93f7b14ae47e17ab43fb81e85eb51b8be3f9a9999999999b93f020000003700";
    const char* kDetections2D = "00010000150000001600000012000000417a7572654b696e65637443616d65726100000001000000150000001600000012000000417a7572654b696e65637443616d6572610000000100000006000000636861697200000000000000000000000000e03f000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000f03f00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000874400000000000086e40000000000000000000000000000044400000000000003e400100000000";
    // a controller's command (tests/test_cdr.py: the same message from Python; tools/gen_msgdefs.py checks Python's
    // bytes against moveit_msgs' definition)
    const char* kRobotState = "000100001f00000020000000010000000000000002000000100000006c6566745f73686f756c6465725f3100100000004c65667448616e6446696e6765727300020000009a9999999999b93f000000000000e03f00000000000000001f000000200000000a000000626173655f6c696e6b00000001000000090000006c6566745f74637000000000010000009a9999999999d93f9a9999999999c93f000000000000f03f00000000000000000000000000000000333333333333e33f9a9999999999e93f00000000000000000000000000";
    const char* kJointState = "000100001700000018000000010000000000000002000000090000006e65636b5f79617700000000100000006c6566745f73686f756c6465725f310002000000000000009a9999999999b93f9a9999999999c9bf0000000002000000000000000000f83f0000000000000000";
    const char* kOdometry = "00010000190000001a000000050000006f646f6d0000000005000000726f6f7400000000000000000000f03f00000000000000c0000000000000000000000000000000000000000000000000333333333333e33f9a9999999999e93f7b14ae47e17a843f00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000333333333333d33f9a9999999999b93f0000000000000000000000000000000000000000000000009a9999999999c9bf00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000e03f";
    const char* kWrench = "000100001b0000001c000000050000004654204c0000000000000000000000000000f83f00000000000000c01f85eb51b89e23409a9999999999b93f9a9999999999c93f333333333333d3bf";
    const char* kClassification = "000100001d0000001e00000012000000417a7572654b696e65637443616d65726100000002000000120000004f70656e696e6720736f6d657468696e6700000000000000000000000000e43f0100000000000000000000000000c03f";
    const char* kRecords = "0001000002000000070000007265636f7264000002000000060000000600000076616c75650000000300000003000000000000000600000000000000000000000000e03f000000000000f0bf0000000000000040fca9f1d24d62503f00000000000000000000000000001c40";
    // the controller manager's and the torque step's (tests/test_controller_manager_msgs.py writes them)
    const char* kListControllers = "00010000020000000500000061726d73000000000700000061637469766500000a000000496d706564616e6365000000e803000001000000170000006c6566745f73686f756c6465725f312f746f72717565000002000000170000006c6566745f73686f756c6465725f312f746f727175650000170000006c6566745f73686f756c6465725f332f746f727175650000000000000000000000000000000000000100000009000000757073747265616d000000000100000004000000612f62000a00000061726d735f7465737400000009000000696e616374697665000000000a000000496d706564616e6365000000e80300000000000002000000170000006c6566745f73686f756c6465725f312f746f727175650000170000006c6566745f73686f756c6465725f332f746f727175650000000000000000000000000000000000000100000009000000757073747265616d000000000100000004000000612f6200";
    const char* kSwitchController = "00010000010000000500000061726d7300000000020000000a00000061726d735f7465737400000002000000780000000200000001000000020000000065cd1d";
    const char* kTorqueStep = "000100000b0000006c6566745f656c626f770000000000000000f83f000000000000d03f";
    const char* kCreateController = "000100000500000061726d73000000000a000000496d706564616e63650000000b0000007b2261223a20312e357d00";
    const char* kConfigResponse = "0001000000000000080000006e6f2066696c65000100000000";
}  // namespace

int main()
{
    using namespace durchblick;
    try
    {
        // ------------------------------------------------------------------ the same bytes as Python
        const std::uint8_t pixels[] = {1, 2, 3, 4, 5, 6};
        Image image;
        image.header = {{1, 500000000}, "cam"};
        image.height = 1;
        image.width = 2;
        image.encoding = "rgb8";
        image.step = 6;
        image.data = pixels;
        expect(hex(image.serialize()) == kImage, "Image bytes");
        const auto image_bytes = image.serialize();
        const Image back = Image::deserialize(image_bytes);
        expect(back.header == image.header and back.encoding == "rgb8" and back.data.size() == 6 and
                   std::memcmp(back.data.data(), pixels, 6) == 0,
               "Image round trip");

        CameraInfo info = CameraInfo::pinhole(640, 480, 500.0, 501.0, 319.5, 239.5, "cam");
        info.header.stamp = {3, 4};
        info.binning_x = 1;
        info.roi.do_rectify = true;
        expect(hex(info.serialize()) == kCameraInfo, "CameraInfo bytes");
        expect(CameraInfo::deserialize(info.serialize()) == info, "CameraInfo round trip");

        ShmImage shm;
        shm.header = {{5, 6}, "f"};
        shm.height = 2;
        shm.width = 3;
        shm.encoding = "16UC1";
        shm.step = 6;
        shm.host = "h";
        shm.segment = "/durchblick-x";
        shm.slot = 1;
        shm.seq = 1ull << 40;
        expect(hex(shm.serialize()) == kShm, "ShmImage bytes");
        expect(ShmImage::deserialize(shm.serialize()).seq == shm.seq, "ShmImage round trip");

        CudaImage cuda;
        cuda.header.frame_id = "g";
        cuda.height = 4;
        cuda.width = 5;
        cuda.encoding = "rgb8";
        cuda.step = 15;
        cuda.host = "h2";
        cuda.endpoint = "durchblick-e";
        cuda.device_uuid = std::string(16 * 2, ' ');
        for (int i = 0; i < 16; ++i)
        {
            cuda.device_uuid[2 * i] = 'a';
            cuda.device_uuid[2 * i + 1] = 'b';
        }
        cuda.slot = 3;
        cuda.seq = 8;
        expect(hex(cuda.serialize()) == kCuda, "CudaImage bytes");

        TFMessage tf;
        tf.transforms.push_back({{{9, 10}, "depth"}, "rgb", {{-0.032, 0.002, 0.004}, {0.1, 0.0, 0.0, 0.995}}});
        expect(hex(tf.serialize()) == kTf, "TFMessage bytes");
        expect(TFMessage::deserialize(tf.serialize()) == tf, "TFMessage back");
        expect(TFMessage::deserialize(TFMessage{}.serialize()).transforms.empty(), "empty TFMessage back");

        Imu imu;
        imu.header = {{11, 12}, "AzureKinectImu"};
        imu.angular_velocity = {0.01, -0.02, 0.03};
        imu.linear_acceleration = {0.1, -9.81, 0.2};
        imu.orientation_covariance[0] = -1.0;  // no orientation
        expect(hex(imu.serialize()) == kImu, "Imu bytes");
        expect(Imu::deserialize(imu.serialize()) == imu, "Imu back");

        DiagnosticArray diagnostics;
        diagnostics.header = {{13, 14}, ""};
        diagnostics.status.push_back({DiagnosticStatus::kWarn, "azure_kinect", "2 dropped", "000123",
                                      {{"captures/s", "30.0"}, {"temperature", "35.5"}}});
        expect(hex(diagnostics.serialize()) == kDiagnostics, "DiagnosticArray bytes");
        expect(DiagnosticArray::deserialize(diagnostics.serialize()) == diagnostics, "DiagnosticArray back");

        const std::vector<PointField> xyzrgb = {{"x", 0, PointField::kFloat32, 1},
                                                {"y", 4, PointField::kFloat32, 1},
                                                {"z", 8, PointField::kFloat32, 1},
                                                {"rgb", 12, PointField::kFloat32, 1}};
        std::vector<std::uint8_t> points(32);
        for (std::size_t i = 0; i < points.size(); ++i)
        {
            points[i] = static_cast<std::uint8_t>(i);
        }
        PointCloud2 cloud;
        cloud.header = {{15, 16}, "depth"};
        cloud.height = 1;
        cloud.width = 2;
        cloud.fields = xyzrgb;
        cloud.point_step = 16;
        cloud.row_step = 32;
        cloud.data = points;
        cloud.is_dense = true;
        expect(hex(cloud.serialize()) == kCloud, "PointCloud2 bytes");
        const std::vector<std::uint8_t> cloud_bytes = cloud.serialize();
        const PointCloud2 cloud_back = PointCloud2::deserialize(cloud_bytes);
        expect(cloud_back.fields == xyzrgb and cloud_back.is_dense and cloud_back.row_step == 32 and
                   std::equal(cloud_back.data.begin(), cloud_back.data.end(), points.begin(), points.end()),
               "PointCloud2 back");

        ShmPointCloud2 shm_cloud;
        shm_cloud.header = {{17, 18}, "d"};
        shm_cloud.height = 2;
        shm_cloud.width = 3;
        shm_cloud.fields = {{"x", 0, PointField::kFloat32, 1}, {"i", 4, PointField::kUint16, 2}};
        shm_cloud.point_step = 8;
        shm_cloud.row_step = 24;
        shm_cloud.host = "h";
        shm_cloud.segment = "/durchblick-y";
        shm_cloud.slot = 2;
        shm_cloud.seq = 1ULL << 33;
        expect(hex(shm_cloud.serialize()) == kShmCloud, "ShmPointCloud2 bytes");
        const ShmPointCloud2 shm_back = ShmPointCloud2::deserialize(shm_cloud.serialize());
        expect(shm_back.fields == shm_cloud.fields and shm_back.seq == shm_cloud.seq and
                   shm_back.segment == shm_cloud.segment and not shm_back.is_dense,
               "ShmPointCloud2 back");

        CudaPointCloud2 cuda_cloud;
        cuda_cloud.header.frame_id = "e";
        cuda_cloud.height = 4;
        cuda_cloud.width = 5;
        cuda_cloud.fields = {xyzrgb.begin(), xyzrgb.begin() + 3};
        cuda_cloud.point_step = 16;
        cuda_cloud.row_step = 80;
        cuda_cloud.is_dense = true;
        cuda_cloud.host = "h3";
        cuda_cloud.endpoint = "durchblick-f";
        for (int i = 0; i < 16; ++i)
        {
            cuda_cloud.device_uuid += "cd";
        }
        cuda_cloud.slot = 1;
        cuda_cloud.seq = 9;
        expect(hex(cuda_cloud.serialize()) == kCudaCloud, "CudaPointCloud2 bytes");
        const CudaPointCloud2 cuda_back = CudaPointCloud2::deserialize(cuda_cloud.serialize());
        expect(cuda_back.endpoint == cuda_cloud.endpoint and cuda_back.device_uuid == cuda_cloud.device_uuid and
                   cuda_back.width == 5,
               "CudaPointCloud2 back");

        // vision_msgs (tests/test_cdr.py: the same messages from Python; tests/test_vision_msgs.py: Python against ROS)
        ObjectHypothesisWithPose hypothesis;
        hypothesis.hypothesis = {"mug", 0.75};
        hypothesis.pose.pose = {{0.1, -0.2, 1.5}, {0.0, 0.0, 0.6, 0.8}};
        hypothesis.pose.covariance[0] = 0.01;
        hypothesis.pose.covariance[35] = 0.5;
        Detection3DArray boxes;
        boxes.header = {{19, 20}, "AzureKinectCamera"};
        boxes.detections.push_back({.header = {{19, 20}, "AzureKinectCamera"},
                                    .results = {hypothesis},
                                    .bbox = {{{0.1, -0.2, 1.5}, {0.0, 0.0, 0.6, 0.8}}, {0.08, 0.12, 0.1}},
                                    .id = "7"});
        expect(hex(boxes.serialize()) == kDetections3D, "Detection3DArray bytes");
        expect(Detection3DArray::deserialize(boxes.serialize()) == boxes, "Detection3DArray back");
        Detection2DArray boxes2d;
        boxes2d.header = {{21, 22}, "AzureKinectCamera"};
        ObjectHypothesisWithPose chair;
        chair.hypothesis = {"chair", 0.5};
        boxes2d.detections.push_back({.header = {{21, 22}, "AzureKinectCamera"},
                                      .results = {chair},
                                      .bbox = {{{320.5, 240.25}, 0.0}, 40.0, 30.0},
                                      .id = ""});
        expect(hex(boxes2d.serialize()) == kDetections2D, "Detection2DArray bytes");
        expect(Detection2DArray::deserialize(boxes2d.serialize()) == boxes2d, "Detection2DArray back");
        Classification actions;
        actions.header = {{29, 30}, "AzureKinectCamera"};
        actions.results = {{"Opening something", 0.625}, {"", 0.125}};
        expect(hex(actions.serialize()) == kClassification, "Classification bytes");
        expect(Classification::deserialize(actions.serialize()) == actions, "Classification back");

        // a robot's state (tests/test_cdr.py: the same messages from Python; tests/test_robot_state.py: against ROS)
        JointState joints;
        joints.header = {{23, 24}, ""};
        joints.name = {"neck_yaw", "left_shoulder_1"};
        joints.position = {0.1, -0.2};
        joints.effort = {1.5, 0.0};  // no velocity: an empty sequence
        expect(hex(joints.serialize()) == kJointState, "JointState bytes");
        expect(JointState::deserialize(joints.serialize()) == joints, "JointState back");
        bool lengths_refused = false;
        try
        {
            JointState wrong = joints;
            wrong.velocity = {0.0};
            wrong.serialize();
        }
        catch (const std::invalid_argument&)
        {
            lengths_refused = true;
        }
        expect(lengths_refused, "a JointState with 1 velocity for 2 joints is refused");

        RobotState command;
        command.joint_state.header = {{31, 32}, ""};
        command.joint_state.name = {"left_shoulder_1", "LeftHandFingers"};
        command.joint_state.position = {0.1, 0.5};
        command.multi_dof_joint_state.header = {{31, 32}, "base_link"};
        command.multi_dof_joint_state.joint_names = {"left_tcp"};
        command.multi_dof_joint_state.transforms = {{{0.4, 0.2, 1.0}, {0.0, 0.0, 0.6, 0.8}}};
        expect(hex(command.serialize()) == kRobotState, "RobotState bytes");
        expect(RobotState::deserialize(command.serialize()) == command, "RobotState back");
        bool unmatched_refused = false;
        try
        {
            RobotState wrong = command;
            wrong.multi_dof_joint_state.joint_names.push_back("right_tcp");  // two frames, one transform
            (void)wrong.serialize();
        }
        catch (const std::invalid_argument&)
        {
            unmatched_refused = true;
        }
        expect(unmatched_refused, "a MultiDOFJointState with 2 frames and 1 transform is refused");
        Odometry odom;
        odom.header = {{25, 26}, "odom"};
        odom.child_frame_id = "root";
        odom.pose.pose = {{1.0, -2.0, 0.0}, {0.0, 0.0, 0.6, 0.8}};
        odom.pose.covariance[0] = 0.01;
        odom.twist.twist = {{0.3, 0.1, 0.0}, {0.0, 0.0, -0.2}};
        odom.twist.covariance[35] = 0.5;
        expect(hex(odom.serialize()) == kOdometry, "Odometry bytes");
        expect(Odometry::deserialize(odom.serialize()) == odom, "Odometry back");
        WrenchStamped wrench;
        wrench.header = {{27, 28}, "FT L"};
        wrench.wrench = {{1.5, -2.0, 9.81}, {0.1, 0.2, -0.3}};
        expect(hex(wrench.serialize()) == kWrench, "WrenchStamped bytes");
        expect(WrenchStamped::deserialize(wrench.serialize()) == wrench, "WrenchStamped back");
        Float64MultiArray records;
        records.layout.dim = {{"record", 2, 6}, {"value", 3, 3}};
        records.data = {0.5, -1.0, 2.0, 1e-3, 0.0, 7.0};
        expect(hex(records.serialize()) == kRecords, "Float64MultiArray bytes");
        expect(Float64MultiArray::deserialize(records.serialize()) == records, "Float64MultiArray back");

        // the controller manager (controller_manager_msgs, durchblick_msgs) and the torque step (ros2_cdr_msgs.hpp)
        const auto controller_state = [](const std::string& name, bool active)
        {
            ControllerState s;
            s.name = name;
            s.state = active ? "active" : "inactive";
            s.type = "Impedance";
            s.update_rate = 1000;
            if (active)
            {
                s.claimed_interfaces = {"left_shoulder_1/torque"};
            }
            s.required_command_interfaces = {"left_shoulder_1/torque", "left_shoulder_3/torque"};
            s.chain_connections = {{"upstream", {"a/b"}}};
            return s;
        };
        ListControllersResponse controllers;
        controllers.controller = {controller_state("arms", true), controller_state("arms_test", false)};
        expect(hex(controllers.serialize()) == kListControllers, "ListControllers_Response bytes");
        expect(ListControllersResponse::deserialize(controllers.serialize()).controller == controllers.controller,
               "ListControllers_Response back");
        SwitchControllerRequest switching;
        switching.activate_controllers = {"arms"};
        switching.deactivate_controllers = {"arms_test", "x"};
        switching.strictness = SwitchControllerRequest::kStrict;
        switching.activate_asap = true;
        switching.timeout = {2, 500000000};
        expect(hex(switching.serialize()) == kSwitchController, "SwitchController_Request bytes");
        const SwitchControllerRequest switched = SwitchControllerRequest::deserialize(switching.serialize());
        expect(switched.deactivate_controllers == switching.deactivate_controllers and
                   switched.timeout == switching.timeout and switched.activate_asap,
               "SwitchController_Request back");
        const TorqueStepRequest step{"left_elbow", 1.5, 0.25};
        expect(hex(step.serialize()) == kTorqueStep, "TorqueStep_Request bytes");
        expect(TorqueStepRequest::deserialize(step.serialize()).joint == "left_elbow", "TorqueStep_Request back");
        const CreateControllerRequest create{"arms", "Impedance", "{\"a\": 1.5}"};
        expect(hex(create.serialize()) == kCreateController, "CreateController_Request bytes");
        const ConfigResponse no_file{false, "no file", ""};
        expect(hex(no_file.serialize()) == kConfigResponse, "GetConfigFile_Response bytes");
        expect(ListControllersRequest::deserialize(ListControllersRequest{}.serialize()).serialize().size() == 5,
               "ListControllers_Request: the one placeholder byte");
        bool truncated = false;
        try
        {
            const std::vector<std::uint8_t> bytes = boxes.serialize();
            Detection3DArray::deserialize(std::span(bytes).first(bytes.size() - 5));
        }
        catch (const CdrError&)
        {
            truncated = true;
        }
        expect(truncated, "a truncated Detection3DArray is refused");

        // rotation matrices to quaternions, through each branch: identity, 90 deg about x, 180 deg about x, y, z
        const auto near = [](const Quaternion& a, const Quaternion& b)
        {
            return std::abs(a.x - b.x) < 1e-12 and std::abs(a.y - b.y) < 1e-12 and std::abs(a.z - b.z) < 1e-12 and
                   std::abs(a.w - b.w) < 1e-12;
        };
        const double h = std::sqrt(0.5);
        expect(near(Transform::from_matrix({1, 0, 0, 0, 1, 0, 0, 0, 1}, {}).rotation, {0, 0, 0, 1}), "identity");
        expect(near(Transform::from_matrix({1, 0, 0, 0, 0, -1, 0, 1, 0}, {}).rotation, {h, 0, 0, h}), "90 deg about x");
        expect(near(Transform::from_matrix({1, 0, 0, 0, -1, 0, 0, 0, -1}, {}).rotation, {1, 0, 0, 0}), "180 about x");
        expect(near(Transform::from_matrix({-1, 0, 0, 0, 1, 0, 0, 0, -1}, {}).rotation, {0, 1, 0, 0}), "180 about y");
        expect(near(Transform::from_matrix({-1, 0, 0, 0, -1, 0, 0, 0, 1}, {}).rotation, {0, 0, 1, 0}), "180 about z");
        expect(Transform::from_matrix({1, 0, 0, 0, 1, 0, 0, 0, 1}, {1, 2, 3}).translation == Vector3{1, 2, 3},
               "translation kept");

        // frames through tf_static: depth -> colour (90 deg about z, then 0.1 m along x), colour -> lens (0.01 m
        // along z)
        TFMessage chain;
        const std::array<double, 9> about_z = {0, -1, 0, 1, 0, 0, 0, 0, 1}, identity = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        chain.transforms.push_back({{{}, "depth"}, "colour", Transform::from_matrix(about_z, {0.1, 0, 0})});
        chain.transforms.push_back({{{}, "colour"}, "lens", Transform::from_matrix(identity, {0, 0, 0.01})});
        const auto apply = [](const RigidTransform& p, const Vector3& v) {
            const auto& r = p.rotation;
            return Vector3{r[0] * v.x + r[1] * v.y + r[2] * v.z + p.translation.x,
                           r[3] * v.x + r[4] * v.y + r[5] * v.z + p.translation.y,
                           r[6] * v.x + r[7] * v.y + r[8] * v.z + p.translation.z};
        };
        const auto close = [](const Vector3& a, const Vector3& b) {
            return std::abs(a.x - b.x) < 1e-12 and std::abs(a.y - b.y) < 1e-12 and std::abs(a.z - b.z) < 1e-12;
        };
        const auto lens_in_depth = lookup(chain, "depth", "lens");
        // the lens origin: (0, 0, 0.01) in colour, turned about z (z stays) and moved by 0.1 m along x
        expect(lens_in_depth and close(apply(*lens_in_depth, {0, 0, 0}), {0.1, 0, 0.01}), "lookup along a chain");
        // a point 1 m along the colour camera's x is 1 m along the depth camera's y (90 deg about z)
        expect(close(apply(*lens_in_depth, {1, 0, 0}), {0.1, 1, 0.01}), "lookup turns");
        const auto depth_in_lens = lookup(chain, "lens", "depth");
        expect(depth_in_lens and close(apply(*depth_in_lens, {0.1, 1, 0.01}), {1, 0, 0}), "lookup inverts");
        expect(lookup(chain, "depth", "depth") and close(apply(*lookup(chain, "depth", "depth"), {1, 2, 3}), {1, 2, 3}),
               "lookup of a frame in itself");
        expect(not lookup(chain, "depth", "elsewhere"), "frames that do not connect");

        bool refused = false;
        try
        {
            std::vector<std::uint8_t> truncated = image_bytes;
            truncated.resize(truncated.size() - 3);
            Image::deserialize(truncated);
        }
        catch (const CdrError&)
        {
            refused = true;
        }
        expect(refused, "a truncated message is refused");

        // ------------------------------------------------------------------ codecs
        std::vector<std::uint16_t> depth(48 * 64);
        for (std::size_t i = 0; i < depth.size(); ++i)
        {
            depth[i] = static_cast<std::uint16_t>(i * 37);
        }
        const CompressedImage png = compress(HostImage{depth.data(), 48, 64, 0}, "16UC1", kPng, 85, {});
        expect(png.format == "16UC1; png compressed 16UC1", "PNG format string");
        const DecodedImage depth_back = decompress(png);
        expect(depth_back.encoding == "16UC1" and depth_back.width == 64 and depth_back.height == 48 and
                   std::memcmp(depth_back.pixels.data(), depth.data(), depth.size() * 2) == 0,
               "16-bit PNG is lossless");

        // RVL: ROS's compressedDepth rvl. 0 0 5 5 0 are the groups 2 (zeros), 2 (others), A 1 (+5), 0 (+0), 1 (zero),
        // 0 (others): the word 0x22A10100 after the header's 12 zero bytes, width and height.
        const std::uint16_t five[5] = {0, 0, 5, 5, 0};
        const CompressedImage rvl_five = compress(HostImage{five, 1, 5, 0}, "16UC1", kRvl, 85, {});
        const std::vector<std::uint8_t> rvl_expected = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 0, 0, 0,
                                                        1, 0, 0, 0, 0x00, 0x01, 0xA1, 0x22};
        expect(rvl_five.format == "16UC1; compressedDepth rvl" and rvl_five.data == rvl_expected,
               "RVL writes ROS's compressedDepth rvl");
        for (std::size_t i = 0; i < depth.size(); ++i)  // holes, steps of every size, the largest jump
        {
            depth[i] = i % 7 < 2 ? 0 : static_cast<std::uint16_t>(i % 11 == 0 ? 65535 : i * 37);
        }
        const CompressedImage rvl = compress(HostImage{depth.data(), 48, 64, 0}, "16UC1", kRvl, 85, {});
        const DecodedImage rvl_back = decompress(rvl);
        expect(rvl_back.encoding == "16UC1" and rvl_back.width == 64 and rvl_back.height == 48 and
                   std::memcmp(rvl_back.pixels.data(), depth.data(), depth.size() * 2) == 0,
               "RVL is lossless");
        std::vector<std::uint16_t> padded(48 * 70, 9);  // rows of 64 pixels with 6 more each
        for (std::size_t y = 0; y < 48; ++y)
        {
            std::memcpy(padded.data() + y * 70, depth.data() + y * 64, 64 * 2);
        }
        expect(compress(HostImage{padded.data(), 48, 64, 140}, "16UC1", kRvl, 85, {}).data == rvl.data,
               "RVL leaves the rows' padding out");
        const auto refused_rvl = [](CompressedImage message, const char* what) {
            try
            {
                decompress(message);
            }
            catch (const EncodingError&)
            {
                return;
            }
            expect(false, what);
        };
        CompressedImage cut = rvl;
        cut.data.resize(cut.data.size() / 2);
        refused_rvl(cut, "RVL bytes that end early are refused");
        CompressedImage larger = rvl;
        larger.data[16] = 49;  // one row more than the bytes hold
        refused_rvl(larger, "an RVL image larger than its bytes is refused");
        CompressedImage huge = rvl;
        huge.data[15] = huge.data[19] = 0x7f;
        refused_rvl(huge, "an RVL header with an impossible size is refused");
        CompressedImage colour = rvl;
        colour.format = "rgb8; compressedDepth rvl";
        refused_rvl(colour, "RVL of another encoding than 16-bit grey is refused");
        CompressedImage ros_png = rvl;
        ros_png.format = "16UC1; compressedDepth png";
        refused_rvl(ros_png, "ROS's compressedDepth png is refused");
        try
        {
            const std::uint8_t pixel[3] = {1, 2, 3};
            compress(HostImage{pixel, 1, 1, 0}, "rgb8", kRvl, 85, {});
            expect(false, "RVL of a colour image is refused");
        }
        catch (const EncodingError&)
        {
        }

        std::vector<std::uint8_t> bgr(32 * 32 * 3);
        for (std::size_t i = 0; i < bgr.size(); i += 3)
        {
            bgr[i] = 200;  // blue
            bgr[i + 1] = 100;
            bgr[i + 2] = 10;
        }
        const CompressedImage jpeg = compress(HostImage{bgr.data(), 32, 32, 0}, "bgr8", kJpeg, 95, {});
        expect(jpeg.format == "bgr8; jpeg compressed bgr8", "JPEG format string");
        const DecodedImage bgr_back = decompress(jpeg);
        expect(bgr_back.encoding == "bgr8" and std::abs(int(bgr_back.pixels[0]) - 200) < 4 and
                   std::abs(int(bgr_back.pixels[2]) - 10) < 4,
               "JPEG keeps the colour order");

        // a camera's JPEG as it is (MJPEG): the bytes unchanged, the format compress() would write
        const CompressedImage passed = jpeg_message(jpeg.data, "rgb8", {{1, 2}, "cam"});
        expect(passed.data == jpeg.data and passed.format == "rgb8; jpeg compressed bgr8" and
                   passed.header.frame_id == "cam",
               "a JPEG frame passes unchanged");
        bool not_jpeg = false, not_8bit = false;
        try
        {
            jpeg_message(png.data, "rgb8", {});
        }
        catch (const EncodingError&)
        {
            not_jpeg = true;
        }
        try
        {
            jpeg_message(jpeg.data, "16UC1", {});
        }
        catch (const EncodingError&)
        {
            not_8bit = true;
        }
        expect(not_jpeg and not_8bit, "jpeg_message refuses a PNG and a 16-bit encoding");

        // ------------------------------------------------------------------ calibration file
        // tests/test_camera_info_file.py's ROS_FILE (checked there against ruamel.yaml)
        const std::string ros_file = "image_width: 1280\n"
                                     "image_height: 720\n"
                                     "camera_name: 'azurekinect'\n"
                                     "camera_matrix:\n"
                                     "  rows: 3\n"
                                     "  cols: 3\n"
                                     "  data: [612.3, 0, 638.7, 0, 612.1, 366.2, 0, 0, 1]\n"
                                     "distortion_model: rational_polynomial\n"
                                     "distortion_coefficients:\n"
                                     "  rows: 1\n"
                                     "  cols: 8\n"
                                     "  data: [0.4519, -2.6871, 0.00041,\n"
                                     "         -0.00012, 1.5383, 0.3321,\n"
                                     "         -2.5038, 1.4682]  # k1 k2 p1 p2 k3 k4 k5 k6\n"
                                     "rectification_matrix:\n"
                                     "  rows: 3\n"
                                     "  cols: 3\n"
                                     "  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]\n"
                                     "projection_matrix:\n"
                                     "  rows: 3\n"
                                     "  cols: 4\n"
                                     "  data: [600.5, 0, 640.25, 0, 0, 601.75, 360.5, 0, 0, 0, 1, 0]\n";
        CameraInfo file_info = CameraInfo::from_yaml(ros_file);
        expect(file_info.width == 1280 and file_info.height == 720 and
                   file_info.distortion_model == "rational_polynomial" and file_info.header.frame_id.empty(),
               "calibration file: size and model");
        expect(file_info.k == std::array<double, 9>{612.3, 0, 638.7, 0, 612.1, 366.2, 0, 0, 1} and
                   file_info.d == std::vector<double>{0.4519, -2.6871, 0.00041, -0.00012, 1.5383, 0.3321, -2.5038,
                                                      1.4682} and
                   file_info.r == std::array<double, 9>{1, 0, 0, 0, 1, 0, 0, 0, 1} and
                   file_info.p == std::array<double, 12>{600.5, 0, 640.25, 0, 0, 601.75, 360.5, 0, 0, 0, 1, 0},
               "calibration file: K, D, R, P");
        file_info.k[0] = 1.0 / 3.0;
        file_info.d[4] = -1e-17;
        expect(CameraInfo::from_yaml(file_info.to_yaml("azurekinect")) == file_info,
               "calibration file: written and read back exactly");
        const auto file_refused = [&](const std::string& from, const std::string& to, const std::string& message)
        {
            std::string text = ros_file;
            text.replace(text.find(from), from.size(), to);
            try
            {
                CameraInfo::from_yaml(text);
            }
            catch (const std::invalid_argument& e)
            {
                return std::string(e.what()).find(message) != std::string::npos;
            }
            return false;
        };
        expect(file_refused("image_height: 720\n", "", "'image_height' missing") and
                   file_refused("  cols: 4\n", "  cols: 3\n", "projection_matrix: 3 x 3") and
                   file_refused("  cols: 8\n", "  cols: 7\n", "distortion_coefficients: 1 x 7 with 8 values") and
                   file_refused("camera_name", "camera_names", "unknown key 'camera_names'") and
                   file_refused("1.4682]", "1.4682] 2", "text after ']'") and
                   file_refused("1.4682]", "1.4682", "list without ']'") and
                   file_refused("612.3", "6x12", "'6x12' is not a number") and
                   file_refused("612.3", "6_12", "'6_12' is not a number") and
                   file_refused("  rows: 1\n", "  rows: 1\n  rows: 1\n", "'rows' unknown or twice"),
               "calibration file: anything else is refused");
    }
    catch (const std::exception& e)
    {
        std::fprintf(stderr, "FAILED: %s\n", e.what());
        return 1;
    }
    std::fprintf(stderr, failures ? "%d failures\n" : "ok\n", failures);
    return failures ? 1 : 0;
}
