// The messages of Durchblick in C++, field for field as in Python's msg.py. Those a robot's controller speaks too
// (builtin_interfaces/Time, std_msgs/Header, tf2_msgs/TFMessage with geometry_msgs' Vector3, Quaternion, Transform,
// TransformStamped, diagnostic_msgs/DiagnosticArray, sensor_msgs/JointState, geometry_msgs/WrenchStamped,
// moveit_msgs/RobotState, std_msgs/Float64MultiArray, rcl_interfaces' parameters) and the CDR writer and reader are
// in ros2_cdr_msgs.hpp, a file shared with the controller's repository, and are named here as durchblick's. This file
// has the others: sensor_msgs/Image, CompressedImage, RegionOfInterest, CameraInfo, Imu, PointCloud2 (with
// PointField), vision_msgs/Detection2DArray, Detection3DArray and Classification (with their parts and
// geometry_msgs' Point, Pose, PoseWithCovariance), nav_msgs/Odometry, and Durchblick's ring descriptors ShmImage,
// CudaImage, ShmPointCloud2 and CudaPointCloud2. serialize() writes CDR as ROS 2 does (XCDR1 little endian, 4-byte
// encapsulation header, alignment counted after it); deserialize() reads it and refuses anything else with CdrError.
#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include "durchblick/ros2_cdr_msgs.hpp"

namespace durchblick
{
    // ------------------------------------------------------------------- shared with the controller (ros2_cdr)

    using ros2_cdr::CdrError;
    using ros2_cdr::CdrReader;
    using ros2_cdr::CdrWriter;

    using ros2_cdr::Header;
    using ros2_cdr::Time;

    using ros2_cdr::Quaternion;
    using ros2_cdr::TFMessage;
    using ros2_cdr::Transform;
    using ros2_cdr::TransformStamped;
    using ros2_cdr::Vector3;

    using ros2_cdr::DiagnosticArray;
    using ros2_cdr::DiagnosticStatus;
    using ros2_cdr::KeyValue;

    using ros2_cdr::JointState;
    using ros2_cdr::MultiDOFJointState;
    using ros2_cdr::RobotState;
    using ros2_cdr::Twist;
    using ros2_cdr::Wrench;
    using ros2_cdr::WrenchStamped;

    using ros2_cdr::Float64MultiArray;
    using ros2_cdr::MultiArrayDimension;
    using ros2_cdr::MultiArrayLayout;

    namespace parameter_type = ros2_cdr::parameter_type;
    using ros2_cdr::DescribeParametersResponse;
    using ros2_cdr::FloatingPointRange;
    using ros2_cdr::GetParametersResponse;
    using ros2_cdr::GetParameterTypesResponse;
    using ros2_cdr::IntegerRange;
    using ros2_cdr::ListParametersRequest;
    using ros2_cdr::ListParametersResponse;
    using ros2_cdr::NamesRequest;
    using ros2_cdr::Parameter;
    using ros2_cdr::ParameterDescriptor;
    using ros2_cdr::ParameterEvent;
    using ros2_cdr::ParametersRequest;
    using ros2_cdr::ParameterValue;
    using ros2_cdr::SetParametersAtomicallyResponse;
    using ros2_cdr::SetParametersResponse;
    using ros2_cdr::SetParametersResult;

    using ros2_cdr::TorqueStepRequest;
    using ros2_cdr::TorqueStepResponse;

    // ------------------------------------------------------------------------------------------- images

    // sensor_msgs/msg/Image. data is a view: serialize() copies from it, deserialize() points into the payload.
    struct Image
    {
        Header header;
        std::uint32_t height = 0;
        std::uint32_t width = 0;
        std::string encoding;
        std::uint8_t is_bigendian = 0;
        std::uint32_t step = 0;
        std::span<const std::uint8_t> data;

        std::vector<std::uint8_t> serialize() const;
        static Image deserialize(std::span<const std::uint8_t> payload);
    };

    struct CompressedImage
    {
        Header header;
        std::string format;
        std::vector<std::uint8_t> data;

        std::vector<std::uint8_t> serialize() const;
        static CompressedImage deserialize(std::span<const std::uint8_t> payload);
    };

    struct RegionOfInterest
    {
        std::uint32_t x_offset = 0;
        std::uint32_t y_offset = 0;
        std::uint32_t height = 0;
        std::uint32_t width = 0;
        bool do_rectify = false;
        bool operator==(const RegionOfInterest&) const = default;
    };

    struct CameraInfo
    {
        Header header;
        std::uint32_t height = 0;
        std::uint32_t width = 0;
        std::string distortion_model;
        std::vector<double> d;
        std::array<double, 9> k{};
        std::array<double, 9> r{};
        std::array<double, 12> p{};
        std::uint32_t binning_x = 0;
        std::uint32_t binning_y = 0;
        RegionOfInterest roi;

        // An ideal camera without distortion: plumb_bob with zero coefficients, R = identity, P = [K | 0].
        static CameraInfo pinhole(std::uint32_t width, std::uint32_t height, double fx, double fy, double cx, double cy,
                                  const std::string& frame_id = "");
        // ROS's calibration file (camera_info_file.cpp): the YAML of camera_calibration_parsers, as ROS's
        // camera_calibration writes it and a ROS driver's camera_info_url names. It has size, K, D with its model, R
        // and P; the header (frame) stays empty, binning and ROI zero. Anything else than that format, a matrix of the
        // wrong size or a missing key throws std::invalid_argument; read_file says which file.
        static CameraInfo from_yaml(const std::string& text);
        static CameraInfo read_file(const std::string& path);
        std::string to_yaml(const std::string& camera_name) const;
        void write_file(const std::string& path, const std::string& camera_name) const;
        std::vector<std::uint8_t> serialize() const;
        static CameraInfo deserialize(std::span<const std::uint8_t> payload);
        bool operator==(const CameraInfo&) const = default;
    };

    // sensor_msgs/msg/Imu: one sample (protocol.hpp: <camera>/imu). As in ROS, orientation_covariance[0] = -1 says
    // that there is no orientation; covariances of zero say that they are unknown.
    struct Imu
    {
        Header header;
        Quaternion orientation;
        std::array<double, 9> orientation_covariance{};
        Vector3 angular_velocity;  // rad/s
        std::array<double, 9> angular_velocity_covariance{};
        Vector3 linear_acceleration;  // m/s^2
        std::array<double, 9> linear_acceleration_covariance{};

        std::vector<std::uint8_t> serialize() const;
        static Imu deserialize(std::span<const std::uint8_t> payload);
        bool operator==(const Imu&) const = default;
    };

    // Fields of sensor_msgs/Image without data, then where the pixels are (protocol.hpp, ring.hpp).
    struct RingImage
    {
        Header header;
        std::uint32_t height = 0;
        std::uint32_t width = 0;
        std::string encoding;
        std::uint8_t is_bigendian = 0;
        std::uint32_t step = 0;
        std::string host;  // boot id of the publisher's host
        std::uint32_t slot = 0;
        std::uint64_t seq = 0;
    };

    struct ShmImage : RingImage
    {
        std::string segment;  // POSIX shared memory name of the host ring

        std::vector<std::uint8_t> serialize() const;
        static ShmImage deserialize(std::span<const std::uint8_t> payload);
    };

    struct CudaImage : RingImage
    {
        std::string endpoint;     // abstract Unix socket of the CUDA ring
        std::string device_uuid;  // 32 hex digits

        std::vector<std::uint8_t> serialize() const;
        static CudaImage deserialize(std::span<const std::uint8_t> payload);
    };

    // sensor_msgs/msg/PointField: one field of a point (name, byte offset in the point, type, number of elements).
    struct PointField
    {
        static constexpr std::uint8_t kInt8 = 1, kUint8 = 2, kInt16 = 3, kUint16 = 4, kInt32 = 5, kUint32 = 6,
                                      kFloat32 = 7, kFloat64 = 8;
        std::string name;
        std::uint32_t offset = 0;
        std::uint8_t datatype = 0;
        std::uint32_t count = 1;
        bool operator==(const PointField&) const = default;
    };

    // Bytes of one element of a PointField datatype; throws for an unknown one.
    std::uint32_t point_field_size(std::uint8_t datatype);

    // sensor_msgs/msg/PointCloud2. An organized cloud has height > 1 (row y, column x as in the image it came from);
    // is_dense is false if it holds invalid points (NaN coordinates). data is a view as Image's.
    struct PointCloud2
    {
        Header header;
        std::uint32_t height = 0;
        std::uint32_t width = 0;
        std::vector<PointField> fields;
        bool is_bigendian = false;
        std::uint32_t point_step = 0;
        std::uint32_t row_step = 0;
        std::span<const std::uint8_t> data;
        bool is_dense = false;

        std::vector<std::uint8_t> serialize() const;
        static PointCloud2 deserialize(std::span<const std::uint8_t> payload);
    };

    // Fields of sensor_msgs/PointCloud2 without data, then where the points are (protocol.hpp, ring.hpp).
    struct RingPointCloud2
    {
        Header header;
        std::uint32_t height = 0;
        std::uint32_t width = 0;
        std::vector<PointField> fields;
        bool is_bigendian = false;
        std::uint32_t point_step = 0;
        std::uint32_t row_step = 0;
        bool is_dense = false;
        std::string host;  // boot id of the publisher's host
        std::uint32_t slot = 0;
        std::uint64_t seq = 0;
    };

    struct ShmPointCloud2 : RingPointCloud2
    {
        std::string segment;  // POSIX shared memory name of the host ring

        std::vector<std::uint8_t> serialize() const;
        static ShmPointCloud2 deserialize(std::span<const std::uint8_t> payload);
    };

    struct CudaPointCloud2 : RingPointCloud2
    {
        std::string endpoint;     // abstract Unix socket of the CUDA ring
        std::string device_uuid;  // 32 hex digits

        std::vector<std::uint8_t> serialize() const;
        static CudaPointCloud2 deserialize(std::span<const std::uint8_t> payload);
    };

    // durchblick_msgs/msg/CompressedPointCloud2: a cloud whose points lie on fixed rays (points.hpp). A point is its
    // ray's direction times its range. `rays` names the table of directions (Rays, answered on cloud/rays). `range`:
    // a 16-bit image of the ranges in steps of `range_step` metres, 0 for no point, as a CompressedImage's format and
    // data (RVL). `color`: the points' colours as an rgb8 image the same way; empty without the field rgb.
    struct CompressedPointCloud2
    {
        Header header;
        std::uint32_t height = 0;
        std::uint32_t width = 0;
        std::vector<PointField> fields;
        std::uint32_t point_step = 0;
        bool is_dense = false;
        std::string rays;
        double range_step = 0.0;
        std::string range_format;
        std::span<const std::uint8_t> range;
        std::string color_format;
        std::span<const std::uint8_t> color;

        std::vector<std::uint8_t> serialize() const;
        static CompressedPointCloud2 deserialize(std::span<const std::uint8_t> payload);
    };

    // durchblick_msgs/msg/Rays: the directions of a CompressedPointCloud2's points, height * width * 3 float32
    // (little endian); `id` names the table.
    struct Rays
    {
        std::string id;
        std::uint32_t height = 0;
        std::uint32_t width = 0;
        std::span<const std::uint8_t> directions;

        std::vector<std::uint8_t> serialize() const;
        static Rays deserialize(std::span<const std::uint8_t> payload);
    };

    // ------------------------------------------------------------------------------------------- vision_msgs
    // Detections of vision nodes (protocol.hpp: <camera>/<node>/<name>): ROS's vision_msgs 4.x (humble, jazzy).

    struct Point  // geometry_msgs/msg/Point
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        bool operator==(const Point&) const = default;
    };

    struct Pose  // geometry_msgs/msg/Pose
    {
        Point position;
        Quaternion orientation;
        bool operator==(const Pose&) const = default;
    };

    struct PoseWithCovariance  // geometry_msgs/msg/PoseWithCovariance: covariance 6x6 row-major
    {
        Pose pose;
        std::array<double, 36> covariance{};
        bool operator==(const PoseWithCovariance&) const = default;
    };

    struct ObjectHypothesis  // vision_msgs/msg/ObjectHypothesis: a label (e.g. an open-vocabulary prompt) and its score
    {
        std::string class_id;
        double score = 0.0;
        bool operator==(const ObjectHypothesis&) const = default;
    };

    struct ObjectHypothesisWithPose
    {
        ObjectHypothesis hypothesis;
        PoseWithCovariance pose;
        bool operator==(const ObjectHypothesisWithPose&) const = default;
    };

    struct Point2D
    {
        double x = 0.0;
        double y = 0.0;
        bool operator==(const Point2D&) const = default;
    };

    struct Pose2D  // vision_msgs/msg/Pose2D: theta in radians
    {
        Point2D position;
        double theta = 0.0;
        bool operator==(const Pose2D&) const = default;
    };

    struct BoundingBox2D  // centre and size in pixels
    {
        Pose2D center;
        double size_x = 0.0;
        double size_y = 0.0;
        bool operator==(const BoundingBox2D&) const = default;
    };

    struct BoundingBox3D  // centre pose and size along its axes (metres)
    {
        Pose center;
        Vector3 size;
        bool operator==(const BoundingBox3D&) const = default;
    };

    struct Detection2D  // one object in an image; id: a tracking id (empty: none)
    {
        Header header;
        std::vector<ObjectHypothesisWithPose> results;
        BoundingBox2D bbox;
        std::string id;
        bool operator==(const Detection2D&) const = default;
    };

    struct Detection2DArray
    {
        Header header;
        std::vector<Detection2D> detections;

        std::vector<std::uint8_t> serialize() const;
        static Detection2DArray deserialize(std::span<const std::uint8_t> payload);
        bool operator==(const Detection2DArray&) const = default;
    };

    struct Detection3D  // one object in 3D; id: a tracking id (empty: none)
    {
        Header header;
        std::vector<ObjectHypothesisWithPose> results;
        BoundingBox3D bbox;
        std::string id;
        bool operator==(const Detection3D&) const = default;
    };

    struct Detection3DArray
    {
        Header header;  // the observation's stamp, the frame of the boxes' poses
        std::vector<Detection3D> detections;

        std::vector<std::uint8_t> serialize() const;
        static Detection3DArray deserialize(std::span<const std::uint8_t> payload);
        bool operator==(const Detection3DArray&) const = default;
    };

    struct Classification  // classes of the whole input, no place (e.g. actions in a clip, stamped as its newest image)
    {
        Header header;
        std::vector<ObjectHypothesis> results;

        std::vector<std::uint8_t> serialize() const;
        static Classification deserialize(std::span<const std::uint8_t> payload);
        bool operator==(const Classification&) const = default;
    };

    // ------------------------------------------------------------------------------------------- robot state
    // A robot's state below its namespace (protocol.hpp): sensor_msgs/JointState and geometry_msgs/WrenchStamped
    // (shared, above) and nav_msgs/Odometry.

    struct TwistWithCovariance  // covariance 6x6 row-major
    {
        Twist twist;
        std::array<double, 36> covariance{};
        bool operator==(const TwistWithCovariance&) const = default;
    };

    // The pose of child_frame_id (the robot's root) in header.frame_id (odom) and its velocity in child_frame_id.
    struct Odometry
    {
        Header header;
        std::string child_frame_id;
        PoseWithCovariance pose;
        TwistWithCovariance twist;

        std::vector<std::uint8_t> serialize() const;
        static Odometry deserialize(std::span<const std::uint8_t> payload);
        bool operator==(const Odometry&) const = default;
    };

    // ------------------------------------------------------------------------------- the controller manager
    // A robot's controller manager (protocol.hpp: <robot>/controller_manager/<service>), ROS 2's controller_manager
    // (ros2_control Jazzy's controller_manager_msgs; an empty request has ROS's one placeholder byte), and Durchblick's
    // services for what ROS does through a controller's parameters and files (durchblick_msgs/srv/*@1): creating a
    // controller from a type and a JSON config, reading a controller's config back, reading a config file of the
    // robot.

    struct Duration  // builtin_interfaces/msg/Duration
    {
        std::int32_t sec = 0;
        std::uint32_t nanosec = 0;
        bool operator==(const Duration&) const = default;
    };

    struct ChainConnection  // controller_manager_msgs/msg/ChainConnection
    {
        std::string name;
        std::vector<std::string> reference_interfaces;
        bool operator==(const ChainConnection&) const = default;
    };

    // controller_manager_msgs/msg/ControllerState. state: unconfigured, inactive, active or finalized; type: the
    // controller's class name on the robot.
    struct ControllerState
    {
        std::string name;
        std::string state;
        std::string type;
        bool is_async = false;
        std::uint16_t update_rate = 0;  // Hz
        std::vector<std::string> claimed_interfaces;
        std::vector<std::string> required_command_interfaces;
        std::vector<std::string> required_state_interfaces;
        bool is_chainable = false;
        bool is_chained = false;
        std::vector<std::string> exported_state_interfaces;
        std::vector<std::string> reference_interfaces;
        std::vector<ChainConnection> chain_connections;
        bool operator==(const ControllerState&) const = default;
    };

    struct ListControllersRequest  // controller_manager_msgs/srv/ListControllers_Request (empty)
    {
        std::vector<std::uint8_t> serialize() const;
        static ListControllersRequest deserialize(std::span<const std::uint8_t> payload);
    };

    struct ListControllersResponse
    {
        std::vector<ControllerState> controller;
        std::vector<std::uint8_t> serialize() const;
        static ListControllersResponse deserialize(std::span<const std::uint8_t> payload);
    };

    struct SwitchControllerRequest  // controller_manager_msgs/srv/SwitchController_Request
    {
        static constexpr std::int32_t kBestEffort = 1, kStrict = 2, kAuto = 3, kForceAuto = 4;
        std::vector<std::string> activate_controllers;
        std::vector<std::string> deactivate_controllers;
        std::int32_t strictness = kStrict;
        bool activate_asap = false;
        Duration timeout;  // zero: infinite
        std::vector<std::uint8_t> serialize() const;
        static SwitchControllerRequest deserialize(std::span<const std::uint8_t> payload);
    };

    struct SwitchControllerResponse
    {
        bool ok = false;
        std::string message;
        std::vector<std::uint8_t> serialize() const;
        static SwitchControllerResponse deserialize(std::span<const std::uint8_t> payload);
    };

    struct UnloadControllerRequest  // controller_manager_msgs/srv/UnloadController_Request
    {
        std::string name;
        std::vector<std::uint8_t> serialize() const;
        static UnloadControllerRequest deserialize(std::span<const std::uint8_t> payload);
    };

    struct OkResponse  // the response of UnloadController (ok), its layout
    {
        bool ok = false;
        std::vector<std::uint8_t> serialize() const;
        static OkResponse deserialize(std::span<const std::uint8_t> payload);
    };

    // durchblick_msgs/srv/CreateController_Request: a controller `name` of the class `type`, configured by `config`
    // (JSON, which the robot side converts to its own config format), created inactive.
    struct CreateControllerRequest
    {
        std::string name;
        std::string type;
        std::string config;
        std::vector<std::uint8_t> serialize() const;
        static CreateControllerRequest deserialize(std::span<const std::uint8_t> payload);
    };

    // The response of CreateController (durchblick_msgs/srv/CreateController_Response): ok, or why not in message.
    struct MessageResponse
    {
        bool ok = false;
        std::string message;
        std::vector<std::uint8_t> serialize() const;
        static MessageResponse deserialize(std::span<const std::uint8_t> payload);
    };

    // durchblick_msgs/srv/GetControllerConfig_Request: the config a controller runs with; GetConfigFile_Request:
    // the robot's config file <type>/<name> (a file of the robot's controller configs, e.g. <type>/<name>.json).
    struct NameRequest  // GetControllerConfig_Request: name
    {
        std::string name;
        std::vector<std::uint8_t> serialize() const;
        static NameRequest deserialize(std::span<const std::uint8_t> payload);
    };

    struct GetConfigFileRequest
    {
        std::string type;
        std::string name;
        std::vector<std::uint8_t> serialize() const;
        static GetConfigFileRequest deserialize(std::span<const std::uint8_t> payload);
    };

    struct ConfigResponse  // GetControllerConfig_Response, GetConfigFile_Response: ok, message, config (JSON)
    {
        bool ok = false;
        std::string message;
        std::string config;
        std::vector<std::uint8_t> serialize() const;
        static ConfigResponse deserialize(std::span<const std::uint8_t> payload);
    };

}  // namespace durchblick
