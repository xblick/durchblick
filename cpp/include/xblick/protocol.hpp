// Durchblick's wire protocol in one place: topic names, message types and their versions, the attachment and the
// stream's info record. C++ uses these constants directly; Python reads them from xblick._core.protocol, so both
// languages always speak the same version. The layouts of the rings and their handshake are versioned in ring.hpp and
// cuda_ring.hpp.
//
// A stream "<namespace>/<camera>/<stream>" (e.g. robot/camera/color) has these keys below it (Zenoh key =
// topic without the leading slash):
//
//   image_raw             sensor_msgs/msg/Image
//   image_raw/compressed  sensor_msgs/msg/CompressedImage   (format as ROS image_transport's compressed plugin)
//   image_raw/shm/<host>         xblick_msgs/msg/ShmImage   (descriptor of a frame in a host ring of that host)
//   image_raw/cuda/<host>/<gpu>  xblick_msgs/msg/CudaImage  (descriptor of a frame in a CUDA ring on that GPU)
//   camera_info           sensor_msgs/msg/CameraInfo        (one per published frame, same header)
//   info                  queryable: the publisher's info record (JSON below), for discovery and transport choice
//   info/<host>/<id>      the publisher's liveliness token (<id>: 16 hex digits of its own), by which the stream's
//                         subscribers learn that a publisher came or went; each publisher its own key, because Zenoh
//                         reports a second token of a key that is held already to nobody
//   want/<host>/<id>      liveliness token of a reader (transport auto) on <host> that reads the stream while the
//                         stream's source has no ring on that host (<id>: 16 hex digits of its own): the relay of
//                         that host (xblick-relay) brings the streams wanted there into rings on it
//
// The keys of the rings name the ring's place: <host> is the host's id (host_id(), the boot id) and <gpu> the GPU's
// UUID (32 hex digits). A descriptor so reaches only readers on that host, and publishers on several hosts (a source
// and relays) offer rings of one stream side by side. A reader subscribes to image_raw/cuda/<host>/* for the rings on
// every GPU of its host.
//
// A point cloud stream "<namespace>/<camera>/<stream>" (e.g. robot/camera/points) has below it, in the same
// manner as an image stream (the points of a frame are written once into a ring; same host: only a descriptor travels):
//
//   cloud                 sensor_msgs/msg/PointCloud2           (the whole cloud over the network: large, see README)
//   cloud/shm/<host>         xblick_msgs/msg/ShmPointCloud2   (descriptor of a cloud in a host ring)
//   cloud/cuda/<host>/<gpu>  xblick_msgs/msg/CudaPointCloud2  (descriptor of a cloud in a CUDA ring)
//   cloud/compressed         xblick_msgs/msg/CompressedPointCloud2  (a cloud on fixed rays: ranges and colours)
//   cloud/rays               queryable, xblick_msgs/msg/Rays  (the rays of cloud/compressed)
//   info                  queryable, as for images
//
// A camera "<namespace>/<camera>" (e.g. robot/camera) with several streams has below it:
//
//   tf_static             tf2_msgs/msg/TFMessage: the fixed transforms between its streams' frames (header.frame_id
//                         of their camera_info), as ROS's /tf_static; published once and answered on request (a
//                         queryable on the same key), so that a subscriber that starts later gets them too
//   tf                    tf2_msgs/msg/TFMessage: the camera's moving transforms, as ROS's /tf: its pose in the
//                         world (header.frame_id "world", child a frame of its streams), stamped as the image taken
//                         from it, one message per image (the simulation; on a robot: from the robot's state)
//   imu                   sensor_msgs/msg/Imu: one message per sample, while somebody subscribes
//   diagnostics           diagnostic_msgs/msg/DiagnosticArray: how the camera's publishers are doing, about once a
//                         second; as ROS's /diagnostics, several publishers (camera, tracker), one status each by name
//
// A robot "<namespace>" (e.g. robot; the namespace of its cameras) has its state below it, as the robot's own software
// or the simulation has it (one message per state sample, the robot's stamp):
//
//   joint_states          sensor_msgs/msg/JointState: position (rad, m), velocity, effort (Nm, N) of every joint;
//                         a quantity the source does not have is an empty sequence, as ROS allows
//   tf                    tf2_msgs/msg/TFMessage: the localisation chain world -> map -> odom -> <root> (each link
//                         its own transform, so that a consumer can replace world -> map
//                         with its own localisation) and every robot frame relative to <root>
//   odom                  nav_msgs/msg/Odometry: the platform's pose in odom and its velocity (twist in <root>)
//   wrench/<sensor>       geometry_msgs/msg/WrenchStamped: a force-torque sensor (N, Nm) in its frame, e.g. wrench/left
//
// A controller "<robot>/controller/<instance>" (its name by role, e.g. arms; the controller speaks
// Zenoh itself or through a bridge) is told what to do by a client (a teleoperation, a
// policy), one message per command:
//
//   command               moveit_msgs/msg/RobotState: joint_state the joint targets by name (positions; e.g. an
//                         arm's posture, a hand's closure), multi_dof_joint_state the pose targets (joint_names the
//                         controlled frames, e.g. a hand's TCP; header.frame_id the robot's root frame), both with the
//                         command's stamp; no attached collision objects. Joints of an arm without a pose target are a
//                         joint target of their own (the controller computes the pose they give). A command whose
//                         stamp is not newer than the last one taken is refused (stale), so that an older command
//                         never follows a newer one; a command is taken whole or refused whole, with a reason on status
//
// and reports below it, one message per recorded control cycle (the controller decides which cycles it records, e.g.
// every 10th):
//
//   joint_states          sensor_msgs/msg/JointState: its joints' position, velocity and commanded effort
//   tf                    tf2_msgs/msg/TFMessage: per arm "<tcp>/measured", "<tcp>/reference" and "<tcp>/target" in
//                         the robot's root frame
//   wrench/<arm>          geometry_msgs/msg/WrenchStamped: the wrench it commands at an arm's TCP
//   record                std_msgs/msg/Float64MultiArray: everything it records, a batch of cycles per message:
//                         layout.dim[0] "record" (cycles), dim[1] "value" (its values); the values' names are its
//                         parameter record_fields, in order
//   status                diagnostic_msgs/msg/DiagnosticArray: one status at its status rate (e.g. every
//                         20 ms): active, level, commands_accepted, commands_rejected and last_reject_reason (the
//                         controller's), link_refused and link_last_refusal (refused before the controller: stale,
//                         unreadable, a frame or joint it does not have); hardware_id the controller's own name
//   diagnostics           diagnostic_msgs/msg/DiagnosticArray: its status, about once a second
//   hands/joint_states    sensor_msgs/msg/JointState: the measured positions of the hands it relays targets to (their
//                         own controllers'; 50 Hz, the rate the controller reads them at)
//
// and its parameters (as every node's, below): e.g. level (the compliance level) and state_decimation, and the service
//
//   torque_step           queryable: xblick_msgs/srv/TorqueStep (ros2_cdr_msgs.hpp): a bring-up torque step on
//                         one joint, if its config enables it; the response says whether it was taken, else why not
//
// A robot's controller manager "<robot>/controller_manager" (as ROS 2's /controller_manager; served by the
// robot's software or a bridge to it) offers queryables, the query's payload the request (CDR), the
// reply the response:
//
//   list_controllers      controller_manager_msgs/srv/ListControllers: every controller, its state (inactive,
//                         active) and type
//   switch_controller     controller_manager_msgs/srv/SwitchController: deactivate and activate controllers
//                         (strictness STRICT or BEST_EFFORT; timeout: waits for the states, zero: the manager's
//                         longest wait)
//   unload_controller     controller_manager_msgs/srv/UnloadController: delete an inactive controller
//   create_controller     xblick_msgs/srv/CreateController: a controller of a type from a JSON config, inactive
//                         (ROS: load_controller and configure_controller with the controller's parameters)
//   get_controller_config xblick_msgs/srv/GetControllerConfig: the config (JSON) a controller runs with
//   get_config_file       xblick_msgs/srv/GetConfigFile: a config file of the robot, <type>/<name> (JSON)
//
// Results of vision nodes about a camera's images go below the camera and the node, "<camera>/<node>/<name>" (e.g.
// robot/camera/boxer/boxes_3d), each sample one message, stamped as the image it is about:
//
//   vision_msgs/msg/Detection2DArray  objects in an image (boxes in pixels; header.frame_id the image's camera frame)
//   vision_msgs/msg/Detection3DArray  objects in 3D (box poses in header.frame_id, a frame of the camera's tf_static)
//   vision_msgs/msg/Classification    classes of the whole input without a place (e.g. actions in a clip of images,
//                                     stamped as its newest image)
//
// and "<camera>/<node>/<name>/info", a queryable for discovery as a stream's: {"v":4,"kind":"detections",
// "xblick":"<package version>","host":"<boot id>","type":"<the ROS type name>"}. A node's label image (pixels are
// ids, 0 none; e.g. a segmenter's instances) is an image stream "<camera>/<node>/<name>" whose info record says
// "labels":true.
//
// A simulation publishes, besides its robot's state and cameras (the robot's topics, as on the robot), its world below
// a namespace of its own ("<world>", e.g. sim) and what its controller is told (the commands above):
//
//   <world>/tf            tf2_msgs/msg/TFMessage: every simulated body's pose in the world (header.frame_id "world",
//                         child the body's name), one message per state sample (by default with every camera frame)
//   <world>/clock         rosgraph_msgs/msg/Clock: the simulated time, as ROS's /clock
//   <world>/events        JSON {"v":1,"stamp_ns":<ns>,"event":"<name>",...}: task events (start, phase, success,
//                         failure, end), one sample each
//   <robot>/command/tf, <robot>/command/joints   what its robot was told, at the control rate: TFMessage (pose
//                         targets) and JointState (positions, velocities or efforts), for recordings and datasets
//
// A blocking publisher (blocking congestion control, for recordings that must not lose a sample) answers
// "<key>/sent" for each of its keys: JSON {"v":1,"key":"<key>","sent":<samples put on the key>,"seq":<the attachment
// seq of the last one>,"matched":<a subscriber is known>}. Its samples carry the attachment (below), whose seq grows
// with every publish (an image stream's: one per frame, the same on all its keys).
//
// Payloads are CDR (XCDR1 little endian, 4-byte encapsulation header) as ROS 2 writes them. Every sample's Zenoh
// encoding is "application/cdr;<schema>": the ROS type name for ROS types (fixed by ROS), "<name>@<version>" for
// Durchblick's own types. A subscriber refuses a sample whose schema is not the one it reads, so publishers and
// readers of different protocol versions fail loudly instead of misreading each other.
//
// Attachment of every image, cloud and camera_info sample: compact JSON
// {"v":1,"seq":<n>,"publish_ns":<ns>,"source":"<s>","publish_called_ns":<ns>}: publish_ns taken right before that
// sample's put (after its encoding or ring copy: each transport its own), publish_called_ns when publish was called
// (the same for a frame's samples; since 0.3.30). A diagnostics sample's has "zid":"<the publishing session's
// id>" too (since 0.3.29; several sessions publish on one <camera>/diagnostics). Readers take the fields they know.
// Info record (reply of <base>/info): JSON {"v":4,"kind":"image"|"points","xblick":"<package version>",
// "host":"<boot id>","transports":[...],"frame_id":"...","source":"...","cuda_device_uuid":"<hex>"|null}, plus
// "encoding":"<ROS encoding>" for an image stream and "fields":["x","y","z",...] for a point cloud stream.
#pragma once

#include <cstdint>
#include <string>
#include <string_view>

namespace xblick::protocol
{
    inline constexpr std::string_view kImageRaw = "image_raw";
    inline constexpr std::string_view kCompressed = "image_raw/compressed";
    inline constexpr std::string_view kShm = "image_raw/shm";
    inline constexpr std::string_view kCuda = "image_raw/cuda";
    inline constexpr std::string_view kAnyGpu = "*";  // a subscriber's last chunk for the CUDA rings of a host
    inline constexpr std::string_view kCameraInfo = "camera_info";
    inline constexpr std::string_view kInfo = "info";
    inline constexpr std::string_view kWant = "want";  // below a stream: the readers' want tokens (above)
    inline constexpr std::string_view kTfStatic = "tf_static";  // below a camera, not a stream
    inline constexpr std::string_view kTf = "tf";                // below a camera
    inline constexpr std::string_view kImu = "imu";                  // below a camera
    inline constexpr std::string_view kDiagnostics = "diagnostics";  // below a camera
    inline constexpr std::string_view kJointStates = "joint_states";  // below a robot
    inline constexpr std::string_view kOdom = "odom";                  // below a robot
    inline constexpr std::string_view kWrench = "wrench";              // below a robot, then the sensor
    inline constexpr std::string_view kCloud = "cloud";                // below a point cloud stream
    inline constexpr std::string_view kCloudShm = "cloud/shm";
    inline constexpr std::string_view kCloudCuda = "cloud/cuda";
    // a cloud whose points lie on fixed rays, as each point's range (RVL) and its colour (JPEG): 0.3.63
    inline constexpr std::string_view kCloudCompressed = "cloud/compressed";
    inline constexpr std::string_view kCloudRays = "cloud/rays";  // queryable: the rays of cloud/compressed
    inline constexpr std::string_view kSent = "sent";        // below a blocking publisher's key: queryable, its count
    inline constexpr std::string_view kClock = "clock";      // below a simulation's world
    inline constexpr std::string_view kEvents = "events";    // below a simulation's world (JSON)
    inline constexpr std::string_view kCommand = "command";  // below a controller (RobotState); below a simulated robot
                                                             // what it was told (joints, tf)
    inline constexpr std::string_view kController = "controller";  // below a robot, then the instance
    inline constexpr std::string_view kRecord = "record";  // below a controller: its records (Float64MultiArray)
    inline constexpr std::string_view kStatus = "status";  // below a controller: its status at its status rate
    inline constexpr std::string_view kTorqueStep = "torque_step";  // below a controller: queryable (TorqueStep)
    inline constexpr std::string_view kControllerManager = "controller_manager";  // below a robot: its services
    inline constexpr std::string_view kListControllers = "list_controllers";
    inline constexpr std::string_view kSwitchController = "switch_controller";
    inline constexpr std::string_view kUnloadController = "unload_controller";
    inline constexpr std::string_view kCreateController = "create_controller";
    inline constexpr std::string_view kGetControllerConfig = "get_controller_config";
    inline constexpr std::string_view kGetConfigFile = "get_config_file";
    // A node's parameters, as ROS 2's (rcl_interfaces): below the node <namespace>/<name> the services, queryables
    // whose query payload is the request (CDR) and whose reply the response, and the liveliness token
    // <namespace>/<name>/parameters while it offers them; every node's ParameterEvents on kParameterEvents (one key,
    // as ROS's /parameter_events; the event's `node` names the node). A ROS mode maps these keys to rmw_zenoh's
    // (/<namespace>/<name>/<service>, /parameter_events, its liveliness tokens); the data stay the same.
    inline constexpr std::string_view kListParameters = "list_parameters";
    inline constexpr std::string_view kDescribeParameters = "describe_parameters";
    inline constexpr std::string_view kGetParameters = "get_parameters";
    inline constexpr std::string_view kGetParameterTypes = "get_parameter_types";
    inline constexpr std::string_view kSetParameters = "set_parameters";
    inline constexpr std::string_view kSetParametersAtomically = "set_parameters_atomically";
    inline constexpr std::string_view kParameterEvents = "parameter_events";
    inline constexpr std::string_view kParametersToken = "parameters";  // below the node

    inline constexpr std::uint32_t kAttachmentVersion = 1;
    // 3 (0.3.59): the rings' keys name host and GPU; 4 (0.5.0): the field "durchblick" is "xblick"
    inline constexpr std::uint32_t kInfoVersion = 4;
    inline constexpr std::uint32_t kSentVersion = 1;  // the reply of <key>/sent

    struct MessageType
    {
        std::string_view name;
        std::uint32_t version;  // 0: a ROS type, defined by ROS and named without a version

        std::string schema() const
        {
            return version == 0 ? std::string(name) : std::string(name) + "@" + std::to_string(version);
        }

        std::string encoding() const
        {
            return "application/cdr;" + schema();
        }
    };

    inline constexpr MessageType kImageType{"sensor_msgs/msg/Image", 0};
    inline constexpr MessageType kCompressedImageType{"sensor_msgs/msg/CompressedImage", 0};
    inline constexpr MessageType kCameraInfoType{"sensor_msgs/msg/CameraInfo", 0};
    inline constexpr MessageType kTFMessageType{"tf2_msgs/msg/TFMessage", 0};
    inline constexpr MessageType kImuType{"sensor_msgs/msg/Imu", 0};
    inline constexpr MessageType kDiagnosticArrayType{"diagnostic_msgs/msg/DiagnosticArray", 0};
    inline constexpr MessageType kShmImageType{"xblick_msgs/msg/ShmImage", 1};
    inline constexpr MessageType kCudaImageType{"xblick_msgs/msg/CudaImage", 1};
    inline constexpr MessageType kPointCloud2Type{"sensor_msgs/msg/PointCloud2", 0};
    inline constexpr MessageType kShmPointCloud2Type{"xblick_msgs/msg/ShmPointCloud2", 1};
    inline constexpr MessageType kCudaPointCloud2Type{"xblick_msgs/msg/CudaPointCloud2", 1};
    inline constexpr MessageType kCompressedPointCloud2Type{"xblick_msgs/msg/CompressedPointCloud2", 1};
    inline constexpr MessageType kRaysType{"xblick_msgs/msg/Rays", 1};
    inline constexpr MessageType kDetection2DArrayType{"vision_msgs/msg/Detection2DArray", 0};
    inline constexpr MessageType kDetection3DArrayType{"vision_msgs/msg/Detection3DArray", 0};
    inline constexpr MessageType kClassificationType{"vision_msgs/msg/Classification", 0};
    inline constexpr MessageType kJointStateType{"sensor_msgs/msg/JointState", 0};
    inline constexpr MessageType kOdometryType{"nav_msgs/msg/Odometry", 0};
    inline constexpr MessageType kWrenchStampedType{"geometry_msgs/msg/WrenchStamped", 0};
    inline constexpr MessageType kClockType{"rosgraph_msgs/msg/Clock", 0};
    inline constexpr MessageType kFloat64MultiArrayType{"std_msgs/msg/Float64MultiArray", 0};
    inline constexpr MessageType kRobotStateType{"moveit_msgs/msg/RobotState", 0};  // a controller's command
    // a node's parameters (rcl_interfaces, as ROS 2's): the event and the services' requests and responses
    inline constexpr MessageType kParameterEventType{"rcl_interfaces/msg/ParameterEvent", 0};
    inline constexpr MessageType kListParametersRequestType{"rcl_interfaces/srv/ListParameters_Request", 0};
    inline constexpr MessageType kListParametersResponseType{"rcl_interfaces/srv/ListParameters_Response", 0};
    inline constexpr MessageType kDescribeParametersRequestType{"rcl_interfaces/srv/DescribeParameters_Request", 0};
    inline constexpr MessageType kDescribeParametersResponseType{"rcl_interfaces/srv/DescribeParameters_Response", 0};
    inline constexpr MessageType kGetParametersRequestType{"rcl_interfaces/srv/GetParameters_Request", 0};
    inline constexpr MessageType kGetParametersResponseType{"rcl_interfaces/srv/GetParameters_Response", 0};
    inline constexpr MessageType kGetParameterTypesRequestType{"rcl_interfaces/srv/GetParameterTypes_Request", 0};
    inline constexpr MessageType kGetParameterTypesResponseType{"rcl_interfaces/srv/GetParameterTypes_Response", 0};
    inline constexpr MessageType kSetParametersRequestType{"rcl_interfaces/srv/SetParameters_Request", 0};
    inline constexpr MessageType kSetParametersResponseType{"rcl_interfaces/srv/SetParameters_Response", 0};
    inline constexpr MessageType kSetParametersAtomicallyRequestType{"rcl_interfaces/srv/SetParametersAtomically_Request", 0};
    inline constexpr MessageType kSetParametersAtomicallyResponseType{"rcl_interfaces/srv/SetParametersAtomically_Response", 0};
    // a controller's torque step and a robot's controller manager (ROS 2's controller_manager_msgs, Durchblick's own)
    inline constexpr MessageType kTorqueStepRequestType{"xblick_msgs/srv/TorqueStep_Request", 1};
    inline constexpr MessageType kTorqueStepResponseType{"xblick_msgs/srv/TorqueStep_Response", 1};
    inline constexpr MessageType kListControllersRequestType{"controller_manager_msgs/srv/ListControllers_Request", 0};
    inline constexpr MessageType kListControllersResponseType{"controller_manager_msgs/srv/ListControllers_Response", 0};
    inline constexpr MessageType kSwitchControllerRequestType{"controller_manager_msgs/srv/SwitchController_Request", 0};
    inline constexpr MessageType kSwitchControllerResponseType{"controller_manager_msgs/srv/SwitchController_Response", 0};
    inline constexpr MessageType kUnloadControllerRequestType{"controller_manager_msgs/srv/UnloadController_Request", 0};
    inline constexpr MessageType kUnloadControllerResponseType{"controller_manager_msgs/srv/UnloadController_Response", 0};
    inline constexpr MessageType kCreateControllerRequestType{"xblick_msgs/srv/CreateController_Request", 1};
    inline constexpr MessageType kCreateControllerResponseType{"xblick_msgs/srv/CreateController_Response", 1};
    inline constexpr MessageType kGetControllerConfigRequestType{"xblick_msgs/srv/GetControllerConfig_Request", 1};
    inline constexpr MessageType kGetControllerConfigResponseType{"xblick_msgs/srv/GetControllerConfig_Response", 1};
    inline constexpr MessageType kGetConfigFileRequestType{"xblick_msgs/srv/GetConfigFile_Request", 1};
    inline constexpr MessageType kGetConfigFileResponseType{"xblick_msgs/srv/GetConfigFile_Response", 1};

    // Durchblick's default Zenoh port (also Zenoh's own default).
    inline constexpr std::uint16_t kDefaultPort = 7447;
}  // namespace xblick::protocol
