# Topics

Every topic of Durchblick with its message type and its rate. [Streams and topics](../concepts/streams-and-topics.md)
explains how the names are built.

In the tables, `<base>` is the name of a stream, for example `robot/camera/color`. "On request" means that
the publisher sends only while a subscriber exists for this topic.

## Camera stream

| Topic | Type | Sent |
| --- | --- | --- |
| `<base>/image_raw` | `sensor_msgs/msg/Image` | on request |
| `<base>/image_raw/compressed` | `sensor_msgs/msg/CompressedImage`. JPEG, PNG for lossless images, or RVL for 16-bit depth ([codecs](../concepts/transports.md#the-codecs-of-compressed)). | on request |
| `<base>/image_raw/shm/<host>` | `durchblick_msgs/msg/ShmImage@1`, the descriptor for a host ring on that host | on request |
| `<base>/image_raw/cuda/<host>/<gpu>` | `durchblick_msgs/msg/CudaImage@1`, the descriptor for a CUDA ring on that GPU of that host | on request, if the publisher has a `cuda_device` |
| `<base>/camera_info` | `sensor_msgs/msg/CameraInfo` with the header of the frame | with every frame, on request |
| `<base>/info` | queryable, JSON: kind (`image` or `points`), host, Durchblick version, transports, encoding or fields | answered when queried |
| `<base>/info/<host>/<id>` as a liveliness token | none. It exists while the publisher runs. `<id>` is a random id of the publisher. | to the subscribers of the stream when a publisher comes or goes |
| `<base>/want/<host>/<id>` as a liveliness token | none. A subscriber with `auto` on `<host>` holds it while the stream's source has no ring on that host. `<id>` is a random id of the subscriber. | to the relay of that host |

`<host>` is the id of the publisher's host (`blick._core.host_id()`, the boot id of the machine). `<gpu>` is the
UUID of the ring's GPU with 32 hex digits. A reader subscribes to `<base>/image_raw/cuda/<host>/*` for the rings on
every GPU of its host.

## Point cloud stream

| Topic | Type | Sent |
| --- | --- | --- |
| `<base>/cloud` | `sensor_msgs/msg/PointCloud2`, the whole cloud | on request |
| `<base>/cloud/compressed` | `durchblick_msgs/msg/CompressedPointCloud2@1`: the ranges of the points on their rays (RVL) and their colours (JPEG or PNG) ([compressed point clouds](../concepts/transports.md#compressed-point-clouds)) | on request, if the publisher has rays |
| `<base>/cloud/rays` | queryable, `durchblick_msgs/msg/Rays@1`: the direction of each point's ray | answered when queried |
| `<base>/cloud/shm/<host>` | `durchblick_msgs/msg/ShmPointCloud2@1`: the fields of the PointCloud2 without data, and the place in a host ring | on request |
| `<base>/cloud/cuda/<host>/<gpu>` | `durchblick_msgs/msg/CudaPointCloud2@1`: the same for a CUDA ring | on request, if the publisher has a `cuda_device` |
| `<base>/info` | as for a camera stream, with kind `points` | answered when queried |

## Camera

| Topic | Type | Content | Sent |
| --- | --- | --- | --- |
| `<camera>/tf_static` | `tf2_msgs/msg/TFMessage` | the fixed transforms between the camera's frames, for example depth to colour. Same role as `/tf_static` in ROS. | once, and answered when queried by later subscribers |
| `<camera>/tf` | `tf2_msgs/msg/TFMessage` | the pose of the camera in the world. Parent frame `world`, child a frame of the camera's streams. | one message per image, with the stamp of the image |
| `<camera>/imu` | `sensor_msgs/msg/Imu` | one message per sample | on request |
| `<camera>/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | one status for each program that publishes for the camera. The attachment names the sending session (`zid`). | about once per second |

## Node results

| Topic | Type | Content | Sent |
| --- | --- | --- | --- |
| `<camera>/<node>/<name>` | `vision_msgs/msg/Detection2DArray` | boxes in pixels | one message per processed image, with the stamp of the image |
| | `vision_msgs/msg/Detection3DArray` | box poses in a frame of the camera's `tf_static` | the same |
| | `Classification` | classes without a place, for example the actions in a clip | the same. A clip uses the stamp of its newest image. |

Example: `robot/camera/boxer/boxes_3d`. Each node repository lists its topics in its README.

## Parameters

A node with parameters has a key `<namespace>/<name>`, for example `robot/camera`. It answers the parameter
services of ROS 2 below this key. See [parameters](parameters.md).

| Topic | Type | Sent |
| --- | --- | --- |
| `<node>/list_parameters`, `describe_parameters`, `get_parameters`, `get_parameter_types`, `set_parameters`, `set_parameters_atomically` | queryables: `rcl_interfaces/srv/<Service>_Request` in, `_Response` out | answered when queried |
| `parameter_events` | `rcl_interfaces/msg/ParameterEvent` of every node. The field `node` is the full name of the node. | at each declaration and each change |
| `<node>/parameters` | liveliness token | while the node offers parameters |

## Robot state

A robot publishes its state below its namespace, for example `robot`. It sends one message per state sample,
stamped with the robot's time.

| Topic | Type | Content |
| --- | --- | --- |
| `<robot>/joint_states` | `sensor_msgs/msg/JointState` | position (rad, m), velocity and effort (Nm, N) of every joint. A quantity that the source lacks is an empty sequence. |
| `<robot>/tf` | `tf2_msgs/msg/TFMessage` | the chain `world` → `map` → `odom` → root, each link as its own transform, and every robot frame relative to the root |
| `<robot>/odom` | `nav_msgs/msg/Odometry` | the platform in `odom`, and its velocity in the root frame |
| `<robot>/wrench/<sensor>` | `geometry_msgs/msg/WrenchStamped` | a force-torque sensor in its frame |

Each link of the localisation chain is a transform of its own, so a consumer can replace `world` → `map` with its
own localisation. On a robot, a bridge from the robot's software publishes these topics.
In a simulation, the simulator publishes them.

A `<camera>/tf` that comes from a robot's state carries the stamp of the state. A reader interpolates the pose at
the stamp of an image.

## Simulated world

A simulation publishes the topics of the robot unchanged. In addition it publishes its world below a namespace of
its own, for example `sim`, and the commands that the robot's controller receives. `blick.sim.SimPublisher`
does this. Every sample carries the simulated time.

| Topic | Type | Content | Default rate |
| --- | --- | --- | --- |
| `<world>/tf` | `tf2_msgs/msg/TFMessage` | the pose of every simulated body in `world`. The child frame is the name of the body. | with every frame of the fastest camera, or `state_hz` |
| `<world>/clock` | `rosgraph_msgs/msg/Clock` | the simulated time, as `/clock` in ROS | the control rate (1 kHz) |
| `<world>/events` | JSON | task events: `start`, `phase`, `success`, `failure` with `reason`, `end`, or others | one sample per event |
| `<robot>/command/joints` | `sensor_msgs/msg/JointState` | the command per joint: position, velocity or effort. The closure of a hand is its position. | the control rate |
| `<robot>/command/tf` | `tf2_msgs/msg/TFMessage` | commanded poses, such as the TCP target of a hand, in `header.frame_id` | the control rate |

An event is a JSON object such as `{"v":1,"stamp_ns":…,"event":"phase","phase":"grasp"}`. We chose JSON over
`std_msgs/String` because MCAP readers show a typed, versioned object directly.

In a simulation, `<robot>/joint_states` also arrives at the control rate. `<robot>/tf` and `<robot>/odom` arrive
with the state of the world. The `<robot>/command/*` topics record what a simulated robot was told, for recordings
and datasets.

## Controller

A client, such as a teleoperation or a policy, drives a controller of the robot on the controller's own key
`<robot>/controller/<instance>`. The instance is the name of the controller by its role, for example `arms`.

**Commands.** The client publishes one `moveit_msgs/msg/RobotState` per command on `…/command`.

| Field | Content |
| --- | --- |
| `multi_dof_joint_state` | the pose targets. `joint_names` are the controlled frames, and `header.frame_id` is the root frame of the robot. |
| `joint_state` | the joint targets, as positions by name |

Both fields carry the stamp of the command. The joints of an arm without a pose target form a joint target of
their own. The controller accepts or refuses a command as a whole. It refuses a command in two cases:

- The stamp is not newer than the stamp of the last accepted command (stale).
- The stamp differs by more than 250 ms from the controller's clock.

**Reports.** The controller publishes below the same key, once per recorded control cycle. Its parameter
`state_decimation` selects every n-th cycle.

| Topic | Type | Content |
| --- | --- | --- |
| `…/command` | `moveit_msgs/msg/RobotState` | the commands. The client publishes, the controller reads. |
| `…/status` | `diagnostic_msgs/msg/DiagnosticArray` | active, level, the commands accepted and rejected with the reason, and the refusals of the link. Sent at the status rate (for example every 20 ms). |
| `…/joint_states` | `sensor_msgs/msg/JointState` | position, velocity and commanded effort of its joints |
| `…/tf` | `tf2_msgs/msg/TFMessage` | per arm `<tcp>/measured`, `<tcp>/reference` and `<tcp>/target` in the root frame |
| `…/wrench/<arm>` | `geometry_msgs/msg/WrenchStamped` | the wrench that it commands at the TCP of the arm |
| `…/record` | `std_msgs/msg/Float64MultiArray` | every value of the recorded cycles, one batch per message. `layout.dim` is `record` × `value`. The parameter `record_fields` names the values. |
| `…/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | its status, about once per second |
| `…/hands/joint_states` | `sensor_msgs/msg/JointState` | the measured positions of the hands to which it relays targets, at 50 Hz |

The controller is the parameter node `<robot>/controller/<instance>` with the parameters `level`,
`state_decimation` and `record_fields` (read-only).

In Python, `topics.controller(robot, instance, name)` returns the keys. `blick.controller` has both sides:
`CommandPublisher` and `ControllerSubscriber` for a client, and `CommandSubscriber` and `ControllerPublisher` for a
controller or a simulation. `CommandSubscriber` refuses stale commands.
