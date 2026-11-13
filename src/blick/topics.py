"""Topic names and the Zenoh key expressions they map to.

A camera stream is ``<namespace>/<camera>/<stream>``, e.g. ``robot/camera/color``, and has the topics of ROS
image_transport and camera_info_manager below it:

* ``<base>/image_raw``             sensor_msgs/Image
* ``<base>/image_raw/compressed``  sensor_msgs/CompressedImage (JPEG, PNG, or RVL for depth)
* ``<base>/camera_info``           sensor_msgs/CameraInfo, one per published frame, with that frame's header
* ``<base>/image_raw/shm/<host>``  durchblick_msgs/ShmImage: the frame in a shared-memory ring of that host
* ``<base>/image_raw/cuda/<host>/<gpu>``  durchblick_msgs/CudaImage: the frame in a CUDA ring (device memory) on
  that GPU (its UUID, 32 hex digits) of that host
* ``<base>/info``                  queryable: the publisher's kind, host, version and transports (JSON), for ``auto``

``shm`` and ``cuda`` are Durchblick's own, in the manner of image_transport plugins: the pixels stay in the ring, the
message says where (msg.py, cpp/include/durchblick/ring.hpp). Their keys name the place of the ring (the host's id,
``_core.host_id()``, and the GPU), so that a descriptor reaches only the readers that can open the ring, and
publishers on several hosts (a source and relays) offer rings of one stream side by side.

A point cloud stream (e.g. ``robot/camera/points``) has the same structure:

* ``<base>/cloud``                 sensor_msgs/PointCloud2 (the whole cloud over the network)
* ``<base>/cloud/shm/<host>``      durchblick_msgs/ShmPointCloud2: the cloud in a shared-memory ring of that host
* ``<base>/cloud/cuda/<host>/<gpu>``  durchblick_msgs/CudaPointCloud2: the cloud in a CUDA ring of that host and GPU
* ``<base>/info``                  as above

A camera ``<namespace>/<camera>`` with several streams has ``<camera>/tf_static`` (tf2_msgs/TFMessage): the fixed
transforms between its streams' frames, as ROS's ``/tf_static``; published once and answered on request;
``<camera>/tf`` (tf2_msgs/TFMessage): its moving transforms, as ROS's ``/tf``, its pose in the world (parent frame
``world``) stamped as the image taken from it, one message per image. And
``<camera>/imu`` (sensor_msgs/Imu, one message per sample) and ``<camera>/diagnostics``
(diagnostic_msgs/DiagnosticArray, about once a second, one status per publisher, as ROS's ``/diagnostics``).

A robot ``<namespace>`` (e.g. ``robot``, the namespace of its cameras) has its state below it, one message per state
sample: ``<robot>/joint_states`` (sensor_msgs/JointState), ``<robot>/tf`` (tf2_msgs/TFMessage: the localisation chain
``world`` -> ``map`` -> ``odom`` -> root, each link its own transform, and every robot frame relative to the root),
``<robot>/odom`` (nav_msgs/Odometry: the platform in ``odom`` and its velocity) and ``<robot>/wrench/<sensor>``
(geometry_msgs/WrenchStamped: a force-torque sensor).

What a robot's controller is told, by a client or by a simulation (below): ``<robot>/command/tf`` (tf2_msgs/TFMessage:
pose targets, child the controlled frame, in the robot's root frame) and ``<robot>/command/joints``
(sensor_msgs/JointState: joint targets by name); the two with the same stamp belong together, joints of an arm without
a pose target of that stamp are a joint target of their own. A controller ``<robot>/controller/<instance>`` reports
per recorded cycle ``joint_states``, ``tf`` (per arm ``<tcp>/measured``, ``/reference``, ``/target``), ``wrench/<arm>``
(the commanded wrench), ``record`` (std_msgs/Float64MultiArray: a batch of cycles x its values, named by its parameter
``record_fields``), ``diagnostics`` (about once a second) and ``hands/joint_states`` (the relayed hands' measured
positions, 50 Hz, the rate the controller reads them at).

A simulation (sim.py) publishes the robot's topics as the robot does, and besides them its world below a namespace of
its own (``sim``): ``<world>/tf`` (tf2_msgs/TFMessage: every body's pose in ``world``), ``<world>/clock``
(rosgraph_msgs/Clock) and ``<world>/events`` (JSON task events); and what the robot's controller is told
(``<robot>/command/*``, above). A blocking publisher answers ``<key>/sent`` with its count of the key
(recording.py checks a recording against it).

The namespace is chosen at start-up (robot name), so the simulation and the robot publish the same topics and
subscribers cannot tell them apart. The Zenoh key is the topic without a leading slash. Note: this is ROS's topic
layout, not rmw_zenoh's key format (domain id, type name and type hash in the key), so ROS 2 nodes on rmw_zenoh do
not see these topics without a bridge.
"""

from __future__ import annotations

import re
import secrets

from ._core import protocol as _protocol

# the names are the C++ core's (cpp/include/durchblick/protocol.hpp), so both languages use the same
IMAGE_RAW = _protocol.IMAGE_RAW
COMPRESSED = _protocol.COMPRESSED
CAMERA_INFO = _protocol.CAMERA_INFO
SHM = _protocol.SHM
CUDA = _protocol.CUDA
INFO = _protocol.INFO
TF_STATIC = _protocol.TF_STATIC
TF = _protocol.TF
WORLD = "world"  # the parent frame of the cameras' poses on <camera>/tf
MAP = "map"  # the localisation's map frame (<robot>/tf: world -> map -> odom -> root)
ODOM_FRAME = "odom"  # the odometry's frame, whose drift the localisation corrects
IMU = _protocol.IMU
DIAGNOSTICS = _protocol.DIAGNOSTICS
JOINT_STATES = _protocol.JOINT_STATES
ODOM = _protocol.ODOM
WRENCH = _protocol.WRENCH
CLOUD = _protocol.CLOUD
CLOUD_SHM = _protocol.CLOUD_SHM
CLOUD_CUDA = _protocol.CLOUD_CUDA
CLOUD_COMPRESSED = _protocol.CLOUD_COMPRESSED
CLOUD_RAYS = _protocol.CLOUD_RAYS
SENT = _protocol.SENT
CLOCK = _protocol.CLOCK
EVENTS = _protocol.EVENTS
COMMAND = _protocol.COMMAND
CONTROLLER = _protocol.CONTROLLER
RECORD = _protocol.RECORD
STATUS = _protocol.STATUS
WORLD_NAMESPACE = "sim"  # a simulation's world, the namespace of <world>/tf, clock, events (SimPublisher's default)

# ROS 2 name rules for one token: letters, digits, underscores, not starting with a digit
_TOKEN = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


class TopicError(ValueError):
    pass


def check_name(path: str) -> str:
    tokens = path.strip("/").split("/")
    for token in tokens:
        if not _TOKEN.match(token):
            raise TopicError(f"{path!r}: {token!r} is not a valid ROS name token ([A-Za-z_][A-Za-z0-9_]*)")
    return "/".join(tokens)


def stream_base(namespace: str, camera: str, stream: str) -> str:
    return check_name(f"{namespace}/{camera}/{stream}")


def image_raw(base: str) -> str:
    return f"{check_name(base)}/{IMAGE_RAW}"


def compressed(base: str) -> str:
    return f"{check_name(base)}/{COMPRESSED}"


def camera_info(base: str) -> str:
    return f"{check_name(base)}/{CAMERA_INFO}"


ANY_GPU = "*"  # a subscriber's key of the CUDA rings on every GPU of a host


def shm(base: str, host: str) -> str:
    """The key of the stream's host ring on ``host`` (``_core.host_id()``)."""
    return f"{check_name(base)}/{SHM}/{host}"


def cuda(base: str, host: str, gpu: str = ANY_GPU) -> str:
    """The key of the stream's CUDA ring on the GPU ``gpu`` (its UUID, 32 hex digits) of ``host``; without ``gpu`` a
    subscriber's key expression for the rings on every GPU of the host."""
    return f"{check_name(base)}/{CUDA}/{host}/{gpu}"


def ring_place(key: str) -> tuple[str, str | None] | None:
    """(key without the place, place) of a ring's key: ``.../image_raw/shm/<host>`` gives (``.../image_raw/shm``,
    ``<host>``), a CUDA ring's place is ``<host>/<gpu>``; None for a key that is no ring's."""
    parts = key.split("/")
    for end, chunks in ((SHM, 1), (CUDA, 2), (CLOUD_SHM, 1), (CLOUD_CUDA, 2)):
        n = end.count("/") + 1
        if len(parts) > n + chunks and "/".join(parts[-n - chunks:-chunks]) == end:
            return "/".join(parts[:-chunks]), "/".join(parts[-chunks:])
    return None


def info(base: str) -> str:
    return f"{check_name(base)}/{INFO}"


def alive(base: str, host: str, publisher: str) -> str:
    """The key of a stream publisher's liveliness token: ``<base>/info/<host>/<publisher>``, ``publisher`` an id of
    its own (``new_publisher_id``). Each publisher has its own key: Zenoh reports a second token of a key that is
    held already to nobody."""
    return f"{check_name(base)}/{INFO}/{host}/{publisher}"


def alive_all(base: str) -> str:
    """The key expression of the liveliness tokens of every publisher of a stream."""
    return f"{check_name(base)}/{INFO}/**"


def new_publisher_id() -> str:
    return secrets.token_hex(8)


WANT = _protocol.WANT


def want(base: str, host: str, reader: str) -> str:
    """The key of a reader's liveliness token for a stream it reads on ``host`` without a ring of the stream's source
    there: ``<base>/want/<host>/<reader>``, ``reader`` an id of its own (``new_publisher_id``). The relay of that host
    (durchblick-relay) brings the streams wanted there into rings on it."""
    return f"{check_name(base)}/{WANT}/{host}/{reader}"


def wanted_on(host: str) -> str:
    """The key expression of the want tokens of every stream on ``host``."""
    return f"**/{WANT}/{host}/*"


def wanted_stream(key: str) -> str:
    """The stream of a want token's key."""
    return "/".join(key.split("/")[:-3])


def tf_static(camera: str) -> str:
    return f"{check_name(camera)}/{TF_STATIC}"


def tf(camera: str) -> str:
    return f"{check_name(camera)}/{TF}"


def imu(camera: str) -> str:
    return f"{check_name(camera)}/{IMU}"


def diagnostics(camera: str) -> str:
    return f"{check_name(camera)}/{DIAGNOSTICS}"


def joint_states(robot: str) -> str:
    return f"{check_name(robot)}/{JOINT_STATES}"


def odom(robot: str) -> str:
    return f"{check_name(robot)}/{ODOM}"


def wrench(robot: str, sensor: str) -> str:
    """A force-torque sensor of a robot, ``<robot>/wrench/<sensor>``, e.g. ``robot/wrench/left``."""
    return check_name(f"{robot}/{WRENCH}/{sensor}")


def detections(camera: str, node: str, name: str) -> str:
    """A vision node's results about a camera's images, ``<camera>/<node>/<name>`` (protocol.hpp), e.g.
    ``robot/camera/boxer/boxes_3d``."""
    return check_name(f"{camera}/{node}/{name}")


def sent(key: str) -> str:
    """The queryable of a blocking publisher's count of ``key``, ``<key>/sent`` (protocol.hpp)."""
    return f"{check_name(key)}/{SENT}"


def clock(world: str) -> str:
    return f"{check_name(world)}/{CLOCK}"


def events(world: str) -> str:
    return f"{check_name(world)}/{EVENTS}"


def command(robot: str, name: str) -> str:
    """What a simulated robot was told, ``<robot>/command/<name>`` (``joints``, ``tf``): a simulation records it
    (sim.py). A controller takes its commands on :func:`controller_command`."""
    return check_name(f"{robot}/{COMMAND}/{name}")


def controller(robot: str, instance: str, name: str = "") -> str:
    """A controller's reports, ``<robot>/controller/<instance>[/<name>]`` (``joint_states``, ``tf``, ``wrench/<arm>``,
    ``record``, ``status``, ``diagnostics``); without ``name`` the controller's own key (its node, for its parameters)."""
    return check_name(f"{robot}/{CONTROLLER}/{instance}" + (f"/{name}" if name else ""))


def controller_command(robot: str, instance: str) -> str:
    """A controller's commands, ``<robot>/controller/<instance>/command`` (moveit_msgs/RobotState: joint targets and
    the controlled frames' pose targets in one message, protocol.hpp)."""
    return controller(robot, instance, COMMAND)


def cloud(base: str) -> str:
    return f"{check_name(base)}/{CLOUD}"


def cloud_compressed(base: str) -> str:
    return f"{check_name(base)}/{CLOUD_COMPRESSED}"


def cloud_rays(base: str) -> str:
    """The queryable that answers the rays of ``cloud/compressed`` (msg.Rays)."""
    return f"{check_name(base)}/{CLOUD_RAYS}"


def cloud_shm(base: str, host: str) -> str:
    return f"{check_name(base)}/{CLOUD_SHM}/{host}"


def cloud_cuda(base: str, host: str, gpu: str = ANY_GPU) -> str:
    return f"{check_name(base)}/{CLOUD_CUDA}/{host}/{gpu}"
