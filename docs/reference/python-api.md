# Python

The Python API by example. Every example assumes `import xblick as db` and an open session.

Python and C++ share one implementation where it matters. The rings are C++ (`xblick::core`), and Python
reaches them through bindings (`xblick._core`). The stream API exists in both languages with the same topics
and messages. `tests/test_cpp_interop.py` runs the two languages against each other with every transport in both
directions. The [C++ reference](cpp-api.md) shows the same API in C++.

## Open a session

```python
session = db.open_session()                         # the router of this PC
session = db.open_session(connect=["other-pc"])     # the router of another PC
```

A script takes the connection options from its command line:

```python
from xblick import session as db_session
db_session.add_arguments(parser)                    # --connect, --listen, --zenoh-mode, --config
session = db_session.open_from_args(args)
```

[Sessions and routers](../concepts/sessions.md) explains the options.

## Publish a camera stream

```python
info = db.CameraInfo.pinhole(640, 480, fx=554.3, fy=554.3, cx=319.5, cy=239.5, frame_id="AzureKinectCamera")
camera = db.CameraPublisher(session, "cam/azurekinect/color", encoding="rgb8", frame_id="AzureKinectCamera",
                            camera_info=info, source="sim")
if camera.subscribed:
    camera.publish(rgb_array, stamp_ns=capture_time_ns)
```

`publish()` accepts a NumPy array or a device array. `camera.subscribed` is false while nobody reads the stream.
You can set `camera_info` and `frame_id` of a publisher at any time.

To use a real calibration, read a ROS calibration file. This is the YAML format of `camera_calibration`. The file
contains no frame, so you set it. Any other format raises `ValueError`.

```python
info = db.CameraInfo.read_file("azurekinect.yaml")
info.header.frame_id = "AzureKinectCamera"
```

## Publish a device array

```python
camera.publish(tensor, stamp_ns=capture_time_ns, stream=torch.cuda.current_stream())
```

Name the CUDA stream on which you computed the array. Without it, `publish()` waits for all GPU work of the
process. [Frames on the GPU](../concepts/gpu-frames.md) explains the reason. A publisher offers a CUDA ring when
you create it with `cuda_device=...`.

## Subscribe

```python
images = db.ImageSubscriber(session, "test/camera/color", transport="auto")
frame = images.latest(timeout=1.0)
```

`latest()` returns the newest frame, or `None` after the timeout.

| Attribute of a frame | Content |
| --- | --- |
| `image` | the pixels, H × W × channels |
| `encoding` | for example `rgb8` or `16UC1` |
| `header` | the stamp and the frame id |
| `seq`, `source` | the sequence number and the source from the attachment |
| `age_s`, `age_at_send_ns` | the [ages](../concepts/timestamps.md#ages) |

| Argument of `ImageSubscriber` | Default | Meaning |
| --- | --- | --- |
| `transport` | `"compressed"` | `auto`, `compressed`, `raw`, `shm` or `cuda`. Use `auto`. |
| `local` | `"auto"` | with `auto` on the publisher's host: `auto`, `shm` or `cuda` |
| `remote` | `"compressed"` | with `auto` on another host: `compressed` or `raw` |
| `device` | `None` | your GPU: an ordinal, `"cuda:N"` or a torch device. Without it the subscriber works on the CPU. |
| `copy` | `True` | `False` returns views into the ring |
| `camera_info` | `False` | also receive the calibration |
| `max_hz` | `None` | hand out at most this many frames per second |

## Read without a copy

```python
views = db.ImageSubscriber(session, "test/camera/color", transport="shm", copy=False)
frame = views.latest(timeout=1.0)
result = process(frame.image)
if not frame.is_valid():        # the publisher replaced the slot meanwhile
    result = None
```

The frame is a view into the ring. Check `is_valid()` after your work. [Rings](../concepts/rings.md) explains when
this is worth it.

## Keep frames on the GPU

```python
gpu = db.ImageSubscriber(session, "test/camera/color", transport="cuda")
tensor = torch.as_tensor(gpu.latest(timeout=1.0).image, device="cuda")
```

`xblick.tensors` converts frames for you: `tensors.rgb(frame, "cuda")` returns an (H, W, 3) `uint8` tensor from
any colour encoding, and `tensors.depth_metres(frame, "cuda")` returns depth in metres.

## Point clouds

```python
clouds = db.PointCloudSubscriber(session, "test/camera/points")
cloud = clouds.latest(timeout=1.0)
xyz = np.stack([cloud.points["x"], cloud.points["y"], cloud.points["z"]], -1)   # NaN where there is no depth
rgb = db.points.unpack_rgb(cloud.points["rgb"])
```

`cloud.points` is a structured NumPy array of shape (height, width) with the fields `x`, `y`, `z` and `rgb`.
`cloud.header.frame_id` is the depth camera. On the publisher's host the cloud comes from a ring. On another host
it arrives as a whole `PointCloud2`, which is large. See the [point cloud guide](../guides/point-clouds.md).

```python
gpu_clouds = db.PointCloudSubscriber(session, "test/camera/points", local="cuda")
```

With `local="cuda"` a cloud is an (h, w, 16) byte array on the GPU.

## Publish and read results

A node publishes its results as `vision_msgs` messages, one message per processed image, with the stamp of that
image.

```python
boxes = db.DetectionPublisher(session, "test/camera/boxer/boxes_3d", db.Detection3DArray)
box = db.Detection3D(results=[db.ObjectHypothesisWithPose(db.ObjectHypothesis("mug", 0.82))],
                     bbox=db.BoundingBox3D(db.Pose(db.Point(0.1, -0.2, 1.5), db.Quaternion()),
                                           db.Vector3(0.08, 0.12, 0.10)), id="7")     # metres
boxes.publish(db.Detection3DArray(db.Header(db.Time.from_ns(frame.stamp_ns), "AzureKinectCamera"), [box]))

reader = db.DetectionSubscriber(session, "test/camera/boxer/boxes_3d", print)   # calls print(message)
```

Classes without a place use `Classification`, for example the actions in a video clip. The message carries the
stamp of the newest image of the clip.

```python
actions = db.DetectionPublisher(session, "test/camera/vjepa2/actions", db.Classification)
actions.publish(db.Classification(db.Header(db.Time.from_ns(frame.stamp_ns), "AzureKinectCamera"),
                                  [db.ObjectHypothesis("Opening something", 0.62)]))
```

## Command a robot controller

A robot's controller listens on `<robot>/controller/<instance>/command` and reports below
`<robot>/controller/<instance>`. A simulation can take the controller's place with the same keys, so a client needs
no special case. The [topic reference](topics.md#controller) lists the keys and the messages.

**Send commands.** One `moveit_msgs/RobotState` per command, with pose targets of the controlled frames (for example
a TCP) and joint targets (an arm's posture, a hand's closure):

```python
from xblick.controller import CommandPublisher

commands = CommandPublisher(session, "robot", "arms", root_frame="base_link")
commands.send({"left_tcp": (position, quaternion_wxyz)}, {"left_shoulder_1": 0.1})
```

Send commands continuously at a fixed rate. The controller refuses a command whose stamp is not newer than the last
one it took, so an older command never follows a newer one. `send` stamps with the wall clock unless you pass
`stamp_ns`.

**Read the state.** `ControllerSubscriber` calls your functions in Zenoh's thread:

```python
from xblick.controller import ControllerSubscriber

def on_records(records, fields): ...      # every batch of records: count x len(fields)
def on_status(status): ...                # DiagnosticStatus at the controller's status rate (for example 50 Hz)

controller = ControllerSubscriber(session, "robot", "arms", on_records, on_status=on_status)
```

| Callback | Content |
| --- | --- |
| `on_records(records, fields)` | every recorded control cycle. The names are the controller's parameter `record_fields`. |
| `on_status(status)` | active, level, commands accepted and refused with the last reason |
| `on_diagnostics(status)` | about once a second |
| `on_hands(names, positions)` | the measured positions of the relayed hands |

The subscriber reads the fields when the controller answers, and raises `ConnectionError` if it does not. A batch of
another width, after a controller of the same name was created again, raises at `check()`.

**Manage controllers.** `ControllerManager` follows ROS 2's controller manager. The robot's software, or a bridge
to it, serves it.

```python
from xblick.controller_manager import ControllerManager

manager = ControllerManager(session, "robot")
config = manager.config_file("Impedance", "robot")       # the robot's configuration file, as a dict
config["zenoh"]["key"] = "robot/controller/arms"
manager.create("teleop_arms", "Impedance", config)        # inactive
manager.switch(activate=["teleop_arms"])                    # all or nothing; waits until active
...
manager.switch(deactivate=["teleop_arms"])
manager.unload("teleop_arms")
```

A read (`list`, `states`, `config`, `config_file`) raises `TimeoutError` when nobody answers and `RuntimeError` when
several managers answer. A command (`create`, `switch`, `unload`, and `torque_step` of a controller) goes out once,
and only after a read of the same manager or controller had exactly one answer. Two managers on one key would both
execute it, so the command is refused before anything is sent. A command raises:

| Exception | Meaning |
| --- | --- |
| `xblick.NotReached` (a `TimeoutError`) | not sent: no manager or controller became known within the timeout. You may try again. |
| `RuntimeError` | not sent (several managers answer the read), or the manager refused it with its reason |
| `xblick.NoAnswer` (a `RuntimeError`, no `TimeoutError`) | sent once, no answer within the timeout. The robot may still execute it. It was not sent again. |

**A command has no expiry.** A robot that is busy or briefly unreachable executes a command when it gets it, also
after the caller got `NoAnswer`. Check the robot's state before you act on a `NoAnswer`, and before you go near the
robot. The `timeout` of `torque_step` is how long the caller waits. It is no deadline for the step.

The controller side, for a simulation that takes the controller's place, is `CommandSubscriber` and
`ControllerPublisher` in the same module.

## More modules

| Module | Content | Read |
| --- | --- | --- |
| `xblick.node`, `history`, `tensors`, `rgbd` | helpers for nodes | [Writing nodes](../guides/writing-nodes.md), [Pairing streams](../guides/pairing-streams.md) |
| `xblick.parameters` | parameters of a node; `declare_options` makes a node's command-line options its parameters | [Parameters](parameters.md) |
| `xblick.poses` | poses of a camera or a robot at the stamp of an image, rotations and quaternions | |
| `xblick.rectify` | `rectify_map(info)`: the rectification map of a CameraInfo, as `cv2.initUndistortRectifyMap` gives it; `rectified_info(info)`: the CameraInfo of the rectified image | [C++](cpp-api.md#rectify-an-image) |
| `xblick.robot`, `sim`, `controller` | robot state, simulated worlds, the controller link | [Topics](topics.md) |
| `xblick.recording`, `video` | recording and playback | [Recording](../guides/recording.md) |
