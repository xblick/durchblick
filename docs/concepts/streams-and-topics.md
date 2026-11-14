# Streams and topics

A stream is the sequence of frames from one source, for example the colour images of one camera. A topic is a
Zenoh key on which samples travel. One stream uses several topics, because each [transport](transports.md) has its
own. This page explains how Durchblick names streams and where a node puts its results. The
[topic reference](../reference/topics.md) lists every topic with its message type.

## The name of a stream

```
<namespace>/<camera>/<stream>        for example   test/camera/color
```

- The **namespace** is the system that the camera belongs to, usually a robot.
- The **camera** is one device. All its streams share one clock and one set of coordinate frames.
- The **stream** is one output of the camera, such as `color`, `depth` or `left`.

You pass this name to a publisher or a subscriber. You never write the topics below it by hand.

## The topics of a stream

Below the name of a stream, Durchblick uses the topic layout of ROS 2's `image_transport` and adds its own topics
for the rings.

| Topic | Content | Who uses it |
| --- | --- | --- |
| `<stream>/image_raw` | the pixels | `raw` subscribers |
| `<stream>/image_raw/compressed` | a JPEG, a PNG, or RVL for depth | `compressed` subscribers |
| `<stream>/image_raw/shm/<host>` | a descriptor for the host ring on that host | `shm` readers on that host |
| `<stream>/image_raw/cuda/<host>/<gpu>` | a descriptor for the CUDA ring on that GPU | `cuda` readers on that host |
| `<stream>/camera_info` | the calibration, with the same header as the frame | subscribers with `camera_info=True` |
| `<stream>/info` | a record of the publisher: host, version, transports, encoding | `auto` subscribers, `xblick list` |

A ring exists on one host, so its topic names the host, and a CUDA ring's topic also names the GPU. A descriptor
therefore reaches only readers that can open the ring. [Transports](transports.md) explains how a subscriber
finds the right topic.

A point cloud stream has the same structure with `cloud` in place of `image_raw`. Its `cloud/compressed` needs
the rays of the points, which the publisher answers on `cloud/rays`
([compressed point clouds](transports.md#compressed-point-clouds)).

## What belongs to a camera

Some topics describe the camera as a whole and sit beside its streams.

| Topic | Content |
| --- | --- |
| `<camera>/tf_static` | the fixed transforms between the camera's frames, for example from depth to colour |
| `<camera>/tf` | the pose of the camera in the world, one message per image |
| `<camera>/imu` | the samples of the camera's inertial sensor |
| `<camera>/diagnostics` | the state of every program that publishes for this camera, about once per second |

## Where a node puts its results

A node reads the streams of a camera and publishes what it computed about them. Its results belong to that camera,
so they sit below it:

```
<camera>/<node>/<name>               for example   test/camera/sam3/instances
```

This rule has three useful consequences.

- A node runs on any camera. You give it the camera's name, and its outputs follow.
- Two cameras can run the same node without a conflict.
- A viewer finds every result of a camera below one prefix. `xblick rerun` uses this to show all results without
  options.

A result carries the stamp of the image that it was computed from. A consumer can therefore match a mask, a pose
and an image of the same instant. [Timestamps](timestamps.md) has the rules.

## Robot state and simulated worlds

A robot publishes its state below its namespace: `<robot>/joint_states`, `<robot>/tf`, `<robot>/odom` and
`<robot>/wrench/<sensor>`. A simulation publishes its world below a namespace of its own, for example `sim/tf`
and `sim/clock`.

A simulation publishes exactly the topics of the robot that it simulates. A consumer therefore needs no second code
path for the simulation. Each sample states in its metadata where it came from: `source` is `sim`, `robot`, `test`,
`video` or `cam`.

## Related pages

- [Topic reference](../reference/topics.md): every topic, type and rate.
- [Protocol](protocol.md): the message types and their versions.
- [Parameters](../reference/parameters.md): the topics on which a node offers its settings.
