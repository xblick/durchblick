# Timestamps

Every sample in Durchblick carries several times. This page explains what each time means, who sets it, and which
times you may compare with each other.

All times are integer nanoseconds since the Unix epoch on the wall clock of a PC, unless a table below states
another clock. Python reads this clock with `time.time_ns()`, C++ with `std::chrono::system_clock`. A program
measures durations, such as a timeout or the milliseconds of a processing stage, with its monotonic clock. These
durations never leave the program as times.

## The stamp is the capture time

The most important time is the *stamp*. It states the instant that the data describes: when the camera exposed the
image, or when the robot measured its joints. The stamp is the field `header.stamp` of every ROS message with a
header. JSON samples have the field `stamp_ns` with the same meaning.

The source sets the stamp, and every node passes it on. A node stamps its result with the stamp of the input that
it computed the result from. It never uses its processing time or its publish time. A consumer can therefore match
a result with the image, the robot state and the results of other nodes for the same instant.

| A node computes its result from | The result carries the stamp of | Nodes |
| --- | --- | --- |
| one image | that image | Boxer, SAM 3, InsightFace, NeMO, robot mask (the depth frame) |
| a colour frame with depth and instances paired to it | the colour frame | the users of `RGBDInputs` (FoundationPose, NeMO, the mesh makers), GEM-X, IVT with `--depth` |
| a stereo pair: each left frame with the nearest right frame | the left image, so that depth and left image pair by equal stamps | Fast-FoundationStereo |
| a stereo pair | the left image | IVT |
| depth with colour | the depth frame | `xblick-points` |
| a clip or a sequence | its newest frame | V-JEPA 2, TAPNext++ |
| a map | the newest frame in the map | nvblox |
| a registration that ran in the background | the frame that the published pose describes | FoundationPose |

We checked all nodes against this table on 2026-11-05.

## All time fields

| Field | Where it is | Who sets it | Meaning |
| --- | --- | --- | --- |
| `header.stamp` | every ROS message with a header | the source, then every node passes it on | the capture time |
| `stamp_ns` | JSON samples, such as bodies and `<world>/events` | as `header.stamp` | the capture time |
| `seq` | attachment | the publisher | the count of the publisher's frames. It is the same on all topics of one frame. |
| `publish_called_ns` | attachment | the publisher, when your code calls `publish()` | the same for all samples of one frame |
| `publish_ns` | attachment | the publisher, directly before it puts the sample | when this sample left the publisher. Each transport of a frame has its own value. |
| `source` | attachment | the publisher | `sim`, `robot`, `test`, `video` or `cam` |
| `zid` | attachment of diagnostics samples | `DiagnosticsPublisher` | the id of the Zenoh session, because several sessions publish on one `<camera>/diagnostics` |
| `receive_ns` | only in the subscriber | the reader, when the sample arrives | the reader's wall clock at arrival |

The attachment is the small JSON object that travels beside the payload. [Protocol](protocol.md) shows it.

`publish_ns` includes the work for the transport. For a `compressed` sample it includes the JPEG encoding, and for
a ring it includes the copy into the ring. The difference `publish_ns - publish_called_ns` is therefore the time
that the publisher needed for this transport.

## Ages

An age is the difference between a clock and the stamp. Durchblick gives you two.

| Age | Definition | Tells you |
| --- | --- | --- |
| Age at send | `publish_ns - stamp` | how old the data was when it left its publisher |
| Age at receive | `receive_ns - stamp` | how old the data is when it reaches you, including the way to you |

| Where | Age at send | Age at receive |
| --- | --- | --- |
| Python | `Frame.age_at_send_ns`, `stamps.age_at_send_ns(sample)`, `DetectionSubscriber.age_at_send_ns` | `Frame.age_s` (seconds) |
| C++ | `Frame::age_at_send_ns()`, `Cloud::age_at_send_ns()` | |
| `xblick hz` | printed | printed |
| Terminal interface | median and 95th percentile over the last second, per topic | the same |
| `xblick rerun` | a plot per topic at `ages/<topic>` in ms. `--no-ages` removes the plots. | |

The age at send is empty when a sample has no attachment or no stamp.

**An age is exact only on one clock.** The age at send compares the stamp with the publisher's clock. The age at
receive compares it with the reader's clock. Both are exact when the source set the stamp on that same PC. When the
stamp comes from another PC, the age contains the difference between the two clocks. It is then as good as the
clock synchronisation of the PCs, for example with chrony or PTP.

## The stamp by source

| Source | How it sets the stamp | You can compare it with |
| --- | --- | --- |
| Azure Kinect (xblick-azure-kinect) | The SDK puts the device timestamp at the middle of the exposure. The node moves it to the host's wall clock with a filtered offset, as Microsoft's ROS driver does. | the camera's other streams exactly, because colour, depth, IR and IMU share one device clock. Other clocks only up to a constant delay, which we have not measured. |
| Roboception rc_visard (xblick-roboception) | the camera's own timestamp of the image. With PTP, which is on by default, the camera's clock follows the PTP master of the network. | the host's clock only when the PTP master follows the host's clock |
| A live simulation | the server's wall clock in the simulation step in which the frame is due, before the frame is rendered. Robot state and object poses get the same stamp. | the other programs of the PC. The rendering time counts into the age of the frame. |
| An offline simulation through `xblick.sim.SimPublisher` | the simulated time, which the simulation also publishes on `<world>/clock` | the other samples of the same simulation |
| A robot state bridge | the time of the state sample in the robot's own system | the clocks of the robot's PCs |
| `xblick play` with an MCAP file | the recorded stamps. In a loop, each pass moves them forward. | the times of the recording |
| `xblick play` with videos | the start of the playback in wall time plus the time of the frame in its video | the wall clock of the playing PC, and the other streams of the same playback exactly |
| A publisher that gets no stamp | `CameraPublisher` and `PointCloudPublisher` use the publish time. | Avoid this. Always pass the capture time. |

The node repositories describe their stamps in detail. The [recording guide](../guides/recording.md) and the
[video guide](../guides/video.md) describe the stamps of a playback.

A camera's `<camera>/tf` that comes from a robot's state carries the stamp of the state. It does not carry the
stamp of an image. A reader interpolates the pose at the stamp of the image. `xblick.poses` does this.

## What follows for your node

- Pass the capture stamp of your input to every `publish()`.
- Rely on stamps that only grow. A looped playback moves the stamps forward on each pass for this reason.
- Pair streams by stamp. Do not pair them by arrival. The [pairing guide](../guides/pairing-streams.md) shows how.
- Publish through Durchblick's publishers. A sample from a plain Zenoh publisher has no attachment, so it has no
  age at send. `camera.CountedPublisher` adds the attachment to any key.

## Status and limits

- The clock synchronisation between the PCs of a robot is not checked yet. Ages across PCs are therefore
  unverified.
- The Azure Kinect's stamp is late by the mean time from the middle of the exposure to the host. We have not
  measured this delay.
- Which instant of the exposure the rc_visard stamps is not checked on a device.
- Two nodes still publish some samples without an attachment: xblick-gem-x (bodies) and xblick-mapping
  (the cuVSLAM topics). State of 2026-11-05.
