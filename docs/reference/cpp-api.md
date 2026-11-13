# C++

The C++ API by example. It has the same topics, messages and behaviour as the [Python API](python-api.md).

The rings are implemented once, in C++ (`durchblick::core`). The stream API is `durchblick::durchblick`, built on
zenoh-cpp. The library opens the CUDA driver (`libcuda.so.1`) at run time. It therefore loads on a host without a
GPU, and only the CUDA ring needs one.

## Use it in CMake

```cmake
find_package(durchblick 0.4 REQUIRED)        # finds zenohc and zenohcxx 1.10, JPEG and PNG itself
target_link_libraries(my_program PRIVATE durchblick::durchblick)
```

| Target | Content |
| --- | --- |
| `durchblick::durchblick` | the stream API |
| `durchblick::core` | the rings alone |
| `durchblick::points` | the point cloud node as a library. Needs `find_package(durchblick 0.4 REQUIRED COMPONENTS points)` and a Durchblick built with `DURCHBLICK_POINTS`. |

## Publish a camera stream

```cpp
#include <durchblick/camera.hpp>

zenoh::Session session = durchblick::open_session({});      // the router of this PC
durchblick::CameraPublisher camera(session, "robot/camera/color",
                                   {.encoding = "rgb8", .frame_id = "AzureKinectCamera", .source = "robot"});
if (camera.subscribed())
    camera.publish(durchblick::HostImage{pixels, height, width}, capture_ns);
```

`publish` also accepts a `DeviceImage` for a frame in GPU memory.

## Pass a camera's JPEG through

A camera that delivers JPEG needs no decoding for subscribers that want JPEG.

```cpp
if (camera.needs_pixels())
    camera.publish(decode(jpeg), capture_ns, jpeg);     // somebody wants pixels: decode once
else
    camera.publish_jpeg(jpeg, capture_ns);              // only JPEG subscribers
```

## Subscribe

```cpp
durchblick::ImageSubscriber images(session, "robot/camera/color", {.transport = durchblick::Transport::Auto});
if (auto frame = images.latest(true, std::chrono::milliseconds(100)))
    use(frame->data(), frame->height, frame->width, frame->step, frame->encoding);
```

`SubscriberOptions` has the same fields as the arguments in Python: `transport`, `local`, `cuda_device`, `max_hz`
and `keep_recent`. `transport_reason()` states which transport the subscriber chose and why.
`frame->age_at_send_ns()` returns the age at send.

A partner stream pairs its samples with the frames of a primary stream, as Python's `PartnerHistory` does
([pairing streams](../guides/pairing-streams.md)). `durchblick/pairing.hpp` has the rule, in `durchblick::core`.

```cpp
durchblick::ImageSubscriber depth(session, "robot/camera/depth", {.keep_recent = 8});
// max_offset_ns: 5 ms; std::nullopt for half the partner's measured period
if (auto stamp = durchblick::partner_of(depth.recent_stamps(), color_frame.stamp_ns(), 5'000'000))
    if (auto frame = depth.at(*stamp, std::chrono::nanoseconds(0)))
        use(color_frame, *frame);
```

`partner_of` returns the stamp of the nearest kept sample within the offset, once no nearer sample can come. It
returns nothing while a nearer one may still come. Call it again when the next sample arrives. `partner_period_ns`
is the measured period.

## Transforms, IMU and diagnostics

```cpp
// the fixed poses between the camera's frames (p_parent = R p_child + t), on robot/camera/tf_static
durchblick::StaticTransformPublisher tf(session, "robot/camera",
    {{{{now, "DepthCamera"}, "ColorCamera", durchblick::Transform::from_matrix(rotation, {0.032, 0.002, 0.004})}}});

durchblick::ImuPublisher imu(session, "robot/camera");
if (imu.subscribed()) imu.publish(sample);              // a durchblick::Imu

durchblick::DiagnosticsPublisher diagnostics(session, "robot/camera");
diagnostics.publish({{now, ""}, {{durchblick::DiagnosticStatus::kOk, "my_camera", "30.0 captures/s", serial, {}}}});
```

A program publishes one diagnostics status per publisher, about once per second.

## Point clouds

A publisher lends you a slot of the CUDA ring. Your kernel writes the cloud directly into the slot, and readers
take it from there.

```cpp
#include <durchblick/points.hpp>

durchblick::PointCloudPublisher points(session, "robot/camera/points",
                                       {.layout = durchblick::xyzrgb_layout(), .frame_id = "AzureKinectDepthCamera",
                                        .cuda_device = 0});
if (points.subscribed()) {
    const std::uint64_t slot = points.loan(height, width);
    my_kernel<<<grid, block, 0, stream>>>(reinterpret_cast<float4*>(slot), ...);
    points.commit(reinterpret_cast<std::uint64_t>(stream), false, capture_ns);
}

durchblick::PointCloudSubscriber clouds(session, "robot/camera/points", {.local = durchblick::Transport::Cuda});
if (auto cloud = clouds.latest(true, std::chrono::milliseconds(100)))
    use(cloud->device_ptr, cloud->height, cloud->width, cloud->point_step);     // data() for a host cloud
```

`commit` takes the CUDA stream of your kernel, so the publisher waits for exactly that work.

## Rectify an image

`durchblick/rectify.hpp` computes the map of OpenCV's `initUndistortRectifyMap` from a CameraInfo, with its R and its
P: the image without lens distortion, turned into the camera of P. For the two cameras of a stereo pair calibrated
with `cv::stereoRectify`, the rectified images share their rows.

```cpp
#include <durchblick/rectify.hpp>

const durchblick::RectifyMap map = durchblick::rectify_map(durchblick::rectify_camera(info));  // once per calibration
const durchblick::PixelLayout layout = durchblick::rectify_layout(frame.encoding);               // rgb8, mono16, ...
std::vector<std::uint8_t> out(std::size_t(frame.height) * frame.width * layout.pixel_bytes());
durchblick::remap(map, layout, frame.data(), frame.step, out.data());
publisher.publish(durchblick::HostImage{out.data(), frame.height, frame.width}, frame.stamp_ns());
```

`remap` interpolates 8-bit images bilinearly and takes the nearest pixel of 16-bit images, so that a depth is never
blended. A pixel whose point lies outside the camera's image is 0. `map.x` and `map.y` are what `nppiRemap` and
`cv::remap` take, for a remap on the GPU. `rectified_info(info)` is the CameraInfo of the rectified image: K is P's
left 3 x 3, the distortion is zero, R is the identity and P is unchanged. The right camera of a pair keeps P[3] = -f b.

`rectify_map` and `remap` are in `durchblick::core` and need no messages. `rectify_camera` and `rectified_info` take a
CameraInfo and are in `durchblick::durchblick`. Python has the same as `blick.rectify`.

## Messages

`durchblick/msg.hpp` contains the message types with the names of ROS. Tests compare their bytes with the ROS
definitions (vision_msgs 4.1.1, in `tests/data/vision_msgs`).

## Headers

| Header | Content |
| --- | --- |
| `camera.hpp` | `CameraPublisher`, `ImageSubscriber`, `StaticTransformPublisher`, `StaticTransformSubscriber`, `TransformPublisher`, `ImuPublisher`, `DiagnosticsPublisher`, `RobotStatePublisher` |
| `points.hpp`, `point_cloud_node.hpp` | point cloud streams, the point cloud node |
| `msg.hpp`, `image.hpp` | messages, encodings and codecs |
| `parameters.hpp` | parameters of a node ([parameters](parameters.md)) |
| `options.hpp` | `Options`: a program's command-line options, parsed and offered as its parameters ([a C++ program's options](parameters.md#a-c-programs-options-as-parameters)) |
| `pairing.hpp` | `partner_of`, `partner_period_ns`: a partner stream's sample of a primary frame, by stamp |
| `protocol.hpp` | topic names, message types and versions |
| `ring.hpp`, `cuda_ring.hpp` | the rings |
