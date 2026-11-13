# Point clouds

The point cloud node `xblick-points` computes coloured point clouds from a depth stream on the GPU. It computes
each cloud once for all readers and publishes it as a point cloud stream at `<camera>/points`.

```bash
xblick-points --depth robot/camera/depth --color robot/camera/color
```

The node works with any camera. It needs the `camera_info` of the streams and the camera's `tf_static`, which
places the colour camera relative to the depth camera. It does not need `tf_static` when depth and colour share one
frame. C++ programs can use the node as a class from `xblick/point_cloud_node.hpp`.

## The cloud

| Property | Value |
| --- | --- |
| Layout | `x, y, z, rgb` in 16 bytes, which is `PointXYZRGB` of PCL. Without `--color`: `x, y, z` and padding. |
| Unit and frame | metres, in the frame of the depth camera (the `frame_id` of its `camera_info`) |
| Organisation | as the depth image, `height` × `width`. `is_dense` is false, as in `depth_image_proc` of ROS. |
| Missing points | NaN where the pixel has no depth, or where the depth exceeds `--max-depth` |
| Depth encodings | `16UC1` or `mono16` in mm, `32FC1` in m |
| Lens models | `plumb_bob` and `rational_polynomial`. The node refuses other models at start. |

## Options

| Option | Default | Meaning |
| --- | --- | --- |
| `--depth STREAM` | | the depth stream |
| `--color STREAM` | none | the colour stream |
| `--max-depth M` | 0, no limit | points beyond this depth become NaN |
| `--max-offset-ms MS` | 10 | the largest difference between the stamps of a depth frame and its colour frame |
| `--max-hz HZ` | every frame | at most this many clouds per second |
| `--camera`, `--output`, `--source`, `--cuda-device` | | the camera, the name of the output stream, the `source` field and the GPU. See `xblick-points --help`. |
| `--range-step M` | 0.001 | the step of `z` in the compressed cloud, which a reader on another host takes |
| `--jpeg-quality Q` | 95 | the quality of the colours in the compressed cloud |
| `--remote compressed\|raw` | | the transport when the camera is on another host. `compressed` uses the cameras' lossless codec for depth (RVL or PNG) and JPEG for colour. |

## How the node computes a cloud

**Geometry.** The node computes the viewing ray of every depth pixel once per calibration on the CPU. This step
inverts the lens distortion by iteration. For each frame, a GPU kernel multiplies the depth by the ray.

**Colour.** The kernel moves each point into the colour camera with `tf_static` and projects it with the colour
camera's distortion. The point takes the colour of the nearest pixel. It is black where the colour camera does not
see it.

The node does not test for occlusion. At the edge of an object, a point that the colour camera cannot see takes the
colour of the surface in front of it. The transformation of the Azure Kinect SDK behaves the same way in this
direction.

**Pairing.** A colour frame belongs to a depth frame when their stamps differ by at most `--max-offset-ms`. The node
takes the nearest colour frame once no nearer one can come, by the rule of `PartnerHistory`. A depth frame without
a colour frame produces no cloud, and the diagnostics count it. With `--max-hz`, depth is the
primary stream and colour is its partner, so the limit loses no pair. The node never decodes the colour frames of
skipped depth frames. See [pairing streams](pairing-streams.md).

**Work only for readers.** Without a reader of the cloud, the node drops its own subscriptions. The camera then
sends it neither depth nor colour. With a reader, the node takes its inputs from the CUDA rings on the camera's
host, without a copy through the CPU.

## Read a cloud

| The reader is | It gets | Cost |
| --- | --- | --- |
| on the node's host, on the GPU | the cloud in the CUDA ring (`local="cuda"`) | no copy |
| on the node's host, on the CPU | the cloud in the host ring | one download from the GPU per cloud, only while somebody reads |
| on another host | the [compressed cloud](../concepts/transports.md#compressed-point-clouds): each point's `z` and colour, about a twentieth of the cloud | one download from the GPU and about 5 ms on the CPU per cloud, only while somebody reads |
| on another host, with `remote="raw"` | the whole `PointCloud2` | 5.9 MB per cloud, 177 MB/s at 30 Hz |

A reader with `auto` on another host takes the compressed cloud. The node passes the rays of the depth camera to
its publisher, so the reader gets the same points: `z` in steps of `--range-step`, and the colours through JPEG.
With depth in millimetres and the default step of 1 mm the coordinates are the same but for the rounding of a
float. A reader older than Durchblick 0.3.63 (Python) or 0.3.64 (C++) takes the whole cloud.

A point beyond 65535 steps (65.5 m at 1 mm) stops the node with an error while a reader takes the compressed
cloud. Set `--max-depth` or a larger `--range-step` for such a camera.

The [Python reference](../reference/python-api.md#point-clouds) and the
[C++ reference](../reference/cpp-api.md#point-clouds) show the reader code.

## Cost and accuracy

Measured on 2026-10-28 with a recording of an Azure Kinect (640 x 576 depth, 720p colour) on an RTX PRO 6000.

| Quantity | Value |
| --- | --- |
| Host time per cloud (input copies, kernel, sends) | 1.7-2.7 ms |
| Rate from a 33 Hz recording | 30.8 clouds per second |
| Delay from the depth stamp to a reader on the same host, CUDA ring | 1.9 ms |
| The same, host ring | 2.0 ms |
| The same, `PointCloud2` over TCP on the same host | 4.7 ms (median) |

Compared with the Azure Kinect SDK and the same calibration:

- Position: all 368,640 points lie at most 0.012 mm from `k4a_calibration_2d_to_3d`.
- Colour: 99.96 % of the points take the pixel that `k4a_calibration_3d_to_2d` gives. The others round to the
  neighbouring pixel.

## Diagnostics

The node reports on `<camera>/diagnostics` with the status `point_cloud`: clouds per second, depth and colour
frames per second, depth frames without a cloud, the transports of its inputs, and the host time per cloud.
