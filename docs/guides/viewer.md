# Viewer

`xblick rerun` shows camera streams and the results of the nodes in [Rerun](https://rerun.io), an open-source
viewer for multimodal data. It places everything on the capture time of the frames.

```bash
pip install "xblick[rerun]"
xblick rerun
```

Without arguments, the command shows every stream, label image and detection topic that it finds. It looks again
every 5 s, so a node that starts later appears too. It starts a viewer on this machine and sends its own layout: a
3D view and a 2D view per stream.

## Common calls

```bash
xblick rerun --connect sim-host                        # everything from another host
xblick rerun --prefix robot/camera               # the streams and results of one camera
xblick rerun --serve --connect sim-host                # a viewer connects with: rerun --connect <printed URL>
xblick rerun --connect-viewer rerun+http://laptop:9876/proxy --connect sim-host  # to a running viewer
xblick rerun robot/camera/color --save head.rrd --connect sim-host         # write a recording file
```

Put the streams before `--connect`, because `--connect` accepts several endpoints. The
[command line reference](../reference/cli.md#xblick-rerun-streams-) lists all options.

## What it draws

| Data | Drawn as |
| --- | --- |
| Colour images (JPEG, PNG) | the image, and the calibration as a pinhole camera |
| Depth (`16UC1` in mm) | a depth image, and a point cloud in 3D |
| 16-bit grey images (IR) | an image |
| Point clouds | coloured points in 3D, at most `--cloud-rate` (5) per second |
| Tracks (clouds with `u, v`, `object_id`, `visible`) | points in 3D and over the image of their frame, one colour per object |
| Bodies | skeletons in 3D and in the images of the camera |
| 3D detections (object poses, boxes) | boxes with class, score and tracking id, coloured by class, in 3D and in the images |
| 2D detections | rectangles in the image of their frame's stream |
| Label images (instances) | one colour per id over the image |
| IMU, diagnostics | time series, a text log |

A camera's `tf_static` places its streams. Each stream sits at the pose of its frame, and the bodies sit with them.
A camera whose pose arrives on `<camera>/tf` moves through the world with its streams, points, bodies and boxes.
The 3D view then has z up.

`--trajectory <key>` draws the poses of any `TFMessage` topic as a path, with axes at the newest pose. Two
trajectories compare two pose sources:

```bash
xblick rerun --trajectory <camera>/vslam/tf --trajectory <camera>/tf
```

## How results are found

Node results announce themselves in the same way as streams. A `DetectionPublisher` answers `<key>/info` with the
kind `detections`. The info record of a label image says `labels`. A publisher sets this with
`CameraPublisher(..., labels=True)`. For a node that does not announce its results, name the topics with
`--detections <key>` and `--segmentation <key>`.

## Rates and ages

For every topic, the viewer plots three time series on the capture timeline.

| Entity | Content |
| --- | --- |
| `rates/<topic>` | samples per second that arrive here, over the last second |
| `ages/<topic>` | the age at send in ms |
| `ages_received/<topic>` | the age at receive in ms. Across PCs it needs synchronised clocks. |

`ages_now` shows the newest values as a table:

| topic | rate Hz | age at send ms | age at receive ms | dropped/s |
| --- | --: | --: | --: | --: |
| robot/camera/color | 30.0 | 0.6 | 1.9 | 0 |

"dropped/s" counts the samples that the publisher sent and that did not arrive. The viewer finds them as gaps in
the sequence numbers. For a stream that the viewer limits itself, such as point clouds, the rate is the limited
rate. `--no-ages` removes all of these. [Timestamps](../concepts/timestamps.md#ages) defines the ages.

## What you should know

- **The viewer keeps publishers busy.** Rerun cannot tell which streams you look at, so `xblick rerun`
  subscribes to every stream that it shows for as long as it runs. A publisher that renders only for subscribers
  renders all that time.
- **A point cloud from another host arrives compressed only where its publisher has rays**, as the point cloud node has. Other clouds arrive whole and are large. See [point clouds](point-clouds.md).
- **A publisher must be reachable.** Zenoh peers do not forward each other's data. A publisher to which the viewer
  has no direct connection must listen on an endpoint of its own.
- **Rerun sends anonymous usage data** unless you turn it off once per machine with `rerun analytics disable`.
- **The viewer binary.** `--spawn` starts the `rerun` program that `rerun-sdk` installed beside this Python
  (`<venv>/bin/rerun`). Otherwise it uses a `rerun` on the `PATH`. If it finds neither, it prints where it looked.
  You therefore do not need an activated virtual environment.
