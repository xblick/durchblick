# Command line

The command `blick` inspects, records and plays streams.
Without a subcommand it opens the [terminal interface](../guides/tui.md). `blick <command> --help` prints every
option.

## Connection options

Every command accepts the same options for its Zenoh session.

| Option | Meaning |
| --- | --- |
| none | Connect to the router of this PC at `127.0.0.1:7447`. |
| `--connect <endpoint>` | Connect to another router or program. An endpoint is a host, `host:port`, or a locator such as `tcp/10.0.0.5:7447`. |
| `--listen <endpoint>` | Listen on this endpoint. |
| `--zenoh-mode peer\|client` | `peer` is the default. Use `client` for a program on another PC that only connects to a router. |
| `--config FILE` | A Zenoh JSON5 configuration. It replaces the other options. |

A peer that only connects does not listen, and a router does not route between such peers. A program on another PC
that only connects to a router therefore uses `--zenoh-mode client`.
[Sessions and routers](../concepts/sessions.md) explains the background.

## Inspect

### `blick list [prefix]`

Lists every stream with its encoding or fields, its transports, its host and its Durchblick version. The command
queries the info records and subscribes to nothing.

| Option | Meaning |
| --- | --- |
| `prefix` | only keys below this prefix, for example `robot` |
| `--seconds S` | how long to look for streams |
| `--traffic` | subscribe to every topic and show rates and sizes |

### `blick hz <stream>`

Subscribes to an image or point cloud stream. Prints the rate, the age at send and at receive, and the missing
frames.

| Option | Meaning |
| --- | --- |
| `--transport auto\|compressed\|raw\|shm` | the transport to measure |
| `--seconds S` | stop after this time |

### `blick snapshot <stream> <file>`

Saves one frame as an image file. PNG keeps 16-bit depth. Options: `--transport`, `--timeout`.

### `blick diagnostics [prefix]`

Prints the state of every program that publishes for a camera, once per second.

| Option | Meaning |
| --- | --- |
| `prefix` | only cameras below this prefix |
| `--values` | every value of a status. The default prints level and message. |
| `--seconds S` | stop after this time |

### `blick rerun [streams ...]`

Shows streams and node results in the Rerun viewer. Without arguments it shows everything that it finds. Needs the
extra `blick[rerun]`. See the [viewer guide](../guides/viewer.md).

| Option | Meaning |
| --- | --- |
| `--prefix P`, `--seconds S` | without streams: where and how long to look for them |
| `--points BASE` | a point cloud stream to show as well. `--cloud-rate` limits its rate. |
| `--detections KEY`, `--segmentation KEY`, `--trajectory KEY` | additional results to draw |
| `--no-ages` | leave out the plots of rates and ages |
| `--stale-after S` | when a stream counts as stale |
| `--spawn` | start a viewer on this machine (the default) |
| `--connect-viewer URL` | send to a viewer that already runs, for example `rerun+http://laptop:9876/proxy` |
| `--serve [PORT]` | let viewers connect to this process (default port 9876). `--web [PORT]` also serves the web viewer. |
| `--save FILE` | write an `.rrd` recording |
| `--report S` | seconds between the status lines |

## Test

### `blick test-camera`

Publishes a moving test pattern on `robot/test_camera/color`.

| Option | Default |
| --- | --- |
| `--namespace`, `--camera`, `--stream` | `robot`, `test_camera`, `color` |
| `--size W H` | 640 480 |
| `--fps` | 15 |
| `--vfov` | the vertical field of view in degrees for the camera info |
| `--frame-id` | the frame id of the images |

### `durchblick-cpp-camera`, `durchblick-cpp-probe`

The C++ counterparts: a test pattern publisher and a prober that prints JSON. `--jpeg-frames` makes the camera
pass JPEG frames through. `--points` tests point cloud streams. Both programs are part of the C++ build.

## Parameters

### `blick param`

Reads and changes the parameters of a node. See [parameters](parameters.md).

| Option | Meaning |
| --- | --- |
| `--nodes` | list the nodes that offer parameters |
| `--node NAMESPACE/NAME` | the node to talk to |
| `names` | only these parameters. The default is all. |
| `--list`, `--get` | print names and values (the default) |
| `--describe` | also print type, range, constraints and read-only |
| `--set NAME:=VALUE` | change a parameter. Repeat the option for several. Values are YAML scalars. |
| `--atomically` | with `--set`: change all or none |
| `--dump FILE` | write the parameters as a ROS params file |
| `--timeout S` | how long to wait for the node |

## Record and play

See the [recording guide](../guides/recording.md) and the [video guide](../guides/video.md). Both commands need the
extra `blick[record]`. Videos need `blick[video]`.

### `blick record`

| Option | Meaning |
| --- | --- |
| `--dir DIR` | where the files go. `${VAR}` and `~` are expanded. |
| `--name NAME` | the name of the recorder, used for its files and its control topics |
| `--file FILE` | one file, relative to `--dir`, without splitting |
| `--paused` | start idle and wait for a command |
| `--namespace NS`, `--prefix P` | record only streams and detections below this prefix |
| `--key KEY_EXPR` | record exactly these key expressions. Repeat the option for several. |
| `--points` | record point clouds too. They are large and can be computed again from depth. |
| `--split-minutes M` | start a new file after this time |
| `--compression none\|lz4\|zstd` | the MCAP compression |
| `--blocking` | for the keys of blocking publishers: block, and check that no sample is lost |
| `--log-time receive\|header` | the log time of a message: its arrival, or its own stamp |
| `--report S` | seconds between the status lines |

### `blick recorder`

Sends commands to a running recorder. `blick recorder --help` lists them.

### `blick play <files ...>`

Plays MCAP files of `blick record` one after another. Files that are not MCAP are played as videos or image
sequences, one stream each, on one clock.

| Option | Meaning |
| --- | --- |
| `--loop` | start again at the end |
| `--rate R` | a multiple of the recorded pace |
| `--pace wall\|step\|clock` | what drives the playback. `--clock KEY` names the clock for `--pace clock`. |
| `--prefix P` | publish below this prefix. The default is the recorded names. |
| `--key PATTERN` | only topics that match this pattern |
| `--cuda-device N` | also offer the frames in a CUDA ring on this GPU |
| `--dir DIR`, `--name NAME` | the directory and the name of the node, for its control topics |
| `--exit` | end when the playback has ended |
| `--report S` | seconds between the status lines |

Options for videos:

| Option | Meaning |
| --- | --- |
| `--stream BASE` | the stream of a video, for example `robot/rc_visard/left`. Repeat it, one per video. |
| `--camera CAMERA` | for one video: its stream is `<camera>/color` |
| `--camera-info FILE` | the ROS calibration file of a video, for its size |
| `--hfov DEG`, `--vfov DEG` | without `--camera-info`: a pinhole camera with this field of view |
| `--frame-id ID` | the frame id of a video |
| `--offset-ms MS` | move the stamps and send times of a video by this time |
| `--size WxH`, `--scale S` | scale the frames |
| `--start S`, `--end S` | play only this part of the videos |
| `--fps FPS` | take the frame times from this rate. The default is the times in the file. |

## GPU sharing

### `blick mps`

Runs NVIDIA's MPS control daemon for the nodes on the GPU of this PC. See
[sharing a GPU](../guides/sharing-a-gpu.md).

| Option | Meaning |
| --- | --- |
| `--pipe-dir DIR` | the directory of the daemon. Clients set `CUDA_MPS_PIPE_DIRECTORY` to it. |
| `--log-dir DIR` | the log of the daemon. The default is `<pipe-dir>/log`. |
| `--name NAME` | its name on Zenoh, `durchblick/mps/<name>`. The default is the host name. |

## Point clouds

### `durchblick-points`

The point cloud node, a C++ program with CUDA. See the [point cloud guide](../guides/point-clouds.md).

```bash
durchblick-points --depth robot/camera/depth --color robot/camera/color
```
