# Command line

The command `xblick` inspects, records and plays streams.
Without a subcommand it opens the [terminal interface](../guides/tui.md). `xblick <command> --help` prints every
option.

## Connection options

Every command accepts the same options for its Zenoh session.

| Option | Meaning |
| --- | --- |
| none | Connect to the router of this PC at `127.0.0.1:7447`. |
| `--connect <endpoint>` | Connect to another router or program. The option takes a Zenoh endpoint, such as `tcp/10.0.0.5:7447`. A host alone means `tcp/<host>:7447`, and `host:port` means `tcp/<host>:<port>`. |
| `--listen <endpoint>` | Listen on this endpoint. A peer without it listens on a free loopback port. |
| `--zenoh-mode peer\|client` | `peer` is the default. Use `client` for a program on another PC that only connects to a router. |
| `--config FILE` | A Zenoh JSON5 configuration. It replaces the other options. |

A router does not route data between peers. Peers reach each other directly, so a Durchblick peer always listens.
Without `--listen` it listens on a free loopback port. A program on another PC that only connects to a router
therefore uses `--zenoh-mode client`.

| Setup | Options |
| --- | --- |
| a router on every PC (the standard) | none. The routers connect with `xblick router --connect tcp/<other PC>:7447`. |
| a visiting program, for example a viewer on a laptop | `--connect tcp/<other PC>:7447 --zenoh-mode client` |
| a peer on a PC without a router, on purpose | `--connect tcp/<other PC>:7447 --listen tcp/0.0.0.0:0`. Every program on the network can then connect to it. |
| two programs without a router | `--listen tcp/127.0.0.1:<port>` on one, `--connect tcp/127.0.0.1:<port>` on the other |

A peer that connects only to other PCs and listens on loopback only prints one line at the start: the programs there
usually cannot reach it. The line names the three ways out with the given endpoints. The messages for an empty
result (`no stream answered`, `nothing received`, `no publisher answers`) then name the same cause. The C++ programs
`xblick-cpp-camera`, `xblick-cpp-probe` and `xblick-points` print the same line and accept `--zenoh-mode`.
[Sessions and routers](../concepts/sessions.md) explains the background.

## Router

### `xblick router`

Runs a Zenoh router from the Zenoh library in the package, until Ctrl-C or SIGTERM. Every PC needs one router; this
one replaces `zenohd`. Nothing starts it on its own. It has no plugins (REST, storage) and reads no Zenoh
configuration file. A port that is taken ends it with one line: a router may run there already. On a loopback
address (`--listen tcp/127.0.0.1:PORT`) the start line says that other PCs cannot connect.

| Option | Meaning |
| --- | --- |
| `--listen ENDPOINT ...` | where it listens. The default is `tcp/0.0.0.0:7447`, where every program looks for the router of its PC. |
| `--connect ENDPOINT ...` | the routers of other PCs (`zenohd`'s `-e`) |

## Inspect

### `xblick list [prefix]`

Lists every stream with its encoding or fields, its transports, its host and its Durchblick version. The command
queries the info records and subscribes to nothing.

| Option | Meaning |
| --- | --- |
| `prefix` | only keys below this prefix, for example `test` |
| `--seconds S` | how long to look for streams |
| `--traffic` | subscribe to every topic and show rates and sizes |

### `xblick hz <stream>`

Subscribes to an image or point cloud stream. Prints the rate, the age at send and at receive, and the missing
frames.

| Option | Meaning |
| --- | --- |
| `--transport auto\|compressed\|raw\|shm` | the transport to measure |
| `--seconds S` | stop after this time |

### `xblick snapshot <stream> <file>`

Saves one frame as an image file. PNG keeps 16-bit depth. Options: `--transport`, `--timeout`.

### `xblick diagnostics [prefix]`

Prints the state of every program that publishes for a camera, once per second.

| Option | Meaning |
| --- | --- |
| `prefix` | only cameras below this prefix |
| `--values` | every value of a status. The default prints level and message. |
| `--seconds S` | stop after this time |

### `xblick rerun [streams ...]`

Shows streams and node results in the Rerun viewer. Without arguments it shows everything that it finds. Needs the
extra `xblick[rerun]`. See the [viewer guide](../guides/viewer.md).

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

### `xblick view <streams ...>`

Shows streams in VLC (one window per stream) or in a browser (all streams side by side). It serves them as MJPEG over
HTTP on 127.0.0.1:8090. Without VLC it asks on a terminal whether to open the browser instead. On a system without a
display (Linux: neither `DISPLAY` nor `WAYLAND_DISPLAY` set) it starts no player and prints how to open the addresses
from another PC. On a terminal, the key `v` opens the VLC windows that are not open (any more) and `b` the browser
view. See the [view guide](../guides/view.md).

| Option | Meaning |
| --- | --- |
| `--browser` | open the page with all streams in the system's browser instead of VLC |
| `--no-player` | start no player; print the addresses only |
| `--http ADDRESS:PORT` | where the HTTP server listens. The default is `127.0.0.1:8090`, this PC only. `0.0.0.0:8090` opens it to the network **without a password**. |
| `--seconds S` | how long to wait for each stream's publisher (default 2 s) |

## Cameras

### `xblick cam [source]`

Publishes a live camera that FFmpeg opens: a V4L2 device such as `/dev/video0`, or an RTSP or HTTP URL. Needs the
extra `xblick[cam]`. See the [guide](../guides/cam.md).

| Option | Meaning |
| --- | --- |
| `source` | `/dev/videoN`, a URL, or with `--format` another FFmpeg input. Without it, choose one of the V4L2 cameras of this PC in a list (arrow keys, Enter). |
| `--stream BASE` | the stream, `<camera>/<stream>`. The default is `cam/<device node>/color`, for example `cam/video0/color`, or `cam/camera/color` for a URL or a `--format` source. A grey camera gets `/mono` in place of `/color`. |
| `--format FORMAT` | FFmpeg's input format, for example `lavfi`. The default is `video4linux2` for `/dev/...` and the protocol for a URL. |
| `--size WxH`, `--fps FPS`, `--pixel-format FORMAT` | the mode to ask the camera for. The camera must deliver it. `--pixel-format` is for V4L2 only. |
| `--camera-info FILE` | a ROS calibration file of the delivered size |
| `--hfov DEG`, `--vfov DEG` | without `--camera-info`: the field of view of the assumed pinhole camera (default 60 degrees horizontal) |
| `--frame-id ID` | the frame id. The default is the stream with `_` in place of `/`. |
| `--timeout S` | end with an error when no frame comes for this long (default 5). With `--reconnect`: open the camera again. |
| `--reconnect [SECONDS]` | open a camera that stops or cannot be opened again every SECONDS (default 2), also at the start. It must come back with its first mode. |
| `--mjpeg reencode\|passthrough` | an MJPEG camera's frames on the transport `compressed`: the decoded frame encoded again (default), or the camera's own JPEG with the standard Huffman tables inserted where a frame has none |
| `--option KEY=VALUE` | an FFmpeg input option, for example `rtsp_transport=tcp` (repeat for more). An option FFmpeg does not have for this source is refused. |
| `--param`, `--params-file`, `--dump-params`, `--params-read-only` | the options as parameters of the node `<camera>` |

## Test

### `xblick test-camera`

Publishes a moving test pattern on `test/camera/color`.

| Option | Default |
| --- | --- |
| `--namespace`, `--camera`, `--stream` | `test`, `camera`, `color` |
| `--size W H` | 640 480 |
| `--fps` | 15 |
| `--vfov` | the vertical field of view in degrees for the camera info |
| `--frame-id` | the frame id of the images |

### `xblick-cpp-camera`, `xblick-cpp-probe`

The C++ counterparts: a test pattern publisher and a prober that prints JSON. `--jpeg-frames` makes the camera
pass JPEG frames through. `--points` tests point cloud streams. Both programs are part of the C++ build.

## Parameters

### `xblick param`

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
extra `xblick[record]`. Videos need `xblick[video]`.

### `xblick record`

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

### `xblick recorder`

Sends commands to a running recorder. `xblick recorder --help` lists them.

### `xblick play <files ...>`

Plays MCAP files of `xblick record` one after another. Files that are not MCAP are played as videos or image
sequences, one stream each, on one clock.

| Option | Meaning |
| --- | --- |
| `--loop` | start again at the end |
| `--rate R` | a multiple of the recorded pace |
| `--pace wall\|step\|clock` | what drives the playback. `--clock KEY` names the clock for `--pace clock`. |
| `--prefix P` | publish below this prefix. The default is the recorded names. |
| `--key KEY_EXPR` | only topics that match this key expression |
| `--cuda-device N` | also offer the frames in a CUDA ring on this GPU |
| `--dir DIR`, `--name NAME` | the directory and the name of the node, for its control topics |
| `--exit` | end when the playback has ended |
| `--report S` | seconds between the status lines |

Options for videos:

| Option | Meaning |
| --- | --- |
| `--stream BASE` | the stream of a video, for example `cam/rc_visard/left`. Repeat it, one per video. |
| `--camera CAMERA` | for one video: its stream is `<camera>/color` |
| `--camera-info FILE` | the ROS calibration file of a video, for its size |
| `--hfov DEG`, `--vfov DEG` | without `--camera-info`: a pinhole camera with this field of view |
| `--frame-id ID` | the frame id of a video |
| `--offset-ms MS` | move the stamps and send times of a video by this time |
| `--size WxH`, `--scale S` | scale the frames |
| `--start S`, `--end S` | play only this part of the videos |
| `--fps FPS` | take the frame times from this rate. The default is the times in the file. |

## GPU sharing

### `xblick mps`

Runs NVIDIA's MPS control daemon for the nodes on the GPU of this PC. See
[sharing a GPU](../guides/sharing-a-gpu.md).

| Option | Meaning |
| --- | --- |
| `--pipe-dir DIR` | the directory of the daemon. Clients set `CUDA_MPS_PIPE_DIRECTORY` to it. |
| `--log-dir DIR` | the log of the daemon. The default is `<pipe-dir>/log`. |
| `--name NAME` | its name on Zenoh, `xblick/mps/<name>`. The default is the host name. |

## Relay

### `xblick relay [arguments]`

Starts the program `xblick-relay` with the arguments as they stand. The relay brings the streams that nodes on
this PC read from other PCs into the rings of this PC. See [relays](../concepts/transports.md#relays).
`xblick relay --help` prints the relay's options.

```bash
pip install "xblick[relay]"
xblick relay --stream test/camera/color
```

The command looks for the program in this order and nowhere else.

| Order | Place | Comes from |
| --- | --- | --- |
| 1 | the installed package `xblick-relay` | `pip install "xblick[relay]"` |
| 2 | `xblick-relay` on `PATH` | a CMake build of [xblick-relay](https://github.com/xblick/xblick-relay) |

If neither exists, the command fails and names both ways to install the relay. If the package is installed without
its program, the command fails as well. It does not try `PATH` then.

## Point clouds

### `xblick-points`

The point cloud node, a C++ program with CUDA. See the [point cloud guide](../guides/point-clouds.md).

```bash
xblick-points --depth test/camera/depth --color test/camera/color
```
