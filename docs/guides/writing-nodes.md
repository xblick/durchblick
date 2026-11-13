# Writing a node

A node is a program that reads streams and publishes results. This page lists what Durchblick gives every node,
shows a skeleton, and ends with a checklist for a release. If you have not written a node yet, start with
[your first node](../getting-started/first-node.md).

## What Durchblick gives a node

Every node needs logging, a clean stop, diagnostics and frames as tensors. Durchblick has a module for each, so
that your code contains only your method.

| Module | What you use | What it does |
| --- | --- | --- |
| `xblick.node` | `start(prog)` | It sets up the logging. It makes SIGTERM end the node as Ctrl-C does, through your cleanup. |
| | `Reporter` | It publishes your counts as diagnostics on `<camera>/diagnostics` about once a second. It writes a log line every ten seconds. |
| | `state()` | It returns the level and the message of a report. The level is OK, or WARN with the problems or with what the node waits for. |
| | `add_arguments(parser, ...)` | It adds `--max-hz` and `--local` to your parser. |
| `xblick.logs` | `setup(prog)` | `start` calls it. Durchblick's and your messages go to stdout. It hides the deprecation warnings of libraries. The log states which publisher a stream found and through which transport. |
| `xblick.history` | `FrameHistory` | A thread of its own takes every frame of a subscriber and keeps the last ones by stamp. Several histories can share one lock. |
| `xblick.tensors` | `rgb`, `depth`, `depth_metres` | They return a frame as a torch tensor on a device. They copy only where a copy is needed. `rgb` accepts `rgb8`, `rgba8`, `bgr8`, `bgra8` and `mono8`. `depth_metres` accepts `16UC1` (mm) and `32FC1` (m). |
| `xblick.rgbd` | `RGBDInputs`, `FramePreparer` | They pair colour, depth and a segmenter's results of one camera by stamp and prepare them on the GPU. See [pairing streams](pairing-streams.md). |
| `xblick.camera` | `check_local` | It stops a node that was told to keep frames on the GPU (`--local cuda`) when the publisher on its host offers no CUDA ring. |
| `xblick.parameters` | `node.Parameters`, `declare_options` | Your options become ROS 2 parameters with type, range, step, unit, choices and a read-only flag. It adds `--param NAME:=VALUE`, `--params-file`, `--dump-params` and `--params-read-only`. Other programs change the live parameters over Zenoh. The module checks each change and passes it to your writer. See [options as parameters](#options-as-parameters). |
| `xblick.poses` | `CameraPoses`, `RobotPoses` | They give a camera's or a robot's poses at an image's stamp. The module also has rotations, quaternions, `transform_of` and `pose_of`. |

A subscriber keeps only the newest frame. [Latest value](../concepts/latest-value.md) explains why. A node that
looks a frame up by its stamp later therefore needs a `FrameHistory`.

## A skeleton

The skeleton reads one colour stream, counts the frames and reports once a second. The camera name is an example.

```python
import threading

import xblick as db
from xblick import node, tensors
from xblick.history import FrameHistory

camera = "robot/camera"                            # an example
node.start("xblick-mynode")                              # before you load models: their libraries log while loading
session = db.open_session()                              # this PC's router
wake = threading.Event()
report = node.Reporter(session, camera, "mynode", "xblick-mynode")
color = FrameHistory(db.ImageSubscriber(session, f"{camera}/color", transport="auto", camera_info=True), 10, wake.set)
try:
    while True:
        wake.wait(0.05)
        wake.clear()
        color.check()                                    # raises a reader's error, e.g. another protocol version
        stamp, frame = color.newest() or (None, None)
        if frame is not None:
            rgb = tensors.rgb(frame, "cuda")             # (H, W, 3) uint8
            ...                                          # your method
            report.counts["frames"] += 1
        if report.due():
            done, seconds = report.take()                # the counts since the last report
            level, message = node.state(f"{done['frames'] / seconds:.1f} frames/s",
                                        waiting=None if done["frames"] else color.subscriber.describe())
            report.publish(level, message, {"frames/s": done["frames"] / seconds})
except KeyboardInterrupt:                                # Ctrl-C or SIGTERM
    pass
finally:
    color.close()                                        # closes its subscriber too
    report.close()
```

Three details matter in this skeleton.

- `ImageSubscriber` uses the transport `compressed` by default. Ask for `transport="auto"` to read a host ring or
  a CUDA ring on the publisher's host. [Transports](../concepts/transports.md) explains the choice.
- `color.check()` raises in your thread the error that ended the reader's thread.
- `node.state` returns WARN while no frame arrives. The message then says what the subscriber waits for.

## Options as parameters

Declare your node's options as parameters, so that the terminal interface and `xblick param` show how the node
runs. Do it right after `parse_args`, before you load a model, because a value given with `--param` or a params file
is written back into `args` there.

```python
node.Parameters.add_arguments(parser)                    # --param, --params-file, --dump-params, --params-read-only
args = parser.parse_args()
session = db_session.open_from_args(args)                 # before the model: the parameters need the session
params = node.Parameters(session, "mynode", args.camera)  # the node <camera>/mynode
params.configure(args)
params.declare_options(parser, args, live={"threshold": {"range": (0.0, 1.0, 0.01)}}, leave_out=("camera",))
params.check_overrides()
...
if score >= params["threshold"]:                          # a live parameter, read where it applies
```

- Make live what is safe to change between two frames: thresholds, limits, prompt words and object lists. Keep
  read-only what sets up a model, an engine or a buffer.
- Read a live parameter in the thread that applies it, or set the component's attribute at the top of your loop.
  Zenoh's thread calls a `write` function, so use one only to check a value, for example that an object's mesh
  exists.
- A node that has a key for changes at run time (`<node>/objects`) sets the parameter after each change there, so
  that both say the same.

[Parameters](../reference/parameters.md#a-nodes-options-as-parameters) has the rules, and its C++ section shows
`xblick::Options`.

## Restarts and changing cameras

Nodes, cameras and routers start and stop in any order. You do not write code for this, but you should know what
the readers do.

- A reader waits for its publisher. It finds the publisher again after a restart.
- A ring disappears with its publisher. The reader counts a frame from such a ring as stale and continues.
- A camera that returns with another resolution or calibration sends its new `camera_info` first. The nodes skip
  frames whose size does not match it. They count these frames in their diagnostics. They use the new camera model
  from the next frame on.
- You can set a publisher's `camera_info` and `frame_id` at any time. Its rings grow when a larger frame arrives.
  A node therefore keeps its output streams across such a change.

We tested this with all nodes in the simulation on 2026-11-01. We started each node before and after its inputs,
and we restarted each node while its readers ran.

## Frames on this host (`--local`)

A publisher on the node's host offers a host ring and possibly a CUDA ring. `--local` selects which one the node
reads.

```python
from xblick import camera, node

node.add_arguments(parser, local=camera.LOCAL)           # --local {auto,shm,cuda}, default auto
args = parser.parse_args()
node.start("xblick-mynode", local=args.local)            # logs "xblick-mynode: frames on this host: auto"
```

| Value | The node reads |
| --- | --- |
| `auto` | the publisher's CUDA ring when that ring is on the node's GPU, otherwise the host ring |
| `shm` | the host ring |
| `cuda` | the CUDA ring. `camera.check_local` stops the node when the publisher offers none. |

The node's GPU is the `device` that you give its subscribers, for example the value of your `--device cuda:0`
option. Durchblick compares the two GPUs by their UUID.

A node that works on the CPU with NumPy or OpenCV passes `local=("auto", "shm")` and gives its subscribers no
device. The parser then refuses `--local cuda` and states the reason. `auto` selects the host ring.

`add_arguments` adds `--local` only when you pass `local`. A node that already has its own `--local` option
therefore does not clash with it.

The start log states each stream's transport and the reason, as soon as the subscriber has chosen. Every report
repeats it as the value `transport <stream>` for the subscriptions that you gave `Reporter.watch`.
[Transports](../concepts/transports.md) explains the transports.

## Before you release a node

1. **Stamp each result with the stamp of its input.** Pass the capture stamp of the frame that you computed the
   result from. A reader can then match the result with the image and with other nodes' results.
   [Timestamps](../concepts/timestamps.md) lists the rule for each kind of input.
2. **Name the CUDA stream when you publish a device array.** Call `publish(tensor, stream=torch.cuda.current_stream())`.
   Without the name, `publish` waits for all work of the process on that GPU. In one measurement this wait cost 7 ms,
   and a named CUDA stream cost 0.1 ms. [Frames on the GPU](../concepts/gpu-frames.md) has the details.
3. **Offer the options as parameters,** with the live ones named in the README ([above](#options-as-parameters)).
4. **Report diagnostics.** Use `Reporter` and count what you skip. A reader of `<camera>/diagnostics` must see
   whether the node works, waits or drops frames.
5. **Fail fast.** Raise with a message that says what to do when the configuration is wrong or an input is missing.
   Do not add a default that hides the mistake. Call `color.check()` in your loop and `camera.check_local` after the
   first frame.
6. **Limit the rate by stamp and pair as a pair.** Offer `--max-hz` through `node.add_arguments`, and limit only the
   primary stream. [Pairing streams](pairing-streams.md) shows how.
7. **Check for the MPS daemon if the node shares a GPU.** Call `xblick.mps.check()` before the first CUDA call.
   [Sharing a GPU](sharing-a-gpu.md) explains why.
8. **Start and stop in any order.** Start the node before its camera, restart the camera, and stop the node with
   SIGTERM. The node must end through its `finally` block.
9. **Measure before you optimise.** [Making a node fast](making-a-node-fast.md) describes the method.
