# Your first node

A node is a program that reads streams and publishes results about them. In this tutorial you write a node that
measures the brightness of a camera image. The method is trivial on purpose, so that you see what belongs to every
node. You need the test camera from [your first stream](first-stream.md).

## The complete node

Save this file as `brightness_node.py`.

```python
import argparse
import threading

import xblick as db
from xblick import node, session as db_session
from xblick.history import FrameHistory

parser = argparse.ArgumentParser()
parser.add_argument("--camera", default="test/camera")
parser.add_argument("--threshold", type=float, default=0.5, help="from this level on an image is bright")
db_session.add_arguments(parser)                      # --connect, --listen, --zenoh-mode, --config
node.Parameters.add_arguments(parser)                 # --param NAME:=VALUE, --params-file, ...
args = parser.parse_args()

node.start("brightness")                              # logging, and SIGTERM ends the node like Ctrl-C
session = db_session.open_from_args(args)
params = node.Parameters(session, "brightness", args.camera)   # the node <camera>/brightness
params.configure(args)
params.declare_options(parser, args, live={"threshold": {"range": (0.0, 1.0, 0.01)}}, leave_out=("camera",))
params.check_overrides()
wake = threading.Event()
color = FrameHistory(db.ImageSubscriber(session, f"{args.camera}/color", transport="auto"), 10, wake.set)
result = db.DetectionPublisher(session, f"{args.camera}/brightness/level", db.Classification)
report = node.Reporter(session, args.camera, "brightness", "brightness")
last = None
try:
    while True:
        wake.wait(0.05)
        wake.clear()
        color.check()                                 # raises an error of the reader thread here
        stamp, frame = color.newest() or (None, None)
        if frame is not None and stamp != last:
            last = stamp
            level = float(frame.image.mean()) / 255.0
            header = db.Header(db.Time.from_ns(stamp), frame.header.frame_id)   # the stamp of the input
            bright = level > params["threshold"]      # a live parameter: read where it applies
            result.publish(db.Classification(header, [db.ObjectHypothesis("bright" if bright else "dark", level)]))
            report.counts["frames"] += 1
        if report.due():
            done, seconds = report.take()
            rate = done["frames"] / seconds
            level_, message = node.state(f"{rate:.1f} frames/s",
                                         waiting=None if done["frames"] else color.subscriber.describe())
            report.publish(level_, message, {"frames/s": rate})
except KeyboardInterrupt:
    pass
finally:
    color.close()
    report.close()
    params.close()
    session.close()
```

## Run it

```bash
xblick test-camera &
python brightness_node.py
```

```
brightness: test/camera/color: publisher found on this host (test), reading shm (this subscriber works on the CPU)
brightness: 14.9 frames/s
```

The first line states which publisher the node found and which transport it chose. The second line appears every
ten seconds. In another terminal, `xblick diagnostics robot` shows the state of the node once per second.

## What each part does

**`node.start("brightness")`** sets up the logging. It also makes SIGTERM end the node in the same way as Ctrl-C,
so that the `finally` block always closes the node cleanly. Call it before you load a model, because model
libraries write log messages while they load.

**`FrameHistory`** reads the subscriber in a thread of its own and keeps the last 10 frames by stamp. It calls
`wake.set` when a frame arrives, so the main loop sleeps until there is work. `color.newest()` returns the newest
frame with its stamp.

**The result topic** is `<camera>/brightness/level`. A node publishes its results below the camera that it
reads. The node then works on any camera, and a viewer finds its results.
[Streams and topics](../concepts/streams-and-topics.md) explains the rule.

**The header of the result** carries the stamp of the input frame. This is the most important rule for a node. A
consumer can match your result with the image only if both have the same stamp.

**`Reporter`** publishes the state of the node on `<camera>/diagnostics` about once per second. `node.state()`
returns OK, or a warning with the reason. Here the node warns while it waits for its first frame, and the message
says what it waits for.

**`declare_options`** makes every option a parameter of the node `<camera>/brightness`. `threshold` is live: change
it while the node runs with `xblick param --node test/camera/brightness --set threshold:=0.3`, or in the
terminal interface (`xblick`, tab Parameters). The other options are read-only and apply at the next start.
[Parameters](../reference/parameters.md#a-nodes-options-as-parameters) explains the rules.

**`color.check()`** raises an error that happened in the reader thread, for example a publisher with another
protocol version. The node then stops with a clear message. A node never continues silently after an error.

## From here to a real node

A real node differs in three places.

| Topic | What changes | Read |
| --- | --- | --- |
| Frames on the GPU | Pass `device="cuda:0"` to the subscriber and convert the frame with `xblick.tensors.rgb(frame, "cuda")`. The frame then stays in GPU memory. | [Frames on the GPU](../concepts/gpu-frames.md) |
| Several inputs | Pair colour, depth and masks by stamp with `xblick.rgbd.RGBDInputs`. | [Pairing streams](../guides/pairing-streams.md) |
| Options and settings | Add `--max-hz` and `--local` with `node.add_arguments`, and choose which options are live. | [Writing nodes](../guides/writing-nodes.md#options-as-parameters), [Parameters](../reference/parameters.md) |

The [writing nodes guide](../guides/writing-nodes.md) ends with a checklist for a release. When your node works
and is too slow, read [making a node fast](../guides/making-a-node-fast.md).
