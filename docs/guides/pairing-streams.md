# Pairing streams and limiting the rate

A node that combines several streams of a camera must use frames of the same instant. This guide explains how to
pair streams by stamp, how to limit the rate of paired streams, and which classes do both for you.

## Why you pair by stamp

A camera delivers colour and depth as two streams. The frames of one exposure carry the same stamp, or stamps a few
milliseconds apart. They arrive at your node at different times, because depth and colour take different paths.

```
time →      0 ms          33 ms         66 ms
colour      C0            C1            C2
depth          D0            D1            D2
```

If your node takes the newest frame of each stream, it can combine C2 with D1. A moving object is then in two
different places in the two images. Pair frames by their stamp. Do not pair them by their arrival.

A subscriber keeps only the newest frame ([latest value](../concepts/latest-value.md)). Pairing needs the recent
frames of each stream, so Durchblick has a class for this.

## Keep recent frames: `FrameHistory`

`xblick.history.FrameHistory` reads every frame of a subscriber in a thread of its own. It keeps the last
frames by stamp.

```python
color = FrameHistory(db.ImageSubscriber(session, f"{camera}/color", transport="auto"), 10, wake.set)
stamp, frame = color.newest()
```

Several histories can share one lock, so that you read a consistent set of them.

## Colour, depth and masks together: `xblick.rgbd`

Most nodes for object poses need the same inputs: colour, depth, and optionally the instances and detections of a
segmenter. `xblick.rgbd` pairs them by stamp and prepares them on the GPU. It needs torch and OpenCV, which
Durchblick does not install. Your node brings them.

```python
import threading
from xblick.rgbd import CameraSpec, FramePreparer, RGBDInputs

spec = CameraSpec.parse("robot/rc_visard:color=left:depth=ffs/depth", segmenter="sam3")
wake = threading.Event()
inputs = RGBDInputs(session, spec, history=10, local="shm", wake=wake)
prepare = FramePreparer("cuda")

wake.wait()
stamp = inputs.newest_complete()                     # the newest stamp that all streams have, or None
frame = prepare(inputs, stamp) if stamp else None    # an RGBDFrame
```

**`CameraSpec`** describes the inputs in one string: `CAMERA[:color=NAME][:depth=NAME][:segmenter=NAME]`. The
parts are separated by colons, because some process managers split arguments at commas.

**`RGBDInputs`** reads all streams in threads of their own. `newest_complete()` returns the newest stamp for which
every stream has a frame.

**`FramePreparer`** returns an `RGBDFrame` with `rgb`, `depth` in metres, `ids`, `K`, `frame_id` and `detections`.
It brings all images to the resolution of the depth image. It removes the lens distortion, so the images fit the
pinhole camera of the matrix P in the colour camera's `CameraInfo`.

To map a per-pixel output of your node back into the camera's own image, with its size and its lens distortion,
use `Distorter`:

```python
labels_in_camera = Distorter(color_info, prepare.undistorter.K, *frame.depth.shape, "cuda").apply(labels)
```

### Collect views of an object

Mesh makers and NeMO need a few views of an object from different sides. `collect_views` gathers them while the
camera moves. It takes views that are at least `min_angle` degrees apart as seen from the object. The mask comes
from the segmenter, selected by label or by instance id.

```python
from xblick.rgbd import collect_views
views = collect_views(session, spec, 4, "cuda", label="mug", min_angle=15.0, timeout=60.0)   # a list of View
```

## Limit the rate: `max_hz`

A vision node often needs fewer frames than the camera delivers. With `max_hz`, a subscriber hands out at most that
many frames per second. It always hands out the newest frame, and it skips the others before it decodes them or
reads them from their ring. It counts them in `skipped_max_hz`.

| Where | How |
| --- | --- |
| Python | the argument `max_hz` of `ImageSubscriber`, `PointCloudSubscriber` and `RGBDInputs` |
| C++ | `SubscriberOptions::max_hz`, `CloudSubscriberOptions::max_hz` |
| A node's command line | `--max-hz HZ`. `--max-hz 0` or `--max-hz none` means every frame. |

**The subscriber chooses frames by their stamps.** It divides time into periods of 1/max_hz seconds, counted from
the Unix epoch. It takes a frame when the stamp lies in a later period than the stamp of the last frame taken. Two
streams of one camera share their stamps, so they select the same frames. A stamp that goes back, as after a
restart of the camera, is taken.

### Limit paired streams as a pair

Do not give each stream of a pair its own limit. Colour and depth of one exposure can have stamps a few
milliseconds apart. When a period boundary falls between the two stamps, one stream takes its frame and the other
skips its frame, and the pair is lost.

We simulated a 30 Hz camera whose colour and depth stamps lie 15 ms apart. One robot camera has 11-17 ms.

| Limit | Pairs lost with separate limits |
| --- | --- |
| 20 Hz | 26 % |
| 30 Hz | none, because every period contains one frame of each stream |

The solution has two roles.

- The **primary** stream has the `max_hz`. It is the colour image, or the stream that drives your node.
- A **partner** stream, such as depth or a mask, keeps its samples undecoded. It decodes only the sample that
  belongs to a primary frame that the node takes.

`history.PartnerHistory` implements the partner in Python. It returns the sample with the nearest stamp within a
maximum offset, once no nearer sample can arrive. Stamps only grow, so a nearer sample can still come only while the
nearest one lies before the primary stamp. A partner is steady when the steps between its kept stamps lie within a
tenth of their median. A steady partner sends its next sample about one step after the newest one. That sample is
nearer only if it comes more than half a step early. `PartnerHistory` therefore takes a sample at most a quarter of
the smallest step before the primary stamp at once. A camera of a stereo pair that stamps slightly earlier pairs
without a wait. A node that skips camera frames is not steady, because its next sample may be the very next camera
frame. For such a partner, and until the partner has two samples, `PartnerHistory` waits for a later sample. The rule
is the same for the default offset and for an explicit one.

In C++ the pieces are `SubscriberOptions::keep_recent`, `ImageSubscriber::recent_stamps` and `ImageSubscriber::at`.
`partner_of` in `xblick/pairing.hpp` chooses the stamp. `PartnerHistory` calls the same function, so Python and
C++ pair alike.

`RGBDInputs` works this way, with colour as the primary stream. Its `max_offset_ns` for the depth is None by
default, which pairs within half the depth's measured period. A value of 0 requires equal stamps. A segmenter's
instances always pair by equal stamps. The point cloud node works this way with depth as the primary stream.

**Counters.** The `overwritten` counter of a plain subscriber counts the frames that were replaced before you took
them. The `overwritten` counter of a partner stays 0. A partner keeps its samples for pairing and does not take
them, so a newer sample that replaces the newest one loses nothing.

### Add `--max-hz` to your node

```python
node.add_arguments(parser, max_hz=30.0)                  # --max-hz HZ, default 30
log = node.start("xblick-mynode", max_hz=args.max_hz)
inputs = RGBDInputs(session, spec, 30, args.local, wake, max_hz=args.max_hz)
report.watch(inputs)
```

- `node.add_arguments(parser, max_hz=...)` adds the option with your node's default. The vision nodes use about
  20-30 Hz. Camera nodes, the point cloud node, nvblox and cuVSLAM give no default and take every frame.
- `node.start(prog, max_hz=...)` writes the rate to the start log, for example `xblick-mynode: max 30 Hz` or
  `xblick-mynode: every frame`.
- `report.watch(...)` adds the values `max-hz` and `skipped for max-hz` to every diagnostics report.

## Related pages

- [Writing nodes](writing-nodes.md): everything else that a node needs.
- [Timestamps](../concepts/timestamps.md): what a stamp means and where it comes from.
- [Rings](../concepts/rings.md): how long a partner frame stays readable.
