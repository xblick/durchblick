# Rings

A ring is the place where a publisher stores frames for the readers on its host. This page explains why Durchblick
uses rings, how a ring works, and what a reader must do with a frame from a ring.

## The problem: one copy per reader

A 1080p colour frame has 6 MB. A publisher that sends the frame over a network connection copies these 6 MB once
for every subscriber. With 16 subscribers on the same host, the publisher spends 26 ms of CPU time per frame on
Zenoh over TCP. At 30 frames per second this is most of one CPU core, and the frames only move from one process to
the next on the same machine.

## The idea: write once, tell everyone where

The publisher writes each frame once into memory that all processes on the host can open. It then sends a small
message, the *descriptor*, which states where the frame is. A reader receives the descriptor and reads the frame in
place.

```mermaid
flowchart LR
  P[Publisher] -- "1. writes the frame" --> R[("Ring: 32 slots")]
  P -- "2. sends a descriptor (slot 7, counter 41)" --> A[Reader A]
  P -- "2." --> B[Reader B]
  A -- "3. reads slot 7" --> R
  B -- "3. reads slot 7" --> R
```

The publisher now needs about 1 ms of CPU time per 1080p frame for 16 readers. An additional reader adds no work
for the publisher.

| 1080p to 16 readers on one host | Publisher CPU per frame |
| --- | --- |
| Zenoh over TCP | 26 ms |
| Ring, readers copy the frame | 1.2 ms |
| Ring, readers use the frame in place | 0.9 ms |

Measured on 2026-10-26 on an AMD EPYC 9335.

## How a ring works

A ring is a fixed set of slots which the publisher fills in turn. You can picture a carousel with 32 trays. The
publisher puts each new frame on the next tray and sends the tray number to the readers. A reader looks at the frame
and leaves it on the tray. After 32 frames the publisher reuses the first slot.

The publisher never waits for a reader. A slow reader can therefore find that the publisher has already replaced the
frame it wanted. Each slot has a counter for this case, a *sequence lock*. The publisher increments the counter when
it starts to write and again when it has finished. A reader compares the counter with the value in the descriptor
before and after it reads. If the counter changed, the reader discards the frame and counts it as `stale`. A reader
therefore never returns a frame that mixes two images.

An image ring has 32 slots by default, so a reader has 31 frame periods to read a frame. At 30 Hz this is about one
second. A point cloud ring has 4 slots, so a reader has 3 frame periods.

A ring grows when the publisher writes a larger frame. A camera that restarts with a higher resolution keeps its
ring and its readers.

## Two kinds of ring

| Ring | Memory | A reader gets | Transport name |
| --- | --- | --- | --- |
| Host ring | POSIX shared memory, `/dev/shm/durchblick-…` | a NumPy array | `shm` |
| CUDA ring | GPU memory that several processes map | a device array on the GPU | `cuda` |

Both kinds use the same slots, counters and descriptors. [Frames on the GPU](gpu-frames.md) explains what is
different for the CUDA ring.

A ring exists on one host. The topic of its descriptors therefore ends with the id of that host, and a reader
subscribes to the topic of its own host. A descriptor never travels to another machine.

## Copy or view

A reader can take a frame from a host ring in two ways.

**Copy (the default).** The reader copies the frame out of the slot and checks the counter. The frame is then yours
and stays valid.

```python
images = db.ImageSubscriber(session, "demo/camera/color", transport="shm")
frame = images.latest(timeout=1.0)
```

**View (`copy=False`).** The reader returns an array that points into the slot. You save the copy, and you must
check afterwards that the publisher did not replace the frame while you worked on it.

```python
views = db.ImageSubscriber(session, "demo/camera/color", transport="shm", copy=False)
frame = views.latest(timeout=1.0)
result = process(frame.image)
if not frame.is_valid():      # the publisher replaced the slot meanwhile
    result = None             # discard the result
```

Use views for short work on large frames. Use copies when your work on one frame can take longer than 31 frame
periods.

## What follows for your node

- Read a frame soon after its descriptor arrives. A subscriber does this for you when you call `latest()`.
- A node that pairs two streams reads the partner frame when it takes the primary frame. The
  [pairing guide](../guides/pairing-streams.md) shows the classes for this.
- A stale frame is normal under load. A reader reports the count in its statistics, and the node goes on.
- A ring drops frames even when the publisher is a [blocking publisher](latest-value.md#blocking-publishers).

## Related pages

- [Transports](transports.md): how a subscriber chooses between a ring and the network.
- [Frames on the GPU](gpu-frames.md): the CUDA ring.
- `cpp/include/durchblick/ring.hpp`: the layout of a host ring and the sequence lock.
