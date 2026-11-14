# Latest value

A Durchblick stream delivers the newest frame. It does not deliver every frame. This page explains the reason, what
it means for a node, and how you get every frame when you need it.

## Why a stream drops frames

A robot acts on what its cameras see now. If a consumer is slower than the camera and the middleware queues the
frames, the consumer works on older and older images. The latency grows until a queue is full, and then frames are
lost anyway.

Durchblick therefore keeps no queue. A stream behaves like a live broadcast: a viewer who looks away misses a scene
and then sees the present again.

- A publisher drops a sample when the network is congested.
- A subscriber keeps only the newest sample. It decodes this sample when you ask for a frame with `latest()`.
- A ring overwrites its oldest slot.

A slow consumer therefore sees fewer frames, and each frame is as fresh as possible. The subscriber counts the
frames that it replaced before you took them (`overwritten`). Every frame carries its capture time, so you can
measure its age. [Timestamps](timestamps.md) explains the ages.

## Decoding only what you take

A subscriber stores the newest sample as it arrived. It decompresses the JPEG, or reads the ring, only for the frame
that you take. A node that needs 50 ms per frame at a 30 Hz camera therefore decodes about 20 frames per second and
spends nothing on the other 10.

You can also set an upper limit. With `max_hz`, a subscriber hands out at most that many frames per second and
skips the others before it decodes them. Nodes offer this as `--max-hz`. The
[pairing guide](../guides/pairing-streams.md) explains how the limit works for streams that belong together.

## When you need every frame

Some tasks need more than the newest frame.

**Pairing streams by stamp.** A node that combines colour and depth needs the two frames of the same instant, and
the newest frame of each stream may not match. `xblick.history.FrameHistory` takes every frame of a subscriber
in its own thread and keeps the last ones by stamp. See the [pairing guide](../guides/pairing-streams.md).

**Recording.** A recording of a simulation must not lose a sample. For this case a publisher can block.

## Blocking publishers

A publisher created with `blocking=True` blocks when a subscriber is slow. It uses Zenoh's [congestion
control](https://docs.rs/zenoh/1.10.1/zenoh/qos/enum.CongestionControl.html) `BLOCK`, so a slow subscriber holds the
publisher up. Use it only where the producer can wait, for example in a simulation that produces data for a recording.

`CameraPublisher`, `RobotStatePublisher`, `TransformPublisher`, `SimPublisher` and `camera.CountedPublisher` accept
`blocking=True`.

A blocking publisher also answers a query on `<key>/sent` for each of its keys. The answer states how many samples
it has put, so that a recorder can check that it received all of them:

```json
{"v": 1, "key": "...", "sent": 1200, "seq": 1199, "matched": true}
```

`sent` is the number of samples put, `seq` is the sequence number of the last sample, and `matched` says whether
the publisher knows a subscriber. The [recording guide](../guides/recording.md) shows the check.

Blocking applies to the samples that travel over Zenoh. A ring still overwrites its oldest slot.

## Related pages

- [Rings](rings.md): how long a frame stays readable in a ring.
- [Transports](transports.md): what a publisher produces, and for whom.
