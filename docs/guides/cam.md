# Live cameras

`xblick cam` publishes a live camera that FFmpeg can open. This covers a webcam or a capture card as a V4L2 device
and an IP camera by its RTSP or HTTP URL. The stream has the same topics as the stream of a camera node, so every
node and tool works on it.

The command needs the extra `xblick[cam]`. It contains PyAV with FFmpeg inside its wheels, so no system FFmpeg is
needed. To play a video file as a camera, use [`xblick play`](video.md).

## Quick start

```bash
pip install "xblick[cam]"
xblick cam /dev/video0 --stream robot/webcam/color        # untested: no V4L2 device was at hand
```

The webcam is now the stream `robot/webcam/color`. Start any node on the camera `robot/webcam`.

```bash
# an IP camera, asked for 1280 x 720 at 25 frames per second (untested: no IP camera was at hand)
xblick cam rtsp://192.0.2.20/stream1 --stream robot/door/color --size 1280x720 --fps 25

# FFmpeg's test pattern as a camera, without any device
xblick cam "testsrc=size=640x480:rate=30,realtime" --format lavfi --stream robot/test/color
```

FFmpeg's filter `realtime` makes the test pattern deliver at its rate, as a camera does. Without it, the filter
graph delivers frames as fast as it can.

## Sources

| Source | How FFmpeg opens it |
| --- | --- |
| `/dev/video0`, any path below `/dev` | FFmpeg's V4L2 input (`video4linux2`) |
| `rtsp://`, `rtsps://`, `http://`, `https://`, `rtmp://`, `udp://`, `tcp://`, `srt://` | FFmpeg's protocol of the URL. FFmpeg takes the format from the stream. |
| anything with `--format FORMAT` | that FFmpeg input format, for example `lavfi` for a filter graph |

`xblick cam` refuses a file. `xblick play` plays files. It also refuses a device path that does not exist and names
the V4L2 devices of the PC.

## Size, rate and pixel format

Without options, the camera delivers its own mode. With options, `xblick cam` asks FFmpeg for a mode.

| Option | FFmpeg's option | Meaning |
| --- | --- | --- |
| `--size WxH` | `video_size` | the frame size |
| `--fps FPS` | `framerate` | the frame rate |
| `--pixel-format FORMAT` | `input_format` | V4L2 only: the format of the device, for example `mjpeg` or `yuyv422` |

A device may change the mode that it was asked for, and a URL or a filter graph ignores these options. `xblick cam`
therefore compares the first frame and the stream with what you asked for. It ends at a difference and names both:

```
xblick cam: testsrc=size=320x240:rate=30,realtime: asked for 640x480, the camera delivers 320x240 rgb24 at 30 frames/s
```

The rate of the stream is the rate that FFmpeg states for it. A camera that states no rate cannot be checked, so
`xblick cam` refuses `--fps` for it. To see the modes of a V4L2 device, run `v4l2-ctl --list-formats-ext -d
/dev/video0` from the package v4l-utils (untested here).

A colour camera becomes an `rgb8` stream, a grey camera `mono8`, and a 16-bit grey camera `16UC1`. The stream
offers `image_raw` with the transports `shm` and `compressed` (JPEG, or RVL for `16UC1`), its `camera_info`, an info
record with the source `cam`, and diagnostics.

## Camera info

Give the calibration of the camera with `--camera-info FILE`, a ROS calibration file as `camera_calibration` writes
it. Its size must be the delivered size, and `xblick cam` refuses another.

Without `--camera-info`, `xblick cam` assumes a pinhole camera, as `xblick play` does for a video. Its horizontal
field of view is 60 degrees, the principal point is the centre of the image, and the distortion is zero.
`--hfov DEG` or `--vfov DEG` sets another field of view. The command writes `camera_info ASSUMED` to its log. A node
that measures in 3D gives wrong results with an assumed calibration.

The frame id is `--frame-id`, by default the name of the stream with `_` in place of `/`.

## Stamps

The stamp of a frame is the time at which its data arrived from FFmpeg on this PC, read from the wall clock. The
camera exposed the frame earlier. The difference is the latency of the camera and its transport, which `xblick cam`
does not know.

Two such cameras therefore do not pair exactly. Their stamps come from two independent arrivals, and their latencies
differ. A stereo or RGB-D node pairs their frames only within a maximum offset. A pair then can hold frames that
were exposed several milliseconds apart. [Pairing streams](pairing-streams.md) explains the maximum offset, and
[timestamps](../concepts/timestamps.md) explains what a stamp can be compared with.

## When the camera stops

`xblick cam` ends with an error and the exit code 1 in these cases:

| Case | Message ends with |
| --- | --- |
| the camera delivers no frame for `--timeout` seconds (default 5), for example after it was unplugged | `no frame for 5 s (waiting for a frame; the camera stopped, or raise --timeout)` |
| the stream ends | `the stream ended` |
| FFmpeg reports an error while reading | `the camera stopped delivering (<FFmpeg's error>)` |
| the camera changes its frame size | `the camera changed its frame size from <old> to <new>` |

The command does not reconnect. Restart it with a process manager if the camera should come back.

Ctrl-C or SIGTERM ends the command within a second, also while FFmpeg waits for the camera. The ring of the stream
is removed from `/dev/shm`.

## Parameters and diagnostics

The options of `xblick cam` are read-only [parameters](../reference/parameters.md) of the node `<camera>`, for
example `robot/webcam`. They apply at the next start. `--param NAME:=VALUE` and `--params-file FILE` set them at the
start, as for every node.

```bash
xblick param --node robot/webcam
```

`<camera>/diagnostics` contains the status `cam` once per second.

| Value | Meaning |
| --- | --- |
| `frames/s` | the frames published in the last second |
| `mode` | the delivered mode, for example `640x480 mjpeg at 30 frames/s` |
| `decode ms` | the mean time from the arrival of a frame's data to the decoded image |

## Status and limits

- Tested on 2026-11-13 with FFmpeg's `lavfi` test pattern and with an HTTP URL that served a Matroska file with
  MJPEG frames. No V4L2 device, RTSP camera or other IP camera was tested.
- The command decodes the JPEG frames of an MJPEG camera and encodes them again for the transport `compressed`.
  It does not pass the camera's JPEG frames on unchanged.
- The command publishes the first video stream of a source. Further FFmpeg options, for example RTSP over TCP,
  cannot be given.
- A camera that stops ends the command. There is no reconnect.
