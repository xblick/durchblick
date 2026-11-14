# Live cameras

`xblick cam` publishes a live camera that FFmpeg can open. This covers a webcam or a capture card as a V4L2 device
and an IP camera by its RTSP or HTTP URL. The stream has the same topics as the stream of a camera node, so every
node and tool works on it.

The command needs the extra `xblick[cam]`. It contains PyAV with FFmpeg inside its wheels, so no system FFmpeg is
needed. To play a video file as a camera, use [`xblick play`](video.md).

## Quick start

```bash
pip install "xblick[cam]"
xblick cam /dev/video0        # untested: no V4L2 device was at hand
```

The webcam is now the stream `cam/video0/color`. Once the first frame is in, the command prints the stream and the
camera that a node takes, each on a line of its own:

```
xblick cam: publishing the stream cam/video0/color
xblick cam: start a node with --camera cam/video0 (the camera, not the stream)
```

Start any node with that `--camera`. Nodes take the camera explicitly. A node that is given the stream instead ends
with one line that names the camera. When the session reached no Zenoh router
and no other session, the command prints a hint once at the start: `nobody can receive this stream yet`, and how
to start the router. It publishes anyway.

```bash
# an IP camera, asked for 1280 x 720 at 25 frames per second, as door/camera/color (untested: no IP camera)
xblick cam rtsp://192.0.2.20/stream1 --stream door/camera/color --size 1280x720 --fps 25

# FFmpeg's test pattern as a camera, without any device: cam/camera/color
xblick cam "testsrc=size=640x480:rate=30,realtime" --format lavfi
```

## Stream names

Without `--stream`, the stream is named after the device node: `/dev/video0` becomes `cam/video0/color`. A URL or a
source with `--format` has no device node and becomes `cam/camera/color`. A grey camera publishes on `.../mono`
instead of `.../color`. `xblick cam` names the stream at the first frame, when it knows the format, and writes the
name to its log. The camera, for example `cam/video0`, is the node of its parameters and diagnostics. `--stream`
replaces the default.

FFmpeg's filter `realtime` makes the test pattern deliver at its rate, as a camera does. Without it, the filter
graph delivers frames as fast as it can.

## Choose a camera

Leave out the source to choose one of the cameras of this PC:

```bash
xblick cam
```

`xblick cam` lists the V4L2 devices that capture video, each with its path and name. Move with the arrow keys and
press Enter to start the marked camera. Esc, `q` or Ctrl-C leave without starting anything, with exit code 1.

```
xblick cam: choose a camera (arrow keys, Enter; Esc or q: none)
> /dev/video0  Integrated Camera: Integrated C
  /dev/video2  USB Capture HDMI
```

A webcam usually has two device nodes, one for video and one for metadata. The list leaves out the metadata node.
The kernel tells the two apart in the capabilities of each node (`VIDIOC_QUERYCAP`). A node that you may not open
stays in the list with the note `capabilities unknown: Permission denied`. Add your user to the group `video` to
open it.

`xblick cam` checks a given `--stream` before it shows the list. Without a terminal, for example in a script or a
service, it shows no list. It ends with an error that names the cameras, and you give the source.

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

Without `--reconnect`, `xblick cam` ends with an error and the exit code 1 in these cases:

| Case | Message ends with |
| --- | --- |
| the camera delivers no frame for `--timeout` seconds (default 5), for example after it was unplugged | `no frame for 5 s (waiting for a frame; the camera stopped, or raise --timeout)` |
| the stream ends | `the stream ended` |
| FFmpeg reports an error while reading | `the camera stopped delivering (<FFmpeg's error>)` |
| the camera changes its frame size | `the camera changed its frame size from <old> to <new>` |

With `--reconnect [SECONDS]`, the command opens the camera again in these cases, every SECONDS (default 2) until
it delivers. It also tries the first opening again, because a robot or a PC may start before its IP camera. With
`--reconnect`, a V4L2 device that does not exist yet is waited for as well.

```bash
xblick cam rtsp://192.0.2.20/stream1 --reconnect 5        # untested: no IP camera was at hand
```

- Each loss is one log line, for example `lost (the stream ended); trying again every 5 s`. Each return is one log
  line, for example `back after 7.3 s`.
- While the camera is away, the diagnostics status `cam` has the level WARN and the message `camera away for 7 s
  (<why>), trying again every 5 s`. The value `losses` counts the losses.
- The camera must come back with the mode of its first frame: the same size and the same format. The stream's
  subscribers and its ring were set up for that mode, so another mode ends the command with an error that names
  both.
- A mode that differs from `--size`, `--fps` or `--pixel-format` ends the command also with `--reconnect`.
- `xblick cam` opens the camera again only after the previous opening has ended, because a V4L2 device cannot be
  opened twice. For a URL, FFmpeg ends a hung connection or read after `--timeout` seconds. A V4L2 device that hangs
  without an error keeps the command waiting until it returns.

Ctrl-C or SIGTERM ends the command within a second, also while FFmpeg waits for the camera and while the command
waits to open it again. The ring of the stream is removed from `/dev/shm`.

## MJPEG cameras

Many USB cameras deliver MJPEG: each frame is a JPEG file. By default, `xblick cam` decodes the frame and encodes
the decoded image again for the transport `compressed`. With `--mjpeg passthrough`, the transport `compressed`
carries the camera's own JPEG unchanged. The rings still get the decoded frame. This saves the second encoding and
its loss of quality.

```bash
xblick cam /dev/video0 --pixel-format mjpeg --mjpeg passthrough      # untested: no camera was at hand
```

- Many cameras leave the Huffman tables out of their frames, as the USB video class allows. Such a frame uses the
  standard tables of the JPEG standard (ITU-T T.81, Annex K.3). `xblick cam` inserts these tables before the
  frame's scan, as FFmpeg's bitstream filter `mjpeg2jpeg` does. It changes nothing else of the frame.
- A camera that does not deliver MJPEG ends the command with an error. A frame that is no JPEG ends it too, because
  the command never switches to encoding by itself.
- The stream keeps its encoding, for example `rgb8`. Each message's format names JPEG, as for an encoded frame.

`passthrough` is not the default, because it was not tested with a real camera. These receivers were tested with
4:2:2 frames that a test camera sent without Huffman tables:

| Receiver | Result |
| --- | --- |
| Python subscriber (Pillow, libjpeg-turbo) | decodes |
| C++ subscriber (libjpeg) | decodes, the same pixels as Pillow in 10 of 10 frames |
| xblick-relay with `--cpu` | decodes, the same pixels as Pillow |
| xblick-relay with nvJPEG | decodes; pixels differ from Pillow's by a mean of about 1 level, as its chroma upsampling differs |

Frames with restart markers were not tested.

## FFmpeg options

`--option KEY=VALUE` passes an input option to FFmpeg. Repeat it for more options. Use it for what `xblick cam` has
no option of its own for. These three options are useful for an RTSP camera. They exist in the FFmpeg of PyAV's
wheel (PyAV 19.0.2, FFmpeg's libavformat 63).

| Option | Meaning |
| --- | --- |
| `rtsp_transport=tcp` | RTSP over TCP, for a network that loses UDP packets |
| `timeout=5000000` | the RTSP socket's timeout, in microseconds |
| `fflags=nobuffer` | less buffering in FFmpeg, for a lower latency |

```bash
xblick cam rtsp://192.0.2.20/stream1 --option rtsp_transport=tcp --option fflags=nobuffer   # untested on a camera
```

`xblick cam` refuses an option that FFmpeg does not have where it applies options for this source. FFmpeg itself
would ignore such an option without a word. The command looks in FFmpeg's generic input options, in the options of
the input format (`--format`, or the URL's scheme such as `rtsp`) and in the options of the URL's protocol (such as
`http`). For an HTTP URL, FFmpeg detects the format from the data, so an option of that format needs `--format`.
The options `video_size`, `framerate` and `input_format` are given with `--size`, `--fps` and `--pixel-format`.

## Parameters and diagnostics

The options of `xblick cam` are read-only [parameters](../reference/parameters.md) of the node `<camera>`, for
example `cam/video0`. They apply at the next start. `--param NAME:=VALUE` and `--params-file FILE` set them at the
start, as for every node.

```bash
xblick param --node cam/video0
```

`<camera>/diagnostics` contains the status `cam` once per second.

| Value | Meaning |
| --- | --- |
| `frames/s` | the frames published in the last second |
| `mode` | the delivered mode, for example `640x480 mjpeg at 30 frames/s` |
| `decode ms` | the mean time from the arrival of a frame's data to the decoded image |
| `losses` | how often the camera was lost (with `--reconnect`) |

## Status and limits

- Tested on 2026-11-13 with FFmpeg's `lavfi` test pattern and with an HTTP URL that served a Matroska file with
  MJPEG frames. No V4L2 device, RTSP camera or other IP camera was tested.
- The list of cameras was tested in a pseudo-terminal with fake devices. Reading the capabilities was tested with
  sample data of a webcam. Neither was tested on a PC with a real camera.
- The command decodes the JPEG frames of an MJPEG camera and encodes them again for the transport `compressed`.
  It does not pass the camera's JPEG frames on unchanged.
- The command publishes the first video stream of a source.
- `--reconnect` was tested with an HTTP server that ended its stream, refused connections and came back, and with a
  file of another size after the return. It was not tested with an unplugged USB camera.
- `--mjpeg passthrough` was tested with a test camera over HTTP (FFmpeg's `mjpeg` format, 4:2:2 frames without
  Huffman tables) and the receivers in the table above. No real MJPEG camera was tested.
- `--option` refuses options of a protocol below another one (the `tcp` under `rtsp`) and FFmpeg's generic URL
  options such as `rw_timeout`, because FFmpeg's public API does not tell whether they exist.
