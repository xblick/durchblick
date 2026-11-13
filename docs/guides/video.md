# Videos as camera streams

`xblick play` turns a video file or an image sequence into a camera stream. The stream has the same topics as
the stream of a camera node, so every node and tool works on it. You can therefore try a node without a camera
and without a recording.

Videos need the extra `xblick[video]`. It contains PyAV with FFmpeg inside its wheels, so no system FFmpeg is
needed.

## Quick start

```bash
xblick play clip.mp4 --camera robot/video --loop
```

The video is now the stream `robot/video/color`. Start any node on the camera `robot/video`.

```bash
# a stereo pair, with frames paired exactly
xblick play left.mp4 right.mp4 --stream robot/rc_visard/left --stream robot/rc_visard/right \
    --camera-info left.yaml --camera-info right.yaml --loop

# colour and depth from two files, the depth 3 ms later
xblick play color.mp4 depth.mkv --stream cam/color --stream cam/depth --offset-ms 0 --offset-ms 3

# an image sequence
xblick play 'frames/*.png' --camera robot/video --fps 10
```

## What counts as a video

`play` recognises an MCAP recording by its content. Every other file is a video:

- a file that FFmpeg reads, such as mp4, mkv, avi or webm,
- a directory of images, sorted by name,
- a glob such as `'frames/*.png'`.

`play` plays recordings one after another and videos at the same time. It refuses a mix of both.

| Source | Encoding of the stream |
| --- | --- |
| a colour video | `rgb8` |
| a grey video | `mono8` |
| a 16-bit grey video, for example FFV1 `gray16le` in Matroska | `16UC1`, which is depth in mm. The compressed transport uses RVL. |
| an image sequence | by the mode of the images: `I;16` gives `16UC1`, `L` gives `mono8`, all others `rgb8` |

An RGB-D camera can therefore be two files, one for colour and one for depth.

Each video stream offers `image_raw` with the transports `shm` and `compressed`, its `camera_info`, an info record
with the source `video`, and diagnostics. With `--cuda-device`, `play` uploads each decoded frame once and also
offers a CUDA ring.

## Options

**Per video.** Give each of these options once per video, in the order of the files, or leave it out. `play`
refuses a count that does not match.

| Option | Meaning |
| --- | --- |
| `--stream BASE` | the name of the stream, for example `robot/rc_visard/left`. Required. For a single video, `--camera CAM` means `CAM/color`. |
| `--offset-ms MS` | moves the stamps and the send times of the stream, forwards or backwards. Use it for videos that are not cut in sync. |
| `--camera-info FILE` | a ROS calibration file for the size of the stream, as `camera_calibration` writes it |
| `--frame-id ID` | the frame id. The default is the name of the stream with `_` in place of `/`. |

**For all videos.**

| Option | Meaning |
| --- | --- |
| `--size WxH`, `--scale F` | scale the frames |
| `--start S`, `--end S` | play only this part, in seconds of the videos |
| `--fps FPS` | take the frame times from this rate. Required for image sequences. |
| `--hfov DEG`, `--vfov DEG` | the field of view of the assumed camera |

Without `--camera-info`, `play` assumes a pinhole camera with a horizontal field of view of 60 degrees and the
principal point in the centre of the image. It writes `camera_info ASSUMED` to its log.

**Shared with recordings.** `--rate`, `--loop`, `--exit`, `--name`, `--report`, `--dir`, `--cuda-device` and the
connection options keep their meaning. The options `--key`, `--prefix`, `--clock` and every `--pace` other than
`wall` work only with recordings, and `play` refuses them for videos.

## Stamps

All videos of one playback use one clock. Each video has one thread for decoding and one for sending.

```
stamp = start of the playback (wall time) + (time of the frame in its video - --start) / --rate + --offset-ms
```

`play` sends each frame at its stamp. This has the following consequences.

- **Exact pairs.** Frames with equal time and equal offset receive the same stamp to the nanosecond in every
  stream. Stereo and RGB-D nodes pair them exactly, with a maximum offset of 0. With an `--offset-ms`, a node
  pairs them within its own maximum offset.
- **Different rates.** Each video keeps its frame rate. A colour video with 20 frames per second and a depth video
  with 10 pair at every second colour frame.
- **No dropped frames.** A frame that cannot go out at its stamp goes out late. The diagnostics count it.
- **Loops.** All streams start again together when the longest video has ended. A shorter video keeps its last
  frame until then. The stamps move forward by the length of the longest video, so they always grow.
- **Pauses.** A pause stops the clock. After it, the stamps continue from the wall time of the resume.

[Timestamps](../concepts/timestamps.md) explains what the stamps can be compared with.

## Control a playback

The play node offers its controls as [parameters](../reference/parameters.md) of the node `xblick/<name>`. The
default name is `xblick/recorder`. They work for videos and recordings alike.

| Parameter | Meaning |
| --- | --- |
| `rate` | a multiple of the pace of the files. The next frame is due at the new rate, counted from the last frame. |
| `paused` | `true` pauses and `false` resumes. Reading it returns the state. |
| `position_s` | the position in seconds of the files. Setting it seeks. A video continues from the key frame before the position and decodes up to the frame. A recording continues from that log time. 0 starts again. After the end, a set plays the files again. |
| `loop` | start again at the end. You can switch it during the playback. |
| `duration_s`, `file` | read-only: the length of the files and the file that plays |

```bash
xblick param --node xblick/recorder --set paused:=true
```

After a seek, the stamps continue to grow, as after a loop. The frame that you seek to arrives one frame period
after the last frame sent. The node reports the values that the playback changes itself, such as the position, in
parameter events about once per second and at the end.

The terminal interface drives a playback through its Parameters tab and shows the position in its Nodes tab. The
commands `xblick recorder pause|resume|stop|status --name <name>` also work. Stepping with `--pace step` exists
only for recordings.

## Diagnostics

Each camera's `<camera>/diagnostics` contains the status `play`.

| Value | Meaning |
| --- | --- |
| frames per second, decode time, position | per stream |
| late frames | in total and in the last second. The status is WARN when frames went out late in the last second. |
| send skew | the spread between the streams of their mean difference "send time minus stamp", in ms. A stereo pair on one PC has about 0.05 ms. |

## Which nodes work on a video

State of the node READMEs on 2026-11-05.

| Node | Needs | On a plain colour video |
| --- | --- | --- |
| xblick-sam3 | colour with `camera_info` | yes (tested, see below) |
| xblick-insightface | colour | yes |
| xblick-vjepa2 | colour | yes |
| xblick-tapnext | colour, depth, and an xblick-sam3 on the stream | with `--no-depth` (2D only), and with SAM 3 or `--no-segmenter` (points by hand) |
| xblick-boxer | colour, depth (`--no-depth` guesses distances), and the pose of the camera on `<camera>/tf` | no. A video has no pose, so the node skips every frame ("no pose"). |
| xblick-gem-x | colour and depth, paired | no. Yes with a 16-bit depth video beside it. |
| xblick-foundationpose, xblick-nemo, xblick-any6d, xblick-recgen, xblick-sam3d-objects | colour and depth | no. Yes with a depth video. |
| xblick-fast-foundationstereo | a rectified stereo pair `<camera>/left` and `<camera>/right` | from two videos with their calibration files (tested, see below) |
| xblick-ivt | a rectified stereo pair as above | from two videos with their calibration files |
| xblick-mapping (cuVSLAM, nvblox) | stereo or RGB-D. nvblox also needs the pose of the camera. | not tested |

**A stereo pair from videos** consists of two rectified videos of the same length and rate, each with its
calibration file. The matrix P in the `camera_info` of the right camera carries the baseline: P[3] = -f·b, which is
the stereo convention of ROS.

## A complete example

Use one terminal for each step.

```bash
# 1. a router on this PC, if none is running
zenohd --no-multicast-scouting -l tcp/127.0.0.1:7447

# 2. the video as robot/video/color, looped
xblick play ~/clip.mp4 --camera robot/video --loop

# 3. nodes on the video, each from its own checkout and on its own GPU
cd xblick-sam3 && CUDA_VISIBLE_DEVICES=0 .venv/bin/xblick-sam3 --camera robot/video --words person cup
cd xblick-insightface && CUDA_VISIBLE_DEVICES=1 .venv/bin/xblick-insightface --camera robot/video

# 4. look at the streams, the reports of the nodes and the position of the playback
xblick
xblick recorder pause         # and: resume
```

Without a router, give every program the same options `--connect tcp/<host>:<port> --listen tcp/127.0.0.1:0`, and
connect the terminal interface with `c`.

## Tested

On 2026-11-05 on a server with a private router and two GPUs.

| Input | Node | Result |
| --- | --- | --- |
| NVIDIA's GEM-X demo video (1280 x 720, 30 frames per second, 7.2 s, looped) | xblick-sam3 with `--words person` | The colour stream had 30.0 Hz and an age at send of 0.6 ms. The instances of SAM 3 had 30.0 Hz and an age at send of 29.5 ms, which is its 29 ms per frame. |
| A stereo pair exported from a simulation recording (1280 x 720, 22 frames per second, 40 s) with calibration files | xblick-fast-foundationstereo with `--max-width 640` | The node paired every pair at 22.0 pairs per second. The depth had an age at send of 14.1 ms. The send skew of the two streams was 0.04 ms. |
