# Recording and playback

`xblick record` writes what cameras and nodes publish into MCAP files. `xblick play` publishes a recording
again on the same topics, with its pace and its stamps. Nodes work on a playback as they work on a robot, so a
recording repeats a situation for a node, a test or a comparison.

Both commands need the extra `xblick[record]`. MCAP is the storage format of rosbag2. The files open in
`ros2 bag`, Foxglove and Rerun.

## Quick start

```bash
xblick record --dir ~/recordings                 # records until Ctrl-C
xblick play ~/recordings/recorder_<time>.mcap --loop
xblick play ~/recordings/recorder_<time>.mcap --prefix replay   # beside the live streams
```

Record and play are one node. A running node accepts commands from anywhere:

```bash
xblick recorder stop                             # also: record, status
xblick recorder play recorder_<time>.mcap --loop --cuda-device 0
xblick recorder pause                            # also: resume
```

## What the recorder records

The recorder finds streams in the same way as `xblick rerun`, and it looks again every 5 s.

| Data | Form in the file |
| --- | --- |
| Image streams | the compressed form as the publisher sends it: JPEG for colour, lossless RVL or PNG for depth, PNG for label images. With the `camera_info`. |
| Detections | as sent |
| Point clouds | only with `--points`. They are large, and a point cloud node can compute them again from depth. |
| Robot state | `joint_states`, `tf`, `odom`, `wrench/*`, found by key pattern |
| Per camera | `tf`, `tf_static`, `imu`, `diagnostics`, bodies, and the `tf` and `odom` of a node |

The recorder leaves out streams without a compressed form, such as float disparity and feature maps. It asks for
`tf_static` at the start of every file.

It stores each message as it arrived. ROS messages are CDR with the definition of their type, and bodies are JSON.
Each channel keeps its Zenoh key, its encoding and the info record of its publisher. `play` can therefore serve the
channel in the same way as the original publisher.

**Where it runs.** The recorder reads only what travels over the network. It never reads a ring. It can therefore
run on any PC that reaches the routers. While it does not record, it subscribes to nothing, so no publisher encodes
for it.

**Size.** 30 s of a simulated scene with a stereo node and NeMO took 86 MB, which is about 10 GB per hour.

**Files.** Every `record` command opens a new file `<dir>/<name>_<UTC time>.mcap`. The recorder starts the next
file every 10 minutes (`--split-minutes`), so a crash loses little. On SIGTERM or Ctrl-C it completes the file.

## Control over Zenoh

The node has the name `recorder` by default (`--name`).

| Key | Role |
| --- | --- |
| `xblick/<name>/control` | accepts the commands `record`, `stop`, `play`, `pause`, `resume` and `step` |
| `xblick/<name>/status` | publishes the state every second and answers queries. States: idle, starting, recording, playing, paused. |

`play` is a JSON command such as `{"command": "play", "files": [...]}`. The files are relative to `--dir`.
Optional fields are `rate`, `loop`, `prefix`, `keys` and `cuda_device`.

The control key is also a queryable. A command sent as a query receives its answer once it took effect: `record`
answers "ready", and `stop` answers with the counts and the check. A refused command answers with the reason.
`xblick recorder <command>` and `recording.request(...)` in Python use this.

The node does not record while it plays, because it would record its own playback. It refuses such a command and
states the reason in its status. `--paused` starts the node idle, so that it waits for a command.

## Record exactly these keys

By default the recorder records what it finds. With `--key` (or `keys` in a command) it subscribes to the given key
expressions and nothing else.

```bash
xblick recorder record --key 'sim/*' --key robot/joint_states --blocking --log-time header --file ep.mcap
```

The recorder looks for the publishers of the keys and answers "ready" once it is subscribed to all of them. It
waits up to `ready_timeout` (2 s) while a literal key has no publisher. The answer contains:

| Field | Content |
| --- | --- |
| `publishers` | the publishers that it found, with the counts of the blocking ones |
| `streams` | the keys whose info record it has |
| `missing` | the literal keys without a publisher |
| `dropping` | the keys of publishers that do not block |

`--file` names the file. The recorder never overwrites a file, and it does not split this file. The metadata
record `xblick_recording` in the file keeps the keys and the counts.

## Log time

Every MCAP message has a log time. `--log-time` selects it.

| Value | Log time | Use |
| --- | --- | --- |
| `receive` (default) | the arrival at the recorder | recordings of a robot |
| `header` | the stamp of the message itself | recordings of a simulation, which then lie on the simulated timeline |

With `header`, the recorder takes the stamp from the header, from a `Clock`, from the first transform of a
`TFMessage`, or from `stamp_ns` of a JSON sample. A message without a stamp is an error. The publish time in MCAP
stays the arrival time.

## Recordings that lose no sample

By default, publishers and the recorder drop samples under load ([latest value](../concepts/latest-value.md)). A
recording for a dataset must be complete. This needs two opt-ins.

1. The publisher is created with `blocking=True`.
2. The recorder is started with `--blocking` and with keys.

The recorder then blocks when its queue is full, and Zenoh holds the blocking publishers up.

**At the start,** the recorder refuses to start while a blocking publisher does not yet know of the subscription,
because the publisher's samples would be lost. A blocking publisher that appears later under a wildcard key is
found within one second.

**At the stop,** the recorder asks every blocking publisher for its count. It waits up to 5 s for samples that are
still under way. It then checks each key by the sequence numbers in the attachments: every sample that was put
after "ready" must be in the file. For a key found after "ready", the check starts at its first sample.

A lost sample, or two publishers on one key, is an error (`RecordingError`, or `ok: false` with the counts over
Zenoh). The file is complete all the same, and its metadata states what is missing.

**Effect.** We sent 2000 samples of 1 MB to a subscriber that needed 2 ms per sample. With blocking, all 2000
arrived, and the publisher was held up for 3.7 s. Without blocking, 325 arrived. Measured on 2026-11-03 on an AMD
EPYC 9335 between two Python sessions over loopback TCP.

**A robot is never held up by accident.** A blocking recorder blocks only for the keys of publishers that opted
in. It records the samples of every other publisher as they arrive and drops them when it falls behind. This also
holds when both kinds are in one recording or in one process. The check at the stop covers the blocking keys and
reports the others under `dropping` with their received and dropped counts. `tests/test_sim_recording.py` covers
this: a dropping publisher sends its 400 samples at full speed to a recorder that needs 5 ms per sample, the
recorder drops them, and the blocking key in the same recording is complete.

## Playback

`play` publishes each image stream as a camera does, through a `CameraPublisher`.

- A `compressed` subscriber receives the recorded JPEG, PNG or RVL unchanged.
- Readers on this PC receive the frame from a host ring. `play` decodes each frame once for them.
- With `--cuda-device`, `play` also offers a CUDA ring.
- `raw` subscribers receive the pixels.

`play` sends the recorded `camera_info`, answers `<stream>/info` for streams and detections, and answers
`tf_static` queries. `--rate` changes the pace, and `--key PATTERN` selects topics.

**Loops.** With `--loop`, each pass moves every stamp forward by the length of the recording plus one period of
its slowest channel. This covers headers, transforms and `stamp_ns` of JSON samples, and it leaves `tf_static`
unchanged. Time therefore goes on as with a camera, and nodes that accept only newer frames continue to work.
`play` refuses `--loop` for a channel whose type it does not know. In a test, NeMO started on a playback and
tracked the objects at 10 frames per second, as on the live simulation.

### Step by step

`--pace` selects what drives the playback.

| Pace | Behaviour |
| --- | --- |
| `wall` (default) | the recorded pace. `rate` and `pause` belong to this pace. |
| `step` | `play` sends a frame only on a `step` command. A frame is the set of all messages with one log time. With log time `header`, this is one simulated instant. |
| `clock` | `play` sends everything up to the time of each `rosgraph_msgs/Clock` sample on the key `--clock`. A policy or a node under test sets the pace. |

A `step` command accepts `frames` (default 1), `until_ns` (all frames up to this log time) or `key` (up to the
next message on this key, and the rest of its frame). It answers once the messages went out.

```bash
xblick play ep.mcap --pace step
xblick recorder step                # or: --frames N, --key KEY
```

With `step` and `clock`, every played key blocks, so that a reader receives every frame. In Python, `Player.step`
does the same.

### Parameters of a playback

The play node also offers its controls as [parameters](../reference/parameters.md) of the node `xblick/<name>`.

| Parameter | Meaning |
| --- | --- |
| `rate`, `paused`, `loop` | as the options |
| `position_s` | a seek. The playback continues from this log time. The stamps keep growing, as in a loop. |
| `duration_s`, `file` | read-only |

The terminal interface shows them as fields in its Parameters tab. The [video guide](video.md) has the full table.

## Recordings in Rerun

Rerun 0.39 opens the files directly, or converts them with `rerun mcap convert`. It shows images, `camera_info` as
pinhole cameras, TF, and joint states as plots. It imports detections, odometry and diagnostics without drawing
them, and it does not import bodies. To see everything as live, run `xblick play` together with
`xblick rerun`.

## Videos

`xblick play` also plays video files and image sequences as camera streams. See the [video guide](video.md).

## How a simulation records

A simulation publishes everything with blocking publishers and controls the recording itself. It starts the
recording with the list of its keys, begins the episode when the recorder is ready, and stops the recording when
the episode is over. The stop states whether every sample is in the file.

### In one process

An offline batch records each episode in its own process with `recording.Recording`. It uses a private session
that needs no router and reaches nobody.

```python
import xblick
from xblick import recording, topics
from xblick.sim import SimPublisher

session = xblick.open_session(local_router=False)      # private: needs no zenohd
sim = SimPublisher(session, "robot", blocking=True)        # robot state, world, commands, events
head = xblick.CameraPublisher(session, "test/camera/color", encoding="rgb8", frame_id="cam",
                                  camera_info=info, source="sim", blocking=True)
keys = sim.keys + [topics.compressed(head.base), topics.camera_info(head.base)]
for n, spec in enumerate(specs):
    with recording.Recording(session, f"episodes/{n:04d}.mcap", keys) as rec:   # blocking, log time = stamps
        for step in episode(spec):                           # 1 ms of simulated time each
            stamp = xblick.Time.from_ns(step.t_ns)
            sim.clock(step.t_ns)
            sim.joint_states(xblick.JointState(xblick.Header(stamp), names, q, dq, tau))
            sim.command_joints(xblick.JointState(xblick.Header(stamp), ["hand_left"], [closure]))
            if step.camera_frame:                            # at the rate of the fastest camera
                sim.bodies(xblick.TFMessage(poses_of_all_bodies(stamp)))
                head.publish(step.image, stamp_ns=step.t_ns)
            if step.phase_changed:
                sim.event(step.t_ns, "phase", phase=step.phase)
        sim.event(step.t_ns, "success" if spec_succeeded else "failure")
    # after the block the file is complete and checked. A missing sample raises RecordingError.
    print(rec.result["check"]["counts"])
```

### With a recorder node

A teleoperation or a policy evaluation uses a recorder node that runs elsewhere
(`xblick record --paused --dir ...`).

```python
ready = recording.request(session, "recorder", "record", keys=keys, blocking=True, log_time="header",
                          file="teleop_0001.mcap")
assert not ready["missing"], ready      # the keys without a publisher
...                                     # the episode
result = recording.request(session, "recorder", "stop")     # a missing sample raises RecordingError
```

A playback with `--pace step` or `--pace clock` then delivers the episode frame by frame on the simulated timeline,
exactly as the simulation published it.

### Cost

We simulated 10 s with 30 joints, the clock and the commands at 1 kHz, 60 bodies and a 640 x 480 JPEG camera at
30 Hz. This gave 30,912 samples.

| | Time |
| --- | --- |
| Publishing loop without a recorder | 0.51 s |
| Publishing loop with the recorder | 1.37 s |
| Stop with its check | 0.21 s more |

The file had 1.4 MB with a smooth synthetic image. Real images make the file larger and leave the number of
samples unchanged. Measured on 2026-11-03 on an AMD EPYC 9335, in one process with one publishing thread.

## Status and limits

- `tests/test_sim_recording.py` tests the complete recording in one process and between two sessions over TCP:
  500 simulated milliseconds of joints, commands, clock and 16 camera frames are all in the file. A lost sample is
  an error, and so is a blocking publisher that does not know of the subscription.
- Not tested with a simulator itself yet.
