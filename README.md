# Durchblick

Durchblick is a middleware for camera streams and perception results. It moves frames between programs in shared
memory and in GPU memory, and it uses the message types of ROS 2 without needing ROS.

## Why Durchblick

A 1080p colour frame has 6 MB. A publisher that sends it to 16 programs over TCP spends 26 ms of CPU time per
frame. Durchblick writes the frame once into shared memory and sends each reader only the location of the frame.
This takes about 1 ms for any number of readers.

- **Each host stores a frame once.** The publisher writes the frame into a ring in shared memory, and all readers
  on that host work on this one copy. [Rings](https://github.com/xblick/xblick/blob/main/docs/concepts/rings.md)
- **Frames stay on the GPU.** A GPU producer and a GPU consumer exchange frames in GPU memory, without a copy to
  the CPU. [Frames on the GPU](https://github.com/xblick/xblick/blob/main/docs/concepts/gpu-frames.md)
- **A publisher works only on request.** It encodes a JPEG only while a subscriber asks for JPEG, and it writes a
  ring only while a reader has it open.
  [Transports](https://github.com/xblick/xblick/blob/main/docs/concepts/transports.md)
- **Latency stays constant under load.** Every stream keeps only its newest frame, so a slow consumer skips frames
  and never receives old ones.
  [Latest value](https://github.com/xblick/xblick/blob/main/docs/concepts/latest-value.md)
- **Every consumer uses the same stream.** The subscriber chooses the transport. The topic, the message type and
  the API are the same on one host and across the network, in Python and in C++, on a robot and in a simulation.
  [Streams and topics](https://github.com/xblick/xblick/blob/main/docs/concepts/streams-and-topics.md)

## Requirements

Linux on x86-64, Python 3.10 to 3.13, and one Zenoh router (`zenohd` 1.10) per PC. An NVIDIA GPU is needed only for
frames in GPU memory. C++ programs need CMake and a C++20 compiler.

## Install

```bash
pip install "xblick[rerun,record]"                      # wheels for Linux x86-64 with glibc 2.28 or newer
zenohd --no-multicast-scouting -l tcp/0.0.0.0:7447     # the router, once per PC
```

To build from a checkout of this repository, run `pip install "./xblick[rerun,record]"`. The build needs CMake
and a C++20 compiler.

Extras and C++: [install](https://github.com/xblick/xblick/blob/main/docs/getting-started/install.md).

## Use

```bash
xblick test-camera       # publish a moving test pattern
xblick list              # show every stream
xblick hz robot/test_camera/color
xblick rerun             # show every stream and result in the Rerun viewer
```

```python
import xblick as db

session = db.open_session()
images = db.ImageSubscriber(session, "robot/test_camera/color", transport="auto")
frame = images.latest(timeout=1.0)        # the newest frame as a NumPy array
```

On the publisher's host, `transport="auto"` reads the frame from a ring. On another host, it receives a JPEG over
the network. Your code is the same in both cases.

Continue with [your first stream](https://github.com/xblick/xblick/blob/main/docs/getting-started/first-stream.md).
Without a command, `xblick` opens a
[terminal interface](https://github.com/xblick/xblick/blob/main/docs/guides/tui.md)
with all streams, nodes and parameters.

## Write a node

A node reads the streams of a camera and publishes its results below that camera, at `<camera>/<node>/<name>`.
The tutorial [your first node](https://github.com/xblick/xblick/blob/main/docs/getting-started/first-node.md)
builds a complete node in 40 lines. The
[writing nodes guide](https://github.com/xblick/xblick/blob/main/docs/guides/writing-nodes.md)
lists what Durchblick gives every node: logging, diagnostics, frames as tensors, streams paired by stamp, rate
limits and parameters.

Three rules matter most.

1. Stamp each result with the capture time of its input.
2. Name your CUDA stream when you publish an array from GPU memory.
3. Fail with a clear message. Do not continue with a default that hides a wrong configuration.

## Nodes

Each perception method is a node in a repository of its own. Every node works on any camera, whether the frames
come from a robot, a simulation or a recording.

| Task | Nodes |
| --- | --- |
| Cameras | [azure-kinect](https://github.com/xblick/xblick-azure-kinect), [roboception](https://github.com/xblick/xblick-roboception), [relay](https://github.com/xblick/xblick-relay) |
| Depth and point clouds | `fast-foundationstereo`, `xblick-points` (this repository) |
| Segmentation, detection, tracking | [sam3](https://github.com/xblick/xblick-sam3), [boxer](https://github.com/xblick/xblick-boxer), [tapnext](https://github.com/xblick/xblick-tapnext) |
| 6D object poses | `foundationpose`, [nemo](https://github.com/xblick/xblick-nemo), `ivt` |
| Meshes of new objects | `recgen`, `sam3d-objects`, `any6d` |
| People | [gem-x](https://github.com/xblick/xblick-gem-x), `insightface`, [vjepa2](https://github.com/xblick/xblick-vjepa2) |
| Camera pose and map | `mapping` (cuVSLAM, nvblox) |
| Recording | `xblick record`, `xblick play` (this repository) |

The [node reference](https://github.com/xblick/xblick/blob/main/docs/reference/nodes.md)
has the data flow between the nodes, what each node publishes, and the GPU memory and load of each.

## Speed

| What | Before | After | Main change |
| --- | --- | --- | --- |
| Publish 1080p to 16 readers on one host | 26 ms CPU | 1 ms | one copy in a ring |
| Azure Kinect 2160p colour for a JPEG consumer | 28 ms | 0.02 ms | the camera's JPEG passed on unchanged |
| Boxer, median per frame | 140 ms | 28 ms | inputs on the GPU, bfloat16 weights, two CUDA streams |
| SAM 3.1 with 8 words | 51.1 ms | 24.4 ms | one pass for all words, 672 px |
| Fast-FoundationStereo, 640 x 480 | 26.3 ms | 11.1 ms | TensorRT |
| FoundationPose tracking | 22.7 ms | 8.5 ms | a port to PyTorch |
| NeMO metric pose | 1.2 s | 10 ms | RANSAC on the GPU |
| IVT localisation of 12 objects | 137-572 ms | 4.9 ms | the pipeline on the GPU |
| nvblox at 60 frames/s input | 8 frames/s | 47 frames/s | the newest complete set of inputs |

Measured on an RTX PRO 6000 and an AMD EPYC 9335.
[Making a node fast](https://github.com/xblick/xblick/blob/main/docs/guides/making-a-node-fast.md) explains
the method, the changes that helped in several nodes, and the changes that did not help.

## Documentation

| | |
| --- | --- |
| **Start** | [Install](https://github.com/xblick/xblick/blob/main/docs/getting-started/install.md) · [Your first stream](https://github.com/xblick/xblick/blob/main/docs/getting-started/first-stream.md) · [Your first node](https://github.com/xblick/xblick/blob/main/docs/getting-started/first-node.md) |
| **Concepts** | [Streams and topics](https://github.com/xblick/xblick/blob/main/docs/concepts/streams-and-topics.md) · [Transports](https://github.com/xblick/xblick/blob/main/docs/concepts/transports.md) · [Rings](https://github.com/xblick/xblick/blob/main/docs/concepts/rings.md) · [Frames on the GPU](https://github.com/xblick/xblick/blob/main/docs/concepts/gpu-frames.md) · [Latest value](https://github.com/xblick/xblick/blob/main/docs/concepts/latest-value.md) · [Timestamps](https://github.com/xblick/xblick/blob/main/docs/concepts/timestamps.md) · [Protocol](https://github.com/xblick/xblick/blob/main/docs/concepts/protocol.md) · [Sessions and routers](https://github.com/xblick/xblick/blob/main/docs/concepts/sessions.md) |
| **Guides** | [Writing nodes](https://github.com/xblick/xblick/blob/main/docs/guides/writing-nodes.md) · [Pairing streams](https://github.com/xblick/xblick/blob/main/docs/guides/pairing-streams.md) · [Making a node fast](https://github.com/xblick/xblick/blob/main/docs/guides/making-a-node-fast.md) · [Sharing a GPU](https://github.com/xblick/xblick/blob/main/docs/guides/sharing-a-gpu.md) · [Recording](https://github.com/xblick/xblick/blob/main/docs/guides/recording.md) · [Video](https://github.com/xblick/xblick/blob/main/docs/guides/video.md) · [Point clouds](https://github.com/xblick/xblick/blob/main/docs/guides/point-clouds.md) · [Viewer](https://github.com/xblick/xblick/blob/main/docs/guides/viewer.md) · [Terminal interface](https://github.com/xblick/xblick/blob/main/docs/guides/tui.md) |
| **Reference** | [Command line](https://github.com/xblick/xblick/blob/main/docs/reference/cli.md) · [Python](https://github.com/xblick/xblick/blob/main/docs/reference/python-api.md) · [C++](https://github.com/xblick/xblick/blob/main/docs/reference/cpp-api.md) · [Topics](https://github.com/xblick/xblick/blob/main/docs/reference/topics.md) · [Parameters](https://github.com/xblick/xblick/blob/main/docs/reference/parameters.md) · [Nodes](https://github.com/xblick/xblick/blob/main/docs/reference/nodes.md) · [Glossary](https://github.com/xblick/xblick/blob/main/docs/reference/glossary.md) |
| **Integrations** | [ROS 2](https://github.com/xblick/xblick/blob/main/docs/integrations/ros2.md) |
| **Project** | [Development](https://github.com/xblick/xblick/blob/main/docs/project/development.md) · [Alternatives](https://github.com/xblick/xblick/blob/main/docs/project/alternatives.md) · [Writing documentation](https://github.com/xblick/xblick/blob/main/docs/project/writing-docs.md) · [Changelog](https://github.com/xblick/xblick/blob/main/CHANGELOG.md) |

Coding agents read [AGENTS.md](https://github.com/xblick/xblick/blob/main/AGENTS.md) and the project skills in
[.agents/skills](https://github.com/xblick/xblick/blob/main/.agents/skills/README.md).

ArmarX integration: `durchblick_armarx`.

## Status

Version 0.5.0. Durchblick is in active development, and a new minor version can change the protocol. All
programs of one setup speak the same protocol. We now released that version.
The [changelog](https://github.com/xblick/xblick/blob/main/CHANGELOG.md) states which versions do. Programs of
0.5 do not work with programs of 0.3 or 0.4. The nodes are tested in simulation and on recordings. Tests on robots
are in progress.

## Origin

Durchblick was started at the Computer Vision for Human-Computer Interaction Lab (cv:hci) at KIT and is developed
further in cooperation with the High Performance Humanoid Technologies Lab (H2T) at KIT.
