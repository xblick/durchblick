# Alternatives

Other projects that pass camera frames or tensors from process to process without a copy, in particular in GPU
memory. We surveyed them in November 2026 from documentation, announcements and pull requests. We did not run them.
Statements marked "unverified" could not be confirmed in the sources.

## Summary

The closest project is `rosidl::Buffer` in ROS 2 Lyrical with NVIDIA's CUDA backend. It follows the same idea as
Durchblick's CUDA ring: processes share GPU memory, and a message contains only a reference. It is part of ROS 2,
and Isaac ROS 5.0 uses it.

We found no project that combines the following properties of Durchblick in one package.

- One publisher serves shared memory, CUDA rings and compressed or raw network streams at the same time, and each
  subscriber chooses.
- A producer never waits for a reader.
- The messages are ROS 2 messages, and no ROS is needed.
- Python and C++ have the same API.
- Camera nodes, a recorder and a viewer belong to it.

## ROS 2

| Project | What it does | Limits |
| --- | --- | --- |
| [ROS 2 Lyrical `rosidl::Buffer`](https://docs.ros.org/en/lyrical/Concepts/Intermediate/About-Buffer-Backends.html) with [`cuda_buffer_backend`](https://index.ros.org/p/cuda_buffer_backend/) (NVIDIA, since 2026-05) | `uint8[]` fields use the memory of a backend. The CUDA backend shares device memory (CUDA VMM/IPC) within a process and between processes on one host, and copies through the CPU otherwise. `rclpy` uses DLPack. All Isaac ROS 5.0 nodes use it ([NVIDIA blog](https://developer.nvidia.com/blog/?p=122571)). | The GPU path works only on the same host, device and Linux user, with `rmw_fastrtps_cpp` or `rmw_zenoh_cpp`. Several subscribers, the reuse of memory and slow readers are unverified. |
| [Isaac ROS NITROS](https://nvidia-isaac-ros.github.io/concepts/nitros/index.html) (before 5.0) | type adaptation and negotiation (REP 2007/2009), GPU memory between nodes | Zero-copy only within one process. The ROS 1 bridge uses CUDA IPC. |
| [ros2_cuda_ipc](https://discourse.openrobotics.org/t/ros2-cuda-ipc-zero-copy-gpu-data-sharing-between-ros-2-processes/57157) (2026-08) | CUDA VMM file descriptors and IPC events between ROS 2 processes, lifetime handling for several consumers, DLPack | experimental, one author |
| [Autoware cuda_blackboard](https://awesome.ecosyste.ms/projects/github.com%2Fautowarefoundation%2Fcuda_blackboard) | GPU data between ROS 2 nodes | Each further subscriber costs a copy from device to device. |
| [Agnocast](https://discourse.openrobotics.org/t/agnocast-callback-isolated-executor-true-zero-copy-ipc-and-middleware-transparent-scheduling-for-ros-2/52678) (Autoware Foundation) | zero-copy between ROS 2 processes | CPU memory only |

## Zenoh and Rust

| Project | What it does | Limits |
| --- | --- | --- |
| [Zenoh shared memory](https://zenoh-cpp.readthedocs.io/en/1.9.0/shared_memory.html) | zero-copy payloads on one host | host memory only |
| [hiroz](https://discourse.openrobotics.org/t/introducing-hiroz-by-zettascale/55081) (ZettaScale, formerly ROS-Z) | ROS 2 in Rust on Zenoh. An open pull request adds a CUDA IPC tensor transport (`zenoh-cuda`). | Not merged. It depends on an open Zenoh pull request (eclipse-zenoh/zenoh#2463). |
| [dora-rs](https://github.com/dora-rs/dora) | a dataflow framework. Zenoh shared memory for host data, CUDA IPC handles for GPU data ([issue #1872](https://github.com/dora-rs/dora/issues/1872)). | The GPU path is an example. Frames that are made on the CPU and used on the GPU are a known gap. |

## NVIDIA pipeline frameworks and GStreamer

| Project | What it does | Limits |
| --- | --- | --- |
| [Holoscan SDK](https://docs.nvidia.com/holoscan/sdk-user-guide/introduction/relevant-technologies) | UCX between fragments (CUDA IPC, GPUDirect RDMA). HoloMQ is a pool of exportable CUDA slots: it sends a handle and tensor metadata, and signals readiness on a CUDA stream ([RidgeRun](https://www.ridgerun.com/post/optimizing-interprocess-gpu-frame-transport-in-holoscan-sdk)). | Documented with one sender and one receiver. A held slot is not reused, so a slow reader can hold the producer up. |
| [DeepStream Gst-nvunixfd](https://docs.nvidia.com/metropolis/deepstream/9.0/text/DS_plugin_gst-nvunixfd.html) | NvBufSurface handles over a Unix socket, from one sink to several sources | GStreamer pipelines, one device |
| [GStreamer cudaipcsink/cudaipcsrc](https://gstreamer.freedesktop.org/documentation/nvcodec/cudaipcsink.html) (since 1.24) | CUDA memory between GStreamer processes, with legacy CUDA IPC or VMM | GStreamer pipelines |
| [NvSciStream](https://developer.nvidia.com/docs/drive/drive-os/7.0.3/public/drive-os-linux-sdk/core-concepts/NvStreams/NvSciStreamSampleApplication1.html) (DRIVE OS) | a camera producer multicast to up to 4 consumers in other processes | NVIDIA's automotive platforms |

## Smaller projects

Three projects solve the same problem inside one application each:
[IsaacTeleop](https://github.com/NVIDIA/IsaacTeleop/pull/1078) sends camera frames as CUDA VMM handles over a Unix
socket, [Isaac Lab](https://github.com/isaac-sim/IsaacLab/pull/8410) sends images to Cosmos by CUDA IPC, and
[cuda-link](https://github.com/forkni/cuda-link) is a CUDA IPC ring between TouchDesigner and Python.

## What this means for Durchblick

We follow two developments: the buffer backends of ROS 2 and the CUDA transport that is proposed for Zenoh.

- A ROS mode of Durchblick could offer its CUDA ring as a `rosidl::Buffer` backend. ROS 2 nodes, including Isaac
  ROS, could then read Durchblick's frames in place. See [Durchblick and ROS 2](../integrations/ros2.md).
- If Zenoh gains GPU payloads, the descriptors of the ring could use them.
