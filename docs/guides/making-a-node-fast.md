# Making a node fast

This guide collects what we learned while we made the transport and twelve nodes faster. It explains the method,
the changes that helped in several nodes, and the changes that did not help. Each node's repository has its own
measurements in detail.

## Results

| What | Before | After | Main change | Details |
| --- | --- | --- | --- | --- |
| Publish 1080p to 16 readers on one host | 26 ms publisher CPU (Zenoh TCP) | about 1 ms | one copy in a ring, descriptors to the readers | [Rings](../concepts/rings.md) |
| Azure Kinect colour at 2160p for a JPEG consumer | 28 ms | 0.02 ms | the camera's JPEG passed on, decoded only for pixel subscribers | xblick-azure-kinect |
| Boxer, median per frame (fastest frames) | 140 ms (85 ms) | 28 ms (17.4 ms) | inputs on the GPU, weights in bfloat16, two CUDA streams | xblick-boxer |
| SAM 3.1 with 8 words, compiled | 51.1 ms at 1008 px | 24.4 ms at 672 px | all words in one pass, mask head only for detections, lower resolution | xblick-sam3 |
| Fast-FoundationStereo, 640 x 480 | 26.3 ms (PyTorch) | 11.1 ms | TensorRT | xblick-fast-foundationstereo |
| FoundationPose tracking | 22.7 ms (the original) | 8.5 ms | a port to PyTorch, merged vertices, compilation, 3 refiner passes | xblick-foundationpose |
| NeMO, metric pose per query | 1.2 s | 10 ms | RANSAC with all hypotheses on the GPU | xblick-nemo |
| Azure Kinect body tracking | 123 ms CPU, 16.4 ms CUDA | 7.0 ms | TensorRT | xblick-azure-kinect |
| IVT localisation of 12 textured objects | 137-572 ms (32 to 1 threads) | 4.9 ms offline, 5.6 ms live | the pipeline in PyTorch on the GPU | xblick-ivt |
| nvblox at 60 frames/s input | 8 frames/s | 47 frames/s | the newest complete set of inputs | xblick-mapping |
| Boxer beside nvblox and the robot mask at 60 frames/s | 62 ms latency | 30 ms | NVIDIA MPS | [Sharing a GPU](sharing-a-gpu.md) |

GPU: RTX PRO 6000 Blackwell unless a node's page states another. CPU: AMD EPYC 9335. The 28 ms for the Azure Kinect
is the per-frame work of the previous capture software, which we reproduced.

## Step 1: measure before you change anything

**Compare variants alternately in one process.** On a shared GPU, the jobs of other users change your timings from
minute to minute. If you measure variant A now and variant B five minutes later, you compare the other jobs. Run
A and B in turn, frame by frame. When other jobs run, also compare the fastest frames. We measured Boxer beside
46-100 % load from other jobs, and the medians alone were misleading.

**Find where the time goes.** A GPU executes work asynchronously, so a Python timer around a line measures only
the time to queue the work. Two tools give true times.

- A CUDA event marks a point in a CUDA stream. The time between two events is the time the GPU needed.
- A synchronisation (`torch.cuda.synchronize()`) after one step makes Python wait for that step. Synchronise the
  steps one by one to see the cost of each.

**Count the kernels.** A *kernel* is one function that the GPU executes, for example one convolution. Each kernel
start costs a few microseconds on the CPU. Count how many kernels one frame starts.

## Step 2: find out what limits the node

A node is slow for one of two reasons, and each reason has different cures.

| The node is | Meaning | Sign | What helps |
| --- | --- | --- | --- |
| GPU-bound | The GPU computes all the time. | few, large kernels, GPU load near 100 % | a faster model form: TensorRT, lower precision, lower resolution |
| Launch-bound | The CPU cannot start kernels fast enough, and the GPU waits between them. | thousands of small kernels per frame, low GPU load | fewer kernels: batching, compilation, no per-frame conversions |

*TensorRT* is NVIDIA's compiler for networks. It merges layers and selects the fastest implementation of each for
your GPU. It halved the time of Fast-FoundationStereo, which is GPU-bound. It did nothing for Boxer, which starts
about 1600 kernels per frame and is launch-bound.

## Changes that helped in several nodes

**Keep the CPU out of the path between the frame and the model.** Resize, convert and sample on the GPU. In Boxer,
a range check `img.max()` on the CPU took 17 ms, and the conversion to a tensor took 12 ms.

**Convert the weights to bfloat16 once.** *bfloat16* is a 16-bit number format that modern GPUs compute much faster
than 32-bit floats. PyTorch's *autocast* converts values to it on the fly. With autocast, Boxer converted its
32-bit weights in every frame, which started about 300 extra kernels. Convert the parameters once at start.
Convert only the parameters: `Module.to(bfloat16)` also converts SAM 3's position buffers, drops their imaginary
part, and SAM 3 then finds nothing without any error message.

**Batch what the model does per item.** SAM 3 processed one word after another, NeMO one hypothesis, IVT one face.
One pass for all items starts the same kernels once.

**Skip work whose result you discard.** SAM 3 now computes masks only for the detections that it keeps. With 8
words this saves 11 ms.

**Use two CUDA streams for two stages.** A *CUDA stream* is a queue of GPU work. Work in two CUDA streams can
overlap. Boxer's two stages on two CUDA streams raised its rate from 21-26 to 27-29 frames per second on a busy
GPU.

**Lower the resolution only with a check of the accuracy.** SAM 3 at 672 px finds 98-99 % of the instances again
that it finds with confidence at 1008 px. This number measures agreement with 1008 px. It does not measure accuracy
against ground truth.

**Use MPS when several nodes share a GPU.** See [sharing a GPU](sharing-a-gpu.md).

**Name your CUDA stream when you publish a device array.** A publish then takes 0.1 ms. Without the name it took
7 ms, because the publisher waited for all GPU work of the process.
[Frames on the GPU](../concepts/gpu-frames.md) explains why.

**Pair streams by the newest complete set.** A node that waits for the partner of every single frame falls behind
at high input rates. nvblox takes the newest set of depth, pose and mask that is complete. See
[pairing streams](pairing-streams.md).

**Keep the CPU clock up between frames.** A node that sleeps between short bursts of work lets the Linux governor
lower the CPU frequency. IVT needed 10.4 ms per stereo pair at 20 pairs per second and 4.9 ms without pauses. Its
thread now sets `uclamp_min`, which tells the scheduler to keep the frequency up.

## Changes that did not help

| Change | Node | Result |
| --- | --- | --- |
| FP8, an 8-bit float format | SAM 3 | no faster, 3.7 % error. Its bfloat16 matrix products already reach 310-380 TFLOPS. |
| `torch.compile` with `max-autotune` | SAM 3 | 3 minutes of compilation, no gain |
| CUDA graphs | SAM 3 | 1 ms of 50 |
| `torch.compile` of DINOv3 | Boxer | within the noise |
| DINOv3 on its own CUDA stream beside OWLv2 | Boxer | 18.8 against 18.2 ms. The node is launch-bound. |
| A separate CUDA stream for a TensorRT engine | Fast-FoundationStereo | 11.16 against 11.17 ms. The engine is GPU-bound. |
| CUDA graphs for the face fit | IVT | 5.20 against 5.14 ms, within the noise |
| Planes and poses on the GPU | IVT | 1.8 ms compiled against 0.8 ms in NumPy. The data are few faces and chains of small solves. |

*`torch.compile`* translates a PyTorch model into optimised code and thereby merges kernels. A *CUDA graph* records
a fixed sequence of kernels and starts it with one call.

The pattern behind the table: a tool that removes kernel starts does not help a GPU-bound stage, and a tool that
speeds up computation does not help a launch-bound stage. Small problems are often faster on the CPU.

## Where each node documents its speed

| Node | What its page covers |
| --- | --- |
| xblick-azure-kinect | the JPEG pass-through, nvJPEG into the CUDA ring (720p: 0.56 ms against 2.02 ms on the CPU), NPP remap for the rectified colour image (0.6-0.7 ms against 3.2-3.7 ms), body tracking with TensorRT |
| `xblick-points` | rays once per calibration and a kernel that writes into the CUDA ring: 1.7-2.7 ms host time per cloud. Readers have the cloud 1.9-2.0 ms after the stamp. [Point clouds](point-clouds.md) |
| xblick-boxer | the four steps from 140 to 28 ms, and where the time goes |
| xblick-sam3 | words in one pass, the mask head, the resolution, compilation with a warm-up |
| xblick-fast-foundationstereo | TensorRT, the choice of the checkpoint by accuracy and time, `--max-width` |
| xblick-foundationpose | the port to PyTorch, merged vertices (rendering 646 to 195 ms), compilation, refiner passes |
| xblick-nemo | the RANSAC on the GPU, detection in bfloat16 (61 against 160 ms) |
| xblick-tapnext | one state of slots for all tracks, catch-up states |
| xblick-mapping | its own build of nvblox (384 feature channels in one map: 7.2 against 14 ms), the feed, MPS |
| xblick-gem-x, xblick-vjepa2, xblick-insightface | persons batched in bfloat16 (GEM-X: 30 ms per person), frames prepared on arrival (V-JEPA 2), InsightFace on ONNX Runtime |
| xblick-ivt | the GPU pipeline step by step (13.0 to 4.9 ms), the CPU clock, what stays on the CPU |
