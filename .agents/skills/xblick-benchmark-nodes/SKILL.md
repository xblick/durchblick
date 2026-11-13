---
name: xblick-benchmark-nodes
description: >
  Use this skill when the user wants numbers on Durchblick nodes: throughput, latency from capture to result, GPU
  load and memory, for one node alone or several together, on one GPU, several GPUs, with or without MPS, or across
  servers and robot PCs, also when they ask whether a setup is fast enough ("can SAM 3 and Boxer share one GPU at
  30 Hz", "what latency does the relay add", "is the new node slower than the old one"). It fixes the input, runs the
  configurations one after another, measures with a bundled script, and reports a comparison. Not for starting a
  setup without measuring (use xblick-run-nodes) and not for optimising a node's code (use xblick-build-node).
compatibility: Durchblick 0.5.x, the nodes' repositories, nvidia-smi; Python via `uv run` in the Durchblick checkout
---

# Benchmark nodes

Done means: every configuration of the plan is measured on the same input, the numbers are in one table with the
setup, and the user knows what each number means and how noisy it is.

Background: [Timestamps](../../../docs/concepts/timestamps.md) (what each age measures),
[Making a node fast](../../../docs/guides/making-a-node-fast.md) (method),
[Sharing a GPU](../../../docs/guides/sharing-a-gpu.md), [Node reference](../../../docs/reference/nodes.md) (earlier
numbers and how they were taken).

## Progress

- [ ] 1. Question and configurations stated
- [ ] 2. Input fixed
- [ ] 3. Machines checked
- [ ] 4. Each configuration measured
- [ ] 5. Repeated where a decision depends on it
- [ ] 6. Reported

## 1. Question and configurations

State the question as numbers: for example "results per second and p95 latency of SAM 3 with Boxer on one GPU, with
and without MPS". List the configurations as a matrix. Read [references/configurations.md](references/configurations.md)
now: it describes the usual axes (one GPU, MPS, several GPUs, across PCs, a relay) and what each changes.

## 2. Fix the input

Use a recording or a video at the camera's real rate, looped, so that every configuration sees the same frames:

```bash
xblick play <recording.mcap> --loop --cuda-device <gpu>
```

The content decides much of a node's load: count the objects, people or words in view, and keep them the same
across configurations.

## 3. Check the machines

- `nvidia-smi` on every GPU of the plan: other users' jobs change the numbers. Prefer idle GPUs. If none is idle,
  record the foreign load, compare variants alternately, and report the fastest frames beside the median.
- The CPU load (`uptime`): simulations and other jobs on the same machine slow nodes too.
- The clocks of the PCs, if the setup spans PCs: an age at receive across PCs is only as good as their
  synchronisation. The age at send is exact on any PC.

## 4. Measure each configuration

Start the configuration with the xblick-run-nodes skill. Wait until every node reports OK and its warm-up has
passed (compilation, TensorRT engines, the first frames). Then measure:

```bash
uv run python <skill>/scripts/measure_topics.py <result topics> --seconds 60 --warmup 10 --gpus <indices>
```

The script prints one JSON line per topic and per GPU: rate, interval, age at send and age at receive (p50, p95,
max), GPU utilisation and memory. Choose the topics carefully:

- **A node's result topic** (`<camera>/<node>/<name>`): its age at send is the latency from the capture of the input
  to the result leaving the node, because a result carries its input's stamp.
- **A stream's ring descriptor** (`<stream>/image_raw/shm/*` or `<stream>/image_raw/cuda/**`; the key ends with
  the host and the GPU of the ring): cheap to subscribe, and shows the camera's
  own rate and age.
- **Never measure a transport that the setup does not use.** Subscribing to `image_raw/compressed` makes the
  publisher encode JPEG for the measurement alone, which changes the load you measure.

Record per node its VRAM from `nvidia-smi --query-compute-apps=pid,used_memory --format=csv` and each node's own
diagnostics values (`xblick diagnostics <namespace> --values`), which often time the node's stages.

Stop the configuration completely before the next one, including MPS daemons, so that one configuration does not
leave load for the next.

## 5. Repeat

Run a configuration that decides something at least twice, at different times. A difference smaller than the
spread between two identical runs is no difference. Report both.

## 6. Report

```markdown
## <question>

Setup: <GPUs and PCs>, input <recording, rate, content>, Durchblick <version>, node commits, date, foreign load.

| Configuration | Node | Results/s | Age at send p50 / p95 [ms] | GPU load | VRAM [GiB] |
| --- | --- | --- | --- | --- | --- |

Spread between repeated runs: <...>. Conclusion: <answer to the question, with its uncertainty>.
```

Add the numbers that are worth keeping to the node's `docs/performance.md` or to the core's node reference, with
the setup.

## Gotchas

- GPU utilisation is the share of time in which some kernel runs, not the share of the GPU's throughput. A node at
  50 % leaves room, but not half the GPU.
- Without MPS, processes on one GPU take turns. Boxer beside nvblox and the robot mask at 60 frames per second had
  62 ms latency without MPS and 30 ms with it. Under MPS, check that nvidia-smi lists each node's memory; if it
  does not, measure each node's VRAM alone, without MPS.
- The first frames after a start are slow (compilation, allocator, engines). Use `--warmup`.
- A viewer or a second measurement subscribed at the same time changes what the publishers produce. Close them.
- `--max-hz` of a node caps its rate. Note it in the setup, or the result looks like a speed limit of the node.
- Looped playback moves the stamps forward on each pass; the ages stay correct.
- `xblick hz <stream>` gives a quick live view of one image stream, but it subscribes to that stream's
  transport. Use the script for numbers.

## When this skill was wrong

If a step failed or misled, propose a fix of this skill to the user after the task.
