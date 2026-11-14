---
name: xblick-build-node
description: >
  Use this skill when the user wants a new Durchblick vision node, wants to wrap a perception model or method
  (segmentation, detection, pose, depth, tracking, bodies, mapping) so that it reads Durchblick camera streams and
  publishes results, or wants to make an existing node faster or leaner on the GPU, also when they just name a model
  ("put SAM 4 on the robot's camera", "can we run FoundationStereo live"). It sets the node up in its own repository,
  follows the rules every node keeps, and optimises it by measurement. Not for starting or benchmarking existing
  nodes (use xblick-run-nodes or xblick-benchmark-nodes).
compatibility: Durchblick 0.5.x, uv, an NVIDIA GPU with CUDA for GPU nodes
---

# Build and optimise a Durchblick node

Done means: the node runs on a replayed camera, its results carry the stamp of their input, it reports diagnostics,
its README and AGENTS.md follow the documentation guide, and its speed is measured and documented with what was
tried.

Background: [Writing nodes](../../../docs/guides/writing-nodes.md),
[Pairing streams](../../../docs/guides/pairing-streams.md),
[Making a node fast](../../../docs/guides/making-a-node-fast.md),
[Frames on the GPU](../../../docs/concepts/gpu-frames.md).

## Progress

- [ ] 1. Inputs, outputs and target rate stated
- [ ] 2. Repository set up from the node pattern
- [ ] 3. Node runs on a replay and publishes stamped results
- [ ] 4. Baseline measured
- [ ] 5. Optimised, one change at a time
- [ ] 6. Documented and committed

## 1. State the node's contract

Write down, and confirm with the user: the input streams (colour, depth, a segmenter's instances), the output topics
below `<camera>/<node>/`, their message types (`vision_msgs` detections, label images, point clouds; JSON only where
no ROS type fits), the target rate and latency, and the GPU memory budget. Look at the
[node reference](../../../docs/reference/nodes.md) for a node that already does something close, and copy its
structure.

## 2. Set up the repository

Follow the pattern of the existing node repositories:

- a uv project pinned to a Durchblick minor version (`pyproject.toml`, `uv.lock`), with the model's own dependencies,
- `tools/setup.sh`, which builds `.venv` and fetches the checkpoints, so that a workspace build step can call it,
- one console script `xblick-<name>`, named after the model, not the task (`xblick-yolox`, not `xblick-detect`),
- the weights in `weights.py` as a list of `xblick.weights.Weight` and a `xblick.weights.Store` with the model's
  terms notice: downloaded at the start, `--download-only` for `tools/setup.sh`, `--no-download`; `ensure` and
  `download_only` get the options that select the weights (`options=["--model", args.model]`); the store's root
  is `xblick.paths.cache_dir(<node>)` (`Store(prog, node=..., terms=...)`, `store = STORE.at(args.weights_dir)`),
  built files in the cache too, a user's files in `data_dir(<node>)`, nothing in the checkout
  ([Where files are](../../../docs/reference/files.md)),
- README, `docs/`, AGENTS.md and a one-line CLAUDE.md as
  [Writing documentation](../../../docs/project/writing-docs.md#a-node-repository) describes.

Start the code from the skeleton in [Your first node](../../../docs/getting-started/first-node.md). Use the
helpers of `xblick.node` (`start`, `Reporter`, `state`, `add_arguments`), `xblick.history`,
`xblick.tensors` and, for colour with depth, `xblick.rgbd`, rather than writing your own.

## 3. Make it correct on a replay

Run it on a recording or a video, never only on a live camera, so that every later measurement has the same input:

```bash
xblick play <recording.mcap> --loop --cuda-device 0    # or a video: xblick play clip.mp4 --camera demo/cam --loop
xblick-<name> --camera <camera> --device cuda:0
xblick rerun                                           # look at the results over the images
```

Check, in this order:

1. Every result carries the stamp of the input it was computed from, never the processing time.
2. A device array is published with its CUDA stream: `publish(..., stream=torch.cuda.current_stream())`.
3. `--local auto` takes the CUDA ring when the camera is on the node's GPU (the start log says why it chose).
4. `--max-hz` limits paired streams as a pair (`RGBDInputs`, `PartnerHistory`).
5. The node reports on `<camera>/diagnostics` every second, with the problems or what it waits for.
6. Its options are parameters, the live ones applied while it runs ("Options as parameters" below). The terminal
   interface shows a node without them as offering nothing.
7. A wrong option, a missing checkpoint or an unreadable input stops the node with a message that says what to do.
   `--camera` is required: the node guesses no camera and no stream. Every default is fixed and documented, never
   chosen from the machine, the installed packages or a measurement.
8. The node survives a restart of the camera and of the router, and a camera that comes back with another size.
9. Drawn results only with `--overlay` (off by default, a start option, not a live parameter):
   `xblick.overlay.Overlay` on `<camera>/<node>/overlay`, put after the results are published, with the colour
   frame's stamp, frame id and camera_info. Without the option no publisher exists; test that
   `query_info(session, f"{camera}/<node>/overlay") == []`.

### Options as parameters

Every option becomes a parameter of `<camera>/<node>` (a node of several cameras: `<node>`), so that the terminal
interface and `xblick param` show how the node runs and `--param`/`--params-file` set any option at the start
([Parameters](../../../docs/reference/parameters.md#a-nodes-options-as-parameters)):

```python
node.Parameters.add_arguments(parser)
args = parser.parse_args(argv)
session = open_from_args(args)                      # before the model loads
params = node.Parameters(session, args.node, camera)
params.configure(args)
params.declare_options(parser, args, live=LIVE, leave_out=("camera", "node"))
params.check_overrides()
...                                                  # only now use args
```

- **Declare right after `parse_args`.** A value given with `--param` is written back into `args` there. An option the
  node used before is set too late, and nothing says so. Open the session first, also before the model loads.
- **Choose the live ones** (`LIVE = {name: declare arguments}`, with a `range` and a `unit`): thresholds, limits,
  rates of searches, prompt words, object lists. Keep read-only what sizes a model, an engine or a buffer
  (resolution, iterations baked into TensorRT, slots), and the device, checkpoint and streams.
- **Apply a live value in the thread that uses it, between frames:** copy it into `args` or set the component's
  attribute at the top of the loop (a detection thread sets its own). Never change a model from Zenoh's thread. A
  value that a library takes at construction gets a property on the node's wrapper that sets the library's attribute
  (check that the library reads it per call).
- **An object or prompt list** is checked in `write` (format, a mesh or feature file that exists), so that a wrong
  list is refused with the reason, and applied by the loop (load the new, drop the others). When the node also has a
  key for changes at run time (`<node>/objects`, `<camera>/<node>/prompts`), set the parameter after each change
  there, so that both say the same.
- **A C++ node** defines its options with `xblick::Options` (`xblick/options.hpp`) instead of parsing
  `argv` by hand: `parse`, open the session, `configure`, `declare` with the live ones' writers, then read the
  options. A writer runs in Zenoh's thread: it locks what it changes, and refuses while that does not exist yet.
- **Test it over Zenoh:** start the node as a process, read its parameters with `ParameterClient`, check that a live
  one is writable and an other read-only, that a wrong value is refused, and that a set takes effect in the results.

## 4. Measure the baseline

Use the xblick-benchmark-nodes skill: one GPU, the replay as input, 60 s after a warm-up. Record the result
topic's rate and age at send (capture to result), the GPU load and the VRAM. Then find where the time goes inside the
node: CUDA events around each stage, each stage synchronised once to time it alone, and the number of kernel
launches per frame.

## 5. Optimise

Decide first whether the node is GPU-bound (few, large kernels, high load) or launch-bound (thousands of small
kernels, low load), because the cures differ. Then try the changes that helped in other nodes, one at a time, each
measured against the baseline on the same replay:

| Change | Helps when |
| --- | --- |
| keep the frame on the GPU from the ring to the model, no CPU checks | any node that touches pixels on the CPU |
| weights in bfloat16 once at start, not autocast per frame | launch-bound models with float32 weights |
| batch per-item work (words, objects, hypotheses) | the model runs once per item |
| skip work whose result is discarded | heads computed for items that are dropped later |
| two CUDA streams for two stages | two stages that can overlap |
| TensorRT | GPU-bound models |
| `torch.compile`, CUDA graphs | launch-bound stages with a fixed shape |
| a lower input resolution, with an accuracy check against the full one | the resolution is higher than the task needs |
| the newest complete set of inputs instead of every frame | nodes that wait for partners |

Keep a table of what you tried, with numbers, including what did not help. A change that costs accuracy needs a
measured comparison, not a visual impression.

## 6. Document and commit

Write the README's "Why it is fast" section and `docs/performance.md` with the setup (GPU, input, date), the table of
changes, and what did not help. Add the node to Durchblick's [node reference](../../../docs/reference/nodes.md) and
README catalogue in the core repository.

## Gotchas

- `ImageSubscriber`'s default transport is `compressed`. Pass `transport="auto"`, or the node decodes JPEGs on the
  CPU on the camera's own host.
- `Module.to(bfloat16)` also casts buffers. SAM 3's rotary buffers lost their imaginary part, and the model then
  found nothing, without an error. Cast the parameters only.
- On a shared GPU, medians mislead. Compare variants alternately in one process, and look at the fastest frames when
  others' jobs run.
- A node that sleeps between short bursts lets the CPU clock drop. A pinned `uclamp_min` on its thread kept IVT at
  4.9 ms per pair instead of 10.4 ms.
- Looped playback moves all stamps forward on every pass. A node must accept stamps that only grow, and never assume
  that a stamp repeats.
- A Zenoh query can stay open past its timeout. Use `session.replies(...)`, never iterate `session.get(...)`.
- A queryable must reply and drop the query at once (end the callback, keep no `Query`). A node that keeps working
  inside the callback keeps the query open. At the query's timeout the caller's own session adds the error reply
  "Timeout". Since 0.5.2 a caller takes that for a second node and refuses to send commands to it ("not sent ... the
  query was still open at its timeout").
- An overlay drawn in the result loop delays every result. Hand it to `xblick.overlay.Overlay` after publishing the
  results; it drops frames instead of queueing them.
- A queryable with a Python callback runs it in zenoh-python's own thread, which is no daemon. A node that ends with
  an exception before it undeclares it (an interrupt while the model loads) then hangs for good. The parameters'
  services avoid it since Durchblick 0.3.56. Give a node's own queryables a channel (`declare_queryable(key)`, then
  `try_recv()` in the loop), and close the parameters in the loop's `finally`.

## When this skill was wrong

If a step failed or misled, propose a fix of this skill to the user after the task.
