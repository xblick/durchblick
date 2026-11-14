---
name: xblick-run-nodes
description: >
  Use this skill when the user wants several Durchblick nodes running together for an experiment or a demo: a camera
  or a replayed recording, perception nodes on one or more GPUs, possibly on several servers or robot PCs, and a
  viewer or a recorder, also when they only describe the setup ("run SAM 3 and FoundationPose on the turntable
  recording", "bring up the vision stack on the second server"). It plans the placement, starts the routers, MPS,
  the input and the nodes in tmux, checks that everything finds everything, and stops it cleanly. Not for measuring
  speed (use xblick-benchmark-nodes) and not for the robot integration's own scenarios.
compatibility: Durchblick 0.5.x with zenohd 1.10, the node repositories with their .venv, tmux, NVIDIA GPUs
---

# Run several nodes for an experiment

Done means: every node of the plan publishes its results, `xblick diagnostics` shows every node OK, the user has
the commands to look at it and to stop it, and a stop leaves no process behind.

Background: [Sessions and routers](../../../docs/concepts/sessions.md),
[Transports](../../../docs/concepts/transports.md), [Sharing a GPU](../../../docs/guides/sharing-a-gpu.md),
[Recording](../../../docs/guides/recording.md), [Video](../../../docs/guides/video.md).

## Progress

- [ ] 1. Placement planned
- [ ] 2. Routers running on every PC
- [ ] 3. MPS daemons running where nodes share a GPU
- [ ] 4. Input running
- [ ] 5. Nodes running, in dependency order
- [ ] 6. Checked
- [ ] 7. Handed over, with the stop command

## 1. Plan the placement

Write the plan as a table and confirm it with the user before you start anything:

| Program | PC | GPU | Inputs | Outputs |
| --- | --- | --- | --- | --- |
| `xblick play rec.mcap --loop --cuda-device 0` | server-a | 0 | | `<camera>/*` |
| `xblick-sam3 --camera <camera> --words cup` | server-a | 0 | colour | `<camera>/sam3/*` |

- Check `nvidia-smi` on every PC first: other users' jobs share the GPUs. Take the VRAM per node from the
  [node reference](../../../docs/reference/nodes.md) and leave a margin.
- Put a node on the GPU of its input where you can. A node on the camera's GPU reads the CUDA ring and copies
  nothing. On another GPU of the same PC it reads the host ring. On another PC it receives JPEG over the network.
- Several nodes on one GPU need MPS, or their kernels take turns.
- A node on another PC than its camera decodes every frame itself. With several such nodes, start a
  `xblick-relay` on that PC, which decodes once and offers the stream in that PC's rings.

## 2. Routers

Every PC needs one router on port 7447. Check first whether one runs (`xblick list` answers, or a `zenohd`
process exists). Start it in tmux:

```bash
zenohd --no-multicast-scouting -l tcp/0.0.0.0:7447                                  # one PC
zenohd --no-multicast-scouting -l tcp/0.0.0.0:7447 -e tcp/<other-pc>:7447            # several PCs
```

A program on a PC without its own router connects as a client: `--zenoh-mode client --connect <pc>`.

## 3. MPS

On each GPU that more than one node shares:

```bash
xblick mps --pipe-dir /tmp/xblick-mps-gpu<N>
```

Start every node on that GPU with `CUDA_MPS_PIPE_DIRECTORY=/tmp/xblick-mps-gpu<N>`. Read
[Sharing a GPU](../../../docs/guides/sharing-a-gpu.md) for how the GPU numbering of the daemon and the clients works.

## 4. Input

Prefer a replay for an experiment, so that a run can be repeated: `xblick play <file.mcap> --loop` or a video
(`xblick play clip.mp4 --camera <camera> --loop`). Add `--cuda-device N` so that nodes on that GPU read a CUDA
ring. For a live camera, start its node (`xblick-azure-kinect`, `xblick-roboception`).

## 5. Nodes

Start each program in its own tmux window, from its repository, with its log written beside the experiment:

```bash
bash assets/start.sh plan.tsv          # see assets/start.sh: one tmux window per line of the plan
```

Or by hand:

```bash
tmux new-window -t exp -n sam3 "cd ~/code/.../xblick-sam3 && CUDA_VISIBLE_DEVICES=0 \
  .venv/bin/xblick-sam3 --camera <camera> --words cup 2>&1 | tee $LOGS/sam3.log"
```

Programs may start in any order: a node waits for its inputs and finds a restarted publisher again. Start
segmenters before the nodes that read their instances anyway, so the start log is easy to read.

## 6. Check

```bash
xblick list                       # every stream and result topic, with its transport and host
xblick diagnostics <namespace>    # every node: OK, or WARN with what it waits for
xblick rerun                      # look at the results
```

Read each node's start log for the transport it chose and why (`cuda (publisher on this GPU)`). A node that waits
names what it waits for in its diagnostics.

## 7. Hand over and stop

Give the user the tmux session name, the commands of step 6, and the stop command. Stop programs by PID with SIGTERM,
in the reverse order of the start, then quit the MPS daemons:

```bash
kill -TERM <pid>                                                       # each node, then the input
echo quit | CUDA_MPS_PIPE_DIRECTORY=/tmp/xblick-mps-gpu<N> nvidia-cuda-mps-control
```

## Gotchas

- A router does not relay between peers. A peer session that only `--connect`s listens on a free loopback port, so
  only peers on the same PC reach it. For a router on another PC, use `--zenoh-mode client` or a router per PC.
- Never stop processes by name pattern on a shared machine. Use the PIDs you started.
- A background shell starts processes with SIGINT ignored. A node on an xblick older than 0.5.4 then ignores
  SIGINT. Stop programs with SIGTERM.
- A viewer that subscribes to every stream makes every publisher produce every transport. On one robot PC this cut the
  camera from 30 to 12 frames per second. Name the streams the viewer shows.
- Whole point clouds over the network are about 177 MB/s at 30 Hz. Run a point cloud node on the reader's PC instead.
- Gated checkpoints (SAM 3.1 and others) need a Hugging Face token at setup. Check `tools/setup.sh` of the node.
- A node whose `.venv` lacks its CUDA libraries fails at its first convolution (`CUDNN_STATUS_SUBLIBRARY_LOADING_FAILED`
  on a robot PC). Put the venv's cuDNN first on `LD_LIBRARY_PATH`.

## When this skill was wrong

If a step failed or misled, propose a fix of this skill to the user after the task.
