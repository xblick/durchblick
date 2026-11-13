# Sharing a GPU between nodes

Several nodes on one GPU slow each other down more than their load suggests. NVIDIA's Multi-Process Service (MPS)
removes most of this effect. This guide explains the reason and shows how to run nodes under MPS.

## Why nodes on one GPU wait for each other

By default, a GPU serves one process at a time. It gives each process a time slice and then switches to the next.
During the slice of one process, the kernels of all other processes wait, even when the GPU has free capacity.

MPS puts one server process between the nodes and the GPU. All nodes send their work to this server, and the GPU
sees a single process. The kernels of different nodes then run side by side.

| Boxer beside nvblox and the robot mask, 60 frames per second | Latency of Boxer |
| --- | --- |
| without MPS | 62 ms |
| with MPS | 30 ms |

The measurement and its setup are in durchblick-mapping (`docs/performance.md`).

## Start the daemon

```bash
blick mps --pipe-dir /tmp/durchblick-mps
```

The command runs NVIDIA's MPS control daemon for as long as it runs itself. When you stop the command, it sends the
daemon `quit`. A wrong `--config` ends the command before the daemon starts.

The command announces itself on Zenoh through the router of the PC:

| Key | Content |
| --- | --- |
| `durchblick/mps/<name>` | a liveliness token. `<name>` is the host name, or `--name`. |
| `durchblick/mps/<name>/status` | a queryable with JSON: the pipe and log directories, the PID of the daemon, `CUDA_VISIBLE_DEVICES`, the GPUs of the PC, and its servers with the PIDs and commands of their clients |

The terminal interface shows the daemon under "Nodes". Without a router the command works all the same. It writes
this to its log, and again when it reaches a router.

## Run a node as a client

Set the pipe directory in the environment of every node:

```bash
CUDA_MPS_PIPE_DIRECTORY=/tmp/durchblick-mps durchblick-boxer ...
```

A node that is meant to run under MPS calls `blick.mps.check()` at start. The node refuses to run when the
daemon is missing. This check is necessary, because CUDA itself would run the node without MPS and without any
message.

**GPU numbering.** The `CUDA_VISIBLE_DEVICES` of a client counts among the GPUs of the daemon. If the daemon sees
only GPU 2, a client names it as 0. On a PC with one GPU, set the variable for neither.

## What happens when the daemon stops

| Event | Effect on the clients |
| --- | --- |
| The daemon receives `quit` (a normal stop of `blick mps`) | A running client continues to work. |
| The daemon or its server receives SIGTERM or SIGKILL | Every CUDA call of the clients fails from then on with `cudaErrorMpsRpcFailure`. Restart the clients. |

## Related pages

- [Making a node fast](making-a-node-fast.md): the other changes that helped.

## Status and limits

- Tested on 2026-10-30 on one host, with the daemon on one GPU of a server and a replayed recording as the camera.
  The nodes ran as MPS clients at 28-30 frames per second.
- Not tested on a robot yet.
