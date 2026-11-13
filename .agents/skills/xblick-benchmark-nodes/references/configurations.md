# Configurations to compare

Each axis changes one thing. Change one axis at a time against a baseline.

| Axis | Values | What changes | What to expect |
| --- | --- | --- | --- |
| Nodes per GPU | one alone, several | competition for the GPU | without MPS the kernels take turns, so each node slows down more than its load suggests |
| MPS | off, on | kernels of several processes run side by side | Boxer beside nvblox and the robot mask at 60 frames/s: 62 ms latency without MPS, 30 ms with |
| GPU of the node | the camera's GPU, another GPU of the PC | CUDA ring or host ring | the camera's GPU reads the CUDA ring without a copy; another GPU reads the host ring, one copy |
| PC of the node | the camera's PC, another PC | ring or network | another PC receives JPEG (or raw) over the network and decodes each frame itself |
| Relay | none, `xblick-relay` on the node's PC | one decode for all nodes of that PC | the relay's decode cost once, instead of each node's |
| Transport over the network | `compressed`, `raw` | encode and bandwidth | raw needs a fast network; compressed costs encoding on the publisher and decoding on the reader |
| Input rate | the camera's rate, `--max-hz` of the node | frames per second the node takes | a node limited by `--max-hz` never shows more |
| Content | objects, people, words in view | the node's work per frame | keep it the same across configurations |

## Commands per axis

```bash
# one node alone on GPU 0, input on the same GPU
xblick play rec.mcap --loop --cuda-device 0
CUDA_VISIBLE_DEVICES=0 .venv/bin/xblick-<node> --camera <camera>

# several nodes on one GPU through MPS
xblick mps --pipe-dir /tmp/xblick-mps-gpu0
CUDA_VISIBLE_DEVICES=0 CUDA_MPS_PIPE_DIRECTORY=/tmp/xblick-mps-gpu0 .venv/bin/xblick-<node> ...

# a node on another PC, through the routers
zenohd --no-multicast-scouting -l tcp/0.0.0.0:7447 -e tcp/<camera-pc>:7447      # on the node's PC
.venv/bin/xblick-<node> --camera <camera>                                        # takes compressed automatically

# the same with a relay on the node's PC
xblick-relay            # relays what the nodes on this PC read from other PCs; start it before or after them
```

Measure on the PC of the node for the age at send, and on any PC for the rate. The age at receive measured on another
PC includes the network and the clock offset.
