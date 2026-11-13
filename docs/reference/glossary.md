# Glossary

The fixed terms of Durchblick. The documentation uses each term with exactly this meaning.

| Term | Meaning |
| --- | --- |
| Attachment | A small JSON object that travels beside a payload. It holds the sequence number, the publish times and the source. [Protocol](../concepts/protocol.md) |
| Camera | One device. Its streams share one clock and one set of coordinate frames. |
| CUDA ring | A ring in GPU memory that several processes map. Transport `cuda`. [Frames on the GPU](../concepts/gpu-frames.md) |
| CUDA stream | A queue of work on a GPU. The GPU executes the work of one CUDA stream in order. It is unrelated to a camera stream. |
| Descriptor | The small message that states where a frame is in a ring: the ring, the slot and the counter. |
| Device array | An array in GPU memory, for example a PyTorch tensor on `cuda:0`. |
| Frame | One image or one point cloud of a stream. |
| Host ring | A ring in POSIX shared memory. Transport `shm`. [Rings](../concepts/rings.md) |
| Info record | The answer of a publisher to a query on `<stream>/info`: host, version, transports, encoding. |
| Kernel | One function that a GPU executes. A network forward pass consists of many kernels. |
| Namespace | The first part of a stream's name. It names the system that the camera belongs to, usually a robot. |
| Node | A program that reads streams and publishes results about them. |
| Publisher | The object, or the program, that offers a stream. |
| Reader | A subscriber that reads frames from a ring. |
| Relay | A program, one per PC, that receives the streams which nodes on its PC read from other PCs once each and offers them in rings on its PC. |
| Ring | A fixed set of slots in memory that the publisher fills in turn. Never called a buffer or a queue. |
| Router | The Zenoh program `zenohd`. One per PC. It lets programs find each other. [Sessions](../concepts/sessions.md) |
| Sequence lock | The counter of a slot. A reader uses it to detect that the publisher replaced the frame. |
| Session | The connection of one program to Zenoh. |
| Slot | One place for one frame in a ring. |
| Stale | A frame that the publisher replaced before or while a reader read it. The reader discards it. |
| Stamp | The capture time of the data, in `header.stamp`. [Timestamps](../concepts/timestamps.md) |
| Stream | The sequence of frames from one source, named `<namespace>/<camera>/<stream>`. |
| Subscriber | The object that receives a stream through one transport. |
| Topic | One Zenoh key. A stream uses several topics, one per transport. |
| Transport | The way in which a frame reaches one subscriber: `compressed`, `raw`, `shm`, `cuda`, or `auto`. [Transports](../concepts/transports.md) |
