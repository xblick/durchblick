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
| Router | A Zenoh session in router mode: `xblick router` or `zenohd`. One per PC. It lets the programs of its PC find each other and carries their data to other PCs. [Sessions](../concepts/sessions.md) |
| Sequence lock | The counter of a slot. A reader uses it to detect that the publisher replaced the frame. |
| Session | The connection of one program to Zenoh. |
| Slot | One place for one frame in a ring. |
| Stale | A frame that the publisher replaced before or while a reader read it. The reader discards it. |
| Stamp | The capture time of the data, in `header.stamp`. [Timestamps](../concepts/timestamps.md) |
| Stream | The sequence of frames from one source, named `<namespace>/<camera>/<stream>`. |
| Subscriber | The object that receives a stream through one transport. |
| Topic | One Zenoh key, the name on which samples travel. The word is ROS 2's. A stream uses several topics, one per transport. A name with wildcards, such as `test/camera/*`, is a Zenoh key expression and covers several topics. |
| Transport | The way in which a frame reaches one subscriber: `compressed`, `raw`, `shm`, `cuda`, or `auto`. Zenoh's own transports, the connections between two Zenoh sessions, are unrelated. [Transports](../concepts/transports.md) |

## Zenoh terms

Durchblick uses these Zenoh terms with Zenoh's meaning.

| Our wording | Zenoh's term | Meaning | Link |
| --- | --- | --- | --- |
| session | session | One program's connection to the Zenoh network, opened with `zenoh.open`. | [docs.rs](https://docs.rs/zenoh/1.10.1/zenoh/session/index.html) |
| peer, client, router | mode (`peer`, `client`, `router`) | A peer connects directly to other peers. A client keeps one connection to a router. A router routes data for others. | [zenoh.io](https://zenoh.io/docs/getting-started/deployment/) |
| router, `xblick router`, `zenohd` | router. `zenohd` is Zenoh's router program. | A session in router mode. Durchblick runs one per PC. | [zenoh.io](https://zenoh.io/docs/getting-started/installation/#installing-the-zenoh-router) |
| the programs of one PC | peer-to-peer region, clique | The peers below a router must all connect to each other. The router is their gateway to other routers. | [zenoh.io](https://zenoh.io/docs/getting-started/deployment/#zenoh-regions) |
| the router tells programs about each other | gossip scouting | Discovery through a session that a program connects to. Durchblick turns multicast scouting off. | [zenoh.io](https://zenoh.io/docs/getting-started/deployment/#scouting) |
| address, `--connect`, `--listen` | endpoint, locator | A locator is `<protocol>/<address>`, such as `tcp/10.0.0.5:7447`. An endpoint is a locator with optional settings after `#`. | [docs.rs](https://docs.rs/zenoh/1.10.1/zenoh/config/struct.EndPoint.html) |
| topic | key | One name on which samples travel. | [zenoh.io](https://zenoh.io/docs/manual/abstractions/#key) |
| key pattern, `--key` | key expression | A name with wildcards (`*`, `**`) that covers a set of keys. | [zenoh.io](https://zenoh.io/docs/manual/abstractions/#key-expression) |
| info record, a node's service | queryable, query, reply, error reply | A queryable answers a query with replies. An error reply carries a message. The caller's own session adds the error reply `Timeout` to a query that is still open at its timeout. | [zenoh.io](https://zenoh.io/docs/manual/abstractions/#queryable) |
| a publisher knows of a subscriber | matching status | Whether a publisher or querier has a matching subscriber or queryable. | [docs.rs](https://docs.rs/zenoh/1.10.1/zenoh/matching/index.html) |
| liveliness token | liveliness token | A key that exists while its session holds it. Others query it or subscribe to its changes. | [docs.rs](https://docs.rs/zenoh/1.10.1/zenoh/liveliness/index.html) |
| attachment | attachment | Bytes that travel beside a sample's payload. Durchblick puts JSON there. | [docs.rs](https://docs.rs/zenoh/1.10.1/zenoh/sample/struct.Sample.html) |
| encoding | encoding | `type/subtype;schema` of a payload, here `application/cdr;<message type>`. | [docs.rs](https://docs.rs/zenoh/1.10.1/zenoh/bytes/struct.Encoding.html) |
| one answer per key | consolidation | How Zenoh merges the replies to one query. Durchblick asks for no consolidation. | [docs.rs](https://docs.rs/zenoh/1.10.1/zenoh/query/enum.ConsolidationMode.html) |
| blocking publisher | congestion control `Block` | A node with a full queue waits instead of dropping. | [docs.rs](https://docs.rs/zenoh/1.10.1/zenoh/qos/enum.CongestionControl.html) |
| admin space | admin space | The keys under `@/` through which a router reports its sessions and links. | [zenoh.io](https://zenoh.io/docs/manual/abstractions/#admin-space) |
| stamp (ours) | timestamp (Zenoh's, HLC) | A router adds Zenoh's timestamp to routed samples. Durchblick's stamp is the ROS header stamp. The two are unrelated. | [zenoh.io](https://zenoh.io/docs/manual/abstractions/#timestamp) |
| host ring (ours) | shared memory (Zenoh's) | Zenoh's shared-memory payloads. Durchblick's rings do not use them. | [docs.rs](https://docs.rs/zenoh/1.10.1/zenoh/shm/index.html) |
| relay (ours) | none | Our program. Zenoh's verb for forwarding data is "route". |  |
