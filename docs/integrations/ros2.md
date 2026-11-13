# Durchblick and ROS 2

Durchblick uses the message types and conventions of ROS 2, and it needs no ROS installation. This page states
exactly what is compatible with ROS 2 and what is not.

## What is identical

| Aspect | State |
| --- | --- |
| Message definitions and bytes on the wire | identical. Tests compare every message type with an independent CDR implementation ([protocol](../concepts/protocol.md)). |
| Topic layout of a camera stream | identical to `image_transport`: `image_raw`, `image_raw/compressed`, `camera_info` |
| Transforms, diagnostics, clock | the types and roles of `/tf`, `/tf_static`, `/diagnostics` and `/clock` |
| Parameters | the messages and services of `rcl_interfaces`, with the same bytes. Only the keys follow Durchblick's naming ([parameters](../reference/parameters.md)). |
| Calibration files | the YAML format of `camera_calibration`. `CameraInfo.read_file` reads it and `write_file` writes it. |
| Recordings | MCAP files with ROS 2 schemas, which ROS 2 tools open ([recording](../guides/recording.md)) |

## What is not compatible

A ROS 2 node on `rmw_zenoh` does not see Durchblick's streams without a bridge. `rmw_zenoh` puts the domain id, the
type name and a type hash into the Zenoh key, and it announces every entity with a liveliness token. Durchblick's
keys are the plain topic names.

Durchblick's own topics for the rings (`image_raw/shm/<host>`, `image_raw/cuda/<host>/<gpu>`) have no counterpart in ROS 2.

## What could be added

- A bridge, or the key format of `rmw_zenoh`, would make the streams visible to ROS 2 nodes. Nobody has needed it
  so far.
- ROS 2 Lyrical has buffer backends (`rosidl::Buffer`), and NVIDIA provides one for CUDA. A ROS mode of Durchblick
  could offer its CUDA ring as such a backend. ROS 2 nodes, including Isaac ROS, could then read Durchblick's frames
  in place. [Alternatives](../project/alternatives.md) describes these projects.

To look at streams you need no ROS. Use [`xblick rerun`](../guides/viewer.md).
