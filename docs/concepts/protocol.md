# Protocol

The protocol is the agreement between publishers and subscribers about the bytes on a topic. This page explains
which messages Durchblick sends, how it adds its own fields, and how versions protect you from misread data.

## ROS 2 messages without ROS

Every payload is a ROS 2 message, for example `sensor_msgs/msg/Image` or `vision_msgs/msg/Detection3DArray`.
Durchblick serialises it as CDR, the binary format of ROS 2, in little-endian byte order with the 4-byte
encapsulation header. The bytes are identical to the bytes that ROS 2 writes.

We chose ROS 2 messages because engineers know them and tools read them. Durchblick does not need a ROS
installation. It has its own implementation of the messages in Python and in C++.

Two tests keep this promise. `tests/test_cdr.py` compares the bytes of every message type with
[rosbags](https://gitlab.com/ternaris/rosbags), which uses the definitions of ROS 2 Lyrical. A second test compares
the C++ messages with the Python bytes.

## The attachment

A ROS 2 header has a stamp and a frame id. Durchblick needs three more fields per sample: a sequence number, the publish
time and the source. These fields travel beside the payload in Zenoh's
[*attachment*](https://docs.rs/zenoh/1.10.1/zenoh/sample/struct.Sample.html), as a small JSON object:

```json
{"v": 1, "seq": 4711, "publish_ns": 1794300000000000000, "source": "robot", "publish_called_ns": 1794299999990000000}
```

A reader takes the fields that it knows and ignores the others. [Timestamps](timestamps.md) explains each field.

## One definition, with versions

`cpp/include/xblick/protocol.hpp` defines the whole protocol: the topic names, the message types and their
versions. The Python package reads these values from the C++ core, so both languages always agree.

Every sample names its schema in its Zenoh [encoding](https://docs.rs/zenoh/1.10.1/zenoh/bytes/struct.Encoding.html):

```
application/cdr;sensor_msgs/msg/Image                 a ROS 2 type
application/cdr;xblick_msgs/msg/ShmImage@1            a Durchblick type, version 1
```

Durchblick's own types are the descriptors for the rings. Their version changes whenever their layout changes. A
subscriber that receives another version raises `ProtocolError` and names both versions. It does not try to read
the sample. The rings and their handshake carry versions in the same way.

For you this means: all publishers and subscribers of one setup speak the same protocol. The
[changelog](../../CHANGELOG.md) states which versions do.

## Related pages

- [Durchblick and ROS 2](../integrations/ros2.md): what is compatible with ROS 2 and what is not.
- [Topic reference](../reference/topics.md): which type travels on which topic.
- [Development](../project/development.md): what you must do when you change a message.
