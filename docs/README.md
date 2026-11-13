# Documentation

Start with the [README](../README.md) of the repository. This page lists every page of the documentation by its
purpose.

## Getting started

Tutorials that lead to a first result.

- [Install](getting-started/install.md)
- [Your first stream](getting-started/first-stream.md)
- [Your first node](getting-started/first-node.md)

## Concepts

Explanations of how Durchblick works and why.

- [Streams and topics](concepts/streams-and-topics.md): how streams are named and where a node puts its results
- [Transports](concepts/transports.md): how a frame reaches a subscriber, and how `auto` chooses
- [Rings](concepts/rings.md): one copy of a frame for all readers of a host
- [Frames on the GPU](concepts/gpu-frames.md): GPU memory shared between processes
- [Latest value](concepts/latest-value.md): why a stream drops frames
- [Timestamps](concepts/timestamps.md): what each time means and which times you may compare
- [Protocol](concepts/protocol.md): ROS 2 messages, the attachment, versions
- [Sessions and routers](concepts/sessions.md): how programs find each other

## Guides

Instructions for one task each.

- [Writing nodes](guides/writing-nodes.md)
- [Pairing streams and limiting the rate](guides/pairing-streams.md)
- [Making a node fast](guides/making-a-node-fast.md)
- [Sharing a GPU between nodes](guides/sharing-a-gpu.md)
- [Recording and playback](guides/recording.md)
- [Videos as camera streams](guides/video.md)
- [Point clouds](guides/point-clouds.md)
- [Viewer](guides/viewer.md)
- [Terminal interface](guides/tui.md)

## Reference

Complete lists for lookup.

- [Command line](reference/cli.md)
- [Python](reference/python-api.md)
- [C++](reference/cpp-api.md)
- [Topics](reference/topics.md)
- [Parameters](reference/parameters.md)
- [Nodes](reference/nodes.md)
- [Glossary](reference/glossary.md)

## Integrations

- [Durchblick and ROS 2](integrations/ros2.md)

## Project

- [Development](project/development.md)
- [Alternatives](project/alternatives.md)
- [Writing documentation](project/writing-docs.md)
- [Changelog](../CHANGELOG.md)

## Origin

Durchblick was started at the Computer Vision for Human-Computer Interaction Lab (cv:hci) at KIT and is developed
further in cooperation with the High Performance Humanoid Technologies Lab (H2T) at KIT.
