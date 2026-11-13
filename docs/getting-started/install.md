# Install

This page shows how to install Durchblick for Python, how to build it for C++, and how to start the router that
every PC needs.

## Requirements

| What | Version | Needed for |
| --- | --- | --- |
| Linux, x86-64 | | everything. Wheels for aarch64 do not exist yet. |
| Python | 3.10 to 3.13 | the Python package and the command line |
| Zenoh router `zenohd` | 1.10 | programs on one PC that find each other without addresses |
| NVIDIA GPU with its driver | | CUDA rings. The package loads without a GPU. |
| CMake and a C++20 compiler | | C++ programs, or a build from source |
| CUDA toolkit with `nvcc` | 12.8 tested | only the point cloud node `xblick-points` |

## Python

Install the package from PyPI. Its wheels are for Linux on x86-64 with glibc 2.28 or newer and Python 3.10 to 3.13.

```bash
pip install "xblick[rerun,record]"
```

To build the package from source, use a checkout. The build needs CMake and a C++20 compiler. It needs no CUDA
toolkit.

```bash
git clone https://github.com/xblick/xblick.git
pip install "./xblick[rerun,record]"
```

Optional features have extras:

| Extra | Adds |
| --- | --- |
| `xblick[rerun]` | the viewer `xblick rerun` |
| `xblick[record]` | `xblick record` and `xblick play` for MCAP files |
| `xblick[video]` | videos as camera streams in `xblick play` |
| `xblick[cam]` | live cameras that FFmpeg opens, as camera streams: `xblick cam` |
| `xblick[relay]` | the [relay](../concepts/transports.md#relays) program, started by `xblick relay` |

In a uv project, add the package with `uv add xblick`. To use a commit that is not released, depend on the
repository at that commit. uv then builds the package from source:

```toml
[tool.uv.sources]
xblick = { git = "https://github.com/xblick/xblick.git", rev = "<tag or commit>" }
```

## C++

1. Install zenoh-c and zenoh-cpp 1.10. The script `tools/install_zenoh.sh <prefix>` installs both.
2. Build and install Durchblick with CMake.
3. Use it in your project:

```cmake
find_package(xblick 0.5 REQUIRED)
target_link_libraries(my_program PRIVATE xblick::xblick)
```

`find_package` finds Zenoh, JPEG and PNG itself. The [C++ reference](../reference/cpp-api.md) has the details, and
[development](../project/development.md) has the full build commands.

## The router

Every PC runs one Zenoh router. The pip packages do not contain it. `zenohd` is the router program of
[Eclipse Zenoh](https://github.com/eclipse-zenoh/zenoh), and its
[releases](https://github.com/eclipse-zenoh/zenoh/releases) have a standalone archive for each platform. For Linux on
x86-64 and version 1.10.1, the archive is `zenoh-1.10.1-x86_64-unknown-linux-gnu-standalone.zip`. It contains the
program `zenohd` and two plugins. Unpack it and put `zenohd` on your `PATH`.

Start the router once per PC and leave it running:

```bash
zenohd --no-multicast-scouting -l tcp/0.0.0.0:7447
```

[Sessions and routers](../concepts/sessions.md) explains what the router does and how several PCs connect.

## Check the installation

```bash
xblick test-camera &     # publishes a moving test pattern
xblick list              # shows the stream test/camera/color
```

`xblick list` prints one line per stream, with the encoding, the transports and the host. Continue with
[your first stream](first-stream.md).

## Other ways to install

To work on Durchblick itself, see [development](../project/development.md).
