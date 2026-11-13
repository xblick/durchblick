# Development

How to build, test and release Durchblick, and where the code is.

## Build and test the Python package

```bash
uv sync --all-extras                      # builds the C++ core with CMake and a C++20 compiler
CUDA_VISIBLE_DEVICES=0 uv run pytest      # the CUDA tests are skipped without a GPU
ruff check --line-length 120 src tests
```

The package builds with scikit-build-core. `cuda.h` comes from the pip package `nvidia-cuda-runtime`, so the build
needs no CUDA toolkit. Python 3.10 or newer is required.

`uv sync` rebuilds the C++ core when a file in `cpp/` changed. An editable install in another project does not.
There, run `uv sync --reinstall-package xblick` after a change in `cpp/`.

## Build the C++ library

```bash
tools/install_zenoh.sh /opt/zenoh       # zenoh-c and zenoh-cpp 1.10
cmake -S . -B build/cpp -G Ninja -DCMAKE_PREFIX_PATH=/opt/zenoh \
      -DXBLICK_CUDA_INCLUDE_DIR=$(uv run python -c "import nvidia, os; print(os.path.join(list(nvidia.__path__)[0], 'cu13', 'include'))")
cmake --build build/cpp && ctest --test-dir build/cpp
XBLICK_CPP_BUILD=build/cpp uv run pytest tests/test_cpp_interop.py        # C++ against Python
```

## Build the point cloud node

The point cloud node needs a CUDA toolkit with `nvcc`. It is not part of the wheel. Its end-to-end test needs a
GPU. CMake looks for `nvcc` on `PATH`. If `nvcc` is not on `PATH`, name it with `-DCMAKE_CUDA_COMPILER`.

```bash
cmake -S . -B build/points ... -DXBLICK_POINTS=ON -DCUDAToolkit_ROOT=<toolkit> \
      -DCMAKE_CUDA_COMPILER=<toolkit>/bin/nvcc
XBLICK_CPP_BUILD=build/points CUDA_VISIBLE_DEVICES=0 uv run pytest tests/test_point_cloud_node.py
```

The test checks every point independently of the node's own computation. It projects each point with the
distortion of the depth camera and expects the point on its own pixel. It expects the colour of the pixel that the
point's projection into the colour camera meets.

## A test that hangs

A hanging test does not block the test session.

| After | What happens |
| --- | --- |
| 90 s | `faulthandler` prints the Python stack of every thread. |
| 120 s | `pytest-timeout` prints the stacks again and ends the session. |

The timeout uses `timeout_method = "thread"`. This method also ends a test that is blocked in a C call, for example
a Zenoh call that does not return. A signal would wait for the call.

- A test that needs longer gets `@pytest.mark.timeout(seconds)`.
- To examine a hang by hand, run pytest with `-o timeout=0`.
- The stacks are Python stacks. For the native threads of Zenoh, run `py-spy dump --native --pid <pid>` during the
  hang. The CI containers do not allow this, because it needs ptrace.

## Change the protocol

| You change | You must raise |
| --- | --- |
| the fields of a Durchblick message | its version in `cpp/include/xblick/protocol.hpp` |
| the layout of a ring | `kVersion` in `cpp/include/xblick/ring.hpp` |

Old and new processes then refuse each other. Raise the minor version of the package in the same change, so that
users see the break.

## CI and releases

The pipeline is `.github/workflows/ci.yml` on GitHub Actions. Its runners have no GPU, so the CUDA tests skip. The
pipeline runs:

- the reference check `tools/check_references.sh`,
- the Python tests on 3.10 and 3.12,
- the C++ library with its tests,
- a project that uses the installed package,
- C++ against Python,
- manylinux wheels for Python 3.10 to 3.13 and the sdist, built by `tools/build_wheels.sh`.

The reference check fails on names of internal systems, hosts and paths in any tracked text file. An exception goes
into `tools/reference_allowlist.txt`, as a whole file or as a file and a text that the line contains.

`tools/build_wheels.sh <out dir> <work dir>` builds the files for PyPI in the image
`quay.io/pypa/manylinux_2_28_x86_64`. It builds a wheel for each Python from 3.10 to 3.13 and repairs it with
auditwheel. It also builds the sdist and imports each wheel in a new environment.

To release a version:

1. Set the version in `CMakeLists.txt` (`project(xblick VERSION x.y.z)`). Python reads it from there.
2. Add the entry to `CHANGELOG.md` and update the status in `README.md`.
3. Merge, then tag `vx.y.z`. The pipeline checks that the tag names the version and builds the wheels.
4. The tag also starts `.github/workflows/publish.yml`. It builds the same files and uploads them to PyPI with
   trusted publishing. PyPI's publisher for `xblick` names the owner `xblick`, the repository `xblick`, the
   workflow `publish.yml` and the environment `pypi`.

The extra `xblick[relay]` names the package `xblick-relay`. `uv lock` resolves every extra, so it fails while no
index has the version it asks for. A new minor version of `xblick-relay`: publish it first, then run `uv lock` here
and release `xblick`.

Node repositories pin a version of `xblick`. After a release that a node needs, raise its pin and run `uv lock`.

## Layout

| Path | Content |
| --- | --- |
| `cpp/include/xblick/protocol.hpp` | the wire protocol: topics, message types and versions |
| `cpp/include/xblick/ring.hpp`, `cuda_ring.hpp` | the rings: memory layout, sequence lock, CUDA handshake |
| `cpp/include/xblick/msg.hpp`, `image.hpp`, `camera.hpp` | C++ messages, encodings and codecs, the camera API |
| `cpp/include/xblick/parameters.hpp` | parameters of a node |
| `cpp/include/xblick/points.hpp`, `point_cloud_node.hpp` | point cloud streams, the point cloud node |
| `cpp/src/stream.hpp` | what image streams and point cloud streams share: the choice of the transport, the access to a ring |
| `cpp/points/` | the point cloud node: camera model and rays, CUDA kernel, the program `xblick-points` |
| `cpp/tools/` | `xblick-cpp-camera`, `xblick-cpp-probe` |
| `cpp/python/bindings.cpp` | `xblick._core` |
| `src/xblick/` transport | `cdr.py`, `msg.py`, `encodings.py`, `codecs.py`, `topics.py`, `session.py`, `camera.py`, `points.py`, `detections.py`, `robot.py`, `sim.py`, `poses.py`, `gpu.py`, `mps.py` |
| `src/xblick/` for nodes | `node.py`, `logs.py`, `diagnostics.py`, `history.py`, `tensors.py`, `rgbd.py`, `paths.py`, `parameters.py` |
| `src/xblick/` tools | `cli.py`, `cam.py` with `v4l2.py`, `rerun_bridge.py`, `recording.py` with `msgdefs.py` and `video.py`, the terminal interface `tui.py` with `survey.py`, `tui_graph.py` and `tui_parameters.py` |
| `tools/gen_msgdefs.py` | generates `msgdefs.py` |
| `tests/`, `cpp/tests/` | Python tests with `test_cpp_interop.py`, C++ tests |
| `.github/workflows/ci.yml` | the CI |
| `tools/check_references.sh`, `tools/reference_allowlist.txt` | the check that no internal names enter the repository, and its exceptions |
| `docs/` | the documentation. [Writing documentation](writing-docs.md) has its rules. |
