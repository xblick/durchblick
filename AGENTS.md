# AGENTS.md

Guidance for coding agents in this repository. Human documentation: `README.md` (entry) and `docs/` (concepts,
guides, reference). Do not duplicate it here. Style rules for documentation: `docs/project/writing-docs.md`.

## What this is

Durchblick: camera streams and perception results between programs, over Zenoh, as ROS 2 message types (CDR),
without ROS. C++20 core in `cpp/` (Python bindings `xblick._core`), Python package `xblick` in `src/xblick/`
(command `xblick`). Each perception method is a node in its own repository (`xblick-<name>`) that depends on this
package. Robot integrations live in repositories of their own. The core never depends on a robot framework.
Zenoh 1.10 docs: zenoh.io/docs (concepts, deployment), API docs.rs/zenoh/1.10.1, config keys DEFAULT_CONFIG.json5 at
tag 1.10.1 (github.com/eclipse-zenoh/zenoh).

Names: every name in code, on the wire and on disk is `xblick` (since 0.5.0): Python package, distribution and
command, C++ namespace, include directory, CMake package and targets, C++ programs, `XBLICK_*` variables, Zenoh keys,
ring names, message types (`xblick_msgs`), directories. Only the prose name of the project is "Durchblick". A new
name follows this rule. Renaming a wire or on-disk name breaks every older program: only with a minor version.

## Skills

Project skills live in `.agents/skills/` (open Agent Skills format); `.claude/skills` links to it. Rules for writing
one: `.agents/skills/README.md`.

| Skill | For |
| --- | --- |
| `xblick-build-node` | a new vision node, or making a node faster or leaner |
| `xblick-run-nodes` | several nodes for an experiment or a demo, on one or more GPUs and PCs |
| `xblick-benchmark-nodes` | throughput, latency, GPU load and memory of nodes and setups |
| `xblick-develop-tui` | the terminal interface (`xblick`): tabs, dialogs, layout, its tests and screenshots |

- Before a task, look for a matching skill and follow it. Read its `SKILL.md` in full.
- After a complex task that is likely to recur and can be abstracted (no batch names, dates or results of one
  experiment), propose a new skill or a fix of an existing one in two or three lines. Write it only after the user's
  yes. Prefer improving a skill over adding one.
- Rules for every task stay in this file; a rule that must always hold belongs in a hook; a skill holds a procedure.
- Node repositories have no skills of their own. An agent working in one uses the skills of this repository.

## Commands

```bash
uv sync --all-extras                      # builds the C++ core (scikit-build-core, C++20); rebuilds when cpp/ changes
uv sync --reinstall-package xblick         # force the rebuild
CUDA_VISIBLE_DEVICES=0 uv run pytest -q   # ~5 min, ~633 tests (2026-11-13); CUDA tests skip without a GPU
uv run pytest -q tests/test_recording.py  # one file
ruff check --line-length 120 src tests    # lint; ruff is not a project dependency
```

C++ library, tools, point cloud node, C++ against Python: `docs/project/development.md`.
CI: `.github/workflows/ci.yml` (reference check, Python 3.10 and 3.12, C++ tests, interop, manylinux wheels).
PyPI files: `tools/build_wheels.sh <out> <work>` in quay.io/pypa/manylinux_2_28_x86_64 (CI job wheels, `.github/workflows/publish.yml`: tag v<version>, trusted publishing, environment pypi).
Reference check: `tools/check_references.sh` (no internal or robot-framework names outside its allowlist).

## Rules

- Fail fast. A wrong configuration, a missing input or a protocol mismatch raises with a message that says what to
  do. Add no default that hides a misconfiguration.
- Protocol: a change of a Durchblick message's fields raises its version in `cpp/include/xblick/protocol.hpp`.
  A ring layout change raises `kVersion` in `ring.hpp`. Either one also raises the package's minor version.
- Version: only in `CMakeLists.txt` (`project(xblick VERSION x.y.z)`). Every user-visible change: patch bump,
  entry in `CHANGELOG.md`, status line in `README.md`. Release: tag `vx.y.z`, CI builds the wheels.
- Python and C++ stay equivalent. A feature of the camera, points or detections API exists in both.
  `tests/test_cpp_interop.py` checks it.
- Documentation changes in the same commit as the code. New topic or option: update `docs/reference/`. History goes
  to `CHANGELOG.md`, never into a page.
- A node stamps a result with the capture stamp of its input, never with the processing time.
- Code style: match the surrounding code. Docstrings say what, comments say why. 120 columns. No new dependency
  without need.

## Traps

| Symptom | Cause and fix |
| --- | --- |
| Programs find nothing | A Zenoh 1.10 router does not route between peers (Zenoh 1.9+: peers of a router form a clique; failover brokering removed); peers connect to each other's listen addresses (locators). A peer with `--connect` and without `--listen` listens on a free loopback port (session.py, C++ `open_session`), which peers on other PCs or behind a tunnel cannot reach. Across PCs: `--zenoh-mode client`, a router per PC, or `--listen tcp/0.0.0.0:0` (open to the whole network). Commands, nodes (`open_from_args`) and the C++ tools (`SessionOptions::program`) print one line at the start when a peer connects only to other PCs and listens on loopback only (`session.across_pcs`, C++ `across_pcs`); `session.why_nothing` adds it to the empty-result messages. The terminal interface refuses a peer that only connects. |
| A query finds nothing right after start | A new session needs time to learn of queryables. Reads: `session.ask` (C++ `xblick::ask`), asked again until a reply. Commands (a set, a controller switch, a recorder command): `session.command` / `ask_once` (C++ `xblick::ask_once`), sent once to each queryable after one is known (Zenoh's default query target BestMatching reaches all while none is declared complete: never set `complete` on a queryable or a `target` on these queries, tests/test_queries.py pins it); first a read of the same node must have exactly one answer and no "Timeout" reply, else not sent (best effort: a node that drops the read, starts after it or serves a broad key still gets it; a session just opened links to the serving sessions one after the other, so its read can miss a second node or reach only a busy one: "no node answered in time"; tests wait until a read reaches all, `knows_all` in tests/test_queries.py). Never send a command again after an empty result: a queryable that dropped it also gives one. `NotReached` (a TimeoutError): not sent. `NoAnswer` (no TimeoutError): sent once, may still run. `PartlyAnswered` (a NoAnswer): one answered, the query still open at its timeout. `SeveralExecuted` (a NoAnswer): several answered and executed. |
| A command or set is refused with "the query was still open at its timeout" although one node serves it | The node keeps the query after its reply (works inside the queryable's callback, or holds the Query): the query stays open, and at its timeout the caller's own session adds the error reply "Timeout" (nothing comes over the network), taken for a second node. A receiver replies and drops the query at once; long work outside the callback. xblick's Parameters and recorder do; check external receivers (the robot's bridge). |
| A command runs after the caller gave up | A command has no expiry on the wire: a busy or briefly unreachable receiver executes it when it gets it, also after `NoAnswer`. Known limitation; a deadline the receiver checks needs a protocol change. |
| A loop over query replies hangs for minutes | Zenoh does not always finalize a query at its timeout. Never `for r in session.get(...)` or `liveliness().get`. Use `session.replies(session, selector, timeout, ...)` (C++ `xblick::replies`). |
| `Session.close()` never returns | Happens after a query Zenoh did not finalize. Open sessions with `open_session` or `open_from_args` (close waits at most 5 s). Wrap an own `zenoh.open` in `session.Session(...)`. |
| A subscriber gets JPEG on the publisher's host | `ImageSubscriber` defaults to `transport="compressed"`. Pass `transport="auto"`. |
| A test cannot open its own CUDA ring | The creating process cannot open a CUDA ring. Read it from a `spawn`ed child process. |
| Device frames arrive wrong or late | `publish()` of a device array needs `stream=` (for PyTorch `torch.cuda.current_stream()`). Without it the publisher waits for all GPU work of the process. |
| Nodes misbehave on a looped playback | `xblick play --loop` moves all stamps forward on each pass. Nodes rely on stamps that only grow. |
| A test or harness never ends | Zenoh threads keep a process alive. Close sessions. Run long checks in the background with `timeout`. A shell script starts background jobs with SIGINT ignored: `xblick` commands, `node.start` (`node.end_on_signals`) and the C++ tools end on it anyway, other programs and nodes on an older xblick may not. Stop by PID with SIGTERM. Never kill by name pattern. |
| Large writes fail | NFS. Build and write large outputs on a local disk. |

## Layout

| Path | Content |
| --- | --- |
| `cpp/include/xblick/protocol.hpp` | wire protocol: topics, message types, versions |
| `cpp/include/xblick/ring.hpp`, `cuda_ring.hpp` | host ring and CUDA ring |
| `cpp/include/xblick/camera.hpp`, `points.hpp`, `msg.hpp`, `parameters.hpp` | C++ API |
| `cpp/include/xblick/rectify.hpp`, `src/xblick/rectify.py` | rectification map from a CameraInfo (R, P), CPU remap; same in C++ and Python |
| `cpp/include/xblick/pairing.hpp` | `partner_of`: partner sample of a primary frame by stamp; Python's `PartnerHistory` calls it via `_core` |
| `cpp/include/xblick/paths.hpp`, `src/xblick/paths.py` | cache, data and config directories; same rule in C++ and Python (`tests/test_paths.py`) |
| `cpp/points/` | point cloud node `xblick-points` (CUDA) |
| `cpp/python/bindings.cpp` | `xblick._core` |
| `src/xblick/camera.py`, `points.py`, `detections.py`, `session.py` | Python API: streams, sessions |
| `src/xblick/node.py`, `history.py`, `tensors.py`, `rgbd.py`, `parameters.py` | helpers for nodes |
| `src/xblick/cli.py`, `tui.py`, `tui_graph.py`, `rerun_bridge.py`, `recording.py`, `video.py`, `cam.py`, `v4l2.py` | tools |
| `src/xblick/robot.py`, `sim.py`, `controller.py`, `poses.py`, `mps.py` | robot state, simulation, controller link, poses, MPS |
| `tests/`, `cpp/tests/` | Python and C++ tests |
| `tools/` | `check_references.sh` (+ `reference_allowlist.txt`), `install_zenoh.sh`, `gen_msgdefs.py`, `setup_python.sh`, `tui_screenshots.py` |
| `.github/workflows/ci.yml` | CI |
| `docs/` | `getting-started/`, `concepts/`, `guides/`, `reference/`, `integrations/`, `project/` |
| `.agents/skills/` | agent skills; `.claude/skills` links here |

## Defaults

- Router: `xblick router` or `zenohd` 1.10, one per PC, listening on `tcp/0.0.0.0:7447`. Sessions connect to
  `127.0.0.1:7447`. Start: `xblick router`, or `zenohd --no-multicast-scouting -l tcp/0.0.0.0:7447`.
- Files: cache `~/.cache/xblick/<node>/`, data `~/.local/share/xblick/<node>/`, config `~/.config/xblick/`;
  `XBLICK_HOME` moves cache and data (docs/reference/files.md).
- Stream name: `<namespace>/<camera>/<stream>`. Node results: `<camera>/<node>/<name>`. Diagnostics:
  `<camera>/diagnostics`.
- Transports: `auto`, `compressed`, `raw`, `shm`, `cuda`. Nodes and the CLI use `auto`.
- Image ring: 32 slots. Point cloud ring: 4 slots.
- Test without a camera: `xblick test-camera` publishes `test/camera/color` (640 x 480, 15 Hz).
  `xblick play <file.mcap> --loop` replays a recording.
- Inspect: `xblick list`, `xblick hz <stream>`, `xblick diagnostics <namespace>`, `xblick rerun`, `xblick`
  (terminal interface).
- Two programs without a router (tests): one with `--listen tcp/127.0.0.1:<port>`, the other with
  `--connect tcp/127.0.0.1:<port>`.

## Node repositories

Rules every node keeps (docs/guides/writing-nodes.md, "Rules every node keeps"): the name identifies the model
(`xblick-yolox`, not `xblick-detect`); `--camera` explicit and required, nothing guessed; fixed, documented defaults
only (nothing that depends on the machine, the installed packages or a measurement; convenience goes into the `xblick`
command); weights downloaded at the start through `xblick.weights` with `--download-only`, `--no-download` and
`--weights-dir`, the terms notice before a download; weights and built files (engines, kernels) in
`xblick.paths.cache_dir(<node>)` (`~/.cache/xblick/<node>/`), what a user made in `data_dir(<node>)`, never in the
checkout, `<node>` the short name (`yolox`), former places named in `Store(former=...)` (docs/reference/files.md); a
queryable replies and drops the query at once; drawn results only with `--overlay` (off by default, a start option):
`<camera>/<node>/overlay`, rgb8, the colour frame's size, stamp, frame id and camera_info, results published first,
drawn by `xblick.overlay.Overlay` only while subscribed.

Same pattern in each: a uv project pinned to a version of `xblick` (`pyproject.toml`, `uv.lock`), `tools/setup.sh`
for `.venv` and checkpoints, a console script `xblick-<name>`, outputs below `<camera>/<node>/`, diagnostics on
`<camera>/diagnostics`, its own `AGENTS.md`. A change here that nodes need: release it, then raise their pin and
run `uv lock`.
