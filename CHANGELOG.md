# Versions

*Durchblick's [overview](README.md).*

Readers and publishers of one setup run the same minor version (0.3.x): the info record's version changed with 0.3.0.
Within 0.3.x, 0.3.59 changed the rings' keys and the info record's version (3): every program of a setup runs 0.3.59
or newer. 0.4.0 speaks the protocol of 0.3.59 to 0.3.68: its programs and those of 0.3.68 work together.

* **0.4.0** The Python package and its command are `blick`: the distribution `blick` (`pip install blick`), `import
  blick` (was `import durchblick`), the extension `blick._core`, the command `blick` (the command `durchblick` is
  gone). **Breaking for Python code and scripts:** replace `durchblick` by `blick` in imports, dependencies and
  command lines. The logger is `blick`. Unchanged: the protocol and every name on the wire or on disk (Zenoh keys,
  ring names, `durchblick_msgs`, the info record's `durchblick` version field, the recorder's node namespace
  `durchblick`, the MCAP library name, the presets directory), the `DURCHBLICK_*` variables, and the C++ side
  (namespace, include directory, CMake package and targets, tools). `blick mps` still prints `durchblick mps: ready`.
  The C++ package version is 0.4.0, so `find_package(durchblick 0.3...)` no longer accepts it (the version file
  requires the same minor version): ask for 0.4. The repository has no robot framework in it: the ArmarX integration
  lives in durchblick_armarx. Examples use the namespace `robot` (`blick test-camera` publishes
  `robot/test_camera/color`). CI on GitHub Actions with a check that no internal
  names enter the repository (`tools/check_references.sh`). Licence: Apache-2.0 (`LICENSE`, `NOTICE`).
* **0.3.68** Pairing by stamp without a wasted frame, also with an explicit offset and in C++: from a steady partner
  (the steps of its kept stamps within a tenth of their median), a nearest sample at most a quarter of the smallest step
  before the primary stamp pairs at once, with the default offset and with an explicit `max_offset_ns`. A partner that
  skips camera frames (a node's depth of some frames) is not steady and waits, as 0.3.65 did: its next sample may be the
  primary frame's own. 0.3.66 judged by the median step alone, and it kept waiting for the next sample (a whole period)
  with an explicit offset. durchblick-objectpoints (`--max-offset-ms 5`) on Fast-FoundationStereo depth stamped 1.7 ms
  before the colour frame waited 67 ms per frame at 15 Hz. The rule is C++ now: `partner_of` and `partner_period_ns` in
  `durchblick/pairing.hpp` (`durchblick::core`, in Python `_core.partner_of`). `PartnerHistory` and the point cloud node
  call it, so both pair alike. Before, the point cloud node waited for a later colour sample whenever the nearest one
  lay before the depth stamp, and it compared with the last sample to arrive instead of the newest stamp. Until two
  samples give a period, a partner waits as before. The point cloud node refuses a negative `max_offset`. Documentation:
  Fast-FoundationStereo stamps its depth and disparity with the left frame's stamp, no longer with the mean of the
  pair's stamps, so depth and left image pair by equal stamps. ArmarX integration: see durchblick_armarx.
  Fix: the C++ `CameraPublisher` and `PointCloudPublisher` undeclare their `<base>/info` (and rays) queryables and
  their liveliness token first when destroyed. Before, a query answered during the destruction read freed members, because a CUDA ring's server takes up
  to 200 ms to stop. durchblick-relay 0.2.1 stopped with `std::length_error`. Compatible with 0.3.67.
* **0.3.67** ArmarX integration: see durchblick_armarx. Durchblick itself is unchanged; compatible with 0.3.66.
* **0.3.66** Pairing by stamp without a wasted frame: `PartnerHistory.partner_of` with the measured offset (the
  default) takes a nearest partner sample that lies just before the primary stamp at once, because the partner's next
  sample comes at least half its period later and cannot be nearer. It waited for that next sample, a whole period.
  On a stereo pair whose right camera stamps 0.24 ms before the left one, durchblick-fast-foundationstereo's depth came
  one frame (67 ms at 15 Hz) later than needed. An explicit `max_offset_ns` keeps the old rule. Compatible with 0.3.65.
* **0.3.65** Rectification in C++ and Python: `durchblick/rectify.hpp` (`rectify_map`, `remap`,
  `rectified_info`, in `durchblick::core` without the messages) and `durchblick.rectify` (`rectify_map`,
  `rectified_info`) compute the map of OpenCV's `initUndistortRectifyMap` from a CameraInfo: its R (a stereo pair's
  rotation), its P (with a baseline), plumb_bob, rational_polynomial and equidistant. The map equals OpenCV's within
  0.002 px. `remap` interpolates 8-bit images bilinearly and takes the nearest pixel of 16-bit ones. For nodes that
  rectify, such as durchblick-rectify. ArmarX integration: see durchblick_armarx. Compatible with 0.3.64.
* **0.3.64** Compressed point clouds in C++: `CloudPublisherOptions::rays` (`CloudRays`), `set_rays()`,
  `Transport::Compressed` and `CloudSubscriberOptions::remote` for clouds, as in Python. The point cloud node
  (`durchblick-points`) passes the rays of its depth camera, so a reader on another PC takes
  its cloud as ranges and colours, about a twentieth of the whole cloud (`--range-step`, `--jpeg-quality`). C++ and
  Python read each other's compressed clouds bit for bit. Compatible with 0.3.63.
* **0.3.63** Compressed point clouds: a `PointCloudPublisher` with `rays` (the direction of each point's ray)
  offers `cloud/compressed`, the cloud as the ranges of its points (16 bit, RVL) and their colours (JPEG of quality
  95, or PNG). The rays go once to each subscriber (`cloud/rays`). A Kinect cloud of 6.19 MB becomes 0.33 MB, in
  5 ms. `PointCloudSubscriber` with `auto` takes it from another PC where the publisher offers it (`remote`).
  Python only (C++: 0.3.64). Compatible with 0.3.62, whose readers take `raw`.
* **0.3.62** RVL for 16-bit depth on `image_raw/compressed`: `codec="rvl"` (C++ `kRvl`, `rvl.hpp`), lossless, on
  Kinect depth nine times as fast to encode as PNG (1.3 against 12.3 ms) and a fifth smaller. The message is ROS's
  `compressedDepth` with the format `rvl` (`16UC1; compressedDepth rvl`). Depth from a video (`durchblick play`)
  goes out as RVL. PNG stays for label images. **A reader older than 0.3.62 stops with an error on an RVL
  stream:** update the readers before the cameras.
* **0.3.61** ArmarX integration: see durchblick_armarx. The libraries are those of 0.3.60.
* **0.3.60** A subscriber with `auto` asks the relay of its host for a stream: it holds the liveliness token
  `<base>/want/<host>/<id>` while the stream has no source with a ring on its host (`topics.want`,
  `topics.wanted_on`, `topics.wanted_stream`; C++ `protocol::kWant`). durchblick-relay 0.2.0 runs once per PC and
  relays the streams wanted there, without being told which. Compatible with 0.3.59: a publisher or relay of 0.3.59
  ignores the token.
* **0.3.59** The rings' keys name the ring's place, and a subscriber keeps its transport up to date. **Protocol
  change: info record version 3; publishers and subscribers of 0.3.58 and older do not work with it.**
  Keys: `<base>/image_raw/shm/<host>` and `<base>/image_raw/cuda/<host>/<gpu>` (`cloud/...` alike), `<host>` the
  host's id and `<gpu>` the GPU's UUID (`topics.shm(base, host)`, `topics.cuda(base, host, gpu)`,
  `topics.ring_place`; C++ `protocol::kAnyGpu`): a descriptor reaches only readers on the ring's host, and
  publishers on several hosts (a source and relays) offer rings of one stream side by side. Every stream publisher
  holds a liveliness token on `<base>/info/<host>/<id>`; a subscriber with `auto` chooses again whenever a publisher of its
  stream comes or goes, so the order in which programs start does not matter (a relay started after its readers is
  taken; when it stops they go back to the network). While it changes its transport a subscriber reads both routes
  until the new one's first frame (at most 2 s) and hands out each frame once, by its stamp. A publisher with
  `cuda_device` needs the CUDA driver when it is made. `blick`'s key tree shows a ring's place by 8 characters.
* **0.3.58** A reader on another GPU of the publisher's host gets its frames on its own GPU, copied from GPU to GPU:
  `CudaRingReader(endpoint, target_device)` copies out of the ring with `cuMemcpyPeerAsync`, over PCIe where the two
  devices reach each other (`direct`), else through host memory. `auto` with a `device` takes the CUDA ring of another
  GPU when its process sees that GPU (`cuda (copied from the publisher's GPU ...)`), else the host ring as before;
  `local="cuda"` with a `device` delivers on that device too. A reader that asks for views (`copy=False`) gets copies
  from a ring on another GPU. `cuda_device_ordinal(uuid)`. 1080p `rgb8`, two RTX PRO 6000: 0.16 ms a frame, against
  0.53 ms through the host ring with an upload; the reader's process holds a CUDA context on the publisher's GPU
  (552 MiB there).
* **0.3.57** `durchblick::Options` (C++, `durchblick/options.hpp`): a program's options defined once (name, default and
  type, help, live), parsed from the command line (unknown, twice, out of range or choices: the reason and the usage,
  exit 2; `--help`), and declared as the node's parameters, read-only but the live ones with their writers; a value
  given at the start with `--param` or a params file becomes the option's. With the session's options
  (`--connect`, `--listen`, `--zenoh-mode`, `--zenoh-config`) and the parameters' own. `ParameterOptions`' members
  have initializers (a designated initializer may leave any out without `-Wmissing-field-initializers`). The relay,
  the Azure Kinect camera and body tracker and the rc_visard node use it.
* **0.3.56** A node's parameter services answer in a daemon thread of their own (queries from Zenoh's channels,
  looked for every 10 ms), no longer in zenoh-python's callback thread, which is no daemon: a node that ended with an
  exception before `Parameters.close()` (an interrupt while its model loads, now that nodes offer their parameters
  from the start) waited for that thread for good. `close()` may be called twice.
* **0.3.55** `Parameters.declare_options(parser, args, live=...)` declares every command-line option of a node as a
  parameter: read-only (at the next start) except the live ones, the help as description, `--param` and
  `--params-file` written back into `args`. The nodes use it, so that the terminal interface shows how each runs.
* **0.3.54** The terminal interface:
  * The Graph tab draws the sessions as boxes in columns and the data between them as arrows with labels
    (`tui_graph.py`, a small layered layout). The arrow keys or a click select a session and colour its edges. The
    mouse over an edge or a box shows its keys and how each is taken. The boxes show each session's diagnostics.
  * The Topics tab is a key tree. Enter folds a prefix. A prefix's publishers and subscribers stand once on the
    highest row below which every key has the same, in new columns "published by" and "subscribed by". Sessions
    have short names there.
  * The connect dialog opens on the endpoints with the cursor in the address field. Typing into a field selects its
    way. The interface opens the dialog by itself when it starts on this PC's router and finds nothing there.
  * `r` and the Rerun tab's "Look again" search the network at once. The tab says when it last searched.
  * `session.endpoint` gives an address without a port 7447 (`tcp/robot-pc` is `tcp/robot-pc:7447`), as
    `robot-pc` was already. It refuses an IPv6 address without brackets and an empty endpoint.
  * The Parameters tab: one line per parameter (name, field, the descriptor's text; under the field below 120
    columns). A node opens when highlighted. ↑/↓ step a number by its range's step and set it; Esc takes an edit
    back; an edited field not yet set is marked; "set" confirms a change. A bool is a one-line checkbox.
  * `tools/tui_screenshots.py` makes the screenshots of docs/guides/tui.md from a made-up network.
* **0.3.53** ArmarX integration: see durchblick_armarx.
* **0.3.52** C++ `auto` on another host refuses a source that does not offer the remote form (`compressed` by
  default) with ProtocolError, as Python's `ImageSubscriber` does, instead of waiting for frames that never come (the
  Jetson's ecam24 node in raw mode offers only `raw`). Relays' records are left out of that check.
* **0.3.51** `find_streams` (`durchblick list`, `rerun`, the recorder, the terminal interface) keeps one record per
  stream, the source's, with its relays' hosts in `relayed_by` (`durchblick list`: "relayed on <host>"); before, a
  relay's record (only rings, no `compressed`) could stand for the stream.
* **0.3.50** Queries take every reply (`replies()`, C++ and Python, without Zenoh's consolidation): with the default
  consolidation two publishers of one stream (a source and its relay) answered `<base>/info` as one, so a subscriber
  or the relay saw one of them at random; a caller that counts the answers of a service (one controller manager per
  robot) saw duplicates merged.
* **0.3.49** Relays (durchblick-relay): the C++ `PublisherOptions` take `transports` (the representations offered;
  a relay offers only its host's rings, `shm`/`cuda`) and `relayed_from` (in the info record, null otherwise), and
  `CameraPublisher::publish(image, Origin)` republishes a frame with the source's header, `seq` and `source`. C++
  `stream_infos` (info records read) and `parse_frame_attachment`. `camera_info` is declared only with a CameraInfo.
  Python's `auto` on a third host no longer refuses a stream whose relay record has no `compressed` form.
* **0.3.48** The terminal interface's Rerun tab lists pose topics (`<camera>/tf`, `<camera>/<node>/tf`, not
  `<robot>/tf`) for `--trajectory`; presets keep them.
* **0.3.47** `durchblick rerun --trajectory KEY`: poses in the world from any TFMessage topic (durchblick-vslam's
  `<camera>/vslam/tf`, `<camera>/tf` to compare) as a path with axes at the newest pose, at `trajectories/<key>`.
* **0.3.46** A robot's controller manager on Zenoh, as ROS 2's (`<robot>/controller_manager`: `list_controllers`,
  `switch_controller`, `unload_controller` with ros2_control Jazzy's controller_manager_msgs, checked byte for byte
  against rosbags; and Durchblick's `create_controller` from a type and a JSON config, `get_controller_config`,
  `get_config_file`), served by the robot's software or a bridge to it (ArmarX
  integration: see durchblick_armarx).
  Its STRICT switch refuses to activate a controller on a joint another active controller holds unless that one is
  deactivated in the same switch. Python: `durchblick.controller_manager` (`ControllerManager`, `torque_step`),
  `durchblick.controller.JointPositionSubscriber` for a joint position controller. A
  controller's `torque_step` service (durchblick_msgs/srv/TorqueStep). The shared `ros2_cdr_msgs.hpp` is version 2
  (`u16`, TorqueStep).
* **0.3.45** The messages a robot's controller speaks (Time, Header, TFMessage and its geometry, DiagnosticArray,
  JointState, WrenchStamped, MultiDOFJointState, RobotState, Float64MultiArray, the rcl_interfaces parameter messages)
  and the CDR writer and reader moved from `msg.hpp`/`msg.cpp`/`parameters.cpp` into
  `cpp/include/durchblick/ros2_cdr_msgs.hpp`, header only, in the namespace `ros2_cdr::v1` and named in `durchblick::`
  as before (no other change in C++ or Python; the bytes are the same, `test_messages`). A controller may
  carry a copy of the file without depending on Durchblick. A change of it raises `ROS2_CDR_MSGS_VERSION` (and the
  inline namespace with it). ArmarX integration: see durchblick_armarx.
* **0.3.44** Paired frames taken together, and 32 ring slots (on a robot, Boxer asked for a colour frame's
  depth after its 2D stage, about 120 ms later; the camera had reused the depth's ring slot by then, so no frame had
  depth). Image rings keep 32 frames by default (`camera.DEFAULT_SLOTS`, C++ `PublisherOptions::slots`): a reader on
  the same host has 31 frame periods, about a second at 30 Hz, instead of 3. Point cloud rings keep 4 (large, and
  nothing pairs them). A ring is allocated only for a transport somebody reads. `PartnerHistory` pairs a frame with
  the nearest partner within half the partner's frame period, measured from its samples, when no `max_offset_ns` is
  given (the new default, also `RGBDInputs`' for the depth; a segmenter's instances still pair by equal stamps): a steady partner stream pairs every frame with its nearest sample,
  whatever its fixed offset (the Azure Kinect's colour and depth_to_color: 11-17 ms), and a missing one is not
  replaced by a neighbour; an explicit value overrides it, 0 asks for equal stamps. `PartnerHistory.wait(stamp,
  timeout)` reads a frame's partner as it comes, for a node that takes a frame and then works on it;
  `RGBDInputs.take(stamp)` reads all of a frame's streams together, and `FramePreparer` uses it before any work. A
  partner overwritten in its ring before it was read counts in `PartnerHistory.lost` (`RGBDInputs.lost()`) and is
  logged at most every 10 s, with the fix (take the pair together, or more slots).
* **0.3.43** A controller's commands addressed, whole and in order (the review of 2026-11-08): one
  `moveit_msgs/msg/RobotState` per command on `<robot>/controller/<instance>/command` (joint targets and the controlled
  frames' pose targets, one stamp), instead of `<robot>/command/tf` and `/command/joints` paired by stamp, which went to
  every linked controller, split a command when a key came late and could apply an older command after a newer one. A
  stale command (stamp not newer than the last one taken) is refused, so is one whose wall-clock stamp is more than
  250 ms off the controller's clock (`CommandSubscriber(max_age=)`), a
  command is taken or refused whole (a frame or joint the controller does not have or twice, mixed stamps), one
  controller's commands are sent one at a time. New
  `<instance>/status` every 20 ms: active, level, accepted and rejected commands, the link's own refusals,
  `hardware_id` the controller's name. `durchblick.controller`: `CommandPublisher(session, robot, controller,
  root_frame)`, `CommandSubscriber(..., on_refused=)`, `ControllerPublisher.status()`, `ControllerSubscriber(on_status=)`;
  `msg.RobotState`, `msg.MultiDOFJointState`, `topics.controller_command`, `topics.STATUS`.
  `<robot>/command/*` stays what a simulation records its robot was told.
* **0.3.42** A controller on Durchblick is named by its role, below `<robot>/controller/<role>`
  (`robot/controller/arms`), as ros2_control names controllers. ArmarX integration: see durchblick_armarx.
* **0.3.41** ArmarX integration: see durchblick_armarx.
* **0.3.40** A robot's controllers on Durchblick (protocol.hpp, topics.py): what a controller is
  told on `<robot>/command/tf` (pose targets) and `<robot>/command/joints` (joint targets; the two with the same stamp
  belong together, ROS's exact-time pairing), what it reports below `<robot>/controller/<instance>`: `joint_states`,
  `tf` (per arm `<tcp>/measured`, `/reference`, `/target`), `wrench/<arm>`, `record` (every value of every recorded
  cycle), `diagnostics` and `hands/joint_states`, with the parameters `level`, `state_decimation` and
  `record_fields`. Python `durchblick.controller`: both sides of it (`CommandPublisher`, `CommandSubscriber` with the
  pairing, `ControllerPublisher`, `ControllerSubscriber`). New type
  `std_msgs/Float64MultiArray` (Python and C++, with its MCAP definition), C++ `MessagePublisher` (one ROS type on one
  key), `topics.controller()`; recordings take `<ns>/controller/**` by default. ArmarX integration: see durchblick_armarx.
* **0.3.39** Fix of 0.3.36's `local="auto"`: a subscriber without a device (`device=None`, the default; C++
  `cuda_device` unset) works on the CPU and takes shared memory, `shm (this subscriber works on the CPU)`; a CUDA
  ring only with a GPU device (`device="cuda:0"`, C++ `.cuda_device = 0`). 0.3.36 to 0.3.38 handed device arrays
  to readers that never asked for a GPU (SAM 3's test reader).
* **0.3.38** Sessions close within 5 s: `open_session` and `open_from_args` return a `session.Session`, the
  zenoh.Session with every attribute (and `isinstance(s, zenoh.Session)`) whose `close()` and the end of a `with`
  block go through `session.close(…, timeout)`; a Zenoh close that does not return (after a query it did not
  finalize) no longer blocks a program or the tests (the v0.3.37 pipeline hung in `with open_session(...)`).
* **0.3.37** Tests only: the viewer test skips without rerun-sdk (CI installs no extras), the not-found message is
  its own test; test_point_clouds waits for the cloud placed in the colour camera's frame (a race on loaded hosts).
  0.3.36's tag pipeline failed on the viewer test, so 0.3.37 is 0.3.36 released.
* **0.3.36** The play node's controls as parameters (node `durchblick/<name>`, recordings and videos): `paused`,
  `position_s` (a seek: a video from the key frame before it decoded on to the frame, a recording from its log
  time; the stamps keep growing; after the end it plays the files again), `loop` at run time, the read-only
  `duration_s` and `file`, beside `rate`; ParameterEvents when the playback changes them itself
  (`Parameters.refresh`, Python and C++); the terminal interface does not overwrite a field being edited.
  `session.replies` reads a query's channel in a short thread instead of Python callbacks (no GIL handover per
  reply; a CI runner under load had measured 12 ms). `durchblick rerun --spawn` finds the viewer next to Python
  before PATH (a venv not activated). `blick --connect ...` (and `--listen`, `--zenoh-mode`, `--config`) starts
  the terminal interface connected, a client by default with endpoints; as a peer it says when it cannot reach
  the peers the admin space shows (a tunnel), and docs/tui.md has the SSH recipe. `durchblick rerun`: rates
  (`rates/<topic>`, Hz), ages at receive (`ages_received/<topic>`), and `ages_now` as a Markdown table with the
  samples a second lost on the way (seq gaps); views named with their units. mcap 1.6 or newer. Same-host frames
  by `local="auto"` by default (Python, C++ `SubscriberOptions::local`): the publisher's CUDA ring if it is on the
  subscriber's GPU (`device`/`cuda_device`, compared by UUID: `_core.cuda_device_uuid`), else shared memory; the
  reason in `describe()`, `transport_reason` and the log; info records name the ring's GPU from the start;
  `session.close(session, timeout)`: a Zenoh close that does not return (after a query Zenoh did not finalize)
  no longer blocks the terminal interface or the tests; `session.replies` cancels a query at its deadline.
  `node.add_arguments(parser, local=...)` adds `--local` (a CPU node: `("auto", "shm")`, `--local cuda` refused). `durchblick play --cuda-device` for videos: a CUDA ring
  per stream.
* **0.3.35** Tests: a hard limit per test, pytest-timeout (a dev dependency) at 120 s, method thread: a hang prints
  every thread's stack and ends the run instead of blocking CI until its job timeout; faulthandler's dump at 90 s
  (was 300 s) before it ([development](docs/development.md)).
* **0.3.34** Queries never block beyond a deadline: `session.replies` (Python) and `durchblick::replies` (C++)
  collect a query's replies until Zenoh ends it or its timeout and as much again (at least 0.5 s) have passed, then
  go on with what came, count it and log it once per selector; every query of Durchblick goes through them (stream
  info, `find_streams`, `tf_static`, the record and play node, the terminal interface and its survey, parameters,
  liveliness). A get with timeout=1.0 had blocked for 5 min in CI ([sessions](docs/sessions.md)).
* **0.3.33** `durchblick play` plays videos and image sequences as camera streams ([video](docs/video.md); the extra
  `durchblick[video]`, PyAV): any file that is not MCAP (by its content), several at once on one clock, one stream
  each (`--stream`, `--camera` for one), stamps from the start's wall time and the frames' PTS (equal PTS: equal
  stamps; `--offset-ms`), sent at their stamps, looped together; `--camera-info` or an assumed pinhole
  (`--hfov`/`--vfov`), `--size`/`--scale`, `--start`/`--end`, `--fps`, `--frame-id`; 16-bit grey videos as
  `16UC1`; source `video`; diagnostics `play` (frames/s, decode ms, position, late frames, send skew). The play
  node's parameter `rate` changes a playback's rate while it plays (recordings too; `recording.Paced`, the pace
  both share). `durchblick record` without the extra `record` fails at its start as before; `durchblick play`
  of videos does not need it.
* **0.3.32** `Parameters.apply_all` (Python and C++) writes in the order the parameters were declared, not by
  name: a setting before its `auto_*` bool, so a control given auto at the start stays auto (before, every such
  control ended manual); a value a write changed elsewhere (the bool a setting switched to manual) is stored as it
  is now, so `apply_all` writes what the device was left with. A partner subscriber (`keep_recent`, Python and C++)
  no longer counts the samples it keeps as `overwritten`.
* **0.3.31** Node parameters as ROS 2's ([parameters](docs/parameters.md)): the rcl_interfaces messages
  (`ParameterDescriptor`, `ParameterValue`, `ParameterEvent`, the services' requests and responses) in Python and C++,
  ROS 2's six parameter services as queryables below the node, `ParameterEvent`s on `parameter_events`, the token
  `<node>/parameters`; `node.Parameters` and C++ `durchblick::Parameters` (descriptors, checks, writers and readers,
  `--param`, `--params-file`, `--dump-params`, `--params-read-only`), `ParameterClient`, `durchblick param`, the
  terminal interface's Parameters tab; `durchblick-cpp-camera --params`, `durchblick-cpp-probe --params`. The terminal
  interface takes the terminal's size also where `COLUMNS` and `LINES` are set (readline sets them for child processes
  before Python 3.13; the interface stayed at 80 x 24 then).
* **0.3.30** The age at send of every sample with an attachment (`publish_ns` minus the stamp, on the publisher's
  clock alone; `publish_ns` taken right before each transport's put, so with its encoding or ring copy, and
  `publish_called_ns` when publish was called, Python and C++): `Frame.age_at_send_ns` (Python, C++ `Frame::age_at_send_ns()`, clouds too), `stamps.age_at_send_ns`
  (`stamps.header_stamp`, which now also reads Durchblick's ring descriptors' stamps), detections publish the
  attachment; shown by `durchblick hz`, the terminal interface (median / p95 at send and at receive, ms) and `durchblick
  rerun` (`ages/<topic>`, `--no-ages`). `--max-hz` for nodes (`node.add_arguments`, `Reporter.watch`): subscriptions
  with `max_hz` (Python and C++) take at most so many frames a second, the newest, chosen by their stamps so that a
  camera's streams pair, and skip the others before decoding them; paired streams limited as a pair (only the
  primary, the partners decoded at its frames' stamps: `history.PartnerHistory`, `RGBDInputs`' `max_offset_ns`, C++
  `SubscriberOptions::keep_recent`, `ImageSubscriber::at`); a node's own default
  (`node.add_arguments(parser, max_hz=30.0)`), `--max-hz 0` or `none` every frame it can take, the rate in effect in
  the start log (`node.start(..., max_hz=)`) and in every report (`max-hz`); `durchblick-points --max-hz` (default
  every frame), `durchblick-cpp-probe --max-hz`.
* **0.3.29** A terminal interface: `durchblick` without a command (Textual, a new dependency): connect and connect
  again, topics with their traffic and last message, sessions, reports, the publish-subscribe graph from the routers'
  admin space, and `durchblick rerun` from chosen streams and options with named presets (`survey.py`, `tui.py`).
  `blick` is the same command as `durchblick`. Diagnostics samples (Python and C++ `DiagnosticsPublisher`) carry the
  attachment with the publishing session's `zid` (`diagnostics.sender`); readers that ignore it read them as before.
  `durchblick mps` announces itself on Zenoh (`durchblick/mps/<name>`, its status with GPUs and clients;
  `--name`, `--connect`, `--listen`, `--config`) and the terminal interface shows it. The interface shows how each
  consumer takes a stream (shm, cuda, or the Zenoh link's protocol with raw or compressed).
  `--zenoh-mode peer|client` for every command (`session.add_arguments`); client mode refuses listen endpoints.
* **0.3.28** Recording simulations: blocking publishers (`blocking=True`, opt-in: the default drops; counts on
  `<key>/sent`) and blocking recordings with a count check at stop (`RecordingError`); `record` with exactly the keys
  given and a *ready* reply; log time from the messages' stamps (`--log-time header`); lockstep playback (`--pace
  step` with the command `step`, `--pace clock`); commands as queries with their results (`recording.request`; status
  version 3); a simulation's topics (`<world>/tf`, `clock`, `events`, `<robot>/command/joints`, `tf`;
  `sim.SimPublisher`; `rosgraph_msgs/Clock`); `recording.Recording` in-process. A blocking recorder never blocks for,
  nor checks, a dropping publisher's keys (reported as `dropping`). Python only: C++ publishers have no blocking mode.
  Every `RobotStatePublisher` and `TransformPublisher` sample now carries the attachment.
* **0.3.27** `--loop` moves the stamps on by the recording's length each pass (they went back to the start, and nodes
  that take only newer frames stopped after the first pass); the player's status counts the `passes`.
* **0.3.26** One node records and plays (`durchblick record`, `durchblick play`; Zenoh commands `record`, `stop`,
  `play`, `pause`, `resume`, status version 2); played image streams go out as a camera's: decoded once into the
  shared-memory ring (and a CUDA ring), the recorded JPEG or PNG unchanged (`CameraPublisher.publish_compressed`).
* **0.3.25** Python's `PointCloudPublisher` publishes an empty cloud (no points; it failed with "slot size 0");
  `durchblick rerun` draws a node's cloud with pixels (`u, v`, durchblick-tapnext's tracks) over its image too.
* **0.3.24** `vision_msgs/Classification` (classes without a place, e.g. actions; Python and C++), published and
  read like the detections, recorded, and shown by `durchblick rerun` as a text log.
* **0.3.23** Shared node code (`node`, `history`, `tensors`, `camera.check_local`, `poses.transform_of`/`pose_of`);
  publishers' rings grow with a larger frame (Python and C++).
* **0.3.22** `uv sync` rebuilds the package when its C++ sources changed (it kept an old `durchblick._core` after a
  pull).
* **0.3.21** The installed CMake package names the CUDA toolkit it was built with (its consumers find it).
* **0.3.20** `find_streams` skips an answer that is no info record instead of failing.
* **0.3.19** A frame whose ring is gone counts as stale instead of ending the reader; `FramePreparer` and the point
  cloud node wait for a camera_info of the frames' size.
* **0.3.18** Logging for the nodes (`logs.setup`), `describe()` of a subscription, `RGBDInputs.waiting()`,
  `DiagnosticsPublisher`.
* **0.3.17** `durchblick record`, `durchblick play` (MCAP).
* **0.3.16** `durchblick rerun`'s default layout; label images over their colour stream, id 0 transparent.
* **0.3.15** A Python `CameraPublisher` refuses a codec that cannot compress its encoding; `codec=None`.
* **0.3.14** `durchblick rerun` without options shows every node's results, also of nodes started later.
* **0.3.13** `durchblick rerun` draws a node's bodies (`<camera>/<node>/bodies`).
* **0.3.12** `rgbd.Distorter`: a node's per-pixel output back into the camera's own image.
* **0.3.11** `paths.expand_path`: `${VAR}` and `~` in a node's path arguments.
* **0.3.10** `rgbd.collect_views`: views of an object from a moving camera.
* **0.3.9** `durchblick.rgbd`: a camera's colour, depth and a segmenter's masks paired by stamp.
* **0.3.8** `durchblick rerun --segmentation` (label images).
* **0.3.7** `find_streams` asks again until its timeout (publishers behind a router).
* **0.3.6** Device arrays published complete and before `publish` returns (`stream=`); NVIDIA MPS (`durchblick mps`).
* **0.3.5** A Zenoh router on every PC, sessions without endpoints connect to it.
* **0.3.4** A robot's state (`<robot>/joint_states`, `tf`, `odom`, `wrench/<sensor>`), poses at an image's stamp
  (`poses.py`). ArmarX integration: see durchblick_armarx.
* **0.3.3** A camera's pose on `<camera>/tf`; C++ `Pose` is ROS's `geometry_msgs/Pose` (0.3.2: `PoseMessage`).
* **0.3.2** Vision results as ROS's `vision_msgs` (`DetectionPublisher`, `DetectionSubscriber`).
* **0.3.1** `--max-depth` for the point cloud node; ROS calibration files for `CameraInfo`.
* **0.3.0** Point cloud streams and `durchblick-points`; `local="cuda"`; info record version 2 (0.2.x refused).
* **0.2.3** `<camera>/imu`, `<camera>/diagnostics`. **0.2.2** `<camera>/tf_static`. **0.2.1** The Rerun viewer, JPEG
  pass-through for MJPEG cameras, device frames downloaded into the shared-memory ring.
