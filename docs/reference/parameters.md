# Parameters

The settings of a node are parameters, for example the exposure of a camera or the threshold of a detector. You
declare them in the node, set them at start, and read and change them at run time over Zenoh.

Durchblick uses the parameter model of ROS 2. A node describes each parameter with a
`rcl_interfaces/msg/ParameterDescriptor`. It offers the six parameter services of ROS 2 and announces changes with
`rcl_interfaces/msg/ParameterEvent`. The messages have the same bytes as in ROS 2. Only the keys follow Durchblick's
naming, so a later ROS mode needs to map only the keys.

The API is `xblick.parameters` in Python, also reachable as `node.Parameters`, and `xblick/parameters.hpp`
in C++. Both have the same model, keys, checks and messages.

## Read and change parameters

```bash
xblick param --nodes                                  # the nodes that offer parameters
xblick param --node robot/camera                # names and values
xblick param --node robot/camera --describe     # with type, range, step, unit, choices, read-only
xblick param --node robot/camera --set color.exposure_time_us:=10000 --set color.gain:=20
xblick param --node robot/camera --dump ~/kinect.yaml    # write a params file
```

`--set` prints `name: set` or `name: refused: <reason>` for each parameter. The command ends with status 1 when the
node refused one. `--atomically` sets all or none.

The terminal interface has a Parameters tab (key `5`). It builds a form from the descriptors of the chosen node.
See the [terminal interface guide](../guides/tui.md).

In a program, use a client:

| Language | API |
| --- | --- |
| Python | `parameters.ParameterClient(session, "<namespace>/<name>")` with `list`, `describe`, `get`, `types`, `set`, `set_atomically`, `all` and `on_event`. `parameters.find_nodes(session)` finds the nodes. |
| C++ | `xblick::ParameterClient` |

The client raises an error when no node answers, or when more than one node answers.

## Set values at start

```bash
xblick-some-node --param threshold:=0.5 --param mode:=accurate --params-file ~/tracker.yaml \
    --dump-params /tmp/effective.yaml
```

| Option | Meaning |
| --- | --- |
| `--param NAME:=VALUE` | one value as a YAML scalar, as `-p` in ROS: `true`, `3`, `0.5`, `"a: b"`, `[1, 2]`. Repeat the option for several. |
| `--params-file FILE` | a ROS params file. Repeat the option for several. A later file wins over an earlier one, and `--param` wins over all files. |
| `--dump-params FILE` | write the parameters in effect to this file, as a params file |
| `--params-read-only` | refuse every change over Zenoh. See [read-only access](#read-only-access). |

### The params file

The format is the params file of ROS 2.

```yaml
/robot/camera/tracker:      # the full name of the node. The leading / is optional.
  ros__parameters:
    threshold: 0.5
    color:
      exposure_time_us: 8000      # the parameter color.exposure_time_us
/**:                              # every node
  ros__parameters:
    draw: false
```

| Node key | Matches |
| --- | --- |
| `/robot/camera/tracker` | this node |
| `tracker` | a node with this name in any namespace |
| `*` | one level |
| `**` | any number of levels. `/**` matches every node. |

A later entry in the file wins over an earlier one. The parser reads the part of YAML that these files use:
mappings, scalars, flow lists such as `[a, b]`, and comments. Any other construct is an error.

### Wrong values stop the node

A node checks every value at start and stops with the cause.

| Error | Found at |
| --- | --- |
| a wrong type, a value outside the range, a value off the step, a value that is not one of the choices | `declare` |
| a name that the node does not declare | `check_overrides`, which lists the names that the node has |

ROS ignores an override for an undeclared parameter. Durchblick treats it as an error, because such an override is
usually a typing mistake, and a mistake in the settings of a camera must not go unnoticed. A wildcard entry (`/**`)
with a name that a node does not have stops that node too.

## Declare parameters in a node

Python:

```python
from xblick import node

parser = argparse.ArgumentParser()
node.Parameters.add_arguments(parser)          # --param, --params-file, --dump-params, --params-read-only
args = parser.parse_args()

params = node.Parameters(session, "tracker", "robot/camera")   # name, namespace
params.configure(args)
params.declare("threshold", 0.3, description="a box's score at least", range=(0.0, 1.0, 0.05))
params.declare("max_boxes", 10, range=(1, 100, 1), unit="boxes")
params.declare("mode", "fast", choices=["fast", "accurate"])
params.declare("model", "base", read_only=True)                       # applies at the next start
params.check_overrides()                                              # after the last declare
...
params["threshold"]                                                   # the current value
params.close()
```

C++:

```cpp
#include "xblick/parameters.hpp"

xblick::Parameters params(session, "camera", "robot/camera");
params.configure(xblick::parameter_args(argc, argv));
params.declare("color.exposure_time_us", std::int64_t{8000},
               {.description = "exposure time", .unit = "us",
                .integer_range = xblick::IntegerRange{500, 33000, 1}},
               [&](const xblick::ParameterValue& v) { return device.set_exposure(v.integer_value); },
               [&] { return xblick::ParameterValue::of(device.exposure()); });
params.declare("fps", std::int64_t{30},
               {.choices = {xblick::ParameterValue::of(std::int64_t{5}),
                            xblick::ParameterValue::of(std::int64_t{30})},
                .read_only = true});
params.check_overrides();
```

Declare the parameters as soon as the node can. A camera node declares them after it opened the device, so that
the ranges are those of the device. Then call `check_overrides()`.

### A node's options as parameters

Every Python node declares its command-line options as parameters with one call. The terminal interface and
`xblick param` then show how the node runs, and `--param` and `--params-file` set any option at the start.

```python
args = parser.parse_args()
params = node.Parameters(session, args.node, camera)
params.configure(args)
params.declare_options(parser, args, live={
    "threshold": {"range": (0.0, 1.0, 0.05)},          # changes while the node runs
    "max_hz": {"default": 0.0, "unit": "frames/s"},     # an option that is None by default needs a typed default
})
params.check_overrides()
...
segmenter.threshold = params["threshold"]               # the node reads a live parameter where it applies it
```

- Each option becomes a parameter named after its `dest` (`--match-iou` is `match_iou`), with its help text as the
  description and its choices.
- The parameters are read-only and apply at the next start, except the live ones. A live parameter's spec takes the
  arguments of `declare`: `range`, `unit`, `choices`, `write`, `read`, and `default`.
- A value given at the start is written back into `args`, converted with the option's `type`. Call
  `declare_options` after `configure` and before the node uses `args`.
- An option whose value is None becomes a read-only string, empty when it was not given.
- Left out are `--help`, the parameters' own options, the Zenoh connection's options, the options in `leave_out`, and
  the parameters that the node declared already.

A live parameter changes in Zenoh's thread. The node reads it where it applies the value, or applies it in `write`
when that is safe from another thread.

### A C++ program's options as parameters

A C++ program defines each option once with `xblick::Options` (`xblick/options.hpp`): its name, its default
and so its type, its help, and whether it is live. The class parses the command line and declares the options as
parameters, as `declare_options` does in Python.

```cpp
xblick::Options options("xblick-relay", "relays image streams of other PCs into this PC's rings");
options.add_list("stream", {}, {.help = "the streams", .metavar = "NAME", .required = true});
options.add("slots", std::int64_t{32}, {.help = "ring slots", .parameter = {.integer_range = IntegerRange{2, 256, 1}}});
options.add("max-rate", 0.0, {.help = "pairs a second", .live = true, .parameter = {.unit = "Hz"}});
options.add_flag("prefer-raw", {.help = "raw frames"});
options.add_optional("cuda-device", parameter_type::kInteger, {.help = "the GPU"});  // no default
options.add_session_options();     // --connect, --listen, --zenoh-mode, --zenoh-config
options.add_parameter_options();   // --param, --params-file, --dump-params, --params-read-only
options.parse(argc, argv);         // --help prints the help; a wrong command line ends the program (2)

zenoh::Session session = xblick::open_session(options.session_options());
xblick::Parameters params(session, "relay", "robot");
params.configure(options.parameter_startup());
options.declare(params, {{"max_rate", [&](const ParameterValue& v) { return tracker.set_rate(v.double_value); }}});
params.check_overrides();
options.integer("slots");          // the value, also when given with --param or a params file
```

- The kinds are a flag, an integer, a number, a text and a list of texts. `add_optional` defines an option without
  a default. Its parameter is a read-only string, empty when the option was not given.
- A parameter is named after its option with `_` for `-`. Its description is the help.
- The parameters are read-only, except the live ones. A live parameter takes its writer from `declare`'s map, by
  parameter name. The writer runs in Zenoh's thread, so it must be safe there, or the program reads the parameter
  where it applies the value.
- A value given at the start becomes the option's value. Call `declare` after `configure` and before the program
  uses the options.
- The session's options and the parameters' own are no parameters. `alias` gives an option an older name.
- An unknown option, an option given twice (but a list), a value for a flag, and a value outside its range or
  choices end the program with the reason and the usage.

### What a declaration contains

| Part | Rule |
| --- | --- |
| Name | letters, digits and `_`, with parts joined by dots, for example `color.exposure_time_us`. The nested mappings of a params file are these dotted names. |
| Type | the type of the default: bool, integer, double, string, bytes, or an array of one of them |
| Range and step | `range=(from, to)` or `(from, to, step)`. C++: `floating_range`, `integer_range`. |
| Choices, unit | a list of allowed values, and a unit as text |
| `read_only` | the value applies at the next start |
| Writer | a function that applies a change, for example to a device |
| Reader | a function that returns the value in effect |

**Type.** Over Zenoh, a node refuses a value of another type. It also refuses an integer for a double parameter,
as ROS does. Text from `--param`, from a params file or from a field of the terminal interface is read for the type
of the parameter: `3` for a double parameter is 3.0, and any text is valid for a string parameter.

**Range and step.** As in ROS, the allowed values are `from + n * step` up to `to`, and `to` itself. Step 0 allows
any value in the range. Some devices have steps that do not lie on such a grid. Such a node publishes step 1 for
integers or step 0, says so in the description, and its writer refuses the values that the device does not accept.

**Choices and unit** travel in `additional_constraints`, the free text field of ROS, in the form
`unit: us; choices: 5, 15, 30`. Either part may be missing. Tools read them with `parameters.constraints`. A node
checks a choice like a range.

**Read-only.** You can set a read-only parameter with `--param` or a params file. A change over Zenoh receives the
answer `read-only: applies at the next start`.

**Writer.** In Python it is `write=`, a function that returns `None` or the reason for a refusal. In C++ it is a
`Writer` that returns `""` or the reason. When the writer fails, the node refuses the change with this reason and
stores nothing. An error of a device thus becomes the reason in `SetParametersResult`. For a change over Zenoh, the
writer runs in a thread of Zenoh.

**Reader.** In Python it is `read=`, in C++ a `Reader`. Use it for a device that can change a value by itself.
`get` and the services answer with the reader's value. Without a reader, the stored value is the value.

### Automatic and manual settings

A setting that a device can control by itself, such as exposure or white balance, has a boolean `<setting>.auto_*`
beside it. Example: `color.exposure_time_us` and `color.auto_exposure`. Setting the value switches the boolean to
manual. The writer does this, and the event shows both changes.

### Events

A node sends a `ParameterEvent` after a declaration (`new_parameters`) and after each change. It compares all
values before and after the change, and the event contains every value that differs. A writer that changes a
second parameter therefore announces both. The node stores such a second value as it is after the change. A
parameter that another writer can change therefore needs a reader.

### Apply all values to a device

`apply_all()` calls the writer of every writable parameter with its value. Use it for a device that otherwise
keeps its last settings. It calls the writers in the order of the declarations, and it raises an error with the
reasons of all writers that failed.

Declare a setting before its `auto_*` boolean. Writing the value switches the device to manual. The boolean is
written after it and then sets automatic or manual as you gave it.

## Read-only access

With `--params-read-only`, a node answers list, describe and get, and it refuses every set with
`parameters are read only on this node (--params-read-only)`. The values from the start still apply. Use it on a
robot in operation, where nobody should change a camera over the network. In code it is `read_only_access=True`,
and in C++ `ParameterStartup::read_only_access`.

## Keys

A node with parameters is `<namespace>/<name>`, for example `robot/camera`. Its full name in the notation of
ROS is `/<namespace>/<name>`.

| Key | Content |
| --- | --- |
| `<node>/list_parameters` | queryable: `ListParameters` |
| `<node>/describe_parameters` | queryable: `DescribeParameters` |
| `<node>/get_parameters` | queryable: `GetParameters` |
| `<node>/get_parameter_types` | queryable: `GetParameterTypes` |
| `<node>/set_parameters` | queryable: `SetParameters` |
| `<node>/set_parameters_atomically` | queryable: `SetParametersAtomically` |
| `parameter_events` | the `ParameterEvent` of every node on one key, as `/parameter_events` in ROS. The field `node` is the full name. |
| `<node>/parameters` | a liveliness token while the node offers parameters. Tools find the nodes through it. |

The payload of a query is the request of the service in CDR (`rcl_interfaces/srv/<Service>_Request`). The payload
of the reply is its response. The key names are constants in `protocol.hpp` (`kListParameters` to
`kParameterEvents`, `kParametersToken`) and in Python (`_core.protocol.LIST_PARAMETERS` and so on).

A ROS mode would put the services at the keys of `rmw_zenoh` and the events on `/parameter_events`. The payloads
would stay as they are. See [Durchblick and ROS 2](../integrations/ros2.md).

## Messages

Both languages have the same messages, in `msg.py` and `msg.hpp`: `ParameterType`, `ParameterValue`, `Parameter`,
`FloatingPointRange`, `IntegerRange`, `ParameterDescriptor`, `SetParametersResult`, `ParameterEvent`, and the
request and response of each service (`ListParametersRequest` to `SetParametersAtomicallyResponse`).
`ParameterValue.of(value)` creates a value message from a value.

## Examples in the tools

- `xblick-cpp-camera --params` declares a few parameters as an example, as the node `<camera>/cpp_camera`.
- `xblick cam` declares its options as read-only parameters of the node `<camera>`.
- `xblick-cpp-probe --params <node> [--set NAME:=VALUE]` reads and sets them from C++.

## Status and limits

- A live parameter changes only where the node reads it. A node whose library takes a setting when it is created
  keeps that option read-only.
