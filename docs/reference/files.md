# Where files are

This page lists where Durchblick and its nodes keep files on a PC, and the variables that move them. It also shows
how to copy a cache to a PC without internet and how several users share one directory.

## The places

| Kind | Content | Path | Moved by |
| --- | --- | --- | --- |
| Cache | Downloaded weights and what a node builds from them, for example TensorRT engines and compiled kernels. You can delete it. The node fetches or builds it again. | `~/.cache/xblick/<node>/` | `XBLICK_HOME`, `XDG_CACHE_HOME`, `--weights-dir` |
| Data | What a user made and cannot download again, for example the object memories of a node. | `~/.local/share/xblick/<node>/` | `XBLICK_HOME`, `XDG_DATA_HOME` |
| Config | The presets of the terminal interface (`rerun_presets.json`). | `~/.config/xblick/` | `XDG_CONFIG_HOME` |
| Rings | The host rings of the streams on this PC. | `/dev/shm/xblick-<16 hex digits>` | nothing |

Example: the YOLOX node keeps its model in `~/.cache/xblick/yolox/yolox_tiny.onnx`. The rule is the same for a node
that runs from a checkout and for one that pip installed. A node never keeps its weights in its checkout.

`<node>` is the node's short name: `yolox`, `sam3`, `gem-x`, `k4abt`. It is the program's name without `xblick-`. It
has lower-case letters, digits, `-` and `_`, starts with a letter or a digit, and has at most 64 characters. Any
other name is refused with an error.

## The variables

| Variable | Effect |
| --- | --- |
| `XBLICK_HOME=/srv/xblick` | Cache in `/srv/xblick/cache/<node>/`, data in `/srv/xblick/data/<node>/`. It takes precedence over the XDG variables. It does not move the config. |
| `XDG_CACHE_HOME`, `XDG_DATA_HOME`, `XDG_CONFIG_HOME` | Cache in `$XDG_CACHE_HOME/xblick/<node>/`, and the same for data and config. |
| `--weights-dir DIR` | A node's option. Its weights are in `DIR` and go there, in place of its cache directory. |

- A leading `~` is expanded in each variable.
- An XDG variable that is empty or relative is ignored, as the XDG Base Directory Specification says.
- An empty `XBLICK_HOME` counts as unset. A relative `XBLICK_HOME` is an error.
- Asking for a path creates nothing. A node creates its directory when it writes the first file.

`XBLICK_HOME` does not move the config. The config holds the presets of one user, while `XBLICK_HOME` often points
to a directory that several users share.

## Weights of a node

A node downloads missing weights at its start into its cache directory. Under a file lock it downloads each file
under a temporary name (`<name>.part`), checks it and renames it. A node that finds every file reads nothing from
the network and writes nothing.

| Option | Effect |
| --- | --- |
| `--download-only` | Download the missing weights, check them and end. It needs no camera, no Zenoh and no GPU. |
| `--no-download` | Never download. End with a message when a file is missing. |
| `--weights-dir DIR` | Use `DIR` in place of the cache directory. |

A node that kept its weights in its checkout before names that former place. When a file is missing in the cache
but is still at the former place, the node does not use it. It prints once the command that moves the directory:

```
xblick-yolox: the weights are kept in /home/me/.cache/xblick/yolox now, and some of them are still at the node's former place. They are not used from there. Move them (then start again):
  mkdir -p /home/me/.cache/xblick && mv /home/me/xblick-yolox/weights /home/me/.cache/xblick/yolox
```

After the message, the node continues as for any missing file. It downloads the file, or it ends with
`--no-download`. When the cache directory exists already, the command copies with `cp -a -n` and then removes the
former place.

## Copy the cache to a PC without internet

1. On a PC with internet, download with the options that select the weights:
   `xblick-yolox --download-only --model s`.
2. Copy the node's directory: `rsync -a ~/.cache/xblick/yolox/ robot:.cache/xblick/yolox/`.
3. On the robot, start the node with `--no-download`. A missing file then ends the node with a message and does not
   start a download.

## One directory for several users or containers

1. Create the directory with a group that all users share, for example `vision`:
   `sudo mkdir -p /srv/xblick && sudo chgrp vision /srv/xblick && sudo chmod 2775 /srv/xblick`.
2. Set `XBLICK_HOME=/srv/xblick` for every user, or mount the directory into each container and set the variable
   there.
3. Give the users a umask of `002`, so that the group can write the files and directories that a download creates.

Each node's cache directory holds a lock file, `.download.lock`. Two nodes that start at once and miss the same
file download it only once. The second waits for the lock and then finds the file. A user who downloads needs write
access to the directory and to the lock file. A node that finds all its files does not open the lock, so a
read-only directory works once it is complete. The lock is a `flock`. On a network file system it holds only if the
server supports such locks. In that case, run `--download-only` on one PC first and then start the nodes.

## Rings

A host ring is a POSIX shared memory object in `/dev/shm/xblick-<16 hex digits>`. Only the user of the publisher
can read it. The publisher removes it when it closes. A publisher that is killed with SIGKILL leaves it behind.
[Rings](../concepts/rings.md) explains them.

## In code

| Language | Function |
| --- | --- |
| Python | `xblick.paths.cache_dir(node)`, `data_dir(node)`, `config_dir()`, `check_node_name(node)` |
| Python | `xblick.weights.Store(prog, node=..., terms=..., former=[...])`, `Store.at(args.weights_dir)`, `Store.file(weight)` |
| C++ | `xblick/paths.hpp` in `xblick::core`: `xblick::cache_dir(node)`, `data_dir(node)`, `config_dir()`, `check_node_name(node)` |
