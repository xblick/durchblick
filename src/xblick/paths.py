"""Paths: where nodes keep their files (``cache_dir``, ``data_dir``, ``config_dir``), and paths given to a node on its
command line (``expand_path``).

Where files are (docs/reference/files.md; the same rule in C++, ``xblick/paths.hpp``):

    cache_dir("yolox")   $XBLICK_HOME/cache/yolox, else $XDG_CACHE_HOME/xblick/yolox, else ~/.cache/xblick/yolox
    data_dir("nemo")     $XBLICK_HOME/data/nemo,   else $XDG_DATA_HOME/xblick/nemo,   else ~/.local/share/xblick/nemo
    config_dir()         $XDG_CONFIG_HOME/xblick, else ~/.config/xblick (XBLICK_HOME does not move it)

The cache holds what can be fetched or built again (weights, TensorRT engines, compiled kernels), the data what a user
made and cannot download again. The rule is the same for a node started from a checkout and one installed with pip.
An XDG variable that is empty or relative (after ``~`` is expanded) is ignored, as the XDG Base Directory
Specification says. ``XBLICK_HOME`` empty counts as unset; relative, it is an error. Asking for a path creates nothing.

A process manager may pass a node's arguments as they stand in its config, without a shell, so
``--objects-dir ${HOME}/...`` would arrive with ``${HOME}`` in it. The nodes take their
directories through ``expand_path``, which also refuses an unset variable rather than make a path of its name.
"""

from __future__ import annotations

import os
import pwd
import re
from pathlib import Path

_VARIABLE = re.compile(r"\$\{?[A-Za-z_][A-Za-z0-9_]*\}?")

HOME_VARIABLE = "XBLICK_HOME"  # moves the cache and the data of every node: <it>/cache/<node>, <it>/data/<node>
NODE_NAME = re.compile(r"[a-z0-9][a-z0-9_-]{0,63}")  # a node's short name: yolox, sam3, gem-x, k4abt


def expand_path(text: str) -> Path:
    """``text`` with its environment variables and ``~`` expanded; ValueError if a variable is not set."""
    expanded = os.path.expandvars(str(text))
    left = _VARIABLE.findall(expanded)
    if left:
        raise ValueError(f"{text}: {', '.join(left)} not set in the environment")
    return Path(os.path.expanduser(expanded))


def check_node_name(node: str) -> str:
    """``node`` if it is a node's short name: lower-case letters, digits, ``-`` and ``_``, starting with a letter or a
    digit, at most 64 characters, and not the program's name (``yolox``, not ``xblick-yolox``); else ValueError."""
    if not isinstance(node, str) or not NODE_NAME.fullmatch(node):
        raise ValueError(f"node name {node!r}: lower-case letters, digits, '-' and '_', starting with a letter or a "
                         "digit, at most 64 characters (yolox, sam3, gem-x)")
    if node == "xblick" or node.startswith(("xblick-", "xblick_")):
        raise ValueError(f"node name {node!r}: the node's short name, without the program's 'xblick-' "
                         f"({node[7:] or 'yolox'})")
    return node


def home() -> Path:
    """The user's home: ``HOME``, else the password database; ValueError if ``HOME`` is set and not absolute."""
    value = os.environ.get("HOME")
    if value is None:
        return Path(pwd.getpwuid(os.getuid()).pw_dir)
    if not os.path.isabs(value):
        raise ValueError(f"HOME={value!r} is not an absolute path")
    return Path(value)


def _tilde(value: str) -> str:
    """``~`` and ``~/...`` with the home expanded (``~user`` stays as it is)."""
    if value == "~" or value.startswith("~/"):
        return str(home()) + value[1:]
    return value


def _xdg(variable: str, default: str) -> Path:
    """An XDG base directory: the variable when it is an absolute path (``~`` expanded), else ``~/<default>``."""
    value = _tilde(os.environ.get(variable, ""))
    return Path(value) if os.path.isabs(value) else home() / default


def xblick_home() -> Path | None:
    """``XBLICK_HOME`` (``~`` expanded), or None when it is unset or empty; ValueError when it is relative."""
    value = _tilde(os.environ.get(HOME_VARIABLE, ""))
    if not value:
        return None
    if not os.path.isabs(value):
        raise ValueError(f"{HOME_VARIABLE}={os.environ[HOME_VARIABLE]!r}: give an absolute path")
    return Path(value)


def cache_dir(node: str) -> Path:
    """Where the node ``node`` keeps what can be fetched or built again: weights, engines, compiled kernels."""
    check_node_name(node)
    base = xblick_home()
    return base / "cache" / node if base is not None else _xdg("XDG_CACHE_HOME", ".cache") / "xblick" / node


def data_dir(node: str) -> Path:
    """Where the node ``node`` keeps what a user made and cannot download again (e.g. a node's object memories)."""
    check_node_name(node)
    base = xblick_home()
    return base / "data" / node if base is not None else _xdg("XDG_DATA_HOME", ".local/share") / "xblick" / node


def config_dir() -> Path:
    """Durchblick's configuration (the terminal interface's presets); ``XBLICK_HOME`` does not move it."""
    return _xdg("XDG_CONFIG_HOME", ".config") / "xblick"
