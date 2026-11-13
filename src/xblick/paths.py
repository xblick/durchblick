"""Paths given to a node on its command line: ``${VAR}``, ``$VAR`` and ``~`` expanded by the node itself.

A process manager may pass a node's arguments as they stand in its config, without a shell, so
``--objects-dir ${HOME}/...`` would arrive with ``${HOME}`` in it. The nodes take their
directories through ``expand_path``, which also refuses an unset variable rather than make a path of its name.
"""

from __future__ import annotations

import os
import re
from pathlib import Path

_VARIABLE = re.compile(r"\$\{?[A-Za-z_][A-Za-z0-9_]*\}?")


def expand_path(text: str) -> Path:
    """``text`` with its environment variables and ``~`` expanded; ValueError if a variable is not set."""
    expanded = os.path.expandvars(str(text))
    left = _VARIABLE.findall(expanded)
    if left:
        raise ValueError(f"{text}: {', '.join(left)} not set in the environment")
    return Path(os.path.expanduser(expanded))
