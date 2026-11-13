"""Logging of Durchblick's programs: one call at the start of a node's ``main``.

    from xblick.logs import setup
    setup("xblick-nemo", ignore=["xFormers is not available"])

Messages of the ``xblick`` package (a stream found, the transport it is read with, a publisher moved) and of the
program's own logger go to stdout as ``<prog>: <message>``: a process manager that starts nodes may log a program's
stdout as INFO and its stderr as WARN, so only what is a warning should reach stderr. Python warnings that tell an
operator nothing are hidden: FutureWarning and DeprecationWarning (the libraries' own deprecations) and the messages in
``ignore`` (a library saying that an optional speed-up is missing, for instance). Every other warning still shows.
"""

from __future__ import annotations

import logging
import re
import sys
import warnings


def setup(prog: str, ignore: list[str] = (), level: int = logging.INFO) -> logging.Logger:
    """Configure the logging of the program ``prog`` (call it before loading models: their libraries warn then).
    Returns the program's logger."""
    logging.basicConfig(level=level, stream=sys.stdout, format=f"{prog}: %(message)s", force=True)
    warnings.filterwarnings("ignore", category=FutureWarning)
    warnings.filterwarnings("ignore", category=DeprecationWarning)
    for message in ignore:
        warnings.filterwarnings("ignore", message=f".*{re.escape(message)}")
    return logging.getLogger(prog)
