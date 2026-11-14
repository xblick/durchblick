"""Checks for every test.

A test whose threads outlive it fails in its teardown, with their names and stacks: a node's server thread, a
subscriber's helper, a Zenoh callback still running, a session left closing. Such a thread used to show up in a later
test instead, as a warning ("Exception in thread ...") or as a hang (issue #6).

A test that hangs while a thread holds the GIL ends the run as well: pytest-timeout (pyproject.toml, 120 s) needs the
GIL to dump the stacks and end the run; a faulthandler watchdog (C, no GIL) dumps every thread's stack and ends the
process HANG_EXIT_S after the test's own timeout. (pytest's faulthandler_timeout, which only dumps, is off: Python has
one such watchdog.)
"""

import faulthandler
import sys
import threading
import time
import traceback

import pytest
from _pytest.faulthandler import fault_handler_stderr_fd_key

THREADS_END_S = 6.0  # how long a test's threads may take to end after its teardown (session.close waits up to 5 s)
HANG_EXIT_S = 30.0   # the watchdog: this long after the test's timeout (pytest-timeout's turn first)


def _stack(thread: threading.Thread) -> str:
    frame = sys._current_frames().get(thread.ident)
    return "".join(traceback.format_stack(frame)) if frame is not None else "    (no Python frame)\n"


@pytest.fixture(autouse=True)
def no_thread_outlives_its_test():
    """Fails the test whose threads still run THREADS_END_S after its teardown. Autouse, so set up first and torn
    down last: after the test's own fixtures closed their sessions. pytest-timeout's timer is not the test's."""
    before = set(threading.enumerate())

    def left():
        return [t for t in threading.enumerate() if t not in before and t.is_alive()
                and not t.name.startswith("pytest_timeout ")]

    yield
    end = time.monotonic() + THREADS_END_S
    while left() and time.monotonic() < end:
        time.sleep(0.02)
    if remaining := left():
        pytest.fail(f"{len(remaining)} thread(s) of this test still run {THREADS_END_S:g} s after it: "
                    + "".join(f"\n--- {t.name} (daemon={t.daemon})\n{_stack(t)}" for t in remaining), pytrace=False)


@pytest.hookimpl(hookwrapper=True)
def pytest_runtest_protocol(item):
    """The GIL-free watchdog for the test: its timeout (marker or pyproject.toml) plus HANG_EXIT_S, then every thread's
    stack on the real stderr and the process ends."""
    marker = item.get_closest_marker("timeout")
    if marker and (marker.args or "timeout" in marker.kwargs):
        timeout = float(marker.args[0] if marker.args else marker.kwargs["timeout"])
    else:
        timeout = float(item.config.getoption("timeout", None) or item.config.getini("timeout") or 0)
    if timeout > 0:
        stderr = item.config.stash[fault_handler_stderr_fd_key]
        faulthandler.dump_traceback_later(timeout + HANG_EXIT_S, exit=True, file=stderr)
    try:
        yield
    finally:
        if timeout > 0:
            faulthandler.cancel_dump_traceback_later()
