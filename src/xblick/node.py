"""What every node does: its start (logging, SIGTERM) and its report (diagnostics about once a second, a log line
every ten seconds).

    log = node.start("xblick-sam3", ignore=["pkg_resources is deprecated"])
    report = node.Reporter(session, camera, "sam3", "xblick-sam3")
    try:
        while True:
            ...                                   # a frame done:
            report.counts["frames"] += 1
            if report.due():
                done, seconds = report.take()     # the counts since the last report
                level, message = node.state(f"{done['frames'] / seconds:.2f} frames/s", problems=[...],
                                            waiting=None if done["frames"] else color.describe())
                report.publish(level, message, {"frames/s": done["frames"] / seconds})
    except KeyboardInterrupt:                     # Ctrl-C, SIGTERM (a process manager: often SIGINT,
        pass                                      # then SIGTERM): the node ends through its cleanup
    finally:
        report.close()

A node's own arguments and ``--max-hz`` (``add_arguments``): at most so many frames a second, the newest, the others
skipped before they are decoded (``ImageSubscriber``/``PointCloudSubscriber``/``RGBDInputs`` ``max_hz``); the report
counts them (``watch``):

    node.add_arguments(parser, max_hz=30.0)             # --max-hz, the node's default (none given: every frame)
    log = node.start("xblick-sam3", max_hz=args.max_hz)       # logs "xblick-sam3: max 30 Hz" / "every frame"
    color = ImageSubscriber(session, base, transport="auto", max_hz=args.max_hz)
    report.watch(color)                                 # "max-hz" (the rate) and "skipped for max-hz" in reports

The diagnostics go to ``<camera>/diagnostics`` (``xblick diagnostics`` shows them); the log line is
``<prog>: <message><log_extra>``.
"""

from __future__ import annotations

import argparse
import collections
import logging
import signal
import time

from .camera import LOCAL
from .diagnostics import DiagnosticsPublisher
from .parameters import Parameters  # noqa: F401 - node.Parameters: a node's parameters (parameters.py)
from .logs import setup


def parse_max_hz(text: str) -> float | None:
    """``--max-hz``'s value: a rate above 0, or None for every frame (``0``, ``none``)."""
    import argparse

    if text.strip().lower() in ("0", "none"):
        return None
    try:
        hz = float(text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"{text}: a rate in Hz, or 0 or none for every frame") from None
    if hz == 0:
        return None
    if not hz > 0:  # negative, nan
        raise argparse.ArgumentTypeError(f"{text}: a rate above 0, or 0 or none for every frame")
    return hz


def rate_text(hz: float | None) -> str:
    """The effective rate in words: ``max 30 Hz`` or ``every frame``."""
    return f"max {hz:g} Hz" if hz else "every frame"


def add_arguments(parser, max_hz: float | None = None, local: tuple[str, ...] | None = None):
    """The arguments every node has: ``--max-hz``, its default ``max_hz`` (the node's own rate; None: every frame);
    ``--local`` (how frames of a publisher on this host reach it; default ``auto``) with the choices ``local``: all of
    camera.LOCAL for a node working on the GPU; ``("auto", "shm")`` for one working on the CPU, which refuses
    ``--local cuda`` with the reason (its subscribers without a device work on the CPU: auto takes shared memory);
    None (the default, so that a node's own ``--local`` does not clash until it uses this one): no ``--local``."""
    if max_hz is not None and not max_hz > 0:
        raise ValueError(f"default max_hz {max_hz}: a rate above 0, or None for every frame")
    parser.add_argument("--max-hz", type=parse_max_hz, default=max_hz, metavar="HZ",
                        help="process at most this many frames a second, always the newest; the frames between are "
                             "skipped before they are decoded or read from their ring; 0 or none: every frame it can "
                             f"take (default: {rate_text(max_hz)})")
    if local is None:
        return
    if "auto" not in local or not set(local) <= set(LOCAL):
        raise ValueError(f"local {local}: auto and some of {', '.join(LOCAL)}")
    gpu = "cuda" in local

    def parse_local(text: str) -> str:
        if text not in LOCAL:
            raise argparse.ArgumentTypeError(f"{text!r}: one of {', '.join(local)}")
        if text not in local:
            raise argparse.ArgumentTypeError(f"{text}: this node works on the CPU, frames reach it through shared "
                                             "memory (--local auto or shm)")
        return text

    parser.add_argument("--local", type=parse_local, default="auto", metavar="{" + ",".join(local) + "}",
                        help="frames of a publisher on this host: auto (the default: "
                             + ("its CUDA ring when that is on this node's GPU, else shared memory" if gpu else
                                "shared memory: this node works on the CPU")
                             + "), shm" + (", or cuda (its CUDA ring; stops if it offers none)" if gpu else ""))


def _subscriptions(watched) -> list:
    """The subscriptions (with ``skipped_max_hz``) in what a node gives ``Reporter.watch``: a subscription, a
    FrameHistory (its subscriber), RGBDInputs (its readers' subscribers)."""
    if hasattr(watched, "skipped_max_hz"):
        return [watched]
    if hasattr(watched, "subscriber"):
        return _subscriptions(watched.subscriber)
    if hasattr(watched, "readers"):
        return [s for r in watched.readers.values() for s in _subscriptions(r)]
    raise TypeError(f"{type(watched).__name__}: a subscription, FrameHistory or RGBDInputs to watch")


_UNSET = object()


def start(prog: str, ignore: list[str] = (), max_hz=_UNSET, local: str | None = None) -> logging.Logger:
    """A node's start: its logging (``logs.setup``) and SIGTERM ending it as Ctrl-C does (KeyboardInterrupt), through
    its cleanup; with ``max_hz`` (its ``--max-hz``) the line ``<prog>: max 30 Hz`` or ``<prog>: every frame``, with
    ``local`` (its ``--local``) ``<prog>: frames on this host: auto``; the transport each stream takes is logged
    when it is chosen (camera.py) and reported (Reporter). Call it before loading models (their libraries warn while
    loading)."""
    def interrupt(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupt)
    log = setup(prog, ignore)
    if max_hz is not _UNSET:
        print(f"{prog}: {rate_text(max_hz)}", flush=True)
    if local is not None:
        print(f"{prog}: frames on this host: {local}", flush=True)
    return log


def state(rate: str, problems: list[str] = (), waiting: str | None = None) -> tuple[int, str]:
    """(level, message) of a report: OK, or WARN with problems or with what it waits for (no frame came)."""
    message = ", ".join([rate, *problems]) + (f", waiting: {waiting}" if waiting else "")
    return (DiagnosticsPublisher.WARN if problems or waiting else DiagnosticsPublisher.OK), message


class Reporter:
    """A node's report about one camera: ``counts`` (any keys) counted by the node; every ``every`` seconds (``due``)
    the counts since the last report (``take``) become a diagnostics status ``name`` on ``<camera>/diagnostics``
    (``publish``), and every ``log_every`` seconds a log line (``<prog>: <log_prefix><message><log_extra>``)."""

    def __init__(self, session, camera: str, name: str, prog: str, log_prefix: str = "", every: float = 1.0,
                 log_every: float = 10.0, max_hz=_UNSET):
        self.counts: collections.Counter = collections.Counter()
        self.prog, self.log_prefix = prog, log_prefix
        self.every, self.log_every = every, log_every
        self._reported = self._logged = time.monotonic()
        self._last: collections.Counter = collections.Counter()
        self._watched: list = []
        self._skipped = 0  # of the watched subscriptions, at the last report
        self._max_hz = max_hz  # the node's --max-hz (in every report), or the watched subscriptions' rate
        self._diagnostics = DiagnosticsPublisher(session, camera, name)

    def watch(self, *subscriptions):
        """Subscriptions (or FrameHistory, RGBDInputs) whose frames left out for ``--max-hz`` every report counts:
        the value ``skipped for max-hz`` (since the last report), and in the message when there were any."""
        for s in subscriptions:
            self._watched.extend(_subscriptions(s))

    def due(self) -> bool:
        return time.monotonic() - self._reported >= self.every

    def take(self) -> tuple[collections.Counter, float]:
        """The counts since the last ``take`` and the seconds they took."""
        now = time.monotonic()
        done = collections.Counter({k: v - self._last[k] for k, v in self.counts.items()})
        seconds = max(now - self._reported, 1e-9)
        self._last, self._reported = collections.Counter(self.counts), now
        return done, seconds

    def publish(self, level: int, message: str, values: dict, log_extra: str = ""):
        """The diagnostics status; the log line too when ``log_every`` passed."""
        hz = self._max_hz
        if hz is _UNSET and self._watched:
            rates = {s.max_rate.hz if s.max_rate is not None else None for s in self._watched}
            hz = rates.pop() if len(rates) == 1 else _UNSET  # several rates: none is the node's
        if hz is not _UNSET:
            values = {**values, "max-hz": rate_text(hz)}
        for s in self._watched:  # the transport each stream reads, and why (camera.Subscription.transport_reason)
            if getattr(s, "transport_reason", ""):
                values = {**values, f"transport {s.base}": s.transport_reason}
        if self._watched:
            skipped = sum(s.skipped_max_hz for s in self._watched)
            values = {**values, "skipped for max-hz": skipped - self._skipped}
            if skipped > self._skipped:
                message += f", {skipped - self._skipped} skipped for max-hz"
            self._skipped = skipped
        self._diagnostics.publish(level, message, values)
        now = time.monotonic()
        if now - self._logged >= self.log_every:
            print(f"{self.prog}: {self.log_prefix}{message}{log_extra}", flush=True)
            self._logged = now

    def close(self):
        self._diagnostics.close()
