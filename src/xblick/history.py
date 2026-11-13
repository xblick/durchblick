"""The last frames of a stream by their stamps, for nodes that pair streams or look a frame up later.

A subscriber keeps only the newest frame: a node busy for tens of milliseconds with one frame would lose the frames
other streams refer to by stamp (the depth of a colour frame, the image a segmenter's boxes are about). A
``FrameHistory`` takes every frame as it comes, in a thread of its own, and keeps the last ones by stamp.

    wake = threading.Event()
    color = FrameHistory(ImageSubscriber(session, "robot/camera/color", transport="auto"), 10, on_put=wake.set)
    wake.wait(0.1)
    frame = color.at(stamp)          # or color.newest()

A stream paired with another (the depth of a colour frame, a mask of an image) is the partner of a primary stream: a
``PartnerHistory`` keeps its last samples as they came, and reads (decodes, or copies out of the publisher's ring)
only the one a primary frame is paired with (``at``; ``wait`` for one that may still come), by default the nearest
within half the partner's frame period. Take a frame's partners when you take the frame, before working on it: a
shared-memory or CUDA sample is valid only until its ring slot is reused. With ``--max-hz`` only the primary is
limited (its subscriber's ``max_hz``); the partner's frames of skipped primary frames are never read, and no pair is
lost to the limit (each stream limited on its own would lose the pairs whose stamps lie on both sides of a period's
edge).

Several histories that a node looks at together can share one lock (``lock``, e.g. a ``threading.Condition`` whose
``notify_all`` is ``on_put``): it then reads and changes ``items`` under that lock directly.
"""

from __future__ import annotations

import collections
import logging
import threading
import time
from collections.abc import Callable

from . import _core

log = logging.getLogger("xblick")


class StampedHistory:
    """The newest ``size`` items by stamp (``items``, in arrival order; ``newest_stamp`` the largest stamp put so far).
    ``on_put`` is called after every put, with ``lock`` held when one was given (else outside the history's own)."""

    def __init__(self, size: int, on_put: Callable[[], None] | None = None, lock=None):
        self.items: collections.OrderedDict = collections.OrderedDict()
        self.newest_stamp = -1
        self.size = size
        self._on_put = on_put
        self._shared = lock is not None
        self.lock = lock if lock is not None else threading.Lock()

    def put(self, stamp_ns: int, item):
        with self.lock:
            self.items[stamp_ns] = item
            self.newest_stamp = max(self.newest_stamp, stamp_ns)
            while len(self.items) > self.size:
                self.items.popitem(last=False)
            if self._shared and self._on_put is not None:
                self._on_put()
        if not self._shared and self._on_put is not None:
            self._on_put()

    def stamps(self) -> set[int]:
        with self.lock:
            return set(self.items)

    def has(self, stamp_ns: int) -> bool:
        """An item of exactly this stamp."""
        with self.lock:
            return stamp_ns in self.items

    def at(self, stamp_ns: int):
        with self.lock:
            return self.items.get(stamp_ns)

    def newest(self):
        """(stamp, item) of the largest stamp kept, or None."""
        with self.lock:
            if not self.items:
                return None
            stamp = max(self.items)
            return stamp, self.items[stamp]


class FrameHistory(StampedHistory):
    """A ``StampedHistory`` of the frames of ``subscriber`` (an ImageSubscriber or PointCloudSubscriber), taken by a
    thread of its own. An error of the subscriber (another protocol version) ends the thread; ``check`` raises it.
    ``close`` also closes the subscriber."""

    def __init__(self, subscriber, size: int, on_put: Callable[[], None] | None = None, lock=None):
        super().__init__(size, on_put, lock)
        self.subscriber = subscriber
        self._error: BaseException | None = None
        self._closed = threading.Event()
        self._thread = threading.Thread(target=self._run, name=f"frames {subscriber.base}", daemon=True)
        self._thread.start()

    def _run(self):
        try:
            while not self._closed.is_set():
                frame = self.subscriber.latest(timeout=0.1)
                if frame is not None:
                    self.put(frame.header.stamp.to_ns(), frame)
        except BaseException as error:  # noqa: BLE001 - raised in the caller's thread by check()
            self._error = error
            if self._on_put is not None:  # wake the reader, whose check() raises it
                if self._shared:
                    with self.lock:
                        self._on_put()
                else:
                    self._on_put()

    def check(self):
        if self._error is not None:
            raise self._error

    def close(self):
        self._closed.set()
        self._thread.join()
        self.subscriber.close()


LOST_HINT = ("the partner frames are read too late: take a frame and its partners together, before working on it "
             "(PartnerHistory.wait, RGBDInputs.take), or give the publisher more ring slots (Durchblick's default 32; "
             "a camera node's slots setting)")


class PartnerHistory:
    """The frames of a partner stream (``subscriber``, an ImageSubscriber), paired with a primary stream's frames by
    stamp: its last ``size`` samples kept as they came, and each read (decoded, or copied out of its publisher's
    shared-memory or CUDA ring) only when a primary frame is paired with it (``at``, ``wait``; ``has`` whether there is
    one), each once. ``on_put`` is called after each sample (wakes the node: the partner may come after the primary).

    The partner of a primary frame is the nearest sample within ``max_offset_ns``. None (the default): within half the
    partner's frame period, measured from its kept samples, so any steady stream pairs every primary frame with its
    nearest sample and a missing one is not replaced by its neighbour; an explicit value overrides that, 0 asks for
    equal stamps.

    A sample over shm or CUDA only names a slot of the publisher's ring, which it overwrites ``slots`` frames later
    (32 by default: about a second at 30 Hz). Read a primary frame's partners when you take the primary frame, not
    after working on it. A partner overwritten before it was read counts in ``lost`` and is logged (at most every
    10 s) with that hint. ``decoded`` counts the frames read. ``close`` also closes the subscriber."""

    LOG_EVERY_S = 10.0

    def __init__(self, subscriber, size: int, max_offset_ns: int | None = None,
                 on_put: Callable[[], None] | None = None):
        if max_offset_ns is not None and max_offset_ns < 0:
            raise ValueError(f"max_offset_ns {max_offset_ns}: not negative (None: half the partner's frame period)")
        self.subscriber = subscriber
        self.size = size
        self.max_offset_ns = max_offset_ns
        self.lost = 0  # partners found but overwritten in their ring before they were read
        self._logged_lost, self._last_log = 0, -float("inf")
        self._arrived = threading.Condition()
        self._arrivals = 0  # samples come so far (wait: whether one came since it looked)
        self._on_put = on_put
        subscriber.keep_recent(size, self._put)
        self._decoded: collections.OrderedDict = collections.OrderedDict()  # primary stamp -> its frame
        self.lock = threading.Lock()

    def _put(self):
        with self._arrived:
            self._arrivals += 1
            self._arrived.notify_all()
        if self._on_put is not None:
            self._on_put()

    @property
    def decoded(self) -> int:
        return self.subscriber.decoded

    def stamps(self) -> set[int]:
        """The stamps of the samples kept (not read)."""
        return set(self.subscriber.recent_stamps())

    def period_ns(self) -> int | None:
        """The partner's frame period: the median step of the stamps kept; None before two of them."""
        return _core.partner_period_ns(self.subscriber.recent_stamps())

    def offset_ns(self) -> int | None:
        """The largest offset that pairs: ``max_offset_ns``, or half the measured period (None while unmeasured)."""
        if self.max_offset_ns is not None:
            return self.max_offset_ns
        period = self.period_ns()
        return None if period is None else period // 2

    def partner_of(self, stamp_ns: int) -> int | None:
        """The stamp of the partner frame of the primary frame stamped ``stamp_ns``: the nearest kept, at most
        ``offset_ns()`` away, once no nearer one can still come; None until then. The C++ ``partner_of``
        (``xblick/pairing.hpp``) decides, so that C++ nodes pair alike.

        Stamps only grow: a later sample is nearer only if it is stamped before ``stamp_ns`` plus the nearest one's
        distance before it. A steady partner (the steps of its kept samples within a tenth of their median) sends its
        next sample about one step after the newest kept, so it can be nearer only by coming more than half a step
        early. A nearest sample at most a quarter of the smallest step before ``stamp_ns`` (a camera of a pair that
        stamps a little earlier) is therefore taken at once, with the measured offset and with an explicit one. A
        partner that is not steady (a node that skips camera frames) or has fewer than two samples waits for a later
        sample."""
        return _core.partner_of(self.subscriber.recent_stamps(), stamp_ns, self.max_offset_ns)

    def has(self, stamp_ns: int) -> bool:
        return self.partner_of(stamp_ns) is not None

    def at(self, stamp_ns: int):
        """The partner's frame of the primary frame stamped ``stamp_ns``, read (once) now; None without one (yet),
        or when its ring slot was overwritten meanwhile (counted in ``lost``, logged with the fix)."""
        with self.lock:
            if stamp_ns in self._decoded:
                return self._decoded[stamp_ns]
        partner = self.partner_of(stamp_ns)
        if partner is None:
            return None
        stale = self.subscriber.stale
        frame = self.subscriber.at(partner, 0)
        if frame is None:
            if self.subscriber.stale > stale:
                self._count_lost()
            return None
        with self.lock:
            self._decoded[stamp_ns] = frame
            while len(self._decoded) > self.size:
                self._decoded.popitem(last=False)
        return frame

    def wait(self, stamp_ns: int, timeout: float):
        """``at``, waiting up to ``timeout`` seconds for the partner to come (it may come after the primary frame);
        None if it does not, or was overwritten. For a node that takes a primary frame and then works on it: its
        partner read with it, before the work."""
        end = time.monotonic() + timeout
        while True:
            with self._arrived:
                seen = self._arrivals
            # outside _arrived: the subscriber's locks are never taken while holding it (its Zenoh thread takes it)
            paired = self.has(stamp_ns)
            frame = self.at(stamp_ns)
            remaining = end - time.monotonic()
            if frame is not None or paired or remaining <= 0 or self.subscriber._error is not None:
                return frame  # None when paired: overwritten in its ring (counted in lost)
            with self._arrived:
                self._arrived.wait_for(lambda seen=seen: self._arrivals != seen, remaining)

    def _count_lost(self):
        with self.lock:
            self.lost += 1
            now = time.monotonic()
            if now - self._last_log < self.LOG_EVERY_S:
                return
            n, self._logged_lost, self._last_log = self.lost - self._logged_lost, self.lost, now
        log.warning("%s: %d partner frame(s) overwritten in the publisher's ring before they were read (%d in all; "
                    "or its ring gone: the publisher restarted); %s", self.subscriber.base, n, self.lost, LOST_HINT)

    def check(self):
        if self.subscriber._error is not None:
            raise self.subscriber._error

    def close(self):
        self.subscriber.close()
