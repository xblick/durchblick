"""Opening a Zenoh session the way Durchblick uses it.

Default: peer mode, multicast scouting off, the PC's Zenoh router. Every PC that runs Durchblick runs one router
(zenohd) on port 7447 (docs/concepts/sessions.md, "Zenoh routers"): without endpoints, a session connects to it at
127.0.0.1:7447 and listens on a free loopback port. Through the router the sessions on one PC find each other and
connect directly (images stay in shared memory or on the GPU, only their descriptors pass Zenoh); between PCs the
routers carry the data. On the robot network nothing is found by multicast by accident. Explicit endpoints
(``--connect``/``--listen``) replace the router (two programs connected directly, a test); a Zenoh JSON5
configuration file replaces everything (``config_file``).
"""

from __future__ import annotations

import json
import logging
import threading
import time
from collections.abc import Sequence

import zenoh

DEFAULT_PORT = 7447
ROUTER = f"tcp/127.0.0.1:{DEFAULT_PORT}"  # the PC's router
LOCAL_LISTEN = "tcp/127.0.0.1:0"         # a free loopback port: the PC's other sessions connect to it directly


def endpoint(address: str, port: int = DEFAULT_PORT) -> str:
    """A Zenoh endpoint from what a person types: ``host`` and ``host:port`` take ``tcp/``, and an address without a
    port takes ``port`` (``robot-pc`` and ``tcp/robot-pc`` are ``tcp/robot-pc:7447``; ``[::1]`` is
    ``tcp/[::1]:7447``). A locator with a port and a protocol without host and port (``unixsock-stream/...``) are
    returned unchanged; so is anything after ``#`` (Zenoh's endpoint configuration)."""
    address = address.strip()
    if not address:
        raise ValueError("an empty Zenoh endpoint")
    locator, hash_, config = address.partition("#")
    protocol, slash, rest = locator.partition("/")
    if not slash:
        protocol, rest = "tcp", locator
    if protocol not in ("tcp", "udp", "tls", "quic", "ws"):
        return address  # unixsock-stream/<path>, serial/...: no port
    if rest.startswith("["):  # IPv6: [addr] or [addr]:port
        has_port = "]:" in rest
    else:
        if rest.count(":") > 1:
            raise ValueError(f"Zenoh endpoint {address!r}: an IPv6 address goes in brackets, e.g. tcp/[::1]:7447")
        has_port = ":" in rest
    if not rest or rest.endswith(":"):
        raise ValueError(f"Zenoh endpoint {address!r}: no host or no port after ':'")
    return f"{protocol}/{rest}{'' if has_port else f':{port}'}{hash_}{config}"


def make_config(connect: Sequence[str] = (), listen: Sequence[str] = (), mode: str = "peer",
                multicast_scouting: bool = False, config_file: str | None = None,
                local_router: bool = True) -> zenoh.Config:
    """Neither ``connect`` nor ``listen`` (nor a config file), with ``local_router``: the PC's router (``ROUTER``),
    listening on ``LOCAL_LISTEN``. ``local_router=False`` without endpoints: a session that reaches nobody."""
    if config_file:
        if connect or listen:
            raise ValueError("give either a Zenoh config file or connect/listen endpoints, not both")
        return zenoh.Config.from_file(config_file)
    if mode not in ("peer", "client"):
        raise ValueError(f"mode {mode!r}: peer or client")
    if local_router and not connect and not listen:
        connect, listen = [ROUTER], [LOCAL_LISTEN] if mode == "peer" else []
    if mode == "client" and not connect:
        raise ValueError("client mode needs an endpoint to connect to (a router or a peer)")
    if mode == "client" and listen:
        raise ValueError("client mode does not listen: give connect endpoints only (or peer mode)")
    config = zenoh.Config()
    config.insert_json5("mode", json.dumps(mode))
    config.insert_json5("scouting/multicast/enabled", json.dumps(bool(multicast_scouting)))
    config.insert_json5("connect/endpoints", json.dumps([endpoint(e) for e in connect]))
    if mode == "peer":
        config.insert_json5("listen/endpoints", json.dumps([endpoint(e) for e in listen]))
    return config


def open_session(connect: Sequence[str] = (), listen: Sequence[str] = (), mode: str = "peer",
                 multicast_scouting: bool = False, config_file: str | None = None,
                 local_router: bool = True) -> Session:
    return Session(zenoh.open(make_config(connect, listen, mode, multicast_scouting, config_file, local_router)))


def add_arguments(parser, prefix: str = ""):
    """``--<prefix>connect``, ``--<prefix>listen``, ``--<prefix>zenoh-mode``, ``--<prefix>config`` for a script's
    argparse parser."""
    p = f"--{prefix}"
    parser.add_argument(f"{p}connect", nargs="*", default=[], metavar="ENDPOINT",
                        help="Zenoh endpoints to connect to: host, host:port or a locator like tcp/192.0.2.10:7447 "
                             "(default, without --listen: this PC's router, 127.0.0.1:7447)")
    parser.add_argument(f"{p}listen", nargs="*", default=[], metavar="ENDPOINT",
                        help="Zenoh endpoints to listen on, e.g. 0.0.0.0:7448 (default, without --connect: a free "
                             "loopback port)")
    parser.add_argument(f"{p}zenoh-mode", choices=("peer", "client"), default="peer",
                        help="peer (default): listens, and the PC's sessions connect to it directly; client: everything "
                             "through the router --connect names, which a session that only connects needs (a router "
                             "does not route between peers that do not listen)")
    parser.add_argument(f"{p}config", default=None, metavar="FILE",
                        help="a Zenoh JSON5 configuration file instead of the endpoints")


def open_from_args(args, prefix: str = "") -> Session:
    key = prefix.replace("-", "_")
    mode = getattr(args, f"{key}zenoh_mode")
    if getattr(args, f"{key}config") and mode != "peer":
        raise ValueError("give the mode in the Zenoh config file, not --zenoh-mode")
    return open_session(connect=getattr(args, f"{key}connect"), listen=getattr(args, f"{key}listen"), mode=mode,
                        config_file=getattr(args, f"{key}config"))


def alone(session) -> str:
    """Why a session that found nothing may have found nothing: it reached no Zenoh router and no other session.
    Empty when it reached one."""
    if list(session.info.routers_zid()) or list(session.info.peers_zid()):
        return ""
    return ("this session reached no Zenoh router and no other session: start zenohd on this PC (a session "
            f"without --connect looks for it at {ROUTER}), or name the router or session with --connect")


# ---------------------------------------------------------------------------------------------- queries

log = logging.getLogger("xblick")
not_finalized: dict[str, int] = {}  # selector -> queries Zenoh did not finalize by their deadline (logged once each)
_not_finalized_lock = threading.Lock()


def deadline_of(timeout: float) -> float:
    """How long a query of ``timeout`` s is waited for at most: its timeout and ``max(0.5, timeout)`` more."""
    return timeout + max(0.5, timeout)


def collect(handler, what: str, timeout: float, cancel=None) -> list:
    """The replies of a query's handler (a get's FIFO channel), read to its end by a short daemon thread: Zenoh's
    channel ends once it finalized the query. Waited for at most ``deadline_of(timeout)``: Zenoh does not always
    finalize a query by its timeout (seen once in CI: a get with timeout=1.0 that iterated for 5 min), and iterating
    its replies would then block for good. At the deadline it cancels the query (``cancel``, the get's
    CancellationToken: Session.close() waits for good for a query Zenoh did not finalize, seen in the tests), goes on
    with the replies that came, counts the query in ``not_finalized`` and logs it once per selector. The thread reads in
    Rust without the GIL, and no Python callback runs per reply: a query under GIL load costs what iterating the get
    costs. Error replies (``r.ok`` None) are kept as they came."""
    got: list = []

    def drain():
        for reply in handler:
            got.append(reply)

    reader = threading.Thread(target=drain, name=f"replies {what}", daemon=True)
    reader.start()
    reader.join(deadline_of(timeout))
    if reader.is_alive() and cancel is not None:
        cancel.cancel()  # the query ended by us: a query Zenoh did not finalize makes Session.close() wait for good
        reader.join(1.0)
    if reader.is_alive() or (cancel is not None and cancel.is_cancelled):
        with _not_finalized_lock:
            not_finalized[what] = not_finalized.get(what, 0) + 1
            first = not_finalized[what] == 1
        if first:
            log.warning(f"query {what} not finalized by Zenoh after {deadline_of(timeout):.1f} s (its timeout "
                        f"{timeout:g} s): going on with the {len(got)} replies that came")
        return list(got)
    return got


def replies(session: zenoh.Session, selector: str, timeout: float, liveliness: bool = False, **get_options) -> list:
    """The replies to a query (``session.get``, or ``session.liveliness().get`` with ``liveliness``; its other
    options in ``get_options``: payload, ...), within a deadline (``collect``). Use it instead of iterating a get's
    replies (CLAUDE.md, "Traps")."""
    getter = session.liveliness().get if liveliness else session.get
    if not liveliness:
        # every reply, not one per key (Zenoh's default consolidation): several publishers answer a stream's info key
        # (a source and its relays), and a caller that expects one node per service key counts the answers
        get_options.setdefault("consolidation", zenoh.ConsolidationMode.NONE)
    cancel = zenoh.CancellationToken()
    return collect(getter(selector, timeout=timeout, cancellation_token=cancel, **get_options), selector, timeout,
                   cancel)


ASK_PERIOD = 0.25  # s: how often ask() sends a read again while nothing answered


def ask(session: zenoh.Session, selector: str, timeout: float, **get_options) -> list:
    """The replies to a query that only reads (an info record, a status, a node's parameter values, rays), asked
    again every ASK_PERIOD seconds until a first reply comes or ``timeout`` passed.

    A session just opened knows no queryable yet: through a router their declarations arrive some 0.1-0.3 s later,
    and a query sent before ends at once without a reply. Asking again is right only for reads: an empty result
    does not prove that no queryable got the query (a queryable that drops it without a reply also gives an empty
    result at once), so a command would run twice. Commands go through ``ask_once``."""
    end = time.monotonic() + timeout
    while True:
        answers = replies(session, selector, max(end - time.monotonic(), 0.01), **get_options)
        if answers or time.monotonic() >= end:
            return answers
        time.sleep(min(ASK_PERIOD, max(0.0, end - time.monotonic())))


CLOSE_TIMEOUT_S = 5.0  # how long a close waits for Zenoh (Session, close)


def close(session, timeout: float = CLOSE_TIMEOUT_S) -> bool:
    """Closes ``session`` (a zenoh.Session or a Session), waiting at most ``timeout`` s: Zenoh's close blocked for
    good after a query it did not finalize (seen in the tests, also with that query cancelled), and a program must
    still end. True when closed; False: it is left closing in a daemon thread, logged."""
    raw = session._session if isinstance(session, Session) else session
    closer = threading.Thread(target=raw.close, name="zenoh close", daemon=True)
    closer.start()
    closer.join(timeout)
    if closer.is_alive():
        log.warning(f"the Zenoh session {raw.info.zid()} did not close within {timeout:g} s: going on without "
                    "waiting for it")
        return False
    return True


class Session:
    """A zenoh.Session (``open_session``, ``open_from_args``) whose close, also at the end of a ``with`` block, waits
    at most CLOSE_TIMEOUT_S (``close``). Everything else is the session's own: attributes go to it, and
    ``isinstance(s, zenoh.Session)`` holds (zenoh.Session cannot be subclassed)."""

    __slots__ = ("_session",)

    def __init__(self, session: zenoh.Session):
        object.__setattr__(self, "_session", session)

    def __getattr__(self, name):
        return getattr(self._session, name)

    @property
    def __class__(self):  # isinstance(s, zenoh.Session); type(s) is still Session
        return zenoh.Session

    def close(self, timeout: float = CLOSE_TIMEOUT_S) -> bool:
        return close(self._session, timeout)

    def __enter__(self) -> Session:
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def __repr__(self) -> str:
        return f"xblick Session({self._session!r})"
