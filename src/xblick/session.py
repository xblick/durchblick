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

import ipaddress
import json
import logging
import socket
import sys
import threading
import time
from collections.abc import Callable, Sequence
from pathlib import Path

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
    listening on ``LOCAL_LISTEN``. A peer with ``connect`` and no ``listen`` listens on ``LOCAL_LISTEN`` as well.
    ``local_router=False`` without endpoints: a session that reaches nobody."""
    if config_file:
        if connect or listen:
            raise ValueError("give either a Zenoh config file or connect/listen endpoints, not both")
        return zenoh.Config.from_file(config_file)
    if mode not in ("peer", "client"):
        raise ValueError(f"mode {mode!r}: peer or client")
    if local_router and not connect and not listen:
        connect, listen = [ROUTER], [LOCAL_LISTEN] if mode == "peer" else []
    elif mode == "peer" and connect and not listen:
        # A peer always listens: behind a Zenoh 1.10 router (zenohd or xblick router) peers reach each other only by
        # connecting directly (gossip), the router does not relay between them; a peer that only connected found
        # nothing (before 0.5.3).
        listen = [LOCAL_LISTEN]
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
    """A session (``make_config``). It keeps the endpoints of ``across_pcs`` for ``why_nothing``."""
    config = make_config(connect, listen, mode, multicast_scouting, config_file, local_router)
    return Session(zenoh.open(config), across_pcs(connect, listen, mode, config_file))


# ---------------------------------------------------------------------------------------------- across PCs

IP_PROTOCOLS = ("tcp", "udp", "tls", "quic", "ws")


def host_of(locator: str) -> str | None:
    """The host of an endpoint (after ``endpoint``), IPv6 without brackets; None for a locator without a host
    (``unixsock-stream/<path>``, ``serial/...``)."""
    protocol, _, rest = endpoint(locator).partition("#")[0].partition("/")
    if protocol not in IP_PROTOCOLS:
        return None
    if rest.startswith("["):
        return rest[1:rest.index("]")]
    return rest.rpartition(":")[0]


def _addresses(host: str) -> list[tuple]:
    """(family, sockaddr) of each address ``host`` resolves to; none when it does not resolve."""
    try:
        return [(family, sockaddr) for family, _, _, _, sockaddr in socket.getaddrinfo(host, None,
                                                                                      type=socket.SOCK_STREAM)]
    except (socket.gaierror, UnicodeError):
        return []


def _named_loopback(host: str) -> bool:
    return host.lower() == "localhost" or host.lower().endswith(".localhost")


def this_pc(host: str) -> bool:
    """True when ``host`` is this PC: ``localhost``, or a name or address that resolves to a loopback address, the
    unspecified address (0.0.0.0, ::) or an address of one of this PC's interfaces. An address is this PC's when a
    socket can bind to it, which sends nothing over the network. A name that does not resolve is not this PC."""
    if _named_loopback(host):
        return True
    for family, sockaddr in _addresses(host):
        ip = ipaddress.ip_address(sockaddr[0].split("%")[0])
        if ip.is_loopback or ip.is_unspecified:
            return True
        with socket.socket(family, socket.SOCK_STREAM) as s:
            try:
                s.bind((sockaddr[0], 0, *sockaddr[2:]))
                return True
            except OSError:
                pass
    return False


def loopback(host: str) -> bool:
    """True when ``host`` is ``localhost`` or every address it resolves to is a loopback address: a session that
    listens there is reachable from this PC only."""
    if _named_loopback(host):
        return True
    addresses = _addresses(host)
    return bool(addresses) and all(ipaddress.ip_address(a[0].split("%")[0]).is_loopback for _, a in addresses)


def across_pcs(connect: Sequence[str], listen: Sequence[str], mode: str, config_file: str | None = None,
               local: Callable[[str], bool] = this_pc, loopback_only: Callable[[str], bool] = loopback) -> list[str]:
    """The ``connect`` endpoints (as ``endpoint`` writes them) when the session is a peer that connects only to other
    PCs and listens on loopback only, else none. Behind a Zenoh 1.10 router peers reach each other only directly, so
    the programs on the other PCs usually cannot reach this one. They can when they listen on network addresses, which
    the options of this session do not show. Decided from the options alone: no option with this PC's own router (a
    ``connect`` endpoint on this PC, ``local``), no ``listen`` on a network address (``loopback_only``), no config
    file, peer mode. A locator without a host (``unixsock-stream/...``) counts as this PC."""
    if config_file or mode != "peer" or not connect:
        return []
    endpoints = [endpoint(e) for e in connect]
    hosts = [host_of(e) for e in endpoints]
    if any(h is None or local(h) for h in hosts):
        return []
    if any(h is not None and not loopback_only(h) for h in map(host_of, listen)):
        return []
    return endpoints


def across_pcs_text(endpoints: Sequence[str], prefix: str = "") -> str:
    """Why such a session (``across_pcs``) usually finds nothing, and the three ways out, with its endpoints: client
    mode, a router on this PC, or listening on the network (the peers there learn its address through their router and
    connect to it)."""
    given = " ".join(endpoints)
    return (f"connecting as a peer to another PC ({given}): programs there usually cannot reach this one, which "
            f"listens on loopback only. Use --{prefix}zenoh-mode client, or start a router on this PC with: xblick "
            f"router --connect {given}, and this program without --{prefix}connect, or make this program reachable from there "
            f"with --{prefix}listen tcp/0.0.0.0:0 (any program on the network can then connect to it)")


def program_name() -> str:
    """The program's name for its messages: the name it was started by (a node's console script)."""
    return Path(sys.argv[0]).name if sys.argv and sys.argv[0] else "python"


def add_arguments(parser, prefix: str = ""):
    """``--<prefix>connect``, ``--<prefix>listen``, ``--<prefix>zenoh-mode``, ``--<prefix>config`` for a script's
    argparse parser."""
    p = f"--{prefix}"
    parser.add_argument(f"{p}connect", nargs="*", default=[], metavar="ENDPOINT",
                        help="Zenoh endpoints to connect to: host, host:port or a locator like tcp/192.0.2.10:7447 "
                             "(default, without --listen: this PC's router, 127.0.0.1:7447)")
    parser.add_argument(f"{p}listen", nargs="*", default=[], metavar="ENDPOINT",
                        help="Zenoh endpoints to listen on, e.g. 0.0.0.0:7448 (default for a peer: a free loopback "
                             "port, so that the PC's other sessions connect to it directly)")
    parser.add_argument(f"{p}zenoh-mode", choices=("peer", "client"), default="peer",
                        help="peer (default): listens, and the PC's sessions connect to it directly; client: everything "
                             "through the router --connect names, which a session needs that connects to another PC's "
                             "router (a router does not relay between peers)")
    parser.add_argument(f"{p}config", default=None, metavar="FILE",
                        help="a Zenoh JSON5 configuration file instead of the endpoints")


def open_from_args(args, prefix: str = "", prog: str | None = None) -> Session:
    """The session of a script's options (``add_arguments``). A peer that connects only to other PCs and listens on
    loopback only (``across_pcs``) is said on stderr first, as ``<prog>: ...`` (default: ``program_name()``). Only a
    notice: the session opens as given."""
    key = prefix.replace("-", "_")
    mode = getattr(args, f"{key}zenoh_mode")
    connect, listen, config_file = getattr(args, f"{key}connect"), getattr(args, f"{key}listen"), getattr(
        args, f"{key}config")
    if config_file and mode != "peer":
        raise ValueError("give the mode in the Zenoh config file, not --zenoh-mode")
    say_across_pcs(connect, listen, mode, config_file, prog or program_name(), prefix)
    return open_session(connect=connect, listen=listen, mode=mode, config_file=config_file)


def say_across_pcs(connect: Sequence[str], listen: Sequence[str], mode: str, config_file: str | None, prog: str,
                   prefix: str = "") -> bool:
    """The notice of ``across_pcs`` on stderr, one line, when it applies. True when said."""
    endpoints = across_pcs(connect, listen, mode, config_file)
    if endpoints:
        print(f"{prog}: {across_pcs_text(endpoints, prefix)}", file=sys.stderr, flush=True)
    return bool(endpoints)


def why_nothing(session) -> str:
    """Why a session that found nothing may have found nothing: it is a peer that connects only to other PCs
    (``across_pcs``, kept by ``open_session``), or it reached no router and no other session (``alone``). Empty when
    neither holds."""
    endpoints = getattr(session, "across_pcs", ())
    return f"this session is {across_pcs_text(endpoints)}" if endpoints else alone(session)


def alone(session) -> str:
    """Why a session that found nothing may have found nothing: it reached no Zenoh router and no other session.
    Empty when it reached one."""
    if list(session.info.routers_zid()) or list(session.info.peers_zid()):
        return ""
    return ("this session reached no Zenoh router and no other session: start a router on this PC with xblick router "
            f"(or zenohd; a session without --connect looks for it at {ROUTER}), or name the router or session with "
            "--connect")


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


def timed_out(reply) -> bool:
    """The error reply by which Zenoh ends a query whose timeout passed before a queryable answered: its text is
    "Timeout" (eclipse-zenoh 1.10; tests/test_queries.py pins it). The queryable may still be working on it."""
    return reply.err is not None and reply.err.payload.to_string() == "Timeout"


class NotReached(TimeoutError):
    """A command was not sent: no queryable for its key became known to the session within the timeout. Safe to try
    again (a TimeoutError, as before)."""


class NoAnswer(RuntimeError):
    """A command was sent once and no answer came within the timeout: it may still be executed (a busy or briefly
    unreachable receiver executes it when it gets it; a command has no expiry on the wire), and it was not sent
    again. Deliberately no TimeoutError, so that ``except TimeoutError: try again`` does not send it a second time;
    a RuntimeError, as a refusal, which callers do not repeat."""


class PartlyAnswered(NoAnswer):
    """A command was answered by some node(s), and at least one more node got it and gave no answer in time (Zenoh's
    "Timeout" reply): that node may still execute it. Not sent again. A NoAnswer, so no TimeoutError."""


class SeveralExecuted(NoAnswer):
    """A command was sent and answered by more than one node: each of them executed it (two nodes on one key, which
    the check before the command did not see). Not sent again. A NoAnswer, so no TimeoutError."""


HOLDING = "a second node, or the answering node still holding the query after its reply"  # why a late reply comes


def ask_once(session: zenoh.Session, selector: str, timeout: float, **get_options) -> list:
    """The replies to a command (a parameter set, a controller switch, a torque step, a recorder command): sent once
    to each queryable of ``selector``, after the session knows one. NotReached when none became known within
    ``timeout`` (nothing sent). The replies as they came, possibly none; ``command`` interprets them.

    Once per queryable: two queryables on the key (two nodes, two bridges) both get it and both may execute it. The
    callers check first, as far as the protocol lets them, that one node serves the key (``count``): a best-effort
    check that catches two nodes that both answer a read, not a guarantee.

    Why the command is never sent again: Zenoh ends a query without any reply in cases that the sender cannot tell
    apart. The session knew no queryable, so nothing went out (the case of a session just opened). Or a queryable
    got the query and dropped it without a reply, which also ends it at once. Or a connection on the way was lost.
    A command sent again in the later cases would run twice. Instead, ask_once waits until a querier on the key
    reports a matching queryable (Zenoh's own routing knowledge, also through a router) and then sends the command
    once. A queryable slower than the timeout gives Zenoh's error reply "Timeout" (``timed_out``).

    Time: up to ``timeout`` for the queryable, then the query's own timeout ``timeout``; when Zenoh does not end the
    query by then, ``collect`` waits up to ``deadline_of(timeout)`` (``timeout + max(0.5, timeout)``) and 1 s more
    for the cancellation. At most about 3 x timeout + 1 s in all (measured in tests/test_queries.py)."""
    end = time.monotonic() + timeout
    querier = session.declare_querier(selector)
    try:
        while not querier.matching_status.matching:
            if time.monotonic() >= end:
                raise NotReached(f"not sent: no node serves {selector} (none became known within {timeout:g} s)")
            time.sleep(0.01)
    finally:
        querier.undeclare()
    return replies(session, selector, timeout, **get_options)


def command(session: zenoh.Session, selector: str, timeout: float, **get_options) -> list:
    """A command sent with ``ask_once`` and its answers checked: the successful replies (one or more), else
    NotReached (not sent), RuntimeError with the first error reply's text (an error reply other than Zenoh's own
    timeout), NoAnswer (sent once, no answer within ``timeout``: it may still be executed; not sent again),
    PartlyAnswered (answered, and a late reply came too: another node may still execute it), or SeveralExecuted
    (answered by several nodes: each executed it). A receiver replies and drops the query at once: one that keeps
    the query after its reply makes Zenoh send a late "Timeout" reply, which looks like a second node."""
    answers = ask_once(session, selector, timeout, **get_options)
    errors = [r.err.payload.to_string() for r in answers if r.err is not None and not timed_out(r)]
    if errors:
        raise RuntimeError(errors[0])
    ok = [r.ok for r in answers if r.ok is not None]
    if not ok:
        raise NoAnswer(f"{selector}: sent once, no answer within {timeout:g} s: it may still be executed; not sent "
                       "again")
    if any(timed_out(r) for r in answers):
        raise PartlyAnswered(f"{selector}: answered by {len(ok)} node(s), and a reply came late after "
                             f"{timeout:g} s ({HOLDING}): a second node may still execute it; not sent again")
    if len(ok) > 1:
        raise SeveralExecuted(f"{selector}: sent, and {len(ok)} nodes answered: each of them executed it; not sent "
                              "again (one key, one node)")
    return ok


def count(answers: list, selector: str) -> list:
    """The successful replies of a read that counts the nodes of a command's key before the command (best effort):
    RuntimeError, "not sent", unless exactly one node answered and none gave Zenoh's "Timeout" reply (a node that
    exists but did not answer in time is a second node, not none; or a single node that keeps the query after its
    reply). TimeoutError when none answered, in time or at all."""
    ok = [r.ok for r in answers if r.ok is not None]
    late = any(timed_out(r) for r in answers)
    if not ok:
        raise TimeoutError(f"not sent: no node answered {selector}" + (" in time" if late else ""))
    if late:
        raise RuntimeError(f"not sent: {len(ok)} node(s) answered {selector}, and a reply came late ({HOLDING})")
    if len(ok) > 1:
        raise RuntimeError(f"not sent: {len(ok)} nodes answer {selector}: one key, one node")
    return ok


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
    ``isinstance(s, zenoh.Session)`` holds (zenoh.Session cannot be subclassed). ``across_pcs``: the endpoints of
    ``across_pcs`` for its options, or none."""

    __slots__ = ("_session", "across_pcs")

    def __init__(self, session: zenoh.Session, across_pcs: Sequence[str] = ()):
        object.__setattr__(self, "_session", session)
        object.__setattr__(self, "across_pcs", list(across_pcs))

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
