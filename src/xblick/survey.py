"""What runs on a Zenoh network: the topics and their traffic, the sessions (one per program) and which of them
publish, subscribe and answer which keys, and the routers between them. For ``xblick list --traffic`` and the
terminal interface (tui.py).

    meter = TrafficMeter(session, "test/**")   # subscribes: publishers produce what it takes (every representation)
    time.sleep(2.0)
    for key, t in meter.snapshot().items():      # cumulative counts, bytes, encoding, last stamp
        ...
    network = query_network(session)             # the routers' admin space: costs the publishers nothing
    for node in network.nodes.values():          # a session: its label, address, keys
        ...
    network.connections()                        # (publisher, key, subscriber, its key expression)
    network.uses(key)                            # who takes a stream's representation, and how (Use)

The admin space is a router's: ``zenohd`` serves it (read only) by default, a session opened in peer or client mode
does not. Without a router nothing of it answers (``Network.routers`` is empty), and the sessions are unknown.
"""

from __future__ import annotations

import collections
import dataclasses
import json
import threading
import time
from collections.abc import Iterable

import numpy as np
import zenoh

from . import _core, topics
from .msg import Message
from .recording import message_types
from .camera import parse_attachment
from .session import replies
from .stamps import age_at_send, header_stamp

ADMIN_KINDS = ("publisher", "subscriber", "queryable", "token")
# a stream's representations by their keys' ends (topics.py); the longer first: image_raw/compressed is not image_raw
REPRESENTATIONS = sorted([(topics.SHM, "shm"), (topics.CUDA, "cuda"), (topics.COMPRESSED, "compressed"),
                          (topics.IMAGE_RAW, "raw"), (topics.CLOUD_SHM, "shm"), (topics.CLOUD_CUDA, "cuda"),
                          (topics.CLOUD_COMPRESSED, "compressed"),
                          (topics.CLOUD, "raw")], key=lambda r: -len(r[0]))
JSON = "application/json"


@dataclasses.dataclass
class Traffic:
    """What came on one key since the meter started: ``count`` samples of ``bytes`` bytes in all; of the last one its
    encoding, its stamp (header, Clock, a TFMessage's first transform, a JSON sample's ``stamp_ns``; None without one)
    and when it came (``time.time_ns``)."""

    count: int = 0
    bytes: int = 0
    encoding: str = ""
    stamp_ns: int | None = None
    received_ns: int = 0


class TrafficMeter:
    """Counts every sample on a key expression, per key, and their ages (stamps.py): at send (the attachment's
    publish_ns minus the stamp, the publisher's clock alone) and at receive (this host's receive time minus the stamp:
    with the way here, and two hosts' clocks). ``take_ages`` hands out the ages since it was called last; ``recent``
    the last samples' of a key. ``keep`` (keys) also keeps their last sample (``last``), for a look at its content.
    Subscribing makes publishers produce what nobody else takes (raw images, every transport): meter what you look
    at."""

    RECENT = 20  # samples per key in ``recent``
    WINDOW_MAX = 10_000  # ages per key between two take_ages (a fast topic and a slow reader)

    def __init__(self, session: zenoh.Session, key_expr: str = "**"):
        self.key_expr = key_expr
        self.keep: set[str] = set()
        self._traffic: dict[str, Traffic] = {}
        self._last: dict[str, tuple[str, bytes]] = {}
        self._ages: dict[str, tuple[list, list]] = {}
        self._recent: dict[str, collections.deque] = {}
        self._lock = threading.Lock()
        self._subscriber = session.declare_subscriber(key_expr, self._on_sample)

    def _on_sample(self, sample: zenoh.Sample):
        received = time.time_ns()
        key, encoding, payload = str(sample.key_expr), str(sample.encoding), sample.payload.to_bytes()
        stamp = _stamp(key, encoding, payload)
        try:
            meta = parse_attachment(sample.attachment.to_bytes() if sample.attachment is not None else None)
        except ValueError:  # another program's attachment: no age at send
            meta = {}
        send = age_at_send(stamp, meta.get("publish_ns"))
        receive = received - stamp if stamp else None
        with self._lock:
            t = self._traffic.setdefault(key, Traffic())
            t.count += 1
            t.bytes += len(payload)
            t.encoding, t.stamp_ns, t.received_ns = encoding, stamp, received
            sends, receives = self._ages.setdefault(key, ([], []))
            if send is not None and len(sends) < self.WINDOW_MAX:
                sends.append(send)
            if receive is not None and len(receives) < self.WINDOW_MAX:
                receives.append(receive)
            self._recent.setdefault(key, collections.deque(maxlen=self.RECENT)).append((stamp, send, receive))
            if key in self.keep:
                self._last[key] = (encoding, payload)

    def take_ages(self) -> dict[str, tuple[list[int], list[int]]]:
        """Key -> (ages at send, ages at receive) in ns of the samples since the last call."""
        with self._lock:
            ages, self._ages = self._ages, {}
            return ages

    def recent(self, key: str) -> list[tuple[int | None, int | None, int | None]]:
        """(stamp, age at send, age at receive) in ns of a key's last samples, the oldest first."""
        with self._lock:
            return list(self._recent.get(key, ()))

    def snapshot(self) -> dict[str, Traffic]:
        """A copy of the counts so far, by key."""
        with self._lock:
            return {k: dataclasses.replace(t) for k, t in self._traffic.items()}

    def last(self, key: str) -> tuple[str, bytes] | None:
        """(encoding, payload) of the last sample on a key of ``keep`` (None before one came)."""
        with self._lock:
            return self._last.get(key)

    def close(self):
        self._subscriber.undeclare()


def _stamp(key: str, encoding: str, payload: bytes) -> int | None:
    try:
        return header_stamp(key, encoding, payload)
    except ValueError:  # a message without a stamp (std_msgs-like, JSON without stamp_ns): no stamp to show
        return None


def median_p95(values: list[int]) -> tuple[float, float] | None:
    """(median, 95th percentile) of ages in ns, as ms; None without any."""
    if not values:
        return None
    ms = np.asarray(values, dtype=np.float64) / 1e6
    return float(np.median(ms)), float(np.percentile(ms, 95))


def rates(before: dict[str, Traffic], after: dict[str, Traffic], seconds: float) -> dict[str, tuple[float, float]]:
    """(samples per second, mean bytes per sample) of each key between two snapshots ``seconds`` apart."""
    out = {}
    for key, t in after.items():
        b = before.get(key, Traffic())
        count = t.count - b.count
        out[key] = (count / seconds, (t.bytes - b.bytes) / count if count else 0.0)
    return out


# ------------------------------------------------------------------------------------------------- message content


def message_types_by_encoding() -> dict[str, type[Message]]:
    return {cls.ENCODING: cls for cls in message_types().values()}


def content(encoding: str, payload: bytes, max_items: int = 16):
    """A sample's content as plain data to look at: a CDR message of a known type as its fields (arrays and long byte
    strings as their shape, long lists cut), JSON as it is, anything else as its size. A CDR sample of an unknown
    type or one that does not decode raises ValueError."""
    if encoding == JSON or encoding.startswith(JSON + ";"):
        return json.loads(payload)
    if encoding.startswith("application/cdr;"):
        cls = message_types_by_encoding().get(encoding)
        if cls is None:
            raise ValueError(f"{encoding}: a type this Durchblick version ({_core.__version__}) does not know")
        return _plain(cls.deserialize(payload), max_items)
    return f"{len(payload)} bytes, {encoding or 'no encoding'}"


def _plain(value, max_items: int):
    if dataclasses.is_dataclass(value):
        return {f.name: _plain(getattr(value, f.name), max_items) for f in dataclasses.fields(value)}
    if isinstance(value, np.ndarray):
        if value.size <= max_items:
            return value.tolist()
        return f"array {value.shape} {value.dtype}"
    if isinstance(value, (bytes, bytearray, memoryview)):
        return f"{len(value)} bytes" if len(value) > max_items else bytes(value).hex()
    if isinstance(value, (list, tuple)):
        items = [_plain(v, max_items) for v in value[:max_items]]
        return items + [f"... {len(value)} in all"] if len(value) > max_items else items
    return value


# ---------------------------------------------------------------------------------------------- the admin space


@dataclasses.dataclass
class Node:
    """A Zenoh session (a program, usually) as the routers know it: what it is (``peer``, ``client``, ``router``), the
    addresses its links come from, the routers it is attached to, and the key expressions it declared."""

    zid: str
    whatami: str = ""
    addresses: set[str] = dataclasses.field(default_factory=set)
    routers: set[str] = dataclasses.field(default_factory=set)
    publishes: set[str] = dataclasses.field(default_factory=set)
    subscribes: set[str] = dataclasses.field(default_factory=set)
    queryables: set[str] = dataclasses.field(default_factory=set)
    tokens: set[str] = dataclasses.field(default_factory=set)

    def declared(self, kind: str) -> set[str]:
        return {"publisher": self.publishes, "subscriber": self.subscribes, "queryable": self.queryables,
                "token": self.tokens}[kind]

    @property
    def label(self) -> str:
        """A name from what it declared: the common prefix of its publishers' and queryables' keys (its
        ``<camera>/diagnostics`` only if it has nothing else, since nodes share it); of a session that only
        subscribes, ``subscribes <prefix>``; a router is ``router``."""
        if self.whatami == "router":
            return "router"
        own = self.publishes | self.queryables | self.tokens
        named = {k for k in own if not k.endswith("/" + topics.DIAGNOSTICS)} or own
        if named:
            return common_prefix(named) or f"{len(named)} keys"
        if self.subscribes:
            return "subscribes " + (common_prefix(self.subscribes) or f"{len(self.subscribes)} keys")
        return "nothing declared"


@dataclasses.dataclass
class Router:
    zid: str
    version: str = ""
    locators: list[str] = dataclasses.field(default_factory=list)
    sessions: int = 0


@dataclasses.dataclass
class Network:
    """What the routers that answered know: themselves and the sessions with their declarations, by zid."""

    routers: dict[str, Router] = dataclasses.field(default_factory=dict)
    nodes: dict[str, Node] = dataclasses.field(default_factory=dict)

    def node(self, zid: str) -> Node:
        return self.nodes.setdefault(zid, Node(zid))

    def publishers(self) -> dict[str, set[str]]:
        """Key -> the zids of its publishers."""
        out: dict[str, set[str]] = {}
        for node in self.nodes.values():
            for key in node.publishes:
                out.setdefault(key, set()).add(node.zid)
        return out

    def uses(self, key: str, leave_out: Iterable[str] = ()) -> list[Use]:
        """Who takes a representation key of a stream (``<base>/image_raw/shm``, ``.../compressed``, ``<base>/cloud``,
        ...; ``representation``) and how: every subscription of another session that intersects it, but those in
        ``leave_out``. Empty for a key that is no representation."""
        what = representation(key)
        if what is None:
            return []
        leave_out = set(leave_out)
        publishers = [self.nodes[z] for z in sorted(self.publishers().get(key, ())) if z in self.nodes]
        expr = zenoh.KeyExpr(key)
        out = []
        for node in sorted(self.nodes.values(), key=lambda n: n.zid):
            if node.zid in leave_out or node.zid in {p.zid for p in publishers}:
                continue
            for s in sorted(node.subscribes):
                if zenoh.KeyExpr(s).intersects(expr):
                    out.append(_use(node, s, what, publishers))
        return out

    def connections(self, leave_out: Iterable[str] = ()) -> list[tuple[str, str, str, str]]:
        """(publisher zid, key, subscriber zid, the subscriber's key expression) of every publisher key that a
        subscription of another session intersects; sessions in ``leave_out`` (e.g. one's own) are left out."""
        leave_out = set(leave_out)
        subscriptions = [(n.zid, s, zenoh.KeyExpr(s)) for n in self.nodes.values() if n.zid not in leave_out
                         for s in n.subscribes]
        out = []
        for key, zids in sorted(self.publishers().items()):
            expr = zenoh.KeyExpr(key)
            for zid in sorted(zids - leave_out):
                out.extend((zid, key, sub, s) for sub, s, sub_expr in subscriptions
                           if sub != zid and expr.intersects(sub_expr))
        return out


@dataclasses.dataclass
class Use:
    """A session's subscription that takes a stream's representation (``raw``, ``compressed``, ``shm``, ``cuda``):
    ``transport`` ``shm`` or ``cuda`` (the ring on the publisher's host), else the Zenoh link's protocol (``tcp``,
    ``udp``, ``unix``... of the session's links to its routers) or ``unknown``; ``detail`` says what is known of the way
    (or why nothing is)."""

    consumer: str
    expr: str
    representation: str
    transport: str
    detail: str

    @property
    def summary(self) -> str:
        """``shm``, ``cuda``, ``tcp-compressed``, ``unknown-raw``..."""
        return self.transport if self.transport in ("shm", "cuda") else f"{self.transport}-{self.representation}"


def representation(key: str) -> str | None:
    """``raw``, ``compressed``, ``shm`` or ``cuda`` of a stream's key (image or point cloud), None for another key."""
    place = topics.ring_place(key)  # a ring's key ends with its place (host, GPU)
    key = place[0] if place is not None else key
    return next((name for end, name in REPRESENTATIONS if key.endswith("/" + end)), None)


def _locator(address: str) -> tuple[str, str]:
    """(protocol, host) of a locator such as ``tcp/192.0.2.4:41464`` or ``tcp/[::1]:7447``."""
    protocol, _, rest = address.partition("/")
    return protocol, rest.rsplit(":", 1)[0]


def _use(node: Node, expr: str, what: str, publishers: list[Node]) -> Use:
    if what in ("shm", "cuda"):
        return Use(node.zid, expr, what, what, f"the {what} ring on the publisher's host")
    if not node.addresses:
        return Use(node.zid, expr, what, "unknown", "the routers that answered know no link of this session")
    protocols = sorted({_locator(a)[0] for a in node.addresses})
    hosts = {_locator(a)[1] for a in node.addresses}
    if node.whatami == "client":
        way = "a client: through its router " + ", ".join(_short(r) for r in sorted(node.routers))
    else:
        way = "a peer: on a direct link to the publisher or through the router (links between peers are not in the " \
              "admin space)"
    where = []
    for p in publishers:
        if p.routers & node.routers:
            same = bool(hosts & {_locator(a)[1] for a in p.addresses})
            where.append(f"on the host of {p.label}" if same else
                         f"at another address than {p.label} (another host, or the same by another interface)")
        else:
            where.append(f"through other routers than {p.label}'s")
    detail = f"{way}; link {', '.join(sorted(node.addresses))}" + (f"; {'; '.join(where)}" if where else "")
    return Use(node.zid, expr, what, "+".join(protocols), detail)


def _short(zid: str) -> str:
    return zid[:8]


def common_prefix(keys: Iterable[str]) -> str:
    """The key tokens all keys start with, e.g. ``test/camera`` of its streams' keys."""
    split = [k.split("/") for k in keys]
    if not split:
        return ""
    prefix = []
    for tokens in zip(*split):
        if len(set(tokens)) != 1 or "*" in tokens[0]:
            break
        prefix.append(tokens[0])
    return "/".join(prefix)


def query_network(session: zenoh.Session, timeout: float = 1.0) -> Network:
    """The routers' admin space (``@/<router>/router``, ``@/<router>/router/<kind>/<key>``) as a Network. A
    declaration a router knows only from another router (a session attached there) is that router's: its own admin
    space has the session, when it answers too."""
    network = Network()
    for key, payload in _admin(session, "@/*/router", timeout):
        record = json.loads(payload)
        router = Router(record["zid"], record.get("version", ""), record.get("locators") or [],
                        len(record.get("sessions") or []))
        network.routers[router.zid] = router
        network.node(router.zid).whatami = "router"
        for s in record.get("sessions") or []:
            node = network.node(s["peer"])
            node.whatami = s.get("whatami", "")
            node.routers.add(router.zid)
            # the router's link's other end: the session's address as the router sees it
            node.addresses.update(link["dst"] for link in s.get("links") or [] if link.get("dst"))
    for kind in ADMIN_KINDS:
        for key, payload in _admin(session, f"@/*/router/{kind}/**", timeout):
            router_zid, declared = _admin_key(key, kind)
            record = json.loads(payload)
            zids = set(record.get("peers") or []) | set(record.get("clients") or [])
            zids |= set(record.get("routers") or []) - {router_zid}
            for zid in zids:
                network.node(zid).declared(kind).add(declared)
    return network


def _admin(session: zenoh.Session, selector: str, timeout: float) -> list[tuple[str, bytes]]:
    return [(str(r.ok.key_expr), r.ok.payload.to_bytes()) for r in replies(session, selector, timeout)
            if r.ok is not None]


def _admin_key(key: str, kind: str) -> tuple[str, str]:
    """(router zid, declared key expression) of ``@/<zid>/router/<kind>/<key expression>``."""
    at, zid, router, got, declared = key.split("/", 4)
    if at != "@" or router != "router" or got != kind:
        raise ValueError(f"{key}: not an admin space key of a router's {kind}s")
    return zid, declared


def liveliness_tokens(session: zenoh.Session, timeout: float = 1.0) -> list[str]:
    """The keys of every liveliness token alive (Zenoh's liveliness; sorted)."""
    return sorted({str(r.ok.key_expr) for r in replies(session, "**", timeout, liveliness=True) if r.ok is not None})
