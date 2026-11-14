"""The terminal interface: ``xblick`` without a command. A Textual app (textual.textualize.io) on one
Zenoh session that shows what runs, for any node: nothing in it knows of a particular node.

* **Connect** (``c``): this PC's router, endpoints (``--connect``/``--listen``, e.g. ``tcp/robot-pc:7447``) or a Zenoh
  configuration file (``--config``, e.g. ``robot-pc.json5``); again at any time, to the same or another.
* **Topics**: every key a session publishes (from the routers' admin space), and with ``m`` the traffic of all of
  them (rate, size, type, last stamp: subscribing to everything, as ``xblick list --traffic``, which makes the
  publishers produce every representation). Enter on a topic subscribes to it alone and shows its last message.
* **Nodes**: the sessions (one per program) with what they declared, named after their keys; the routers; the
  reports on ``<camera>/diagnostics`` and the record and play nodes' status; liveliness tokens.
* **Graph**: the sessions as boxes and the data between them as arrows (tui_graph.py); a session selected (arrow
  keys, a click) or the mouse over an edge names the keys, how they are taken and what the session reports.
* **Rerun**: streams, results and options of ``xblick rerun``, as presets with a name; launched as a child
  process (stopped with SIGTERM, as its Ctrl-C) on the interface's connection.

The routers' admin space (``survey.query_network``) has the sessions and their declarations; without a router (only
peers) the interface says so and shows what the streams' info records and the traffic tell.
"""

from __future__ import annotations

import collections
import contextlib
import dataclasses
import functools
import importlib.util
import io
import json
import os
import shlex
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import ClassVar

import zenoh
from rich.console import Group
from rich.pretty import Pretty
from rich.text import Text
from textual.app import App, ComposeResult
from textual.containers import Horizontal, Vertical, VerticalScroll
from textual.screen import ModalScreen
from textual.widgets import (Button, DataTable, Footer, Header, Input, Label, RadioButton, RadioSet, RichLog, Select,
                             SelectionList, Static, TabbedContent, TabPane)
from textual.widgets.selection_list import Selection
from textual.worker import get_current_worker

from . import diagnostics, parameters, survey, topics
from . import session as zsession
from .camera import find_streams
from .paths import expand_path
from .tui_graph import GraphEdge, GraphNode, GraphView
from .tui_parameters import ParametersPane

PRESETS_VERSION = 1
UNREACHABLE_S = 5.0  # a peer without a link to it for so long is not reachable from here
MIN_SIZE = (80, 24)  # columns, rows: below it the interface says so instead of a broken layout
WIDE = 120  # from this width the Rerun tab's lists stand side by side
GRAPH_HELP = Text("arrow keys or a click: a session and its edges (out green, in cyan); the mouse over an edge or a "
                  "box: its keys and how they are taken; ▶ data flows into a box, ◀ out of it (a cycle)", style="dim")
RERUN_KINDS = {"bases": "image", "clouds": "points", "detections": "detections", "segmentations": "labels",
               "trajectories": "poses"}


def _short(zid: str) -> str:
    return zid[:8]


def short_names(labels: dict[str, str]) -> dict[str, str]:
    """Sessions' names without what another session's name says already: below the longest other name that is a
    prefix of it (``sam3`` below ``test/camera``, ``tapnext/tracks``), else whole (``robot``,
    ``xblick/recorder/status``); ``subscribes <prefix>`` likewise after its word. Two names that would be the same
    stay whole."""
    def split(label: str) -> tuple[str, list[str]]:
        word, _, rest = label.partition(" ") if label.startswith("subscribes ") else ("", "", label)
        return word, rest.split("/")

    tokens = {tuple(split(label)[1]) for label in labels.values() if not label.startswith("subscribes ")}
    names = {}
    for zid, label in labels.items():
        word, own = split(label)
        cut = max((len(t) for t in tokens if len(t) < len(own) and tuple(own[:len(t)]) == t), default=0)
        names[zid] = " ".join(filter(None, (word, "/".join(own[cut:]))))
    count: dict[str, int] = {}
    for name in names.values():
        count[name] = count.get(name, 0) + 1
    return {zid: name if count[name] == 1 else labels[zid] for zid, name in names.items()}


def _clock(ns: int | None) -> str:
    if ns is None:
        return ""
    return time.strftime("%H:%M:%S", time.localtime(ns / 1e9)) + f".{ns // 1_000_000 % 1000:03d}"


# ------------------------------------------------------------------------------------- connection and presets


@dataclasses.dataclass
class Connection:
    """A Zenoh connection as the commands take it: nothing (this PC's router), ``connect``/``listen`` endpoints in
    ``mode`` peer or client, or a configuration file (absolute, so that a preset works from anywhere). A peer that only
    connects is refused: the router does not route between peers that do not listen, so it would receive nothing."""

    connect: list[str] = dataclasses.field(default_factory=list)
    listen: list[str] = dataclasses.field(default_factory=list)
    config: str | None = None
    mode: str = "peer"

    def __post_init__(self):
        if self.config and (self.connect or self.listen or self.mode != "peer"):
            raise ValueError("give either a Zenoh config file or connect/listen endpoints and a mode, not both")
        if self.mode not in ("peer", "client"):
            raise ValueError(f"mode {self.mode!r}: peer or client")
        if self.mode == "client" and (self.listen or not self.connect):
            raise ValueError("client mode: an endpoint to connect to (the router), nothing to listen on")
        # what a person types (robot-pc, 192.0.2.10:7447) as Zenoh endpoints, shown so in the status line
        self.connect = [zsession.endpoint(e) for e in self.connect]
        self.listen = [zsession.endpoint(e) for e in self.listen]
        if self.mode == "peer" and self.connect and not self.listen:
            raise ValueError("a peer that only connects receives nothing (the router does not route between peers "
                             "that do not listen): listen too (tcp/127.0.0.1:0 on this PC) or use client mode")
        if self.config:
            path = expand_path(self.config).resolve()
            if not path.is_file():
                raise FileNotFoundError(f"Zenoh config file {self.config}: no such file")
            self.config = str(path)

    def arguments(self) -> list[str]:
        """The commands' ``--connect``, ``--listen``, ``--config`` for it."""
        if self.config:
            return ["--config", self.config]
        return ((["--zenoh-mode", "client"] if self.mode == "client" else [])
                + (["--connect", *self.connect] if self.connect else [])
                + (["--listen", *self.listen] if self.listen else []))

    def describe(self) -> str:
        if self.config:
            return f"config {self.config}"
        if not (self.connect or self.listen):
            return f"this PC's router ({zsession.ROUTER})"
        return " ".join(self.arguments())

    def open(self) -> zenoh.Session:
        return zsession.open_session(connect=self.connect, listen=self.listen, mode=self.mode, config_file=self.config)

    def to_dict(self) -> dict:
        return dataclasses.asdict(self)

    @classmethod
    def from_dict(cls, d: dict) -> Connection:
        return cls(list(d.get("connect") or []), list(d.get("listen") or []), d.get("config"), d.get("mode", "peer"))


@dataclasses.dataclass
class RerunSelection:
    """What ``xblick rerun`` is to show and how: its streams (``bases``), point clouds, detections, label images,
    trajectories, options, the viewer (``spawn``, ``serve`` with a port, ``save`` with a file, ``connect-viewer`` with a URL), further
    arguments, and the Zenoh connection. No streams and no clouds: rerun takes every stream that answers."""

    bases: list[str] = dataclasses.field(default_factory=list)
    clouds: list[str] = dataclasses.field(default_factory=list)
    detections: list[str] = dataclasses.field(default_factory=list)
    segmentations: list[str] = dataclasses.field(default_factory=list)
    trajectories: list[str] = dataclasses.field(default_factory=list)
    prefix: str = ""
    stale_after: float | None = None  # None: rerun's default
    cloud_rate: float | None = None
    viewer: str = "spawn"
    viewer_value: str = ""
    extra: str = ""  # further arguments, as on a command line
    connection: Connection = dataclasses.field(default_factory=Connection)

    VIEWERS: ClassVar[tuple[str, ...]] = ("spawn", "serve", "save", "connect-viewer")

    def arguments(self) -> list[str]:
        """``xblick rerun``'s arguments (after ``rerun``)."""
        if self.viewer not in self.VIEWERS:
            raise ValueError(f"viewer {self.viewer!r}: one of {', '.join(self.VIEWERS)}")
        if self.viewer in ("save", "connect-viewer") and not self.viewer_value:
            raise ValueError(f"--{self.viewer} needs a {'file' if self.viewer == 'save' else 'URL'}")
        args = [f"--{self.viewer}"] + ([self.viewer_value] if self.viewer_value else [])
        args += list(self.bases)
        for option, keys in (("--detections", self.detections), ("--segmentation", self.segmentations),
                             ("--points", self.clouds), ("--trajectory", self.trajectories)):
            for key in keys:
                args += [option, key]
        if self.prefix:
            args += ["--prefix", self.prefix]
        if self.stale_after is not None:
            args += ["--stale-after", f"{self.stale_after:g}"]
        if self.cloud_rate is not None:
            args += ["--cloud-rate", f"{self.cloud_rate:g}"]
        args += shlex.split(self.extra)
        return args + self.connection.arguments()  # last: --connect takes several endpoints

    def check(self) -> list[str]:
        """The arguments, checked by the command's own parser; a wrong one is a ValueError with argparse's reason."""
        from . import cli

        args = self.arguments()
        reason = io.StringIO()
        with contextlib.redirect_stderr(reason), contextlib.redirect_stdout(io.StringIO()):
            try:
                cli.parse(["rerun", *args])
            except SystemExit:
                raise ValueError(reason.getvalue().strip().splitlines()[-1]) from None
        return args

    def command(self) -> list[str]:
        """The child process's command line (this interpreter's Durchblick)."""
        return [sys.executable, "-m", "xblick.cli", "rerun", *self.check()]

    def to_dict(self) -> dict:
        d = dataclasses.asdict(self)
        d["connection"] = self.connection.to_dict()
        return d

    @classmethod
    def from_dict(cls, d: dict) -> RerunSelection:
        fields = {f.name for f in dataclasses.fields(cls)}
        unknown = set(d) - fields
        if unknown:
            raise ValueError(f"a preset with fields {sorted(unknown)} this version does not know")
        d = dict(d)
        d["connection"] = Connection.from_dict(d.get("connection") or {})
        return cls(**d)


def presets_path() -> Path:
    """``$XDG_CONFIG_HOME/xblick/rerun_presets.json`` (``~/.config`` without it)."""
    base = os.environ.get("XDG_CONFIG_HOME") or os.path.expanduser("~/.config")
    return Path(base) / "xblick" / "rerun_presets.json"


class Presets:
    """Named RerunSelections in a JSON file (``{"v": 1, "presets": {name: selection}}``). A file of another version
    or form is refused, not overwritten."""

    def __init__(self, path: Path | str | None = None):
        self.path = Path(path) if path is not None else presets_path()

    def load(self) -> dict[str, RerunSelection]:
        if not self.path.exists():
            return {}
        try:
            data = json.loads(self.path.read_text())
        except ValueError as error:
            raise ValueError(f"{self.path}: no presets file ({error}); move it away") from None
        if not isinstance(data, dict) or data.get("v") != PRESETS_VERSION or not isinstance(data.get("presets"), dict):
            raise ValueError(f"{self.path}: not a presets file of version {PRESETS_VERSION}; move it away")
        out = {}
        for name, d in data["presets"].items():
            try:
                out[name] = RerunSelection.from_dict(d)
            except (TypeError, ValueError, OSError) as error:
                raise ValueError(f"{self.path}: preset {name!r}: {error}") from None
        return out

    def _write(self, presets: dict[str, RerunSelection]):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        data = {"v": PRESETS_VERSION, "presets": {n: s.to_dict() for n, s in sorted(presets.items())}}
        part = self.path.with_suffix(".part")
        part.write_text(json.dumps(data, indent=1) + "\n")
        part.replace(self.path)  # whole or not at all

    def save(self, name: str, selection: RerunSelection):
        if not name.strip():
            raise ValueError("a preset needs a name")
        selection.arguments()  # a preset that cannot run is not saved
        presets = self.load()
        presets[name.strip()] = selection
        self._write(presets)

    def delete(self, name: str):
        presets = self.load()
        if name not in presets:
            raise KeyError(f"no preset {name!r} in {self.path}")
        del presets[name]
        self._write(presets)


# ------------------------------------------------------------------------------------------------- the screens


class ConnectScreen(ModalScreen[Connection | None]):
    """The connection to open: this PC's router, endpoints, or a configuration file. It opens on the endpoints with
    the cursor in the address (the interface starts on this PC's router, and the dialog is opened to change that), or
    on the config file when that is the current connection. Typing into a field chooses its way; an address alone
    (``robot-pc``) is ``tcp/robot-pc:7447`` in client mode."""

    FIELDS: ClassVar = {"connect-endpoints": 1, "listen": 1, "config": 2}  # the input -> the way it belongs to

    BINDINGS: ClassVar = [("escape", "cancel", "Cancel")]

    def __init__(self, current: Connection, hint: str = ""):
        super().__init__()
        self.current = current
        self.hint = hint  # why the dialog opened by itself

    def compose(self) -> ComposeResult:
        c = self.current
        mode = 2 if c.config else 1
        with VerticalScroll(id="connect-dialog", can_focus=False):  # the keys go to the choice first
            yield Label("Zenoh connection", classes="title")
            with RadioSet(id="connect-mode", compact=True):
                for i, text in enumerate((f"this PC's router ({zsession.ROUTER})", "endpoints", "config file")):
                    yield RadioButton(text, value=i == mode, compact=True)
            yield Label("endpoints: connect to (several: spaces; host alone: tcp, port 7447)")
            yield Input(" ".join(c.connect), placeholder="e.g. robot-pc or tcp/192.0.2.10:7447",
                        id="connect-endpoints", compact=True)
            yield Label("endpoints: mode")
            yield Select([("client: everything through the router (a viewer, another PC)", "client"),
                          ("peer: listening too (on this PC: tcp/127.0.0.1:0)", "peer")],
                         value=c.mode if (c.connect or c.listen) else "client", allow_blank=False, id="zenoh-mode",
                         compact=True)
            yield Label("endpoints, peer: listen on")
            yield Input(" ".join(c.listen), placeholder="e.g. tcp/127.0.0.1:0", id="listen", compact=True)
            yield Label("config file")
            yield Input(c.config or "", placeholder="a Zenoh JSON5 file, e.g. robot-pc.json5", id="config",
                        compact=True)
            yield Static(Text(self.hint, style="yellow"), id="connect-error")
            with Horizontal(id="connect-buttons"):
                yield Button("Connect", variant="primary", id="connect-ok", compact=True)
                yield Button("Cancel", id="connect-cancel", compact=True)

    def on_mount(self):
        self.query_one("#config" if self.current.config else "#connect-endpoints", Input).focus()

    def on_input_changed(self, event: Input.Changed):
        way = self.FIELDS.get(event.input.id)
        radios = self.query_one("#connect-mode", RadioSet)
        if way is not None and event.value.strip() and radios.pressed_index != way:
            list(radios.query(RadioButton))[way].value = True

    def on_button_pressed(self, event: Button.Pressed):
        if event.button.id == "connect-cancel":
            self.dismiss(None)
        elif event.button.id == "connect-ok":
            self._submit()

    def on_input_submitted(self, event: Input.Submitted):
        self._submit()

    def _submit(self):
        mode = self.query_one("#connect-mode", RadioSet).pressed_index
        connect = self.query_one("#connect-endpoints", Input).value.split()
        listen = self.query_one("#listen", Input).value.split()
        zenoh_mode = self.query_one("#zenoh-mode", Select).value
        config = self.query_one("#config", Input).value.strip()
        try:
            if mode == 0:
                connection = Connection()
            elif mode == 1:
                if not (connect or listen):
                    raise ValueError("endpoints: give at least one to connect to (or to listen on)")
                connection = Connection(connect, listen, mode=zenoh_mode)
            else:
                if not config:
                    raise ValueError("config file: give its path")
                connection = Connection(config=config)
        except (ValueError, OSError) as error:
            self.query_one("#connect-error", Static).update(Text(str(error), style="bold red"))
            return
        self.dismiss(connection)

    def action_cancel(self):
        self.dismiss(None)


@dataclasses.dataclass
class TopicRow:
    """A row of the Topics tab: a key (``key``) or a prefix of keys (``key`` None) with the keys below it
    (``below``). ``path`` is the row's id (a prefix ends with ``/``); ``show_pub``/``show_sub``: the publishers or
    subscribers stand on this row (not on a prefix above it, which has the same for every key below)."""

    path: str
    label: str
    key: str | None
    below: list[str]
    show_pub: bool = True
    show_sub: bool = True

    def common(self, cells: dict[str, tuple[frozenset, frozenset]]) -> tuple[frozenset, frozenset]:
        pubs = {cells[k][0] for k in self.below}
        subs = {cells[k][1] for k in self.below}
        return (next(iter(pubs)) if len(pubs) == 1 else frozenset(),
                next(iter(subs)) if len(subs) == 1 else frozenset())


def topic_tree(keys, collapsed: set[str] = frozenset(), cells: dict | None = None) -> list[TopicRow]:
    """The keys as a directory tree, depth first: a prefix with one child and no key of its own joins it
    (``test/camera/``), guides (``├─``, ``└─``) show the depth, a prefix in ``collapsed`` hides what is below it
    (``▸``, with the number of keys). A key that is also a prefix (``.../image_raw`` and ``.../image_raw/shm``) is one
    row. A ring's key ends with its place, the host's id and the GPU's UUID (topics.py): the tree shows the first 8
    characters of each. With ``cells`` (key -> publishers, subscribers), a key's or prefix's sessions stand only on the highest row
    below which every key has the same."""
    root: dict = {}
    for key in keys:
        node = root
        place = topics.ring_place(key)  # a ring's key ends with its host's id and its GPU's: shown by 8 characters
        tokens = key.split("/") if place is None else place[0].split("/") + [c[:8] for c in place[1].split("/")]
        for token in tokens:
            node = node.setdefault(token, {})
        node["\0"] = key  # a key ends here
    rows: list[TopicRow] = []

    def leaves(node) -> list[str]:
        out = [node["\0"]] if "\0" in node else []
        for token, child in node.items():
            if token != "\0":
                out += leaves(child)
        return out

    def walk(node, prefix: str, guide: str, last: bool, top: bool, shown: tuple[bool, bool]):
        names = [prefix]
        # join a chain of prefixes with one child and no key of their own
        while "\0" not in node and len(node) == 1:
            token, node = next(iter(node.items()))
            names.append(token)
        path = "/".join(n for n in names if n)
        children = sorted(t for t in node if t != "\0")
        key = node.get("\0")
        is_prefix = bool(children)
        row_path = path + "/" if is_prefix else path
        folded = is_prefix and row_path in collapsed
        name = path[len(guide_path[0]):].lstrip("/") if not top else path
        mark = ("▸ " if folded else "▾ ") if is_prefix else ""
        branch = "" if top else ("└─ " if last else "├─ ")
        below = leaves(node)
        label = guide + branch + mark + name + ("/" if is_prefix else "") + (f"  ({len(below)} keys)" if folded else "")
        row = TopicRow(row_path, label, key, below)
        if cells is not None:
            pubs = {cells[k][0] for k in below}
            subs = {cells[k][1] for k in below}
            row.show_pub = not shown[0] and (len(pubs) == 1 or not is_prefix)
            row.show_sub = not shown[1] and (len(subs) == 1 or not is_prefix)
            shown = (shown[0] or (row.show_pub and is_prefix), shown[1] or (row.show_sub and is_prefix))
        rows.append(row)
        if folded:
            return
        saved = guide_path[0]
        guide_path[0] = path
        inner = guide + ("" if top else ("   " if last else "│  "))
        for n, token in enumerate(children):
            walk(node[token], path + "/" + token if path else token, inner, n == len(children) - 1, False, shown)
        guide_path[0] = saved

    guide_path = [""]
    for token in sorted(t for t in root if t != "\0"):
        guide_path[0] = ""
        walk(root[token], token, "", True, True, (False, False))
    return rows


def pose_topics(network: survey.Network | None) -> list[str]:
    """The keys for ``xblick rerun --trajectory``: a camera's ``<camera>/tf`` and a node's ``<camera>/<node>/tf``.
    Not ``<robot>/tf``: its frames form a tree below the world, and the bridge takes poses in the world only."""
    if network is None:
        return []
    return sorted(k for k in network.publishers() if k.endswith("/" + topics.TF) and k.count("/") >= 2)


@dataclasses.dataclass
class Discovery:
    """One look at the network (in the discovery thread)."""

    network: survey.Network | None = None
    streams: list[tuple[str, dict]] = dataclasses.field(default_factory=list)
    tokens: list[str] = dataclasses.field(default_factory=list)
    statuses: dict[str, dict] = dataclasses.field(default_factory=dict)  # xblick/.../status -> its JSON
    error: str | None = None
    unreachable: list[str] = dataclasses.field(default_factory=list)  # peers this peer has no link to (zids)


def statuses(session: zenoh.Session, timeout: float = 0.5) -> dict[str, dict]:
    """What Durchblick's programs answer on their status keys below ``xblick/`` (JSON), by key: every record and
    play node's (``xblick/<name>/status``), every MPS daemon's (``xblick/mps/<name>/status``)."""
    out = {}
    for reply in zsession.replies(session, "xblick/**/status", timeout):
        if reply.ok is not None:
            out[str(reply.ok.key_expr)] = json.loads(reply.ok.payload.to_bytes())
    return out


class XblickApp(App):
    """The interface on one session at a time; ``connection`` is opened at the start (default: this PC's router)."""

    TITLE = "Durchblick"
    HORIZONTAL_BREAKPOINTS: ClassVar = [(0, "-too-narrow"), (MIN_SIZE[0], "-narrow"), (WIDE, "-wide")]
    VERTICAL_BREAKPOINTS: ClassVar = [(0, "-too-short"), (MIN_SIZE[1], "-tall")]
    CSS = """
    #status { height: auto; max-height: 3; padding: 0 1; background: $boost; }
    /* the tabs take what is left: taller (auto) they made the screen scroll, the status line under the header */
    #main { height: 1fr; }
    #tabs { height: 1fr; }
    #tabs > ContentSwitcher, #tabs TabPane { height: 1fr; }
    DataTable { height: 1fr; }
    .detail { height: 1fr; border-top: solid $primary; padding: 0 1; }
    #report-table { border-top: solid $primary; }
    #graph-scroll { height: 3fr; overflow-x: auto; }
    #tab-topics DataTable { height: 2fr; }
    #tab-topics .detail { height: 1fr; }
    #rerun-top { height: 2fr; }
    #rerun-streams { width: 1fr; height: 1fr; }
    #rerun-form { width: 1fr; height: 1fr; }
    /* narrower than WIDE: the Rerun tab's lists one above the other */
    Screen.-narrow #rerun-top { layout: vertical; }
    #rerun-command { height: auto; padding: 0 1; background: $boost; }
    #rerun-buttons, #preset-buttons { height: auto; }
    #rerun-log { height: 1fr; min-height: 3; border-top: solid $primary; }
    #too-small { display: none; height: 1fr; content-align: center middle; text-style: bold; }
    Screen.-too-narrow #main, Screen.-too-short #main { display: none; }
    Screen.-too-narrow #too-small, Screen.-too-short #too-small { display: block; }
    ConnectScreen { align: center middle; }
    #connect-dialog { width: 100%; max-width: 90; height: auto; max-height: 100%; padding: 0 1;
                      border: thick $primary; background: $surface; }
    #connect-buttons { height: auto; }
    /* the dialog opens with the cursor in a field: the choice's dot alone says the way, not Textual's blurred cursor
       on the first one */
    #connect-mode:blur > RadioButton.-selected > .toggle--label { background: transparent; }
    #connect-error { height: auto; }
    """
    BINDINGS: ClassVar = [("c", "connect", "Connect"), ("m", "measure", "Measure traffic"),
                          ("escape", "close_detail", "Close topic"), ("1", "tab('tab-topics')", "Topics"),
                          ("2", "tab('tab-nodes')", "Nodes"), ("3", "tab('tab-graph')", "Graph"),
                          ("4", "tab('tab-rerun')", "Rerun"), ("5", "tab('tab-params')", "Parameters"),
                          ("r", "refresh", "Look again"),
                          ("q", "quit", "Quit")]
    MAIN: ClassVar = {"tab-topics": "#topic-table", "tab-nodes": "#node-table", "tab-graph": "#graph",
                      "tab-rerun": "#rerun-streams",
                      "tab-params": "#param-nodes"}  # what a tab focuses: keys (arrows, Enter) go to it

    def __init__(self, connection: Connection | None = None, presets: Presets | None = None,
                 discover_every: float = 2.0):
        super().__init__()
        self.connection = connection or Connection()
        self.presets = presets or Presets()
        self.discover_every = discover_every
        self.session: zenoh.Session | None = None
        self.generation = 0  # of the session; results of an older one are dropped
        self.discovery = Discovery()
        self.meter: survey.TrafficMeter | None = None  # every topic (m)
        self.detail: survey.TrafficMeter | None = None  # the selected topic
        self.detail_key: str | None = None
        self._before: tuple[float, dict] | None = None  # the last snapshot, for rates
        self._rates: dict[str, tuple[float, float]] = {}
        self._ages: dict[str, tuple] = {}  # key -> ((median, p95) ms at send or None, the same at receive)
        # (camera, name) -> (received ns, status, the sender's zid or None before 0.3.29)
        self._reports: dict[tuple[str, str], tuple[int, object, str | None]] = {}
        self._reports_lock = threading.Lock()
        self._report_errors: list[str] = []
        self._diagnostics_subscriber = None
        self._selected_node: str | None = None
        self._collapsed: set[str] = set()
        self._topic_rows: dict[str, TopicRow] = {}  # the Topics tab's folded prefixes (ending with /)
        self._graph_hovered = None  # what the mouse is over in the graph
        self._auto_connect = self.connection == Connection()  # nothing found on this PC: the dialog, once
        self._discover_now = threading.Event()  # r: look again now, not at the next period
        self._discovered_at: float | None = None
        self._rerun: subprocess.Popen | None = None
        self.rerun_output: list[str] = []  # the child's lines, as in the log
        self._known_streams: set[str] = set()
        self.uses: dict[str, list[survey.Use]] = {}  # representation key -> who takes it, how (this interface left out)
        self._measure_after_open = False  # measuring when the connection changed: again on the next
        self.error: str | None = None

    # ------------------------------------------------------------------------------------------------ layout

    def compose(self) -> ComposeResult:
        yield Header()
        yield Static(f"terminal too small: it needs at least {MIN_SIZE[0]} x {MIN_SIZE[1]}", id="too-small")
        with Vertical(id="main"):
            yield Static(id="status")
            with TabbedContent(id="tabs"):
                with TabPane("Topics", id="tab-topics"):
                    yield DataTable(id="topic-table", cursor_type="row")
                    with VerticalScroll(classes="detail"):
                        yield Static("Enter on a topic: its publishers, subscribers and last message",
                                     id="topic-detail")
                with TabPane("Nodes", id="tab-nodes"):
                    yield DataTable(id="node-table", cursor_type="row")
                    yield DataTable(id="report-table", cursor_type="row")
                    with VerticalScroll(classes="detail"):
                        yield Static("Enter on a session: what it declared and reports", id="node-detail")
                with TabPane("Graph", id="tab-graph"):
                    with VerticalScroll(id="graph-scroll"):
                        yield GraphView(id="graph")
                    with VerticalScroll(classes="detail", id="graph-detail-scroll"):
                        yield Static(GRAPH_HELP, id="graph-detail")
                with TabPane("Rerun", id="tab-rerun"):
                    with Horizontal(id="rerun-top"):
                        yield SelectionList(id="rerun-streams")
                        with VerticalScroll(id="rerun-form"):
                            yield Static(id="rerun-found")
                            yield Label("prefix (without streams: only those below it)")
                            yield Input(id="rerun-prefix", placeholder="e.g. test", compact=True)
                            yield Label("--stale-after (s)")
                            yield Input(id="rerun-stale-after", placeholder="default", type="number", compact=True)
                            yield Label("--cloud-rate (clouds/s)")
                            yield Input(id="rerun-cloud-rate", placeholder="default", type="number", compact=True)
                            yield Label("viewer")
                            yield Select([(v, v) for v in RerunSelection.VIEWERS], value="spawn", allow_blank=False,
                                         id="rerun-viewer", compact=True)
                            yield Input(id="rerun-viewer-value", placeholder="serve: port; save: file; "
                                        "connect-viewer: URL", compact=True)
                            yield Label("further arguments")
                            yield Input(id="rerun-extra", placeholder="e.g. --report 2", compact=True)
                            yield Label("presets")
                            yield Select([], prompt="preset", id="preset-select", compact=True)
                            yield Input(id="preset-name", placeholder="name to save as", compact=True)
                            with Horizontal(id="preset-buttons"):
                                yield Button("Save", id="preset-save", compact=True)
                                yield Button("Load", id="preset-load", compact=True)
                                yield Button("Delete", id="preset-delete", variant="error", compact=True)
                    yield Static(id="rerun-command")
                    with Horizontal(id="rerun-buttons"):
                        yield Button("Launch xblick rerun", variant="primary", id="rerun-launch", compact=True)
                        yield Button("Stop", id="rerun-stop", disabled=True, compact=True)
                        yield Button("Look again (r)", id="rerun-refresh", compact=True)
                    yield RichLog(id="rerun-log", max_lines=2000)
                with TabPane("Parameters", id="tab-params"):
                    yield ParametersPane(id="params")
        yield Footer()

    def on_mount(self):
        topics_table = self.query_one("#topic-table", DataTable)
        # what changes first, after the key: on a narrow terminal the rest is a horizontal scroll away
        # ages: median / p95 over the last second; at send on the publisher's clock alone, at receive with the way
        # here and both clocks
        for label, key in (("topic", "key"), ("Hz", "hz"), ("published by", "by"), ("subscribed by", "subs"),
                           ("taken as", "transport"), ("kB", "kb"), ("age at send ms", "send"),
                           ("age at receive ms", "age"), ("type", "type"), ("stamp", "stamp")):
            topics_table.add_column(label, key=key)
        nodes = self.query_one("#node-table", DataTable)
        for label, key in (("session", "label"), ("kind", "kind"), ("zid", "zid"), ("address", "address"),
                           ("pub", "pub"), ("sub", "sub"), ("query", "query"), ("router", "router")):
            nodes.add_column(label, key=key)
        reports = self.query_one("#report-table", DataTable)
        for label, key in (("topic", "topic"), ("name", "name"), ("level", "level"), ("message", "message"),
                           ("age s", "age"), ("session", "session")):
            reports.add_column(label, key=key)
        self._load_presets()
        self._update_command()
        self.query_one("#topic-table", DataTable).focus()
        self.set_interval(1.0, self._tick)
        self._open(self.connection)

    def action_tab(self, tab: str):
        """Shows a tab with its main widget focused: a pane's widget taking the focus makes its pane the active one."""
        self.query_one(self.MAIN[tab]).focus()

    def on_tabbed_content_tab_activated(self, event: TabbedContent.TabActivated):
        """A tab clicked: its main widget takes the keys (arrows, Enter)."""
        main = self.MAIN.get(event.pane.id)
        if main is not None and self.focused not in event.pane.query(main):
            self.query_one(main).focus()

    # --------------------------------------------------------------------------------------------- the session

    def action_connect(self):
        self.push_screen(ConnectScreen(self.connection), self._connect_to)

    def _connect_to(self, connection: Connection | None):
        if connection is not None:
            self._open(connection)

    def _open(self, connection: Connection):
        """Closes the session (in the thread that opens the next) and opens ``connection``'s."""
        old = self._close_session()
        self.connection = connection
        self.generation += 1
        self.error = None
        self._status(f"connecting: {connection.describe()}")
        self.run_worker(functools.partial(self._open_thread, connection, old, self.generation), thread=True,
                        group="open", exclusive=True)

    def _open_thread(self, connection: Connection, old: zenoh.Session | None, generation: int):
        if old is not None:
            zsession.close(old)
        try:
            session = connection.open()
        except Exception as error:  # noqa: BLE001 - shown: what Zenoh said, nothing opened
            self.call_from_thread(self._open_failed, generation, f"{type(error).__name__}: {error}")
            return
        self.call_from_thread(self._opened, generation, session)

    def _open_failed(self, generation: int, error: str):
        if generation == self.generation:
            self.error = f"not connected ({self.connection.describe()}): {error}"
            self._status()
            self.notify(self.error, severity="error", timeout=10)
            self._offer_connect()

    def _offer_connect(self):
        """Started on this PC's router and found nothing there: the connect dialog, once, on the address."""
        if self._auto_connect and not isinstance(self.screen, ConnectScreen):
            self._auto_connect = False
            self.push_screen(ConnectScreen(self.connection, "nothing found on this PC's router: give the address of "
                                           f"another (a host alone: tcp, port {zsession.DEFAULT_PORT})"),
                             self._connect_to)

    def action_refresh(self):
        """Looks at the network again now (streams, sessions, tokens), instead of at the next period."""
        self._discover_now.set()
        self.query_one("#rerun-found", Static).update(Text("looking...", style="yellow"))

    def _opened(self, generation: int, session: zenoh.Session):
        if generation != self.generation:  # another connection was asked for meanwhile
            threading.Thread(target=session.close, daemon=True).start()
            return
        self.session = session
        self.discovery = Discovery()
        with self._reports_lock:
            self._reports.clear()
        self._diagnostics_subscriber = session.declare_subscriber("**/" + topics.DIAGNOSTICS, self._on_diagnostics)
        self.run_worker(functools.partial(self._discover_thread, session, generation), thread=True,
                        group="discover", exclusive=True)
        self._status()

    def _close_session(self) -> zenoh.Session | None:
        """Undeclares the interface's subscribers; the session, to be closed by the caller (or None)."""
        self.workers.cancel_group(self, "discover")
        for meter in (self.meter, self.detail):
            if meter is not None:
                meter.close()
        measuring = self.meter is not None
        self.meter = self.detail = None
        self._measure_after_open = measuring
        if self._diagnostics_subscriber is not None:
            self._diagnostics_subscriber.undeclare()
            self._diagnostics_subscriber = None
        session, self.session = self.session, None
        return session

    def _on_diagnostics(self, sample: zenoh.Sample):
        try:
            camera, array = diagnostics.read(sample)
            sender = diagnostics.sender(sample)
        except Exception as error:  # noqa: BLE001 - shown in the reports: a publisher of another version
            with self._reports_lock:
                self._report_errors = [f"{sample.key_expr}: {error}"]
            return
        now = time.time_ns()
        with self._reports_lock:
            for status in array.status:
                self._reports[(camera, status.name)] = (now, status, sender)

    def _discover_thread(self, session: zenoh.Session, generation: int):
        worker = get_current_worker()
        missing_since: dict[str, float] = {}  # peers the admin space shows and this peer has no link to, since when
        while not worker.is_cancelled:
            started = time.monotonic()
            try:
                found = Discovery(survey.query_network(session, 1.0), find_streams(session, "", 1.0),
                                  survey.liveliness_tokens(session, 0.5), statuses(session, 0.5))
                found.unreachable = self._unreachable(session, found.network, missing_since)
            except Exception as error:  # noqa: BLE001 - shown; discovery goes on
                if worker.is_cancelled:
                    return
                found = Discovery(error=f"{type(error).__name__}: {error}")
            if worker.is_cancelled:
                return
            try:
                self.call_from_thread(self._discovered, generation, found)
            except RuntimeError:  # the app has ended
                return
            while (not worker.is_cancelled and time.monotonic() - started < self.discover_every
                   and not self._discover_now.is_set()):
                time.sleep(0.05)
            self._discover_now.clear()

    def _unreachable(self, session: zenoh.Session, network, missing_since: dict[str, float]) -> list[str]:
        """As a peer: the peers of the admin space it has had no link to for UNREACHABLE_S (their locators are not
        reachable from here, e.g. another host's loopback over a tunnel: their streams' queries go unanswered)."""
        if self.connection.mode != "peer" or self.connection.config or network is None:
            return []
        own = str(session.info.zid())
        linked = {str(z) for z in session.info.peers_zid()}
        now = time.monotonic()
        missing = {z for z, n in network.nodes.items() if n.whatami == "peer" and z != own and z not in linked}
        for zid in list(missing_since):
            if zid not in missing:
                del missing_since[zid]
        for zid in missing:
            missing_since.setdefault(zid, now)
        return sorted(z for z in missing if now - missing_since[z] >= UNREACHABLE_S)

    def _discovered(self, generation: int, found: Discovery):
        if generation != self.generation or self._ended():
            return
        self.discovery = found
        self._discovered_at = time.time()
        self.uses = self._find_uses()
        if self._auto_connect:
            own = str(self.session.info.zid()) if self.session else ""
            others = [z for z, n in (found.network.nodes.items() if found.network else ()) if n.whatami != "router"
                      and z != own]
            if found.network is None or not found.network.routers or not (others or found.streams):
                self._offer_connect()
            else:
                self._auto_connect = False
        self._status()
        self._show_nodes()
        self._show_graph()
        self._show_rerun_streams()
        suffix = "/" + parameters.P.PARAMETERS_TOKEN  # the nodes offering parameters: their liveliness tokens
        self.query_one(ParametersPane).show_nodes(sorted(t[:-len(suffix)] for t in found.tokens if t.endswith(suffix)),
                                                  self.session)
        if self._measure_after_open and self.session is not None and self.meter is None:
            self._measure_after_open = False
            self.action_measure()

    def _status(self, text: str | None = None):
        status = self.query_one("#status", Static)
        if text is not None:
            status.update(Text(text, style="yellow"))
            return
        if self.error:
            status.update(Text(self.error, style="bold red"))
            return
        d = self.discovery
        line = Text(f"{self.connection.describe()}  zid {_short(str(self.session.info.zid())) if self.session else '-'}"
                    "  ")
        if d.network is None:
            line.append("looking..." if d.error is None else "", style="yellow")
        elif not d.network.routers:
            line.append("no router answered (admin space): sessions and graph unknown; connect to a zenohd router",
                        style="bold yellow")
        else:
            sessions = sum(1 for n in d.network.nodes.values() if n.whatami != "router")
            line.append(f"{len(d.network.routers)} router(s), {sessions} session(s), {len(d.streams)} stream(s)")
        if d.error:
            line.append(f"  discovery failed: {d.error}", style="bold red")
        if d.unreachable:
            line.append(f"  {len(d.unreachable)} peer(s) not reachable from here (their addresses are not this "
                        "host's: a tunnel?): connect as a client (c, client mode; xblick --connect <router>)",
                        style="bold yellow")
        if self.meter is not None:
            line.append("  measuring every topic (m: stop)", style="magenta")
        status.update(line)

    # ---------------------------------------------------------------------------------------------- the topics

    def action_measure(self):
        """Subscribes to every topic (rates, sizes, types), or stops it."""
        if self.meter is not None:
            self.meter.close()
            self.meter = None
            self._rates, self._before = {}, None
        elif self.session is None:
            self.notify("not connected", severity="error")
        else:
            self.meter = survey.TrafficMeter(self.session, "**")
            self.notify("subscribed to every topic: publishers now produce every representation (m to stop)")
        self._status()

    def action_close_detail(self):
        if self.detail is not None:
            self.detail.close()
        self.detail, self.detail_key = None, None
        self.query_one("#topic-detail", Static).update("Enter on a topic: its publishers, subscribers and last message")

    def _find_uses(self) -> dict[str, list[survey.Use]]:
        network = self.discovery.network
        if network is None:
            return {}
        own = [str(self.session.info.zid())] if self.session else []
        return {key: uses for key in network.publishers() if (uses := network.uses(key, leave_out=own))}

    def _stream_uses(self, base: str) -> list[survey.Use]:
        return [u for key, uses in sorted(self.uses.items()) if key.startswith(base + "/") for u in uses]

    def _labels(self) -> dict[str, str]:
        network = self.discovery.network
        own = str(self.session.info.zid()) if self.session else ""
        labels = {zid: node.label for zid, node in network.nodes.items()} if network else {}
        labels[own] = "this interface"
        return labels

    def _ended(self) -> bool:
        """The app is ending: its widgets are being removed while timers and the discovery thread still call in."""
        try:
            return not self.is_running or not self.screen_stack[0].query("#topic-table")
        except IndexError:  # no screen any more
            return True

    def _tick(self):
        if self._ended():
            return
        if self.meter is not None:
            now, snapshot = time.monotonic(), self.meter.snapshot()
            if self._before is not None:
                self._rates = survey.rates(self._before[1], snapshot, now - self._before[0])
            self._before = (now, snapshot)
        # the ages of the last second, by key: of every topic measured, else of the selected one
        ages = self.detail.take_ages() if self.detail is not None else {}
        if self.meter is not None:
            ages.update(self.meter.take_ages())
        self._ages = {key: (survey.median_p95(sends), survey.median_p95(receives))
                      for key, (sends, receives) in ages.items()}
        self._show_topics()
        self._show_reports()
        self._show_detail()

    def _show_topics(self):
        """The keys as a tree (``topic_tree``): a row per key and per prefix, a prefix's publishers and subscribers
        once on it when every key below has the same; rates, sizes and ages where measured."""
        table = self.query_one("#topic-table", DataTable)
        network = self.discovery.network
        publishers = network.publishers() if network else {}
        traffic = self.meter.snapshot() if self.meter is not None else {}
        if self.detail is not None:
            traffic.update(self.detail.snapshot())
        labels = short_names(self._labels())
        own = str(self.session.info.zid()) if self.session else ""
        subscriptions = ([(n.zid, zenoh.KeyExpr(s)) for n in network.nodes.values() if n.zid != own
                          for s in n.subscribes] if network else [])
        now = time.time_ns()
        keys = set(publishers) | set(traffic)

        def names(zids) -> str:
            return ", ".join(sorted({labels.get(z, _short(z)) for z in zids}))

        def subscribers(key: str) -> frozenset:
            expr = zenoh.KeyExpr(key)
            return frozenset(z for z, s in subscriptions if z not in publishers.get(key, ()) and s.intersects(expr))

        cells = {key: (frozenset(publishers.get(key, ())), subscribers(key)) for key in keys}
        rows = {}
        self._topic_rows = {row.path: row for row in topic_tree(keys, self._collapsed, cells)}
        for row in self._topic_rows.values():
            if row.key is None:  # a prefix: its sessions when the same for every key below it
                pubs, subs = row.common(cells)
                rows[row.path] = (Text(row.label, style="bold"), "", Text(names(pubs) if row.show_pub else ""),
                                  Text(names(subs) if row.show_sub else ""), "", "", "", "", "", "")
                continue
            key = row.key
            t = traffic.get(key)
            hz, size = self._rates.get(key, (None, None))
            send, receive = self._ages.get(key, (None, None))
            pubs, subs = cells[key]
            rows[row.path] = (Text(row.label), f"{hz:.1f}" if hz is not None else "",
                              Text(names(pubs) if row.show_pub else ""), Text(names(subs) if row.show_sub else ""),
                              Text(_count(self.uses.get(key, []))), f"{size / 1e3:.1f}" if size else "",
                              _ms_pair(send), _ms_pair(receive) if receive else (
                                  _ms((now - t.stamp_ns) / 1e6) if t and t.stamp_ns else ""),
                              t.encoding.removeprefix("application/cdr;") if t else "", _clock(t.stamp_ns) if t else "")
        _sync(table, rows, ordered=True)

    def on_data_table_row_selected(self, event: DataTable.RowSelected):
        if event.data_table.id == "topic-table":
            row = self._topic_rows.get(event.row_key.value)
            if row is None:
                return
            if row.key is None:  # a prefix: folded or unfolded
                self._collapsed ^= {row.path}
                self._show_topics()
            else:
                self._select_topic(row.key)
        elif event.data_table.id == "node-table":
            self._selected_node = event.row_key.value
            self._show_node_detail()

    def _select_topic(self, key: str):
        if self.session is None:
            return
        if self.detail is not None:
            self.detail.close()
        self.detail_key = key
        self.detail = survey.TrafficMeter(self.session, key)
        self.detail.keep.add(key)
        self._show_detail()

    def _show_detail(self):
        if self.detail is None or self.detail_key is None:
            return
        key = self.detail_key
        network = self.discovery.network
        labels = self._labels()
        text = Text()
        text.append(key + "\n", style="bold")
        if network is not None:
            pubs = network.publishers().get(key, set())
            text.append("published by: " + (", ".join(f"{labels.get(z)} ({_short(z)})" for z in sorted(pubs))
                                            or "no session the routers know") + "\n")
            expr = zenoh.KeyExpr(key)
            subs = sorted({f"{labels.get(n.zid)} ({s})" for n in network.nodes.values() for s in n.subscribes
                           if zenoh.KeyExpr(s).intersects(expr)})
            text.append("subscribed by: " + (", ".join(subs) or "nobody") + "\n")
        # the message before the long parts (the stream's record, the consumers' ways): a small pane shows it first
        tail = Text()
        for base, record in self.discovery.streams:
            if key.startswith(base + "/"):
                text.append(f"offers {', '.join(record.get('transports', [])) or 'nothing it says'}; in use: "
                            f"{_count(self._stream_uses(base)) or 'nothing'}\n")
                tail.append(f"stream {base}: {json.dumps(record)}\n")
        if key in self.uses:
            tail.append("taken by:\n", style="bold")
            for use in self.uses[key]:
                tail.append(f"  {labels.get(use.consumer, _short(use.consumer))} ({use.expr}): {use.summary}, "
                            f"{use.detail}\n")
        t = self.detail.snapshot().get(key)
        last = self.detail.last(key)
        widget = self.query_one("#topic-detail", Static)
        if t is None or last is None:
            text.append("waiting for a message...\n", style="yellow")
            widget.update(Group(text, tail))
            return
        text.append(f"{t.count} messages since selected, last {len(last[1]) / 1e3:.1f} kB, {t.encoding}, "
                    f"stamp {_clock(t.stamp_ns) or 'none'}\n")
        recent = self.detail.recent(key)[-5:]
        if recent:  # per sample: at send exact (the publisher's clock), at receive with the way and both clocks
            text.append("age ms at send / at receive, the last samples: " + ", ".join(
                f"{_ms(s / 1e6) if s is not None else '-'} / {_ms(r / 1e6) if r is not None else '-'}"
                for _, s, r in recent) + "\n")
        try:
            body = Pretty(survey.content(*last), expand_all=False)
        except Exception as error:  # noqa: BLE001 - shown: a message this version cannot read
            body = Text(f"cannot show it: {type(error).__name__}: {error}", style="bold red")
        widget.update(Group(text, body, tail))

    # ----------------------------------------------------------------------------------------------- the nodes

    def _show_nodes(self):
        network = self.discovery.network
        table = self.query_one("#node-table", DataTable)
        rows = {}
        if network is not None:
            labels = self._labels()
            for zid, node in network.nodes.items():
                rows[zid] = (Text(labels.get(zid, node.label)), node.whatami, _short(zid),
                             ", ".join(sorted(node.addresses)), str(len(node.publishes)), str(len(node.subscribes)),
                             str(len(node.queryables)), ", ".join(_short(r) for r in sorted(node.routers)))
        _sync(table, rows)
        self._show_node_detail()

    def _show_reports(self):
        table = self.query_one("#report-table", DataTable)
        now = time.time_ns()
        rows = {}
        with self._reports_lock:
            reports = dict(self._reports)
            errors = list(self._report_errors)
        labels = self._labels()
        for (camera, name), (received, status, sender) in reports.items():
            level = diagnostics.level_name(status.level)
            style = {"OK": "green", "WARN": "yellow", "ERROR": "bold red", "STALE": "dim"}.get(level, "")
            rows[f"{camera}\t{name}"] = (Text(topics.diagnostics(camera)), Text(name), Text(level, style=style),
                                         Text(status.message), f"{(now - received) / 1e9:.1f}",
                                         Text(_sender(sender, labels)))
        for key, status in self.discovery.statuses.items():
            state = status.get("state", "running" if status.get("kind") == "mps" else "")
            rows[key] = (Text(key), Text(str(status.get("name", ""))), Text(str(state)), Text(_status_line(status)),
                         "", "")
        for error in errors:
            rows["error\t" + error] = (Text("error", style="bold red"), "", "", Text(error, style="red"), "", "")
        _sync(table, rows)

    def _show_node_detail(self):
        zid = self._selected_node
        network = self.discovery.network
        widget = self.query_one("#node-detail", Static)
        if zid is None or network is None:
            return
        node = network.nodes.get(zid)
        if node is None:
            widget.update(Text(f"{zid}: gone", style="yellow"))
            return
        text = Text()
        text.append(f"{self._labels().get(zid, node.label)}  {node.whatami}  {zid}\n", style="bold")
        if zid in network.routers:
            r = network.routers[zid]
            text.append(f"router {r.version}\nlocators {', '.join(r.locators)}; {r.sessions} sessions\n")
        if node.addresses:
            text.append("address " + ", ".join(sorted(node.addresses)) + "\n")
        for kind, keys in (("publishes", node.publishes), ("subscribes", node.subscribes),
                           ("answers", node.queryables), ("liveliness tokens", node.tokens)):
            if keys:
                text.append(f"{kind}:\n", style="bold")
                text.append("".join(f"  {k}\n" for k in sorted(keys)))
        with self._reports_lock:
            reports = dict(self._reports)
        own = sorted((c, n, s) for (c, n), (_, s, sender) in reports.items() if sender == zid)
        if own:
            text.append("reports:\n", style="bold")
            for camera, name, status in own:
                values = ", ".join(f"{kv.key} {kv.value}" for kv in status.values)
                text.append(f"  {topics.diagnostics(camera)} {name} {diagnostics.level_name(status.level)}: "
                            f"{status.message}" + (f"  [{values}]" if values else "") + "\n")
        cameras = {k[: -len(topics.DIAGNOSTICS) - 1] for k in node.publishes if k.endswith("/" + topics.DIAGNOSTICS)}
        unknown = sorted(f"{topics.diagnostics(c)} {n}" for (c, n), (_, _, sender) in reports.items()
                         if sender is None and c in cameras)
        if unknown:
            text.append("reports on its diagnostics keys without a sender (Durchblick before 0.3.29), any of them "
                        "may be its own: " + ", ".join(unknown) + "\n", style="yellow")
        labels = self._labels()
        for base, record in self.discovery.streams:
            if any(k.startswith(base + "/") for k in node.publishes):
                text.append(f"stream {base} offers {', '.join(record.get('transports', []))}; in use: "
                            f"{_count(self._stream_uses(base)) or 'nothing'}\n")
        given = [(key, u) for key in sorted(node.publishes) for u in self.uses.get(key, [])]
        if given:
            text.append("its keys taken by:\n", style="bold")
            text.append("".join(f"  {key} -> {labels.get(u.consumer, _short(u.consumer))}: {u.summary}\n"
                                for key, u in given))
        takes = [(key, u) for key, uses in sorted(self.uses.items()) for u in uses if u.consumer == zid]
        if takes:
            text.append("takes:\n", style="bold")
            text.append("".join(f"  {key}: {u.summary}, {u.detail}\n" for key, u in takes))
        for key in sorted(node.queryables):
            if key in self.discovery.statuses:
                text.append(f"{key}:\n{json.dumps(self.discovery.statuses[key], indent=1)}\n")
        widget.update(text)

    # ----------------------------------------------------------------------------------------------- the graph

    def _graph_parts(self) -> tuple[list[GraphNode], list[GraphEdge], dict[tuple[str, str], list[tuple[str, str]]]]:
        """The graph's boxes (sessions but the routers and this interface), its edges (publisher -> subscriber, a
        label from how the keys are taken), and per edge its keys with how each is taken."""
        network = self.discovery.network
        if network is None:
            return [], [], {}
        own = str(self.session.info.zid()) if self.session else ""
        keys: dict[tuple[str, str], list[tuple[str, str]]] = {}
        for pub, key, sub, _expr in network.connections(leave_out=[own]):
            taken = next((u.summary for u in self.uses.get(key, []) if u.consumer == sub), "")
            pair = keys.setdefault((pub, sub), [])
            if (key, taken) not in pair:
                pair.append((key, taken))
        with self._reports_lock:
            reports = dict(self._reports)
        nodes = []
        short = short_names({z: n.label for z, n in network.nodes.items()})
        for n in sorted((n for n in network.nodes.values() if n.zid != own and n.whatami != "router"),
                        key=lambda n: (n.label, n.zid)):
            lines = [f"{n.whatami} {_short(n.zid)}"]
            for (_camera, name), (_, status, sender) in sorted(reports.items()):
                if sender == n.zid:
                    lines.append(f"{diagnostics.level_name(status.level)} {status.message}")
            if not any(pub == n.zid for pub, _ in keys) and n.publishes:
                lines.append(f"{len(n.publishes)} keys, nobody takes them")
            nodes.append(GraphNode(n.zid, short[n.zid], lines))
        edges = []
        for (pub, sub), pairs in sorted(keys.items()):
            ways = sorted({t for _, t in pairs if t})
            if len(ways) == 1 and all(t for _, t in pairs):
                label = ways[0]
            elif len(pairs) == 1:
                label = pairs[0][0].rsplit("/", 1)[-1]
            else:
                label = f"{len(pairs)} keys"
            edges.append(GraphEdge(pub, sub, label))
        return nodes, edges, keys

    def _show_graph(self):
        nodes, edges, self._graph_keys = self._graph_parts()
        self.query_one("#graph", GraphView).show(nodes, edges)
        self._show_graph_detail()

    def on_graph_view_selected(self, event: GraphView.Selected):
        self._show_graph_detail()

    def on_graph_view_hovered(self, event: GraphView.Hovered):
        self._graph_hovered = event.what
        self._show_graph_detail()

    def _show_graph_detail(self):
        """What the mouse is over, else the selected session: its reports, what it sends and what it takes."""
        graph = self.query_one("#graph", GraphView)
        widget = self.query_one("#graph-detail", Static)
        labels = self._labels()
        keys = getattr(self, "_graph_keys", {})
        what = self._graph_hovered or (("node", graph.selected) if graph.selected else None)
        if what is None:
            widget.update(GRAPH_HELP)
            return
        text = Text()
        if what[0] == "edges":
            for k in sorted(what[1]):
                if k >= len(graph.edges):
                    continue
                e = graph.edges[k]
                text.append(f"{labels.get(e.source, _short(e.source))} -> {labels.get(e.target, _short(e.target))}\n",
                            style="bold yellow")
                text.append("".join(f"  {key}" + (f"  [{taken}]" if taken else "") + "\n"
                                    for key, taken in keys.get((e.source, e.target), [])))
            widget.update(text)
            return
        zid = what[1]
        node = next((n for n in graph.nodes if n.id == zid), None)
        if node is None:
            widget.update(GRAPH_HELP)
            return
        text.append(f"{labels.get(zid, node.title)}  ({', '.join(node.lines)})\n", style="bold magenta")
        for (pub, sub), pairs in sorted(keys.items()):
            if pub == zid:
                text.append(f"  sends to {labels.get(sub, _short(sub))}: ", style="green")
                text.append(", ".join(k + (f" [{t}]" if t else "") for k, t in pairs) + "\n")
        for (pub, sub), pairs in sorted(keys.items()):
            if sub == zid:
                text.append(f"  takes from {labels.get(pub, _short(pub))}: ", style="cyan")
                text.append(", ".join(k + (f" [{t}]" if t else "") for k, t in pairs) + "\n")
        widget.update(text)

    # ----------------------------------------------------------------------------------------------- rerun

    def _rerun_found(self) -> dict[str, list[str]]:
        from .cli import _bridged

        return {**_bridged(self.discovery.streams), "trajectories": pose_topics(self.discovery.network)}

    def _show_rerun_streams(self):
        found = self._rerun_found()
        count = sum(len(keys) for keys in found.values())
        self.query_one("#rerun-found", Static).update(Text(
            f"{count} found at {time.strftime('%H:%M:%S', time.localtime(self._discovered_at or time.time()))}; "
            f"looked for every {self.discover_every:g} s (r: now)", style="dim"))
        options = {f"{kind}\t{key}" for kind, keys in found.items() for key in keys}
        if options == self._known_streams:
            return
        self._known_streams = options
        self._set_rerun_options(found, self._selection_values())

    def _selection_values(self) -> set[str]:
        return set(self.query_one("#rerun-streams", SelectionList).selected)

    def _set_rerun_options(self, found: dict[str, list[str]], selected: set[str]):
        """The streams to choose from: those found, and selected ones not found (a preset's, a node not running)."""
        listed = {f"{kind}\t{key}" for kind, keys in found.items() for key in keys}
        widget = self.query_one("#rerun-streams", SelectionList)
        highlighted = widget.highlighted
        widget.clear_options()
        for value in sorted(listed | selected, key=lambda v: (v.split("\t")[1], v)):
            kind, key = value.split("\t")
            missing = "" if value in listed else "  (not found)"
            widget.add_option(Selection(Text(f"{key}  {RERUN_KINDS[kind]}{missing}"), value, value in selected))
        if widget.option_count:  # a highlighted line, so that Space chooses from the start
            widget.highlighted = min(highlighted or 0, widget.option_count - 1)
        self._update_command()

    def selection(self) -> RerunSelection:
        """What the Rerun tab says now."""
        chosen: dict[str, list[str]] = {kind: [] for kind in RERUN_KINDS}
        for value in sorted(self._selection_values()):
            kind, key = value.split("\t")
            chosen[kind].append(key)

        def number(widget_id: str) -> float | None:
            text = self.query_one(widget_id, Input).value.strip()
            return float(text) if text else None

        return RerunSelection(**chosen, prefix=self.query_one("#rerun-prefix", Input).value.strip(),
                              stale_after=number("#rerun-stale-after"), cloud_rate=number("#rerun-cloud-rate"),
                              viewer=self.query_one("#rerun-viewer", Select).value,
                              viewer_value=self.query_one("#rerun-viewer-value", Input).value.strip(),
                              extra=self.query_one("#rerun-extra", Input).value.strip(), connection=self.connection)

    def apply(self, selection: RerunSelection):
        """Shows a selection (a preset) in the Rerun tab."""
        values = {f"{kind}\t{key}" for kind in RERUN_KINDS for key in getattr(selection, kind)}
        self._set_rerun_options(self._rerun_found(), values)
        self.query_one("#rerun-prefix", Input).value = selection.prefix
        self.query_one("#rerun-stale-after", Input).value = ("" if selection.stale_after is None
                                                             else f"{selection.stale_after:g}")
        self.query_one("#rerun-cloud-rate", Input).value = ("" if selection.cloud_rate is None
                                                            else f"{selection.cloud_rate:g}")
        self.query_one("#rerun-viewer", Select).value = selection.viewer
        self.query_one("#rerun-viewer-value", Input).value = selection.viewer_value
        self.query_one("#rerun-extra", Input).value = selection.extra
        if selection.connection != self.connection:
            self.notify(f"the preset's connection: {selection.connection.describe()}")
            self._open(selection.connection)
        self._update_command()

    def _update_command(self):
        widget = self.query_one("#rerun-command", Static)
        try:
            args = self.selection().check()
        except (ValueError, OSError) as error:
            widget.update(Text(f"xblick rerun: {error}", style="bold red"))
            return
        widget.update(Text("xblick rerun " + shlex.join(args)))

    def on_selection_list_selected_changed(self, event):
        self._update_command()

    def on_input_changed(self, event: Input.Changed):
        if event.input.id and event.input.id.startswith("rerun-"):
            self._update_command()

    def on_select_changed(self, event: Select.Changed):
        if event.select.id == "rerun-viewer":
            self._update_command()

    def on_button_pressed(self, event: Button.Pressed):
        handlers = {"rerun-launch": self.launch, "rerun-refresh": self.action_refresh, "rerun-stop": self.stop_rerun, "preset-save": self._save_preset,
                    "preset-load": self._load_preset, "preset-delete": self._delete_preset}
        if event.button.id in handlers:
            handlers[event.button.id]()

    def _load_presets(self) -> dict[str, RerunSelection]:
        try:
            presets = self.presets.load()
        except ValueError as error:
            self.notify(str(error), severity="error", timeout=10)
            presets = {}
        self.query_one("#preset-select", Select).set_options([(n, n) for n in sorted(presets)])
        return presets

    def _chosen_preset(self) -> str | None:
        value = self.query_one("#preset-select", Select).value
        if value is Select.BLANK:
            self.notify("choose a preset first", severity="error")
            return None
        return value

    def _save_preset(self):
        name = self.query_one("#preset-name", Input).value.strip()
        try:
            self.presets.save(name, self.selection())
        except (ValueError, OSError) as error:
            self.notify(f"not saved: {error}", severity="error", timeout=10)
            return
        self._load_presets()
        self.query_one("#preset-select", Select).value = name
        self.notify(f"preset {name!r} saved in {self.presets.path}")

    def _load_preset(self):
        name = self._chosen_preset()
        if name is None:
            return
        presets = self._load_presets()
        if name not in presets:
            self.notify(f"no preset {name!r} in {self.presets.path}", severity="error")
            return
        self.query_one("#preset-select", Select).value = name
        self.query_one("#preset-name", Input).value = name
        self.apply(presets[name])

    def _delete_preset(self):
        name = self._chosen_preset()
        if name is None:
            return
        try:
            self.presets.delete(name)
        except (KeyError, ValueError, OSError) as error:
            self.notify(str(error), severity="error")
            return
        self._load_presets()
        self.notify(f"preset {name!r} deleted")

    def launch(self):
        """Starts ``xblick rerun`` with the tab's selection as a child process; its output goes to the log."""
        if self._rerun is not None and self._rerun.poll() is None:
            self.notify("xblick rerun runs already: stop it first", severity="error")
            return
        if importlib.util.find_spec("rerun") is None:
            self.notify("xblick rerun needs the extra: pip install 'xblick[rerun]'", severity="error",
                        timeout=10)
            return
        try:
            command = self.selection().command()
        except (ValueError, OSError) as error:
            self.notify(f"xblick rerun: {error}", severity="error", timeout=10)
            return
        self._rerun_line("$ xblick " + shlex.join(command[3:]), "bold")
        # a session of its own (no Ctrl-C of the terminal); stopped by PID with SIGTERM, its Ctrl-C
        self._rerun = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                       stderr=subprocess.STDOUT, text=True, bufsize=1, start_new_session=True,
                                       env={**os.environ, "PYTHONUNBUFFERED": "1"})
        self.query_one("#rerun-stop", Button).disabled = False
        self.run_worker(functools.partial(self._pump, self._rerun), thread=True, group="rerun")

    def _rerun_line(self, line: str, style: str = ""):
        self.rerun_output.append(line)
        self.query_one("#rerun-log", RichLog).write(Text(line, style=style))

    def _pump(self, process: subprocess.Popen):
        for line in process.stdout:
            try:
                self.call_from_thread(self._rerun_line, line.rstrip("\n"))
            except RuntimeError:  # the app has ended
                return
        code = process.wait()
        with contextlib.suppress(RuntimeError):
            self.call_from_thread(self._rerun_ended, process, code)

    def _rerun_ended(self, process: subprocess.Popen, code: int):
        self._rerun_line(f"xblick rerun ended, exit code {code}", "green" if code == 0 else "bold red")
        if process is self._rerun:
            self.query_one("#rerun-stop", Button).disabled = True

    def stop_rerun(self, timeout: float = 10.0):
        """SIGTERM to the child (its cleanup sends what Rerun buffers), SIGKILL after ``timeout``."""
        process = self._rerun
        if process is None or process.poll() is not None:
            return
        process.send_signal(signal.SIGTERM)
        try:
            process.wait(timeout)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()

    def on_unmount(self):
        for pane in self.query(ParametersPane):  # (gone already when the app ends)
            pane.close()
        self.stop_rerun()
        session = self._close_session()
        if session is not None:
            zsession.close(session)


def _ms(value: float) -> str:
    """Milliseconds: whole from 10 ms, else to 0.1 ms."""
    return f"{value:.0f}" if abs(value) >= 10 else f"{value:.1f}"


def _ms_pair(stats: tuple[float, float] | None) -> str:
    """``median / p95`` ms, empty without values."""
    return f"{_ms(stats[0])} / {_ms(stats[1])}" if stats else ""


def _count(uses: list[survey.Use]) -> str:
    """``shm×2, tcp-compressed×1``: the uses by how they take it."""
    counts = collections.Counter(u.summary for u in uses)
    return ", ".join(f"{summary}×{n}" for summary, n in sorted(counts.items()))


def _sender(zid: str | None, labels: dict[str, str]) -> str:
    if zid is None:
        return "unknown (before 0.3.29)"
    return f"{labels.get(zid, 'gone')} ({_short(zid)})"


def _status_line(status: dict) -> str:
    """A ``xblick/.../status`` record in a line (a record and play node's, the MPS daemon's)."""
    if status.get("kind") == "mps":
        if "servers_error" in status:
            return f"{status['pipe_dir']}: {status['servers_error']}"
        clients = [c for s in status.get("servers", []) for c in s["clients"]]
        gpus = ", ".join(g["name"] for g in status.get("gpus", [])) or status.get("gpus_error", "")
        names = ", ".join(sorted({str(c["command"] or c["pid"]) for c in clients}))
        return (f"MPS at {status['pipe_dir']}, {len(clients)} clients" + (f" ({names})" if names else "")
                + f", CUDA_VISIBLE_DEVICES {status.get('cuda_visible_devices')}, {gpus}")
    if status.get("state") == "recording":
        return f"{status.get('file')}, {status.get('messages')} messages, {status.get('dropped')} dropped"
    if status.get("play"):
        return f"at {status['play'].get('position_s', 0):.1f} s, {status['play'].get('sent')} messages"
    return str(status.get("state", ""))


def _sync(table: DataTable, rows: dict, ordered: bool = False):
    """The table's rows made ``rows`` (row key -> cells), sorted by key (``ordered``: in the dict's order), without
    clearing it (the cursor stays)."""
    present = [key.value for key in table.rows]
    wanted = list(rows) if ordered else sorted(rows)
    if (present != wanted) if ordered else (set(present) != set(rows)):
        cursor = table.cursor_row
        table.clear()
        for key in wanted:
            table.add_row(*rows[key], key=key)
        if rows:
            table.move_cursor(row=min(cursor, len(rows) - 1))
        return
    columns = list(table.columns)
    for key, cells in rows.items():
        for column, value in zip(columns, cells):
            if str(table.get_cell(key, column)) != str(value):
                table.update_cell(key, column, value, update_width=True)


def run(connection: Connection | None = None) -> int:
    """The interface in this terminal; ends with q. Without a terminal it refuses (a command is wanted)."""
    if not (sys.stdin.isatty() and sys.stdout.isatty()):
        raise SystemExit("xblick: without a command it opens the terminal interface, which needs a terminal; "
                         "give a command (xblick -h)")
    # The terminal's own size, also after a resize: Textual asks shutil.get_terminal_size, which takes COLUMNS and
    # LINES first when they are set, and would keep the interface at that size whatever the terminal is. They are set
    # where nobody meant them for us: readline (Python before 3.13, e.g. a test harness) puts 80 x 24 into the
    # environment of every child process.
    for name in ("COLUMNS", "LINES"):
        os.environ.pop(name, None)
    XblickApp(connection).run()
    return 0
