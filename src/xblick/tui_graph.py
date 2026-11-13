"""The terminal interface's graph: the sessions as boxes in columns, the data between them as lines with arrows.

A layered drawing (Sugiyama's method, in small): a session that takes data from another stands in a column to the
right of it; an edge over several columns passes the columns between them as a line of its own; the order in a column
follows the mean position of the neighbours, a few sweeps to and fro, which leaves few crossings. A cycle (two
sessions that take each other's data) is drawn with one edge against the flow: its arrow points out of the box
on the right (``◀``) instead of into it (``▶``).

Between two columns every box that sends has one vertical lane (a bus) that branches into the boxes it sends to; each
edge enters its box on a row of its own, with a short label (the transport or the key) before the arrow. A horizontal
line crosses another edge's lane as a line over it (``─``), and lines of one edge join with box-drawing junctions.

The widget (``GraphView``) selects a session with the arrow keys or a click and colours its edges (out green, in
cyan); the mouse over an edge or a box names it in the view's detail line (``hovered``), and ``describe`` tells the
caller what to show.
"""

from __future__ import annotations

import dataclasses
from typing import ClassVar

from rich.style import Style
from rich.text import Text
from textual.binding import Binding
from textual.events import Click, Leave, MouseMove
from textual.geometry import Region, Size
from textual.message import Message
from textual.reactive import reactive
from textual.widget import Widget

N, E, S, W = 1, 2, 4, 8  # the directions a line leaves a cell in
LINES = {N | S: "│", E | W: "─", N | E: "└", N | W: "┘", S | E: "┌", S | W: "┐", N | S | E: "├", N | S | W: "┤",
         E | W | S: "┬", E | W | N: "┴", N | E | S | W: "┼", N: "│", S: "│", E: "─", W: "─"}
LABEL = 15  # the room for an edge's label before its arrow
MIN_BOX, MAX_BOX = 18, 34  # a box's width
LANE = 2  # columns per lane


@dataclasses.dataclass
class GraphNode:
    """A box: ``title`` on its border, ``lines`` in it (cut to the box's width)."""

    id: str
    title: str
    lines: list[str] = dataclasses.field(default_factory=list)


@dataclasses.dataclass
class GraphEdge:
    """Data from ``source`` to ``target``; ``label`` stands before the arrow (cut to LABEL)."""

    source: str
    target: str
    label: str = ""


@dataclasses.dataclass
class _Element:
    """A box or, for an edge over several columns, a point of it in a column between (``dummy``)."""

    id: str
    layer: int
    dummy: bool = False
    width: int = 1
    height: int = 1
    x: int = 0
    y: int = 0
    in_rows: dict[int, int] = dataclasses.field(default_factory=dict)  # segment -> its row on the left side
    out_row: int = 0


@dataclasses.dataclass
class _Segment:
    """A piece of an edge between two neighbouring columns."""

    edge: int
    left: str
    right: str


@dataclasses.dataclass
class Drawing:
    """A laid-out graph: per cell its line bits and the edges through it, the boxes' regions, and what is drawn on
    top (box borders, text, arrows)."""

    width: int = 0
    height: int = 0
    bits: dict[tuple[int, int], int] = dataclasses.field(default_factory=dict)
    edges_at: dict[tuple[int, int], set[int]] = dataclasses.field(default_factory=dict)
    glyphs: dict[tuple[int, int], tuple[str, str, object]] = dataclasses.field(default_factory=dict)  # char, kind, id
    boxes: dict[str, Region] = dataclasses.field(default_factory=dict)
    layer_of: dict[str, int] = dataclasses.field(default_factory=dict)
    reversed: set[int] = dataclasses.field(default_factory=set)  # edges drawn against the flow


def _layers(ids: list[str], edges: list[GraphEdge]) -> tuple[dict[str, int], set[int]]:
    """The column of every box (the longest path from a box nothing sends to) and the edges reversed to break the
    cycles (depth-first search: an edge back to a box on the search's path)."""
    out: dict[str, list[tuple[str, int]]] = {i: [] for i in ids}
    for n, e in enumerate(edges):
        out[e.source].append((e.target, n))
    state: dict[str, int] = {}
    back: set[int] = set()

    def visit(u: str):
        state[u] = 1
        for v, n in sorted(out[u]):
            if state.get(v) == 1:
                back.add(n)
            elif v not in state:
                visit(v)
        state[u] = 2

    for i in ids:
        if i not in state:
            visit(i)
    preds: dict[str, list[str]] = {i: [] for i in ids}
    for n, e in enumerate(edges):
        a, b = (e.target, e.source) if n in back else (e.source, e.target)
        preds[b].append(a)
    layer: dict[str, int] = {}

    def depth(u: str) -> int:
        if u not in layer:
            layer[u] = 0  # (a DAG now: no recursion back to u)
            layer[u] = max((depth(p) + 1 for p in preds[u]), default=0)
        return layer[u]

    for i in ids:
        depth(i)
    return layer, back


def layout(nodes: list[GraphNode], edges: list[GraphEdge]) -> Drawing:
    """The drawing of ``nodes`` and ``edges`` (edges of a box to itself and of unknown boxes are left out)."""
    known = {n.id: n for n in nodes}
    edges = [e for e in edges if e.source in known and e.target in known and e.source != e.target]
    ids = [n.id for n in nodes]
    layer, back = _layers(ids, edges)
    drawing = Drawing(layer_of=dict(layer), reversed=set(back))
    elements: dict[str, _Element] = {}
    for n in nodes:
        width = max([len(n.title) + 4, *(len(line) + 4 for line in n.lines), MIN_BOX])
        elements[n.id] = _Element(n.id, layer[n.id], width=min(width, MAX_BOX))
    segments: list[_Segment] = []
    for k, e in enumerate(edges):
        a, b = (e.target, e.source) if k in back else (e.source, e.target)
        chain = [a]
        for step in range(layer[a] + 1, layer[b]):
            dummy = f"\0{k}\0{step}"
            elements[dummy] = _Element(dummy, step, dummy=True)
            chain.append(dummy)
        chain.append(b)
        segments.extend(_Segment(k, left, right) for left, right in zip(chain, chain[1:]))
    count = max((el.layer for el in elements.values()), default=-1) + 1
    columns = _order(elements, segments, count, ids)
    # heights: a row per edge entering on the left, at least two of text
    entering: dict[str, list[int]] = {i: [] for i in elements}
    for s, seg in enumerate(segments):
        entering[seg.right].append(s)
    for el in elements.values():
        if not el.dummy:
            text = len(known[el.id].lines)
            el.height = max(text, len(entering[el.id]), 2) + 2
    tallest = _rows(elements, segments, columns)
    for el in elements.values():
        el.out_row = el.y if el.dummy else el.y + el.height // 2
        if el.dummy:
            el.in_rows = {s: el.y for s in entering[el.id]}
        else:
            ordered = sorted(entering[el.id], key=lambda s: elements[segments[s].left].out_row)
            el.in_rows = {s: el.y + 1 + r for r, s in enumerate(ordered)}
    # columns' x: a box column, then the lanes of the boxes sending across the gap and the labels' room
    senders = [_lanes(elements, segments, c) for c in range(count)]
    x = 0
    gaps = []
    for c, col in enumerate(columns):
        width = max((elements[i].width for i in col), default=1)
        for i in col:
            elements[i].x = x
            if elements[i].dummy:
                elements[i].width = width
        x += width
        gaps.append(x)
        x += 2 + LANE * len(senders[c]) + LABEL + 2 if c < count - 1 else 0
    drawing.width, drawing.height = x, tallest
    # lines: lanes first, then the horizontals over them
    lane_x = {}
    for c in range(count):
        for n, sender in enumerate(senders[c]):
            lane_x[sender] = gaps[c] + 2 + LANE * n
    verticals, horizontals = [], []
    for s, seg in enumerate(segments):
        a, b = elements[seg.left], elements[seg.right]
        lx, ya, yb = lane_x[seg.left], a.out_row, b.in_rows[s]
        verticals.append((lx, ya, yb, seg.edge))
        horizontals.append((ya, a.x + a.width, lx, seg.edge))
        horizontals.append((yb, lx, b.x - 1, seg.edge))
        if a.dummy:
            horizontals.append((ya, a.x - 1, a.x + a.width, seg.edge))
        if not b.dummy:
            k = seg.edge
            arrow = "◀" if k in back else "▶"
            drawing.glyphs[(b.x - 1, yb)] = (arrow, "arrow", k)
            label = edges[k].label[:LABEL]
            for n, ch in enumerate(label):
                drawing.glyphs[(b.x - 2 - len(label) + n, yb)] = (ch, "label", k)
    for lx, y0, y1, k in verticals:
        _vline(drawing, lx, y0, y1, k)
    for y, x0, x1, k in horizontals:
        _hline(drawing, y, x0, x1, k)
    for el in elements.values():
        if not el.dummy:
            _box(drawing, el, known[el.id], bool(entering[el.id]) or any(s.left == el.id for s in segments),
                 any(s.left == el.id for s in segments))
    _compact(drawing)
    return drawing


def _compact(drawing: Drawing):
    """Leaves out the rows that only lines pass straight through (or nothing): the row placement aligns ports and
    leaves such rows between the columns' boxes. The row above a box stays when a box stands above it in the same
    columns (boxes do not touch)."""
    glyph_rows = {y for _, y in drawing.glyphs}
    regions = list(drawing.boxes.values())
    under = {b.y - 1 for b in regions
             if any(a.bottom <= b.y and a.x < b.right and b.x < a.right for a in regions if a is not b)}
    crossing: dict[int, bool] = {}
    for (_, y), bits in drawing.bits.items():
        crossing[y] = crossing.get(y, False) or bits != N | S
    gone = [y for y in range(drawing.height) if y not in glyph_rows and y not in under and not crossing.get(y)]
    if not gone:
        return
    shift = [0] * (drawing.height + 1)
    n = 0
    for y in range(drawing.height + 1):
        shift[y] = n
        if y in set(gone):
            n += 1
    drop = set(gone)
    drawing.bits = {(x, y - shift[y]): b for (x, y), b in drawing.bits.items() if y not in drop}
    drawing.edges_at = {(x, y - shift[y]): e for (x, y), e in drawing.edges_at.items() if y not in drop}
    drawing.glyphs = {(x, y - shift[y]): g for (x, y), g in drawing.glyphs.items()}
    drawing.boxes = {i: Region(r.x, r.y - shift[r.y], r.width, r.height) for i, r in drawing.boxes.items()}
    drawing.height -= len(gone)


def _rows(elements: dict[str, _Element], segments: list[_Segment], columns: list[list[str]]) -> int:
    """The elements' rows (``y``), in each column's order with a row between: sweeps to and fro move each element
    to the mean row of its neighbours' ports, as far as the order lets it, which keeps the lines straight. Returns
    the drawing's height."""
    left: dict[str, list[str]] = {i: [] for i in elements}
    right: dict[str, list[str]] = {i: [] for i in elements}
    for seg in segments:
        left[seg.right].append(seg.left)
        right[seg.left].append(seg.right)

    def port(el: _Element) -> int:  # the row of an element's ports, from its top
        return 0 if el.dummy else el.height // 2

    for col in columns:  # stacked first
        y = 0
        for i in col:
            elements[i].y = y
            y += elements[i].height + 1
    for sweep in range(6):
        order = range(len(columns)) if sweep % 2 == 0 else range(len(columns) - 1, -1, -1)
        neighbours = left if sweep % 2 == 0 else right
        for c in order:
            y = 0
            for i in columns[c]:
                el = elements[i]
                near = [elements[j].y + port(elements[j]) for j in neighbours[i]]
                wanted = round(sum(near) / len(near)) - port(el) if near else el.y
                el.y = max(wanted, y)
                y = el.y + el.height + 1
    top = min((el.y for el in elements.values()), default=0)
    for el in elements.values():
        el.y -= top
    return max((el.y + el.height for el in elements.values()), default=0)


def _lanes(elements: dict[str, _Element], segments: list[_Segment], column: int) -> list[str]:
    """The senders of a gap (the elements of ``column`` with segments to the next), left to right: a sender's line
    to its lane must not run on the row of a line that leaves another lane to its box, so that lane stands further
    right; else by row."""
    sends = sorted({seg.left for seg in segments if elements[seg.left].layer == column},
                   key=lambda i: elements[i].out_row)
    before: dict[str, set[str]] = {i: set() for i in sends}  # sender -> the senders whose lanes stand left of it
    for s, seg in enumerate(segments):
        if seg.left not in before:
            continue
        row = elements[seg.right].in_rows[s]
        for other in sends:
            if other != seg.left and elements[other].out_row == row:
                before[seg.left].add(other)
    ordered: list[str] = []
    while len(ordered) < len(sends):
        ready = [i for i in sends if i not in ordered and before[i] <= set(ordered)]
        ordered.append(ready[0] if ready else next(i for i in sends if i not in ordered))  # a cycle: by row
    return ordered


def _order(elements: dict[str, _Element], segments: list[_Segment], count: int, ids: list[str]) -> list[list[str]]:
    """The elements of each column in an order with few crossings: the mean position of the neighbours, sweeps down
    and up (barycentre heuristic); the first order is the boxes' given order."""
    rank = {i: n for n, i in enumerate(ids)}
    columns = [sorted((i for i, el in elements.items() if el.layer == c), key=lambda i: (rank.get(i, len(rank)), i))
               for c in range(count)]
    left: dict[str, list[str]] = {i: [] for i in elements}
    right: dict[str, list[str]] = {i: [] for i in elements}
    for seg in segments:
        left[seg.right].append(seg.left)
        right[seg.left].append(seg.right)
    for sweep in range(8):
        down = sweep % 2 == 0
        order = range(1, count) if down else range(count - 2, -1, -1)
        for c in order:
            near = columns[c - 1] if down else columns[c + 1]
            position = {i: n for n, i in enumerate(near)}
            neighbours = left if down else right

            def centre(i, n, position=position, neighbours=neighbours):
                ps = [position[j] for j in neighbours[i] if j in position]
                return sum(ps) / len(ps) if ps else n

            columns[c] = [i for _, _, i in sorted((centre(i, n), n, i) for n, i in enumerate(columns[c]))]
    return columns


def _put(drawing: Drawing, x: int, y: int, bits: int, edge: int, over: bool):
    """Line bits into a cell. A cell of another edge: joined if both are the same lane (one sender), else the newer
    line is drawn over the older (``over``: a horizontal crossing a lane)."""
    cell = (x, y)
    if cell in drawing.bits and over and edge not in drawing.edges_at[cell]:
        drawing.bits[cell] = bits if drawing.bits[cell] & (N | S) and not drawing.bits[cell] & (E | W) else (
            drawing.bits[cell] | bits)
    else:
        drawing.bits[cell] = drawing.bits.get(cell, 0) | bits
    drawing.edges_at.setdefault(cell, set()).add(edge)


def _vline(drawing: Drawing, x: int, y0: int, y1: int, edge: int):
    lo, hi = min(y0, y1), max(y0, y1)
    for y in range(lo, hi + 1):
        bits = (N if y > lo else 0) | (S if y < hi else 0)
        if bits:
            _put(drawing, x, y, bits, edge, over=False)


def _hline(drawing: Drawing, y: int, x0: int, x1: int, edge: int):
    lo, hi = min(x0, x1), max(x0, x1)
    for x in range(lo, hi + 1):
        bits = (W if x > lo else 0) | (E if x < hi else 0)
        if bits:
            _put(drawing, x, y, bits, edge, over=True)


def _box(drawing: Drawing, el: _Element, node: GraphNode, _connected: bool, sends: bool):
    x0, y0, w, h = el.x, el.y, el.width, el.height
    drawing.boxes[el.id] = Region(x0, y0, w, h)
    title = _cut(node.title, w - 4)
    top = "┌─" + title + "─" * (w - 3 - len(title)) + "┐"
    for n, ch in enumerate(top):
        drawing.glyphs[(x0 + n, y0)] = (ch, "title" if 2 <= n < 2 + len(title) else "border", el.id)
    for n, ch in enumerate("└" + "─" * (w - 2) + "┘"):
        drawing.glyphs[(x0 + n, y0 + h - 1)] = (ch, "border", el.id)
    for row in range(1, h - 1):
        y = y0 + row
        drawing.glyphs[(x0, y)] = ("┤" if y in el.in_rows.values() else "│", "border", el.id)
        drawing.glyphs[(x0 + w - 1, y)] = ("├" if sends and y == el.out_row else "│", "border", el.id)
        text = _cut(node.lines[row - 1], w - 4) if row - 1 < len(node.lines) else ""
        for n in range(w - 2):
            ch = text[n - 1] if 1 <= n <= len(text) else " "
            drawing.glyphs[(x0 + 1 + n, y)] = (ch, "text", el.id)


def _cut(text: str, width: int) -> str:
    """At most ``width`` characters; a longer text keeps its end (a key's last tokens say most)."""
    return text if len(text) <= width else "…" + text[len(text) - width + 1:]


# ------------------------------------------------------------------------------------------------------ the widget


class GraphView(Widget, can_focus=True):
    """The drawing as a widget of its size (in a scrolling container): arrow keys and clicks select a box, the mouse
    names what it is over. ``Selected`` and ``Hovered`` tell the parent."""

    DEFAULT_CSS = """
    GraphView { width: auto; height: auto; }
    """
    BINDINGS: ClassVar = [Binding("up", "move(0, -1)", "Up", show=False),
                          Binding("down", "move(0, 1)", "Down", show=False),
                          Binding("left", "move(-1, 0)", "Left", show=False),
                          Binding("right", "move(1, 0)", "Right", show=False)]
    selected: reactive[str | None] = reactive(None)
    hovered: reactive[tuple[str, object] | None] = reactive(None)  # ("node", id) or ("edges", {edge numbers})

    class Selected(Message):
        def __init__(self, node: str | None):
            super().__init__()
            self.node = node

    class Hovered(Message):
        def __init__(self, what: tuple[str, object] | None):
            super().__init__()
            self.what = what

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.nodes: list[GraphNode] = []
        self.edges: list[GraphEdge] = []
        self.drawing = Drawing()
        self._signature = None

    def show(self, nodes: list[GraphNode], edges: list[GraphEdge]):
        """Draws ``nodes`` and ``edges`` anew if they changed; the selection stays while its box does."""
        signature = (tuple((n.id, n.title, tuple(n.lines)) for n in nodes),
                     tuple((e.source, e.target, e.label) for e in edges))
        if signature == self._signature:
            return
        self._signature = signature
        self.nodes, self.edges = nodes, edges
        self.drawing = layout(nodes, edges)
        if self.selected not in self.drawing.boxes:
            self.selected = next(iter(sorted(self.drawing.boxes, key=self._place)), None)
        if self.hovered is not None and self.hovered[0] == "node" and self.hovered[1] not in self.drawing.boxes:
            self.hovered = None
        elif self.hovered is not None and self.hovered[0] == "edges":
            self.hovered = None  # edges are numbered anew
        self.refresh(layout=True)

    def _place(self, node: str) -> tuple[int, int]:
        region = self.drawing.boxes[node]
        return region.x, region.y

    def get_content_width(self, container: Size, viewport: Size) -> int:
        return max(self.drawing.width, 1)

    def get_content_height(self, container: Size, viewport: Size, width: int) -> int:
        return max(self.drawing.height, 1)

    def watch_selected(self, node: str | None):
        if node is not None and node in self.drawing.boxes:
            self.scroll_visible_region(self.drawing.boxes[node])
        self.post_message(self.Selected(node))
        self.refresh()

    def watch_hovered(self, what):
        self.post_message(self.Hovered(what))
        self.refresh()

    def scroll_visible_region(self, region: Region):
        parent = self.parent
        if parent is not None and hasattr(parent, "scroll_to_region"):
            parent.scroll_to_region(region.translate(self.region.offset - parent.region.offset + parent.scroll_offset),
                                    animate=False)

    def action_move(self, dx: int, dy: int):
        """The next box up or down in the column, or the nearest in the next column left or right that has one."""
        boxes = self.drawing.boxes
        if not boxes:
            return
        if self.selected not in boxes:
            self.selected = next(iter(sorted(boxes, key=self._place)))
            return
        here = boxes[self.selected]
        column = self.drawing.layer_of[self.selected]
        if dy:
            same = sorted((b for b in boxes if self.drawing.layer_of[b] == column), key=lambda b: boxes[b].y)
            n = same.index(self.selected) + dy
            if 0 <= n < len(same):
                self.selected = same[n]
            return
        layers = sorted({self.drawing.layer_of[b] for b in boxes})
        n = layers.index(column) + dx
        if 0 <= n < len(layers):
            there = [b for b in boxes if self.drawing.layer_of[b] == layers[n]]
            self.selected = min(there, key=lambda b: abs(boxes[b].y + boxes[b].height // 2
                                                        - here.y - here.height // 2))

    def _at(self, x: int, y: int) -> tuple[str, object] | None:
        for node, region in self.drawing.boxes.items():
            if region.contains(x, y):
                return ("node", node)
        glyph = self.drawing.glyphs.get((x, y))
        if glyph is not None and glyph[1] in ("arrow", "label"):
            return ("edges", frozenset({glyph[2]}))
        edges = self.drawing.edges_at.get((x, y))
        return ("edges", frozenset(edges)) if edges else None

    def on_mouse_move(self, event: MouseMove):
        what = self._at(event.x, event.y)
        if what != self.hovered:
            self.hovered = what

    def on_leave(self, event: Leave):
        self.hovered = None

    def on_click(self, event: Click):
        what = self._at(event.x, event.y)
        if what is not None and what[0] == "node":
            self.selected = what[1]
        self.focus()

    def render(self) -> Text:
        d = self.drawing
        sel = self.selected
        hovered_edges = self.hovered[1] if self.hovered and self.hovered[0] == "edges" else frozenset()
        hovered_node = self.hovered[1] if self.hovered and self.hovered[0] == "node" else None
        out_edges = {k for k, e in enumerate(self.edges) if e.source == sel}
        in_edges = {k for k, e in enumerate(self.edges) if e.target == sel}

        def edge_style(edges) -> Style:
            if edges & hovered_edges:
                return Style(color="yellow", bold=True)
            if edges & out_edges:
                return Style(color="green", bold=True)
            if edges & in_edges:
                return Style(color="cyan", bold=True)
            return Style(dim=True) if sel is not None else Style()

        text = Text(no_wrap=True, end="")
        for y in range(d.height):
            row = Text(no_wrap=True, end="")
            for x in range(d.width):
                glyph = d.glyphs.get((x, y))
                if glyph is not None:
                    ch, kind, owner = glyph
                    if kind in ("arrow", "label"):
                        style = edge_style({owner})
                        if kind == "label" and not ({owner} & (hovered_edges | out_edges | in_edges)):
                            style = Style(dim=True, italic=True)
                    elif owner == sel:
                        style = Style(color="magenta", bold=kind != "text")
                    elif owner == hovered_node:
                        style = Style(color="yellow", bold=kind == "title")
                    else:
                        style = Style(bold=True) if kind == "title" else Style(dim=kind == "border")
                    row.append(ch, style)
                elif (x, y) in d.bits:
                    row.append(LINES.get(d.bits[(x, y)], " "), edge_style(d.edges_at[(x, y)]))
                else:
                    row.append(" ")
            text.append_text(row)
            if y < d.height - 1:
                text.append("\n")
        return text
