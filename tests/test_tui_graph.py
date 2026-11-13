"""The terminal interface's drawn graph (tui_graph.py), its key tree and short names (tui.py), and the endpoints
people type (session.endpoint): no network, only the functions."""

import pytest

from xblick import tui
from xblick.session import endpoint
from xblick.tui_graph import LINES, GraphEdge, GraphNode, GraphView, layout


@pytest.mark.parametrize("typed, wanted", [
    ("robot-pc", "tcp/robot-pc:7447"), ("robot-pc:7448", "tcp/robot-pc:7448"),
    ("tcp/robot-pc", "tcp/robot-pc:7447"), ("tcp/192.0.2.10:7447", "tcp/192.0.2.10:7447"),
    ("udp/192.0.2.10", "udp/192.0.2.10:7447"), ("[::1]", "tcp/[::1]:7447"), ("tcp/[::1]:7448", "tcp/[::1]:7448"),
    ("unixsock-stream/tmp/z.sock", "unixsock-stream/tmp/z.sock"), (" robot-pc ", "tcp/robot-pc:7447"),
    ("tcp/h#iface=eth0", "tcp/h:7447#iface=eth0"), ("127.0.0.1:0", "tcp/127.0.0.1:0")])
def test_endpoints_take_tcp_and_the_default_port(typed, wanted):
    assert endpoint(typed) == wanted


@pytest.mark.parametrize("typed", ["", "::1", "tcp/host:", "tcp/"])
def test_endpoints_that_say_too_little_are_refused(typed):
    with pytest.raises(ValueError):
        endpoint(typed)


def test_a_connection_is_shown_with_its_endpoints_as_zenoh_takes_them():
    c = tui.Connection(connect=["robot-pc"], mode="client")
    assert c.connect == ["tcp/robot-pc:7447"] and c.describe() == "--zenoh-mode client --connect tcp/robot-pc:7447"


HOST = "0c9a1f6e-5b0d-4a52-9d3e-7f1b2c3d4e5f"  # a host id, as the rings' keys carry it
KEYS = ["robot/camera/color/camera_info", "robot/camera/color/image_raw/compressed",
        f"robot/camera/color/image_raw/shm/{HOST}", f"robot/camera/depth/image_raw/shm/{HOST}",
        "robot/camera/diagnostics", "robot/camera/sam3/instances", "robot/joint_states",
        "robot/camera/color/image_raw", "sim/clock"]


def test_the_keys_as_a_tree():
    camera, sam3 = frozenset({"k"}), frozenset({"s"})
    cells = {k: (sam3 if "sam3" in k or k.endswith("diagnostics") else camera, frozenset()) for k in KEYS}
    cells["robot/joint_states"] = (frozenset({"b"}), frozenset())
    rows = {r.path: r for r in tui.topic_tree(KEYS, cells=cells)}
    labels = [r.label for r in rows.values()]
    assert labels[:3] == ["▾ robot/", "├─ ▾ camera/", "│  ├─ ▾ color/"]
    # a chain of single children: one row; a ring's place (its host's id) by its first 8 characters
    assert "│  ├─ depth/image_raw/shm/0c9a1f6e" in labels
    assert rows["robot/camera/depth/image_raw/shm/0c9a1f6e"].key == KEYS[3]
    assert "sim/clock" in labels
    # a key that is also a prefix: one row, the key's
    assert rows["robot/camera/color/image_raw/"].key == "robot/camera/color/image_raw"
    # the publisher once: on color/ (every key below is the camera's), not on its keys, not on camera/ (mixed)
    assert rows["robot/camera/color/"].show_pub and not rows["robot/camera/color/camera_info"].show_pub
    assert not rows["robot/camera/"].show_pub and rows["robot/camera/sam3/instances"].show_pub
    folded = [r.label for r in tui.topic_tree(KEYS, {"robot/camera/color/"})]
    assert "│  ├─ ▸ color/  (4 keys)" in folded and not any("camera_info" in label for label in folded)


def test_short_names_drop_what_another_session_names_already():
    names = tui.short_names({"a": "robot/camera", "b": "robot/camera/sam3", "r": "robot",
                             "t": "robot/camera/tapnext/tracks", "x": "xblick/recorder/status",
                             "v": "subscribes robot/camera", "c": "robot2/camera/sam3",
                             "d": "robot2/camera"})
    assert names == {"a": "camera", "b": "robot/camera/sam3", "r": "robot", "t": "tapnext/tracks",
                     "x": "xblick/recorder/status", "v": "subscribes camera",
                     "c": "robot2/camera/sam3", "d": "robot2/camera"}  # two sam3: both whole


NODES = [GraphNode("k", "camera", ["peer 1", "OK 30.0 frames/s"]), GraphNode("s", "sam3", ["peer 2"]),
         GraphNode("f", "foundationpose", ["peer 3"]), GraphNode("t", "tapnext", ["peer 4"]),
         GraphNode("v", "viewer", ["client 5"]), GraphNode("lonely", "relay", ["peer 6"])]
EDGES = [GraphEdge("k", "s", "shm"), GraphEdge("k", "f", "cuda"), GraphEdge("s", "f", "instances"),
         GraphEdge("k", "v", "tcp-compressed"), GraphEdge("f", "v", "tf"), GraphEdge("s", "t", "instances"),
         GraphEdge("t", "s", "tracks")]


def test_the_layout_puts_takers_right_of_what_they_take():
    d = layout(NODES, EDGES)
    x = {n: d.boxes[n].x for n in d.boxes}
    assert x["k"] < x["s"] < x["f"] < x["v"] and set(d.boxes) == {n.id for n in NODES}
    # one cycle (s and t take each other's data): one edge drawn against the flow
    assert len(d.reversed) == 1 and EDGES[next(iter(d.reversed))].source in ("s", "t")
    for a in d.boxes.values():  # boxes do not overlap
        for b in d.boxes.values():
            assert a is b or not a.overlaps(b)


def test_every_edge_has_its_arrow_and_label_and_no_line_runs_into_a_box():
    d = layout(NODES, EDGES)
    arrows = {owner for (ch, kind, owner) in d.glyphs.values() if kind == "arrow"}
    assert arrows == set(range(len(EDGES)))
    assert sum(ch == "◀" for ch, kind, _ in d.glyphs.values() if kind == "arrow") == 1
    text = GraphView()
    text.nodes, text.edges, text.drawing = NODES, EDGES, d
    plain = text.render().plain
    for label in ("shm▶", "cuda▶", "tcp-compressed▶", "tf▶"):
        assert label in plain.replace("─", ""), label
    for (x, y) in d.bits:
        assert not any(r.contains(x, y) for r in d.boxes.values()), (x, y)
    # every line cell is a box-drawing character of its directions
    assert all(LINES.get(bits) for bits in d.bits.values())


def test_hovering_an_edge_and_moving_the_selection():
    view = GraphView()
    view.nodes, view.edges, view.drawing = NODES, EDGES, layout(NODES, EDGES)
    arrow = next(cell for cell, (ch, kind, owner) in view.drawing.glyphs.items() if kind == "arrow" and owner == 0)
    assert view._at(*arrow) == ("edges", frozenset({0}))
    box = view.drawing.boxes["s"]
    assert view._at(box.x + 1, box.y + 1) == ("node", "s")
    view.selected = "k"
    view.action_move(1, 0)  # right: the nearest box in the next column
    assert view.selected == "s"
    view.action_move(-1, 0)
    assert view.selected == "k"
