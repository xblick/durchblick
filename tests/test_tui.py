"""The terminal interface (tui.py) and what it looks with (survey.py): a router session in the test process (as
zenohd: admin space on), fake publishers of a camera, a node's diagnostics, a node subscribing to the camera, and the
app headless (``App.run_test``)."""

import asyncio
import io
import json
import re
import shlex
import subprocess
import sys
import threading
import time
import types
from importlib.metadata import entry_points
from pathlib import Path

import numpy as np
import pytest
import zenoh
from rich.console import Console
from test_camera import free_port, pattern, wait_until

from blick import cli, survey, topics, tui
from blick.camera import CameraPublisher
from blick.diagnostics import DiagnosticsPublisher
from blick.msg import CameraInfo
from blick.session import close as close_session
from blick.session import Session, open_session

CAMERA = "robot/fakecam"
BASE = f"{CAMERA}/color"
HOST = "0c9a1f6e-5b0d-4a52-9d3e-7f1b2c3d4e5f"  # a host id, as the rings' keys carry it
GPU = "7" * 32  # a GPU's UUID


def open_router(port: int) -> zenoh.Session:
    """A Zenoh router as zenohd runs one: admin space on (zenohd enables it, a library session does not)."""
    config = zenoh.Config()
    config.insert_json5("mode", '"router"')
    config.insert_json5("listen/endpoints", json.dumps([f"tcp/127.0.0.1:{port}"]))
    config.insert_json5("scouting/multicast/enabled", "false")
    config.insert_json5("adminspace/enabled", "true")
    return Session(zenoh.open(config))


class FakeNetwork:
    """A router, a camera publishing frames and its diagnostics, and a node ("tracker") that subscribes to the
    camera's compressed images and reports on the camera's diagnostics topic; each its own session, as programs."""

    def __init__(self):
        self.port = free_port()
        self.router = open_router(self.port)
        self.endpoint = f"tcp/127.0.0.1:{self.port}"
        self.camera_session = open_session(connect=[self.endpoint], listen=["127.0.0.1:0"])
        self.node_session = open_session(connect=[self.endpoint], listen=["127.0.0.1:0"])
        info = CameraInfo.pinhole(64, 48, 50.0, 50.0, 31.5, 23.5, frame_id="fake_color")
        self.camera = CameraPublisher(self.camera_session, BASE, encoding="rgb8", frame_id="fake_color",
                                      camera_info=info, source="test")
        self.camera_diagnostics = DiagnosticsPublisher(self.camera_session, CAMERA, "fakecam")
        self.node_diagnostics = DiagnosticsPublisher(self.node_session, CAMERA, "tracker")
        self.node_results = self.node_session.declare_publisher(f"{CAMERA}/tracker/tracks")
        self.node_boxes = self.node_session.declare_publisher(f"{CAMERA}/tracker/boxes")  # declared, nothing sent
        self.node_subscriber = self.node_session.declare_subscriber(f"{BASE}/image_raw/compressed", lambda s: None)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._publish, daemon=True)
        self._thread.start()

    def connection(self) -> tui.Connection:
        return tui.Connection(connect=[self.endpoint], listen=["127.0.0.1:0"])

    def _publish(self):
        seq = 0
        while not self._stop.is_set():
            seq += 1
            self.camera.publish(pattern(seq))
            if seq % 5 == 0:
                self.camera_diagnostics.publish(DiagnosticsPublisher.OK, "20 frames/s", {"frames/s": 20.0})
                self.node_diagnostics.publish(DiagnosticsPublisher.WARN, "waiting: frames", {"tracks": 0})
                self.node_results.put(json.dumps({"v": 1, "stamp_ns": time.time_ns(), "tracks": []}),
                                      encoding=zenoh.Encoding.APPLICATION_JSON)
            time.sleep(0.05)

    def close(self):
        self._stop.set()
        self._thread.join()
        self.node_subscriber.undeclare()
        self.node_results.undeclare()
        self.node_boxes.undeclare()
        self.node_diagnostics.close()
        self.camera_diagnostics.close()
        self.camera.close()
        for session in (self.node_session, self.camera_session, self.router):
            close_session(session)


@pytest.fixture
def fake():
    network = FakeNetwork()
    yield network
    network.close()


# ------------------------------------------------------------------------------------------------------ survey


def test_admin_space_names_the_sessions_and_connects_them(fake):
    with open_session(connect=[fake.endpoint], listen=["127.0.0.1:0"]) as z:
        def complete():
            n = survey.query_network(z, 1.0)
            return n if any(c[1] == f"{BASE}/image_raw/compressed" for c in n.connections()) else None

        wait_until(complete, timeout=10)
        network = complete()
        camera_zid, node_zid = str(fake.camera_session.info.zid()), str(fake.node_session.info.zid())
        assert list(network.routers) == [str(fake.router.info.zid())]
        assert network.nodes[camera_zid].label == BASE  # its stream's keys; the shared diagnostics left out
        assert network.nodes[node_zid].label == f"{CAMERA}/tracker"  # its results
        assert network.nodes[camera_zid].whatami == "peer" and network.nodes[camera_zid].addresses
        assert f"{BASE}/info" in network.nodes[camera_zid].queryables
        assert (camera_zid, f"{BASE}/image_raw/compressed", node_zid, f"{BASE}/image_raw/compressed") in \
            network.connections()
        assert network.publishers()[f"{CAMERA}/diagnostics"] == {camera_zid, node_zid}


def test_without_a_router_nothing_of_the_admin_space_answers(fake):
    port = free_port()
    with open_session(listen=[f"127.0.0.1:{port}"]) as a, open_session(connect=[f"127.0.0.1:{port}"]):
        network = survey.query_network(a, 0.5)
        assert network.routers == {} and network.nodes == {}


def test_traffic_meter_counts_and_shows_content(fake):
    with open_session(connect=[fake.endpoint], listen=["127.0.0.1:0"]) as z:
        meter = survey.TrafficMeter(z, f"{BASE}/camera_info")
        meter.keep.add(f"{BASE}/camera_info")
        wait_until(lambda: meter.snapshot().get(f"{BASE}/camera_info", survey.Traffic()).count >= 3, timeout=10)
        t = meter.snapshot()[f"{BASE}/camera_info"]
        assert t.encoding == CameraInfo.ENCODING and t.stamp_ns and t.bytes > 0
        message = survey.content(*meter.last(f"{BASE}/camera_info"))
        assert message["width"] == 64 and message["header"]["frame_id"] == "fake_color"
        meter.close()


def test_content_of_json_unknown_types_and_raw_bytes():
    assert survey.content("application/json", b'{"a": 1}') == {"a": 1}
    assert survey.content("text/plain", b"abc") == "3 bytes, text/plain"
    with pytest.raises(ValueError, match="does not know"):
        survey.content("application/cdr;foo_msgs/msg/Bar", b"\0\1\0\0")
    assert survey._plain(np.zeros((100, 3)), 16) == "array (100, 3) float64"
    assert survey._plain(list(range(20)), 4) == [0, 1, 2, 3, "... 20 in all"]


def test_uses_say_how_a_representation_is_taken():
    """From the declarations and the routers' links: rings by their key, Zenoh by the session's link protocol; what
    the admin space cannot show (links between peers, a session without a known link) is said, not guessed."""
    net = survey.Network()
    router = net.node("r" * 32)
    router.whatami = "router"
    camera = survey.Node("c" * 32, "peer", {"tcp/127.0.0.1:5001"}, {router.zid}, {f"{BASE}/image_raw/compressed",
                         f"{BASE}/image_raw/shm/{HOST}", f"{BASE}/image_raw", f"{BASE}/camera_info"})
    local = survey.Node("a" * 32, "peer", {"tcp/127.0.0.1:5002"}, {router.zid}, subscribes={f"{BASE}/image_raw/shm/{HOST}"})
    laptop = survey.Node("b" * 32, "client", {"tcp/192.0.2.9:6000"}, {router.zid},
                         subscribes={f"{BASE}/image_raw/compressed"})
    peer = survey.Node("d" * 32, "peer", {"tcp/127.0.0.1:5003"}, {router.zid}, subscribes={f"{CAMERA}/**"})
    lost = survey.Node("e" * 32, "peer", subscribes={f"{BASE}/image_raw"})
    for n in (camera, local, laptop, peer, lost):
        net.nodes[n.zid] = n
    assert survey.representation(f"{BASE}/image_raw/compressed") == "compressed"
    assert survey.representation(f"{BASE}/image_raw") == "raw" and survey.representation(f"{BASE}/camera_info") is None
    assert survey.representation(f"robot/x/points/cloud/cuda/{HOST}/{GPU}") == "cuda"
    assert survey.representation(f"{BASE}/image_raw/shm/{HOST}") == "shm"
    assert topics.ring_place(f"{BASE}/image_raw/cuda/{HOST}/{GPU}") == (f"{BASE}/image_raw/cuda", f"{HOST}/{GPU}")
    assert topics.ring_place(f"{BASE}/image_raw/compressed") is None and topics.ring_place(f"{BASE}/shm") is None
    shm = net.uses(f"{BASE}/image_raw/shm/{HOST}")
    assert [(u.consumer, u.summary) for u in shm] == [(local.zid, "shm"), (peer.zid, "shm")]
    [client] = net.uses(f"{BASE}/image_raw/compressed", leave_out=[peer.zid])
    assert client.summary == "tcp-compressed" and "a client: through its router rrrrrrrr" in client.detail
    assert f"at another address than {BASE}" in client.detail
    raw = {u.consumer: u for u in net.uses(f"{BASE}/image_raw")}
    assert raw[peer.zid].summary == "tcp-raw" and "links between peers are not in the admin space" in \
        raw[peer.zid].detail and f"on the host of {BASE}" in raw[peer.zid].detail
    assert raw[lost.zid].summary == "unknown-raw" and "know no link" in raw[lost.zid].detail
    assert net.uses(f"{BASE}/camera_info") == []
    assert tui._count(shm + [client]) == "shm×2, tcp-compressed×1"


def test_common_prefix():
    assert survey.common_prefix(["a/b/c", "a/b/d/e"]) == "a/b"
    assert survey.common_prefix(["a/b"]) == "a/b"
    assert survey.common_prefix(["x/b", "y/b"]) == ""
    assert survey.common_prefix(["a/**", "a/b"]) == "a"


# --------------------------------------------------------------------------------------- rerun's selection


@pytest.fixture
def zenoh_config(tmp_path) -> Path:
    """A Zenoh configuration file: a client of another PC's router."""
    path = tmp_path / "robot-pc.json5"
    path.write_text('{ mode: "client", connect: { endpoints: ["tcp/robot-pc:7447"] }, '
                    'scouting: { multicast: { enabled: false } } }\n')
    return path


def robot_preset(config: Path) -> tui.RerunSelection:
    camera = "robot2/camera"
    return tui.RerunSelection(bases=[f"{camera}/color_rect", f"{camera}/depth_to_color_rect"],
                              clouds=[f"{camera}/nvblox/surface", f"{camera}/points"],
                              detections=[f"{camera}/boxer/detections_2d", f"{camera}/boxer/boxes_3d",
                                          f"{camera}/sam3/detections_2d"],
                              segmentations=[f"{camera}/sam3/instances"],
                              connection=tui.Connection(config=str(config)))


def comparable(argv: list[str]) -> dict:
    args = vars(cli.parse(["rerun", *argv]))
    args.pop("run")
    for key in ("streams", "points", "detections", "segmentation"):
        args[key] = sorted(args[key] or [])
    return args


def test_a_preset_runs_rerun_with_its_selection(zenoh_config):
    camera = "robot2/camera"
    command = (f"--spawn --config {zenoh_config} {camera}/color_rect {camera}/depth_to_color_rect "
               f"--detections {camera}/boxer/detections_2d --detections {camera}/boxer/boxes_3d "
               f"--detections {camera}/sam3/detections_2d --segmentation {camera}/sam3/instances "
               f"--points {camera}/nvblox/surface --points {camera}/points")
    assert comparable(robot_preset(zenoh_config).check()) == comparable(shlex.split(command))


def test_selection_arguments_and_their_check(zenoh_config):
    s = tui.RerunSelection(bases=["robot/x/color"], stale_after=5.0, cloud_rate=2.0, viewer="serve",
                           viewer_value="9877", extra="--report 2", connection=tui.Connection(connect=["h1", "h2"], mode="client"))
    assert s.check() == ["--serve", "9877", "robot/x/color", "--stale-after", "5", "--cloud-rate", "2",
                         "--report", "2", "--zenoh-mode", "client", "--connect", "tcp/h1:7447", "tcp/h2:7447"]
    with pytest.raises(ValueError, match="a peer that only connects receives nothing"):
        tui.Connection(connect=["h1"])  # the router does not route between peers that do not listen
    with pytest.raises(ValueError, match="nothing to listen on"):
        tui.Connection(connect=["h1"], listen=["127.0.0.1:0"], mode="client")
    assert tui.RerunSelection().check() == ["--spawn"]  # nothing chosen: rerun takes every stream
    with pytest.raises(ValueError, match="needs a file"):
        tui.RerunSelection(viewer="save").check()
    with pytest.raises(ValueError, match="invalid float"):
        tui.RerunSelection(extra="--cloud-rate fast").check()
    with pytest.raises(FileNotFoundError, match="no such file"):
        tui.Connection(config="/nonexistent/zenoh.json5")
    with pytest.raises(ValueError, match="not both"):
        tui.Connection(connect=["h"], config=str(zenoh_config))


def test_pose_topics_for_trajectories():
    published = {"robot2/tf": {"bridge"}, "robot2/odom": {"bridge"}, "robot2/camera/tf": {"bridge"},
                 "robot2/camera/tf_static": {"camera"}, "robot2/camera/vslam/tf": {"vslam"},
                 "robot2/camera/vslam/odom": {"vslam"}}
    network = types.SimpleNamespace(publishers=lambda: published)
    assert tui.pose_topics(network) == ["robot2/camera/tf", "robot2/camera/vslam/tf"]
    assert tui.pose_topics(None) == []
    selection = tui.RerunSelection(bases=["robot2/camera/color"], trajectories=tui.pose_topics(network))
    args = selection.check()  # the command's own parser takes them
    assert args[args.index("--trajectory") + 1] == "robot2/camera/tf"
    assert tui.RerunSelection.from_dict(selection.to_dict()).trajectories == selection.trajectories


def test_presets_save_load_delete(tmp_path, zenoh_config):
    presets = tui.Presets(tmp_path / "presets.json")
    assert presets.load() == {}
    presets.save("lab", robot_preset(zenoh_config))
    presets.save("local", tui.RerunSelection(bases=["robot/x/color"]))
    loaded = presets.load()
    assert sorted(loaded) == ["lab", "local"] and loaded["lab"] == robot_preset(zenoh_config)
    presets.delete("local")
    assert list(presets.load()) == ["lab"]
    with pytest.raises(ValueError, match="a name"):
        presets.save(" ", tui.RerunSelection())
    (tmp_path / "presets.json").write_text('{"v": 99, "presets": {}}')
    with pytest.raises(ValueError, match="version 1"):
        presets.load()
    with pytest.raises(ValueError, match="version 1"):  # a file it does not understand is not overwritten
        presets.save("x", tui.RerunSelection())
    assert json.loads((tmp_path / "presets.json").read_text())["v"] == 99


# ---------------------------------------------------------------------------------------------- the app


async def until(pilot, condition, timeout=15.0):
    end = time.monotonic() + timeout
    while not condition():
        if time.monotonic() > end:
            raise TimeoutError
        await pilot.pause(0.1)


def text_of(app, widget_id: str) -> str:
    """What a Static shows, as plain text."""
    console = Console(width=200, record=True, file=io.StringIO())
    console.print(app.query_one(widget_id).content)
    return console.export_text()


def cells(app, table_id: str) -> list[str]:
    table = app.query_one(table_id)
    return [" | ".join(str(c) for c in table.get_row(key)) for key in table.rows]


def topic_row(app, key: str) -> str:
    """The Topics tab's row of a key (its cells joined with " | "), empty while it has none."""
    table = app.query_one("#topic-table")
    row = next((r for r in app._topic_rows.values() if r.key == key), None)
    if row is None or row.path not in {k.value for k in table.rows}:
        return ""
    return " | ".join(str(c) for c in table.get_row(row.path))


def run_app(app, test, size=(160, 50)):
    async def main():
        async with app.run_test(size=size) as pilot:
            await test(pilot)

    asyncio.run(main())


def test_app_shows_topics_nodes_reports_and_graph(fake, tmp_path):
    app = tui.DurchblickApp(fake.connection(), tui.Presets(tmp_path / "presets.json"), discover_every=0.5)

    async def test(pilot):
        await until(pilot, lambda: app.discovery.network is not None and len(app.discovery.network.routers) == 1)
        await until(pilot, lambda: topic_row(app, f"{BASE}/camera_info"))
        # a tree: the camera's prefix once (the tracker publishes below it too: no publisher on it), the stream's
        # prefix with its publisher, which its keys do not repeat
        rows = cells(app, "#topic-table")
        assert rows[0].startswith(f"▾ {CAMERA}/ |  |  |"), rows[0]
        stream = next(r for r in rows if "▾ color/" in r)
        assert f"| {BASE} |" in stream, stream  # the camera session, named after its keys
        assert topic_row(app, f"{BASE}/camera_info").split(" | ")[2] == ""
        await until(pilot, lambda: any("tracker" in row and "WARN" in row for row in cells(app, "#report-table")))
        assert any("fakecam" in row and "OK" in row for row in cells(app, "#report-table"))
        # each report names its session (the diagnostics attachment's zid)
        await until(pilot, lambda: any(r.startswith(f"{CAMERA}/diagnostics | tracker |") and f"| {CAMERA}/tracker ("
                                       in r for r in cells(app, "#report-table")))
        app._selected_node = str(fake.node_session.info.zid())
        app._show_node_detail()
        node_detail = text_of(app, "#node-detail")
        assert f"{CAMERA}/diagnostics tracker WARN: waiting: frames" in node_detail and "fakecam OK" not in node_detail
        rows = cells(app, "#node-table")
        assert any(row.startswith(f"{CAMERA}/tracker |") for row in rows)
        assert any(row.startswith("this interface |") for row in rows)
        assert any("router" in row for row in rows)
        status = text_of(app, "#status")
        assert "1 router(s)" in status and "1 stream(s)" in status

        # how the camera's representations are taken: the tracker's compressed images over its TCP link
        await until(pilot, lambda: "| tcp-compressed×1 |" in topic_row(app, f"{BASE}/image_raw/compressed"))
        assert f"| {CAMERA}/tracker |" in topic_row(app, f"{BASE}/image_raw/compressed")  # its subscriber

        # a topic's detail: its publisher, its subscriber, its decoded message
        app._select_topic(f"{BASE}/camera_info")
        await until(pilot, lambda: "width" in text_of(app, "#topic-detail"))
        detail = text_of(app, "#topic-detail")
        assert f"published by: {CAMERA}" in detail and "fake_color" in detail
        await until(pilot, lambda: "age ms at send / at receive" in text_of(app, "#topic-detail"))
        assert "offers raw, compressed, shm; in use: tcp-compressed×1" in detail
        app._select_topic(f"{BASE}/image_raw/compressed")
        await until(pilot, lambda: "taken by:" in text_of(app, "#topic-detail"))
        assert f"{CAMERA}/tracker ({BASE}/image_raw/compressed): tcp-compressed, a peer" in \
            text_of(app, "#topic-detail").replace("\n", " ")
        await pilot.press("escape")
        assert app.detail is None

        # traffic of every topic
        await pilot.press("m")
        await until(pilot, lambda: app._rates.get(f"{BASE}/camera_info", (0, 0))[0] > 5)
        assert "sensor_msgs/msg/CameraInfo" in topic_row(app, f"{BASE}/camera_info")
        # ages at send and at receive, median / p95 in ms (the fake camera publishes as it stamps: about 0 at send)
        await until(pilot, lambda: app._ages.get(f"{BASE}/camera_info", (None, None))[0] is not None)
        send, receive = topic_row(app, f"{BASE}/camera_info").split(" | ")[6:8]
        assert re.fullmatch(r"\d+(\.\d)? / \d+(\.\d)?", send) and re.fullmatch(r"\d+(\.\d)? / \d+(\.\d)?", receive)
        assert app._ages[f"{BASE}/camera_info"][0][1] < 50  # p95 at send, ms
        await pilot.press("m")
        assert app.meter is None

        # the graph: the camera's compressed images go to the tracker
        graph = app.query_one("#graph")
        camera, node = str(fake.camera_session.info.zid()), str(fake.node_session.info.zid())
        await until(pilot, lambda: any(e.source == camera and e.target == node for e in graph.edges))
        edge = next(k for k, e in enumerate(graph.edges) if e.source == camera and e.target == node)
        assert graph.edges[edge].label == "tcp-compressed"
        drawn = graph.render().plain
        assert CAMERA in drawn and "tcp-compressed▶" in drawn.replace("─", "")
        # the mouse over the edge names its key and how it is taken; a selected session what it sends and takes
        graph.hovered = ("edges", frozenset({edge}))
        await pilot.pause(0.2)
        assert f"{BASE}/image_raw/compressed  [tcp-compressed]" in text_of(app, "#graph-detail")
        graph.hovered = None
        graph.selected = node
        await pilot.pause(0.2)
        assert f"takes from {BASE}: {BASE}/image_raw/compressed [tcp-compressed]" in text_of(app, "#graph-detail")

    run_app(app, test)


def test_app_reconnects_and_shows_a_failed_connection(fake, tmp_path):
    app = tui.DurchblickApp(fake.connection(), tui.Presets(tmp_path / "presets.json"), discover_every=0.5)

    async def test(pilot):
        await until(pilot, lambda: app.discovery.network is not None and app.discovery.network.routers)
        first = app.session
        # connected to nobody (a port without a router): no admin space, said so
        lonely = tui.Connection(listen=[f"127.0.0.1:{free_port()}"])
        app._open(lonely)
        await until(pilot, lambda: app.session is not None and app.session is not first
                    and app.discovery.network is not None)
        assert "no router answered" in text_of(app, "#status")
        # back through the connect screen
        await pilot.press("c")
        screen = app.screen
        assert isinstance(screen, tui.ConnectScreen)
        screen.query_one("#connect-mode").query("RadioButton")[1].value = True
        screen.query_one("#connect-endpoints").value = fake.endpoint
        screen.query_one("#listen").value = "tcp/127.0.0.1:0"
        await pilot.click("#connect-ok")
        await until(pilot, lambda: app.connection.connect == [fake.endpoint] and app.discovery.network is not None
                    and app.discovery.network.routers)
        # a config file that is not there: the dialog says so and stays
        await pilot.press("c")
        screen = app.screen
        screen.query_one("#connect-mode").query("RadioButton")[2].value = True
        screen.query_one("#config").value = str(tmp_path / "missing.json5")
        await pilot.click("#connect-ok")
        assert isinstance(app.screen, tui.ConnectScreen)
        assert "no such file" in text_of(screen, "#connect-error")
        await pilot.press("escape")

    run_app(app, test)


def test_app_rerun_tab_presets_and_launch(fake, tmp_path):
    pytest.importorskip("rerun")
    app = tui.DurchblickApp(fake.connection(), tui.Presets(tmp_path / "presets.json"), discover_every=0.5)
    recording = tmp_path / "fake.rrd"

    async def test(pilot):
        await until(pilot, lambda: app.discovery.streams)
        streams = app.query_one("#rerun-streams")
        await until(pilot, lambda: streams.option_count == 1)
        streams.select(f"bases\t{BASE}")
        app.query_one("#rerun-viewer").value = "save"
        app.query_one("#rerun-viewer-value").value = str(recording)
        app.query_one("#rerun-stale-after").value = "4"
        await pilot.pause(0.2)
        command = text_of(app, "#rerun-command")
        assert command.startswith(f"blick rerun --save {recording} {BASE} --stale-after 4 --connect")

        # saved, changed, loaded back
        app.query_one("#preset-name").value = "fake"
        app.query_one("#preset-save").press()
        await pilot.pause(0.2)
        assert list(app.presets.load()) == ["fake"]
        app.query_one("#rerun-stale-after").value = ""
        app.query_one("#preset-select").value = "fake"
        app.query_one("#preset-load").press()
        await pilot.pause(0.2)
        assert app.query_one("#rerun-stale-after").value == "4"

        # a preset's stream that is not there is listed as not found, and stays chosen
        app.apply(tui.RerunSelection(bases=[BASE], segmentations=[f"{CAMERA}/seg/instances"],
                                     connection=app.connection))
        assert streams.option_count == 2 and f"segmentations\t{CAMERA}/seg/instances" in streams.selected

        # launched and stopped: the recording has the camera's frames
        app.apply(app.presets.load()["fake"])
        app.query_one("#rerun-launch").press()
        await pilot.pause(0.2)
        await until(pilot, lambda: any(line.startswith("bridging") for line in app.rerun_output),
                    timeout=60)
        await pilot.pause(2.0)
        app.stop_rerun()
        await until(pilot, lambda: "blick rerun ended, exit code 0" in app.rerun_output)
        assert recording.stat().st_size > 10_000

    run_app(app, test)


# ------------------------------------------------------------------------------------------- the command


def test_no_command_opens_the_interface(monkeypatch):
    called = []
    monkeypatch.setattr(tui, "run", lambda connection=None: called.append(connection) or 0)
    assert cli.main([]) == 0 and called == [None]  # this PC's router


def test_without_a_terminal_the_interface_refuses():
    result = subprocess.run([sys.executable, "-m", "blick.cli"], stdin=subprocess.DEVNULL, capture_output=True,
                            text=True, timeout=60, check=False)
    assert result.returncode == 1 and "needs a terminal" in result.stderr


def test_commands_do_not_import_textual():
    code = ("import sys; from blick import cli; cli.build_parser(); "
            "print(any(m.split('.')[0] == 'textual' for m in sys.modules))")
    result = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, timeout=60, check=True)
    assert result.stdout.strip() == "False"


def test_blick_is_durchblick(monkeypatch, capsys):
    scripts = {e.name: e.value for e in entry_points(group="console_scripts") if e.name in cli.SCRIPTS}
    assert scripts == {"blick": "blick.cli:main"}
    monkeypatch.setattr(sys, "argv", ["/usr/bin/blick", "-h"])
    with pytest.raises(SystemExit):
        cli.main()
    assert capsys.readouterr().out.startswith("usage: blick ")
    monkeypatch.setattr(sys, "argv", ["/x/something-else", "-h"])
    with pytest.raises(SystemExit):
        cli.main()
    assert capsys.readouterr().out.startswith("usage: blick ")


@pytest.mark.parametrize("size", [(80, 24), (250, 70)])
def test_app_fits_the_terminal(fake, tmp_path, size):
    """At the smallest size supported and a large one: no screen scroll (the status line under the header), the key
    column whole, the detail panes, every tab's main widget and the connect dialog on the screen, the Rerun tab's
    lists side by side only when wide."""
    app = tui.DurchblickApp(fake.connection(), tui.Presets(tmp_path / "presets.json"), discover_every=0.5)
    width, height = size

    def on_screen(widget):
        r = widget.region
        return r.width > 0 and r.height > 0 and r.x >= 0 and r.y >= 0 and r.right <= width and r.bottom <= height

    async def test(pilot):
        await until(pilot, lambda: app.discovery.streams and cells(app, "#topic-table"))
        screen = app.screen
        assert screen.virtual_size.height <= screen.size.height, "the screen scrolls"
        status = app.query_one("#status")
        assert status.region.y == 1 and on_screen(status) and "router(s)" in text_of(app, "#status")
        table = app.query_one("#topic-table")
        longest = max(len(r.label) for r in app._topic_rows.values())
        # the column's auto width follows its rows on a later refresh (slow runners measured it before)
        await until(pilot, lambda: table.columns["key"].get_render_width(table) >= longest)
        assert table.region.width >= longest
        app._select_topic(f"{BASE}/camera_info")
        await until(pilot, lambda: "width" in text_of(app, "#topic-detail"))
        detail = app.query_one("#topic-detail").parent
        assert on_screen(detail) and detail.region.height >= 3
        for tab, widget in tui.DurchblickApp.MAIN.items():
            app.action_tab(tab)
            await pilot.pause(0.3)
            assert app.focused is app.query_one(widget), (tab, app.focused)
            assert on_screen(app.query_one(widget)), (tab, app.query_one(widget).region)
        app.action_tab("tab-rerun")
        await pilot.pause(0.3)
        streams, form = app.query_one("#rerun-streams"), app.query_one("#rerun-form")
        if width < tui.WIDE:
            assert streams.region.x == form.region.x and streams.region.bottom <= form.region.y
        else:
            assert streams.region.y == form.region.y and streams.region.right <= form.region.x
        assert on_screen(app.query_one("#rerun-command")) and on_screen(app.query_one("#rerun-launch"))
        await pilot.press("c")
        await pilot.pause(0.3)
        dialog = app.screen
        assert isinstance(dialog, tui.ConnectScreen)
        for widget_id in ("#connect-dialog", "#connect-mode", "#connect-endpoints", "#config", "#connect-ok"):
            assert on_screen(dialog.query_one(widget_id)), widget_id
        await pilot.press("escape")

    run_app(app, test, size=size)


def test_too_small_says_so(tmp_path):
    app = tui.DurchblickApp(tui.Connection(listen=["127.0.0.1:0"]), tui.Presets(tmp_path / "presets.json"))

    async def test(pilot):
        await pilot.pause(0.5)
        note = app.query_one("#too-small")
        assert note.display and not app.query_one("#main").display
        assert f"needs at least {tui.MIN_SIZE[0]} x {tui.MIN_SIZE[1]}" in text_of(app, "#too-small")
        await pilot.resize_terminal(*tui.MIN_SIZE)
        await pilot.pause(0.5)
        assert not note.display and app.query_one("#main").display

    run_app(app, test, size=(tui.MIN_SIZE[0] - 1, tui.MIN_SIZE[1]))


def test_the_interface_takes_session_options(monkeypatch):
    """``blick --connect ...``: the interface's first connection; with endpoints a client unless it listens or is
    told peer (a peer over a tunnel cannot reach the other sessions)."""
    c = cli._interface_connection(["--connect", "127.0.0.1:7447"], "blick")
    assert (c.connect, c.listen, c.mode) == (["tcp/127.0.0.1:7447"], [], "client")
    c = cli._interface_connection(["--connect", "tcp/a:7447", "--listen", "tcp/127.0.0.1:0"], "blick")
    assert c.mode == "peer" and c.listen == ["tcp/127.0.0.1:0"]
    with pytest.raises(SystemExit):  # a peer that only connects receives nothing: refused
        cli._interface_connection(["--connect", "tcp/a:7447", "--zenoh-mode", "peer"], "blick")
    assert cli._interface_connection([], "blick") is None
    started = []
    monkeypatch.setattr(tui, "run", lambda connection=None: started.append(connection) or 0)
    monkeypatch.setattr(sys, "argv", ["/usr/bin/blick"])
    assert cli.main(["--connect", "tcp/127.0.0.1:7447"]) == 0
    assert started[0].mode == "client" and started[0].connect == ["tcp/127.0.0.1:7447"]


def test_a_peer_warns_of_peers_it_cannot_reach(fake, monkeypatch, tmp_path):  # noqa: F811
    """As a peer, a session the admin space shows but this one has no link to for UNREACHABLE_S (over a tunnel:
    another host's loopback addresses) is said in the status line, with client mode as the way out; as a client
    nothing is said. (A peer that does not listen still links: it connects to this one, so the lack of a link is
    simulated.)"""
    monkeypatch.setattr(tui, "UNREACHABLE_S", 0.0)

    class Info:
        def __init__(self, own, linked):
            self.own, self.linked = own, linked

        def zid(self):
            return self.own

        def peers_zid(self):
            return self.linked

    class Node:
        def __init__(self, whatami):
            self.whatami = whatami

    network = survey.Network()
    network.nodes = {"me": Node("peer"), "near": Node("peer"), "far": Node("peer"), "cli": Node("client"),
                     "r": Node("router")}
    session = type("S", (), {"info": Info("me", ["near"])})()
    as_peer = type("A", (), {"connection": tui.Connection(connect=["tcp/x:7447"], listen=["tcp/127.0.0.1:0"])})()
    assert tui.DurchblickApp._unreachable(as_peer, session, network, {}) == ["far"]
    as_client = type("A", (), {"connection": tui.Connection(connect=["tcp/x:7447"], mode="client")})()
    assert tui.DurchblickApp._unreachable(as_client, session, network, {}) == []
    # and in the status line
    connection = tui.Connection(connect=[fake.endpoint], listen=["tcp/127.0.0.1:0"], mode="peer")
    app = tui.DurchblickApp(connection, tui.Presets(tmp_path / "presets.json"), discover_every=0.5)

    async def test(pilot):
        await until(pilot, lambda: app.discovery.network is not None)
        app.discovery.unreachable = ["far"]
        app._status()
        await pilot.pause(0.1)
        assert "1 peer(s) not reachable from here" in text_of(app, "#status")

    run_app(app, test, size=(250, 45))


def test_without_anything_on_this_pc_the_connect_dialog_opens_on_the_address(fake, tmp_path):
    """Started on this PC's router with none there: the dialog by itself, the cursor in the address; the address
    typed without tcp/ connects (client mode, the dialog's default)."""
    import socket

    with socket.socket() as s:
        if s.connect_ex(("127.0.0.1", 7447)) == 0:
            pytest.skip("a router runs on this PC (port 7447): the interface finds it")
    app = tui.DurchblickApp(presets=tui.Presets(tmp_path / "presets.json"), discover_every=0.5)

    async def test(pilot):
        await until(pilot, lambda: isinstance(app.screen, tui.ConnectScreen)
                    and app.focused is app.screen.query_one("#connect-endpoints"))
        await pilot.press(*fake.endpoint.removeprefix("tcp/"), "enter")
        await until(pilot, lambda: app.discovery.network is not None and len(app.discovery.network.routers) == 1)
        assert app.connection == tui.Connection(connect=[fake.endpoint], mode="client")
        app._open(tui.Connection())  # once: back on this PC's router, nothing opens by itself again
        await pilot.pause(3.0)
        assert not isinstance(app.screen, tui.ConnectScreen)

    run_app(app, test)
