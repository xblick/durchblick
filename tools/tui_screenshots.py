"""Screenshots of the terminal interface for docs/guides/tui.md, from a network made up for them.

    uv run python tools/tui_screenshots.py              # docs/media/tui/*.png
    uv run python tools/tui_screenshots.py --out DIR    # elsewhere

The network runs in this process, each "program" a Zenoh session of its own on a router with the admin space on (as
zenohd): an Azure Kinect (color and depth, frames from a pattern), SAM 3 taking the color images by shm, FoundationPose
taking color, depth and SAM 3's instances, TAPNext and SAM 3 taking each other's results (a cycle), a robot state
bridge, a recorder and a viewer as a client taking compressed images. SAM 3 offers some of xblick-sam3's
parameters, with its names, defaults and live ones (its options as declare_options makes them). Only the camera sends images; the nodes send
their results and diagnostics as small JSON. Nothing of a real node runs.

The interface runs headless (``App.run_test``) at a fixed size; each screen is Textual's SVG export, made a PNG by
headless Chromium (Playwright's, ``CHROMIUM`` to give another) with DejaVu Sans Mono embedded: Chromium finds no
system fonts here, and the SVG's own font comes from the web.
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import glob
import json
import os
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

import numpy as np
import zenoh

from xblick import tui
from xblick.camera import CameraPublisher
from xblick.diagnostics import DiagnosticsPublisher
from xblick.msg import CameraInfo
from xblick import _core
from xblick.parameters import Parameters
from xblick.session import Session, open_session
from xblick.session import close as close_session

REPO = Path(__file__).resolve().parents[1]
CAMERA = "robot/camera"
HOST = _core.host_id()  # the rings' keys name the host
SIZE = (150, 42)  # columns, rows of the screenshots
FONT = Path("/usr/share/fonts/truetype/dejavu")


def free_port() -> int:
    import socket

    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class DemoNetwork:
    """The made-up network (see the module's docstring)."""

    def __init__(self):
        port = free_port()
        self.endpoint = f"tcp/127.0.0.1:{port}"
        config = zenoh.Config()
        config.insert_json5("mode", '"router"')
        config.insert_json5("listen/endpoints", json.dumps([self.endpoint]))
        config.insert_json5("scouting/multicast/enabled", "false")
        config.insert_json5("adminspace/enabled", "true")
        self.router = Session(zenoh.open(config))
        self.sessions, self.closers = [], []
        self._stop = threading.Event()
        self._threads = []

        kinect = self._session()
        info = CameraInfo.pinhole(64, 48, 50.0, 50.0, 31.5, 23.5, frame_id="azurekinect_color")
        color = CameraPublisher(kinect, f"{CAMERA}/color", encoding="rgb8", frame_id="azurekinect_color",
                                camera_info=info, source="demo")
        depth = CameraPublisher(kinect, f"{CAMERA}/depth_to_color", encoding="mono16", frame_id="azurekinect_color",
                                camera_info=info, source="demo", codec=None)
        kinect_diag = DiagnosticsPublisher(kinect, CAMERA, "azurekinect")
        self.closers += [color.close, depth.close, kinect_diag.close]

        def frames():
            seq = 0
            while not self._stop.is_set():
                seq += 1
                y, x = np.mgrid[0:48, 0:64]
                color.publish(np.stack([(x + seq) % 256, y * 4 % 256, np.full_like(x, seq % 256)], -1)
                              .astype(np.uint8))
                depth.publish((x * 40 + seq).astype(np.uint16))
                if seq % 15 == 0:
                    kinect_diag.publish(DiagnosticsPublisher.OK, "30.0 frames/s", {"frames/s": 30.0})
                time.sleep(1 / 30)

        self._thread(frames)
        self.sam3 = self._node("sam3", takes=[f"{CAMERA}/color/image_raw/shm/{HOST}", f"{CAMERA}/tapnext/tracks"],
                               gives=["instances", "boxes"], rate=12.4)
        self._node("foundationpose", takes=[f"{CAMERA}/color/image_raw/shm/{HOST}",
                                            f"{CAMERA}/depth_to_color/image_raw/shm/{HOST}",
                                            f"{CAMERA}/sam3/instances"], gives=["poses", "tf"], rate=8.1)
        self._node("tapnext", takes=[f"{CAMERA}/color/image_raw/shm/{HOST}", f"{CAMERA}/sam3/instances"],
                   gives=["tracks"], rate=29.7)
        self.params = Parameters(self.sam3, "sam3", CAMERA)
        self.params.declare("threshold", 0.5, range=(0.0, 1.0, 0.01), description="--threshold: the least score of an "
                            "instance")
        self.params.declare("match_iou", 0.3, range=(0.01, 1.0, 0.01), description="--match-iou: the least overlap of "
                            "an instance's mask with an object's last one for it to keep that object's id")
        self.params.declare("max_missing", 15, range=(0, 300, 1), unit="frames", description="--max-missing: frames an "
                            "object may go unseen and keep its id")
        self.params.declare("words", ["cup,box,bottle"], description="--words: text prompts")
        self.params.declare("resolution", 672, read_only=True, description="--resolution: the size frames are scaled to")
        self.params.declare("max_hz", 30.0, read_only=True, description="--max-hz: at most this many frames a second")
        self.params.check_overrides()
        self.closers.append(self.params.close)
        bridge = self._session()
        robot = [bridge.declare_publisher(k) for k in ("robot/joint_states", "robot/tf",
                                                         "robot/controller/arms/status")]

        def state():
            while not self._stop.is_set():
                for publisher in robot:
                    publisher.put(json.dumps({"stamp_ns": time.time_ns()}),
                                  encoding=zenoh.Encoding.APPLICATION_JSON)
                time.sleep(0.01)

        self._thread(state)
        recorder = self._session()
        status = recorder.declare_queryable(
            "xblick/recorder/status",
            lambda q: q.reply("xblick/recorder/status", json.dumps(
                {"name": "recorder", "state": "recording", "file": "/data/robot/demo.mcap", "messages": 18234,
                 "dropped": 0}), encoding=zenoh.Encoding.APPLICATION_JSON))
        recorder_subs = [recorder.declare_subscriber(k, lambda s: None) for k in ("robot/joint_states", "robot/tf")]
        self.closers += [status.undeclare, *[s.undeclare for s in recorder_subs]]
        viewer = self._session(client=True)
        viewer_subs = [viewer.declare_subscriber(k, lambda s: None)
                       for k in (f"{CAMERA}/color/image_raw/compressed", f"{CAMERA}/sam3/instances",
                                 f"{CAMERA}/foundationpose/tf")]
        self.closers += [s.undeclare for s in viewer_subs]

    def _session(self, client: bool = False):
        session = (open_session(connect=[self.endpoint], mode="client") if client else
                   open_session(connect=[self.endpoint], listen=["127.0.0.1:0"]))
        self.sessions.append(session)
        return session

    def _thread(self, target):
        thread = threading.Thread(target=target, daemon=True)
        thread.start()
        self._threads.append(thread)

    def _node(self, name: str, takes: list[str], gives: list[str], rate: float):
        """A vision node: subscribes to ``takes``, publishes ``<camera>/<name>/<give>`` and its diagnostics."""
        session = self._session()
        subscribers = [session.declare_subscriber(k, lambda s: None) for k in takes]
        publishers = [session.declare_publisher(f"{CAMERA}/{name}/{g}") for g in gives]
        diag = DiagnosticsPublisher(session, CAMERA, name)
        self.closers += [*[s.undeclare for s in subscribers], *[p.undeclare for p in publishers], diag.close]

        def run():
            n = 0
            while not self._stop.is_set():
                n += 1
                for publisher in publishers:
                    publisher.put(json.dumps({"v": 1, "stamp_ns": time.time_ns()}),
                                  encoding=zenoh.Encoding.APPLICATION_JSON)
                if n % max(int(rate), 1) == 0:
                    diag.publish(DiagnosticsPublisher.OK, f"{rate:.1f} frames/s", {"frames/s": rate})
                time.sleep(1 / rate)

        self._thread(run)
        return session

    def close(self):
        self._stop.set()
        for thread in self._threads:
            thread.join()
        for close in self.closers:
            close()
        for session in [*self.sessions, self.router]:
            close_session(session)


def chromium() -> str:
    if os.environ.get("CHROMIUM"):
        return os.environ["CHROMIUM"]
    found = sorted(glob.glob(str(Path.home() / ".cache/ms-playwright/chromium-*/chrome-linux*/chrome")))
    if not found:
        raise SystemExit("no Chromium: install Playwright's (playwright install chromium) or give CHROMIUM")
    return found[-1]


def to_png(svg: Path, png: Path, size: tuple[int, int]):
    """The SVG as a PNG of its own size, with DejaVu Sans Mono embedded for every font the SVG names."""
    faces = "".join(
        f"@font-face {{ font-family: 'Xblick Mono'; font-weight: {weight}; "
        f"src: url(data:font/ttf;base64,{base64.b64encode((FONT / file).read_bytes()).decode()}); }}\n"
        for weight, file in (("normal", "DejaVuSansMono.ttf"), ("bold", "DejaVuSansMono-Bold.ttf")))
    text = svg.read_text()
    width, height = (float(v) for v in text.split('viewBox="', 1)[1].split('"', 1)[0].split()[2:4])
    html = (f"<!doctype html><meta charset='utf-8'><style>{faces} html, body {{ margin: 0; background: #1e1e1e; }}"
            f" svg text, svg tspan {{ font-family: 'Xblick Mono' !important; }}</style>{text}")
    with tempfile.TemporaryDirectory() as tmp:
        page = Path(tmp) / "page.html"
        page.write_text(html)
        subprocess.run([chromium(), "--headless=new", "--no-sandbox", "--disable-gpu", "--hide-scrollbars",
                        f"--screenshot={png}", f"--window-size={int(width)},{int(height)}", page.as_uri()],
                       check=True, capture_output=True, timeout=120)


async def capture(network: DemoNetwork, out: Path):
    app = tui.XblickApp(tui.Connection(connect=[network.endpoint], mode="client"),
                        tui.Presets(out / ".presets.json"), discover_every=1.0)
    shots = []

    async def until(pilot, condition, timeout=20.0):
        end = time.monotonic() + timeout
        while not condition():
            if time.monotonic() > end:
                raise TimeoutError
            await pilot.pause(0.1)

    def save(name: str):
        path = out / f"{name}.svg"
        path.write_text(app.export_screenshot(title=f"xblick: {name}"))
        shots.append(path)

    async with app.run_test(size=SIZE) as pilot:
        await until(pilot, lambda: app.discovery.network is not None and len(app.discovery.network.nodes) >= 9)
        await until(pilot, lambda: len(app.query_one("#graph").edges) >= 9)
        await pilot.press("m")  # rates, sizes and ages of every topic
        await pilot.pause(3.5)
        save("topics")
        app._select_topic(f"{CAMERA}/color/image_raw/compressed")
        await until(pilot, lambda: app.detail.last(f"{CAMERA}/color/image_raw/compressed") is not None)
        await pilot.pause(1.5)
        save("topic-detail")
        await pilot.press("escape", "m")
        await pilot.press("3")
        graph = app.query_one("#graph")
        sam3 = str(network.sam3.info.zid())
        graph.selected = sam3
        await pilot.pause(0.5)
        save("graph")
        edge = next(k for k, e in enumerate(graph.edges) if e.target == sam3 and e.label == "shm")
        graph.hovered = ("edges", frozenset({edge}))
        await pilot.pause(0.5)
        save("graph-edge")
        graph.hovered = None
        await pilot.press("2")
        await pilot.pause(1.0)
        save("nodes")
        await pilot.press("4")
        await pilot.pause(1.0)
        save("rerun")
        await pilot.press("5")
        pane = app.query_one(tui.ParametersPane)
        await until(pilot, lambda: f"{CAMERA}/sam3" in pane.nodes)
        app.query_one("#param-nodes").highlighted = pane.nodes.index(f"{CAMERA}/sam3")
        await until(pilot, lambda: len(pane.descriptors) == 6)
        await pilot.pause(0.5)
        i = [d.name for d in pane.descriptors].index("max_missing")
        app.query_one(f"#param-{i}").focus()
        await pilot.press("down", "down")  # stepped and set
        await until(pilot, lambda: network.params["max_missing"] == 13)
        j = [d.name for d in pane.descriptors].index("threshold")
        app.query_one(f"#param-{j}").value = "1.5"
        app.query_one(f"#param-{j}").focus()
        await pilot.press("enter")  # refused: the node's reason under the field
        await pilot.pause(1.0)
        app.query_one(f"#param-{i}").focus()
        app.query_one(f"#param-{i}").value = "30"  # edited, not set yet
        await pilot.pause(0.3)
        save("parameters")
        await pilot.press("escape")
        await pilot.press("c")
        await pilot.pause(0.5)
        await pilot.press(*"robot-pc")
        await pilot.pause(0.3)
        save("connect")
        await pilot.press("escape")
    return shots


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--out", type=Path, default=REPO / "docs" / "media" / "tui")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    network = DemoNetwork()
    with tempfile.TemporaryDirectory() as tmp:
        try:
            shots = asyncio.run(capture(network, Path(tmp)))
        finally:
            network.close()
        for svg in shots:
            png = args.out / svg.with_suffix(".png").name
            to_png(svg, png, SIZE)
            print(png)
    return 0


if __name__ == "__main__":
    sys.exit(main())
