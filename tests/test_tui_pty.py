"""The terminal interface in a real terminal: ``xblick`` in a pseudo-terminal driven by virtui
(github.com/honeybadge-labs/virtui), against the fake network of test_tui. Catches what ``App.run_test`` does not: a
real terminal's size and layout (the status line scrolled away under the header) and keys reaching the widgets
(focus). Skipped without virtui (on PATH or in ~/.local/bin); it runs its own virtui daemon on a private socket."""

import json
import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

import pytest
from test_tui import BASE, CAMERA, fake  # noqa: F401 - fake is a fixture

VIRTUI = shutil.which("virtui") or (str(Path.home() / ".local/bin/virtui")
                                    if (Path.home() / ".local/bin/virtui").exists() else None)
pytestmark = pytest.mark.skipif(VIRTUI is None, reason="virtui is not installed")
ANSI = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]")


class Terminal:
    """One virtui session on a private daemon."""

    def __init__(self, socket: Path):
        self.socket = str(socket)
        self.session = None

    def __call__(self, *args, timeout=60.0) -> str:
        result = subprocess.run([VIRTUI, "--socket", self.socket, *args], capture_output=True, text=True,
                                timeout=timeout, check=False)
        if result.returncode != 0:
            raise AssertionError(f"virtui {' '.join(args)}: {result.stdout}{result.stderr}\n{self.screen()}")
        return result.stdout

    def run(self, command: list[str], env: dict, cols=170, rows=45):
        out = self("--json", "run", "--cols", str(cols), "--rows", str(rows),
                   *[a for k, v in env.items() for a in ("--env", f"{k}={v}")], *command)
        self.session = json.loads(out)["session_id"]

    def press(self, *keys):
        for key in keys:  # one by one, with the app's time to act (focus moves, a dialog mounts)
            self("press", self.session, key)
            time.sleep(0.2)

    def type(self, text):
        self("type", self.session, text)
        time.sleep(0.2)

    def wait(self, text=None, regex=None, timeout_ms=20000):
        flag = ["--text", text] if text is not None else ["--regex", regex]
        self("wait", self.session, *flag, "--timeout", str(timeout_ms), timeout=timeout_ms / 1000 + 10)

    def connect(self, endpoint: str):
        """Connects as a user does. Started without a router on this PC the dialog opens by itself (else ``c``), on
        the address: typed without ``tcp/`` (the default), Enter."""
        try:
            self.wait("Zenoh connection", timeout_ms=8000)
        except AssertionError:  # a router on this PC: the interface found it
            self.press("c")
            self.wait("Zenoh connection")
        self.type(endpoint.removeprefix("tcp/"))
        self.press("Enter")
        self.wait("router(s)")

    def state(self) -> dict:
        return json.loads(self("--json", "sessions", "show", self.session))["sessions"][0]

    def screen(self) -> str:
        if self.session is None:
            return ""
        result = subprocess.run([VIRTUI, "--socket", self.socket, "screenshot", self.session], capture_output=True,
                                text=True, timeout=30, check=False)
        return ANSI.sub("", result.stdout)


@pytest.fixture
def terminal(tmp_path):
    t = Terminal(tmp_path / "v.sock")
    t("daemon", "start")
    yield t
    if t.session is not None:
        subprocess.run([VIRTUI, "--socket", t.socket, "kill", t.session], capture_output=True, timeout=30,
                       check=False)
    t("daemon", "stop")


@pytest.mark.parametrize("size", [(170, 45), (80, 24)])
def test_blick_in_a_real_terminal(fake, terminal, tmp_path, size):  # noqa: F811
    xblick = str(Path(sys.executable).parent / "xblick")
    # COLUMNS and LINES as readline leaves them for child processes (Python before 3.13): the interface must take the
    # terminal's size all the same, also after the resizes below
    terminal.run([xblick], {"XDG_CONFIG_HOME": str(tmp_path / "config"), "COLUMNS": "80", "LINES": "24"}, *size)
    terminal.wait("Topics")

    # connect through the dialog, as a user does: the address, client mode (the default there), Enter
    terminal.connect(fake.endpoint)
    lines = terminal.screen().splitlines()
    assert "Durchblick" in lines[0] and "--zenoh-mode client --connect" in lines[1], lines[:3]

    # m measures: rates in the table, the status line says so
    terminal.press("m")
    terminal.wait(regex=r"─ camera_info +\d+\.\d")  # Hz next to the topic (the tree's leaf)
    assert "measuring every topic" in terminal.screen()

    # the table is a tree: Enter on a prefix folds it and unfolds it again; on a key (camera_info, the first key of
    # the camera's stream) it shows the topic
    terminal.press("Enter")
    terminal.wait(regex=r"▸ robot/fakecam/ +\(\d+ keys\)")
    terminal.press("Enter")
    terminal.wait("├─ ▾ color/")
    terminal.press("ArrowDown", "ArrowDown", "Enter")
    terminal.wait("published by:")
    # the message's width: in view at once in a large pane, else the pane scrolled (Tab focuses it) until it shows;
    # the message may come a little later than the pane
    for attempt in range(20):
        if "'width': 64" in terminal.screen():
            break
        if attempt == 0:
            terminal.press("Tab")
        terminal.press("PageDown")
        time.sleep(0.3)
    terminal.wait("'width': 64")
    terminal.press("1")  # the table again

    terminal.press("2")
    terminal.wait(f"{CAMERA}/tracker")
    terminal.wait("waiting: frames")
    terminal.press("Enter")  # the first session's detail (rows by zid: a router or a session)
    terminal.wait(regex=r"router v1\.|address tcp/")

    terminal.press("3")
    terminal.wait("tcp-compressed")  # the edge's label, before its arrow
    terminal.press("ArrowRight")  # the tracker, right of the camera: what it takes
    terminal.wait("takes from")

    terminal.press("4")
    terminal.wait(f"{BASE}  image")
    terminal.press("Space")
    terminal.wait(f"xblick rerun --spawn {BASE} --zenoh-mode client")

    # resized: too small says so, larger has the status line again
    terminal("resize", "--cols", "60", "--rows", "20", terminal.session)
    terminal.wait("terminal too small: it needs at least 80 x 24")
    terminal("resize", "--cols", "250", "--rows", "70", terminal.session)
    terminal.wait(f"xblick rerun --spawn {BASE} --zenoh-mode client --connect {fake.endpoint}")
    assert "terminal too small" not in terminal.screen()
    assert "--zenoh-mode client --connect" in terminal.screen().splitlines()[1]

    terminal.press("1")
    terminal.press("q")  # ends cleanly: exit code 0, no traceback on the screen
    end = time.monotonic() + 15
    while (state := terminal.state())["running"] and time.monotonic() < end:
        time.sleep(0.2)
    assert not state["running"] and state["exit_code"] == 0, terminal.screen()
    assert "Traceback" not in terminal.screen()


@pytest.mark.parametrize("size", [(170, 45), (80, 24)])
def test_parameters_in_a_real_terminal(fake, terminal, tmp_path, size):  # noqa: F811
    """The Parameters tab in a real terminal: the node, its form, a value refused with the reason, one taken."""
    from xblick.parameters import Parameters

    params = Parameters(fake.node_session, "tracker", "robot/fakecam")
    params.declare("draw", True)
    params.declare("max_boxes", 10, range=(1, 100, 1), unit="boxes", description="at most so many")
    params.declare("model", "base", read_only=True)
    params.check_overrides()
    try:
        xblick = str(Path(sys.executable).parent / "xblick")
        terminal.run([xblick], {"XDG_CONFIG_HOME": str(tmp_path / "config")}, *size)
        terminal.wait("Topics")
        terminal.connect(fake.endpoint)
        terminal.press("5")
        terminal.wait(f"{CAMERA}/tracker")
        terminal.press("Enter")
        terminal.wait("max_boxes")
        terminal.wait("string  read-only")
        terminal.press("Tab", "Tab")  # the checkbox, then max_boxes
        terminal.press("End", "Backspace", "Backspace")
        terminal.type("500")
        terminal.press("Enter")
        terminal.wait("500 is outside 1 to 100")
        if os.environ.get("XBLICK_TUI_SCREENS"):      # a directory for the screen's text (documentation)
            (Path(os.environ["XBLICK_TUI_SCREENS"]) / f"parameters-{size[0]}x{size[1]}.txt").write_text(
                terminal.screen())
        terminal.press("End", "Backspace", "Backspace", "Backspace")
        terminal.type("50")
        terminal.press("Enter")
        end = time.monotonic() + 10
        while params["max_boxes"] != 50 and time.monotonic() < end:
            time.sleep(0.1)
        assert params["max_boxes"] == 50
        terminal.press("q")
    finally:
        params.close()
