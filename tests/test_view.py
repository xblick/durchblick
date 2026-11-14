"""xblick view (view.py): the streams' JPEGs unchanged as MJPEG over HTTP, in order; two clients; a slow client that
holds up nobody else and is cleaned up when it goes; the index page; depth coloured with the fixed map; the errors (no
such stream, a point cloud, no compressed transport, a taken port, VLC not installed); the start lines and Ctrl-C."""

import io
import os
import pty
import select
import signal
import socket
import termios
import subprocess
import sys
import threading
import time
import urllib.request

import numpy as np
import pytest
from PIL import Image
from test_camera import free_port, pattern, sessions, wait_until  # noqa: F401 - sessions is a fixture

from xblick import topics, view
from xblick.camera import CameraPublisher
from xblick.codecs import RVL, compress
from xblick.msg import CompressedImage
from xblick.overlay import depth_colours
from xblick.points import XYZRGB, PointCloudPublisher
from xblick.session import open_session


def read_frame(stream) -> bytes:
    """The next part of a multipart/x-mixed-replace response: its JPEG."""
    assert stream.readline() == f"--{view.BOUNDARY}\r\n".encode()
    length = None
    while (line := stream.readline()) != b"\r\n":
        name, _, value = line.decode().partition(":")
        if name.lower() == "content-length":
            length = int(value)
        elif name.lower() == "content-type":
            assert value.strip() == "image/jpeg"
    jpeg = stream.read(length)
    assert stream.readline() == b"\r\n"
    return jpeg


class Publishing:
    """A thread publishing pattern frames as fast as ``period`` lets it, until ``stop``."""

    def __init__(self, publisher, period=0.02, size=(48, 64)):
        self.publisher, self.period, self.size = publisher, period, size
        self.stop, self.count, self.slowest = threading.Event(), 0, 0.0
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        while not self.stop.is_set():
            started = time.monotonic()
            self.publisher.publish(pattern(self.count, *self.size))
            self.slowest = max(self.slowest, time.monotonic() - started)
            self.count += 1
            time.sleep(self.period)

    def close(self):
        self.stop.set()
        self.thread.join(5)


@pytest.fixture
def served(sessions):  # noqa: F811
    a, b = sessions
    publisher = CameraPublisher(a, "t/view/color", encoding="rgb8", frame_id="cam")
    sent = []  # the JPEGs on the compressed topic, as any subscriber gets them
    tap = b.declare_subscriber(topics.compressed("t/view/color"),
                               lambda s: sent.append(bytes(CompressedImage.deserialize(s.payload.to_bytes()).data)))
    publishing = Publishing(publisher)
    v = view.View(b, ["t/view/color"], ("127.0.0.1", 0))
    yield v, sent, publishing
    v.close()
    publishing.close()
    tap.undeclare()
    publisher.close()


def test_a_client_gets_the_jpegs_unchanged_and_in_order(served):
    v, sent, _ = served
    with urllib.request.urlopen(v.url("t/view/color"), timeout=10) as response:
        assert response.headers["Content-Type"] == f"multipart/x-mixed-replace; boundary={view.BOUNDARY}"
        got = [read_frame(response) for _ in range(8)]
    time.sleep(0.2)
    positions = [sent.index(jpeg) for jpeg in got]  # each one byte for byte as published
    assert positions == sorted(positions) and len(set(positions)) == len(positions)  # in order, none twice
    assert Image.open(io.BytesIO(got[0])).format == "JPEG"
    wait_until(lambda: v.clients == 0, timeout=5.0)  # the client went: its thread ended


def test_two_clients(served):
    v, _, _ = served
    with urllib.request.urlopen(v.url("t/view/color"), timeout=10) as one, \
            urllib.request.urlopen(v.url("t/view/color"), timeout=10) as two:
        for _ in range(5):
            assert read_frame(one) and read_frame(two)
        assert v.clients == 2


def test_a_slow_client_holds_up_neither_the_subscriber_nor_the_others(sessions):  # noqa: F811
    a, b = sessions
    publisher = CameraPublisher(a, "t/view/big", encoding="rgb8", frame_id="cam", jpeg_quality=100)
    publishing = Publishing(publisher, period=0.01, size=(480, 640))  # some 100-300 kB per frame
    v = view.View(b, ["t/view/big"], ("127.0.0.1", 0))
    slow = socket.create_connection(("127.0.0.1", v.port))
    slow.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4096)
    try:
        slow.sendall(b"GET /t/view/big.mjpg HTTP/1.1\r\nHost: x\r\n\r\n")  # and then reads nothing
        wait_until(lambda: v.clients == 1, timeout=5.0)
        time.sleep(1.0)  # its socket buffers fill; its thread blocks in a write
        with urllib.request.urlopen(v.url("t/view/big"), timeout=10) as fast:
            started = time.monotonic()
            frames = [read_frame(fast) for _ in range(20)]
            elapsed = time.monotonic() - started
        assert len(set(frames)) == 20 and elapsed < 5.0, elapsed  # the fast one is served as before
        received = v.feeds["t/view/big"].number
        time.sleep(0.5)
        assert v.feeds["t/view/big"].number > received  # the subscriber goes on
        assert publishing.slowest < 0.5, publishing.slowest  # the publisher was never held up
    finally:
        slow.close()
        wait_until(lambda: v.clients == 0, timeout=view.CLIENT_TIMEOUT_S + 5)  # the slow one cleaned up as well
        v.close()
        publishing.close()
        publisher.close()


def test_the_index_page_lists_the_streams(sessions):  # noqa: F811
    a, b = sessions
    one = CameraPublisher(a, "t/view/one", encoding="rgb8", frame_id="cam")
    two = CameraPublisher(a, "t/view/two", encoding="rgb8", frame_id="cam")
    v = view.View(b, ["t/view/one", "t/view/two"], ("127.0.0.1", 0))
    try:
        with urllib.request.urlopen(v.url(), timeout=10) as response:
            assert response.headers["Content-Type"] == "text/html; charset=utf-8"
            page = response.read().decode()
        for base in ("t/view/one", "t/view/two"):
            assert f'<img src="/{base}.mjpg"' in page and f"<figcaption>{base}</figcaption>" in page
        assert "<script" not in page and "http" not in page.replace("http-equiv", "")  # nothing from outside
        with pytest.raises(urllib.error.HTTPError) as error:
            urllib.request.urlopen(v.url("t/view/three"), timeout=10)
        assert error.value.code == 404
    finally:
        v.close()
        one.close()
        two.close()


def test_depth_is_coloured_with_the_fixed_map():
    depth = np.zeros((64, 96), np.uint16)
    depth[:, 16:32], depth[:, 32:48], depth[:, 48:64], depth[:, 64:] = 300, 2650, 5000, 9000
    for codec in (RVL, "png"):
        jpeg = view.to_jpeg(compress(depth, "16UC1", codec), view.DEPTH_RANGE)
        image = np.asarray(Image.open(io.BytesIO(jpeg)).convert("RGB")).astype(int)
        expected = depth_colours(depth).astype(int)
        for x in (8, 24, 40, 56, 80):  # the middle of each band: black, red, green, blue, blue
            assert np.abs(image[32, x] - expected[32, x]).max() <= 12, (codec, x, image[32, x], expected[32, x])
    grey = view.to_jpeg(compress(pattern(1)[..., 0].copy(), "mono8", "png"), view.DEPTH_RANGE)
    assert Image.open(io.BytesIO(grey)).size == (64, 48)  # an 8-bit PNG: encoded as JPEG, not coloured


def test_streams_that_cannot_be_shown_are_one_error_each(sessions):  # noqa: F811
    a, b = sessions
    image = CameraPublisher(a, "t/view/color", encoding="rgb8", frame_id="cam")
    disparity = CameraPublisher(a, "t/view/disparity", encoding="32FC1", frame_id="cam", codec=None)
    cloud = PointCloudPublisher(a, "t/view/points", fields=XYZRGB[0], point_step=16, frame_id="cam")
    try:
        with pytest.raises(view.ViewError, match=r"t/view/nothing: no publisher answers within 0.5 s; image streams "
                                                 r"found: .*t/view/color"):
            view.View(b, ["t/view/nothing"], ("127.0.0.1", 0), seconds=0.5)
        with pytest.raises(view.ViewError, match="t/view/points: a point cloud stream"):
            view.View(b, ["t/view/points"], ("127.0.0.1", 0))
        with pytest.raises(view.ViewError, match="t/view/disparity: no compressed transport"):
            view.View(b, ["t/view/color", "t/view/disparity"], ("127.0.0.1", 0))
        with pytest.raises(view.ViewError, match="name the streams"):
            view.View(b, [], ("127.0.0.1", 0))
        with socket.socket() as taken:
            taken.bind(("127.0.0.1", 0))
            taken.listen()
            port = taken.getsockname()[1]
            with pytest.raises(view.ViewError, match=f"cannot listen on 127.0.0.1:{port} .*the port is taken"):
                view.View(b, ["t/view/color"], ("127.0.0.1", port))
        with pytest.raises(view.ViewError, match="ADDRESS:PORT"):
            view.parse_listen("8090")
    finally:
        cloud.close()
        disparity.close()
        image.close()


def cli(*args):
    return [sys.executable, "-m", "xblick.cli", "view", *args]


@pytest.fixture
def remote():
    """A session that listens, with the streams t/view/cli and t/view/cli2: the command line connects to it."""
    port = free_port()
    listening = open_session(listen=[f"127.0.0.1:{port}"])
    publishers = [CameraPublisher(listening, base, encoding="rgb8", frame_id="cam")
                  for base in ("t/view/cli", "t/view/cli2")]
    yield port, publishers[0]
    for publisher in publishers:
        publisher.close()
    listening.close()


def stand_in(directory, name, calls, wait=True):
    """An executable ``name`` that appends its arguments to ``calls`` (and its pid with them to ``calls``.pids) and
    then, with ``wait``, waits as VLC does: exec keeps the pid, so ending that pid closes the "window". A browser
    command returns at once: it hands the page to the running browser, and Python's webbrowser waits for it."""
    directory.mkdir(exist_ok=True)
    (directory / name).write_text(f'#!/bin/sh\necho "$@" >> {calls}\necho "$$ $@" >> {calls}.pids\n'
                                  + ("exec sleep 60\n" if wait else ""))
    (directory / name).chmod(0o755)


def environment(tmp_path, vlc: bool, display: bool) -> dict:
    """A PATH with Python, sh and sleep, and with ``vlc`` a stand-in VLC; BROWSER a stand-in browser (Python's
    webbrowser takes it first), so that no real one starts; with ``display`` a DISPLAY, else neither DISPLAY nor
    WAYLAND_DISPLAY. The stand-ins write their arguments to tmp_path/vlc_calls and tmp_path/browser_calls."""
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir(exist_ok=True)
    for tool in (sys.executable, "/bin/sh", "/bin/sleep"):
        os.symlink(tool, bin_dir / os.path.basename(tool))
    if vlc:
        stand_in(bin_dir, "vlc", tmp_path / "vlc_calls")
    stand_in(tmp_path / "browser", "browser", tmp_path / "browser_calls", wait=False)
    env = {k: v for k, v in os.environ.items() if k not in ("DISPLAY", "WAYLAND_DISPLAY")}
    env.update(PATH=str(bin_dir), BROWSER=str(tmp_path / "browser" / "browser"))
    if display:
        env["DISPLAY"] = ":77"
    return env


def calls(tmp_path, name) -> list[str]:
    path = tmp_path / f"{name}_calls"
    return path.read_text().splitlines() if path.exists() else []


def interrupt(process, sig=signal.SIGINT, timeout=3.0) -> str:
    started = time.monotonic()
    process.send_signal(sig)
    _, err = process.communicate(timeout=10)
    assert process.returncode == 0 and time.monotonic() - started < timeout and "Traceback" not in (err or ""), err
    return err


def addresses(port, *bases, host="127.0.0.1"):
    """The start lines up to the player's."""
    reach = ("(this PC only; --http 0.0.0.0:PORT opens it to the network)" if host == "127.0.0.1" else
             "(open to the network: anyone who can reach this address can watch, there is no password)")
    return ([f"xblick view: serving {len(bases)} stream(s) on http://{host}:{port}/ {reach}\n"]
            + [f"xblick view: {b}: http://127.0.0.1:{port}/{b}.mjpg\n" for b in bases]
            + [f"xblick view: all of them: http://127.0.0.1:{port}/\n"]
            + [f"xblick view: by hand: vlc --network-caching=100 http://127.0.0.1:{port}/{b}.mjpg\n" for b in bases])


def headless_line(port):
    host = socket.gethostname()
    return ("xblick view: this system has no display (neither DISPLAY nor WAYLAND_DISPLAY is set), so no player is "
            f"started; open them from another PC: start with --http 0.0.0.0:{port} (no password), or forward the port "
            f"with ssh -L {port}:127.0.0.1:{port} {host} and open http://127.0.0.1:{port}/ there\n")


SERVING = "xblick view: serving; Ctrl-C to stop\n"  # without a terminal
SERVING_KEYS = "xblick view: serving; press v for VLC, b for the browser view, Ctrl-C to end\n"  # on a terminal
NOTHING = "xblick view: this system has no display (neither DISPLAY nor WAYLAND_DISPLAY is set): nothing started\n"


class Command:
    """``xblick view`` as a subprocess with stdin and stdout pipes (no terminal) or a pseudo-terminal."""

    def __init__(self, zenoh_port, *args, env, terminal=False, stdin=subprocess.DEVNULL):
        self.port = free_port()
        argv = cli(*args, "--http", f"127.0.0.1:{self.port}", "--connect", f"127.0.0.1:{zenoh_port}")
        self.out = ""
        if terminal:
            self.master, slave = pty.openpty()
            self.saved = termios.tcgetattr(slave)
            self.slave = slave
            self.process = subprocess.Popen(argv, stdin=slave, stdout=slave, stderr=subprocess.PIPE, text=True,
                                            env=env)
        else:
            self.master = self.slave = None
            self.process = subprocess.Popen(argv, stdin=stdin, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                            text=True, env=env)

    def until(self, text: str, timeout=20.0) -> list[str]:
        """The lines printed until (and with) the first line that contains ``text``; also a prompt without its
        newline."""
        lines = []
        if self.master is None:
            end = time.monotonic() + timeout
            while not lines or text not in lines[-1]:
                assert time.monotonic() < end and self.process.poll() is None, "".join(lines)
                lines.append(self.process.stdout.readline())
            return lines
        end = time.monotonic() + timeout
        while text not in self.out:
            assert time.monotonic() < end, self.out
            if select.select([self.master], [], [], 0.2)[0]:
                self.out += os.read(self.master, 4096).decode(errors="replace").replace("\r\n", "\n")
        cut = self.out.index(text) + len(text)
        cut = self.out.find("\n", cut) + 1 or len(self.out)
        shown, self.out = self.out[:cut], self.out[cut:]
        return shown.splitlines(keepends=True)

    def key(self, k: str):
        os.write(self.master, k.encode())

    def serves(self):
        with urllib.request.urlopen(f"http://127.0.0.1:{self.port}/", timeout=10) as response:
            assert "t/view/cli.mjpg" in response.read().decode()

    def mode_restored(self):
        assert termios.tcgetattr(self.slave) == self.saved  # cbreak undone

    def close(self):
        if self.process.poll() is None:
            self.process.kill()
            self.process.communicate()
        for fd in (self.master, self.slave):
            if fd is not None:
                os.close(fd)


@pytest.fixture
def command(remote):
    made = []

    def make(*args, **kwargs):
        made.append(Command(remote[0], *args, **kwargs))
        return made[-1]

    yield make
    for c in made:
        c.close()


TWO = ("t/view/cli", "t/view/cli2")


def test_vlc_starts_one_window_per_stream(command, tmp_path):
    c = command(*TWO, env=environment(tmp_path, vlc=True, display=True))
    lines = c.until("serving;")
    assert lines == addresses(c.port, *TWO) + ["xblick view: started VLC (2 window(s))\n", SERVING], "".join(lines)
    wait_until(lambda: len(calls(tmp_path, "vlc")) == 2, timeout=10.0)
    assert sorted(calls(tmp_path, "vlc")) == [f"--network-caching=100 http://127.0.0.1:{c.port}/t/view/cli.mjpg",
                                              f"--network-caching=100 http://127.0.0.1:{c.port}/t/view/cli2.mjpg"]
    interrupt(c.process)  # the stand-ins end with it (communicate would otherwise wait for their pipes)
    assert calls(tmp_path, "browser") == []


def test_closing_the_vlc_windows_leaves_it_serving(command, tmp_path):
    env = environment(tmp_path, vlc=False, display=True)
    stand_in(tmp_path / "bin", "vlc", tmp_path / "vlc_calls", wait=False)  # a VLC whose window is closed at once
    c = command(*TWO, env=env)
    lines = c.until("were closed")
    assert lines[-3:] == ["xblick view: started VLC (2 window(s))\n", SERVING,
                          "xblick view: the VLC windows were closed; still serving. "
                          f"At http://127.0.0.1:{c.port}/ until Ctrl-C.\n"], "".join(lines)
    c.serves()
    interrupt(c.process)


def test_without_vlc_and_without_a_terminal_it_does_not_ask_and_leaves_stdin_alone(command, tmp_path):
    r, w = os.pipe()
    os.write(w, b"vb\nyes\n")  # input that is not for it
    os.close(w)
    try:
        c = command("t/view/cli", env=environment(tmp_path, vlc=False, display=True), stdin=r)
        lines = c.until("serving;")
        assert lines == addresses(c.port, "t/view/cli") + [
            "xblick view: VLC is not installed (no vlc on PATH); --browser opens the browser view\n", SERVING], \
            "".join(lines)
        c.serves()
        time.sleep(1.0)
        interrupt(c.process)
        assert calls(tmp_path, "browser") == [] and calls(tmp_path, "vlc") == []
        assert os.read(r, 100) == b"vb\nyes\n"  # nothing of the pipe's input was read
    finally:
        os.close(r)


@pytest.mark.parametrize("answer", ["y", "n", ""])
def test_without_vlc_on_a_terminal_it_asks_for_the_browser(command, tmp_path, answer):
    c = command("t/view/cli", env=environment(tmp_path, vlc=False, display=True), terminal=True)
    shown = c.until("[y/N] ")
    assert shown[-2:] == ["xblick view: VLC is not installed (no vlc on PATH)\n",
                          "xblick view: open the browser view instead? [y/N] "], "".join(shown)
    c.key(f"{answer}\n")
    assert c.until("serving;")[-1] == SERVING_KEYS
    c.serves()  # either way it keeps serving
    if answer == "y":
        wait_until(lambda: calls(tmp_path, "browser") == [f"http://127.0.0.1:{c.port}/"], timeout=10.0)
    else:
        time.sleep(0.3)
        assert calls(tmp_path, "browser") == []
    interrupt(c.process)
    c.mode_restored()


def test_browser_opens_the_grid_page(command, tmp_path):
    c = command("t/view/cli", "--browser", env=environment(tmp_path, vlc=True, display=True))
    assert c.until("serving;")[-1] == SERVING
    wait_until(lambda: calls(tmp_path, "browser") == [f"http://127.0.0.1:{c.port}/"], timeout=10.0)
    interrupt(c.process)
    assert calls(tmp_path, "vlc") == []


def test_keys_v_reopens_closed_windows_only_and_b_opens_the_browser(command, tmp_path):
    env = environment(tmp_path, vlc=False, display=True)
    stand_in(tmp_path / "bin", "vlc", tmp_path / "vlc_calls")  # waits: a window that stays open
    c = command(*TWO, env=env, terminal=True)
    assert c.until("serving;")[-2:] == ["xblick view: started VLC (2 window(s))\n", SERVING_KEYS]
    wait_until(lambda: len(calls(tmp_path, "vlc")) == 2, timeout=10.0)
    c.key("v")  # both still open: nothing started
    assert c.until("VLC")[-1] == "xblick view: the VLC windows are open\n"
    first = sorted(calls(tmp_path, "vlc"))
    pids = (tmp_path / "vlc_calls.pids").read_text().splitlines()
    os.kill(int(next(line for line in pids if line.endswith("/t/view/cli2.mjpg")).split()[0]), signal.SIGTERM)
    time.sleep(1.0)  # the window of cli2 closed; that of cli stays open
    c.key("v")
    assert c.until("VLC")[-1] == "xblick view: started VLC (1 window(s))\n"
    wait_until(lambda: len(calls(tmp_path, "vlc")) == 3, timeout=10.0)
    assert calls(tmp_path, "vlc")[2] == f"--network-caching=100 http://127.0.0.1:{c.port}/t/view/cli2.mjpg"
    assert first == sorted(calls(tmp_path, "vlc")[:2])  # the same arguments as at the start
    c.key("b")
    wait_until(lambda: calls(tmp_path, "browser") == [f"http://127.0.0.1:{c.port}/"], timeout=10.0)
    interrupt(c.process)
    c.mode_restored()


def test_keys_after_all_windows_closed(command, tmp_path):
    env = environment(tmp_path, vlc=False, display=True)
    stand_in(tmp_path / "bin", "vlc", tmp_path / "vlc_calls", wait=False)  # windows that close at once
    c = command(*TWO, env=env, terminal=True)
    closed = c.until("were closed")
    assert closed[-1] == ("xblick view: the VLC windows were closed; still serving. Press v for VLC, b for the "
                          "browser view, Ctrl-C to end.\n"), "".join(closed)
    c.key("v")
    assert c.until("VLC (")[-1] == "xblick view: started VLC (2 window(s))\n"
    assert c.until("were closed")[-1].startswith("xblick view: the VLC windows were closed")  # once more
    wait_until(lambda: len(calls(tmp_path, "vlc")) == 4, timeout=10.0)
    assert sorted(calls(tmp_path, "vlc")[:2]) == sorted(calls(tmp_path, "vlc")[2:])  # the same arguments again
    interrupt(c.process, signal.SIGTERM)  # SIGTERM restores the terminal as well
    c.mode_restored()


@pytest.mark.parametrize("vlc, option, terminal", [(True, None, False), (False, None, False), (False, None, True),
                                                   (True, None, True), (True, "--browser", False),
                                                   (False, "--browser", True)])
def test_headless_starts_no_player_and_asks_nothing(command, tmp_path, vlc, option, terminal):
    args = ("t/view/cli",) + ((option,) if option else ())
    c = command(*args, env=environment(tmp_path, vlc=vlc, display=False), terminal=terminal)
    lines = c.until("serving;")
    assert "".join(lines).count("[y/N]") == 0
    assert lines[-len(addresses(c.port, "t/view/cli")) - 2:] == addresses(c.port, "t/view/cli") + [
        headless_line(c.port), SERVING_KEYS if terminal else SERVING], "".join(lines)
    c.serves()
    if terminal:
        for k in "vb":
            c.key(k)
            assert c.until("nothing started")[-1] == NOTHING
    interrupt(c.process)
    assert calls(tmp_path, "vlc") == [] and calls(tmp_path, "browser") == []
    if terminal:
        c.mode_restored()


def test_no_player_still_has_the_keys(command, tmp_path):
    c = command("t/view/cli", "--no-player", env=environment(tmp_path, vlc=True, display=True), terminal=True)
    assert c.until("serving;")[-1] == SERVING_KEYS
    assert calls(tmp_path, "vlc") == []
    c.key("v")
    assert c.until("VLC (")[-1] == "xblick view: started VLC (1 window(s))\n"
    wait_until(lambda: calls(tmp_path, "vlc") == [f"--network-caching=100 http://127.0.0.1:{c.port}/t/view/cli.mjpg"])
    interrupt(c.process)
    c.mode_restored()


@pytest.mark.parametrize("host", ["127.0.0.1", "0.0.0.0"])
def test_no_player_prints_the_addresses_and_ends_on_ctrl_c(remote, host):
    zenoh_port, publisher = remote
    port = free_port()
    process = subprocess.Popen(cli("t/view/cli", "--no-player", "--http", f"{host}:{port}", "--connect",
                                   f"127.0.0.1:{zenoh_port}"), stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE, text=True)
    try:
        expected = addresses(port, "t/view/cli", host=host) + [SERVING]
        lines = [process.stdout.readline() for _ in expected]
        assert lines == expected, "".join(lines)
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/t/view/cli.mjpg", timeout=10) as response:
            end = time.monotonic() + 10
            while not select.select([response.fp], [], [], 0.05)[0] and time.monotonic() < end:
                publisher.publish(pattern(2))
            assert read_frame(response)
            interrupt(process)  # with a client connected
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate()


def test_the_help_says_there_is_no_password():
    out = subprocess.run(cli("--help"), capture_output=True, text=True, timeout=60).stdout
    assert "WITHOUT A PASSWORD" in " ".join(out.split())
