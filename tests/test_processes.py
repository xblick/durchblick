"""The commands as separate processes behind ``xblick router``, in the default (peer) mode, as a user starts them:
a publisher, then ``list``, ``hz`` (network and the move to the shm ring), ``param`` (liveliness and a query), a
client-mode session and ``xblick view``; the default port without any option (skipped when 7447 is taken); and the
long-running commands ending on SIGTERM and on SIGINT as background jobs of a script (a non-interactive shell starts
those with SIGINT ignored).

0.5.3 found nothing here: a peer that only connected did not listen, and a Zenoh 1.10 router does not relay between
peers (they reach each other directly), while the earlier router tests used sessions that listened."""

import os
import shutil
import signal
import socket
import subprocess
import sys
import time
import urllib.request

import pytest
from test_camera import free_port
from test_view import read_frame

try:
    import av

    LAVFI = "lavfi" in av.formats_available
except ImportError:
    LAVFI = False

XBLICK = [sys.executable, "-m", "xblick.cli"]
ENV = {**os.environ, "PYTHONUNBUFFERED": "1"}


def start(*args, env=ENV) -> subprocess.Popen:
    return subprocess.Popen([*XBLICK, *args], stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, env=env)


def run(*args, timeout=60) -> subprocess.CompletedProcess:
    return subprocess.run([*XBLICK, *args], stdin=subprocess.DEVNULL, capture_output=True, text=True,
                          timeout=timeout, env=ENV)


def first_line(process: subprocess.Popen, prefix: str) -> str:
    line = process.stdout.readline()
    assert line.startswith(prefix), line + (process.stdout.read() if process.poll() is not None else "")
    return line


def end(process: subprocess.Popen, sig=signal.SIGTERM) -> int:
    if process.poll() is None:
        process.send_signal(sig)
    try:
        process.communicate(timeout=10)
    except subprocess.TimeoutExpired:
        process.kill()
        process.communicate()
        raise AssertionError(f"{process.args[3:5]} did not end on {sig!r}")
    return process.returncode


@pytest.fixture
def router():
    """``xblick router`` on a free loopback port, and its endpoint."""
    port = free_port()
    endpoint = f"tcp/127.0.0.1:{port}"
    process = start("router", "--listen", endpoint)
    line = first_line(process, f"xblick router: listening on {endpoint}; ")
    assert ("other PCs cannot connect (it listens on a loopback address only; --listen tcp/0.0.0.0:"
            f"{port} opens it to them)") in line
    yield endpoint
    assert end(process) == 0


@pytest.fixture
def camera(router):
    """``xblick test-camera`` behind the router, in peer mode with only --connect, publishing test/camera/color."""
    process = start("test-camera", "--connect", router, "--size", "160", "120")
    first_line(process, "xblick test-camera:")
    time.sleep(1.0)
    yield router
    assert end(process) == 0


def test_peers_find_each_other_through_the_router(camera):
    r = camera
    out = run("list", "--connect", r)
    assert out.returncode == 0 and out.stdout.startswith("test/camera/color "), out.stdout + out.stderr
    out = run("list", "--connect", r, "--zenoh-mode", "client")  # a client as well
    assert out.returncode == 0 and out.stdout.startswith("test/camera/color "), out.stdout + out.stderr
    out = run("list", "--traffic", "--connect", r)
    assert out.returncode == 0 and "test/camera/color/image_raw " in out.stdout, out.stdout + out.stderr
    out = run("hz", "test/camera/color", "--transport", "raw", "--seconds", "3", "--connect", r)  # over the network
    assert out.stdout.startswith("transport: raw") and " Hz " in out.stdout and "no frames" not in out.stdout, \
        out.stdout + out.stderr
    out = run("hz", "test/camera/color", "--seconds", "3", "--connect", r)  # auto: to the ring of this host
    assert out.stdout.startswith("transport: shm") and " Hz " in out.stdout and "no frames" not in out.stdout, \
        out.stdout + out.stderr


@pytest.mark.skipif(not LAVFI, reason="PyAV without lavfi (xblick[cam])")
def test_a_node_answers_queries_through_the_router(router):
    node = start("cam", "testsrc=size=160x120:rate=15,realtime", "--format", "lavfi", "--stream", "t/lav/color",
                 "--connect", router)
    try:
        first_line(node, "xblick cam:")
        time.sleep(1.0)
        out = run("param", "--nodes", "--connect", router)  # liveliness tokens
        assert out.returncode == 0 and "t/lav" in out.stdout.split(), out.stdout + out.stderr
        out = run("param", "--node", "t/lav", "--get", "--connect", router)  # a query and its reply
        assert out.returncode == 0 and out.stdout.strip(), out.stdout + out.stderr
    finally:
        assert end(node) == 0


def test_view_serves_frames_through_the_router(camera):
    port = free_port()
    env = {k: v for k, v in ENV.items() if k not in ("DISPLAY", "WAYLAND_DISPLAY")}  # headless: no player
    view = start("view", "test/camera/color", "--http", f"127.0.0.1:{port}", "--connect", camera, env=env)
    try:
        lines = [view.stdout.readline() for _ in range(6)]
        assert lines[0].startswith(f"xblick view: serving 1 stream(s) on http://127.0.0.1:{port}/"), "".join(lines)
        assert lines[4].startswith("xblick view: this system has no display"), "".join(lines)
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/", timeout=10) as response:
            assert '<img src="/test/camera/color.mjpg"' in response.read().decode()
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/test/camera/color.mjpg", timeout=10) as response:
            frames = [read_frame(response) for _ in range(5)]
        assert all(f.startswith(b"\xff\xd8") for f in frames) and len(set(frames)) > 1
    finally:
        assert end(view) == 0


def port_free(port: int) -> bool:
    """Nothing listens on the port (a connection attempt is refused); a bind test would also fail on connections
    of earlier tests that are still in TIME_WAIT."""
    with socket.socket() as s:
        return s.connect_ex(("127.0.0.1", port)) != 0


@pytest.mark.skipif(not port_free(7447), reason="port 7447 is taken (a router runs on this PC)")
def test_the_default_port_without_any_option():
    router = start("router")
    try:
        first_line(router, "xblick router: listening on tcp/0.0.0.0:7447; programs on this PC find it without options")
        camera = start("test-camera", "--size", "160", "120")
        try:
            first_line(camera, "xblick test-camera:")
            time.sleep(1.0)
            out = run("list")
            assert out.returncode == 0 and out.stdout.startswith("test/camera/color "), out.stdout + out.stderr
            out = run("hz", "test/camera/color", "--seconds", "2")
            assert " Hz " in out.stdout and "no frames" not in out.stdout, out.stdout + out.stderr
        finally:
            assert end(camera) == 0
    finally:
        assert end(router) == 0


def commands(router: str) -> dict:
    return {
        "router": ["router", "--listen", f"tcp/127.0.0.1:{free_port()}"],
        "test-camera": ["test-camera", "--connect", router, "--namespace", "sig"],
        "view": ["view", "test/camera/color", "--no-player", "--http", f"127.0.0.1:{free_port()}", "--connect",
                 router],
        "cam": ["cam", "testsrc=size=160x120:rate=15,realtime", "--format", "lavfi", "--stream", "sig/lav/color",
                "--connect", router],
    }


@pytest.mark.skipif(shutil.which("bash") is None, reason="needs bash")
@pytest.mark.parametrize("name", ["router", "test-camera", "view", "cam"])
@pytest.mark.parametrize("sig", ["INT", "TERM"])
def test_a_background_job_of_a_script_ends_on_sigint_and_sigterm(camera, name, sig):
    if name == "cam" and not LAVFI:
        pytest.skip("PyAV without lavfi (xblick[cam])")
    argv = " ".join(f"'{a}'" for a in [*XBLICK, *commands(camera)[name]])
    script = ("sleep 30 &\ngrep '^SigIgn' /proc/$!/status\nkill $!\n"  # a background job: SIGINT ignored
              f"{argv} > /dev/null 2>&1 &\npid=$!\nsleep 4\nkill -{sig} $pid\n"
              # at most 5 s; a job that does not end is killed, so that no test leaves it running
              "for i in $(seq 50); do kill -0 $pid 2>/dev/null || break; sleep 0.1; done\n"
              "if kill -0 $pid 2>/dev/null; then kill -KILL $pid; wait $pid; echo exit=hung; "
              "else wait $pid; echo exit=$?; fi\n")
    started = time.monotonic()
    out = subprocess.run(["bash", "-c", script], capture_output=True, text=True, timeout=60, env=ENV)
    ignored = int(out.stdout.split()[1], 16)
    assert ignored & (1 << (signal.SIGINT - 1)), out.stdout  # the shell did start background jobs so
    assert out.stdout.strip().endswith("exit=0"), out.stdout + out.stderr
    assert time.monotonic() - started < 4 + 5  # ended within some seconds of the signal


CPP_BUILD = os.environ.get("XBLICK_CPP_BUILD")


@pytest.mark.skipif(not CPP_BUILD, reason="XBLICK_CPP_BUILD (the C++ build directory) is not set")
def test_a_cpp_peer_behind_the_router(router):
    """The C++ session rule is Python's: a peer with --connect only listens on a free loopback port."""
    camera = subprocess.Popen([os.path.join(CPP_BUILD, "xblick-cpp-camera"), "--connect", router, "--base",
                               "t/cppr/color", "--size", "64", "48", "--fps", "30", "--seconds", "30"],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        time.sleep(1.5)
        out = run("list", "--connect", router)
        assert out.returncode == 0 and out.stdout.startswith("t/cppr/color "), out.stdout + out.stderr
        out = run("hz", "t/cppr/color", "--transport", "raw", "--seconds", "2", "--connect", router)
        assert " Hz " in out.stdout and "no frames" not in out.stdout, out.stdout + out.stderr
    finally:
        assert end(camera) == 0
