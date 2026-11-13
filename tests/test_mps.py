"""blick.mps without a GPU: check() passes without CUDA_MPS_PIPE_DIRECTORY and refuses one where no daemon
answers (none there, or the socket an ended daemon left); serve() refuses a pipe directory too long for its sockets."""

import socket

import pytest

from blick import mps


def test_check_without_mps(monkeypatch):
    monkeypatch.delenv(mps.PIPE_ENV, raising=False)
    mps.check()


def test_check_refuses_a_directory_without_daemon(monkeypatch, tmp_path):
    monkeypatch.setenv(mps.PIPE_ENV, str(tmp_path))
    with pytest.raises(RuntimeError, match="no MPS control daemon answers"):
        mps.check()


def test_check_refuses_the_socket_of_an_ended_daemon(monkeypatch, tmp_path):
    with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as s:
        s.bind(str(tmp_path / "control"))  # bound, never listening: as the socket file a daemon leaves behind
    monkeypatch.setenv(mps.PIPE_ENV, str(tmp_path))
    with pytest.raises(RuntimeError, match="no MPS control daemon answers"):
        mps.check()


def test_check_passes_when_the_daemon_answers(monkeypatch, tmp_path):
    with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as s:
        s.bind(str(tmp_path / "control"))
        s.listen()
        monkeypatch.setenv(mps.PIPE_ENV, str(tmp_path))
        mps.check()


def test_serve_refuses_a_long_pipe_directory(tmp_path):
    with pytest.raises(ValueError, match="Unix sockets take"):
        mps.serve(str(tmp_path / ("x" * 120)))


# ----------------------------------------------------------------------------- on Zenoh, with a stand-in daemon

FAKE_CONTROL = '''#!{python}
# a stand-in for nvidia-cuda-mps-control (tests/test_mps.py): with -f the daemon, listening on <pipe>/control until
# told quit; without, one command from stdin (quit, get_server_list, get_client_list <pid>)
import os, signal, socket, sys
pipe = os.environ["CUDA_MPS_PIPE_DIRECTORY"]
pid_file = os.path.join(pipe, "fake.pid")
if sys.argv[1:] == ["-f"]:
    s = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    path = os.path.join(pipe, "control")
    if os.path.exists(path):
        os.unlink(path)
    s.bind(path)
    s.listen()
    s.settimeout(0.2)
    with open(pid_file, "w") as f:
        f.write(str(os.getpid()))
    signal.signal(signal.SIGUSR1, lambda *a: sys.exit(0))
    signal.signal(signal.SIGINT, signal.SIG_IGN)  # as the real daemon
    while True:
        try:
            s.accept()[0].close()
        except socket.timeout:
            pass
command = sys.stdin.read().split()
if command == ["quit"]:
    os.kill(int(open(pid_file).read()), signal.SIGUSR1)
elif command == ["get_server_list"]:
    print(4242)
elif command[:1] == ["get_client_list"] and command[1:] == ["4242"]:
    print(open(pid_file).read())
else:
    sys.exit(f"unknown command {{command}}")
'''
FAKE_SMI = '''#!/bin/sh
echo "0, GPU-00000000-fake, Fake GPU"
'''


@pytest.fixture
def fake_tools(tmp_path):
    """PATH with stand-ins for nvidia-cuda-mps-control and nvidia-smi."""
    import os
    import sys

    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    for name, text in (("nvidia-cuda-mps-control", FAKE_CONTROL.format(python=sys.executable)),
                       ("nvidia-smi", FAKE_SMI)):
        (bin_dir / name).write_text(text)
        (bin_dir / name).chmod(0o755)
    return {**os.environ, "PATH": f"{bin_dir}:{os.environ['PATH']}"}


class Lines:
    """A child's output lines, read in a thread (readline alone would block a test that waits for one)."""

    def __init__(self, process):
        import threading

        self.lines, self._process = [], process
        threading.Thread(target=lambda: [self.lines.append(line.rstrip("\n")) for line in process.stdout],
                         daemon=True).start()

    def wait_for(self, text, timeout=20.0):
        import time

        end = time.monotonic() + timeout
        while not any(text in line for line in self.lines):
            if time.monotonic() > end or self._process.poll() is not None:
                raise TimeoutError(f"no line with {text!r} in {self.lines}")
            time.sleep(0.05)


def start_mps(env, pipe, *zenoh_args):
    import subprocess
    import sys

    process = subprocess.Popen([sys.executable, "-m", "blick.cli", "mps", "--pipe-dir", str(pipe), "--name",
                                "testpc", *zenoh_args], env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               text=True)
    return process, Lines(process)


def stop_mps(process, lines):
    import signal

    process.send_signal(signal.SIGTERM)
    assert process.wait(20) == 0, lines.lines
    lines.wait_for("blick mps: quit")


def test_mps_announces_itself_on_zenoh(fake_tools, tmp_path):
    from test_camera import free_port, wait_until
    from test_tui import open_router

    from blick import survey, tui
    from blick.session import open_session

    port = free_port()
    router = open_router(port)
    pipe = tmp_path / "p"
    process, lines = start_mps(fake_tools, pipe, "--connect", f"127.0.0.1:{port}", "--listen", "127.0.0.1:0")
    try:
        lines.wait_for(mps.READY)
        assert any(line == "blick mps: on Zenoh as durchblick/mps/testpc (status durchblick/mps/testpc/status)"
                   for line in lines.lines), lines.lines
        daemon_pid = int((pipe / "fake.pid").read_text())
        with open_session(connect=[f"127.0.0.1:{port}"], listen=["127.0.0.1:0"]) as z:
            # a new session learns of the queryable over time (CLAUDE.md): asked again until it answers
            wait_until(lambda: "durchblick/mps/testpc/status" in tui.statuses(z, 0.5), timeout=10)
            status = tui.statuses(z, 2.0)["durchblick/mps/testpc/status"]
            assert status["kind"] == "mps" and status["v"] == mps.STATUS_VERSION and status["name"] == "testpc"
            assert status["pipe_dir"] == str(pipe.resolve()) and status["daemon_pid"] == daemon_pid
            assert status["gpus"] == [{"index": 0, "uuid": "GPU-00000000-fake", "name": "Fake GPU"}]
            [server] = status["servers"]
            assert server["pid"] == 4242 and [c["pid"] for c in server["clients"]] == [daemon_pid]
            assert "1 clients" in tui._status_line(status) and "Fake GPU" in tui._status_line(status)
            assert "durchblick/mps/testpc" in survey.liveliness_tokens(z)
            network = survey.query_network(z)
            assert "durchblick/mps/testpc" in {n.label for n in network.nodes.values()}
        stop_mps(process, lines)
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        router.close()


def test_mps_without_a_router_runs_and_says_so(fake_tools, tmp_path):
    from test_camera import free_port
    from test_tui import open_router

    port = free_port()
    process, lines = start_mps(fake_tools, tmp_path / "p", "--connect", f"127.0.0.1:{port}", "--listen",
                               "127.0.0.1:0")
    router = None
    try:
        lines.wait_for(mps.READY)
        assert any("no Zenoh router reached yet" in line for line in lines.lines), lines.lines
        router = open_router(port)  # Zenoh goes on connecting
        lines.wait_for("blick mps: Zenoh router reached: announced")
        stop_mps(process, lines)
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        if router is not None:
            router.close()


def test_mps_with_a_wrong_zenoh_config_ends_before_the_daemon(fake_tools, tmp_path):
    import subprocess
    import sys

    config = tmp_path / "zenoh.json5"
    config.write_text("{ mode: ")
    pipe = tmp_path / "p"
    result = subprocess.run([sys.executable, "-m", "blick.cli", "mps", "--pipe-dir", str(pipe), "--config",
                             str(config)], env=fake_tools, capture_output=True, text=True, timeout=60, check=False)
    assert result.returncode != 0 and "expected" in result.stderr and not (pipe / "fake.pid").exists()
