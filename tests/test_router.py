"""xblick router (router.py): the router from the Zenoh library, as its own program. Programs in peer and client mode
find each other through it as through zenohd: a stream that switches to its ring on this host, a query and a command,
liveliness tokens, the admin space the terminal interface reads; a taken port; Ctrl-C and SIGTERM end it, exit 0."""

import signal
import subprocess
import sys
import time

import numpy as np
import pytest
from test_camera import free_port, pattern

from xblick import survey
from xblick.camera import CameraPublisher, ImageSubscriber
from xblick.parameters import ParameterClient, Parameters, find_nodes
from xblick.session import open_session


def start(port: int) -> subprocess.Popen:
    process = subprocess.Popen([sys.executable, "-m", "xblick.cli", "router", "--listen", f"tcp/127.0.0.1:{port}"],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    line = process.stdout.readline()
    assert line == (f"xblick router: listening on tcp/127.0.0.1:{port}; programs on this PC connect with --connect "
                    f"tcp/127.0.0.1:{port}; other PCs cannot connect (it listens on a loopback address only; --listen "
                    f"tcp/0.0.0.0:{port} opens it to them); Ctrl-C to stop\n"), line + process.stderr.read()
    return process


@pytest.fixture
def router():
    port = free_port()
    process = start(port)
    yield port
    process.send_signal(signal.SIGTERM)
    process.communicate(timeout=10)
    assert process.returncode == 0


@pytest.mark.parametrize("mode", ["peer", "client"])
def test_programs_find_each_other_through_it(router, mode):
    endpoint = [f"tcp/127.0.0.1:{router}"]
    publisher_session = open_session(connect=endpoint, listen=["127.0.0.1:0"])
    reader = open_session(connect=endpoint, mode=mode, **({"listen": ["127.0.0.1:0"]} if mode == "peer" else {}))
    publisher = CameraPublisher(publisher_session, "t/router/color", encoding="rgb8", frame_id="cam")
    params = Parameters(publisher_session, "node", "t/router")
    params.declare("x", 0.0)
    try:
        images = ImageSubscriber(reader, "t/router/color", transport="auto")
        frame, end = None, time.monotonic() + 20
        while (frame is None or images.transport != "shm") and time.monotonic() < end:
            publisher.publish(pattern(3))
            frame = images.latest(timeout=0.1)
        assert images.transport == "shm" and frame is not None  # the ring of this host, through the router
        np.testing.assert_array_equal(frame.image, pattern(3))
        assert "t/router/node" in find_nodes(reader, 2.0)  # liveliness
        client = ParameterClient(reader, "t/router/node", 2.0)
        assert client.get(["x"])[0].value() == 0.0  # a read
        assert client.set({"x": 2.0})[0].successful and params["x"] == 2.0  # a command
        network = survey.query_network(reader, 2.0)  # the admin space, as the terminal interface reads it
        assert len(network.routers) == 1
        images.close()
    finally:
        params.close()
        publisher.close()
        reader.close()
        publisher_session.close()


def test_a_taken_port_is_one_line(router):
    out = subprocess.run([sys.executable, "-m", "xblick.cli", "router", "--listen", f"tcp/127.0.0.1:{router}"],
                         capture_output=True, text=True, timeout=30)
    assert out.returncode == 1 and out.stdout == ""
    assert out.stderr == (f"xblick router: cannot listen on tcp/127.0.0.1:{router}: the port is taken; a router "
                          "(zenohd or xblick router) may run there already, else give another one with --listen\n")


def test_ctrl_c_ends_it_with_exit_0():
    process = start(free_port())
    started = time.monotonic()
    process.send_signal(signal.SIGINT)
    out, err = process.communicate(timeout=10)
    assert process.returncode == 0 and time.monotonic() - started < 3.0 and "Traceback" not in err
