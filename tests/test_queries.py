"""Queries of a session just opened behind a router (the default setup of every command line tool): through the
router the queryables become known to the new session some 0.1-0.3 s later, and a query sent before ends at once
without a reply. Reads are asked again until a first reply (session.ask); each test counts how often the queryable
ran."""

import threading
import time

import pytest
from test_camera import free_port
from test_tui import open_router

from xblick import cli, controller_manager
from xblick.camera import CameraPublisher, wait_for_info
from xblick.msg import ControllerState, ListControllersResponse
from xblick.session import NotReached, ask, ask_once, open_session, replies


class Network:
    """A router, a serving session behind it, and new sessions (as programs) behind it."""

    def __init__(self):
        self.port = free_port()
        self.router = open_router(self.port)
        self.endpoint = [f"tcp/127.0.0.1:{self.port}"]
        self.server = open_session(connect=self.endpoint, listen=["127.0.0.1:0"])
        self._queryables = []
        self.calls: dict[str, int] = {}
        self._lock = threading.Lock()

    def serve(self, key: str, answer=lambda query: b"ok", delay: float = 0.0):
        """A queryable on ``key`` that counts its queries; ``answer(query)`` the reply's payload, None: no reply."""
        def handle(query):
            with self._lock:
                self.calls[key] = self.calls.get(key, 0) + 1
            if delay:
                time.sleep(delay)
            payload = answer(query)
            if payload is not None:
                query.reply(query.key_expr, payload)

        self._queryables.append(self.server.declare_queryable(key, handle))
        self.calls[key] = 0

    def new_session(self):
        return open_session(connect=self.endpoint, listen=["127.0.0.1:0"])

    def close(self):
        for q in self._queryables:
            q.undeclare()
        self.server.close()
        self.router.close()


@pytest.fixture
def network():
    n = Network()
    time.sleep(0.3)
    yield n
    n.close()


def test_a_read_is_asked_again_until_it_is_answered(network):
    network.serve("t/read")
    for _ in range(5):
        with network.new_session() as s:
            assert [r.ok.payload.to_bytes() for r in ask(s, "t/read", 2.0)] == [b"ok"]
    assert network.calls["t/read"] == 5  # one answered query per session: none sent after the first reply


def test_a_single_query_right_after_the_start_finds_nothing(network):
    """The reason for ask: at least once in ten new sessions a single query ends at once without a reply (it did
    8 of 10 times on the test PC)."""
    network.serve("t/once")
    empty = 0
    for _ in range(10):
        with network.new_session() as s:
            empty += not replies(s, "t/once", 2.0)
    assert empty >= 1


def test_xblick_hz_knows_a_point_cloud_stream_from_a_new_session(network):
    publisher = CameraPublisher(network.server, "t/info/color", encoding="rgb8", frame_id="cam")
    try:
        for _ in range(3):
            with network.new_session() as s:
                (record,) = wait_for_info(s, "t/info/color", 2.0)
                assert record["kind"] == "image"
                assert not cli._is_cloud(s, "t/info/color")
    finally:
        publisher.close()


def test_the_controller_managers_reads_from_a_new_session(network):
    state = ControllerState(name="arms", state="active", type="Impedance")
    network.serve("robot/controller_manager/list_controllers",
                  lambda q: bytes(ListControllersResponse([state]).serialize()))
    for _ in range(3):
        with network.new_session() as s:
            assert controller_manager.ControllerManager(s, "robot", timeout=2.0).states() == {"arms": "active"}
    assert network.calls["robot/controller_manager/list_controllers"] == 3


# ---------------------------------------------------------------------------------- commands: exactly once
# A command (a parameter set, a controller manager's create, switch or unload, a torque step, a recorder command) is
# sent exactly once, after the session knows the queryable (session.ask_once). Each test counts the handler's runs.


def test_an_empty_result_does_not_mean_that_nobody_got_the_query(network):
    """Why a command is not asked again: a queryable that drops the query without a reply ends it at once with no
    reply, as a session that knows no queryable does. session.ask (for reads) sends such a query again."""
    network.serve("t/silent", lambda q: None)
    with network.new_session() as s:
        time.sleep(0.5)  # the queryable known
        assert ask(s, "t/silent", 0.6) == []
    assert network.calls["t/silent"] >= 2  # a read is sent again: harmless for a read, wrong for a command


@pytest.mark.parametrize("kind", ["fast", "silent", "slow"])
def test_ask_once_sends_a_command_exactly_once(network, kind):
    """From sessions just opened behind the router: a handler that answers, one that gives no reply, one that answers
    after the timeout. Each ran once per command."""
    answer = None if kind == "silent" else (lambda q: b"done")
    network.serve("t/command", answer, delay=1.0 if kind == "slow" else 0.0)
    for _ in range(5):
        with network.new_session() as s:
            got = ask_once(s, "t/command", 0.4)
            assert [r.ok.payload.to_bytes() for r in got if r.ok is not None] == ([b"done"] if kind == "fast" else [])
    deadline = time.monotonic() + 10  # the handler works the queries one after another (1 s each when slow)
    while network.calls["t/command"] < 5 and time.monotonic() < deadline:
        time.sleep(0.05)
    time.sleep(1.5)  # nothing more comes
    assert network.calls["t/command"] == 5


def test_ask_once_without_a_queryable_sends_nothing(network):
    network.serve("t/elsewhere")
    with network.new_session() as s:
        with pytest.raises(NotReached, match="no queryable for t/command became known within 0.3 s: nothing was sent"):
            ask_once(s, "t/command", 0.3)
    assert network.calls["t/elsewhere"] == 0


def test_a_parameter_set_runs_the_nodes_writer_once(network):
    """set_parameters is a command (a node may act on a set): once per set, also with a writer slower than the
    client's timeout, which then reports "sent once" and does not send it again."""
    from xblick.parameters import ParameterClient, Parameters

    writes = []

    def write(value):
        writes.append(value)
        if value > 5:
            time.sleep(1.0)  # slower than the client's timeout

    params = Parameters(network.server, "node", "t/w")
    params.declare("x", 0.0, write=write)
    try:
        for i in range(1, 4):
            with network.new_session() as s:
                assert ParameterClient(s, "t/w/node", 2.0).set({"x": float(i)})[0].successful
        with network.new_session() as s:
            with pytest.raises(TimeoutError, match="sent once, no answer within 0.3 s"):
                ParameterClient(s, "t/w/node", 0.3).set({"x": 9.0})
        time.sleep(1.5)
        assert writes == [1.0, 2.0, 3.0, 9.0]
    finally:
        params.close()


def test_the_controller_managers_commands_run_once(network):
    from xblick.msg import (CreateControllerResponse, SwitchControllerResponse, TorqueStepResponse,
                            UnloadControllerResponse)

    m = "robot/controller_manager"
    network.serve(f"{m}/create_controller", lambda q: bytes(CreateControllerResponse(True, "").serialize()))
    network.serve(f"{m}/switch_controller", lambda q: bytes(SwitchControllerResponse(True, "switched").serialize()))
    network.serve(f"{m}/unload_controller", lambda q: bytes(UnloadControllerResponse(True).serialize()))
    network.serve("robot/controller/arms/torque_step", lambda q: bytes(TorqueStepResponse(True, "").serialize()))
    for _ in range(3):
        with network.new_session() as s:
            manager = controller_manager.ControllerManager(s, "robot", timeout=2.0)
            manager.create("arms", "Impedance", {"a": 1})
        with network.new_session() as s:
            assert controller_manager.ControllerManager(s, "robot", timeout=2.0).switch(activate=["arms"]) == "switched"
        with network.new_session() as s:
            controller_manager.ControllerManager(s, "robot", timeout=2.0).unload("arms")
        with network.new_session() as s:
            controller_manager.torque_step(s, "robot/controller/arms", "joint1", 1.0, 0.1)
    assert {k: v for k, v in network.calls.items()} == {f"{m}/create_controller": 3, f"{m}/switch_controller": 3,
                                                       f"{m}/unload_controller": 3,
                                                       "robot/controller/arms/torque_step": 3}


def test_a_controller_command_without_an_answer_is_not_repeated(network):
    network.serve("robot/controller_manager/switch_controller", lambda q: None)  # takes it, answers nothing
    with network.new_session() as s:
        with pytest.raises(TimeoutError, match="sent once, no answer"):
            controller_manager.ControllerManager(s, "robot", timeout=0.5).switch(activate=["arms"], timeout=0.0)
    time.sleep(0.5)
    assert network.calls["robot/controller_manager/switch_controller"] == 1


def test_a_recorder_command_runs_once(network):
    import json

    from xblick import recording

    status = {"state": "idle", "play": None}
    network.serve("xblick/rec/status", lambda q: json.dumps(status).encode())
    network.serve("xblick/rec/control", lambda q: json.dumps({"v": recording.REPLY_VERSION, "ok": True,
                                                              "result": {"state": "recording"}}).encode())
    for _ in range(3):
        with network.new_session() as s:
            assert recording.request(s, "rec", "record", timeout=2.0) == {"state": "recording"}
    assert network.calls["xblick/rec/control"] == 3


@pytest.mark.skipif(not __import__("os").environ.get("XBLICK_CPP_BUILD"), reason="XBLICK_CPP_BUILD is not set")
def test_a_cpp_parameter_set_runs_the_writer_once(network):
    """C++ ParameterClient (xblick-cpp-probe --params --set), each run a new session behind the router."""
    import os
    import subprocess

    from xblick.parameters import Parameters

    writes = []
    params = Parameters(network.server, "node", "t/c")
    params.declare("x", 0.0, write=lambda v: writes.append(v))
    probe = os.path.join(os.environ["XBLICK_CPP_BUILD"], "xblick-cpp-probe")
    try:
        for i in range(1, 4):
            out = subprocess.run([probe, "--connect", network.endpoint[0], "--listen", "tcp/127.0.0.1:0", "--params",
                                  "t/c/node", "--set", f"x:={i}.0"], capture_output=True, text=True, timeout=30)
            assert out.returncode == 0, out.stderr
        assert writes == [1.0, 2.0, 3.0]
    finally:
        params.close()
