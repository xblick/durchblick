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
from xblick.session import ask, open_session, replies


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
