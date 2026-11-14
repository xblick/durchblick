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
        self._sessions = []
        self.calls: dict[str, int] = {}
        self._lock = threading.Lock()

    def serve(self, key: str, answer=lambda query: b"ok", delay: float = 0.0, server=None, name: str | None = None):
        """A queryable on ``key`` that counts its queries (in ``calls[name or key]``); ``answer(query)`` the reply's
        payload, None: no reply; ``server``: another serving session (a second node)."""
        name = name or key

        def handle(query):
            with self._lock:
                self.calls[name] = self.calls.get(name, 0) + 1
            if delay:
                time.sleep(delay)
            payload = answer(query)
            if payload is not None:
                query.reply(query.key_expr, payload)

        self._queryables.append((server or self.server).declare_queryable(key, handle))
        self.calls[name] = 0

    def second_server(self):
        """Another serving session behind the router (a second node, a forgotten second bridge)."""
        self._sessions.append(open_session(connect=self.endpoint, listen=["127.0.0.1:0"]))
        return self._sessions[-1]

    def new_session(self):
        return open_session(connect=self.endpoint, listen=["127.0.0.1:0"])

    def close(self):
        for q in self._queryables:
            q.undeclare()
        for extra in self._sessions:
            extra.close()
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


# ------------------------------------------------------------------------------ commands: once per queryable
# A command (a parameter set, a controller manager's create, switch or unload, a torque step, a recorder command) is
# sent once to each queryable of its key, after the session knows one (session.ask_once, session.command), and only
# after a read of the same node had exactly one answer. Each test counts the handlers' runs.

import json  # noqa: E402

from xblick import recording  # noqa: E402
from xblick.msg import (CreateControllerResponse, ListParametersResponse, SwitchControllerResponse,  # noqa: E402
                        TorqueStepResponse, UnloadControllerResponse)
from xblick.session import NoAnswer, SeveralExecuted, command, timed_out  # noqa: E402

M = "robot/controller_manager"
STATE = ControllerState(name="arms", state="active", type="Impedance")


def manager(network, server=None, name_suffix=""):
    """A controller manager's queryables (list answering, the commands answering ok)."""
    for service, payload in (("list_controllers", ListControllersResponse([STATE])),
                             ("create_controller", CreateControllerResponse(True, "")),
                             ("switch_controller", SwitchControllerResponse(True, "switched")),
                             ("unload_controller", UnloadControllerResponse(True))):
        network.serve(f"{M}/{service}", lambda q, b=bytes(payload.serialize()): b, server=server,
                      name=f"{M}/{service}{name_suffix}")


def controller(network, answer=True):
    network.serve("robot/controller/arms/list_parameters",
                  lambda q: bytes(ListParametersResponse(["level"], []).serialize()))
    network.serve("robot/controller/arms/torque_step",
                  (lambda q: bytes(TorqueStepResponse(True, "").serialize())) if answer else (lambda q: None))


def test_an_empty_result_does_not_mean_that_nobody_got_the_query(network):
    """Why a command is not asked again: a queryable that drops the query without a reply ends it at once with no
    reply, as a session that knows no queryable does. session.ask (for reads) sends such a query again."""
    network.serve("t/silent", lambda q: None)
    with network.new_session() as s:
        time.sleep(0.5)  # the queryable known
        assert ask(s, "t/silent", 0.6) == []
    assert network.calls["t/silent"] >= 2  # a read is sent again: harmless for a read, wrong for a command


def test_zenohs_timeout_reply_is_pinned():
    """session.timed_out compares Zenoh's error reply with its text "Timeout" (eclipse-zenoh 1.10, the version in
    uv.lock): a change of that text fails here, not silently in the callers."""
    from importlib.metadata import version

    from xblick.session import open_session as open_

    port = free_port()
    a = open_(listen=[f"tcp/127.0.0.1:{port}"])
    b = open_(connect=[f"tcp/127.0.0.1:{port}"])
    q = a.declare_queryable("t/slow", lambda query: (time.sleep(0.8), query.reply(query.key_expr, b"late")))
    try:
        time.sleep(0.5)
        (reply,) = replies(b, "t/slow", 0.2)
        assert reply.err is not None and reply.err.payload.to_string() == "Timeout" and timed_out(reply), \
            f"eclipse-zenoh {version('eclipse-zenoh')}"
        assert version("eclipse-zenoh").startswith("1.10.")
    finally:
        time.sleep(1.0)
        q.undeclare()
        b.close()
        a.close()


@pytest.mark.parametrize("kind", ["fast", "silent", "slow"])
def test_a_command_is_sent_once(network, kind):
    """From sessions just opened behind the router: a handler that answers, one that gives no reply, one that answers
    after the timeout. Each ran once per command; the two last are NoAnswer."""
    answer = None if kind == "silent" else (lambda q: b"done")
    network.serve("t/command", answer, delay=1.0 if kind == "slow" else 0.0)
    for _ in range(5):
        with network.new_session() as s:
            if kind == "fast":
                assert [r.payload.to_bytes() for r in command(s, "t/command", 0.4)] == [b"done"]
            else:
                with pytest.raises(NoAnswer, match="t/command: sent once, no answer within 0.4 s: it may still be "
                                                   "executed; not sent again"):
                    command(s, "t/command", 0.4)
    deadline = time.monotonic() + 10  # the handler works the queries one after another (1 s each when slow)
    while network.calls["t/command"] < 5 and time.monotonic() < deadline:
        time.sleep(0.05)
    time.sleep(1.5)  # nothing more comes
    assert network.calls["t/command"] == 5


def test_the_two_cases_are_told_apart_by_type(network):
    """NotReached (nothing sent) is a TimeoutError, as before; NoAnswer (sent once) is not, so that a caller's
    ``except TimeoutError: try again`` cannot send a command a second time."""
    network.serve("t/silent", lambda q: None)
    with network.new_session() as s:
        try:
            command(s, "t/nobody", 0.3)
        except TimeoutError as error:
            assert isinstance(error, NotReached) and str(error).startswith("not sent: no node serves t/nobody")
        with pytest.raises(NoAnswer):
            try:
                command(s, "t/silent", 0.3)
            except TimeoutError:
                pytest.fail("NoAnswer caught as a TimeoutError")
    assert issubclass(NoAnswer, RuntimeError) and not issubclass(NoAnswer, TimeoutError)
    assert network.calls["t/silent"] == 1


def test_without_a_queryable_nothing_is_sent_within_the_timeout(network):
    network.serve("t/elsewhere")
    with network.new_session() as s:
        started = time.monotonic()
        with pytest.raises(NotReached, match="not sent: no node serves t/command"):
            ask_once(s, "t/command", 0.3)
        assert time.monotonic() - started < 0.6
    assert network.calls["t/elsewhere"] == 0


def test_the_time_a_command_takes_at_most(network):
    """The bound in ask_once's docstring: up to timeout for the queryable, then the query's own timeout (Zenoh ends
    it and adds the "Timeout" reply), so about 2 x timeout here; 3 x timeout + 1 s when Zenoh does not end it."""
    network.serve("t/slow", lambda q: b"late", delay=2.0)
    with network.new_session() as s:
        started = time.monotonic()
        with pytest.raises(NoAnswer):
            command(s, "t/slow", 0.5)
        took = time.monotonic() - started
    assert took < 2 * 0.5 + 0.3, took


def test_late_execution_after_the_caller_gave_up(network):
    """A command has no expiry: a receiver busy with an earlier query executes it after the caller got NoAnswer. This
    pins the behaviour (and its message), which the guide and AGENTS.md state; a deadline would need a wire
    change."""
    executed = []

    def handle(query):
        if query.payload is not None and query.payload.to_bytes() == b"busy":
            time.sleep(1.5)
            return b"busy done"
        executed.append(time.monotonic())
        return b"done"

    network.serve("t/arm", handle)
    with network.new_session() as busy, network.new_session() as s:
        time.sleep(0.5)
        threading.Thread(target=lambda: replies(busy, "t/arm", 3.0, payload=b"busy"), daemon=True).start()
        time.sleep(0.1)
        with pytest.raises(NoAnswer, match="it may still be executed; not sent again"):
            command(s, "t/arm", 0.4)
        gave_up = time.monotonic()
        deadline = time.monotonic() + 5
        while not executed and time.monotonic() < deadline:
            time.sleep(0.05)
    assert executed and executed[0] > gave_up  # executed after the caller had given up


@pytest.mark.parametrize("mode", ["client", "two_routers_peer", "two_routers_client"])
def test_a_command_is_sent_once_in_other_topologies(network, mode):
    """A client session behind the router, and sessions behind a second router chained to the first."""
    network.serve("t/command", lambda q: b"done")
    extra = None
    if mode == "client":
        endpoint = network.endpoint
    else:
        import json as json_
        import zenoh

        port = free_port()
        config = zenoh.Config()
        config.insert_json5("mode", '"router"')
        config.insert_json5("listen/endpoints", json_.dumps([f"tcp/127.0.0.1:{port}"]))
        config.insert_json5("connect/endpoints", json_.dumps(network.endpoint))
        config.insert_json5("scouting/multicast/enabled", "false")
        extra = zenoh.open(config)
        endpoint = [f"tcp/127.0.0.1:{port}"]
    try:
        for _ in range(5):
            peer = mode == "two_routers_peer"
            with open_session(connect=endpoint, listen=["127.0.0.1:0"] if peer else [],
                              mode="peer" if peer else "client") as s:
                assert [r.payload.to_bytes() for r in command(s, "t/command", 2.0)] == [b"done"]
    finally:
        if extra is not None:
            extra.close()
    assert network.calls["t/command"] == 5


def test_a_parameter_set_runs_the_nodes_writer_once(network):
    """set_parameters is a command (a node may act on a set): once per set, also with a writer slower than the
    client's timeout, which then gives NoAnswer and is not sent again."""
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
            with pytest.raises(NoAnswer, match="sent once, no answer within 0.3 s"):
                ParameterClient(s, "t/w/node", 0.3).set({"x": 9.0})
        time.sleep(1.5)
        assert writes == [1.0, 2.0, 3.0, 9.0]
    finally:
        params.close()


def test_two_nodes_on_one_key_get_no_command(network):
    """Two nodes serving one key would both execute a command: it is refused before anything is sent, where the
    protocol lets the sender count them (a read of the node with one answer). Handlers 0 and 0."""
    from xblick.parameters import ParameterClient, Parameters

    writes = {"a": [], "b": []}
    second = network.second_server()
    nodes = [Parameters(session, "node", "t/two") for session in (network.server, second)]
    for params, which in zip(nodes, "ab"):
        params.declare("x", 0.0, write=lambda v, w=which: writes[w].append(v))
    manager(network)
    manager(network, server=second, name_suffix=" (second)")
    for service in ("status", "control"):
        network.serve(f"xblick/rec/{service}", lambda q: json.dumps({"state": "idle", "play": None}).encode())
        network.serve(f"xblick/rec/{service}", lambda q: json.dumps({"state": "idle", "play": None}).encode(),
                      server=second, name=f"xblick/rec/{service} (second)")
    try:
        time.sleep(0.5)
        with network.new_session() as s:
            with pytest.raises(RuntimeError, match="not sent: 2 nodes answer t/two/node/list_parameters"):
                ParameterClient(s, "t/two/node", 2.0).set({"x": 1.0})
            for call in (lambda m: m.create("arms", "Impedance", {}), lambda m: m.switch(activate=["arms"]),
                         lambda m: m.unload("arms")):
                with pytest.raises(RuntimeError, match="not sent: 2 nodes answer robot/controller_manager/list_cont"):
                    call(controller_manager.ControllerManager(s, "robot", timeout=2.0))
            with pytest.raises(RuntimeError, match="not sent: 2 nodes answer xblick/rec/status"):
                recording.request(s, "rec", "record", timeout=2.0)
        assert writes == {"a": [], "b": []}
        for service in ("create_controller", "switch_controller", "unload_controller"):
            assert network.calls[f"{M}/{service}"] == 0 and network.calls[f"{M}/{service} (second)"] == 0
        assert network.calls["xblick/rec/control"] == 0 and network.calls["xblick/rec/control (second)"] == 0
    finally:
        for params in nodes:
            params.close()


def test_the_controller_managers_commands_run_once(network):
    manager(network)
    controller(network)
    for _ in range(3):
        with network.new_session() as s:
            controller_manager.ControllerManager(s, "robot", timeout=2.0).create("arms", "Impedance", {"a": 1})
        with network.new_session() as s:
            assert controller_manager.ControllerManager(s, "robot", timeout=2.0).switch(activate=["arms"]) == "switched"
        with network.new_session() as s:
            controller_manager.ControllerManager(s, "robot", timeout=2.0).unload("arms")
        with network.new_session() as s:
            controller_manager.torque_step(s, "robot/controller/arms", "joint1", 1.0, 0.1)
    assert {k: v for k, v in network.calls.items() if "list" not in k} == {
        f"{M}/create_controller": 3, f"{M}/switch_controller": 3, f"{M}/unload_controller": 3,
        "robot/controller/arms/torque_step": 3}


@pytest.mark.parametrize("which", ["create", "switch", "unload", "torque_step"])
def test_a_controller_command_without_an_answer_is_not_repeated(network, which):
    """A manager or a controller that takes a command and answers nothing: NoAnswer, the handler ran once."""
    network.serve(f"{M}/list_controllers", lambda q: bytes(ListControllersResponse([STATE]).serialize()))
    key = {"create": f"{M}/create_controller", "switch": f"{M}/switch_controller", "unload": f"{M}/unload_controller",
           "torque_step": "robot/controller/arms/torque_step"}[which]
    if which == "torque_step":
        controller(network, answer=False)
    else:
        network.serve(key, lambda q: None)
    with network.new_session() as s:
        m = controller_manager.ControllerManager(s, "robot", timeout=0.5)
        call = {"create": lambda: m.create("arms", "Impedance", {}, timeout=0.5),
                "switch": lambda: m.switch(activate=["arms"], timeout=0.0),
                "unload": lambda: m.unload("arms"),
                "torque_step": lambda: controller_manager.torque_step(s, "robot/controller/arms", "j", 1.0, 0.1,
                                                                      timeout=0.5)}[which]
        with pytest.raises(NoAnswer, match=f"{key}: sent once, no answer"):
            call()
    time.sleep(0.5)
    assert network.calls[key] == 1


def recorder(network, control):
    network.serve("xblick/rec/status", lambda q: json.dumps({"state": "idle", "play": None}).encode())
    network.serve("xblick/rec/control", control)


def test_a_recorder_command_runs_once(network):
    recorder(network, lambda q: json.dumps({"v": recording.REPLY_VERSION, "ok": True,
                                            "result": {"state": "recording"}}).encode())
    for _ in range(3):
        with network.new_session() as s:
            assert recording.request(s, "rec", "record", timeout=2.0) == {"state": "recording"}
    assert network.calls["xblick/rec/control"] == 3


@pytest.mark.parametrize("kind", ["silent", "slow"])
def test_a_recorder_command_without_an_answer(network, kind):
    """No answer to a record command: NoAnswer "sent once ... not sent again", the node ran it once."""
    recorder(network, (lambda q: None) if kind == "silent" else
             (lambda q: (time.sleep(1.0), json.dumps({"v": recording.REPLY_VERSION, "ok": True,
                                                      "result": {}}).encode())[1]))
    with network.new_session() as s:
        with pytest.raises(NoAnswer, match="xblick/rec/control: sent once, no answer within 0.4 s"):
            recording.request(s, "rec", "record", timeout=0.4)
    time.sleep(1.2)
    assert network.calls["xblick/rec/control"] == 1


@pytest.mark.skipif(not __import__("os").environ.get("XBLICK_CPP_BUILD"), reason="XBLICK_CPP_BUILD is not set")
@pytest.mark.parametrize("slow", [False, True])
def test_a_cpp_parameter_set_runs_the_writer_once(network, slow):
    """C++ ParameterClient (xblick-cpp-probe --params --set), each run a new session behind the router; a writer
    slower than the client's timeout (2 s): NoAnswer "sent once", the writer ran once."""
    import os
    import subprocess

    from xblick.parameters import Parameters

    writes = []

    def write(value):
        writes.append(value)
        if slow:
            time.sleep(3.0)

    params = Parameters(network.server, "node", "t/c")
    params.declare("x", 0.0, write=write)
    probe = os.path.join(os.environ["XBLICK_CPP_BUILD"], "xblick-cpp-probe")
    try:
        for i in range(1, 2 if slow else 4):
            out = subprocess.run([probe, "--connect", network.endpoint[0], "--listen", "tcp/127.0.0.1:0", "--params",
                                  "t/c/node", "--set", f"x:={i}.0"], capture_output=True, text=True, timeout=30)
            if slow:
                assert out.returncode != 0 and "set_parameters: sent once, no answer within 2000 ms: it may still " \
                                                "be executed; not sent again" in out.stderr, out.stderr
            else:
                assert out.returncode == 0, out.stderr
        time.sleep(3.5 if slow else 0.0)
        assert writes == ([1.0] if slow else [1.0, 2.0, 3.0])
    finally:
        params.close()


def test_the_command_line_says_which_case_it_was(network, capsys):
    """xblick param --set: "not sent" (two nodes) and "sent once ... it may still be executed" each as one line."""
    from xblick.parameters import Parameters

    writes = []
    params = Parameters(network.server, "node", "t/cli")
    params.declare("x", 0.0, write=lambda v: (writes.append(v), time.sleep(1.0) if v > 5 else None)[1])
    second = network.second_server()
    twins = [Parameters(session, "node", "t/twin") for session in (network.server, second)]
    for twin in twins:
        twin.declare("x", 0.0)
    endpoint = ["--connect", network.endpoint[0], "--listen", "tcp/127.0.0.1:0"]
    try:
        time.sleep(0.5)
        assert cli.main(["param", "--node", "t/cli/node", "--set", "x:=9.0", "--timeout", "0.5", *endpoint]) == 1
        err = capsys.readouterr().err
        assert err == ("xblick param: t/cli/node/set_parameters: sent once, no answer within 0.5 s: it may still be "
                       "executed; not sent again\n"), err
        assert cli.main(["param", "--node", "t/twin/node", "--set", "x:=1.0", *endpoint]) == 1
        err = capsys.readouterr().err
        assert err.startswith("xblick param: ") and "2 nodes answer" in err and err.count("\n") == 1, err
        time.sleep(1.2)
        assert writes == [9.0]
    finally:
        params.close()
        for twin in twins:
            twin.close()


# ------------------------------------------------------------------- the check for a second node: what it catches
# The check before a command (session.count) is best effort: it catches two nodes that both answer the read, and a
# node that has not finished the read at its timeout (the "Timeout" reply that the caller's session adds). It does
# not catch a second node that drops the read, that appears after it, or a second torque_step server without
# parameters: those get the command (pinned below).

def busy(params, name="x"):
    """Keeps a Parameters node busy for 3 s: a local set whose writer blocks, so that its services answer late."""
    from xblick.msg import Parameter, ParameterValue, SetParametersRequest

    threading.Thread(target=lambda: params.respond("set_parameters", SetParametersRequest(
        [Parameter(name, ParameterValue.of(1.0))])), daemon=True).start()
    time.sleep(0.2)


def test_a_busy_second_node_gets_no_set(network):
    """Two nodes of one name, the second busy: the read is still open at its timeout ("Timeout"), so the set is not
    sent; the first node's writer ran 0 times, the second's only for its own local set."""
    from xblick.parameters import ParameterClient, Parameters

    writes = {"a": [], "b": []}
    second = network.second_server()
    a = Parameters(network.server, "node", "t/busy")
    a.declare("x", 0.0, write=lambda v: writes["a"].append(v))
    b = Parameters(second, "node", "t/busy")
    b.declare("x", 0.0, write=lambda v: (writes["b"].append(v), time.sleep(3.0)))
    try:
        time.sleep(0.5)
        busy(b)
        with network.new_session() as s:
            with pytest.raises(RuntimeError, match="not sent: 1 node\\(s\\) answered t/busy/node/list_parameters, and "
                                                   "the query was still open at its timeout \\(a second node, or the "
                                                   "answering node"):
                ParameterClient(s, "t/busy/node", 1.0).set({"x": 2.0})
        time.sleep(3.0)
        assert writes == {"a": [], "b": [1.0]}
    finally:
        a.close()
        b.close()


def test_a_command_answered_by_one_node_while_another_is_late(network):
    """If a second node gets slow only after the read, the command's own result has one answer and the "Timeout"
    reply: PartlyAnswered (a NoAnswer, no TimeoutError), never success. Both ran it once."""
    from xblick.session import PartlyAnswered

    second = network.second_server()
    network.serve("t/both", lambda q: b"done", name="fast")
    network.serve("t/both", lambda q: b"late", delay=1.5, server=second, name="slow")
    with network.new_session() as s:
        time.sleep(0.5)
        with pytest.raises(PartlyAnswered, match="t/both: answered by 1 node\\(s\\), and the query was still open "
                                                 "after 0.5 s "
                                                 ".*a second node may still execute it"):
            command(s, "t/both", 0.5)
    time.sleep(1.5)
    assert network.calls == {"fast": 1, "slow": 1}
    assert issubclass(PartlyAnswered, NoAnswer) and not issubclass(PartlyAnswered, TimeoutError)


@pytest.mark.parametrize("read", ["slow", "silent"])
def test_a_second_manager_that_answers_its_read_late_or_not_at_all(network, read):
    """A second manager whose list_controllers is slower than the timeout: not sent (0 and 0). One that drops the
    read (no reply at all) is not seen by the check: it gets the switch, the caller learns it afterwards (pinned)."""
    second = network.second_server()
    manager(network)
    network.serve(f"{M}/list_controllers", (lambda q: bytes(ListControllersResponse([STATE]).serialize())) if read ==
                  "slow" else (lambda q: None), delay=3.0 if read == "slow" else 0.0, server=second,
                  name="second list")
    network.serve(f"{M}/switch_controller", lambda q: bytes(SwitchControllerResponse(True, "b").serialize()),
                  server=second, name="second switch")
    with network.new_session() as s:
        time.sleep(0.5)  # the second manager was declared just now: known to the router first (else it "appears later")
        m = controller_manager.ControllerManager(s, "robot", timeout=1.0)
        if read == "slow":
            with pytest.raises(RuntimeError, match="not sent: 1 node\\(s\\) answered .*list_controllers, and the "
                                                   "query was still open at its timeout"):
                m.switch(activate=["arms"])
        else:
            with pytest.raises(SeveralExecuted, match="switch_controller: sent, and 2 nodes answered: each of them "
                                                      "executed it"):
                m.switch(activate=["arms"])
    time.sleep(0.5)
    ran = 0 if read == "slow" else 1
    assert network.calls[f"{M}/switch_controller"] == ran and network.calls["second switch"] == ran


def test_a_second_torque_step_server_without_parameters_gets_the_step(network):
    """The torque step's check counts the controller's parameter servers, not its torque_step servers: a second
    torque_step server without parameters (an older bridge) gets the step; the caller learns it afterwards."""
    controller(network)
    second = network.second_server()
    network.serve("robot/controller/arms/torque_step", lambda q: bytes(TorqueStepResponse(True, "").serialize()),
                  server=second, name="second step")
    with network.new_session() as s:
        with pytest.raises(SeveralExecuted, match="robot/controller/arms/torque_step: sent, and 2 nodes answered"):
            controller_manager.torque_step(s, "robot/controller/arms", "j", 1.0, 0.1)
    assert network.calls["robot/controller/arms/torque_step"] == 1 and network.calls["second step"] == 1


def test_a_read_slower_than_its_timeout_stops_the_command(network):
    """A manager whose read takes longer than the timeout: "not sent" after about the timeout (it went out before
    0.5.2: fails safe); its switch ran 0 times."""
    network.serve(f"{M}/list_controllers", lambda q: bytes(ListControllersResponse([STATE]).serialize()), delay=1.5)
    network.serve(f"{M}/switch_controller", lambda q: bytes(SwitchControllerResponse(True, "").serialize()))
    with network.new_session() as s:
        started = time.monotonic()
        with pytest.raises(TimeoutError, match="not sent: no node answered robot/controller_manager/"
                                               "list_controllers in time"):
            controller_manager.ControllerManager(s, "robot", timeout=1.0).switch(activate=["arms"])
        took = time.monotonic() - started
    assert took < 1.0 + 0.6, took
    time.sleep(1.0)
    assert network.calls[f"{M}/switch_controller"] == 0


def test_send_command_goes_through_request(network):
    """recording.send_command is request (checked, sent once) and then the node's status."""
    recorder(network, lambda q: json.dumps({"v": recording.REPLY_VERSION, "ok": True, "result": {}}).encode())
    with network.new_session() as s:
        assert recording.send_command(s, "rec", "record") == {"state": "idle", "play": None}
        with pytest.raises(TimeoutError, match="not sent: no node answered xblick/nobody/status"):
            recording.request(s, "nobody", "record", timeout=0.5)
    assert network.calls["xblick/rec/control"] == 1


@pytest.mark.skipif(not __import__("os").environ.get("XBLICK_CPP_BUILD"), reason="XBLICK_CPP_BUILD is not set")
def test_a_cpp_set_with_a_busy_second_node(network):
    """xblick-cpp-probe --set while a second node of the name is busy: not sent (the C++ check), writers 0 and only
    the second's own local set."""
    import os
    import subprocess

    from xblick.parameters import Parameters

    writes = {"a": [], "b": []}
    second = network.second_server()
    a = Parameters(network.server, "node", "t/cbusy")
    a.declare("x", 0.0, write=lambda v: writes["a"].append(v))
    b = Parameters(second, "node", "t/cbusy")
    b.declare("x", 0.0, write=lambda v: (writes["b"].append(v), time.sleep(5.0)))
    probe = os.path.join(os.environ["XBLICK_CPP_BUILD"], "xblick-cpp-probe")
    try:
        time.sleep(0.5)
        busy(b)
        out = subprocess.run([probe, "--connect", network.endpoint[0], "--listen", "tcp/127.0.0.1:0", "--params",
                              "t/cbusy/node", "--set", "x:=2.0"], capture_output=True, text=True, timeout=30)
        assert out.returncode != 0 and "not sent: 1 node(s) answered t/cbusy/node/list_parameters, and the query " \
                                       "was still open at its timeout" in \
            out.stderr, out.stderr
        time.sleep(5.0)
        assert writes == {"a": [], "b": [1.0]}
    finally:
        a.close()
        b.close()


def test_one_slow_node_is_not_taken_for_a_second(network):
    """A single node whose count read answers only late: "not sent: no node answered ... in time", a TimeoutError
    that names no second node; its command ran 0 times."""
    from xblick.parameters import ParameterClient, Parameters

    writes = []
    a = Parameters(network.server, "node", "t/slow1")
    a.declare("x", 0.0, write=lambda v: (writes.append(v), time.sleep(3.0)))
    try:
        time.sleep(0.5)
        busy(a)
        with network.new_session() as s:
            with pytest.raises(TimeoutError, match="not sent: no node answered t/slow1/node/list_parameters in time"):
                ParameterClient(s, "t/slow1/node", 1.0).set({"x": 2.0})
        time.sleep(3.0)
        assert writes == [1.0]  # its own local set only
    finally:
        a.close()


def test_a_node_that_holds_the_query_after_its_reply(network):
    """The receiver rule, pinned: a single manager that replies at once and then keeps working inside the query's
    callback keeps the query open until its timeout, and the caller's session adds the "Timeout" reply; the check
    takes it for a possible second node and sends nothing ("a second node, or the answering node still holding the
    query"). Its switch ran 0 times."""
    def reply_then_hold(payload):
        def handle(query):
            query.reply(query.key_expr, payload)
            time.sleep(1.5)  # still inside the callback: the query is not finalized

        return handle

    network.server.declare_queryable(f"{M}/list_controllers",
                                     reply_then_hold(bytes(ListControllersResponse([STATE]).serialize())))
    network.serve(f"{M}/switch_controller", lambda q: bytes(SwitchControllerResponse(True, "").serialize()))
    with network.new_session() as s:
        time.sleep(0.5)
        with pytest.raises(RuntimeError, match="not sent: 1 node\\(s\\) answered .*list_controllers, and the query was "
                                               "still open at its timeout \\(a second node, or the answering node "
                                               "still holding the "
                                               "query after its reply\\)"):
            controller_manager.ControllerManager(s, "robot", timeout=0.5).switch(activate=["arms"])
    time.sleep(1.5)
    assert network.calls[f"{M}/switch_controller"] == 0


def test_a_command_reaches_every_queryable_of_its_key():
    """"Sent once to each queryable of its key" holds because Zenoh's default query target (BestMatching) delivers a
    query to every matching queryable while none of them is declared complete. Pinned in the sources: no Durchblick
    queryable sets ``complete`` and no query sets a ``target``. The behaviour itself, two queryables that both get a
    command, is pinned by test_a_second_torque_step_server_without_parameters_gets_the_step."""
    import re
    from pathlib import Path

    root = Path(__file__).resolve().parents[1]
    files = [*root.glob("src/xblick/**/*.py"), *root.glob("cpp/**/*.cpp"), *root.glob("cpp/**/*.hpp")]
    assert len(files) > 20
    pattern = re.compile(r"\bcomplete\s*=(?!=)|\.complete\b|QueryTarget|query_target|Z_QUERY_TARGET|\.target\s*=(?!=)|"
                         r"\btarget\s*=\s*zenoh")
    found = [f"{f.relative_to(root)}:{n}: {line.strip()}" for f in files
             for n, line in enumerate(f.read_text().splitlines(), 1) if pattern.search(line)]
    assert found == [], "\n".join(found)
