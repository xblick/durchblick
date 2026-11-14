"""A node's parameters (parameters.py): declarations checked, values at the start (--param, params files), ROS 2's
services over Zenoh, events, read-only access, the params file's YAML."""

import argparse
import threading
from pathlib import Path

import pytest
from test_camera import free_port, sessions, wait_until  # noqa: F401 - sessions is a fixture

from xblick import node
from xblick.msg import ParameterType, ParameterValue
from xblick.parameters import (READ_ONLY_ACCESS, RESTART, ParameterClient, ParameterError, Parameters,
                               dump_params_file, find_nodes, parse_yaml, read_params_file)

NS, NAME = "robot/x", "boxer"


def declare_all(params):
    changes = []

    def write_threshold(value):
        if value > 0.9:
            return "above 0.9 the node finds nothing"
        changes.append(("threshold", value))
        return None

    params.declare("threshold", 0.3, description="a box's score at least", range=(0.0, 1.0, 0.05),
                   write=write_threshold)
    params.declare("max_boxes", 10, range=(1, 100, 1), unit="boxes")
    params.declare("mode", "fast", choices=["fast", "accurate"])
    params.declare("model", "base", read_only=True, description="the model's weights")
    params.declare("draw", True)
    params.declare("labels", ["cup", "box"])
    params.declare("detector.nms", 0.5, range=(0.0, 1.0))
    params.check_overrides()
    return changes


def test_declare_check_and_set(sessions):  # noqa: F811
    a, _ = sessions
    params = Parameters(a, NAME, NS)
    changes = declare_all(params)
    assert params.name == "/robot/x/boxer" and params["threshold"] == 0.3 and params["labels"] == ["cup", "box"]
    assert params.set("threshold", 0.45).successful and params["threshold"] == 0.45 and changes == [("threshold", 0.45)]
    assert params.set("threshold", 0.47).reason == "0.47 is not 0 plus a multiple of 0.05"
    assert params.set("threshold", 0.95).reason == "above 0.9 the node finds nothing"  # the node refused
    assert params.set("threshold", 1.5).reason == "1.5 is outside 0 to 1"
    assert params.set("threshold", 1).reason == "integer given, the parameter is double"
    assert params.set("max_boxes", 0).reason == "0 is outside 1 to 100"
    assert params.set("mode", "slow").reason == "slow is none of fast, accurate"
    assert params.set("model", "large").reason == RESTART
    assert params.set("nothing", 1).reason == "no parameter 'nothing'"
    assert params["threshold"] == 0.45  # refused changes change nothing
    assert params.descriptor("max_boxes").additional_constraints == "unit: boxes"
    assert params.descriptor("mode").additional_constraints == "choices: fast, accurate"
    with pytest.raises(ParameterError, match="its default: 2 is outside"):
        params.declare("bad", 2, range=(0, 1))
    with pytest.raises(ParameterError, match="declared already"):
        params.declare("draw", False)
    with pytest.raises(ParameterError, match="its default: 1 is none of a"):
        params.declare("c", 1, choices=["a"])
    params.close()


def test_values_at_the_start(sessions, tmp_path):  # noqa: F811
    a, _ = sessions
    (tmp_path / "all.yaml").write_text("""
/**:
  ros__parameters:
    draw: false          # every node
/robot/x/boxer:
  ros__parameters:
    threshold: 0.5
    detector:
      nms: 0.25
    labels: [cup, "a, b"]
""")
    parser = argparse.ArgumentParser()
    Parameters.add_arguments(parser)
    args = parser.parse_args(["--params-file", str(tmp_path / "all.yaml"), "--param", "max_boxes:=20",
                              "--param", "threshold:=0.6", "--dump-params", str(tmp_path / "dump.yaml")])
    params = Parameters(a, NAME, NS)
    params.configure(args)
    declare_all(params)
    assert (params["threshold"], params["max_boxes"], params["draw"], params["detector.nms"], params["labels"]) == \
        (0.6, 20, False, 0.25, ["cup", "a, b"])  # --param over the file, an integer text for a double taken
    dumped = (tmp_path / "dump.yaml").read_text()
    assert read_params_file(tmp_path / "dump.yaml", params.name) == {
        "detector.nms": 0.25, "draw": False, "labels": ["cup", "a, b"], "max_boxes": 20, "mode": "fast",
        "model": "base", "threshold": 0.6}, dumped
    params.close()
    # a value outside its range, a typo
    for given, error in ((["--param", "threshold:=2"], "threshold := '2' \\(--param\\): 2 is outside 0 to 1"),
                         (["--param", "treshold:=0.5"], "has no parameter treshold \\(given by --param\\)")):
        params = Parameters(a, NAME, NS)
        params.configure(parser.parse_args(given))
        with pytest.raises(ParameterError, match=error):
            declare_all(params)
        params.close()
    params = Parameters(a, "n2", NS)
    with pytest.raises(ParameterError, match="NAME:=VALUE"):
        params.configure(parser.parse_args(["--param", "threshold=0.5"]))
    params.close()


def test_services_and_events_over_zenoh(sessions):  # noqa: F811
    a, b = sessions
    params = Parameters(a, NAME, NS)
    declare_all(params)
    client = ParameterClient(b, f"{NS}/{NAME}")
    wait_until(lambda: f"{NS}/{NAME}" in find_nodes(b, 0.3), timeout=10)
    events = []
    client.on_event(events.append)
    names = client.list()
    assert names == ["detector.nms", "draw", "labels", "max_boxes", "mode", "model", "threshold"]
    assert client.list(["detector"]) == ["detector.nms"] and client.list(depth=1) == [n for n in names if "." not in n]
    descriptors, values = client.all()
    by = dict(zip(names, descriptors))
    assert by["threshold"].floating_point_range[0].step == 0.05 and by["model"].read_only
    assert by["mode"].additional_constraints == "choices: fast, accurate"
    assert [v.value() for v in values] == [0.5, True, ["cup", "box"], 10, "fast", "base", 0.3]
    assert client.types(["threshold", "draw", "none"]) == [ParameterType.DOUBLE, ParameterType.BOOL, 0]
    assert client.get(["none"])[0].type == ParameterType.NOT_SET
    results = client.set({"threshold": 0.35, "max_boxes": 0, "model": "large"})
    assert [r.successful for r in results] == [True, False, False] and results[2].reason == RESTART
    wait_until(lambda: any(e.changed_parameters for e in events))
    event = next(e for e in events if e.changed_parameters)
    assert event.node == "/robot/x/boxer" and [(p.name, p.value.value()) for p in event.changed_parameters] == [
        ("threshold", 0.35)]
    # atomically: all or nothing
    result = client.set_atomically({"threshold": 0.4, "max_boxes": 1000})
    assert not result.successful and "max_boxes: 1000 is outside 1 to 100" in result.reason
    assert params["threshold"] == 0.35
    assert client.set_atomically({"threshold": 0.4, "max_boxes": 50}).successful
    assert (params["threshold"], params["max_boxes"]) == (0.4, 50)
    client.close()
    params.close()
    wait_until(lambda: f"{NS}/{NAME}" not in find_nodes(b, 0.3), timeout=10)


def test_read_only_access(sessions):  # noqa: F811
    a, b = sessions
    parser = argparse.ArgumentParser()
    Parameters.add_arguments(parser)
    params = Parameters(a, NAME, NS)
    params.configure(parser.parse_args(["--params-read-only"]))
    declare_all(params)
    client = ParameterClient(b, f"{NS}/{NAME}")
    wait_until(lambda: f"{NS}/{NAME}" in find_nodes(b, 0.3), timeout=10)
    descriptors, values = client.all()
    assert all(d.read_only for d in descriptors) and values
    assert client.set({"threshold": 0.35})[0].reason == READ_ONLY_ACCESS
    assert params.set("threshold", 0.35).successful  # the node itself still can
    params.close()


def test_yaml_of_params_files(tmp_path):
    text = dump_params_file({"/a/b": {"x.y": 0.3, "m": "base", "n": 3, "l": [1.0, 2.5], "s": "a: b # c",
                                      "e": "", "t": "true", "f": float("inf"), "b": False, "i": [1, 2]}})
    (tmp_path / "p.yaml").write_text(text)
    assert read_params_file(tmp_path / "p.yaml", "/a/b") == {"b": False, "e": "", "f": float("inf"), "i": [1, 2],
                                                             "l": [1.0, 2.5], "m": "base", "n": 3, "s": "a: b # c",
                                                             "t": "true", "x.y": 0.3}
    assert read_params_file(tmp_path / "p.yaml", "/a/c") == {}
    for bad, error in (("a:\n\tb: 1\n", "tabs"), ("a\n", "'key: value' expected"), ("/n:\n  x: 1\n",
                                                                                        "ros__parameters")):
        (tmp_path / "bad.yaml").write_text(bad)
        with pytest.raises(ParameterError, match=error):
            read_params_file(tmp_path / "bad.yaml", "/n")
    assert parse_yaml("a:\n  b:\n    c: [1, 'x', [2, 3]]\n") == {"a": {"b": {"c": [1, "x", [2, 3]]}}}


def test_node_parameters_are_the_module():
    assert node.Parameters is Parameters
    assert ParameterValue.of(0.5).value() == 0.5


def test_on_change_runs_and_failures_refuse(sessions):  # noqa: F811
    a, _ = sessions
    params = Parameters(a, "n3", NS)
    called = threading.Event()

    def broken(value):
        called.set()
        raise RuntimeError("device gone")

    params.declare("gain", 1.0, write=broken)
    result = params.set("gain", 2.0)
    assert called.is_set() and not result.successful and "device gone" in result.reason and params["gain"] == 1.0
    params.close()


class FakeDevice:
    """A camera control with an automatic mode: setting the value switches it to manual; in auto the device changes
    the value by itself."""

    def __init__(self):
        self.auto, self.value = True, 100

    def set_value(self, value):
        if value == 13:
            return "the device refused 13"
        self.auto, self.value = False, value
        return None


def test_readers_writers_and_events_of_every_change(sessions):  # noqa: F811
    """get returns what the device uses (its reader); a write that changes another parameter (the value switching
    the auto mode off) shows in the event, which compares every value before and after; a device's refusal is the
    reason; apply_all writes every value at the start."""
    a, b = sessions
    device = FakeDevice()
    params = Parameters(a, "camera", NS)
    params.declare("color.exposure_us", 100, unit="us", range=(1, 1000, 1), read=lambda: device.value,
                   write=device.set_value)
    params.declare("color.auto_exposure", True, read=lambda: device.auto,
                   write=lambda v: setattr(device, "auto", v))
    params.declare("fps", 30, choices=[5, 15, 30], read_only=True)
    params.check_overrides()
    assert params.descriptor("fps").additional_constraints == "choices: 5, 15, 30"
    client = ParameterClient(b, f"{NS}/camera")
    wait_until(lambda: f"{NS}/camera" in find_nodes(b, 0.3), timeout=10)
    events = []
    client.on_event(events.append)
    device.value = 120  # the device in auto mode
    assert [v.value() for v in client.get(["color.exposure_us", "color.auto_exposure"])] == [120, True]
    assert client.set({"color.exposure_us": 200})[0].successful
    wait_until(lambda: any(e.changed_parameters for e in events))
    changed = {p.name: p.value.value() for p in next(e for e in events if e.changed_parameters).changed_parameters}
    assert changed == {"color.exposure_us": 200, "color.auto_exposure": False}
    assert client.set({"color.exposure_us": 13})[0].reason == "the device refused 13"
    assert client.set({"fps": 15})[0].reason == RESTART
    device.value = 7
    params.apply_all()
    assert (device.value, device.auto) == (200, False)  # the stored values written again
    client.close()
    params.close()


def test_params_file_node_names(tmp_path):
    (tmp_path / "p.yaml").write_text("camera:\n  ros__parameters:\n    a: 1\n"
                                     "robot/x/camera:\n  ros__parameters:\n    b: 2\n"
                                     "/robot/*/camera:\n  ros__parameters:\n    c: 3\n"
                                     "/other:\n  ros__parameters:\n    d: 4\n")
    assert read_params_file(tmp_path / "p.yaml", "/robot/x/camera") == {"a": 1, "b": 2, "c": 3}


def test_xblick_param(capsys, tmp_path):
    """``xblick param``: the nodes, a node's parameters described, changed (a refusal's reason), dumped."""
    from test_camera import free_port

    from xblick import cli
    from xblick.session import open_session

    port = free_port()
    a = open_session(listen=[f"127.0.0.1:{port}"])
    b = open_session(connect=[f"127.0.0.1:{port}"])
    params = Parameters(a, NAME, NS)
    declare_all(params)
    connect = ["--connect", f"127.0.0.1:{port}", "--listen", "127.0.0.1:0"]
    wait_until(lambda: f"{NS}/{NAME}" in find_nodes(b, 0.3), timeout=10)
    assert cli.main(["param", "--nodes", *connect]) == 0
    assert f"{NS}/{NAME}" in capsys.readouterr().out
    assert cli.main(["param", "--node", f"{NS}/{NAME}", "--describe", "threshold", "model", *connect]) == 0
    out = capsys.readouterr().out.splitlines()
    assert out == ["threshold = 0.3  (double; 0 to 1 step 0.05)  a box's score at least",
                   "model = base  (string; read-only)  the model's weights"]
    assert cli.main(["param", "--node", f"{NS}/{NAME}", "--set", "threshold:=0.5", "--set", "max_boxes:=0",
                     *connect]) == 1
    assert capsys.readouterr().out.splitlines() == ["threshold: set", "max_boxes: refused: 0 is outside 1 to 100"]
    assert params["threshold"] == 0.5
    assert cli.main(["param", "--node", f"{NS}/{NAME}", "--dump", str(tmp_path / "d.yaml"), *connect]) == 0
    assert read_params_file(tmp_path / "d.yaml", params.name)["threshold"] == 0.5
    params.close()
    b.close()
    a.close()


def test_apply_all_in_declaration_order():
    """apply_all writes in the order of declaration, not of the names: the value (which switches the device to
    manual) before its auto_* bool, so a control given auto at the start ends in auto."""
    device = FakeDevice()
    written = []

    def write_value(v):
        written.append("color.exposure_time_us")
        return device.set_value(v)

    def write_auto(v):
        written.append("color.auto_exposure")
        device.auto = v

    params = Parameters(None, "camera", NS)
    params.declare("color.exposure_time_us", 8000, write=write_value, read=lambda: device.value)
    params.declare("color.auto_exposure", True, write=write_auto, read=lambda: device.auto)
    params.check_overrides()
    params.apply_all()
    assert written == ["color.exposure_time_us", "color.auto_exposure"]
    assert device.auto is True and device.value == 8000
    # a value set later switches to manual; the bool, changed by that write, is stored so: apply_all keeps manual
    assert params.set("color.exposure_time_us", 9000).successful and device.auto is False
    params.apply_all()
    assert device.auto is False and device.value == 9000
    device.value = 7000  # the device changed it by itself: refresh stores it
    params.refresh()
    device.value = 0
    params.apply_all()
    assert device.value == 7000


def options_parser():
    import argparse

    from xblick import node
    from xblick.session import add_arguments

    parser = argparse.ArgumentParser(prog="xblick-test")
    parser.add_argument("--camera", required=True, help="the camera's base")
    parser.add_argument("--threshold", type=float, default=0.5, help="the least score (default %(default)s)")
    parser.add_argument("--words", nargs="*", default=[], help="prompts")
    parser.add_argument("--checkpoint", default=None, help="the weights")
    parser.add_argument("--gpu", type=int, default=None)
    parser.add_argument("--mode", choices=["fast", "accurate"], default="fast")
    parser.add_argument("--out", type=Path, default=Path("/tmp/out"))
    parser.add_argument("--no-depth", action="store_true")
    add_arguments(parser)
    node.add_arguments(parser, max_hz=None)
    Parameters.add_arguments(parser)
    return parser


def test_a_nodes_options_as_parameters():
    parser = options_parser()
    args = parser.parse_args(["--camera", "robot/k", "--param", "gpu:=3", "--param", "threshold:=0.7",
                              "--param", "checkpoint:=/w.pt", "--max-hz", "0"])
    params = Parameters(None, "test", "robot/k")
    params.configure(args)
    names = params.declare_options(parser, args, live={"threshold": {"range": (0.0, 1.0, 0.05)},
                                                        "max_hz": {"default": 0.0, "unit": "frames/s"}})
    params.check_overrides()
    # every option but argparse's, the parameters' own and the connection's
    assert set(names) == {"camera", "threshold", "words", "checkpoint", "gpu", "mode", "out", "no_depth", "max_hz"}
    # a value given at the start reaches args, with the option's type
    assert (args.gpu, args.threshold, args.checkpoint, args.out) == (3, 0.7, "/w.pt", Path("/tmp/out"))
    assert args.max_hz is None  # the live stand-in for None given back as None
    # read-only but the live ones, with the help as description
    assert params.descriptor("camera").read_only and not params.descriptor("threshold").read_only
    assert params.descriptor("threshold").description == "--threshold: the least score (default 0.5)"
    assert params.set("threshold", 0.9).successful and params["threshold"] == 0.9
    assert not params.set("threshold", 1.5).successful
    assert params["mode"] == "fast" and params["words"] == [] and params["no_depth"] is False


def test_options_that_do_not_fit_are_refused():
    parser = options_parser()
    args = parser.parse_args(["--camera", "c"])
    with pytest.raises(ParameterError, match="no options"):
        Parameters(None, "test").declare_options(parser, args, live={"thresh": {}})
    with pytest.raises(ParameterError, match="give its type"):
        Parameters(None, "test").declare_options(parser, args, live={"gpu": {}})
    params = Parameters(None, "test")
    params.declare("threshold", 0.1)  # declared by the node itself: left as it is
    params.declare_options(parser, args)
    assert params["threshold"] == 0.1


def test_a_node_that_ends_with_an_exception_before_close_ends():
    """Parameters offered, then an exception (an interrupt while the model loads) before close(): the process ends.
    With the services in zenoh-python's callbacks it waited for good for their thread, which is no daemon."""
    import subprocess  # noqa: PLC0415
    import sys  # noqa: PLC0415

    script = ("from xblick.session import open_session\nfrom xblick.parameters import Parameters\n"
              "s = open_session(listen=['127.0.0.1:0'])\np = Parameters(s, 'probe', 'test')\np.declare('a', 1)\n"
              "raise KeyboardInterrupt\n")
    out = subprocess.run([sys.executable, "-c", script], capture_output=True, text=True, timeout=30)
    assert out.returncode != 0 and "KeyboardInterrupt" in out.stderr


def test_a_session_just_opened_through_a_router_finds_the_node():
    """xblick param as a peer behind a router (the default setup): its session is new when it asks, and through the
    router the node's queryables and token become known to it only some 0.1-0.3 s later. A single query sent at once
    ends with no reply; find_nodes and the client's calls ask again until the timeout."""
    from test_tui import open_router

    from xblick.session import open_session

    port = free_port()
    router = open_router(port)
    endpoint = [f"tcp/127.0.0.1:{port}"]
    node_session = open_session(connect=endpoint, listen=["127.0.0.1:0"])
    params = Parameters(node_session, NAME, NS)
    params.declare("threshold", 0.3)
    try:
        for i in range(3):  # each a new session, as each xblick param command is
            with open_session(connect=endpoint, listen=["127.0.0.1:0"]) as asker:
                assert f"{NS}/{NAME}" in find_nodes(asker, 2.0)
            with open_session(connect=endpoint, listen=["127.0.0.1:0"]) as asker:
                value = ParameterClient(asker, f"{NS}/{NAME}", 2.0).get(["threshold"])[0].value()
                assert value == pytest.approx(0.3 + i * 0.1)
            with open_session(connect=endpoint, listen=["127.0.0.1:0"]) as asker:
                assert ParameterClient(asker, f"{NS}/{NAME}", 2.0).set({"threshold": 0.4 + i * 0.1})[0].successful
        assert params["threshold"] == pytest.approx(0.6)
        with open_session(connect=endpoint, listen=["127.0.0.1:0"]) as asker:
            with pytest.raises(TimeoutError, match="no answer"):
                ParameterClient(asker, f"{NS}/nobody", 0.5).get(["threshold"])
    finally:
        params.close()
        node_session.close()
        router.close()
