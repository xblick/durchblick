"""A peer that connects only to other PCs and listens on loopback only (session.across_pcs): the rule, the notice at
the start of a command, a node and a C++ program, and the cause in the "nothing found" messages.

192.0.2.10 and 2001:db8::1 are documentation addresses: never this PC, and a connection to them never succeeds. A
peer session opens at once with such an endpoint (Zenoh goes on trying in the background), so the processes here end
by themselves; each one is bounded by a timeout all the same."""

import os
import socket
import subprocess
import sys
from pathlib import Path

import pytest

from xblick import session
from xblick.session import across_pcs, across_pcs_text, loopback, this_pc

OTHER = "tcp/192.0.2.10:7447"
NOTICE = ("connecting as a peer to another PC (tcp/192.0.2.10:7447): programs there usually cannot reach this one, "
          "which listens on loopback only. Use --zenoh-mode client, or start a router on this PC with: xblick router "
          "--connect tcp/192.0.2.10:7447, and this program without --connect, or make this program reachable from there "
          "with --listen tcp/0.0.0.0:0 (any program on the network can then connect to it)")
TIMEOUT_S = 60  # a process here takes about 2 s


def own_address() -> str | None:
    """An address of one of this PC's network interfaces (not loopback), or None. A UDP socket's connect only
    chooses the route: it sends nothing."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        try:
            s.connect(("192.0.2.10", 7447))
        except OSError:
            return None
        address = s.getsockname()[0]
    return None if address.startswith("127.") else address


# The rule alone, with this PC as a fixed set of hosts: (connect, listen, mode, config file) -> notice or not.
HERE = {"127.0.0.1", "localhost", "::1", "0.0.0.0", "198.51.100.7", "pc-here"}  # 198.51.100.7: "this PC's address"
LOOPBACK = {"127.0.0.1", "localhost", "::1"}
CASES = [
    # (connect, listen, mode, config, expected endpoints)
    ([], [], "peer", None, []),                                                  # this PC's router (default)
    (["192.0.2.10"], [], "peer", None, [OTHER]),                                 # another host
    (["tcp/192.0.2.10:7447"], [], "peer", None, [OTHER]),
    (["other-pc"], [], "peer", None, ["tcp/other-pc:7447"]),                     # a name of another host
    (["[2001:db8::1]"], [], "peer", None, ["tcp/[2001:db8::1]:7447"]),          # IPv6
    (["192.0.2.10", "192.0.2.11:7448"], [], "peer", None, [OTHER, "tcp/192.0.2.11:7448"]),  # several, all other PCs
    (["192.0.2.10"], [], "client", None, []),                                    # client: the router routes for it
    (["127.0.0.1"], [], "peer", None, []),                                       # loopback
    (["localhost:7447"], [], "peer", None, []),
    (["[::1]:7447"], [], "peer", None, []),                                      # IPv6 loopback
    (["198.51.100.7"], [], "peer", None, []),                                    # this PC's own address
    (["pc-here"], [], "peer", None, []),                                         # this PC's name
    (["192.0.2.10", "127.0.0.1"], [], "peer", None, []),                         # and this PC's router
    (["192.0.2.10"], ["127.0.0.1:0"], "peer", None, [OTHER]),                   # listens on loopback only
    (["192.0.2.10"], ["tcp/[::1]:7448"], "peer", None, [OTHER]),
    (["192.0.2.10"], ["0.0.0.0:7448"], "peer", None, []),                        # listens on every address
    (["192.0.2.10"], ["198.51.100.7:7448"], "peer", None, []),                  # on a network address of this PC
    (["192.0.2.10"], ["127.0.0.1:0", "0.0.0.0:7448"], "peer", None, []),
    (["192.0.2.10"], ["unixsock-stream/tmp/z.sock"], "peer", None, [OTHER]),    # no network address
    (["unixsock-stream/tmp/z.sock"], [], "peer", None, []),                      # a locator without a host: this PC
    ([], [], "peer", "zenoh.json5", []),                                         # a config file: not read here
    ([], ["0.0.0.0:7448"], "peer", None, []),                                    # listens only
]


@pytest.mark.parametrize("connect,listen,mode,config,expected", CASES)
def test_the_rule(connect, listen, mode, config, expected):
    assert across_pcs(connect, listen, mode, config, local=HERE.__contains__,
                      loopback_only=LOOPBACK.__contains__) == expected


def test_this_pc_and_loopback_on_this_pc():
    """The real classification: loopback, the unspecified address, this PC's name and interface addresses; a
    documentation address and a name that does not resolve are another PC."""
    for host in ("127.0.0.1", "127.0.1.1", "::1", "localhost", "0.0.0.0", "::", socket.gethostname()):
        assert this_pc(host), host
    for host in ("192.0.2.10", "2001:db8::1", "no-such-host.invalid"):
        assert not this_pc(host), host
    for host in ("127.0.0.1", "::1", "localhost"):
        assert loopback(host), host
    for host in ("0.0.0.0", "::", "192.0.2.10", "no-such-host.invalid"):
        assert not loopback(host), host
    mine = own_address()
    if mine is None:
        pytest.skip("this PC has no route and so no network address to test with")
    assert this_pc(mine) and not loopback(mine)
    assert across_pcs([mine], [], "peer") == []
    assert across_pcs(["192.0.2.10"], [mine], "peer") == []
    assert across_pcs(["192.0.2.10"], ["127.0.0.1:0"], "peer") == [OTHER]


def test_the_notice_names_both_ways_out_with_the_endpoints():
    assert across_pcs_text([OTHER]) == NOTICE
    prefixed = across_pcs_text([OTHER], "node-")
    assert "--node-zenoh-mode client" in prefixed and "--node-listen tcp/0.0.0.0:0" in prefixed
    two = across_pcs_text([OTHER, "tcp/192.0.2.11:7448"])
    assert "xblick router --connect tcp/192.0.2.10:7447 tcp/192.0.2.11:7448," in two


def run(command: list[str], **kwargs) -> subprocess.CompletedProcess:
    return subprocess.run(command, capture_output=True, text=True, timeout=TIMEOUT_S, **kwargs)


def xblick(*arguments: str) -> subprocess.CompletedProcess:
    return run([sys.executable, "-m", "xblick.cli", *arguments])


def test_a_command_says_it_once_at_the_start_and_when_it_found_nothing():
    done = xblick("list", "--connect", "192.0.2.10", "--seconds", "0.3")
    lines = done.stderr.splitlines()
    assert done.returncode == 1 and lines[0] == f"xblick list: {NOTICE}", done.stderr
    assert lines[-1] == f"no stream answered within 0.3 s; this session is {NOTICE}"
    assert sum(NOTICE in line for line in lines) == 2  # the start line and the "nothing found" line, nothing else
    traffic = xblick("list", "--traffic", "--connect", "192.0.2.10", "--seconds", "0.3")
    assert traffic.stderr.splitlines()[-1] == f"nothing received within 0.3 s; this session is {NOTICE}"


@pytest.mark.parametrize("options", [["--zenoh-mode", "client"], ["--listen", "0.0.0.0:0"]])
def test_a_command_does_not_say_it_when_the_rule_does_not_hold(options):
    done = xblick("list", "--connect", "192.0.2.10", "--seconds", "0.3", *options)
    assert "connecting as a peer" not in done.stdout + done.stderr


def test_a_command_to_this_pc_does_not_say_it():
    from test_camera import free_port

    done = xblick("list", "--connect", f"127.0.0.1:{free_port()}", "--seconds", "0.3")
    assert done.returncode == 1 and "connecting as a peer" not in done.stderr
    assert "this session reached no Zenoh router" in done.stderr  # the hint of a session that reached nobody


NODE = '''
import argparse
from xblick import node, session

parser = argparse.ArgumentParser()
session.add_arguments(parser)
args = parser.parse_args()
node.start("xblick-testnode")
with session.open_from_args(args):
    print("opened", flush=True)
'''


def test_a_node_says_it_without_a_change_of_its_own(tmp_path):
    script = tmp_path / "xblick-testnode"
    script.write_text(NODE)
    done = run([sys.executable, str(script), "--connect", "192.0.2.10"])
    assert done.returncode == 0 and done.stdout.strip() == "opened", done.stderr
    assert done.stderr.splitlines()[0] == f"xblick-testnode: {NOTICE}"
    done = run([sys.executable, str(script), "--connect", "192.0.2.10", "--zenoh-mode", "client"])
    assert "connecting as a peer" not in done.stderr


def test_the_view_names_it_when_no_publisher_answers():
    from xblick import view

    with session.open_session(connect=["192.0.2.10"]) as z:
        assert z.across_pcs == [OTHER]
        with pytest.raises(view.ViewError, match="this session is connecting as a peer to another PC"):
            view.View(z, ["t/across/color"], seconds=0.3)
    with session.open_session(connect=["192.0.2.10"], listen=["0.0.0.0:0"]) as z:
        assert z.across_pcs == [] and "connecting as a peer" not in session.why_nothing(z)


@pytest.mark.skipif(not os.environ.get("XBLICK_CPP_BUILD"), reason="XBLICK_CPP_BUILD (the C++ build directory) is not set")
def test_a_cpp_program_says_it_by_the_same_rule():
    probe = str(Path(os.environ["XBLICK_CPP_BUILD"]) / "xblick-cpp-probe")
    done = run([probe, "--connect", "192.0.2.10", "--base", "t/across/color", "--frames", "1", "--timeout", "0.5"])
    assert done.stderr.splitlines()[0] == f"xblick-cpp-probe: {NOTICE}", done.stderr
    done = run([probe, "--connect", "192.0.2.10", "--listen", "0.0.0.0:0", "--base", "t/across/color", "--frames", "1",
                "--timeout", "0.5"])
    assert "connecting as a peer" not in done.stderr
