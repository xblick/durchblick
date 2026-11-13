"""Sessions: without endpoints the PC's router, explicit endpoints exactly as given."""

import json
import threading
import time

import pytest
import zenoh

from blick import session
from blick.session import LOCAL_LISTEN, ROUTER, make_config, replies


def endpoints(config):
    return json.loads(config.get_json("connect/endpoints")), json.loads(config.get_json("listen/endpoints"))


def test_without_endpoints_the_pcs_router():
    assert endpoints(make_config()) == ([ROUTER], [LOCAL_LISTEN])
    assert ROUTER == "tcp/127.0.0.1:7447" and LOCAL_LISTEN == "tcp/127.0.0.1:0"


def test_explicit_endpoints_replace_the_router():
    assert endpoints(make_config(listen=["127.0.0.1:7500"])) == ([], ["tcp/127.0.0.1:7500"])
    assert endpoints(make_config(connect=["robot-pc-4"])) == (["tcp/robot-pc-4:7447"], [])
    assert endpoints(make_config(local_router=False)) == ([], [])


def test_client_mode_connects_to_the_router_without_listening():
    config = make_config(mode="client")
    assert json.loads(config.get_json("connect/endpoints")) == [ROUTER]
    with pytest.raises(ValueError, match="client mode"):
        make_config(mode="client", local_router=False)


def test_zenoh_mode_on_the_command_line():
    import argparse

    from blick import session

    parser = argparse.ArgumentParser()
    session.add_arguments(parser)
    args = parser.parse_args(["--zenoh-mode", "client", "--connect", "robot-pc"])
    assert (args.zenoh_mode, args.connect) == ("client", ["robot-pc"])
    assert parser.parse_args([]).zenoh_mode == "peer"
    with pytest.raises(ValueError, match="client mode does not listen"):
        make_config(connect=["robot-pc"], listen=["127.0.0.1:0"], mode="client")
    with pytest.raises(ValueError, match="in the Zenoh config file"):
        session.open_from_args(parser.parse_args(["--zenoh-mode", "client", "--config", "x.json5"]))


# ------------------------------------------------------------------------------------------ query deadlines


class FakeReply:
    def __init__(self, ok):
        self.ok = ok


class FakeChannel:
    """A get's reply channel: these replies, then its end (``ends``) or none for good (blocked, as Zenoh's channel
    of a query it never finalizes)."""

    def __init__(self, items, ends=True):
        self.items, self.ends = list(items), ends
        self.never = threading.Event()

    def __iter__(self):
        yield from self.items
        if not self.ends:
            self.never.wait()


def test_replies_end_with_the_channel():
    started = time.monotonic()
    assert session.collect(FakeChannel(["a", "b"]), "x/ends", timeout=5.0) == ["a", "b"]
    assert time.monotonic() - started < 0.05


def test_error_replies_are_passed_on():
    error, ok = FakeReply(None), FakeReply("payload")
    assert session.collect(FakeChannel([error, ok]), "x/errors", timeout=1.0) == [error, ok]


def test_replies_return_at_the_deadline_when_zenoh_never_finalizes(caplog):
    session.not_finalized.pop("x/never", None)
    started = time.monotonic()
    with caplog.at_level("WARNING", logger="blick"):
        got = session.collect(FakeChannel(["a"], ends=False), "x/never", timeout=0.2)
        again = session.collect(FakeChannel([], ends=False), "x/never", timeout=0.2)
    elapsed = time.monotonic() - started
    assert got == ["a"] and again == []
    assert 1.3 < elapsed < 1.6  # twice: the timeout and max(0.5, timeout) more
    assert session.not_finalized["x/never"] == 2
    said = [r.message for r in caplog.records if "x/never" in r.message]
    assert len(said) == 1 and "not finalized by Zenoh" in said[0]  # logged once


def test_replies_of_real_queries():
    """Real queries (a queryable in the same session, a liveliness token): their replies, ended by Zenoh's
    finalization, not slower than a loaded runner allows."""
    z = session.open_session(local_router=False)
    try:
        queryable = z.declare_queryable("t/q", lambda q: q.reply("t/q", b"yes"))
        token = z.liveliness().declare_token("t/alive")
        assert [r.ok.payload.to_bytes() for r in replies(z, "t/q", 1.0)] == [b"yes"]
        assert [str(r.ok.key_expr) for r in replies(z, "t/*", 1.0, liveliness=True)] == ["t/alive"]
        started = time.monotonic()
        assert replies(z, "t/nobody", 0.2) == []
        assert time.monotonic() - started < 0.5  # finalized by its timeout at the latest

        def median(call):
            times = []
            for _ in range(100):
                started = time.perf_counter()
                call()
                times.append(time.perf_counter() - started)
            return sorted(times)[len(times) // 2]

        helped = median(lambda: replies(z, "t/q", 1.0))
        assert helped < 0.05, helped  # a bound for a loaded CI runner; docs/concepts/sessions.md has a measurement
        token.undeclare()
        queryable.undeclare()
    finally:
        z.close()


def test_close_waits_at_most_its_timeout(caplog):
    """session.close: a session closes; one whose close does not return (Zenoh's, after a query it did not finalize)
    is left closing after the timeout, logged."""
    z = session.open_session(local_router=False)
    assert session.close(z, timeout=5.0) is True

    class Stuck:
        class info:
            @staticmethod
            def zid():
                return "abc"

        def close(self):
            threading.Event().wait()

    started = time.monotonic()
    with caplog.at_level("WARNING", logger="blick"):
        assert session.close(Stuck(), timeout=0.2) is False
    assert time.monotonic() - started < 1.0 and "did not close within 0.2 s" in caplog.text


def test_open_session_closes_within_its_timeout(monkeypatch, caplog):
    """open_session's sessions are zenoh's (isinstance, every attribute) with a close that waits at most
    CLOSE_TIMEOUT_S, also at the end of a with block: a Zenoh close that does not return does not block it."""
    z = session.open_session(local_router=False)
    assert isinstance(z, zenoh.Session) and type(z) is session.Session
    assert str(z.info.zid()) and z.declare_publisher("t/p") is not None
    with z:
        pass  # closed at the end, at once
    assert z.is_closed()
    stuck = session.open_session(local_router=False)
    monkeypatch.setattr(session, "CLOSE_TIMEOUT_S", 0.2)
    real = stuck._session

    class Blocking:
        info = real.info

        def close(self):
            threading.Event().wait()

    object.__setattr__(stuck, "_session", Blocking())
    started = time.monotonic()
    with caplog.at_level("WARNING", logger="blick"):
        assert session.close(stuck, timeout=0.2) is False
    assert time.monotonic() - started < 1.0 and "did not close within" in caplog.text
    real.close()
