"""``xblick router``: a Zenoh router from the Zenoh library inside the package, so that a PC needs no ``zenohd``.

It runs only when started (no program starts it on its own) and listens where the documentation starts ``zenohd``,
``tcp/0.0.0.0:7447``, so that every program finds it at ``session.ROUTER`` (``127.0.0.1:7447``) as it finds
``zenohd``. Multicast scouting is off and the admin space is on (read only), as ``zenohd`` has it: the terminal
interface reads the network from it. What ``zenohd`` has and this router does not: plugins (the REST plugin, the
storage manager), a configuration file (``zenohd -c``) and ``zenohd``'s other command line options.
"""

from __future__ import annotations

import json
import socket
import sys
import time

import zenoh

from . import session as zsession

LISTEN = f"tcp/0.0.0.0:{zsession.DEFAULT_PORT}"  # zenohd's address in the documentation


class RouterError(RuntimeError):
    """The router cannot start: one line for the user."""


def config(listen: list[str], connect: list[str] = ()) -> zenoh.Config:
    """A router's configuration: listening on ``listen``, connected to the routers ``connect`` (other PCs), no
    multicast scouting, the admin space readable."""
    c = zenoh.Config()
    c.insert_json5("mode", json.dumps("router"))
    c.insert_json5("listen/endpoints", json.dumps([zsession.endpoint(e) for e in listen]))
    c.insert_json5("connect/endpoints", json.dumps([zsession.endpoint(e) for e in connect]))
    c.insert_json5("scouting/multicast/enabled", "false")
    c.insert_json5("adminspace/enabled", "true")
    return c


def open_router(listen: list[str], connect: list[str] = ()) -> zsession.Session:
    """The router, open. RouterError when it cannot listen (a port taken: a router may run there already)."""
    try:
        return zsession.Session(zenoh.open(config(listen, connect)))
    except zenoh.ZError as error:
        text = str(error)
        if "Address already in use" in text:
            raise RouterError(f"cannot listen on {', '.join(listen)}: the port is taken; a router (zenohd or xblick "
                              "router) may run there already, else give another one with --listen") from None
        raise RouterError(f"cannot start a router on {', '.join(listen)}: {text.split(' at /')[0]}") from None


def _loopback(locator: str) -> bool:
    """A locator such as tcp/127.0.0.1:7447 on a loopback address (127.0.0.0/8, ::1, localhost)."""
    host = locator.split("/", 1)[-1].rsplit(":", 1)[0].strip("[]")
    return host == "localhost" or host == "::1" or host.startswith("127.")


def serve(listen: list[str], connect: list[str] = (), prog: str = "xblick router") -> int:
    """Runs the router until Ctrl-C or SIGTERM (KeyboardInterrupt); the exit code."""
    try:
        router = open_router(listen, connect)
    except RouterError as error:
        print(f"{prog}: {error}", file=sys.stderr, flush=True)
        return 1
    first = zsession.endpoint(listen[0])
    port = first.rsplit(":", 1)[1]
    here = ("programs on this PC find it without options" if port == str(zsession.DEFAULT_PORT)
            else f"programs on this PC connect with --connect tcp/127.0.0.1:{port}")
    if all(_loopback(zsession.endpoint(e)) for e in listen):
        others = (f"other PCs cannot connect (it listens on a loopback address only; --listen tcp/0.0.0.0:{port} "
                  "opens it to them)")
    else:
        others = f"other PCs connect with --connect tcp/<this PC>:{port} (this PC: {socket.gethostname()})"
    print(f"{prog}: listening on {', '.join(zsession.endpoint(e) for e in listen)}; {here}; {others}; Ctrl-C to stop",
          flush=True)
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        router.close()
    return 0
