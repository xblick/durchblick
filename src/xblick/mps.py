"""NVIDIA MPS for the nodes on one GPU: their kernels run side by side instead of in time slices of one process each
(xblick-mapping's README: with the robot mask, Boxer and nvblox on one GPU at 60 frames/s Boxer's latency 62 ->
30 ms, nvblox's 36 -> 22 ms).

``xblick mps --pipe-dir DIR`` runs NVIDIA's control daemon (``nvidia-cuda-mps-control -f``) as long as it runs
itself, as a process manager that starts and stops programs wants a program. A process is a client when its
``CUDA_MPS_PIPE_DIRECTORY`` is that DIR (whatever starts the nodes sets it). Measured with NVIDIA driver 580
(2026-10-30):

* Stopped with Ctrl-C or SIGTERM, it sends the daemon ``quit``: clients that run keep working, new ones start without
  MPS. The daemon itself ignores SIGINT; SIGTERM or SIGKILL to it (or to its server) end the server, and every CUDA
  call of its clients fails from then on (``cudaErrorMpsRpcFailure``): they have to be restarted. If this process is
  killed, the daemon goes with it (parent death signal).
* A process started with ``CUDA_MPS_PIPE_DIRECTORY`` where no daemon runs uses the GPU without MPS, without a word:
  ``check()`` in a node, before it uses CUDA, makes that an error.
* The daemon names Unix sockets in DIR: a longer path than they take makes it exit at once without a message.
* A client's CUDA_VISIBLE_DEVICES counts among the daemon's GPUs: with a daemon started on GPU 2 alone, a client
  that names GPU 2 finds none (it names 0). On a PC with one GPU, neither sets it.

On Zenoh it announces itself, once the daemon answers (the session's threads come after the daemon's start, which
forks): a liveliness token ``xblick/mps/<name>`` and the queryable ``xblick/mps/<name>/status`` (JSON, below),
``<name>`` the host's by default. Without a router it runs all the same, says so, and says when one is reached
(Zenoh goes on connecting); a Zenoh configuration that is wrong ends it before the daemon starts.

    {"v": 2, "kind": "mps", "name": ..., "host": ..., "pipe_dir": ..., "log_dir": ..., "pid": <this process>,
     "daemon_pid": ..., "cuda_visible_devices": <its value or null>, "gpus": [{"index", "uuid", "name"}] (nvidia-smi:
     every GPU of the PC), "servers": [{"pid", "clients": [{"pid", "command"}]}] (the daemon's get_server_list and
     get_client_list, asked at each query), "xblick": <version>; "gpus_error", "servers_error" instead when the
     tool failed}
"""

from __future__ import annotations

import ctypes
import json
import os
import re
import signal
import socket
import subprocess
import sys
import time
from pathlib import Path

import zenoh

from . import _core, topics
from . import session as zsession
from .node import end_on_signals

PIPE_ENV = "CUDA_MPS_PIPE_DIRECTORY"
LOG_ENV = "CUDA_MPS_LOG_DIRECTORY"
# sockaddr_un's path (108 bytes with its terminating zero) holds DIR/ and the daemon's longest socket name
SOCKET_PATH_MAX = 107
LONGEST_SOCKET = "control_privileged"


READY = "xblick mps: ready"      # printed once the daemon answers: a process manager can wait for this line
STATUS_VERSION = 2  # 2 (0.5.0): the field "durchblick" is "xblick"
CONTROL = "nvidia-cuda-mps-control"


def default_name() -> str:
    """The host's name as one key token (``robot-pc`` -> ``robot_pc``)."""
    name = re.sub(r"[^A-Za-z0-9_]", "_", socket.gethostname())
    return name if re.match(r"[A-Za-z_]", name) else "h_" + name


def token_key(name: str) -> str:
    return topics.check_name(f"xblick/mps/{name}")


def status_key(name: str) -> str:
    return f"{token_key(name)}/status"


def _answers(pipe: str) -> bool:
    """Whether an MPS control daemon answers in this pipe directory (its socket stays behind when it ends)."""
    with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as s:
        try:
            s.connect(os.path.join(pipe, "control"))
        except (ConnectionRefusedError, FileNotFoundError):
            return False
    return True


def check():
    """With CUDA_MPS_PIPE_DIRECTORY set, MPS is meant: raises unless an MPS control daemon answers there (CUDA would
    run this process without MPS, unseen). Call it before the process uses CUDA."""
    pipe = os.environ.get(PIPE_ENV)
    if not pipe:
        return
    if not _answers(pipe):
        raise RuntimeError(f"{PIPE_ENV}={pipe}, but no MPS control daemon answers there: start it first (xblick "
                           f"mps), or unset {PIPE_ENV} to run without MPS")


def _die_with_parent():
    """In the daemon's process before exec: SIGKILL when this process ends, so that no daemon is left unmanaged."""
    pr_set_pdeathsig = 1
    if ctypes.CDLL(None, use_errno=True).prctl(pr_set_pdeathsig, signal.SIGKILL) != 0:
        os._exit(1)


def _control(env: dict, command: str) -> str:
    """The daemon's answer to one command (``nvidia-cuda-mps-control`` reads it from stdin)."""
    return subprocess.run([CONTROL], env=env, input=command + "\n", capture_output=True, text=True, timeout=5.0,
                          check=True).stdout


def _servers(env: dict) -> list[dict]:
    """The daemon's servers (one per GPU context in use) with their clients' PIDs and commands (on this host)."""
    servers = []
    for pid in _control(env, "get_server_list").split():
        clients = []
        for client in _control(env, f"get_client_list {pid}").split():
            try:
                command = Path(f"/proc/{client}/comm").read_text().strip()
            except OSError:  # ended meanwhile, or another user's
                command = None
            clients.append({"pid": int(client), "command": command})
        servers.append({"pid": int(pid), "clients": clients})
    return servers


def _gpus() -> tuple[list[dict] | None, str | None]:
    """(the PC's GPUs from nvidia-smi, None) or (None, why not)."""
    try:
        out = subprocess.run(["nvidia-smi", "--query-gpu=index,uuid,name", "--format=csv,noheader"],
                             capture_output=True, text=True, timeout=10.0, check=True).stdout
    except (OSError, subprocess.SubprocessError) as error:
        return None, f"nvidia-smi: {error}"
    rows = [[c.strip() for c in line.split(",", 2)] for line in out.splitlines() if line.strip()]
    return [{"index": int(i), "uuid": u, "name": n} for i, u, n in rows], None


class Announcement:
    """The daemon on Zenoh: liveliness token and status queryable (module docstring)."""

    def __init__(self, session: zenoh.Session, name: str, record: dict, env: dict):
        self.name, self.record, self._env = name, record, env
        self._token = session.liveliness().declare_token(token_key(name))
        self._queryable = session.declare_queryable(status_key(name), self._answer)

    def status(self) -> dict:
        status = dict(self.record)
        try:
            status["servers"] = _servers(self._env)
        except (OSError, subprocess.SubprocessError) as error:
            status["servers_error"] = f"{CONTROL}: {error}"
        return status

    def _answer(self, query: zenoh.Query):
        query.reply(status_key(self.name), json.dumps(self.status()), encoding=zenoh.Encoding.APPLICATION_JSON)

    def close(self):
        self._queryable.undeclare()
        self._token.undeclare()


def serve(pipe_dir: str, log_dir: str | None = None, name: str | None = None, connect=(), listen=(),
          config_file: str | None = None, mode: str = "peer") -> int:
    """Runs the MPS control daemon for this process's GPUs (CUDA_VISIBLE_DEVICES) until Ctrl-C or SIGTERM, then sends
    it quit and waits for it; nonzero if the daemon ends by itself or does not end. Announced on Zenoh as ``name``
    (default: the host's) through the session ``connect``/``listen``/``config_file`` give (default: the PC's
    router)."""
    name = name or default_name()
    token_key(name)  # a wrong name ends it here, before the daemon
    config = zsession.make_config(connect, listen, mode, config_file=config_file)
    zsession.say_across_pcs(connect, listen, mode, config_file, "xblick mps")
    pipe = Path(pipe_dir).resolve()
    log = Path(log_dir).resolve() if log_dir else pipe / "log"
    longest = len(str(pipe / LONGEST_SOCKET))
    if longest > SOCKET_PATH_MAX:
        raise ValueError(f"--pipe-dir {pipe}: its socket paths would be {longest} characters, Unix sockets take "
                         f"{SOCKET_PATH_MAX} (the daemon exits without a message): use a shorter directory")
    pipe.mkdir(parents=True, exist_ok=True)
    log.mkdir(parents=True, exist_ok=True)
    env = {**os.environ, PIPE_ENV: str(pipe), LOG_ENV: str(log)}
    # Ctrl-C (ExternalApplicationManager's stop) also when started with SIGINT ignored, as a background job of a script
    # is: Python keeps an ignored SIGINT instead of raising KeyboardInterrupt
    end_on_signals()
    if _answers(str(pipe)):
        raise RuntimeError(f"an MPS control daemon runs at {pipe} already")
    # preexec_fn is unsafe in a process with threads: this one has none yet (the Zenoh session comes after)
    daemon = subprocess.Popen([CONTROL, "-f"], env=env, preexec_fn=_die_with_parent)  # noqa: PLW1509
    session = announcement = None
    try:
        ready_by = time.monotonic() + 10.0
        while not _answers(str(pipe)):
            if daemon.poll() is not None or time.monotonic() > ready_by:
                print(f"xblick mps: the control daemon did not come up (exit {daemon.poll()}): see "
                      f"{log}/control.log", file=sys.stderr, flush=True)
                if daemon.poll() is None:  # no clients yet: nothing to keep
                    daemon.kill()
                    daemon.wait()
                return 1
            time.sleep(0.05)
        gpus, gpus_error = _gpus()
        record = {"v": STATUS_VERSION, "kind": "mps", "name": name, "host": socket.gethostname(), "pipe_dir": str(pipe),
                  "log_dir": str(log), "pid": os.getpid(), "daemon_pid": daemon.pid,
                  "cuda_visible_devices": os.environ.get("CUDA_VISIBLE_DEVICES"), "xblick": _core.__version__,
                  **({"gpus": gpus} if gpus_error is None else {"gpus_error": gpus_error})}
        try:
            session = zsession.Session(zenoh.open(config), zsession.across_pcs(connect, listen, mode, config_file))
            announcement = Announcement(session, name, record, env)
        except zenoh.ZError as error:  # no clients yet: the daemon goes too
            print(f"xblick mps: no Zenoh session ({error}): stopping the daemon", file=sys.stderr, flush=True)
            daemon.kill()
            daemon.wait()
            return 1
        routers = _wait_for_router(session, 2.0)
        print(f"xblick mps: on Zenoh as {token_key(name)} (status {status_key(name)})"
              + ("" if routers else ": no Zenoh router reached yet, announced once one is (Zenoh goes on connecting)"),
              flush=True)
        print(f"{READY}: control daemon {daemon.pid} at {pipe} (log {log}); clients: {PIPE_ENV}={pipe}", flush=True)
        while (code := _wait(daemon, 1.0)) is None:
            now = bool(session.info.routers_zid())
            if now != routers:
                routers = now
                print("xblick mps: " + ("Zenoh router reached: announced" if now else
                                        "no Zenoh router any more: not announced until one is reached"),
                      flush=True)
        print(f"xblick mps: the control daemon ended by itself (exit {code}): see {log}/control.log",
              file=sys.stderr, flush=True)
        return 1
    except KeyboardInterrupt:
        # quit: the clients that run keep working (SIGTERM to the daemon would end them)
        subprocess.run(["nvidia-cuda-mps-control"], env=env, input="quit\n", text=True, check=True)
        try:
            daemon.wait(10.0)
        except subprocess.TimeoutExpired:
            print(f"xblick mps: the control daemon {daemon.pid} did not end 10 s after quit", file=sys.stderr,
                  flush=True)
            return 1
        print("xblick mps: quit; clients that run keep working", flush=True)
        return 0
    finally:
        if announcement is not None:
            announcement.close()
        if session is not None:
            session.close()


def _wait(process: subprocess.Popen, timeout: float) -> int | None:
    try:
        return process.wait(timeout)
    except subprocess.TimeoutExpired:
        return None


def _wait_for_router(session: zenoh.Session, timeout: float) -> bool:
    end = time.monotonic() + timeout
    while not session.info.routers_zid():
        if time.monotonic() > end:
            return False
        time.sleep(0.05)
    return True
