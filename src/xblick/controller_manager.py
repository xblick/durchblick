"""A robot's controller manager over Zenoh (protocol.hpp: ``<robot>/controller_manager/<service>``), as ROS 2's
controller_manager: list the controllers, activate and deactivate them, delete them, and, with Durchblick's own services,
create one from a type and a JSON config, read back the config it runs with and read a config file of the robot. The
robot's software, or a bridge to it, serves it.

::

    manager = ControllerManager(session, "robot")
    config = manager.config_file("Impedance", "robot")        # the robot's file, as a dict
    config["zenoh"]["key"] = "robot/controller/arms"
    manager.create("teleop_arms", "Impedance", config)         # inactive; RuntimeError with the reason if refused
    assert manager.config("teleop_arms") == config               # what it runs with (numbers within float32)
    manager.switch(activate=["teleop_arms"])                     # STRICT: all or nothing; waits until active
    ...
    manager.switch(deactivate=["teleop_arms"])
    manager.unload("teleop_arms")

A read (list, config, config_file) raises TimeoutError when nobody answers and RuntimeError when several managers
answer. Before a command (create, switch, unload, torque step), a read of the same node must have exactly one answer
and no "Timeout" reply, else the command is not sent (RuntimeError, or TimeoutError when none answered). This
is a best-effort check that catches two nodes that both answer the read. A second node that does not answer the read
(hung, dropping queries), that appears between the read and the command, or that serves a broad key expression still
gets the command. For a torque step the read counts the controller's parameter servers, not its torque_step servers.
Then the command is sent once (session.command): NotReached (a TimeoutError) when no manager became known, nothing
sent; RuntimeError with the manager's reason when it refuses; NoAnswer (no TimeoutError) when it went out and no
answer came: the robot may still execute it later, and it was not sent again; PartlyAnswered (a NoAnswer) when one
answered and another got it and did not answer in time.

Time of a command, at the worst (Zenoh not ending a query by its timeout; session.collect's deadline): the read with
the manager's ``timeout`` T (up to T + max(0.5, T) + 1 s), then ask_once with the command's timeout C (up to C for the
manager to become known, then up to C + max(0.5, C) + 1 s for the reply). ``switch()`` with the defaults (T = 2 s,
C = 2 + 5 s): up to 5 s + 7 s + 15 s = 27 s; a few ms when the manager answers at once. A manager whose read takes
longer than T gets nothing ("not sent"); before 0.5.2 the command went out. This fails safe.

A command has no expiry: a busy or briefly unreachable robot executes it when it gets it, also after the caller
gave up (NoAnswer). Check the robot's state before acting on a NoAnswer.
"""

from __future__ import annotations

import json

import zenoh

from . import _core, topics
from .msg import (
    CreateControllerRequest,
    CreateControllerResponse,
    Duration,
    GetConfigFileRequest,
    GetConfigFileResponse,
    GetControllerConfigRequest,
    GetControllerConfigResponse,
    ListControllersRequest,
    ListControllersResponse,
    SwitchControllerRequest,
    SwitchControllerResponse,
    TorqueStepRequest,
    TorqueStepResponse,
    UnloadControllerRequest,
    UnloadControllerResponse,
)
from .session import ask, command, count, timed_out

P = _core.protocol

STATES = ("unconfigured", "inactive", "active", "finalized")


def _read(session: zenoh.Session, key: str, request, response_type, timeout: float):
    """A request that only reads, asked again while nothing answers (session.ask); exactly one answer."""
    answers = ask(session, key, timeout, payload=bytes(request.serialize()))
    errors = [reply.err.payload.to_string() for reply in answers if reply.err is not None and not timed_out(reply)]
    if errors:
        raise RuntimeError(errors[0])
    ok = [reply.ok for reply in answers if reply.ok is not None]
    if not ok:
        raise TimeoutError(f"no answer from {key} within {timeout:g} s")
    if len(ok) > 1:
        raise RuntimeError(f"{len(ok)} nodes answer {key}: one robot, one controller manager")
    return response_type.deserialize(ok[0].payload.to_bytes())


def _command(session: zenoh.Session, key: str, request, response_type, timeout: float):
    """A command, sent once (session.command); its one answer."""
    ok = command(session, key, timeout, payload=bytes(request.serialize()))
    return response_type.deserialize(ok[0].payload.to_bytes())


def _count(session: zenoh.Session, read_key: str, request, timeout: float, key: str):
    """The best-effort check before a command on ``key``: the read ``read_key`` of the same node must have exactly
    one answer and no "Timeout" reply (session.count), else nothing is sent."""
    try:
        count(ask(session, read_key, timeout, payload=bytes(request.serialize())), read_key)
    except RuntimeError as error:
        raise RuntimeError(f"{error} (the command {key})") from None
    except TimeoutError as error:
        raise TimeoutError(f"{error} (the command {key})") from None


class ControllerManager:
    """The controller manager of the robot ``robot`` (its Durchblick namespace, e.g. robot)."""

    def __init__(self, session: zenoh.Session, robot: str, timeout: float = 5.0):
        self.key = f"{topics.check_name(robot.strip('/'))}/{P.CONTROLLER_MANAGER}"
        self.timeout = timeout
        self._session = session

    def _call(self, service: str, request, response_type, timeout: float | None = None, read: bool = False):
        key = f"{self.key}/{service}"
        timeout = self.timeout if timeout is None else timeout
        if read:
            return _read(self._session, key, request, response_type, timeout)
        # one manager, as far as a read shows (two would both execute it): its list_controllers has one answer
        _count(self._session, f"{self.key}/{P.LIST_CONTROLLERS}", ListControllersRequest(), self.timeout, key)
        return _command(self._session, key, request, response_type, timeout)

    def list(self) -> list:
        """Every controller (controller_manager_msgs ControllerState: name, state, type, ...)."""
        return self._call(P.LIST_CONTROLLERS, ListControllersRequest(), ListControllersResponse, read=True).controller

    def states(self) -> dict:
        """Name -> state (inactive, active) of every controller."""
        return {c.name: c.state for c in self.list()}

    def create(self, name: str, type: str, config: dict, timeout: float = 15.0):
        """Create the controller ``name`` of the class ``type`` from ``config``, inactive. RuntimeError with the
        reason when the manager refuses it (an existing name, an unknown type, a config the controller rejects)."""
        response = self._call(P.CREATE_CONTROLLER, CreateControllerRequest(name, type, json.dumps(config)),
                              CreateControllerResponse, timeout)
        if not response.ok:
            raise RuntimeError(f"controller {name} ({type}) not created: {response.message}")

    def config(self, name: str) -> dict:
        """The config the controller ``name`` runs with."""
        response = self._call(P.GET_CONTROLLER_CONFIG, GetControllerConfigRequest(name), GetControllerConfigResponse,
                              read=True)
        if not response.ok:
            raise RuntimeError(f"no config of controller {name}: {response.message}")
        return json.loads(response.config)

    def config_file(self, type: str, name: str) -> dict:
        """The robot's config file ``<type>/<name>``."""
        response = self._call(P.GET_CONFIG_FILE, GetConfigFileRequest(type, name), GetConfigFileResponse, read=True)
        if not response.ok:
            raise RuntimeError(f"no config file {type}/{name}: {response.message}")
        return json.loads(response.config)

    def switch(self, activate=(), deactivate=(), strict: bool = True, timeout: float = 5.0) -> str:
        """Deactivate ``deactivate``, then activate ``activate``, and wait up to ``timeout`` s for the states. STRICT
        (default): all or nothing; else BEST_EFFORT. Returns the manager's message; RuntimeError when not ok."""
        request = SwitchControllerRequest(list(activate), list(deactivate),
                                          SwitchControllerRequest.STRICT if strict else SwitchControllerRequest.BEST_EFFORT,
                                          False, Duration.from_seconds(timeout))
        response = self._call(P.SWITCH_CONTROLLER, request, SwitchControllerResponse, timeout + self.timeout)
        if not response.ok:
            raise RuntimeError(f"switch refused: {response.message}")
        return response.message

    def unload(self, name: str):
        """Delete the inactive controller ``name``. RuntimeError when it is active or unknown."""
        if not self._call(P.UNLOAD_CONTROLLER, UnloadControllerRequest(name), UnloadControllerResponse).ok:
            raise RuntimeError(f"controller {name} not unloaded (active, or unknown)")


def torque_step(session: zenoh.Session, key: str, joint: str, torque: float, duration: float,
                timeout: float = 2.0) -> None:
    """A controller's bring-up torque step (its service ``<key>/torque_step``; key: ``<robot>/controller/<topic>``):
    ``torque`` [Nm] on ``joint`` for ``duration`` [s]. RuntimeError with the reason when it refuses.

    Not sent unless exactly one controller answers a read of its parameters (``<key>/list_parameters``; every
    controller offers parameters) and none answers it late. Best effort: the read counts the controller's parameter
    servers, not its torque_step servers, so a second torque_step server without parameters (an older bridge), one
    that drops the read, or one that starts after it still gets the step. ``timeout`` [s]: how long to wait for the
    controller to become known and then for its answer; it is no deadline for the step. NoAnswer: sent once, no
    answer within ``timeout``; the controller may still execute the step later (a command has no expiry), and it was
    not sent again."""
    from .msg import ListParametersRequest

    key = topics.check_name(key.strip('/'))
    _count(session, f"{key}/{P.LIST_PARAMETERS}", ListParametersRequest([], 0), timeout, f"{key}/{P.TORQUE_STEP}")
    response = _command(session, f"{key}/{P.TORQUE_STEP}", TorqueStepRequest(joint, float(torque), float(duration)),
                        TorqueStepResponse, timeout)
    if not response.ok:
        raise RuntimeError(f"torque step refused: {response.message}")
