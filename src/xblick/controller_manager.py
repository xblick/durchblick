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
answer. A command (create, switch, unload, torque step) is first refused, before anything is sent, unless exactly one
manager (one controller) answers a read of it; then it is sent once (session.command): NotReached (a TimeoutError)
when no manager became known, nothing sent; RuntimeError with the manager's reason when it refuses; NoAnswer (no
TimeoutError) when it went out and no answer came: the robot may still execute it later, and it was not sent again.

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
from .session import ask, command, timed_out

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
    if len(ok) > 1:
        raise RuntimeError(f"{len(ok)} nodes answered {key}: each of them got the command; one robot, one manager")
    return response_type.deserialize(ok[0].payload.to_bytes())


def _one_server(check, key: str):
    """``check()``, a read of the node that serves ``key``: a RuntimeError (several answer) or TimeoutError (none
    does) of it means the command is not sent."""
    try:
        check()
    except RuntimeError as error:
        raise RuntimeError(f"not sent: {key}: {error}") from None
    except TimeoutError as error:
        raise TimeoutError(f"not sent: {key}: {error}") from None


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
        # one manager, else nothing is sent (two would both execute it): its list_controllers has one answer
        _one_server(self.list, key)
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
    controller offers parameters). ``timeout`` [s]: how long to wait for the controller to become known and then for
    its answer; it is no deadline for the step. NoAnswer: sent once, no answer within ``timeout``; the controller may
    still execute the step later (a command has no expiry), and it was not sent again."""
    from .parameters import ParameterClient

    key = topics.check_name(key.strip('/'))
    _one_server(lambda: ParameterClient(session, key, timeout).list(), f"{key}/{P.TORQUE_STEP}")
    response = _command(session, f"{key}/{P.TORQUE_STEP}", TorqueStepRequest(joint, float(torque), float(duration)),
                        TorqueStepResponse, timeout)
    if not response.ok:
        raise RuntimeError(f"torque step refused: {response.message}")
