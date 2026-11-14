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

Every call raises RuntimeError with the manager's reason when it refuses, TimeoutError when nobody answers, and
RuntimeError when several managers answer (one robot, one manager).
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
from .session import ask, replies

P = _core.protocol

STATES = ("unconfigured", "inactive", "active", "finalized")


def _call(session: zenoh.Session, key: str, request, response_type, timeout: float, read: bool = False):
    """One service call; ``read``: a request that only reads, asked again while nothing answers (session.ask)."""
    answers = (ask if read else replies)(session, key, timeout, payload=bytes(request.serialize()))
    errors = [reply.err.payload.to_string() for reply in answers if reply.err is not None]
    if errors:
        raise RuntimeError(errors[0])
    ok = [reply.ok for reply in answers if reply.ok is not None]
    if not ok:
        raise TimeoutError(f"no answer from {key} within {timeout:g} s")
    if len(ok) > 1:
        raise RuntimeError(f"{len(ok)} nodes answer {key}: one robot, one controller manager")
    return response_type.deserialize(ok[0].payload.to_bytes())


class ControllerManager:
    """The controller manager of the robot ``robot`` (its Durchblick namespace, e.g. robot)."""

    def __init__(self, session: zenoh.Session, robot: str, timeout: float = 5.0):
        self.key = f"{topics.check_name(robot.strip('/'))}/{P.CONTROLLER_MANAGER}"
        self.timeout = timeout
        self._session = session

    def _call(self, service: str, request, response_type, timeout: float | None = None, read: bool = False):
        return _call(self._session, f"{self.key}/{service}", request, response_type,
                     self.timeout if timeout is None else timeout, read)

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
    ``torque`` [Nm] on ``joint`` for ``duration`` [s]. RuntimeError with the reason when it refuses."""
    response = _call(session, f"{topics.check_name(key.strip('/'))}/{P.TORQUE_STEP}",
                     TorqueStepRequest(joint, float(torque), float(duration)), TorqueStepResponse, timeout)
    if not response.ok:
        raise RuntimeError(f"torque step refused: {response.message}")
