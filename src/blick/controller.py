"""A robot's controller over Durchblick (topics.py, protocol.hpp): what it is told on
``<robot>/controller/<instance>/command`` and what it reports below ``<robot>/controller/<instance>``.

The sides, each usable on its own: a client sends commands (CommandPublisher) and reads the state
(ControllerSubscriber); a controller, or a simulation in its place, takes the commands (CommandSubscriber) and
publishes its state (ControllerPublisher). A controller in C++ (protocol.hpp, ros2_cdr_msgs.hpp) follows the same
rules.

    commands = CommandPublisher(session, "robot", "arms", root_frame="base_link")
    commands.send({"left_tcp": (position, quaternion_wxyz)}, {"left_shoulder_1": 0.1, ...})

    def take(command: Command): ...                              # command.poses, command.joints, command.stamp_ns
    reader = CommandSubscriber(session, "robot", "arms", take)  # in a thread of its own

    state = ControllerPublisher(session, "robot", "arms", fields, arms, root_frame="base_link")
    state.records(records)                                       # records: count x len(fields)
    state.status({"active": "true", ...})                        # at the controller's status rate

    controller = ControllerSubscriber(session, "robot", "arms", on_records, on_status=...)

**Commands.** One message per command, moveit_msgs/RobotState: ``joint_state`` holds the joint targets (an arm's
posture, a hand's closure) and ``multi_dof_joint_state`` the controlled frames' pose targets (``joint_names`` the
frames, e.g. a TCP, ``header.frame_id`` the robot's root frame), both with the command's stamp. Addressed to one
controller, so that two clients of two controllers do not mix. A command whose stamp is not newer than the last one
taken is refused (stale), so an older command never follows a newer one; a command with poses but no frame too.

**Records.** ``<controller>/record`` (std_msgs/Float64MultiArray) carries every value of every recorded control cycle,
a batch per message (``layout.dim``: "record" x "value"); the values' names are the controller's parameter
``record_fields``. Per record also ``joint_states``, ``tf`` (per arm
``<tcp>/measured``, ``/reference``, ``/target``) and ``wrench/<arm>``; ``hands/joint_states`` and ``diagnostics``
besides.

**Status.** ``<controller>/status`` (diagnostic_msgs/DiagnosticArray with one status) at the controller's status rate
(for example 50 Hz): whether it is active, its level, the commands it accepted and refused with the last reason
(the controller's and its link's, e.g. a stale or misaddressed command), so that a client reacts within a status
period, not a diagnostics period (``diagnostics``: about once a second).
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Callable

import numpy as np
import zenoh

from . import topics
from .camera import CountedPublisher, cdr_encoding
from .diagnostics import DiagnosticsPublisher
from .msg import (DiagnosticArray, DiagnosticStatus, Float64MultiArray, Header, JointState, KeyValue,
                  MultiArrayDimension, MultiArrayLayout, MultiDOFJointState, Quaternion, RobotState, TFMessage, Time,
                  Transform, TransformStamped, Vector3, Wrench, WrenchStamped)
from .parameters import ParameterClient
from .robot import RobotStatePublisher, RobotStateSubscriber


MAX_COMMAND_AGE = 0.25  # s: a command stamped further off the controller's clock is refused
WALL_CLOCK_NS = 1_000_000_000 * 10**9  # a stamp after this (2001) is a wall-clock time, checked against the clock


@dataclass
class Command:
    """One command: ``poses`` by controlled frame: (position [m], quaternion [w, x, y, z]) in ``frame`` (the robot's
    root frame); ``joints`` by name; its stamp."""

    stamp_ns: int
    frame: str = ""
    poses: dict = field(default_factory=dict)
    joints: dict = field(default_factory=dict)


def command_message(command: Command) -> RobotState:
    """The RobotState of a command (module docstring)."""
    stamp = Time.from_ns(command.stamp_ns)
    names = list(command.joints)
    frames = list(command.poses)
    transforms = []
    for child in frames:
        position, q = command.poses[child]
        p = np.asarray(position, dtype=float)
        w, x, y, z = (float(v) for v in q)
        transforms.append(Transform(Vector3(*p), Quaternion(x, y, z, w)))
    return RobotState(JointState(Header(stamp, ""), names, np.array([float(command.joints[n]) for n in names])),
                      MultiDOFJointState(Header(stamp, command.frame), frames, transforms))


def command_of(message: RobotState) -> Command:
    """The command of a RobotState; ValueError if its two parts do not have one stamp, or a joint has no position."""
    j, d = message.joint_state, message.multi_dof_joint_state
    if j.header.stamp != d.header.stamp:
        raise ValueError(f"command: joint_state stamped {j.header.stamp.to_ns()}, multi_dof_joint_state "
                         f"{d.header.stamp.to_ns()} (one command, one stamp)")
    if len(j.position) != len(j.name):
        raise ValueError("command: joint_state needs a position per joint")
    for names, what in ((j.name, "joint"), (d.joint_names, "frame")):
        if len(set(names)) != len(names):
            raise ValueError(f"command: a {what} twice ({sorted(n for n in set(names) if names.count(n) > 1)})")
    command = Command(j.header.stamp.to_ns(), d.header.frame_id,
                      joints={n: float(v) for n, v in zip(j.name, j.position, strict=True)})
    for name, t in zip(d.joint_names, d.transforms, strict=True):
        r, v = t.rotation, t.translation
        command.poses[name] = (np.array([v.x, v.y, v.z]), np.array([r.w, r.x, r.y, r.z]))
    return command


class CommandPublisher:
    """A client's commands to one controller, ``<robot>/controller/<controller>/command`` (module docstring). Stamps
    must increase: ValueError otherwise, as the controller's side would refuse the command as stale."""

    def __init__(self, session: zenoh.Session, robot: str, controller: str, root_frame: str):
        self.root_frame = root_frame
        self.key = topics.controller_command(robot, controller)
        self._publisher = CountedPublisher(session, self.key, cdr_encoding(RobotState))
        self._last_ns = -1

    def send(self, poses: dict | None = None, joints: dict | None = None, stamp_ns: int | None = None) -> int:
        """``poses``: {controlled frame: (position, quaternion [w, x, y, z])} in the root frame; ``joints``:
        {name: position}. ``stamp_ns``: the command's time (default: now, the wall clock, which gives the controller
        its latency). Returns the stamp."""
        stamp_ns = time.time_ns() if stamp_ns is None else int(stamp_ns)
        if stamp_ns <= self._last_ns:
            raise ValueError(f"command stamp {stamp_ns} is not newer than the last one sent, {self._last_ns}")
        message = command_message(Command(stamp_ns, self.root_frame, dict(poses or {}), dict(joints or {})))
        self._publisher.put(bytes(message.serialize()))
        self._last_ns = stamp_ns
        return stamp_ns

    def close(self):
        self._publisher.close()


class CommandSubscriber:
    """Calls ``callback(command)`` once per command of ``<robot>/controller/<controller>/command``, in arrival order, in
    a thread of its own (not Zenoh's: a callback may take its time). A command whose stamp is not newer than the last
    one taken is refused as stale, one whose stamp is a wall-clock time (after 2001) more than ``max_age`` s
    off this PC's clock as too old or from the future (the clocks of client and controller disagree), one with poses
    without a frame, a joint or frame twice or an unreadable one as malformed: ``on_refused(reason)`` is called (in the same thread) and :attr:`refused` counts them. :meth:`check` raises an
    error of the callbacks."""

    def __init__(self, session: zenoh.Session, robot: str, controller: str, callback: Callable[[Command], None],
                 on_refused: Callable[[str], None] | None = None, max_age: float = MAX_COMMAND_AGE):
        self._callback = callback
        self.max_age = max_age
        self._on_refused = on_refused
        self.refused = 0
        self._lock = threading.Condition()
        self._queue: list = []  # commands, or refusal reasons (str), in arrival order
        self._last_ns = -1
        self._closed = False
        self._error: BaseException | None = None
        self._subscriber = session.declare_subscriber(topics.controller_command(robot, controller), self._on_sample)
        self._thread = threading.Thread(target=self._run, name="durchblick-commands", daemon=True)
        self._thread.start()

    def check(self):
        if self._error is not None:
            raise self._error

    def _on_sample(self, sample: zenoh.Sample):
        try:
            if str(sample.encoding) != str(cdr_encoding(RobotState)):
                item = f"command: {sample.encoding}, not {cdr_encoding(RobotState)}"
            else:
                command = command_of(RobotState.deserialize(sample.payload.to_bytes()))
                item = command
                if command.poses and not command.frame:
                    item = "command: poses without their frame (multi_dof_joint_state.header.frame_id)"
        except ValueError as error:
            item = f"command: {error}"
        with self._lock:
            if isinstance(item, Command) and item.stamp_ns > WALL_CLOCK_NS:
                off = (time.time_ns() - item.stamp_ns) * 1e-9
                if abs(off) > self.max_age:
                    item = (f"command: stamped {off:+.3f} s from now (more than {self.max_age} s: too old, or the "
                            "clocks of client and controller disagree)")
            if isinstance(item, Command):
                if item.stamp_ns <= self._last_ns:
                    item = f"command: stamp {item.stamp_ns} is not newer than the last one taken, {self._last_ns}"
                else:
                    self._last_ns = item.stamp_ns
            self._queue.append(item)
            self._lock.notify_all()

    def _run(self):
        while True:
            with self._lock:
                while not self._queue and not self._closed:
                    self._lock.wait(timeout=0.1)
                if self._closed:
                    return
                ready, self._queue = self._queue, []
            for item in ready:
                try:
                    if isinstance(item, str):
                        self.refused += 1
                        if self._on_refused is not None:
                            self._on_refused(item)
                    else:
                        self._callback(item)
                except BaseException as error:  # noqa: BLE001 - raised in the caller's thread by check()
                    self._error = error
                    return

    def close(self):
        with self._lock:
            self._closed = True
            self._lock.notify_all()
        self._thread.join(timeout=2.0)
        self._subscriber.undeclare()


def _status_array(name: str, hardware_id: str, values: dict) -> DiagnosticArray:
    return DiagnosticArray(Header(Time.from_ns(time.time_ns()), ""),
                           [DiagnosticStatus(0, name, "", hardware_id,
                                             [KeyValue(str(k), str(v)) for k, v in values.items()])])


@dataclass
class ArmFields:
    """Where an arm's values are in a record (StateRecord.h's names): ``name``, its controlled frame ``tcp``, its
    ``joints``."""

    name: str
    tcp: str
    joints: list


class ControllerPublisher:
    """A controller's state below ``<robot>/controller/<instance>`` (module docstring), from its records (rows of
    ``fields``' values). Not thread-safe: call it from one thread."""

    def __init__(self, session: zenoh.Session, robot: str, instance: str, fields: list[str], arms: list[ArmFields],
                 root_frame: str, typed: bool = True):
        self.key = topics.controller(robot, instance)
        self.fields = list(fields)
        self.root_frame = root_frame
        self.typed = typed
        index = {n: i for i, n in enumerate(self.fields)}
        self._time = index["time"]
        self._arms = [(a, [index[f"{a.name}/q/{j}"] for j in a.joints], [index[f"{a.name}/dq/{j}"] for j in a.joints],
                       [index[f"{a.name}/torque/{j}"] for j in a.joints], index[f"{a.name}/measured_pose/x"],
                       index[f"{a.name}/reference_pose/x"], index[f"{a.name}/target_pose/x"],
                       index[f"{a.name}/commanded_wrench/fx"]) for a in arms]
        self._names = [j for a in arms for j in a.joints]
        self._state = RobotStatePublisher(session, self.key)
        self._hands = RobotStatePublisher(session, f"{self.key}/hands")
        self._records = CountedPublisher(session, topics.controller(robot, instance, topics.RECORD),
                                         cdr_encoding(Float64MultiArray))
        self._diagnostics = DiagnosticsPublisher(session, self.key, "controller", instance)
        self._status = CountedPublisher(session, topics.controller(robot, instance, topics.STATUS),
                                        cdr_encoding(DiagnosticArray))
        self._instance = instance

    def records(self, records: np.ndarray):
        """Publish recorded cycles (count x len(fields)): the batch on record, per record joint_states, tf, wrench."""
        records = np.atleast_2d(np.asarray(records, dtype=np.float64))
        count, stride = records.shape
        if stride != len(self.fields):
            raise ValueError(f"records of {stride} values, the fields name {len(self.fields)}")
        self._records.put(bytes(Float64MultiArray(MultiArrayLayout([
            MultiArrayDimension("record", count, count * stride), MultiArrayDimension("value", stride, stride)]),
            records.reshape(-1)).serialize()))
        if not self.typed:
            return
        for row in records:
            stamp = Time.from_ns(int(round(row[self._time] * 1e9)))
            q, dq, tau, transforms = [], [], [], []
            for arm, qi, dqi, ti, measured, reference, target, wrench in self._arms:
                q += list(row[qi])
                dq += list(row[dqi])
                tau += list(row[ti])
                for suffix, first in (("measured", measured), ("reference", reference), ("target", target)):
                    x, y, z, qw, qx, qy, qz = row[first:first + 7]
                    transforms.append(TransformStamped(Header(stamp, self.root_frame), f"{arm.tcp}/{suffix}",
                                                       Transform(Vector3(x, y, z), Quaternion(qx, qy, qz, qw))))
                w = row[wrench:wrench + 6]
                self._state.wrench(arm.name, WrenchStamped(Header(stamp, self.root_frame),
                                                           Wrench(Vector3(*w[:3]), Vector3(*w[3:]))))
            self._state.joint_states(JointState(Header(stamp, ""), self._names, np.array(q), np.array(dq),
                                                np.array(tau)))
            self._state.tf(TFMessage(transforms))

    def hands(self, positions: dict, stamp_ns: int | None = None):
        """The relayed hands' measured positions by joint name (NaN: unknown)."""
        names = list(positions)
        self._hands.joint_states(JointState(Header(Time.from_ns(time.time_ns() if stamp_ns is None else stamp_ns), ""),
                                            names, np.array([float(positions[n]) for n in names])))

    def diagnostics(self, level: int, message: str, values: dict):
        self._diagnostics.publish(level, message, values)

    def status(self, values: dict):
        """The status at the controller's status rate (module docstring): e.g. active, level, commands_accepted,
        commands_rejected, last_reject_reason."""
        self._status.put(bytes(_status_array("status", self._instance, values).serialize()))

    def close(self):
        for p in (self._state, self._hands, self._records, self._diagnostics, self._status):
            p.close()


class ControllerSubscriber:
    """A client's view of a controller: ``on_records(records, fields)`` with every batch of records (count x
    len(fields), the names from the controller's parameter ``record_fields``), in Zenoh's thread; ``on_hands(names,
    positions)``, ``on_status(status)`` (the status at the controller's status rate) and ``on_diagnostics(status)``
    (about once a second; both diagnostic_msgs/DiagnosticStatus) likewise, if given. The fields are read when the
    controller answers (``timeout``); ConnectionError if it does not. Between two controllers of one name (a
    controller deleted and created again) the fields may change: a batch of another width raises (:meth:`check`)."""

    def __init__(self, session: zenoh.Session, robot: str, instance: str,
                 on_records: Callable[[np.ndarray, list[str]], None], on_hands: Callable | None = None,
                 on_diagnostics: Callable | None = None, timeout: float = 5.0, on_status: Callable | None = None):
        self.key = topics.controller(robot, instance)
        deadline = time.monotonic() + timeout
        while True:
            try:
                self.fields = list(ParameterClient(session, self.key, timeout=1.0).get(["record_fields"])[0]
                                   .string_array_value)
                break
            except Exception as error:  # noqa: BLE001 - it may not be up yet
                if time.monotonic() >= deadline:
                    raise ConnectionError(f"controller {self.key}: no record_fields within {timeout} s "
                                          f"({error})") from error
                time.sleep(0.2)
        self._on_records = on_records
        self._error: BaseException | None = None
        self._records = session.declare_subscriber(topics.controller(robot, instance, topics.RECORD), self._on_sample)
        self._hands = (RobotStateSubscriber(session, topics.joint_states(f"{self.key}/hands"), JointState,
                                            lambda m: on_hands(list(m.name), np.asarray(m.position)))
                       if on_hands is not None else None)
        self._on_diagnostics = on_diagnostics
        self._diagnostics = (session.declare_subscriber(
            topics.diagnostics(self.key), lambda s: self._on_diagnostic_array(s, "diagnostics", on_diagnostics))
            if on_diagnostics is not None else None)
        self._status = (session.declare_subscriber(
            topics.controller(robot, instance, topics.STATUS),
            lambda s: self._on_diagnostic_array(s, "status", on_status))
            if on_status is not None else None)

    def _on_diagnostic_array(self, sample: zenoh.Sample, name: str, callback: Callable):
        if self._error is not None:
            return
        try:
            if str(sample.encoding) != str(cdr_encoding(DiagnosticArray)):
                raise TypeError(f"{self.key}/{name}: {sample.encoding}, not {cdr_encoding(DiagnosticArray)}")
            for status in DiagnosticArray.deserialize(sample.payload.to_bytes()).status:
                callback(status)
        except BaseException as error:  # noqa: BLE001 - raised in the caller's thread by check()
            self._error = error

    def index(self, name: str) -> int:
        return self.fields.index(name)

    def _on_sample(self, sample: zenoh.Sample):
        if self._error is not None:
            return
        try:
            if str(sample.encoding) != str(cdr_encoding(Float64MultiArray)):
                raise TypeError(f"{self.key}/record: {sample.encoding}, not {cdr_encoding(Float64MultiArray)}")
            m = Float64MultiArray.deserialize(sample.payload.to_bytes())
            count, stride = (m.layout.dim[0].size, m.layout.dim[1].size) if len(m.layout.dim) == 2 else (0, 0)
            if stride != len(self.fields) or len(m.data) != count * stride:
                raise ValueError(f"{self.key}/record: {count} x {stride} values, the fields name {len(self.fields)}")
            self._on_records(np.asarray(m.data).reshape(count, stride), self.fields)
        except BaseException as error:  # noqa: BLE001 - raised in the caller's thread by check()
            self._error = error

    def check(self):
        if self._error is not None:
            raise self._error
        if self._hands is not None:
            self._hands.check()

    def close(self):
        self._records.undeclare()
        if self._hands is not None:
            self._hands.close()
        for s in (self._diagnostics, self._status):
            if s is not None:
                s.undeclare()


class JointPositionSubscriber:
    """A client's view of a joint position controller (control's ZenohJointPosition, below
    ``<robot>/controller/<instance>``): ``on_measured(message)`` with each ``joint_states`` (the measured positions,
    velocities and efforts; NaN where a joint senses none), ``on_target(message)`` with each ``target/joint_states``
    (the setpoints it holds, moving towards the commanded targets at its speed limit), ``on_status(status)`` (its
    status, diagnostic_msgs/DiagnosticStatus) if given; all in Zenoh's thread. Commands go through
    :class:`CommandPublisher` (``send(joints=...)``). Errors are raised by :meth:`check`."""

    def __init__(self, session: zenoh.Session, robot: str, instance: str, on_measured: Callable,
                 on_target: Callable | None = None, on_status: Callable | None = None):
        self.key = topics.controller(robot, instance)
        self._error: BaseException | None = None
        self._measured = RobotStateSubscriber(session, topics.joint_states(self.key), JointState, on_measured)
        self._target = (RobotStateSubscriber(session, topics.joint_states(f"{self.key}/target"), JointState, on_target)
                        if on_target is not None else None)
        self._status = (session.declare_subscriber(topics.controller(robot, instance, topics.STATUS),
                                                   lambda s: self._on_status(s, on_status))
                        if on_status is not None else None)

    def _on_status(self, sample: zenoh.Sample, callback: Callable):
        if self._error is not None:
            return
        try:
            if str(sample.encoding) != str(cdr_encoding(DiagnosticArray)):
                raise TypeError(f"{self.key}/status: {sample.encoding}, not {cdr_encoding(DiagnosticArray)}")
            for status in DiagnosticArray.deserialize(sample.payload.to_bytes()).status:
                callback(status)
        except BaseException as error:  # noqa: BLE001 - raised in the caller's thread by check()
            self._error = error

    def check(self):
        if self._error is not None:
            raise self._error
        self._measured.check()
        if self._target is not None:
            self._target.check()

    def close(self):
        self._measured.close()
        if self._target is not None:
            self._target.close()
        if self._status is not None:
            self._status.undeclare()
