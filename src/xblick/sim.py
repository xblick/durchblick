"""What a simulation publishes besides its cameras, every sample stamped with the simulated time (topics.py,
protocol.hpp):

* the robot's state as the robot's (robot.py): ``<robot>/joint_states`` at the control rate, ``<robot>/tf`` and
  ``<robot>/odom`` with the state;
* its world below a namespace of its own (``sim``): ``<world>/tf`` (tf2_msgs/TFMessage: every body's pose in
  ``world``, child the body's name), ``<world>/clock`` (rosgraph_msgs/Clock) and ``<world>/events`` (JSON task
  events: ``{"v":1,"stamp_ns":...,"event":"phase","phase":"grasp"}``; ``start``, ``phase``, ``success``, ``failure``,
  ``end``, or any other name);
* what its controller is told: ``<robot>/command/joints`` (sensor_msgs/JointState: per joint the commanded position,
  velocity or effort; a hand's closure as its position) and ``<robot>/command/tf`` (tf2_msgs/TFMessage: commanded
  poses such as a hand's TCP target).

Rates, by the samples' simulated stamps: joint states, commands and the clock at most ``control_hz`` (1000, the
controller's 1 ms), the state (bodies, the robot's tf and odom) at most ``state_hz``: by default every call, so call
:meth:`bodies` with the frames of the fastest camera, or give its rate. A call faster than its rate is skipped
(returns False). Events are never skipped. With ``blocking`` every key blocks instead of dropping and answers
``<key>/sent``, so that a recording can be checked to have every sample (recording.py, "how a simulation records"):

    sim = SimPublisher(session, "robot", blocking=True)
    sim.clock(stamp_ns)
    sim.joint_states(JointState(Header(Time.from_ns(stamp_ns)), names, position, velocity, effort))
    sim.command_joints(JointState(Header(Time.from_ns(stamp_ns)), ["hand_left"], [0.8]))
    sim.bodies(TFMessage([TransformStamped(Header(Time.from_ns(stamp_ns), "world"), "mug", transform), ...]))
    sim.event(stamp_ns, "phase", phase="grasp")
"""

from __future__ import annotations

import json

import zenoh

from . import topics
from .camera import CountedPublisher, cdr_encoding
from .msg import Clock, JointState, Message, Odometry, TFMessage, Time
from .robot import RobotStatePublisher

CONTROL_HZ = 1000.0  # the simulated controller's cycle, 1 ms
EVENT_VERSION = 1


class Rate:
    """At most ``hz`` samples a second of simulated time (None: every one), on a fixed schedule from the first stamp,
    so that steps that do not divide the period keep the rate on average; after a gap the schedule starts again.
    Stamps that go back are an error."""

    def __init__(self, hz: float | None):
        if hz is not None and hz <= 0:
            raise ValueError(f"rate {hz} Hz: > 0, or None for every sample")
        self.period_ns = None if hz is None else round(1e9 / hz)
        self._due: int | None = None
        self._last: int | None = None

    def due(self, stamp_ns: int) -> bool:
        if self._last is not None and stamp_ns < self._last:
            raise ValueError(f"stamp {stamp_ns} ns before the last one ({self._last} ns): simulated time goes back")
        self._last = stamp_ns
        if self.period_ns is None:
            return True
        # a hundredth of a period of tolerance: simulated stamps are sums of float steps
        if self._due is not None and stamp_ns + self.period_ns // 100 < self._due:
            return False
        late = self._due is None or stamp_ns - self._due >= self.period_ns
        self._due = (stamp_ns if late else self._due) + self.period_ns
        return True


def _stamp_of(message: Message) -> int:
    if isinstance(message, TFMessage):
        if not message.transforms:
            raise ValueError("an empty TFMessage has no stamp")
        return message.transforms[0].header.stamp.to_ns()
    return message.header.stamp.to_ns()


class SimPublisher:
    """A simulation's robot state, world, commands and events (module docstring). Not thread-safe: call it from the
    simulation's loop. ``source`` goes into every sample's attachment (``sim``)."""

    def __init__(self, session: zenoh.Session, robot: str, world: str = topics.WORLD_NAMESPACE,
                 control_hz: float | None = CONTROL_HZ, state_hz: float | None = None, blocking: bool = False,
                 source: str = "sim"):
        self.robot_name = topics.check_name(robot)
        self.world = topics.check_name(world)
        if self.world == self.robot_name:
            raise ValueError(f"world {world!r}: not the robot's namespace (its tf is the robot's)")
        self.blocking = blocking
        self.robot = RobotStatePublisher(session, robot, blocking=blocking, source=source)

        def publisher(key: str, encoding) -> CountedPublisher:
            return CountedPublisher(session, key, encoding, blocking=blocking, source=source)

        self._bodies = publisher(topics.tf(world), cdr_encoding(TFMessage))
        self._clock = publisher(topics.clock(world), cdr_encoding(Clock))
        self._events = publisher(topics.events(world), zenoh.Encoding.APPLICATION_JSON)
        self._command_joints = publisher(topics.command(robot, "joints"), cdr_encoding(JointState))
        self._command_tf = publisher(topics.command(robot, "tf"), cdr_encoding(TFMessage))
        self._rates = {name: Rate(control_hz) for name in ("joint_states", "clock", "command_joints", "command_tf")}
        self._rates.update({name: Rate(state_hz) for name in ("bodies", "tf", "odom")})

    @property
    def keys(self) -> list[str]:
        """The key expressions of everything it publishes (the wrenches as ``<robot>/wrench/*``): what to record."""
        return [topics.joint_states(self.robot_name), topics.tf(self.robot_name), topics.odom(self.robot_name),
                f"{self.robot_name}/{topics.WRENCH}/*", self._bodies.key, self._clock.key, self._events.key,
                self._command_joints.key, self._command_tf.key]

    def _due(self, name: str, message: Message) -> bool:
        return self._rates[name].due(_stamp_of(message))

    def joint_states(self, message: JointState) -> bool:
        if not self._due("joint_states", message):
            return False
        self.robot.joint_states(message)
        return True

    def tf(self, message: TFMessage) -> bool:
        """The robot's ``<robot>/tf`` (robot.py: the localisation chain and every robot frame relative to the root)."""
        if not self._due("tf", message):
            return False
        self.robot.tf(message)
        return True

    def odom(self, message: Odometry) -> bool:
        if not self._due("odom", message):
            return False
        self.robot.odom(message)
        return True

    def bodies(self, message: TFMessage) -> bool:
        """Every simulated body's pose in the world (``<world>/tf``)."""
        if not self._due("bodies", message):
            return False
        self._bodies.put(bytes(_typed(message, TFMessage).serialize()))
        return True

    def command_joints(self, message: JointState) -> bool:
        if not self._due("command_joints", message):
            return False
        self._command_joints.put(bytes(_typed(message, JointState).serialize()))
        return True

    def command_tf(self, message: TFMessage) -> bool:
        if not self._due("command_tf", message):
            return False
        self._command_tf.put(bytes(_typed(message, TFMessage).serialize()))
        return True

    def clock(self, stamp_ns: int) -> bool:
        message = Clock(Time.from_ns(stamp_ns))
        if not self._rates["clock"].due(stamp_ns):
            return False
        self._clock.put(bytes(message.serialize()))
        return True

    def event(self, stamp_ns: int, event: str, **fields):
        """A task event (``start``, ``phase`` with ``phase=...``, ``success``, ``failure`` with ``reason=...``,
        ``end``, ...): JSON on ``<world>/events``; ``fields`` must be JSON."""
        if not isinstance(event, str) or not event:
            raise ValueError(f"event {event!r}: a name")
        reserved = {"v", "stamp_ns", "event"} & set(fields)
        if reserved:
            raise ValueError(f"event fields {sorted(reserved)} are the event's own")
        self._events.put(json.dumps({"v": EVENT_VERSION, "stamp_ns": int(stamp_ns), "event": event, **fields},
                                    separators=(",", ":")).encode())

    def close(self):
        for publisher in (self._bodies, self._clock, self._events, self._command_joints, self._command_tf):
            publisher.close()
        self.robot.close()


def _typed(message, message_type: type):
    if not isinstance(message, message_type):
        raise TypeError(f"{message_type.__name__} expected, not {type(message).__name__}")
    return message
