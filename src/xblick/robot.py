"""A robot's state over Durchblick, below its namespace (topics.py, protocol.hpp): ``<robot>/joint_states``
(sensor_msgs/JointState), ``<robot>/tf`` (tf2_msgs/TFMessage: world -> map -> odom -> root and every robot frame
relative to the root), ``<robot>/odom`` (nav_msgs/Odometry) and ``<robot>/wrench/<sensor>`` (geometry_msgs/WrenchStamped),
one message per state sample, stamped with the robot's time. On the robot its own software, or a bridge to
it, is the source, in the simulation the simulator.

    state = RobotStatePublisher(session, "robot")
    state.joint_states(JointState(header=Header(Time.from_ns(stamp_ns)), name=[...], position=[...]))
    state.tf(TFMessage(transforms=[...]))

    reader = RobotStateSubscriber(session, topics.joint_states("robot"), JointState, callback)  # callback(message)
"""

from __future__ import annotations

from typing import Callable

import zenoh

from . import topics
from .camera import CountedPublisher, _check_encoding, cdr_encoding
from .msg import JointState, Odometry, TFMessage, WrenchStamped

ROBOT_TYPES = (JointState, TFMessage, Odometry, WrenchStamped)


class RobotStatePublisher:
    """Publishes one robot's state; every message is sent (with the attachment and its seq, camera.CountedPublisher).
    A force-torque sensor's key is declared with its first measurement. ``blocking``: blocking congestion control and
    the counts on ``<key>/sent``, for recordings that must have every sample (recording.py); the default drops."""

    def __init__(self, session: zenoh.Session, robot: str, blocking: bool = False, source: str = ""):
        self.robot = topics.check_name(robot)
        self.blocking = blocking
        self.source = source
        self._session = session
        self._joint_states = self._publisher(topics.joint_states(robot), JointState)
        self._tf = self._publisher(topics.tf(robot), TFMessage)
        self._odom = self._publisher(topics.odom(robot), Odometry)
        self._wrenches: dict[str, CountedPublisher] = {}

    def _publisher(self, key: str, message_type: type) -> CountedPublisher:
        return CountedPublisher(self._session, key, cdr_encoding(message_type), blocking=self.blocking,
                                source=self.source)

    @property
    def keys(self) -> list[str]:
        """The keys declared so far (a sensor's wrench from its first measurement on)."""
        return [p.key for p in (self._joint_states, self._tf, self._odom, *self._wrenches.values())]

    def joint_states(self, message: JointState):
        self._joint_states.put(bytes(_typed(message, JointState).serialize()))

    def tf(self, message: TFMessage):
        self._tf.put(bytes(_typed(message, TFMessage).serialize()))

    def odom(self, message: Odometry):
        self._odom.put(bytes(_typed(message, Odometry).serialize()))

    def wrench(self, sensor: str, message: WrenchStamped):
        if sensor not in self._wrenches:
            self._wrenches[sensor] = self._publisher(topics.wrench(self.robot, sensor), WrenchStamped)
        self._wrenches[sensor].put(bytes(_typed(message, WrenchStamped).serialize()))

    def close(self):
        for publisher in (self._joint_states, self._tf, self._odom, *self._wrenches.values()):
            publisher.close()


class RobotStateSubscriber:
    """Calls ``callback(message)`` with every message of ``message_type`` on one key of a robot's state, in Zenoh's
    thread. A sample of another type or that does not decode stops the deliveries; :meth:`check` raises it."""

    def __init__(self, session: zenoh.Session, key: str, message_type: type, callback: Callable):
        if message_type not in ROBOT_TYPES:
            raise TypeError(f"{message_type.__name__}: one of {', '.join(t.__name__ for t in ROBOT_TYPES)}")
        self.key = topics.check_name(key)
        self.message_type = message_type
        self._callback = callback
        self._error: BaseException | None = None
        self._subscriber = session.declare_subscriber(self.key, self._on_sample)

    def _on_sample(self, sample: zenoh.Sample):
        if self._error is not None:
            return
        try:
            _check_encoding(self.key, str(sample.encoding), self.message_type)
            self._callback(self.message_type.deserialize(sample.payload.to_bytes()))
        except BaseException as error:  # noqa: BLE001 - raised in the caller's thread by check()
            self._error = error

    def check(self):
        if self._error is not None:
            raise self._error

    def close(self):
        self._subscriber.undeclare()


def _typed(message, message_type: type):
    if not isinstance(message, message_type):
        raise TypeError(f"{message_type.__name__} expected, not {type(message).__name__}")
    return message
