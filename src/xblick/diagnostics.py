"""A node's state on ``<camera>/diagnostics`` (diagnostic_msgs/DiagnosticArray, as ROS's ``/diagnostics``), about once a
second: one status named after the node, its level, a message and numeric values. ``xblick diagnostics`` shows
them, ``xblick rerun`` plots them.
Every sample's attachment names the publishing session (``zid``, ``sender``): several nodes report on one camera's key.

    diagnostics = DiagnosticsPublisher(session, "test/camera", "nemo")
    diagnostics.publish(DiagnosticsPublisher.OK, "9.9 frames/s", {"frames/s": 9.9, "objects": 3})
"""

from __future__ import annotations

import time

import zenoh

from . import topics
from .camera import _attachment, _check_encoding, cdr_encoding, parse_attachment
from .msg import DiagnosticArray, DiagnosticStatus, Header, KeyValue, Time

LEVEL_NAMES = {DiagnosticStatus.OK: "OK", DiagnosticStatus.WARN: "WARN", DiagnosticStatus.ERROR: "ERROR",
               DiagnosticStatus.STALE: "STALE"}


def level_name(level: int) -> str:
    return LEVEL_NAMES.get(level, str(level))


def read(sample: zenoh.Sample) -> tuple[str, DiagnosticArray]:
    """(camera, its diagnostics) of a sample on ``<camera>/diagnostics``; another type raises ProtocolError."""
    key = str(sample.key_expr)
    _check_encoding(key, str(sample.encoding), DiagnosticArray)
    return key[: -len(topics.DIAGNOSTICS) - 1], DiagnosticArray.deserialize(sample.payload.to_bytes())


def sender(sample: zenoh.Sample) -> str | None:
    """The zid of the session that published a diagnostics sample (its attachment's ``zid``); None from a publisher
    before Durchblick 0.3.29, which sent no attachment."""
    attachment = sample.attachment
    return parse_attachment(attachment.to_bytes() if attachment is not None else None).get("zid")


class DiagnosticsPublisher:
    OK, WARN, ERROR, STALE = DiagnosticStatus.OK, DiagnosticStatus.WARN, DiagnosticStatus.ERROR, DiagnosticStatus.STALE

    def __init__(self, session: zenoh.Session, camera: str, name: str, hardware_id: str = ""):
        self.name = name
        self.hardware_id = hardware_id
        self.seq = 0
        self._zid = str(session.info.zid())  # in every sample: the nodes of a camera share its diagnostics key
        self._publisher = session.declare_publisher(topics.diagnostics(camera), encoding=cdr_encoding(DiagnosticArray))

    def publish(self, level: int, message: str, values: dict):
        status = DiagnosticStatus(level, self.name, message, self.hardware_id,
                                  [KeyValue(k, f"{v:.2f}" if isinstance(v, float) else str(v)) for k, v in values.items()])
        self.seq += 1
        self._publisher.put(bytes(DiagnosticArray(Header(Time.from_ns(time.time_ns()), ""), [status]).serialize()),
                            attachment=_attachment(self.seq, "", self._zid))

    def close(self):
        self._publisher.undeclare()
