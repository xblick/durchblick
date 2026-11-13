"""The time a sample carries and how old it was when it was sent.

* ``header_stamp``: the stamp a sample carries (its header's: the capture of the image it is or is about).
* ``age_at_send_ns``: ``publish_ns`` of its attachment minus that stamp: how old the sample was when it went out of
  its publisher (``publish_ns`` is taken right before each transport's put: with its encoding or its copy into a
  ring; ``publish_called_ns`` is when publish was called), measured on one host, so exact whatever the clocks of other
  hosts. ``Frame.age_at_send_ns`` (camera, points) is the same for a received frame.

The age at receive (``receive_ns`` minus the stamp) includes the way to the receiver and compares two hosts' clocks:
meaningful between hosts only with synchronised clocks.
"""

from __future__ import annotations

import dataclasses
import functools
import json
import struct

import zenoh

from .camera import parse_attachment
from .msg import Header, Message, TFMessage, Time
from .msgdefs import MSGDEFS

CDR = "application/cdr;"
JSON = "application/json"


@functools.cache
def _starting_with_a_stamp() -> frozenset[str]:
    """The CDR encodings of the message types that start with a header or a time (ROS's and Durchblick's own)."""
    found, todo = set(), [Message]
    while todo:
        cls = todo.pop()
        todo.extend(cls.__subclasses__())
        if cls.TYPE and dataclasses.is_dataclass(cls):
            fields = dataclasses.fields(cls)  # none: an empty message (e.g. ListControllers_Request)
            if fields and fields[0].type in ("Header", Header, "Time", Time):
                found.add(cls.ENCODING)
    return frozenset(found)


def header_stamp(key: str, encoding: str, payload: bytes) -> int:
    """The stamp (ns) a sample carries: its header's (a message that starts with std_msgs/Header, also Durchblick's
    ring descriptors), a Clock's, the first transform's of a TFMessage, a JSON sample's ``stamp_ns``. A sample without
    one is a ValueError."""
    if encoding.startswith(CDR):
        name = encoding[len(CDR):]
        if len(payload) < 4 or payload[0] != 0 or payload[1] != 1:
            raise ValueError(f"{key}: not little-endian CDR")
        if name == TFMessage.TYPE:
            if len(payload) < 16 or struct.unpack_from("<I", payload, 4)[0] == 0:
                raise ValueError(f"{key}: a TFMessage without transforms has no stamp")
            offset = 8
        elif (encoding in _starting_with_a_stamp()
              or MSGDEFS.get(name, "").startswith(("std_msgs/Header header\n", "builtin_interfaces/Time clock\n"))):
            offset = 4
        else:
            raise ValueError(f"{key}: {name} has no header stamp (record it with log_time receive)")
        if len(payload) < offset + 8:
            raise ValueError(f"{key}: {len(payload)} bytes, too short for a stamp")
        sec, nanosec = struct.unpack_from("<iI", payload, offset)
        stamp = sec * 1_000_000_000 + nanosec
    elif encoding == JSON:
        sample = json.loads(payload)
        stamp = sample.get("stamp_ns") if isinstance(sample, dict) else None
        if not isinstance(stamp, int):
            raise ValueError(f"{key}: a JSON sample without an integer stamp_ns")
    else:
        raise ValueError(f"{key}: {encoding!r} has no stamp it can read (record it with log_time receive)")
    if stamp < 0:
        raise ValueError(f"{key}: stamp {stamp} ns before 1970: no MCAP log time")
    return stamp


def age_at_send(stamp_ns: int | None, publish_ns: int | None) -> int | None:
    """``publish_ns - stamp_ns``, None without either (or with a stamp of 0, which no capture has)."""
    if not stamp_ns or publish_ns is None:
        return None
    return publish_ns - stamp_ns


def age_at_send_ns(sample: zenoh.Sample) -> int | None:
    """How old a sample's stamp was when its publisher sent it (module docstring); None without the attachment's
    ``publish_ns`` (a publisher that sends none, before Durchblick 0.3.30 also detections) or without a stamp."""
    attachment = sample.attachment
    publish_ns = parse_attachment(attachment.to_bytes() if attachment is not None else None).get("publish_ns")
    if publish_ns is None:
        return None
    try:
        stamp = header_stamp(str(sample.key_expr), str(sample.encoding), sample.payload.to_bytes())
    except ValueError:  # a message without a stamp
        return None
    return age_at_send(stamp, publish_ns)
