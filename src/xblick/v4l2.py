"""The V4L2 devices of this PC and whether each captures video: ``xblick cam`` without a source lists them (cam.py).

A webcam usually has two or more device nodes: one that captures video and one that delivers metadata (UVC). The
kernel tells them apart in the node's capabilities, which the ioctl VIDIOC_QUERYCAP returns (struct v4l2_capability,
linux/videodev2.h): with V4L2_CAP_DEVICE_CAPS set, ``device_caps`` holds the node's own capabilities, and a node
captures video when they contain V4L2_CAP_VIDEO_CAPTURE or V4L2_CAP_VIDEO_CAPTURE_MPLANE. The ioctl needs the node
opened; a node this user cannot open (no access to the group ``video``) is listed with its capabilities unknown.
The name is the ioctl's ``card``, else the node's name in sysfs (/sys/class/video4linux/<node>/name).
"""

from __future__ import annotations

import errno
import fcntl
import os
import re
import struct
from dataclasses import dataclass
from pathlib import Path

# struct v4l2_capability: driver[16], card[32], bus_info[32], version, capabilities, device_caps, reserved[3]
CAPABILITY = struct.Struct("<16s32s32sIII12x")
VIDIOC_QUERYCAP = (2 << 30) | (CAPABILITY.size << 16) | (ord("V") << 8) | 0  # _IOR('V', 0, struct v4l2_capability)
CAP_VIDEO_CAPTURE = 0x00000001
CAP_VIDEO_CAPTURE_MPLANE = 0x00001000
CAP_META_CAPTURE = 0x00800000
CAP_DEVICE_CAPS = 0x80000000
SYSFS = Path("/sys/class/video4linux")


@dataclass
class Capability:
    """What VIDIOC_QUERYCAP says about a device node."""

    driver: str
    card: str
    bus_info: str
    capabilities: int
    device_caps: int

    @property
    def node_caps(self) -> int:
        """The node's own capabilities: device_caps when the driver fills it, else those of the whole device."""
        return self.device_caps if self.capabilities & CAP_DEVICE_CAPS else self.capabilities

    @property
    def captures_video(self) -> bool:
        return bool(self.node_caps & (CAP_VIDEO_CAPTURE | CAP_VIDEO_CAPTURE_MPLANE))


def parse_capability(data: bytes) -> Capability:
    """struct v4l2_capability as VIDIOC_QUERYCAP filled it."""
    if len(data) != CAPABILITY.size:
        raise ValueError(f"struct v4l2_capability: {CAPABILITY.size} bytes, not {len(data)}")
    driver, card, bus_info, _version, capabilities, device_caps = CAPABILITY.unpack(data)

    def text(raw: bytes) -> str:
        return raw.split(b"\0", 1)[0].decode("utf-8", "replace").strip()

    return Capability(text(driver), text(card), text(bus_info), capabilities, device_caps)


@dataclass
class Device:
    """A /dev/video* node: ``captures`` True or False from its capabilities, None when they could not be read
    (``why``)."""

    path: str
    name: str
    captures: bool | None
    why: str = ""

    def text(self) -> str:
        return f"{self.path}  {self.name or '(no name)'}" + (f"  ({self.why})" if self.why else "")


def _number(path: Path) -> tuple[int, str]:
    digits = re.sub(r"\D", "", path.name)
    return (int(digits) if digits else -1, path.name)


def query(path: Path) -> Capability:
    """VIDIOC_QUERYCAP of a device node (OSError when it cannot be opened or asked)."""
    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
    try:
        return parse_capability(fcntl.ioctl(fd, VIDIOC_QUERYCAP, bytes(CAPABILITY.size)))
    finally:
        os.close(fd)


def devices(dev: Path = Path("/dev"), sysfs: Path = SYSFS) -> list[Device]:
    """Every /dev/video* node in number order, with its name and whether it captures video."""
    found = []
    for path in sorted(dev.glob("video*"), key=_number):
        name_file = sysfs / path.name / "name"
        name = name_file.read_text().strip() if name_file.is_file() else ""
        try:
            capability = query(path)
        except OSError as error:
            reason = errno.errorcode.get(error.errno, "") if error.strerror is None else error.strerror
            found.append(Device(str(path), name, None, f"capabilities unknown: {reason}"))
            continue
        found.append(Device(str(path), capability.card or name, capability.captures_video))
    return found


def capture_devices(listed: list[Device]) -> list[Device]:
    """The nodes that capture video, and those whose capabilities are unknown (they may)."""
    return [d for d in listed if d.captures is not False]
