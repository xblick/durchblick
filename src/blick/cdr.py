"""Plain CDR (XCDR1, little endian) as ROS 2 puts messages on the wire.

A serialized message starts with the 4-byte encapsulation header ``00 01 00 00`` (CDR_LE). Primitives are aligned
to their own size, counted from the first byte after that header. Strings are a uint32 length (including the
terminating NUL), the bytes and the NUL; sequences are a uint32 count followed by the elements; fixed-size arrays
have no count. This is what rmw implementations (Fast DDS, Cyclone DDS, Zenoh) and rosbag2 use for ROS 2 messages,
so payloads written here can be read by ROS 2 tooling and the other way round (tests/test_cdr.py compares the bytes
with rosbags).
"""

from __future__ import annotations

import struct

import numpy as np

ENCAPSULATION_CDR_LE = b"\x00\x01\x00\x00"
_HEADER = len(ENCAPSULATION_CDR_LE)


class CdrError(ValueError):
    """A payload that is not the CDR this module reads (wrong header, truncated, inconsistent lengths)."""


class Writer:
    def __init__(self):
        self.buffer = bytearray(ENCAPSULATION_CDR_LE)

    def _align(self, size: int):
        pad = (-(len(self.buffer) - _HEADER)) % size
        if pad:
            self.buffer.extend(b"\x00" * pad)

    def _pack(self, fmt: str, size: int, value):
        self._align(size)
        self.buffer.extend(struct.pack(fmt, value))

    def u8(self, value: int):
        self.buffer.append(value & 0xFF)

    def bool(self, value: bool):
        self.buffer.append(1 if value else 0)

    def u16(self, value: int):
        self._pack("<H", 2, value)

    def i32(self, value: int):
        self._pack("<i", 4, value)

    def u32(self, value: int):
        self._pack("<I", 4, value)

    def u64(self, value: int):
        self._pack("<Q", 8, value)

    def i64(self, value: int):
        self._pack("<q", 8, value)

    def f64(self, value: float):
        self._pack("<d", 8, value)

    def string(self, value: str):
        data = value.encode("utf-8")
        if b"\x00" in data:
            raise CdrError(f"string with a NUL byte cannot be serialized: {value!r}")
        self.u32(len(data) + 1)
        self.buffer.extend(data)
        self.buffer.append(0)

    def f64_array(self, values, length: int):
        values = np.asarray(values, dtype="<f8").reshape(-1)
        if values.size != length:
            raise CdrError(f"fixed array of {length} doubles, got {values.size}")
        self._align(8)
        self.buffer.extend(values.tobytes())

    def f64_sequence(self, values):
        values = np.asarray(values, dtype="<f8").reshape(-1)
        self.u32(values.size)
        if values.size:
            self._align(8)
            self.buffer.extend(values.tobytes())

    def u8_sequence(self, data):
        view = memoryview(data).cast("B")
        self.u32(view.nbytes)
        self.buffer.extend(view)

    def i64_sequence(self, values):
        values = list(values)
        self.u32(len(values))
        for v in values:
            self.i64(v)

    def bool_sequence(self, values):
        values = list(values)
        self.u32(len(values))
        self.buffer.extend(bytes(1 if v else 0 for v in values))

    def string_sequence(self, values):
        values = list(values)
        self.u32(len(values))
        for v in values:
            self.string(v)

    def getvalue(self) -> bytearray:
        return self.buffer


class Reader:
    def __init__(self, data):
        self.view = memoryview(data).cast("B")
        if self.view.nbytes < _HEADER:
            raise CdrError(f"{self.view.nbytes} bytes are no CDR message")
        if bytes(self.view[:_HEADER]) != ENCAPSULATION_CDR_LE:
            raise CdrError(f"encapsulation {bytes(self.view[:_HEADER]).hex()} is not CDR little endian "
                           f"({ENCAPSULATION_CDR_LE.hex()})")
        self.pos = _HEADER

    def _align(self, size: int):
        self.pos += (-(self.pos - _HEADER)) % size

    def _take(self, size: int) -> memoryview:
        end = self.pos + size
        if end > self.view.nbytes:
            raise CdrError(f"truncated message: needs {end} bytes, has {self.view.nbytes}")
        chunk = self.view[self.pos:end]
        self.pos = end
        return chunk

    def _unpack(self, fmt: str, size: int):
        self._align(size)
        return struct.unpack(fmt, self._take(size))[0]

    def u8(self) -> int:
        return self._take(1)[0]

    def bool(self) -> bool:
        return self._take(1)[0] != 0

    def u16(self) -> int:
        return self._unpack("<H", 2)

    def i32(self) -> int:
        return self._unpack("<i", 4)

    def u32(self) -> int:
        return self._unpack("<I", 4)

    def u64(self) -> int:
        return self._unpack("<Q", 8)

    def i64(self) -> int:
        return self._unpack("<q", 8)

    def f64(self) -> float:
        return self._unpack("<d", 8)

    def string(self) -> str:
        length = self.u32()
        if length == 0:
            return ""  # some writers send an empty string without the NUL
        raw = self._take(length)
        if raw[-1] != 0:
            raise CdrError("string without terminating NUL")
        return bytes(raw[:-1]).decode("utf-8")

    def f64_array(self, length: int) -> np.ndarray:
        self._align(8)
        return np.frombuffer(self._take(8 * length), dtype="<f8").astype(float)

    def f64_sequence(self) -> np.ndarray:
        count = self.u32()
        if count == 0:
            return np.zeros(0)
        self._align(8)
        return np.frombuffer(self._take(8 * count), dtype="<f8").astype(float)

    def u8_sequence(self) -> memoryview:
        """A view into the payload, not a copy."""
        return self._take(self.u32())

    def i64_sequence(self) -> list[int]:
        return [self.i64() for _ in range(self.u32())]

    def bool_sequence(self) -> list[bool]:
        return [b != 0 for b in bytes(self._take(self.u32()))]

    def string_sequence(self) -> list[str]:
        return [self.string() for _ in range(self.u32())]

    def expect_end(self):
        # trailing bytes up to the next 4-byte boundary are padding some writers add
        if self.view.nbytes - self.pos >= 4:
            raise CdrError(f"{self.view.nbytes - self.pos} bytes left after the message")
