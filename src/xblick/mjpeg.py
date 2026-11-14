"""MJPEG frames passed on as JPEG files: ``xblick cam --mjpeg passthrough`` (cam.py).

Many USB (UVC) cameras send MJPEG frames without Huffman tables: the frames use the standard tables of the JPEG
standard, and the UVC payload format leaves them out. A JPEG decoder that does not know this convention refuses
such a frame. ``with_huffman_tables`` therefore inserts a DHT segment with the standard tables before the frame's
first SOS segment when the frame has no DHT segment of its own there, as FFmpeg's bitstream filter mjpeg2jpeg does.
Everything else of the frame stays as it is (an AVI1 APP0 segment too: decoders skip APP segments they do not know).

The tables are those of ITU-T T.81 (the JPEG standard), Annex K.3, tables K.3 to K.6 (luminance and chrominance,
DC and AC): the counts of codes of each length 1 to 16 (BITS) and the symbols in code order (HUFFVAL).
"""

from __future__ import annotations

import struct

# T.81 Table K.3: DC luminance; Table K.4: DC chrominance
DC_LUMINANCE_BITS = (0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0)
DC_CHROMINANCE_BITS = (0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0)
DC_VALUES = tuple(range(12))
# T.81 Table K.5: AC luminance
AC_LUMINANCE_BITS = (0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 0x7D)
AC_LUMINANCE_VALUES = bytes.fromhex(
    "01020300041105122131410613516107227114328191a1082342b1c11552d1f0"
    "2433627282090a161718191a25262728292a3435363738393a434445464748494a"
    "535455565758595a636465666768696a737475767778797a838485868788898a"
    "92939495969798999aa2a3a4a5a6a7a8a9aab2b3b4b5b6b7b8b9bac2c3c4c5c6"
    "c7c8c9cad2d3d4d5d6d7d8d9dae1e2e3e4e5e6e7e8e9eaf1f2f3f4f5f6f7f8f9fa")
# T.81 Table K.6: AC chrominance
AC_CHROMINANCE_BITS = (0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 0x77)
AC_CHROMINANCE_VALUES = bytes.fromhex(
    "000102031104052131061241510761711322328108144291a1b1c109233352f0"
    "156272d10a162434e125f11718191a262728292a35363738393a434445464748"
    "494a535455565758595a636465666768696a737475767778797a828384858687"
    "88898a92939495969798999aa2a3a4a5a6a7a8a9aab2b3b4b5b6b7b8b9bac2c3"
    "c4c5c6c7c8c9cad2d3d4d5d6d7d8d9dae2e3e4e5e6e7e8e9eaf2f3f4f5f6f7f8f9fa")

SOI, DHT, SOS, EOI, TEM = 0xD8, 0xC4, 0xDA, 0xD9, 0x01
STANDALONE = {SOI, EOI, TEM, *range(0xD0, 0xD8)}  # markers without a length (RSTn among them)


class NotJpeg(ValueError):
    """A frame that is not a JPEG as a pass-through needs it."""


def _table(table_class: int, table_id: int, bits, values) -> bytes:
    if sum(bits) != len(values) or len(bits) != 16:
        raise AssertionError(f"Huffman table {table_class}/{table_id}: {sum(bits)} codes, {len(values)} values")
    return bytes([table_class << 4 | table_id, *bits]) + bytes(values)


def _dht_segment() -> bytes:
    body = (_table(0, 0, DC_LUMINANCE_BITS, DC_VALUES) + _table(1, 0, AC_LUMINANCE_BITS, AC_LUMINANCE_VALUES)
            + _table(0, 1, DC_CHROMINANCE_BITS, DC_VALUES) + _table(1, 1, AC_CHROMINANCE_BITS, AC_CHROMINANCE_VALUES))
    return b"\xff\xc4" + struct.pack(">H", len(body) + 2) + body


STANDARD_DHT = _dht_segment()


def scan_start(jpeg: bytes) -> tuple[int, bool]:
    """(offset of the first SOS marker, whether a DHT segment comes before it). NotJpeg for anything else than a
    JPEG starting with SOI whose segments up to SOS are complete."""
    if len(jpeg) < 4 or jpeg[0] != 0xFF or jpeg[1] != SOI:
        raise NotJpeg("no JPEG: it does not start with the SOI marker")
    i, dht = 2, False
    while i + 4 <= len(jpeg):
        if jpeg[i] != 0xFF:
            raise NotJpeg(f"no JPEG: no marker at byte {i}")
        marker = jpeg[i + 1]
        if marker == 0xFF:  # fill byte
            i += 1
            continue
        if marker == SOS:
            return i, dht
        if marker in STANDALONE:
            if marker == EOI:
                raise NotJpeg("no JPEG: it ends before its scan")
            i += 2
            continue
        length = struct.unpack(">H", jpeg[i + 2:i + 4])[0]
        if length < 2 or i + 2 + length > len(jpeg):
            raise NotJpeg(f"no JPEG: segment {marker:#04x} at byte {i} runs past the end")
        dht = dht or marker == DHT
        i += 2 + length
    raise NotJpeg("no JPEG: no scan (SOS)")


def with_huffman_tables(jpeg: bytes) -> bytes:
    """The frame, with the standard Huffman tables inserted before its scan if it has none (module docstring);
    unchanged if it has its own."""
    sos, dht = scan_start(jpeg)
    return jpeg if dht else jpeg[:sos] + STANDARD_DHT + jpeg[sos:]
