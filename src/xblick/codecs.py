"""sensor_msgs/CompressedImage as ROS image_transport's ``compressed`` plugin writes it.

The ``format`` field is ``"<original encoding>; <jpeg|png> compressed <compressed encoding>"``, e.g.
``"rgb8; jpeg compressed bgr8"``: the picture was rgb8, the file holds it as bgr8 (the channel order OpenCV's
imencode takes; a JPEG/PNG file itself stores colour the same way either way). Decoding gives back the original
encoding where the codec keeps the information (colour order, 16-bit grey); alpha is lost in JPEG.

16-bit grey and depth images also take ``rvl`` (Wilson's RVL, lossless; cpp/include/xblick/rvl.hpp): on Kinect
depth it encodes about nine times as fast as PNG and is a fifth smaller. Its message is the one ROS's
``compressedDepth`` plugin writes for a 16UC1 image with its format ``rvl``: the format field is
``"<encoding>; compressedDepth rvl"``. Of that plugin nothing else is read (its PNG has a header in front, its 32FC1
depth is quantised).
"""

from __future__ import annotations

import io

import numpy as np
from PIL import Image as PILImage

from . import _core
from .encodings import EncodingError, check_array
from .msg import CompressedImage, Header

JPEG = "jpeg"
PNG = "png"
RVL = "rvl"

_GREY8 = ("mono8", "8UC1")
_GREY16 = ("mono16", "16UC1")
CODEC_ENCODINGS = {JPEG: ("rgb8", "rgba8", "bgr8", "bgra8") + _GREY8,
                   PNG: ("rgb8", "bgr8", "rgba8", "bgra8") + _GREY8 + _GREY16,
                   RVL: _GREY16}


def check_codec(encoding: str, codec: str):
    """Raise if ``codec`` cannot compress images of ``encoding`` (a publisher checks this when it is made, not when
    the first compressed subscriber comes)."""
    if codec not in CODEC_ENCODINGS:
        raise EncodingError(f"unknown codec {codec!r} (jpeg, png, rvl)")
    if encoding not in CODEC_ENCODINGS[codec]:
        hint = "rvl or png" if encoding in _GREY16 else "codec=None: no compressed transport"
        raise EncodingError(f"{codec} cannot compress {encoding} images ({', '.join(CODEC_ENCODINGS[codec])}); "
                            f"use {hint}")


def compress(array: np.ndarray, encoding: str, codec: str = JPEG, quality: int = 85,
             header: Header | None = None) -> CompressedImage:
    check_array(array, encoding)
    if codec == JPEG:
        if encoding in ("rgb8", "rgba8"):
            pil, target = PILImage.fromarray(np.ascontiguousarray(array[..., :3])), "bgr8"
        elif encoding in ("bgr8", "bgra8"):
            pil, target = PILImage.fromarray(np.ascontiguousarray(array[..., 2::-1])), "bgr8"
        elif encoding in _GREY8:
            pil, target = PILImage.fromarray(array), "mono8"
        else:
            raise EncodingError(f"JPEG takes 8-bit colour or grey images, not {encoding} (depth: use png)")
        buffer = io.BytesIO()
        pil.save(buffer, format="JPEG", quality=int(quality))
    elif codec == PNG:
        if encoding in ("rgb8", "bgr8"):
            rgb = array if encoding == "rgb8" else array[..., ::-1]
            pil, target = PILImage.fromarray(np.ascontiguousarray(rgb)), "bgr8"
        elif encoding in ("rgba8", "bgra8"):
            rgba = array if encoding == "rgba8" else array[..., [2, 1, 0, 3]]
            pil, target = PILImage.fromarray(np.ascontiguousarray(rgba)), "bgra8"
        elif encoding in _GREY8:
            pil, target = PILImage.fromarray(array), "mono8"
        elif encoding in _GREY16:
            pil, target = PILImage.fromarray(np.ascontiguousarray(array, dtype=np.uint16)), encoding
        else:
            raise EncodingError(f"PNG takes 8-bit colour, 8- or 16-bit grey images, not {encoding}")
        buffer = io.BytesIO()
        pil.save(buffer, format="PNG", compress_level=1)  # fast; PNG is lossless at every level
    elif codec == RVL:
        if encoding not in _GREY16:
            raise EncodingError(f"RVL takes 16-bit grey and depth images, not {encoding}")
        return CompressedImage(header=header or Header(), format=f"{encoding}; compressedDepth rvl",
                               data=_core.rvl_encode(np.ascontiguousarray(array, dtype=np.uint16)))
    else:
        raise EncodingError(f"unknown codec {codec!r} (jpeg, png, rvl)")
    return CompressedImage(header=header or Header(), format=f"{encoding}; {codec} compressed {target}",
                           data=buffer.getvalue())


def parse_format(fmt: str) -> tuple[str | None, str]:
    """(original encoding or None, codec) of a CompressedImage.format. Accepts the bare ``"jpeg"``/``"png"`` older
    publishers write."""
    text = fmt.strip().lower()
    if ";" in fmt:
        original, rest = (part.strip() for part in fmt.split(";", 1))
        codec = rest.split()[0].lower() if rest else ""
        if codec == "compresseddepth":  # ROS's compressedDepth plugin
            if rest.lower().split() != ["compresseddepth", RVL] or original not in _GREY16:
                raise EncodingError(f"compressed format {fmt!r}: of ROS's compressedDepth only rvl of a 16-bit image "
                                    "is read")
            return original, RVL
        if codec not in (JPEG, PNG):
            raise EncodingError(f"compressed format {fmt!r}: unknown codec {codec!r}")
        return original, codec
    if text in (JPEG, "jpg"):
        return None, JPEG
    if text == PNG:
        return None, PNG
    raise EncodingError(f"compressed format {fmt!r} is not one of image_transport's")


def decompress(message: CompressedImage) -> tuple[np.ndarray, str]:
    """(pixels, encoding). The encoding is the original one where the file allows it: rgb8/bgr8 for colour JPEG,
    rgba8/bgra8 for colour PNG with alpha, mono8 or the 16-bit grey encoding otherwise."""
    original, codec = parse_format(message.format)
    if codec == RVL:
        data = np.frombuffer(memoryview(message.data), np.uint8)
        try:
            array = np.empty(_core.rvl_shape(data), np.uint16)
            _core.rvl_decode(data, array)
        except _core.RvlError as error:
            raise EncodingError(str(error)) from None
        return array, original
    pil = PILImage.open(io.BytesIO(memoryview(message.data)))
    pil.load()
    if pil.mode in ("I;16", "I;16B", "I;16L", "I"):
        array = np.asarray(pil).astype(np.uint16)
        return array, original if original in _GREY16 else "mono16"
    if pil.mode == "L":
        return np.asarray(pil), original if original in _GREY8 else "mono8"
    if pil.mode == "RGBA":
        rgba = np.asarray(pil)
        if original == "bgra8":
            return np.ascontiguousarray(rgba[..., [2, 1, 0, 3]]), "bgra8"
        return rgba, "rgba8"
    rgb = np.asarray(pil.convert("RGB"))
    if original in ("bgr8", "bgra8"):
        return np.ascontiguousarray(rgb[..., ::-1]), "bgr8"
    return rgb, "rgb8"
