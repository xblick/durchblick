"""Image encodings of ROS (sensor_msgs/image_encodings.hpp) and the conversion between sensor_msgs/Image and NumPy
arrays, as cv_bridge does it.

Conventions on top of ROS that Durchblick relies on (documented in the README):
* depth in ``16UC1`` is millimetres, 0 = no measurement; depth in ``32FC1`` is metres, NaN = no measurement
  (the conventions of ROS depth_image_proc);
* instance masks are ``32SC1`` with 0 = background.
"""

from __future__ import annotations

import re
import sys
from dataclasses import dataclass

import numpy as np

from .msg import Header, Image


class EncodingError(ValueError):
    """An encoding that is unknown, or an array that does not fit its encoding."""


@dataclass(frozen=True)
class EncodingInfo:
    channels: int
    dtype: np.dtype


_NAMED = {
    "rgb8": (3, np.uint8), "bgr8": (3, np.uint8), "rgba8": (4, np.uint8), "bgra8": (4, np.uint8),
    "rgb16": (3, np.uint16), "bgr16": (3, np.uint16), "rgba16": (4, np.uint16), "bgra16": (4, np.uint16),
    "mono8": (1, np.uint8), "mono16": (1, np.uint16),
    "bayer_rggb8": (1, np.uint8), "bayer_bggr8": (1, np.uint8), "bayer_gbrg8": (1, np.uint8),
    "bayer_grbg8": (1, np.uint8), "bayer_rggb16": (1, np.uint16), "bayer_bggr16": (1, np.uint16),
    "bayer_gbrg16": (1, np.uint16), "bayer_grbg16": (1, np.uint16),
}
_GENERIC = re.compile(r"^(8|16|32|64)(U|S|F)C(\d+)$")
_DTYPES = {("8", "U"): np.uint8, ("8", "S"): np.int8, ("16", "U"): np.uint16, ("16", "S"): np.int16,
           ("16", "F"): np.float16, ("32", "S"): np.int32, ("32", "F"): np.float32, ("64", "F"): np.float64}


def info(encoding: str) -> EncodingInfo:
    if encoding in _NAMED:
        channels, dtype = _NAMED[encoding]
        return EncodingInfo(channels, np.dtype(dtype))
    m = _GENERIC.match(encoding)
    if m and (m.group(1), m.group(2)) in _DTYPES:
        # 16FC is not in ROS's list (OpenCV has CV_16F); allowed for half-precision model outputs
        return EncodingInfo(int(m.group(3)), np.dtype(_DTYPES[(m.group(1), m.group(2))]))
    raise EncodingError(f"unknown image encoding {encoding!r}")


def check_array(array: np.ndarray, encoding: str) -> EncodingInfo:
    """Raise if ``array`` (H x W or H x W x C) does not have the dtype and channel count of ``encoding``."""
    enc = info(encoding)
    if array.dtype != enc.dtype:
        raise EncodingError(f"{encoding} needs {enc.dtype}, the array is {array.dtype}")
    channels = 1 if array.ndim == 2 else (array.shape[2] if array.ndim == 3 else None)
    if channels != enc.channels:
        raise EncodingError(f"{encoding} has {enc.channels} channel(s), the array has shape {array.shape}")
    return enc


def numpy_to_image(array: np.ndarray, encoding: str, header: Header | None = None) -> Image:
    """A sensor_msgs/Image of ``array`` (rows without padding, little endian). Copies only if the array is not
    C-contiguous."""
    check_array(array, encoding)
    array = np.ascontiguousarray(array)
    if array.dtype.byteorder == ">" or (array.dtype.byteorder == "=" and sys.byteorder == "big"):
        array = array.astype(array.dtype.newbyteorder("<"))
    height, width = array.shape[:2]
    return Image(header=header or Header(), height=height, width=width, encoding=encoding, is_bigendian=0,
                 step=width * array.itemsize * (1 if array.ndim == 2 else array.shape[2]),
                 data=memoryview(array).cast("B"))


def image_to_numpy(image: Image) -> np.ndarray:
    """The pixels of ``image`` as an array (H x W for one channel, H x W x C otherwise). A view into
    ``image.data`` where possible; read-only when the data is."""
    enc = info(image.encoding)
    dtype = enc.dtype.newbyteorder(">" if image.is_bigendian else "<")
    row_bytes = image.width * enc.channels * dtype.itemsize
    if image.step < row_bytes:
        raise EncodingError(f"step {image.step} is smaller than a row of {image.width} {image.encoding} pixels "
                            f"({row_bytes} bytes)")
    buffer = memoryview(image.data).cast("B")
    if buffer.nbytes < image.step * image.height:
        raise EncodingError(f"{buffer.nbytes} bytes of data for {image.height} rows of {image.step} bytes")
    rows = np.frombuffer(buffer, dtype=np.uint8, count=image.step * image.height).reshape(image.height, image.step)
    pixels = rows[:, :row_bytes].view(dtype)
    shape = (image.height, image.width) if enc.channels == 1 else (image.height, image.width, enc.channels)
    pixels = pixels.reshape(shape)
    if dtype.byteorder == ">" or (dtype.byteorder == "<" and sys.byteorder == "big"):
        pixels = pixels.astype(dtype.newbyteorder("="))
    return pixels
