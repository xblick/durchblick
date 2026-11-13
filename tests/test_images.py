"""Encodings (NumPy <-> sensor_msgs/Image) and codecs (sensor_msgs/CompressedImage)."""

import numpy as np
import pytest

from blick.codecs import compress, decompress, parse_format
from blick.encodings import EncodingError, image_to_numpy, info, numpy_to_image
from blick.msg import CompressedImage, Image

RNG = np.random.default_rng(0)


@pytest.mark.parametrize("encoding,shape,dtype", [
    ("rgb8", (4, 5, 3), np.uint8), ("bgra8", (4, 5, 4), np.uint8), ("mono8", (4, 5), np.uint8),
    ("16UC1", (4, 5), np.uint16), ("32FC1", (4, 5), np.float32), ("32SC1", (4, 5), np.int32),
    ("32FC3", (4, 5, 3), np.float32), ("mono16", (4, 5), np.uint16),
])
def test_numpy_round_trip_through_cdr(encoding, shape, dtype):
    array = (RNG.random(shape) * 1000).astype(dtype)
    message = Image.deserialize(numpy_to_image(array, encoding).serialize())
    back = image_to_numpy(message)
    assert back.dtype == array.dtype and back.shape == array.shape and np.array_equal(back, array)


def test_padded_rows_and_big_endian():
    pixels = np.arange(12, dtype=">u2").reshape(3, 4)
    padded = np.zeros((3, 10), np.uint8)
    padded[:, :8] = pixels.view(np.uint8).reshape(3, 8)
    image = Image(height=3, width=4, encoding="16UC1", is_bigendian=1, step=10, data=padded.tobytes())
    assert np.array_equal(image_to_numpy(image), pixels.astype(np.uint16))


def test_mismatches_are_errors():
    with pytest.raises(EncodingError):
        info("rgb9")
    with pytest.raises(EncodingError):
        numpy_to_image(np.zeros((2, 2, 3), np.float32), "rgb8")
    with pytest.raises(EncodingError):
        numpy_to_image(np.zeros((2, 2), np.uint8), "rgb8")
    with pytest.raises(EncodingError):
        image_to_numpy(Image(height=2, width=2, encoding="rgb8", step=6, data=b"\x00" * 6))


def test_non_contiguous_input():
    array = (RNG.random((6, 8, 3)) * 255).astype(np.uint8)[::2, ::2]
    assert np.array_equal(image_to_numpy(numpy_to_image(array, "rgb8")), array)


def smooth_rgb(h=48, w=64):
    y, x = np.mgrid[0:h, 0:w]
    return np.stack([x * 255 // w, y * 255 // h, (x + y) * 255 // (w + h)], -1).astype(np.uint8)


def test_jpeg_colour_keeps_channel_order():
    rgb = smooth_rgb()
    message = compress(rgb, "rgb8", "jpeg", quality=95)
    assert message.format == "rgb8; jpeg compressed bgr8"
    back, encoding = decompress(message)
    assert encoding == "rgb8" and back.shape == rgb.shape
    assert np.abs(back.astype(int) - rgb).mean() < 3
    bgr = np.ascontiguousarray(rgb[..., ::-1])
    back, encoding = decompress(compress(bgr, "bgr8", "jpeg", quality=95))
    assert encoding == "bgr8" and np.abs(back.astype(int) - bgr).mean() < 3


@pytest.mark.parametrize("encoding", ["rgb8", "bgr8", "rgba8", "bgra8", "mono8", "16UC1", "mono16"])
def test_png_is_lossless(encoding):
    channels, dtype = info(encoding).channels, info(encoding).dtype
    shape = (7, 9) if channels == 1 else (7, 9, channels)
    array = (RNG.random(shape) * (65535 if dtype == np.uint16 else 255)).astype(dtype)
    back, back_encoding = decompress(compress(array, encoding, "png"))
    assert back_encoding == encoding and back.dtype == dtype and np.array_equal(back, array)


@pytest.mark.parametrize("encoding", ["16UC1", "mono16"])
def test_rvl_is_lossless_and_ros_compressed_depth(encoding):
    """RVL keeps every 16-bit value, with holes, noise and the largest jumps; its message is the one ROS's
    compressedDepth plugin writes for the format rvl."""
    array = (RNG.random((37, 53)) * 65535).astype(np.uint16)
    array[RNG.random(array.shape) < 0.3] = 0
    array[5:9] = 0
    array[20, ::2], array[20, 1::2] = 65535, 1
    message = compress(array, encoding, "rvl")
    assert message.format == f"{encoding}; compressedDepth rvl"
    back, back_encoding = decompress(message)
    assert back_encoding == encoding and back.dtype == np.uint16 and np.array_equal(back, array)
    for flat in (np.zeros((4, 6), np.uint16), np.full((4, 6), 1234, np.uint16), np.ascontiguousarray(array[:, ::2])):
        assert np.array_equal(decompress(compress(flat, encoding, "rvl"))[0], flat)
    # 0 0 5 5 0: the groups 2 (zeros), 2 (others), A 1 (+5), 0 (+0), 1 (zero), 0 (others) in one word, after ROS's
    # ConfigHeader (12 zero bytes), the width and the height
    five = compress(np.array([[0, 0, 5, 5, 0]], np.uint16), "16UC1", "rvl")
    assert bytes(five.data) == bytes(12) + (5).to_bytes(4, "little") + (1).to_bytes(4, "little") + bytes.fromhex("0001a122")


def test_rvl_refuses_what_it_cannot_hold_or_read():
    array = (RNG.random((20, 30)) * 5000).astype(np.uint16)
    message = compress(array, "16UC1", "rvl")
    with pytest.raises(EncodingError, match="16-bit"):
        compress(np.zeros((4, 4, 3), np.uint8), "rgb8", "rvl")
    for data, match in ((bytes(message.data)[:len(message.data) // 2], "end before"),
                        (bytes(message.data)[:10], "fewer than the header"),
                        (bytes(message.data)[:16] + (21).to_bytes(4, "little") + bytes(message.data)[20:], "RVL"),
                        (bytes(12) + b"\xff" * 8 + bytes(message.data)[20:], "no image")):
        with pytest.raises(EncodingError, match=match):
            decompress(CompressedImage(format=message.format, data=data))
    for fmt in ("16UC1; compressedDepth png", "16UC1; compressedDepth", "rgb8; compressedDepth rvl",
                "32FC1; compressedDepth rvl"):
        with pytest.raises(EncodingError, match="only rvl of a 16-bit image"):
            decompress(CompressedImage(format=fmt, data=bytes(message.data)))


def test_jpeg_refuses_depth():
    with pytest.raises(EncodingError):
        compress(np.zeros((4, 4), np.uint16), "16UC1", "jpeg")


def test_format_strings():
    assert parse_format("rgb8; jpeg compressed bgr8") == ("rgb8", "jpeg")
    assert parse_format("jpeg") == (None, "jpeg")
    assert parse_format("16UC1; png compressed 16UC1") == ("16UC1", "png")
    with pytest.raises(EncodingError):
        parse_format("16UC1; compressedDepth png")
