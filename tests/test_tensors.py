"""Frames as torch tensors: colour encodings to RGB, depth to metres (on the CPU; the nodes' devices take the same
path)."""

import numpy as np
import pytest

import blick as db
from blick import tensors

torch = pytest.importorskip("torch")


def frame(image, encoding):
    return db.camera.Frame(image=image, encoding=encoding, header=db.Header(), key="k", receive_ns=0)


def test_colour_frames_become_rgb():
    rgb = np.arange(2 * 3 * 3, dtype=np.uint8).reshape(2, 3, 3)
    alpha = np.full((2, 3, 1), 255, np.uint8)
    for encoding, image in [("rgb8", rgb), ("bgr8", rgb[..., ::-1]), ("rgba8", np.concatenate([rgb, alpha], 2)),
                            ("bgra8", np.concatenate([rgb[..., ::-1], alpha], 2))]:
        np.testing.assert_array_equal(tensors.rgb(frame(np.ascontiguousarray(image), encoding), "cpu").numpy(), rgb,
                                      err_msg=encoding)
    grey = rgb[..., 0].copy()
    np.testing.assert_array_equal(tensors.rgb(frame(grey, "mono8"), "cpu").numpy(), np.repeat(grey[..., None], 3, 2))
    with pytest.raises(ValueError, match="16UC1"):
        tensors.rgb(frame(np.zeros((2, 3), np.uint16), "16UC1"), "cpu")


def test_depth_in_metres():
    np.testing.assert_allclose(tensors.depth_metres(frame(np.array([[0, 1500]], np.uint16), "16UC1"), "cpu"),
                               [[0.0, 1.5]])
    np.testing.assert_allclose(tensors.depth_metres(frame(np.array([[np.nan, 2.5]], np.float32), "32FC1"), "cpu"),
                               [[0.0, 2.5]])
    values, per_metre = tensors.depth(frame(np.array([[7]], np.uint16), "16UC1"), "cpu")
    assert values.dtype == torch.uint16 and per_metre == 1000.0  # as it came: the node scales when it needs to
    with pytest.raises(ValueError, match="mono16"):  # instance ids (Durchblick's segmenters), not millimetres
        tensors.depth(frame(np.zeros((1, 1), np.uint16), "mono16"), "cpu")
