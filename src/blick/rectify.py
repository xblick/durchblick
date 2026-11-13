"""A camera's image rectified, as ROS's image_proc and OpenCV's initUndistortRectifyMap compute it: without the lens
distortion, turned by the CameraInfo's R into the camera of its P (C++: ``durchblick/rectify.hpp``, the same code).

``rectify_map(info)`` gives every rectified pixel the point of the camera's image that it shows, as ``cv2.remap`` and
``rgbd.Undistorter`` take it. ``rectified_info(info)`` is the CameraInfo of the rectified image: K = P's left 3 x 3, no
distortion, R = I, P as given (the right camera of a stereo pair keeps P[3] = -f b). durchblick-rectify publishes
rectified streams with these."""

from __future__ import annotations

import numpy as np

from . import _core
from .msg import CameraInfo


def rectify_map(info: CameraInfo) -> tuple[np.ndarray, np.ndarray]:
    """(x, y), float32 (height, width): where in the camera's image each rectified pixel lies (pixel centres at
    integers; -1 behind the camera). Distortion plumb_bob, rational_polynomial, equidistant or none; R a rotation (all
    zeros: none); P without skew (all zeros: K). Raises ValueError for anything else."""
    x = np.empty((info.height, info.width), np.float32)
    y = np.empty_like(x)
    r = np.asarray(info.r, np.float64)
    if not r.any():
        r = np.eye(3).reshape(9)
    _core.rectify_map(info.width, info.height, np.ascontiguousarray(info.k, np.float64), info.distortion_model,
                      np.ascontiguousarray(info.d, np.float64), np.ascontiguousarray(r),
                      np.ascontiguousarray(info.p, np.float64), x, y)
    return x, y


def rectified_info(info: CameraInfo) -> CameraInfo:
    """The CameraInfo of the rectified image, with the same header (stamp, frame)."""
    p = np.asarray(info.p, np.float64)
    k = np.asarray(info.k, np.float64)
    if p.any():
        out = CameraInfo.pinhole(info.width, info.height, p[0], p[5], p[2], p[6])
        out.p = p.copy()
    else:
        out = CameraInfo.pinhole(info.width, info.height, k[0], k[4], k[2], k[5])
    out.header = info.header
    return out
