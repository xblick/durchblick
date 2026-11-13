"""blick.rectify (C++ rectify.hpp through _core) against OpenCV's initUndistortRectifyMap: every distortion
model, a stereo pair's R and P, P all zeros; and the rectified CameraInfo."""

import numpy as np
import pytest

from blick.msg import CameraInfo
from blick.rectify import rectified_info, rectify_map

cv2 = pytest.importorskip("cv2")

W, H = 320, 200


def camera(model: str, d, r=None, p=None) -> CameraInfo:
    info = CameraInfo.pinhole(W, H, 183.0, 182.5, 153.7, 108.2, frame_id="Stereo_Left")
    info.distortion_model = model
    info.d = np.asarray(d, np.float64)
    if r is not None:
        info.r = np.asarray(r, np.float64).reshape(9)
    if p is not None:
        info.p = np.asarray(p, np.float64).reshape(12)
    return info


R_LEFT = cv2.Rodrigues(np.array([0.012, -0.008, 0.017]))[0]
P_RIGHT = np.array([[175.3, 0.0, 130.1, -175.3 * 0.176], [0.0, 175.3, 115.2, 0.0], [0.0, 0.0, 1.0, 0.0]])

CASES = {
    "none": camera("plumb_bob", [0, 0, 0, 0, 0]),
    "plumb_bob": camera("plumb_bob", [-0.28, 0.09, 0.001, -0.0005, -0.012]),
    "rational, stereo left": camera("rational_polynomial",
                                    [1.7147, 0.1705, 0.0002, 0.0002, -0.0261, 2.0905, 0.6933, -0.0524], R_LEFT,
                                    np.hstack([P_RIGHT[:, :3], np.zeros((3, 1))])),
    "rational, stereo right": camera("rational_polynomial",
                                     [3.1189, 0.2451, 0.0001, -0.0001, -0.1075, 3.4965, 1.2856, -0.2254], R_LEFT.T,
                                     P_RIGHT),
    "equidistant": camera("equidistant", [-0.013, 0.021, -0.017, 0.004]),
}


def opencv_map(info: CameraInfo):
    k = info.k.reshape(3, 3)
    r = info.r.reshape(3, 3)
    p = info.p.reshape(3, 4)
    if info.distortion_model == "equidistant":
        return cv2.fisheye.initUndistortRectifyMap(k, info.d, r, p, (W, H), cv2.CV_32FC1)
    return cv2.initUndistortRectifyMap(k, info.d, r, p, (W, H), cv2.CV_32FC1)


@pytest.mark.parametrize("name", list(CASES))
def test_the_map_is_opencvs(name):
    info = CASES[name]
    x, y = rectify_map(info)
    ox, oy = opencv_map(info)
    inside = (ox >= 0) & (oy >= 0) & (ox <= W - 1) & (oy <= H - 1)  # where a rectified pixel takes a value
    assert inside.mean() > 0.5, inside.mean()
    assert np.abs(x - ox)[inside].max() < 2e-3 and np.abs(y - oy)[inside].max() < 2e-3, \
        (np.abs(x - ox)[inside].max(), np.abs(y - oy)[inside].max())


def test_a_camera_info_without_p_or_r_takes_k_and_no_rotation():
    info = CASES["plumb_bob"]
    bare = camera("plumb_bob", info.d)
    bare.p = np.zeros(12)
    bare.r = np.zeros(9)
    x, y = rectify_map(bare)
    ox, oy = rectify_map(info)  # P = [K | 0], R = I
    np.testing.assert_array_equal(x, ox)
    np.testing.assert_array_equal(y, oy)


def test_the_rectified_camera_info():
    info = CASES["rational, stereo right"]
    info.header.stamp.sec = 7
    out = rectified_info(info)
    np.testing.assert_allclose(out.k.reshape(3, 3), P_RIGHT[:, :3])
    np.testing.assert_allclose(out.p.reshape(3, 4), P_RIGHT)  # keeps P[3] = -f b
    assert out.distortion_model == "plumb_bob" and not out.d.any()
    np.testing.assert_allclose(out.r.reshape(3, 3), np.eye(3))
    assert out.header.frame_id == "Stereo_Left" and out.header.stamp.sec == 7
    assert (out.width, out.height) == (W, H)


def test_what_cannot_be_rectified_is_refused():
    with pytest.raises(ValueError, match="rotation"):
        rectify_map(camera("plumb_bob", [0, 0, 0, 0, 0], r=np.diag([1.0, 2.0, 1.0])))
    with pytest.raises(ValueError, match="distortion model"):
        rectify_map(camera("kannala_brandt", [0.1, 0, 0, 0]))
