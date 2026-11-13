"""ROS's calibration file (CameraInfo.from_yaml / to_yaml, msg.py): a file as ROS's camera_calibration writes it, read
and checked against ruamel.yaml (an independent YAML parser, from the dev dependencies through rosbags); written and
read back exactly; anything else than the format is refused. cpp/tests/test_messages.cpp checks C++'s reader on the
same file."""

import numpy as np
import pytest
from ruamel.yaml import YAML

from xblick.msg import CameraInfo

# as camera_calibration writes it (ost.yaml), with a comment and one list over three lines as hand-edited files have
ROS_FILE = """\
image_width: 1280
image_height: 720
camera_name: 'azurekinect'
camera_matrix:
  rows: 3
  cols: 3
  data: [612.3, 0, 638.7, 0, 612.1, 366.2, 0, 0, 1]
distortion_model: rational_polynomial
distortion_coefficients:
  rows: 1
  cols: 8
  data: [0.4519, -2.6871, 0.00041,
         -0.00012, 1.5383, 0.3321,
         -2.5038, 1.4682]  # k1 k2 p1 p2 k3 k4 k5 k6
rectification_matrix:
  rows: 3
  cols: 3
  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
projection_matrix:
  rows: 3
  cols: 4
  data: [600.5, 0, 640.25, 0, 0, 601.75, 360.5, 0, 0, 0, 1, 0]
"""


def same(a: CameraInfo, b: CameraInfo) -> bool:
    return ((a.width, a.height, a.distortion_model) == (b.width, b.height, b.distortion_model)
            and all(np.array_equal(getattr(a, f), getattr(b, f)) for f in "dkrp"))


def test_ros_file():
    info = CameraInfo.from_yaml(ROS_FILE)
    doc = YAML(typ="safe").load(ROS_FILE)
    assert (info.width, info.height) == (doc["image_width"], doc["image_height"]) == (1280, 720)
    assert info.distortion_model == doc["distortion_model"] == "rational_polynomial"
    np.testing.assert_array_equal(info.k, doc["camera_matrix"]["data"])
    np.testing.assert_array_equal(info.d, doc["distortion_coefficients"]["data"])
    np.testing.assert_array_equal(info.r, doc["rectification_matrix"]["data"])
    np.testing.assert_array_equal(info.p, doc["projection_matrix"]["data"])
    assert info.header.frame_id == "" and info.binning_x == 0


def test_written_and_read_back_exactly(tmp_path):
    info = CameraInfo.from_yaml(ROS_FILE)
    info.k[0] = 1 / 3  # a value without a short decimal form
    info.d[4] = -1e-17
    path = tmp_path / "azurekinect.yaml"
    info.write_file(path, "azurekinect")
    back = CameraInfo.read_file(path)
    assert same(back, info)
    assert YAML(typ="safe").load(path.read_text())["camera_name"] == "azurekinect"
    plumb = CameraInfo.pinhole(80, 60, 70.0, 71.0, 39.5, 29.5)
    assert same(CameraInfo.from_yaml(plumb.to_yaml("sim")), plumb)


@pytest.mark.parametrize("change, message", [
    (("image_height: 720\n", ""), "'image_height' missing"),
    (("  cols: 4\n", "  cols: 3\n"), "projection_matrix: 3 x 3"),
    (("  cols: 8\n", "  cols: 7\n"), "distortion_coefficients: 1 x 7 with 8 values"),
    (("camera_name", "camera_names"), "unknown key 'camera_names'"),
    (("1.4682]", "1.4682] 2"), "text after ']'"),
    (("1.4682]", "1.4682"), "list without ']'"),
    (("612.3", "6x12"), "'6x12' is not a number"),
    (("612.3", "6_12"), "'6_12' is not a number"),
    (("  rows: 1\n", "  rows: 1\n  rows: 1\n"), "'rows' unknown or twice"),
])
def test_anything_else_is_refused(change, message):
    with pytest.raises(ValueError, match=message):
        CameraInfo.from_yaml(ROS_FILE.replace(*change))


def test_errors_name_the_file(tmp_path):
    path = tmp_path / "broken.yaml"
    path.write_text(ROS_FILE.replace("image_width: 1280\n", ""))
    with pytest.raises(ValueError, match="broken.yaml: calibration file, line .*'image_width' missing"):
        CameraInfo.read_file(path)
