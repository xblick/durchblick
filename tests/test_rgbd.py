"""blick.rgbd: the camera spec, the camera models, the stamp pairing; the undistortion where torch and OpenCV are
installed (not by Durchblick itself: run them in a node's environment, e.g. durchblick-foundationpose's)."""

import threading

import numpy as np
import pytest

import blick as db
from blick.history import StampedHistory
from blick.rgbd import CameraSpec, check_same_camera, scaled_k


def test_camera_spec_defaults_and_options():
    head = CameraSpec.parse("robot/camera", segmenter="sam3")
    assert head.keys == {"color": "robot/camera/color", "depth": "robot/camera/depth",
                         "instances": "robot/camera/sam3/instances",
                         "detections": "robot/camera/sam3/detections_2d"}
    rc = CameraSpec.parse("robot/rc_visard:color=left:depth=ffs/depth")
    assert rc.keys == {"color": "robot/rc_visard/left", "depth": "robot/rc_visard/ffs/depth"}
    assert str(rc) == "robot/rc_visard:color=left:depth=ffs/depth"
    assert CameraSpec.parse(str(head)) == head  # the segmenter is written out, so a default does not get lost
    assert CameraSpec.parse("c:segmenter=sam3_rc", segmenter="sam3").segmenter == "sam3_rc"
    for bad in ("", "robot/camera:colour=x", "robot/camera:depth", "robot/camera:depth=a:depth=b"):
        with pytest.raises(ValueError):
            CameraSpec.parse(bad)


def test_scaled_k_keeps_the_centre_between_pixel_centres():
    info = db.CameraInfo.pinhole(1280, 960, 1000.0, 1000.0, 639.5, 479.5, frame_id="left")
    np.testing.assert_allclose(scaled_k(info, 480, 640), [[500, 0, 319.5], [0, 500, 239.5], [0, 0, 1]])


def test_depth_and_colour_must_be_one_camera_up_to_scale():
    color = db.CameraInfo.pinhole(1280, 960, 1000.0, 1000.0, 639.5, 479.5, frame_id="left")
    check_same_camera(color, db.CameraInfo.pinhole(640, 480, 500.0, 500.0, 319.5, 239.5, frame_id="left"))
    with pytest.raises(ValueError, match="one camera"):
        check_same_camera(color, db.CameraInfo.pinhole(640, 480, 520.0, 520.0, 319.5, 239.5, frame_id="left"))


def test_stamped_history_keeps_the_newest_and_wakes():
    wake = threading.Event()
    h = StampedHistory(2, wake.set)
    for stamp in (1, 2, 3):
        h.put(stamp, f"frame {stamp}")
    assert wake.is_set() and h.stamps() == {2, 3} and h.at(3) == "frame 3" and h.at(1) is None


def test_a_pinhole_camera_is_not_remapped():
    torch = pytest.importorskip("torch")
    pytest.importorskip("cv2")
    from blick.rgbd import Undistorter

    info = db.CameraInfo.pinhole(640, 480, 500.0, 500.0, 319.5, 239.5, frame_id="c")
    u = Undistorter(info, 480, 640, "cpu")
    assert u.identity
    image = torch.arange(12).reshape(3, 4)
    assert u.apply(image, "nearest") is image


def test_a_distorted_point_lands_where_the_pinhole_camera_sees_it():
    torch = pytest.importorskip("torch")
    cv2 = pytest.importorskip("cv2")
    from blick.rgbd import Undistorter

    w, h = 640, 480
    k = np.array([[500.0, 0, 319.5], [0, 500.0, 239.5], [0, 0, 1]])
    d = np.array([-0.3, 0.1, 0.001, -0.002, 0.0])
    info = db.CameraInfo.pinhole(w, h, 500.0, 500.0, 319.5, 239.5, frame_id="c")
    info.distortion_model, info.d = "plumb_bob", d
    point = np.array([[0.25, -0.15, 1.0]])
    distorted, _ = cv2.projectPoints(point, np.zeros(3), np.zeros(3), k, d)
    image = np.zeros((h, w), dtype=np.float32)
    u0, v0 = np.round(distorted[0, 0]).astype(int)
    image[v0 - 1:v0 + 2, u0 - 1:u0 + 2] = 1.0  # a 3 x 3 dot where the lens images the point
    out = Undistorter(info, h, w, "cpu").apply(torch.as_tensor(image), "bilinear").numpy()
    vs, us = np.nonzero(out > 0.5)
    expected = (k @ point[0])[:2] / point[0, 2]  # where the pinhole camera of P (= K) sees it
    assert abs(us.mean() - expected[0]) < 1.0 and abs(vs.mean() - expected[1]) < 1.0


def test_views_apart_seen_from_the_object():
    from blick.rgbd import far_enough, view_direction

    cam = np.eye(4)
    cam[:3, 3] = [0.0, 0.0, -1.0]  # 1 m behind the object (at the origin), looking along +z
    d0 = view_direction(cam, np.array([0.0, 0.0, 1.0]))
    np.testing.assert_allclose(d0, [0.0, 0.0, -1.0])
    turned = np.eye(4)
    a = np.radians(20)
    turned[:3, :3] = [[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]]
    turned[:3, 3] = turned[:3, :3] @ np.array([0.0, 0.0, -1.0])  # the camera 20 deg around the object
    d1 = view_direction(turned, np.array([0.0, 0.0, 1.0]))
    assert far_enough(d1, [d0], 15.0) and not far_enough(d1, [d0], 25.0)


def test_segmenter_mask_by_instance_or_largest_label():
    torch = pytest.importorskip("torch")
    from blick.rgbd import RGBDFrame, segmenter_mask

    ids = torch.zeros(4, 6, dtype=torch.int32)
    ids[0, :2], ids[2:, 2:] = 3, 5  # instance 3: 2 px, instance 5: 8 px, both "mug"
    header = db.Header(db.Time.from_ns(1), "c")
    box = db.BoundingBox2D(db.Pose2D(db.Point2D(0.0, 0.0), 0.0), 1.0, 1.0)
    dets = db.Detection2DArray(header, [db.Detection2D(header, [db.ObjectHypothesisWithPose(db.ObjectHypothesis("mug", 0.9))],
                                                       box, i) for i in ("3", "5")])
    frame = RGBDFrame(None, torch.ones(4, 6), ids, np.eye(3), "c", dets)
    mask, used = segmenter_mask(frame, label="mug")
    assert used == "5" and int(mask.sum()) == 8
    assert segmenter_mask(frame, instance="3")[1] == "3"
    with pytest.raises(ValueError, match="labelled"):
        segmenter_mask(frame, label="cup")
    with pytest.raises(ValueError, match="no segmenter"):
        segmenter_mask(RGBDFrame(None, None, None, np.eye(3), "c", None), label="mug")


def test_expand_path_expands_variables_and_refuses_unset_ones(monkeypatch):
    from blick.paths import expand_path

    monkeypatch.setenv("DURCHBLICK_TEST_DIR", "/data/objects")
    monkeypatch.delenv("DURCHBLICK_TEST_UNSET", raising=False)
    assert str(expand_path("${DURCHBLICK_TEST_DIR}/NeMO")) == "/data/objects/NeMO"
    assert str(expand_path("$DURCHBLICK_TEST_DIR")) == "/data/objects"
    assert not str(expand_path("~/x")).startswith("~")
    with pytest.raises(ValueError, match="DURCHBLICK_TEST_UNSET"):
        expand_path("${DURCHBLICK_TEST_UNSET}/x")


def test_distorter_takes_labels_back_into_the_distorted_image():
    torch = pytest.importorskip("torch")
    cv2 = pytest.importorskip("cv2")
    from blick.rgbd import Distorter, Undistorter

    w, h = 640, 480
    k = np.array([[500.0, 0, 319.5], [0, 500.0, 239.5], [0, 0, 1]])
    d = np.array([-0.3, 0.1, 0.001, -0.002, 0.0])
    info = db.CameraInfo.pinhole(w, h, 500.0, 500.0, 319.5, 239.5, frame_id="c")
    info.distortion_model, info.d = "plumb_bob", d
    u = Undistorter(info, 240, 320, "cpu")  # the frames at half the colour's size, as a depth's
    point = np.array([[0.25, -0.15, 1.0]])
    labels = torch.zeros(240, 320, dtype=torch.int32)
    x, y = np.round((u.K @ point[0])[:2] / point[0, 2]).astype(int)  # where the pinhole frame sees the point
    labels[y - 2:y + 3, x - 2:x + 3] = 7
    back = Distorter(info, u.K, 240, 320, "cpu").apply(labels)
    assert back.shape == (480, 640)
    distorted, _ = cv2.projectPoints(point, np.zeros(3), np.zeros(3), k, d)
    vs, us = np.nonzero(back.numpy() == 7)
    assert abs(us.mean() - distorted[0, 0, 0]) < 2.0 and abs(vs.mean() - distorted[0, 0, 1]) < 2.0
