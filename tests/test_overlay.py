"""xblick.overlay, the drawn-results convention: the stream exists only when the node makes it, draws only while
somebody subscribes, forwards stamp, frame id and camera_info, keeps one frame waiting; the drawing helpers; the depth
colours (shared with xblick view)."""

import argparse
import threading
import time

import numpy as np
import pytest
from test_camera import sessions, wait_until  # noqa: F401 - sessions is a fixture

from xblick import overlay
from xblick.camera import ImageSubscriber, query_info
from xblick.msg import CameraInfo


def test_the_option_is_off_by_default():
    parser = argparse.ArgumentParser()
    overlay.add_argument(parser)
    assert parser.parse_args([]).overlay is False and parser.parse_args(["--overlay"]).overlay is True


def test_the_stream_draws_only_for_a_subscriber_and_forwards_the_frame(sessions):  # noqa: F811
    a, b = sessions
    assert query_info(b, "t/cam/node/overlay", timeout=0.5) == []  # no overlay object: no stream
    drawn = []

    def draw(rgb, results):
        drawn.append(results)
        return overlay.draw_boxes(rgb, [(10, 10, 40, 30)], ["cup 0.9"], [overlay.colour(1)])

    o = overlay.Overlay(a, "t/cam", "node", draw)
    info = CameraInfo.pinhole(64, 48, 50.0, 50.0, 31.5, 23.5, frame_id="cam")
    rgb = np.zeros((48, 64, 3), np.uint8)
    try:
        o.put(1_000, "cam", rgb, "r1", camera_info=info)
        time.sleep(0.3)
        assert drawn == [] and o.published == 0  # nobody subscribes: nothing drawn
        images = ImageSubscriber(b, "t/cam/node/overlay", transport="raw", camera_info=True)
        wait_until(lambda: o.publisher.subscribed, timeout=5.0)
        end = time.monotonic() + 10
        frame, stamp = None, 2_000
        while frame is None and time.monotonic() < end:
            o.put(stamp, "cam", rgb, "r2", camera_info=info)
            frame = images.latest(timeout=0.2)
            stamp += 1
        assert frame is not None and frame.encoding == "rgb8" and frame.image.shape == (48, 64, 3)
        assert frame.header.frame_id == "cam" and 2_000 <= frame.stamp_ns < stamp
        assert frame.image[10, 25].tolist() == list(overlay.colour(1))  # the box's top edge
        wait_until(lambda: images.camera_info is not None, timeout=5.0)
        assert images.camera_info.k[0] == 50.0
        assert o.base == "t/cam/node/overlay"
        images.close()
    finally:
        o.close()
    o.check()


def test_a_newer_frame_replaces_the_waiting_one(sessions):  # noqa: F811
    a, b = sessions
    release = threading.Event()
    drawn = []

    def slow_draw(rgb, results):
        drawn.append(results)
        release.wait(5)
        return rgb

    o = overlay.Overlay(a, "t/cam", "slow", slow_draw)
    images = ImageSubscriber(b, "t/cam/slow/overlay", transport="raw")
    try:
        wait_until(lambda: o.publisher.subscribed, timeout=5.0)
        rgb = np.zeros((8, 8, 3), np.uint8)
        o.put(1, "c", rgb, "first")
        wait_until(lambda: drawn == ["first"], timeout=5.0)
        for name in ("second", "third", "fourth"):  # while it draws the first: one waits, the newest
            o.put(2, "c", rgb, name)
        release.set()
        wait_until(lambda: len(drawn) == 2, timeout=5.0)
        time.sleep(0.3)
        assert drawn == ["first", "fourth"] and o.replaced == 2
    finally:
        images.close()
        o.close()


def test_an_error_in_the_drawing_thread_is_raised_by_check(sessions):  # noqa: F811
    a, b = sessions
    o = overlay.Overlay(a, "t/cam", "bad", lambda rgb, r: 1 / 0)
    images = ImageSubscriber(b, "t/cam/bad/overlay", transport="raw")
    try:
        wait_until(lambda: o.publisher.subscribed, timeout=5.0)
        o.put(1, "c", np.zeros((8, 8, 3), np.uint8), None)
        wait_until(lambda: o._error is not None, timeout=5.0)
        with pytest.raises(RuntimeError, match="the overlay failed: ZeroDivisionError"):
            o.check()
    finally:
        images.close()
        o.close()


def test_the_drawing_helpers():
    rgb = np.zeros((60, 80, 3), np.uint8)
    labels = np.zeros((60, 80), np.int32)
    labels[10:20, 10:20] = 3
    masked = overlay.draw_masks(rgb, labels, alpha=1.0)
    assert masked[15, 15].tolist() == list(overlay.colour(3)) and masked[40, 40].tolist() == [0, 0, 0]
    points = overlay.draw_points(rgb, [(10, 10), (50, 40), (70, 5)], edges=[(0, 1)], valid=[True, True, False])
    assert points[10, 10].tolist() == [255, 255, 0] and points[25, 30].tolist() == [255, 255, 0]  # on the edge
    assert points[5, 70].tolist() == [0, 0, 0]  # the invalid one left out
    tracks = overlay.draw_tracks(rgb, [[(5, 50), (30, 50)], []])
    assert tracks[50, 20].tolist() == list(overlay.colour(0))
    assert overlay.colour(1) != overlay.colour(2) and overlay.colour(5) == overlay.colour(5)
    assert rgb.sum() == 0  # the input untouched


def test_depth_colours_have_a_fixed_range():
    depth = np.array([[0, 300, 100, 2650, 5000, 9000]], np.uint16)
    c = overlay.depth_colours(depth)
    assert c[0, 0].tolist() == [0, 0, 0]  # no depth
    assert c[0, 1].tolist() == [255, 0, 0] and c[0, 2].tolist() == [255, 0, 0]  # near and nearer: red
    assert c[0, 3].tolist() == [0, 255, 0]  # half way: green (hue 120 degrees)
    assert c[0, 4].tolist() == [0, 0, 255] and c[0, 5].tolist() == [0, 0, 255]  # far and farther: blue
    assert (overlay.depth_colours(depth, 300, 5000) == c).all()  # the same every time: no per-frame range
