"""xblick rerun: frames, calibration and body tracking results reach a Rerun recording, over real Zenoh
sessions."""

import contextlib
import io
import json
import sys
import time
from pathlib import Path

import numpy as np
import pytest
from PIL import Image as PILImage
from test_camera import BASE, pattern, sessions, wait_until  # noqa: F401 - sessions is a fixture

from xblick.camera import (CameraPublisher, CountedPublisher, ProtocolError, StaticTransformPublisher,
                           TransformPublisher, cdr_encoding, find_streams)
from xblick.codecs import PNG
from xblick.detections import DetectionPublisher
from xblick.msg import (BoundingBox2D, BoundingBox3D, CameraInfo, Classification, Detection2D, Detection2DArray,
                        Detection3D, Detection3DArray, DiagnosticArray, DiagnosticStatus, Header, Imu, KeyValue,
                        ObjectHypothesis, ObjectHypothesisWithPose, Point, Point2D, PointField, Pose, Pose2D, Quaternion, Time,
                        TFMessage, Transform, TransformStamped, Vector3)
from xblick.points import XYZRGB, PointCloudPublisher, pack_rgb, points_dtype
from xblick import rerun_bridge
from xblick.cli import _bridged
from xblick.rerun_bridge import K4ABT_BODY_32, TIMELINE, RerunBridge, label_color
from xblick.topics import WORLD

CAMERA = BASE.rpartition("/")[0]


class FakeRerun:
    """What the bridge logs: (entity, archetype name, fields, time)."""

    class ViewCoordinates:
        RDF = ("ViewCoordinates", {"xyz": "RDF"})
        RIGHT_HAND_Z_UP = ("ViewCoordinates", {"xyz": "RFU"})

    def __init__(self):
        self.logged = []
        self.blueprints = []
        self._time = None

    def set_time(self, timeline, *, timestamp):
        assert timeline == TIMELINE
        self._time = timestamp

    def log(self, entity, archetype, static=False):
        self.logged.append((entity, *archetype, None if static else self._time))

    @staticmethod
    def Pinhole(**fields):
        return "Pinhole", fields

    @staticmethod
    def EncodedImage(**fields):
        return "EncodedImage", fields

    @staticmethod
    def DepthImage(image, **fields):
        return "DepthImage", {"image": image, **fields}

    @staticmethod
    def Image(image, **fields):
        return "Image", {"image": image, **fields}

    @staticmethod
    def SegmentationImage(image, **fields):
        return "SegmentationImage", {"image": image, **fields}

    @staticmethod
    def Points3D(positions, **fields):
        return "Points3D", {"positions": positions, **fields}

    @staticmethod
    def Points2D(positions, **fields):
        return "Points2D", {"positions": positions, **fields}

    @staticmethod
    def Boxes3D(**fields):
        return "Boxes3D", fields

    @staticmethod
    def LineStrips2D(strips, **fields):
        return "LineStrips2D", {"strips": strips, **fields}

    @staticmethod
    def Quaternion(*, xyzw):
        return tuple(xyzw)

    @staticmethod
    def ClassDescription(**fields):
        return fields

    @staticmethod
    def AnnotationContext(classes):
        return "AnnotationContext", {"classes": classes}

    @staticmethod
    def Clear(*, recursive):
        return "Clear", {"recursive": recursive}

    @staticmethod
    def Transform3D(**fields):
        return "Transform3D", fields

    @staticmethod
    def TransformAxes3D(**fields):
        return "TransformAxes3D", fields

    @staticmethod
    def LineStrips3D(strips, **fields):
        return "LineStrips3D", {"strips": strips, **fields}

    @staticmethod
    def Scalars(values):
        return "Scalars", {"values": list(values)}

    @staticmethod
    def SeriesLines(**fields):
        return "SeriesLines", fields

    @staticmethod
    def TextLog(text, **fields):
        return "TextLog", {"text": text, **fields}

    @staticmethod
    def TextDocument(text, **fields):
        return "TextDocument", {"text": text, **fields}

    @staticmethod
    def escape_entity_path_part(part):
        return part.replace("/", "\\/")

    class blueprint:  # noqa: N801 - rerun.blueprint; a layout as {view name: its contents}
        @staticmethod
        def Spatial3DView(*, origin, name):
            return name, None

        @staticmethod
        def Spatial2DView(*, origin, name, contents):
            return name, contents

        @staticmethod
        def TimeSeriesView(*, origin, name):
            return name, "time series"

        @staticmethod
        def TextLogView(*, origin, name):
            return name, "text log"

        @staticmethod
        def TextDocumentView(*, origin, name):
            return name, "text document"

        @staticmethod
        def Grid(*views, **shares):  # containers: their views flattened into one {name: contents}
            return dict(v for view in views for v in (view.items() if isinstance(view, dict) else [view]))

        Horizontal = Vertical = Tabs = Grid

        @staticmethod
        def Blueprint(layout):
            return layout

    def send_blueprint(self, blueprint):
        self.blueprints.append(blueprint)


def of(rr, kind):
    return [entry for entry in rr.logged if entry[1] == kind]


def without_ages(layout: dict) -> dict:
    """A layout without the rates' and ages' views (test_ages_at_send has them)."""
    return {name: v for name, v in layout.items()
            if name not in ("age at send ms", "age at receive ms", "rate Hz", "rates and ages now")}


def test_frames_and_calibration(sessions):  # noqa: F811
    a, b = sessions
    info = CameraInfo.pinhole(64, 48, 50.0, 50.0, 31.5, 23.5, frame_id="cam")
    publisher = CameraPublisher(a, BASE, encoding="rgb8", frame_id="cam", camera_info=info, source="sim")
    rr = FakeRerun()
    bridge = RerunBridge(b, rr, [BASE])
    wait_until(lambda: publisher.subscribed)
    stamps = [time.time_ns() + k for k in range(3)]
    for k, stamp in enumerate(stamps):
        publisher.publish(pattern(k), stamp_ns=stamp)
        wait_until(lambda k=k: len(of(rr, "EncodedImage")) == k + 1)
    wait_until(lambda: len(of(rr, "Pinhole")) == 1)
    time.sleep(0.2)
    bridge.check()
    images = of(rr, "EncodedImage")
    assert [entity for entity, *_ in images] == [BASE] * 3
    assert [t for *_, t in images] == [np.datetime64(s, "ns") for s in stamps]
    for k, (_, _, fields, _) in enumerate(images):
        assert fields["media_type"] == "image/jpeg"
        # the publisher's JPEG as it is: standard colour order, decodes to the picture
        decoded = np.asarray(PILImage.open(io.BytesIO(fields["contents"])).convert("RGB"))
        assert np.abs(decoded.astype(int) - pattern(k)).mean() < 6
    # the calibration once: it does not change from frame to frame
    pinholes = of(rr, "Pinhole")
    assert len(pinholes) == 1
    _, _, fields, _ = pinholes[0]
    assert np.array_equal(fields["image_from_camera"], info.k.reshape(3, 3))
    assert fields["resolution"] == [64, 48] and fields["camera_xyz"] == FakeRerun.ViewCoordinates.RDF
    # the scene in camera axes, once
    assert [(e, f, t) for e, kind, f, t in rr.logged if kind == "ViewCoordinates"] == [("/", {"xyz": "RDF"}, None)]
    assert bridge.frames[BASE] == 3
    bridge.close()
    publisher.close()


def sixteen_bit(seed=0):
    return (np.arange(48 * 64, dtype=np.uint16).reshape(48, 64) * 7 + seed).astype(np.uint16)


def test_sixteen_bit_images(sessions):  # noqa: F811
    a, b = sessions
    depth = CameraPublisher(a, f"{CAMERA}/depth", encoding="16UC1", frame_id="depth_cam", codec=PNG)
    ir = CameraPublisher(a, f"{CAMERA}/ir", encoding="mono16", frame_id="depth_cam", codec=PNG)
    rr = FakeRerun()
    bridge = RerunBridge(b, rr, [f"{CAMERA}/depth", f"{CAMERA}/ir"])
    wait_until(lambda: depth.subscribed and ir.subscribed)
    depth.publish(sixteen_bit(1))
    ir.publish(sixteen_bit(2))
    wait_until(lambda: len(of(rr, "DepthImage")) == 1 and len(of(rr, "Image")) == 1)
    bridge.check()
    # depth in millimetres (REP 118) as a depth image, infrared as a grey image, both lossless
    (entity, _, fields, _), = of(rr, "DepthImage")
    assert entity == f"{CAMERA}/depth" and fields["meter"] == 1000.0
    assert fields["image"].dtype == np.uint16 and np.array_equal(fields["image"], sixteen_bit(1))
    (entity, _, fields, _), = of(rr, "Image")
    assert entity == f"{CAMERA}/ir" and np.array_equal(fields["image"], sixteen_bit(2))
    bridge.close()
    depth.close()
    ir.close()


def bodies_json(ids, stamp_ns, frame_id="depth_cam", version=1):
    """A result as xblick-k4abt publishes it: body k's joint j at (k, j, 1000 + j) mm."""
    return json.dumps({"v": version, "model": "K4ABT_BODY_32", "stamp_ns": stamp_ns, "frame_id": frame_id,
                       "latency_ms": 10.0,
                       "bodies": [{"id": k, "joints": {name: {"position_mm": [k, j, 1000 + j],
                                                              "orientation_wxyz": [1, 0, 0, 0], "confidence": 2}
                                                       for j, name in enumerate(K4ABT_BODY_32)}}
                                  for k in ids]})


def test_bodies(sessions):  # noqa: F811
    a, b = sessions
    depth_base, color_base = f"{CAMERA}/depth", f"{CAMERA}/color"
    info = CameraInfo.pinhole(64, 48, 50.0, 50.0, 31.5, 23.5, frame_id="depth_cam")
    depth = CameraPublisher(a, depth_base, encoding="16UC1", frame_id="depth_cam", camera_info=info, codec=PNG)
    color = CameraPublisher(a, color_base, encoding="rgb8", frame_id="color_cam",
                            camera_info=CameraInfo.pinhole(64, 48, 50.0, 50.0, 31.5, 23.5, frame_id="color_cam"))
    bodies = a.declare_publisher(f"{CAMERA}/bodies")
    rr = FakeRerun()
    bridge = RerunBridge(b, rr, [color_base, depth_base])
    assert bridge.cameras == [CAMERA]
    # the layout from the start: a 3D view and a 2D view per stream, no bodies in them yet
    assert without_ages(rr.blueprints[-1]) == {"3D": None, color_base: ["$origin/**"], depth_base: ["$origin/**"]}
    wait_until(lambda: depth.subscribed and color.subscribed)
    time.sleep(0.3)  # the bridge's bodies subscriber, declared with the others, reaches the publisher

    # bodies beside the streams (Rerun draws no 3D content below a pinhole): in 3D at once
    bodies.put(bodies_json([1, 2], 100))
    wait_until(lambda: bridge.frames[f"{CAMERA}/bodies"] == 1 or bridge._error is not None)
    bridge.check()
    (entity, _, context, t), = of(rr, "AnnotationContext")
    assert entity == f"{CAMERA}/bodies" and t is None  # static
    (skeleton,) = context["classes"]
    assert skeleton["info"] == (0, "K4ABT_BODY_32") and len(skeleton["keypoint_annotations"]) == 32
    assert len(skeleton["keypoint_connections"]) == 31 and (0, 1) in skeleton["keypoint_connections"]  # pelvis-spine
    points = {e: (f, t) for e, _, f, t in of(rr, "Points3D")}
    assert set(points) == {f"{CAMERA}/bodies/1", f"{CAMERA}/bodies/2"}
    fields, t = points[f"{CAMERA}/bodies/2"]
    assert t == np.datetime64(100, "ns")
    assert np.allclose(fields["positions"][5], [0.002, 0.005, 1.005])  # mm -> m
    assert list(fields["keypoint_ids"]) == list(range(32)) and fields["class_ids"] == 0

    # once the depth stream's calibration names the bodies' frame, its 2D view shows them; colour's does not
    depth.publish(sixteen_bit())
    color.publish(pattern())
    wait_until(lambda: len(of(rr, "Pinhole")) == 2)
    bridge.check()
    assert without_ages(rr.blueprints[-1]) == {"3D": None, color_base: ["$origin/**"],
                                 depth_base: ["$origin/**", f"{CAMERA}/bodies/**"]}

    # a body that left is cleared
    rr.logged.clear()
    bodies.put(bodies_json([1], 300))
    wait_until(lambda: bridge.frames[f"{CAMERA}/bodies"] == 2 or bridge._error is not None)
    bridge.check()
    assert [(e, kind) for e, kind, *_ in rr.logged if kind == "Clear"] == [(f"{CAMERA}/bodies/2", "Clear")]
    assert [e for e, *_ in of(rr, "Points3D")] == [f"{CAMERA}/bodies/1"]

    # another version of the bodies' JSON is refused, naming both
    bodies.put(bodies_json([1], 400, version=2))
    wait_until(lambda: bridge._error is not None)
    with pytest.raises(ProtocolError, match="version 2, this bridge reads 1"):
        bridge.check()
    bridge.close()
    bodies.undeclare()
    depth.close()
    color.close()


def test_results_of_a_topic_gone_quiet_are_cleared(sessions):  # noqa: F811
    a, b = sessions
    color_base = f"{CAMERA}/color"
    color = CameraPublisher(a, color_base, encoding="rgb8", frame_id="color_cam",
                            camera_info=CameraInfo.pinhole(64, 48, 50.0, 50.0, 31.5, 23.5, frame_id="color_cam"))
    bodies = a.declare_publisher(f"{CAMERA}/bodies")
    rr = FakeRerun()
    bridge = RerunBridge(b, rr, [color_base], stale_after=0.5)
    time.sleep(0.3)  # the bridge's bodies subscriber reaches the publisher
    clears = lambda: [e for e, kind, *_ in rr.logged if kind == "Clear"]  # noqa: E731

    # while results come nothing is cleared
    for stamp in (100, 200, 300, 400):
        bodies.put(bodies_json([1, 2], stamp))
        time.sleep(0.2)
    bridge.check()
    assert clears() == []

    # the topic goes quiet: each body it drew is cleared, once, after the last stamp
    wait_until(lambda: len(clears()) == 2 or bridge._error is not None)
    bridge.check()
    assert sorted(clears()) == [f"{CAMERA}/bodies/1", f"{CAMERA}/bodies/2"]
    assert rr.logged[-1][-1] > np.datetime64(400, "ns")
    time.sleep(0.7)
    assert len(clears()) == 2

    # it comes back: drawn again, and nothing to clear for a body that was already cleared
    rr.logged.clear()
    bodies.put(bodies_json([1], 500))
    wait_until(lambda: len(of(rr, "Points3D")) == 1 or bridge._error is not None)
    bridge.check()
    assert clears() == []
    bridge.close()
    bodies.undeclare()
    color.close()


def test_a_nodes_bodies_below_the_camera(sessions):  # noqa: F811
    """xblick-gem-x's <camera>/gem_x/bodies beside the Azure Kinect tracker's <camera>/bodies: each its own entity,
    each shown in the 2D view of the stream in its own frame (gem-x: the colour camera's)."""
    a, b = sessions
    depth_base, color_base = f"{CAMERA}/depth", f"{CAMERA}/color"
    depth = CameraPublisher(a, depth_base, encoding="16UC1", frame_id="depth_cam", codec=PNG,
                            camera_info=CameraInfo.pinhole(64, 48, 50.0, 50.0, 31.5, 23.5, frame_id="depth_cam"))
    color = CameraPublisher(a, color_base, encoding="rgb8", frame_id="color_cam",
                            camera_info=CameraInfo.pinhole(64, 48, 50.0, 50.0, 31.5, 23.5, frame_id="color_cam"))
    kinect, gemx = a.declare_publisher(f"{CAMERA}/bodies"), a.declare_publisher(f"{CAMERA}/gem_x/bodies")
    rr = FakeRerun()
    bridge = RerunBridge(b, rr, [color_base, depth_base])
    wait_until(lambda: depth.subscribed and color.subscribed)
    time.sleep(0.3)
    kinect.put(bodies_json([1], 100))
    gemx.put(bodies_json([1, 2], 100, frame_id="color_cam"))
    wait_until(lambda: (bridge.frames[f"{CAMERA}/bodies"] == 1 and bridge.frames[f"{CAMERA}/gem_x/bodies"] == 1)
               or bridge._error is not None)
    bridge.check()
    assert {e for e, *_ in of(rr, "Points3D")} == {f"{CAMERA}/bodies/1", f"{CAMERA}/gem_x/bodies/1",
                                                     f"{CAMERA}/gem_x/bodies/2"}
    depth.publish(sixteen_bit())
    color.publish(pattern())
    wait_until(lambda: len(of(rr, "Pinhole")) == 2)
    bridge.check()
    assert without_ages(rr.blueprints[-1]) == {"3D": None, color_base: ["$origin/**", f"{CAMERA}/gem_x/bodies/**"],
                                 depth_base: ["$origin/**", f"{CAMERA}/bodies/**"]}
    bridge.close()
    kinect.undeclare()
    gemx.undeclare()
    depth.close()
    color.close()


def depth_to_color(parent="depth_cam", child="color_cam"):
    """The colour camera 32 mm to the depth camera's right, turned 90 degrees about x (a pose easy to check)."""
    h = np.sqrt(0.5)
    return [TransformStamped(Header(frame_id=parent), child, Transform(Vector3(0.032, 0.0, 0.0),
                                                                       Quaternion(h, 0.0, 0.0, h)))]


def test_static_transforms_place_the_streams(sessions):  # noqa: F811
    a, b = sessions
    depth_base, color_base = f"{CAMERA}/depth", f"{CAMERA}/color"
    depth = CameraPublisher(a, depth_base, encoding="16UC1", frame_id="depth_cam", codec=PNG,
                            camera_info=CameraInfo.pinhole(64, 48, 50.0, 50.0, 31.5, 23.5, frame_id="depth_cam"))
    color = CameraPublisher(a, color_base, encoding="rgb8", frame_id="color_cam",
                            camera_info=CameraInfo.pinhole(64, 48, 50.0, 50.0, 31.5, 23.5, frame_id="color_cam"))
    transforms = StaticTransformPublisher(a, CAMERA, depth_to_color())  # before the bridge: it asks for them
    bodies = a.declare_publisher(f"{CAMERA}/bodies")
    rr = FakeRerun()
    bridge = RerunBridge(b, rr, [color_base, depth_base])
    wait_until(lambda: depth.subscribed and color.subscribed)
    time.sleep(0.3)
    depth.publish(sixteen_bit())
    color.publish(pattern())
    bodies.put(bodies_json([1], 100))
    wait_until(lambda: (len(of(rr, "Pinhole")) == 2 and bridge.frames[f"{CAMERA}/bodies"] == 1)
               or bridge._error is not None)
    wait_until(lambda: len({e for e, *_ in of(rr, "Transform3D")}) == 3 or bridge._error is not None)
    bridge.check()
    placed = {e: (f, t) for e, _, f, t in of(rr, "Transform3D")}  # the last pose logged per entity
    fields, t = placed[color_base]
    assert t is None  # static
    assert np.allclose(fields["translation"], [0.032, 0, 0])
    assert np.allclose(fields["mat3x3"], [[1, 0, 0], [0, 0, -1], [0, 1, 0]])  # 90 degrees about x
    for entity in (depth_base, f"{CAMERA}/bodies"):  # the root frame: where the camera's entity is
        assert np.allclose(placed[entity][0]["mat3x3"], np.eye(3)) and np.allclose(placed[entity][0]["translation"], 0)
    # both streams are related to the bodies' frame now: both 2D views show them
    assert without_ages(rr.blueprints[-1]) == {"3D": None, color_base: ["$origin/**", f"{CAMERA}/bodies/**"],
                                 depth_base: ["$origin/**", f"{CAMERA}/bodies/**"]}
    bridge.close()
    transforms.close()
    bodies.undeclare()
    depth.close()
    color.close()


def test_imu_and_diagnostics(sessions):  # noqa: F811
    a, b = sessions
    depth_base = f"{CAMERA}/depth"
    rr = FakeRerun()
    bridge = RerunBridge(b, rr, [depth_base])
    time.sleep(0.3)
    for k in range(3):
        imu = Imu(Header(Time(100, k), "AzureKinectImu"), angular_velocity=Vector3(0.1 * k, 0.0, -0.1),
                  linear_acceleration=Vector3(0.0, -9.81, 0.01 * k))
        a.put(f"{CAMERA}/imu", bytes(imu.serialize()), encoding=cdr_encoding(Imu))
    diagnostics = DiagnosticArray(Header(Time(101, 0)), [DiagnosticStatus(
        DiagnosticStatus.WARN, "azure_kinect", "2 dropped", "000123",
        [KeyValue("captures/s", "29.5"), KeyValue("firmware rgb", "1.6.110")])])
    a.put(f"{CAMERA}/diagnostics", bytes(diagnostics.serialize()), encoding=cdr_encoding(DiagnosticArray))
    wait_until(lambda: (bridge.frames[f"{CAMERA}/imu"] == 3 and bridge.frames[f"{CAMERA}/diagnostics"] == 1)
               or bridge._error is not None)
    bridge.check()
    scalars = [(e, f["values"], t) for e, kind, f, t in rr.logged if kind == "Scalars"]
    acceleration = [(v, t) for e, v, t in scalars if e == f"{CAMERA}/imu/linear_acceleration"]
    assert [v for v, _ in acceleration] == [[0.0, -9.81, 0.0], [0.0, -9.81, 0.01], [0.0, -9.81, 0.02]]
    assert [t for _, t in acceleration] == [np.datetime64(100 * 10**9 + k, "ns") for k in range(3)]
    assert (f"{CAMERA}/imu/angular_velocity", [0.2, 0.0, -0.1], np.datetime64(100 * 10**9 + 2, "ns")) in scalars
    names = {e: f["names"] for e, kind, f, t in rr.logged if kind == "SeriesLines"}
    assert names == {f"{CAMERA}/imu/linear_acceleration": ["x", "y", "z"],
                     f"{CAMERA}/imu/angular_velocity": ["x", "y", "z"]}
    (entity, _, text, _), = of(rr, "TextLog")
    assert entity == f"{CAMERA}/diagnostics/azure_kinect"
    assert text == {"text": "azure_kinect WARN: 2 dropped", "level": "WARN"}
    # numbers become curves, the rest (firmware) does not
    assert [(e, v) for e, v, _ in scalars if "/diagnostics/" in e] == [
        (f"{CAMERA}/diagnostics/azure_kinect/captures\\/s", [29.5])]
    assert without_ages(rr.blueprints[-1]) == {"3D": None, depth_base: ["$origin/**"], f"{CAMERA}/imu": "time series",
                                 f"{CAMERA}/diagnostics": "text log", f"{CAMERA}/diagnostics values": "time series"}
    bridge.close()


def small_cloud():
    """A 4 x 5 XYZRGB cloud with two invalid points; red = column."""
    points = np.zeros((4, 5), points_dtype(*XYZRGB))
    v, u = np.mgrid[0:4, 0:5].astype(np.float32)
    points["x"], points["y"], points["z"] = u * 0.1, v * 0.1, 1.0
    points["z"][0, :2] = np.nan
    points["rgb"] = pack_rgb(np.stack([u, v, np.full_like(u, 9)], -1).astype(np.uint8))
    return points


def test_point_clouds(sessions):  # noqa: F811
    a, b = sessions
    cloud_base = f"{CAMERA}/points"
    transforms = StaticTransformPublisher(a, CAMERA, depth_to_color())
    publisher = PointCloudPublisher(a, cloud_base, fields=XYZRGB[0], point_step=16, frame_id="color_cam")
    rr = FakeRerun()
    bridge = RerunBridge(b, rr, [], clouds=[cloud_base], cloud_rate=50.0)
    wait_until(lambda: publisher.subscribed)
    publisher.publish(small_cloud(), stamp_ns=5 * 10**9)
    wait_until(lambda: bridge.frames[cloud_base] == 1 or bridge._error is not None)
    # placed at the camera's origin first, then in the colour camera's frame once tf_static came (slow hosts: later)
    wait_until(lambda: np.allclose({e: f for e, _, f, _ in of(rr, "Transform3D")}.get(cloud_base, {}).get(
        "translation", [0, 0, 0]), [0.032, 0, 0]) or bridge._error is not None)
    bridge.check()
    (entity, _, fields, t), = of(rr, "Points3D")
    assert entity == cloud_base and t == np.datetime64(5 * 10**9, "ns")
    assert fields["positions"].shape == (18, 3) and np.isfinite(fields["positions"]).all()  # the valid ones
    np.testing.assert_array_equal(fields["colors"][:3], [[2, 0, 9], [3, 0, 9], [4, 0, 9]])
    placed = {e: f for e, _, f, _ in of(rr, "Transform3D")}
    assert np.allclose(placed[cloud_base]["translation"], [0.032, 0, 0])  # in the colour camera's frame
    bridge.close()
    publisher.close()
    transforms.close()


def test_a_nodes_tracks_over_its_image(sessions):  # noqa: F811
    """A node's cloud (<camera>/<node>/<name>) belongs to the camera: placed by its frame through the camera's
    transforms; coloured by object_id; its visible points' u, v in the 2D view of the stream of its frame."""
    a, b = sessions
    color_base, key = f"{CAMERA}/color", f"{CAMERA}/tapnext/tracks"
    color = CameraPublisher(a, color_base, encoding="rgb8", frame_id="color_cam",
                            camera_info=CameraInfo.pinhole(64, 48, 50.0, 50.0, 31.5, 23.5, frame_id="color_cam"))
    transforms = StaticTransformPublisher(a, CAMERA, depth_to_color())
    fields = [PointField("x", 0, PointField.FLOAT32), PointField("y", 4, PointField.FLOAT32),
              PointField("z", 8, PointField.FLOAT32), PointField("u", 12, PointField.FLOAT32),
              PointField("v", 16, PointField.FLOAT32), PointField("track_id", 20, PointField.UINT32),
              PointField("object_id", 24, PointField.UINT16), PointField("visible", 26, PointField.UINT8)]
    publisher = PointCloudPublisher(a, key, fields=fields, point_step=32, frame_id="color_cam")
    rr = FakeRerun()
    bridge = RerunBridge(b, rr, [color_base], clouds=[key], cloud_rate=50.0)
    wait_until(lambda: color.subscribed and publisher.subscribed)
    color.publish(pattern())
    wait_until(lambda: bridge._frame_id.get(color_base) == "color_cam" or bridge._error is not None)
    tracks = np.zeros((1, 3), points_dtype(fields, 32))
    tracks["x"], tracks["y"], tracks["z"] = [[0.1, 0.2, np.nan]], [[0.0, 0.1, np.nan]], [[1.0, 1.0, np.nan]]
    tracks["u"], tracks["v"] = [[10.0, 20.0, 30.0]], [[5.0, 6.0, 7.0]]
    tracks["track_id"], tracks["object_id"], tracks["visible"] = [[1, 2, 3]], [[4, 7, 7]], [[1, 1, 0]]
    publisher.publish(tracks, stamp_ns=5 * 10**9)
    wait_until(lambda: bridge.frames[key] == 1 or bridge._error is not None)
    wait_until(lambda: key in {e for e, *_ in of(rr, "Transform3D")} or bridge._error is not None)
    bridge.check()
    assert CAMERA in bridge.cameras and f"{CAMERA}/tapnext" not in bridge.cameras
    (entity, _, points, _), = of(rr, "Points3D")
    assert entity == key and points["positions"].shape == (2, 3)  # the hidden one has no position
    np.testing.assert_array_equal(points["colors"], [label_color("4"), label_color("7")])
    (entity, _, points, t), = of(rr, "Points2D")
    assert entity == f"{color_base}/tapnext/tracks" and t == np.datetime64(5 * 10**9, "ns")
    np.testing.assert_array_equal(points["positions"], [[10, 5], [20, 6]])  # only the visible ones
    placed = {e: f for e, _, f, _ in of(rr, "Transform3D")}
    assert np.allclose(placed[key]["translation"], [0.032, 0, 0])  # in the colour camera's frame
    for closable in (bridge, publisher, transforms, color):
        closable.close()


def test_point_clouds_in_the_world(sessions):  # noqa: F811
    a, b = sessions
    cloud_base = f"{CAMERA}/nvblox/surface"
    publisher = PointCloudPublisher(a, cloud_base, fields=XYZRGB[0], point_step=16, frame_id=WORLD)
    rr = FakeRerun()
    bridge = RerunBridge(b, rr, [], clouds=[cloud_base], cloud_rate=50.0)
    wait_until(lambda: publisher.subscribed)
    publisher.publish(small_cloud(), stamp_ns=5 * 10**9)
    wait_until(lambda: bridge.frames[cloud_base] == 1 or bridge._error is not None)
    bridge.check()
    (entity, _, fields, _), = of(rr, "Points3D")
    assert entity == f"world/{cloud_base}"  # at the root, not below the camera's entity and its pose
    assert fields["positions"].shape == (18, 3)
    assert not [e for e, *_ in of(rr, "Transform3D") if e == entity or e.startswith(f"{CAMERA}")]
    assert ("/", "ViewCoordinates", {"xyz": "RFU"}, None) in rr.logged  # the scene z up, as with camera poses
    bridge.close()
    publisher.close()


def test_find_streams(sessions):  # noqa: F811
    a, b = sessions
    publisher = CameraPublisher(a, BASE, encoding="rgb8", frame_id="cam", source="sim")
    # an answer that is no info record (not UTF-8, not JSON) is left out, discovery goes on
    garbled = a.declare_queryable("robot/garbled/info", lambda q: q.reply("robot/garbled/info", b"\xff\xfe\x00"))
    found = find_streams(b, "robot", timeout=2.0)
    garbled.undeclare()
    assert [base for base, _ in found] == [BASE]
    assert found[0][1]["source"] == "sim"
    assert find_streams(b, "robot2", timeout=0.5) == []
    publisher.close()


def test_node_results_are_found(sessions):  # noqa: F811
    """Detections and label images answer <key>/info as streams do: xblick rerun finds them without options, and
    bridges those that start later."""
    a, b = sessions
    color_base, key = f"{CAMERA}/color", f"{CAMERA}/sam3/instances"
    boxes_key = f"{CAMERA}/foundationpose/detections_3d"
    color = CameraPublisher(a, color_base, encoding="rgb8", frame_id="color_cam")
    labels = CameraPublisher(a, key, encoding="mono16", frame_id="color_cam", codec=PNG, labels=True)
    with pytest.raises(ValueError, match="mono8 or mono16"):
        CameraPublisher(a, f"{CAMERA}/sam3/other", encoding="rgb8", frame_id="color_cam", labels=True)
    # a stream without a compressed form is not bridged (the bridge reads compressed images)
    disparity = CameraPublisher(a, f"{CAMERA}/ffs/disparity", encoding="32FC1", frame_id="color_cam", codec=None)
    found = _bridged(find_streams(b, "robot", timeout=2.0))
    assert found == {"bases": [color_base], "clouds": [], "detections": [], "segmentations": [key]}
    bridge = RerunBridge(b, FakeRerun(), **found)
    boxes = DetectionPublisher(a, boxes_key, Detection3DArray)
    found = _bridged(find_streams(b, "robot", timeout=2.0))
    assert found["detections"] == [boxes_key]
    assert dict(find_streams(b, "robot", timeout=1.0))[boxes_key]["type"] == "vision_msgs/msg/Detection3DArray"
    assert bridge.add(**found) == [boxes_key]
    assert bridge.add(**found) == [] and bridge.detections == [boxes_key]
    for closable in (bridge, boxes, labels, color, disparity):
        closable.close()


def pose_at(x, stamp, parent="world"):
    return [TransformStamped(Header(Time.from_ns(stamp), parent), "color_cam",
                             Transform(Vector3(x, 2.0, 3.0), Quaternion(0.0, 0.0, 0.0, 1.0)))]


def test_trajectory(sessions, monkeypatch):  # noqa: F811
    """Poses on a node's key: outside the camera's entity, its path as a line, axes at the newest pose."""
    monkeypatch.setattr(rerun_bridge, "TRAIL_PERIOD_S", 0.0)
    a, b = sessions
    key = f"{CAMERA}/vslam/tf"
    entity = f"trajectories/{key}"
    publisher = CountedPublisher(a, key, cdr_encoding(TFMessage))
    color = CameraPublisher(a, f"{CAMERA}/color", encoding="rgb8", frame_id="color_cam")
    rr = FakeRerun()
    bridge = RerunBridge(b, rr, [f"{CAMERA}/color"], trajectories=[key])
    stamp = time.time_ns()
    time.sleep(0.2)
    for i, x in enumerate((1.0, 1.001, 1.1)):  # the second too close to the first for a point of the line
        publisher.put(bytes(TFMessage(transforms=pose_at(x, stamp + i)).serialize()))
        wait_until(lambda: bridge.frames[f"trajectories/{key}"] == i + 1)
    bridge.check()
    poses = [f for e, kind, f, _ in rr.logged if e == f"{entity}/pose" and kind == "Transform3D"]
    assert [p["translation"][0] for p in poses] == [1.0, 1.001, 1.1]
    assert ("/", "ViewCoordinates", {"xyz": "RFU"}, None) in rr.logged
    lines = [f for e, kind, f, _ in rr.logged if e == entity and kind == "LineStrips3D"]
    assert len(lines) == 1 and np.allclose(lines[0]["strips"][0][:, 0], [1.0, 1.1])
    assert not any(e == CAMERA and kind == "Transform3D" for e, kind, *_ in rr.logged)  # the camera does not move
    publisher.put(bytes(TFMessage(transforms=pose_at(2.0, stamp + 9, parent="base_link")).serialize()))
    wait_until(lambda: bridge._error is not None)
    with pytest.raises(ProtocolError, match="not in 'world'"):
        bridge.check()
    bridge.close()
    color.close()
    publisher.close()


def test_real_rerun_takes_a_trajectory(sessions):  # noqa: F811
    rr = pytest.importorskip("rerun")
    a, b = sessions
    rr.init("xblick-test", strict=True)      # strict: a rejected archetype raises instead of a warning
    rr.memory_recording()
    key = f"{CAMERA}/vslam/tf"
    publisher = CountedPublisher(a, key, cdr_encoding(TFMessage))
    color = CameraPublisher(a, f"{CAMERA}/color", encoding="rgb8", frame_id="color_cam")
    bridge = RerunBridge(b, rr, [f"{CAMERA}/color"], trajectories=[key])
    time.sleep(0.2)
    stamp = time.time_ns()
    for i, x in enumerate((1.0, 1.1)):
        publisher.put(bytes(TFMessage(transforms=pose_at(x, stamp + i)).serialize()))
        wait_until(lambda: bridge.frames[f"trajectories/{key}"] == i + 1 or bridge._error is not None)
    bridge.check()
    bridge.close()
    color.close()
    publisher.close()


def test_real_rerun_takes_what_the_bridge_logs(sessions):  # noqa: F811
    rr = pytest.importorskip("rerun")
    a, b = sessions
    rr.init("xblick-test", strict=True)      # strict: a rejected archetype raises instead of a warning
    rr.memory_recording()
    info = CameraInfo.pinhole(64, 48, 50.0, 50.0, 31.5, 23.5, frame_id="cam")
    publisher = CameraPublisher(a, BASE, encoding="rgb8", frame_id="cam", camera_info=info)
    bridge = RerunBridge(b, rr, [BASE])
    wait_until(lambda: publisher.subscribed)
    publisher.publish(pattern())
    wait_until(lambda: bridge.frames[BASE] == 1 or bridge._error is not None)
    wait_until(lambda: len(bridge._calibration) == 1 or bridge._error is not None)
    bridge.check()
    bridge.close()
    publisher.close()


def test_real_rerun_takes_depth_and_bodies(sessions):  # noqa: F811
    rr = pytest.importorskip("rerun")
    a, b = sessions
    rr.init("xblick-test", strict=True)
    rr.memory_recording()
    base = f"{CAMERA}/depth"
    info = CameraInfo.pinhole(64, 48, 50.0, 50.0, 31.5, 23.5, frame_id="depth_cam")
    depth = CameraPublisher(a, base, encoding="16UC1", frame_id="depth_cam", camera_info=info, codec=PNG)
    ir = CameraPublisher(a, f"{CAMERA}/ir", encoding="mono16", frame_id="depth_cam", codec=PNG)
    bodies = a.declare_publisher(f"{CAMERA}/bodies")
    transforms = StaticTransformPublisher(a, CAMERA, depth_to_color())
    bridge = RerunBridge(b, rr, [base, f"{CAMERA}/ir"])
    wait_until(lambda: depth.subscribed and ir.subscribed)
    depth.publish(sixteen_bit())
    ir.publish(sixteen_bit())
    wait_until(lambda: len(bridge._calibration) == 1 or bridge._error is not None)
    time.sleep(0.3)
    bodies.put(bodies_json([1, 2], 100))
    wait_until(lambda: bridge.frames[f"{CAMERA}/bodies"] == 1 or bridge._error is not None)
    bodies.put(bodies_json([1], 200))
    wait_until(lambda: bridge.frames[f"{CAMERA}/bodies"] == 2 or bridge._error is not None)
    wait_until(lambda: bridge.frames[base] == 1 and bridge.frames[f"{CAMERA}/ir"] == 1 or bridge._error is not None)
    wait_until(lambda: f"{CAMERA}/bodies" in bridge._placed or bridge._error is not None)
    a.put(f"{CAMERA}/imu", bytes(Imu(Header(Time(100, 0), "imu")).serialize()), encoding=cdr_encoding(Imu))
    status = DiagnosticStatus(DiagnosticStatus.STALE, "azure_kinect", "m", "", [KeyValue("captures/s", "30")])
    a.put(f"{CAMERA}/diagnostics", bytes(DiagnosticArray(Header(Time(100, 1)), [status]).serialize()),
          encoding=cdr_encoding(DiagnosticArray))
    wait_until(lambda: (bridge.frames[f"{CAMERA}/imu"] == 1 and bridge.frames[f"{CAMERA}/diagnostics"] == 1)
               or bridge._error is not None)
    bridge.check()
    bridge.close()
    transforms.close()
    bodies.undeclare()
    depth.close()
    ir.close()


def test_real_rerun_takes_point_clouds(sessions):  # noqa: F811
    rr = pytest.importorskip("rerun")
    a, b = sessions
    rr.init("xblick-test", strict=True)
    rr.memory_recording()
    cloud_base = f"{CAMERA}/points"
    transforms = StaticTransformPublisher(a, CAMERA, depth_to_color())
    publisher = PointCloudPublisher(a, cloud_base, fields=XYZRGB[0], point_step=16, frame_id="color_cam")
    bridge = RerunBridge(b, rr, [], clouds=[cloud_base])
    wait_until(lambda: publisher.subscribed)
    publisher.publish(small_cloud())
    wait_until(lambda: bridge.frames[cloud_base] == 1 or bridge._error is not None)
    bridge.check()
    bridge.close()
    publisher.close()
    transforms.close()


def hypothesis(label, score):
    return ObjectHypothesisWithPose(ObjectHypothesis(label, score))


def test_detections(sessions):  # noqa: F811
    """3D boxes at their key's entity, placed by their frame (the colour camera's, beside the depth camera); 2D boxes
    in the image of the stream of their frame, rotated by theta; an empty message clears them; another type on a key
    is an error."""
    a, b = sessions
    depth_base, color_base = f"{CAMERA}/depth", f"{CAMERA}/color"
    depth = CameraPublisher(a, depth_base, encoding="16UC1", frame_id="depth_cam", codec=PNG,
                            camera_info=CameraInfo.pinhole(64, 48, 50.0, 50.0, 31.5, 23.5, frame_id="depth_cam"))
    color = CameraPublisher(a, color_base, encoding="rgb8", frame_id="color_cam",
                            camera_info=CameraInfo.pinhole(64, 48, 50.0, 50.0, 31.5, 23.5, frame_id="color_cam"))
    transforms = StaticTransformPublisher(a, CAMERA, depth_to_color())
    boxes_key, boxes2d_key = f"{CAMERA}/boxer/boxes_3d", f"{CAMERA}/boxer/detections_2d"
    boxes = DetectionPublisher(a, boxes_key, Detection3DArray)
    boxes2d = DetectionPublisher(a, boxes2d_key, Detection2DArray)
    rr = FakeRerun()
    bridge = RerunBridge(b, rr, [depth_base, color_base], detections=[boxes_key, boxes2d_key])
    wait_until(lambda: color.subscribed and depth.subscribed)
    stamp = time.time_ns()
    depth.publish(np.zeros((48, 64), np.uint16), stamp_ns=stamp)
    color.publish(pattern(0), stamp_ns=stamp)
    wait_until(lambda: len(of(rr, "Pinhole")) == 2)
    h = np.sqrt(0.5)
    box = Detection3D(results=[hypothesis("cup", 0.25), hypothesis("mug", 0.75)],
                      bbox=BoundingBox3D(Pose(Point(0.1, -0.2, 1.5), Quaternion(0.0, 0.0, h, h)),
                                         Vector3(0.08, 0.12, 0.1)), id="7")
    boxes.publish(Detection3DArray(Header(Time.from_ns(stamp), "color_cam"), [box]))
    rotated = Detection2D(results=[hypothesis("chair", 0.5)],
                          bbox=BoundingBox2D(Pose2D(Point2D(32.0, 24.0), np.pi / 2), 20.0, 10.0))
    boxes2d.publish(Detection2DArray(Header(Time.from_ns(stamp), "color_cam"), [rotated]))
    wait_until(lambda: of(rr, "Boxes3D") and of(rr, "LineStrips2D"))
    bridge.check()
    entity, _, fields, logged_at = of(rr, "Boxes3D")[0]
    assert entity == boxes_key and logged_at == np.datetime64(stamp, "ns")
    assert fields["centers"] == [(0.1, -0.2, 1.5)] and fields["half_sizes"] == [(0.04, 0.06, 0.05)]
    assert fields["quaternions"] == [(0.0, 0.0, h, h)] and fields["labels"] == ["mug 0.75 #7"]
    # placed as the colour stream is: in the colour camera's frame, beside the depth camera
    placed = {e: f for e, kind, f, _ in rr.logged if kind == "Transform3D"}
    assert np.allclose(placed[boxes_key]["translation"], placed[color_base]["translation"])
    assert np.allclose(placed[boxes_key]["mat3x3"], placed[color_base]["mat3x3"])
    # and projected into the images of both streams (one transform tree): the depth view takes them once the bridge
    # has the camera's tf_static, which may come after the boxes (a loaded host)
    projected = {"3D": None, depth_base: ["$origin/**", f"{boxes_key}/**"],
                 color_base: ["$origin/**", f"{boxes_key}/**"]}
    with contextlib.suppress(TimeoutError):
        wait_until(lambda: without_ages(rr.blueprints[-1]) == projected)
    assert without_ages(rr.blueprints[-1]) == projected
    entity, _, fields, _ = of(rr, "LineStrips2D")[0]
    assert entity == f"{color_base}/boxer/detections_2d" and fields["labels"] == ["chair 0.50"]
    # 20 x 10 turned by 90 degrees: 10 wide, 20 high around (32, 24)
    corners = np.array(fields["strips"][0])
    assert np.allclose(corners.min(axis=0), (27.0, 14.0)) and np.allclose(corners.max(axis=0), (37.0, 34.0))
    # nothing found: cleared
    boxes.publish(Detection3DArray(Header(Time.from_ns(stamp + 1), "color_cam"), []))
    wait_until(lambda: (boxes_key, "Clear") in [(e, kind) for e, kind, *_ in rr.logged])
    # a key that carries another type
    a.put(boxes_key, b"not cdr", encoding=cdr_encoding(Imu))
    wait_until(lambda: bridge._error is not None or any(r._error for r in bridge._detection_readers))
    with pytest.raises(ProtocolError, match="none of"):
        bridge.check()
    for closable in (bridge, boxes, boxes2d, transforms, color, depth):
        closable.close()


def test_classification(sessions):  # noqa: F811
    """A Classification (xblick-vjepa2's actions) as a text log line at its key, best first, with a view of its
    own once it came."""
    a, b = sessions
    color_base, key = f"{CAMERA}/color", f"{CAMERA}/vjepa2/actions"
    color = CameraPublisher(a, color_base, encoding="rgb8", frame_id="color_cam",
                            camera_info=CameraInfo.pinhole(64, 48, 50.0, 50.0, 31.5, 23.5, frame_id="color_cam"))
    actions = DetectionPublisher(a, key, Classification)
    rr = FakeRerun()
    bridge = RerunBridge(b, rr, [color_base], detections=[key])
    assert key not in rr.blueprints[-1]
    wait_until(lambda: actions._publisher.matching_status.matching)
    stamp = time.time_ns()
    actions.publish(Classification(Header(Time.from_ns(stamp), "color_cam"),
                                   [ObjectHypothesis("Pouring something", 0.1),
                                    ObjectHypothesis("Opening something", 0.625)]))
    wait_until(lambda: of(rr, "TextLog"))
    bridge.check()
    (entity, _, fields, logged_at), = of(rr, "TextLog")
    assert entity == key and logged_at == np.datetime64(stamp, "ns")
    assert fields["text"] == "Opening something 0.62 | Pouring something 0.10"
    assert rr.blueprints[-1][key] == "text log"
    for closable in (bridge, actions, color):
        closable.close()


def instances():
    """A label image as xblick-sam3 publishes it: ids 1..3 (two over 255 apart: mono16), 0 elsewhere."""
    ids = np.zeros((48, 64), np.uint16)
    ids[5:20, 5:30], ids[25:40, 10:20], ids[30:45, 40:60] = 1, 2, 300
    return ids


def test_segmentation(sessions):  # noqa: F811
    """A label image as a segmentation image in the 2D view of the stream of its frame (not as an image of its own),
    its ids lossless; before that stream's camera_info it waits; another encoding is an error."""
    a, b = sessions
    color_base, key = f"{CAMERA}/color", f"{CAMERA}/sam3/instances"
    info = CameraInfo.pinhole(64, 48, 50.0, 50.0, 31.5, 23.5, frame_id="color_cam")
    color = CameraPublisher(a, color_base, encoding="rgb8", frame_id="color_cam", camera_info=info)
    labels = CameraPublisher(a, key, encoding="mono16", frame_id="color_cam", camera_info=info, codec=PNG)
    rr = FakeRerun()
    with pytest.raises(ValueError, match="a stream and a segmentation"):
        RerunBridge(b, rr, [color_base, key], segmentations=[key])
    bridge = RerunBridge(b, rr, [color_base], segmentations=[key])
    wait_until(lambda: color.subscribed and labels.subscribed)
    stamp = time.time_ns()
    color.publish(pattern(0), stamp_ns=stamp)
    wait_until(lambda: len(of(rr, "Pinhole")) == 1)
    labels.publish(instances(), stamp_ns=stamp)
    wait_until(lambda: of(rr, "SegmentationImage") or bridge._error is not None)
    bridge.check()
    (entity, _, fields, logged_at), = of(rr, "SegmentationImage")
    assert entity == f"{color_base}/sam3/instances" and logged_at == np.datetime64(stamp, "ns")
    assert np.array_equal(fields["image"], instances())
    assert [e for e, kind, *_ in rr.logged if kind in ("Image", "EncodedImage", "DepthImage")] == [color_base]
    other = CameraPublisher(a, key, encoding="rgb8", frame_id="color_cam", camera_info=info, codec=PNG)
    other.publish(pattern(1), stamp_ns=stamp + 1)
    wait_until(lambda: bridge._error is not None)
    with pytest.raises(ProtocolError, match="mono8 or mono16"):
        bridge.check()
    for closable in (bridge, labels, other, color):
        closable.close()


def test_segmentation_goes_to_the_colour_image(sessions):  # noqa: F811
    """A depth stream of the same frame, bridged first: the label image is drawn over the colour image."""
    a, b = sessions
    depth_base, color_base, key = f"{CAMERA}/depth", f"{CAMERA}/color", f"{CAMERA}/nemo/instances"
    info = CameraInfo.pinhole(64, 48, 50.0, 50.0, 31.5, 23.5, frame_id="cam")
    depth = CameraPublisher(a, depth_base, encoding="16UC1", frame_id="cam", camera_info=info, codec=PNG)
    color = CameraPublisher(a, color_base, encoding="rgb8", frame_id="cam", camera_info=info)
    labels = CameraPublisher(a, key, encoding="mono16", frame_id="cam", camera_info=info, codec=PNG, labels=True)
    rr = FakeRerun()
    bridge = RerunBridge(b, rr, [depth_base, color_base], segmentations=[key])
    wait_until(lambda: depth.subscribed and color.subscribed and labels.subscribed)
    stamp = time.time_ns()
    depth.publish(sixteen_bit(), stamp_ns=stamp)
    color.publish(pattern(0), stamp_ns=stamp)
    wait_until(lambda: bridge.frames[depth_base] == 1 and bridge.frames[color_base] == 1)
    labels.publish(instances(), stamp_ns=stamp)
    wait_until(lambda: of(rr, "SegmentationImage") or bridge._error is not None)
    bridge.check()
    assert [e for e, *_ in of(rr, "SegmentationImage")] == [f"{color_base}/nemo/instances"]
    # id 0 (none) transparent, not tinting the image
    (entity, _, context, _), = of(rr, "AnnotationContext")
    assert entity == f"{color_base}/nemo/instances" and context["classes"] == [{"info": (0, "none", (0, 0, 0, 0))}]
    # the colour image's view before the depth image's, though bridged second
    assert list(without_ages(rr.blueprints[-1]))[1:] == [color_base, depth_base]
    for closable in (bridge, labels, color, depth):
        closable.close()


def test_real_rerun_takes_segmentation(sessions):  # noqa: F811
    rr = pytest.importorskip("rerun")
    a, b = sessions
    rr.init("xblick-test", strict=True)
    rr.memory_recording()
    color_base, key = f"{CAMERA}/color", f"{CAMERA}/sam3/instances"
    info = CameraInfo.pinhole(64, 48, 50.0, 50.0, 31.5, 23.5, frame_id="color_cam")
    color = CameraPublisher(a, color_base, encoding="rgb8", frame_id="color_cam", camera_info=info)
    labels = CameraPublisher(a, key, encoding="mono16", frame_id="color_cam", camera_info=info, codec=PNG)
    bridge = RerunBridge(b, rr, [color_base], segmentations=[key])
    wait_until(lambda: color.subscribed and labels.subscribed)
    color.publish(pattern(0))
    wait_until(lambda: len(bridge._calibration) == 1 or bridge._error is not None)
    time.sleep(0.3)
    labels.publish(instances())
    wait_until(lambda: bridge.frames[key] == 1 or bridge._error is not None)
    bridge.check()
    for closable in (bridge, labels, color):
        closable.close()


def test_camera_in_the_world(sessions):  # noqa: F811
    """A pose on <camera>/tf moves the camera's entity: the root of its static transforms (the depth camera) placed
    where the pose of the named frame (the colour camera, 32 mm beside it and turned) puts it; the scene z up."""
    a, b = sessions
    depth_base, color_base = f"{CAMERA}/depth", f"{CAMERA}/color"
    depth = CameraPublisher(a, depth_base, encoding="16UC1", frame_id="depth_cam", codec=PNG,
                            camera_info=CameraInfo.pinhole(64, 48, 50.0, 50.0, 31.5, 23.5, frame_id="depth_cam"))
    color = CameraPublisher(a, color_base, encoding="rgb8", frame_id="color_cam",
                            camera_info=CameraInfo.pinhole(64, 48, 50.0, 50.0, 31.5, 23.5, frame_id="color_cam"))
    transforms = StaticTransformPublisher(a, CAMERA, depth_to_color())
    world = TransformPublisher(a, CAMERA)
    rr = FakeRerun()
    bridge = RerunBridge(b, rr, [depth_base, color_base])
    wait_until(lambda: color.subscribed and depth.subscribed)
    stamp = time.time_ns()
    depth.publish(np.zeros((48, 64), np.uint16), stamp_ns=stamp)
    color.publish(pattern(0), stamp_ns=stamp)
    # the colour stream placed beside the depth camera: tf_static and its frame are known
    wait_until(lambda: any(e == color_base and kind == "Transform3D" and np.allclose(f["translation"], [0.032, 0, 0])
                           for e, kind, f, _ in rr.logged))
    world.publish([TransformStamped(Header(Time.from_ns(stamp), "world"), "color_cam",
                                    Transform(Vector3(1.0, 2.0, 3.0), Quaternion(0.0, 0.0, 0.0, 1.0)))])
    wait_until(lambda: any(e == CAMERA and kind == "Transform3D" for e, kind, *_ in rr.logged))
    bridge.check()
    entity, _, fields, logged_at = [x for x in rr.logged if x[0] == CAMERA and x[1] == "Transform3D"][0]
    assert logged_at == np.datetime64(stamp, "ns")
    # the colour camera is the depth camera's x + 32 mm, turned 90 deg about x: the depth camera is 32 mm back along
    # the world's x, turned back
    h = np.sqrt(0.5)
    turned = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])  # quaternion (h, 0, 0, h)
    assert np.isclose(h * h * 2, 1.0)
    assert np.allclose(fields["translation"], [1.0 - 0.032, 2.0, 3.0])
    assert np.allclose(fields["mat3x3"], turned.T)
    assert ("/", "ViewCoordinates", {"xyz": "RFU"}, None) in rr.logged
    # a pose that is not in the world
    world.publish([TransformStamped(Header(Time.from_ns(stamp), "base_link"), "color_cam", Transform())])
    wait_until(lambda: bridge._error is not None)
    with pytest.raises(ProtocolError, match="not in 'world'"):
        bridge.check()
    for closable in (bridge, world, transforms, color, depth):
        closable.close()


def test_save_survives_sigterm(tmp_path):
    """xblick rerun --save ended by SIGTERM (kill, a service manager) writes what it bridged, the world poses
    included: it ends as on Ctrl-C and flushes Rerun (it lost the camera's poses and the scene's axes before)."""
    import os
    import re
    import signal
    import subprocess
    import sys
    from pathlib import Path

    from test_camera import free_port

    from xblick.session import open_session
    pytest.importorskip("rerun")
    rerun_cli = Path(sys.executable).parent / "rerun"
    if not rerun_cli.exists():
        pytest.skip("no rerun command next to this Python")
    port = free_port()
    session = open_session(listen=[f"127.0.0.1:{port}"])
    info = CameraInfo.pinhole(64, 48, 50.0, 50.0, 31.5, 23.5, frame_id="cam")
    color = CameraPublisher(session, f"{CAMERA}/color", encoding="rgb8", frame_id="cam", camera_info=info)
    world = TransformPublisher(session, CAMERA)
    out = tmp_path / "saved.rrd"
    bridge = subprocess.Popen([sys.executable, "-m", "xblick.cli", "rerun", f"{CAMERA}/color", "--save", str(out),
                               "--report", "1", "--connect", f"127.0.0.1:{port}"],
                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        wait_until(lambda: color.subscribed, timeout=20.0)
        for k in range(20):
            stamp = time.time_ns()
            color.publish(pattern(k), stamp_ns=stamp)
            world.publish([TransformStamped(Header(Time.from_ns(stamp), "world"), "cam",
                                            Transform(Vector3(0.01 * k, 0.0, 1.5), Quaternion(0.0, 0.0, 0.0, 1.0)))])
            time.sleep(0.05)
        time.sleep(1.0)
        os.kill(bridge.pid, signal.SIGTERM)
        output, _ = bridge.communicate(timeout=20)
        assert bridge.returncode == 0, output
    finally:
        if bridge.poll() is None:
            bridge.kill()
        color.close()
        world.close()
        session.close()
    stats = subprocess.run([str(rerun_cli), "rrd", "stats", str(out)], capture_output=True, text=True, check=True)
    counts = dict(re.findall(r"^(/\S*): (\d+)$", stats.stdout, re.M))
    assert counts.get(f"/{CAMERA}") and counts.get("/") and counts.get(f"/{CAMERA}/color"), stats.stdout



def test_ages_at_send(sessions):  # noqa: F811
    """Per topic the age at send in ms at ages/<topic> (images, detections), on the capture timeline; the newest at
    ages_now; their views in the layout. ages=False: none."""
    a, b = sessions
    publisher = CameraPublisher(a, BASE, encoding="rgb8", frame_id="cam")
    boxes = DetectionPublisher(a, f"{CAMERA}/node/boxes", Detection2DArray)
    rr = FakeRerun()
    bridge = RerunBridge(b, rr, [BASE], detections=[f"{CAMERA}/node/boxes"])
    wait_until(lambda: publisher.subscribed)
    time.sleep(0.3)
    stamp = time.time_ns() - 25_000_000  # captured 25 ms before it is published
    publisher.publish(pattern(1), stamp_ns=stamp)
    boxes.publish(Detection2DArray(header=Header(Time.from_ns(stamp), "cam"), detections=[]))
    wait_until(lambda: {e for e, kind, *_ in rr.logged if kind == "Scalars"} >= {f"ages/{BASE}",
                                                                                f"ages/{CAMERA}/node/boxes"})
    bridge.check()
    for entity, _, fields, t in of(rr, "Scalars"):
        if entity.startswith("ages/"):
            assert 25 <= fields["values"][0] < 80 and t == np.datetime64(stamp, "ns"), (entity, fields)
    # ages_now: written at the first age, then at most once a second with each topic's newest. Which of the two came
    # first differs between hosts, so a second round after a second has to bring both into the text.
    time.sleep(1.1)
    stamp = time.time_ns() - 25_000_000
    publisher.publish(pattern(2), stamp_ns=stamp)
    boxes.publish(Detection2DArray(header=Header(Time.from_ns(stamp), "cam"), detections=[]))

    def newest_now():
        return ([f["text"] for e, _, f, _ in of(rr, "TextDocument") if e == "ages_now"] or [""])[-1]

    wait_until(lambda: f"| {BASE} |" in newest_now() and f"| {CAMERA}/node/boxes |" in newest_now())
    assert {"age at send ms", "age at receive ms", "rate Hz", "rates and ages now"} <= set(rr.blueprints[-1])
    bridge.close()

    rr = FakeRerun()
    bridge = RerunBridge(b, rr, [BASE], ages=False)
    time.sleep(0.3)
    publisher.publish(pattern(2))
    wait_until(lambda: of(rr, "EncodedImage"))
    time.sleep(0.2)
    assert not [e for e, *_ in rr.logged if e.startswith(("ages", "rates"))]
    bridge.close()
    boxes.close()
    publisher.close()


def test_rates_and_drops(sessions):  # noqa: F811
    """rates/<topic> in Hz (a 20 Hz publisher: within 10 %), ages_received/<topic>, and the table at ages_now with its
    labelled columns, its legend and the samples a second the publisher sent that did not arrive (seq gaps)."""
    a, b = sessions
    publisher = CameraPublisher(a, BASE, encoding="rgb8", frame_id="cam")
    rr = FakeRerun()
    bridge = RerunBridge(b, rr, [BASE])
    wait_until(lambda: publisher.subscribed)
    time.sleep(0.3)
    for k in range(40):  # 2 s at 20 Hz, every fifth frame two seqs on: 4 missing a second
        if k % 5 == 4:
            publisher.seq += 1
        publisher.publish(pattern(k % 3), stamp_ns=time.time_ns())
        time.sleep(0.05)
    time.sleep(0.2)
    bridge.check()
    rates = [f["values"][0] for e, _, f, _ in of(rr, "Scalars") if e == f"rates/{BASE}"]
    assert len(rates) > 30 and abs(sorted(rates[10:])[len(rates[10:]) // 2] - 20.0) < 2.0, rates
    received = [f["values"][0] for e, _, f, _ in of(rr, "Scalars") if e == f"ages_received/{BASE}"]
    assert received and all(0 <= ms < 100 for ms in received)
    text = [f["text"] for e, _, f, _ in of(rr, "TextDocument") if e == "ages_now"][-1]
    assert "| topic | rate Hz | age at send ms | age at receive ms | dropped/s |" in text
    row = next(line for line in text.splitlines() if line.startswith(f"| {BASE} |"))
    rate, send, receive, dropped = (c.strip() for c in row.strip("|").split("|")[1:])
    assert abs(float(rate) - 20.0) < 2.0 and float(send) >= 0 and float(receive) >= 0 and int(dropped) in (3, 4, 5)
    assert "age at send: capture to sent, on the sender's clock" in text and "synchronised clocks" in text
    assert "gaps in the attachment's seq" in text
    bridge.close()
    publisher.close()


def test_the_viewer_next_to_python_without_path(monkeypatch):
    """The viewer is rerun-sdk's next to this Python even without PATH (a venv not activated)."""
    pytest.importorskip("rerun")  # the extra xblick[rerun] brings the viewer; CI installs without extras
    monkeypatch.setenv("PATH", "")
    found = rerun_bridge.viewer_executable()
    assert found == str(Path(sys.executable).parent / "rerun")


def test_without_a_viewer_the_command_says_where_it_looked(monkeypatch, tmp_path):
    """Neither next to this Python nor on PATH: the command names both places."""
    monkeypatch.setenv("PATH", "")
    monkeypatch.setattr(sys, "executable", str(tmp_path / "python"))
    with pytest.raises(SystemExit, match=f"neither {tmp_path}/rerun .* nor 'rerun' on PATH"):
        rerun_bridge.viewer_executable()
