"""A camera's colour and depth (and, optionally, a segmenter's instances and detections) for nodes that work on RGB-D
frames: which streams they read (``CameraSpec``), the streams read in threads of their own and paired by stamp
(``RGBDInputs``), and one frame of them on the GPU, at the depth's resolution, undistorted into the pinhole camera of
the colour CameraInfo's P (``FramePreparer`` -> ``RGBDFrame``; ``Undistorter`` as ROS's image_proc rectifies).
Views of an object for nodes that build a model of it from several (``collect_views``: the camera moving, views at
least an angle apart seen from the object, the object's mask from the camera's segmenter).

Used by xblick-foundationpose, its mesh makers and xblick-nemo. ``FramePreparer`` and ``Undistorter`` need
torch and OpenCV, which Durchblick itself does not install: the nodes using them bring both.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass

import numpy as np

from .camera import ImageSubscriber
from .detections import DetectionSubscriber
from . import tensors
from .history import FrameHistory, PartnerHistory, StampedHistory

KEYS = ("color", "depth", "segmenter")


@dataclass(frozen=True)
class CameraSpec:
    """A camera and the streams below it: ``<camera>/<color>``, ``<camera>/<depth>`` (16UC1 mm in the colour camera's
    geometry) and, if ``segmenter`` is set, ``<camera>/<segmenter>/{instances,detections_2d}`` (e.g. an xblick-sam3
    instance on the colour stream)."""

    camera: str
    color: str = "color"
    depth: str = "depth"
    segmenter: str | None = None

    @classmethod
    def parse(cls, text: str, segmenter: str | None = None) -> CameraSpec:
        """``CAMERA[:color=NAME][:depth=NAME][:segmenter=NAME]``, e.g. ``robot/rc_visard:color=left:depth=ffs/depth``
        (colons: some process managers split a program's arguments at commas). ``segmenter``: the node's
        default (None: no segmenter unless the text names one)."""
        camera, *options = text.strip().split(":")
        if not camera:
            raise ValueError(f"{text!r}: CAMERA[:color=NAME][:depth=NAME][:segmenter=NAME] expected")
        values = {}
        for option in options:
            key, sep, value = option.partition("=")
            if not sep or key not in KEYS or not value:
                raise ValueError(f"{text!r}: {option!r} is not one of {', '.join(k + '=NAME' for k in KEYS)}")
            if key in values:
                raise ValueError(f"{text!r}: {key} given twice")
            values[key] = value
        values.setdefault("segmenter", segmenter)
        return cls(camera, **values)

    @property
    def keys(self) -> dict:
        keys = {"color": f"{self.camera}/{self.color}", "depth": f"{self.camera}/{self.depth}"}
        if self.segmenter is not None:
            keys |= {"instances": f"{self.camera}/{self.segmenter}/instances",
                     "detections": f"{self.camera}/{self.segmenter}/detections_2d"}
        return keys

    def __str__(self) -> str:
        defaults = CameraSpec("")
        return self.camera + "".join(f":{k}={getattr(self, k)}" for k in KEYS
                                     if getattr(self, k) is not None and getattr(self, k) != getattr(defaults, k))


def scaled_k(info, height: int, width: int) -> np.ndarray:
    """A CameraInfo's K for its image resized to ``width`` x ``height`` (pixel centres: c' = (c + 0.5) s - 0.5)."""
    return _scaled(np.asarray(info.k, dtype=np.float64).reshape(3, 3), info, height, width)


def _scaled(m: np.ndarray, info, height: int, width: int) -> np.ndarray:
    m = m.copy()
    sx, sy = width / info.width, height / info.height
    m[0, 0], m[0, 2] = m[0, 0] * sx, (m[0, 2] + 0.5) * sx - 0.5
    m[1, 1], m[1, 2] = m[1, 1] * sy, (m[1, 2] + 0.5) * sy - 0.5
    return m


def check_same_camera(color_info, depth_info):
    """Colour and depth have to be one camera up to scale (the depth in the colour camera's geometry: the same K up to
    scale, the same distortion)."""
    k = scaled_k(color_info, depth_info.height, depth_info.width)
    d = np.asarray(depth_info.k, dtype=np.float64).reshape(3, 3)
    same_distortion = (color_info.distortion_model == depth_info.distortion_model
                       and np.allclose(np.asarray(color_info.d), np.asarray(depth_info.d), atol=1e-6))
    if not np.allclose(k, d, rtol=1e-3, atol=0.5) or not same_distortion:
        raise ValueError(f"colour ({color_info.width} x {color_info.height}, K {np.round(color_info.k, 2).tolist()}, "
                         f"{color_info.distortion_model or 'no distortion'}) and depth ({depth_info.width} x "
                         f"{depth_info.height}, K {np.round(depth_info.k, 2).tolist()}, "
                         f"{depth_info.distortion_model or 'no distortion'}) are not one camera up to scale: the depth "
                         "has to be in the colour camera's geometry (Azure Kinect: depth_to_color)")


class Undistorter:
    """Remaps images of a camera (its CameraInfo, at ``height`` x ``width``) into the pinhole camera of its P (or of K
    if P is unset): ``K`` that camera, ``apply`` the remap (bilinear for colour, nearest for depth and labels). Without
    distortion and with P = K nothing is remapped (``identity``). Distortion models: plumb_bob, rational_polynomial
    (OpenCV's), equidistant (fisheye)."""

    def __init__(self, info, height: int, width: int, device):
        import cv2
        import torch

        k = scaled_k(info, height, width)
        d = np.asarray(info.d, dtype=np.float64)
        p = np.asarray(info.p, dtype=np.float64).reshape(3, 4)
        new = _scaled(p[:, :3], info, height, width) if p[0, 0] > 0 else k
        r = np.asarray(info.r, dtype=np.float64).reshape(3, 3)
        if not r.any():
            r = np.eye(3)
        self.K = new
        distorted = d.size > 0 and np.any(d != 0)
        self.identity = not distorted and np.allclose(new, k) and np.allclose(r, np.eye(3))
        if self.identity:
            return
        model = info.distortion_model
        if model in ("plumb_bob", "rational_polynomial", ""):
            mx, my = cv2.initUndistortRectifyMap(k, d, r, new, (width, height), cv2.CV_32FC1)
        elif model == "equidistant":
            mx, my = cv2.fisheye.initUndistortRectifyMap(k, d[:4], r, new, (width, height), cv2.CV_32FC1)
        else:
            raise ValueError(f"distortion model {model!r}: plumb_bob, rational_polynomial or equidistant")
        # grid_sample's coordinates: -1 and 1 the outer edges of the corner pixels (align_corners=False)
        grid = np.stack([(mx + 0.5) / width * 2 - 1, (my + 0.5) / height * 2 - 1], axis=-1)
        self._grid = torch.as_tensor(grid, dtype=torch.float32, device=device)[None]

    def apply(self, image, mode: str):
        """``image`` (H, W) or (H, W, C) torch tensor -> the same in the pinhole camera; 0 outside the image."""
        import torch.nn.functional as F

        if self.identity:
            return image
        x = image[None, None] if image.ndim == 2 else image.permute(2, 0, 1)[None]
        out = F.grid_sample(x.float(), self._grid, mode=mode, padding_mode="zeros", align_corners=False)[0]
        out = out[0] if image.ndim == 2 else out.permute(1, 2, 0)
        return out.round().to(image.dtype) if not image.is_floating_point() else out


class Distorter:
    """The way back from an Undistorter's pinhole camera (``K``, ``height`` x ``width``: an RGBDFrame's) into a camera's
    own images (its CameraInfo, at its own size): for a node's per-pixel output on the frames (labels, masks) that has
    to fit the camera's own colour image, e.g. a segmenter's instances. Each of the camera's pixels takes the pinhole
    image's pixel it shows (nearest; outside the pinhole image: 0)."""

    def __init__(self, info, K: np.ndarray, height: int, width: int, device):
        import cv2
        import torch

        k = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
        d = np.asarray(info.d, dtype=np.float64)
        r = np.asarray(info.r, dtype=np.float64).reshape(3, 3)
        if not r.any():
            r = np.eye(3)
        v, u = np.mgrid[0:info.height, 0:info.width].astype(np.float64)
        pixels = np.stack([u, v], -1).reshape(-1, 1, 2)
        model = info.distortion_model
        if model in ("plumb_bob", "rational_polynomial", ""):
            to = cv2.undistortPoints(pixels, k, d, R=r, P=K)
        elif model == "equidistant":
            to = cv2.fisheye.undistortPoints(pixels, k, d[:4], R=r, P=K)
        else:
            raise ValueError(f"distortion model {model!r}: plumb_bob, rational_polynomial or equidistant")
        to = np.round(to.reshape(info.height, info.width, 2)).astype(np.int64)
        inside = (to[..., 0] >= 0) & (to[..., 0] < width) & (to[..., 1] >= 0) & (to[..., 1] < height)
        self.size = (info.height, info.width)
        self._index = torch.as_tensor(np.where(inside, to[..., 1] * width + to[..., 0], 0), device=device)
        self._inside = torch.as_tensor(inside, device=device)

    def apply(self, image):
        """``image`` (height, width) torch, the pinhole camera's -> (the camera's height, width), 0 outside."""
        import torch

        out = image.reshape(-1)[self._index]
        return torch.where(self._inside, out, torch.zeros((), dtype=out.dtype, device=out.device))


class RGBDInputs:
    """A camera's streams (colour, depth; with a segmenter its instances and detections), paired by stamp: the colour
    stream is the primary one, taken frame by frame (FrameHistory); depth and instances are its partners
    (PartnerHistory), their frames read only when a colour frame is paired with them: the depth nearest within
    ``max_offset_ns`` (None, the default: half the depth's frame period; 0: the same stamp), the instances of the
    same stamp. ``max_hz`` (a node's
    ``--max-hz``): at most so many colour frames a second, the newest (the others not read, nor their partners'); a
    pair is never lost to it. The detections are small and all kept. ``take`` reads a frame's streams together: call
    it when you take a frame (``newest_complete``), before working on it (FramePreparer does)."""

    def __init__(self, session, spec: CameraSpec, history: int, local: str, wake: threading.Event,
                 max_hz: float | None = None, max_offset_ns: int | None = None, device=None):
        self.spec = spec
        keys = spec.keys
        self.readers = {"color": FrameHistory(ImageSubscriber(session, keys["color"], transport="auto",
                                                              camera_info=True, local=local, max_hz=max_hz,
                                                              device=device),
                                              history, wake.set)}
        # the segmenter's instances are stamped as the colour frame they segment: equal stamps only (with an offset, a
        # frame whose own mask is missing would get a neighbour's mask, which its detections do not describe)
        offsets = {"depth": max_offset_ns, "instances": 0}
        self.readers.update({k: PartnerHistory(ImageSubscriber(session, keys[k], transport="auto", camera_info=True,
                                                               local=local, device=device), history, offsets[k],
                                               wake.set)
                             for k in ("depth", "instances") if k in keys})
        self.histories = dict(self.readers)
        self.detections = None
        if "detections" in keys:
            self.histories["detections"] = StampedHistory(history, wake.set)
            self.detections = DetectionSubscriber(
                session, keys["detections"], lambda m: self.histories["detections"].put(m.header.stamp.to_ns(), m))

    def newest_complete(self) -> int | None:
        """The newest colour stamp all of its partners have (depth and instances within max_offset_ns, detections
        exactly), none of them decoded for it."""
        others = [h for k, h in self.histories.items() if k != "color"]
        for stamp in sorted(self.histories["color"].stamps(), reverse=True):
            if all(h.has(stamp) for h in others):
                return stamp
        return None

    def take(self, stamp: int) -> dict | None:
        """The frames of colour stamp ``stamp`` by stream (color, depth, instances, detections; those the camera
        has), read now, all together; None if one of them is missing or was overwritten in its ring (the partner's
        ``lost``)."""
        frames = {k: h.at(stamp) for k, h in self.histories.items()}
        return None if any(f is None for f in frames.values()) else frames

    def lost(self) -> int:
        """The partner frames overwritten in their ring before they were read (PartnerHistory.lost), all streams."""
        return sum(r.lost for r in self.readers.values() if isinstance(r, PartnerHistory))

    def waiting(self) -> str:
        """What keeps the camera's frames from coming complete, for a node's log: each stream's state, and whether
        they share a stamp (a segmenter of another camera never does)."""
        keys = self.spec.keys
        parts = [f"{keys[k]}: {r.subscriber.describe()}" for k, r in self.readers.items()]
        if self.detections is not None:
            parts.append(f"{keys['detections']}: {self.detections.describe()}")
        if all(h.stamps() for h in self.histories.values()) and self.newest_complete() is None:
            parts.append("all of them come, but no colour frame has its partners (stamps too far apart?)")
        return "; ".join(parts)

    def check(self):
        for reader in self.readers.values():
            reader.check()
        if self.detections is not None:
            self.detections.check()

    def close(self):
        for reader in self.readers.values():
            reader.close()
        if self.detections is not None:
            self.detections.close()


@dataclass
class RGBDFrame:
    """One frame on the GPU, at the depth's resolution, in the pinhole camera ``K``."""

    rgb: object  # (H, W, 3) uint8
    depth: object  # (H, W) float32 [m]
    ids: object  # (H, W) int32: the segmenter's instance ids, 0 none; None without a segmenter
    K: np.ndarray
    frame_id: str
    detections: object  # the segmenter's Detection2DArray of the frame; None without a segmenter


class FramePreparer:
    """Turns a stamp of a camera's RGBDInputs into an RGBDFrame; keeps the undistortion of the last camera models."""

    def __init__(self, device):
        self.device = device
        self.undistorter = None
        self.model_key = None

    def __call__(self, inputs: RGBDInputs, stamp: int) -> RGBDFrame | None:
        """The frame of ``stamp`` (all of the camera's streams have it); None until both camera models came, and for
        a frame whose size is not its stream's newest camera model (a camera that came back with another size: its
        new model or its new frames came first). Raises ValueError when the streams do not fit (encodings, colour and
        depth not one camera)."""
        import torch
        import torch.nn.functional as F

        device = self.device
        frames = inputs.take(stamp)  # all of the frame's streams first, before any work on them
        if frames is None:
            return None
        color, depth_frame = frames["color"], frames["depth"]
        color_info = inputs.readers["color"].subscriber.camera_info
        depth_info = inputs.readers["depth"].subscriber.camera_info
        if color_info is None or depth_info is None:
            return None
        height, width = depth_frame.image.shape[:2]
        if (depth_info.height, depth_info.width) != (height, width) or \
                (color_info.height, color_info.width) != tuple(color.image.shape[:2]):
            return None
        key = (tuple(color_info.k), tuple(color_info.d), tuple(color_info.p), color_info.width, color_info.height,
               tuple(depth_info.k), tuple(depth_info.d), height, width)
        if self.model_key != key:
            check_same_camera(color_info, depth_info)
            self.undistorter = Undistorter(color_info, height, width, device)
            self.model_key = key
        # from shared memory uploaded, from a CUDA ring on its GPU as it is
        depth = tensors.depth_metres(depth_frame, device)
        rgb = tensors.rgb(color, device).contiguous()
        if rgb.shape[:2] != (height, width):
            rgb = F.interpolate(rgb.permute(2, 0, 1)[None].float(), size=(height, width), mode="area")[0]
            rgb = rgb.permute(1, 2, 0).round().to(torch.uint8)
        u = self.undistorter
        ids = detections = None
        if "instances" in frames:
            instances = frames["instances"]
            if instances.encoding != "mono16":
                raise ValueError(f"instances {instances.encoding}: mono16 expected")
            ids = tensors.tensor(instances, device).to(torch.int32)
            if ids.shape != (height, width):
                ids = F.interpolate(ids[None, None].float(), size=(height, width), mode="nearest")[0, 0].int()
            ids = u.apply(ids, "nearest")
            detections = frames["detections"]
        return RGBDFrame(u.apply(rgb, "bilinear"), u.apply(depth, "nearest"), ids, u.K,
                         color_info.header.frame_id or color.header.frame_id, detections)


@dataclass
class View:
    """One view of an object: the frame (an RGBDFrame), the object's mask in it (torch bool, the frame's size), its
    stamp, the segmenter's instance id of the object and the camera's pose in the world (None for a single view)."""

    frame: RGBDFrame
    mask: object
    stamp: int
    instance: str
    cam_in_world: np.ndarray | None = None


def segmenter_mask(frame: RGBDFrame, instance: str | None = None, label: str | None = None):
    """The mask (torch bool) of the segmenter's instance ``instance``, or of its largest instance labelled ``label``,
    and that instance's id; ValueError if there is none (or the camera has no segmenter)."""
    if frame.detections is None or frame.ids is None:
        raise ValueError("the camera has no segmenter (its spec's :segmenter=NAME)")
    if (instance is None) == (label is None):
        raise ValueError("an instance id or a label (one of them)")
    detections = frame.detections.detections
    if instance is not None:
        if not any(d.id == str(instance) for d in detections):
            raise ValueError(f"no instance {instance} in the frame; there are "
                             f"{[(d.id, d.results[0].hypothesis.class_id if d.results else '') for d in detections]}")
        return frame.ids == int(instance), str(instance)
    candidates = [d for d in detections if d.results and d.results[0].hypothesis.class_id == label]
    if not candidates:
        raise ValueError(f"no instance labelled {label!r} in the frame; labels: "
                         f"{sorted({d.results[0].hypothesis.class_id for d in detections if d.results})}")
    best = max(candidates, key=lambda d: int((frame.ids == int(d.id)).sum()))
    return frame.ids == int(best.id), best.id


def object_centre(frame: RGBDFrame, mask) -> np.ndarray | None:
    """The object's centre in the camera (the median of its mask's points with depth); None without depth."""
    import torch

    vs, us = torch.nonzero(mask & (frame.depth > 0.001), as_tuple=True)
    if len(us) == 0:
        return None
    z = frame.depth[vs, us]
    K = frame.K
    pts = torch.stack([(us - K[0, 2]) * z / K[0, 0], (vs - K[1, 2]) * z / K[1, 1], z], dim=1)
    return pts.median(dim=0).values.cpu().numpy().astype(np.float64)


def view_direction(cam_in_world: np.ndarray, centre_in_cam: np.ndarray) -> np.ndarray:
    """The unit direction from the object's centre to the camera, in the world."""
    centre = cam_in_world[:3, :3] @ centre_in_cam + cam_in_world[:3, 3]
    d = cam_in_world[:3, 3] - centre
    return d / np.linalg.norm(d)


def far_enough(direction: np.ndarray, taken: list[np.ndarray], min_angle_deg: float) -> bool:
    """Whether ``direction`` is at least ``min_angle_deg`` from every direction taken."""
    cos = np.cos(np.radians(min_angle_deg))
    return all(float(direction @ t) <= cos for t in taken)


def collect_views(session, spec: CameraSpec, n_views: int, device, *, instance: str | None = None,
                  label: str | None = None, min_angle: float = 15.0, timeout: float = 30.0, min_pixels: int = 400,
                  local: str = "auto", log=None) -> list[View]:
    """The next ``n_views`` views of an object from a camera with a segmenter (its spec's ``:segmenter=NAME``): the
    object is the segmenter's instance ``instance``, or its largest instance labelled ``label`` in the first view and
    that instance (its tracking id) after; a view is kept when, seen from the object (its centre from the depth), the
    camera (its pose from ``<camera>/tf``) is at least ``min_angle`` degrees from every view kept, and its mask has at
    least ``min_pixels``. Move the camera or the object meanwhile. ValueError if the camera has no segmenter, or if the
    views do not come within ``timeout`` seconds (the message says why: frames without the object are waited out). ``log``: a function of a line, per view kept."""
    from .poses import CameraPoses

    if spec.segmenter is None:
        raise ValueError(f"{spec}: no segmenter (its spec's :segmenter=NAME) for the views' masks")
    wake = threading.Event()
    inputs = RGBDInputs(session, spec, 30, local, wake, device=device)
    prepare = FramePreparer(device)
    poses = CameraPoses(session, spec.camera) if n_views > 1 else None
    views: list[View] = []
    directions: list[np.ndarray] = []
    last, seen, without_pose, missing = None, 0, 0, None
    try:
        end = time.monotonic() + timeout
        while len(views) < n_views and time.monotonic() < end:
            wake.wait(0.05)
            wake.clear()
            inputs.check()
            stamp = inputs.newest_complete()
            if stamp is None or stamp == last:
                continue
            last = stamp
            frame = prepare(inputs, stamp)
            if frame is None:
                continue
            try:
                mask, used = segmenter_mask(frame, views[0].instance if views else instance,
                                            label if not views else None)
            except ValueError as error:
                missing = error  # the object not in this frame (a segmenter misses it now and then): the next one
                continue
            seen += 1
            if int(mask.sum()) < min_pixels:
                continue
            if n_views == 1:
                return [View(frame, mask, stamp, used)]
            pose = poses.wait(stamp, 0.2)
            if pose is None:
                without_pose += 1
                continue
            centre = object_centre(frame, mask)
            if centre is None:
                continue
            direction = view_direction(pose, centre)
            if far_enough(direction, directions, min_angle):
                views.append(View(frame, mask, stamp, used, pose))
                directions.append(direction)
                if log is not None:
                    log(f"view {len(views)}/{n_views} of instance {used} ({int(mask.sum())} px)")
        if len(views) < n_views:
            raise ValueError(
                f"{len(views)} of {n_views} views {min_angle} deg apart within {timeout} s ({seen} frames with the "
                f"object, {without_pose} without a pose on {spec.camera}/tf); move the camera, or fewer views"
                if views or seen else f"no frame of {spec} with the object within {timeout} s"
                + (f" (the last: {missing})" if missing is not None else ""))
        return views
    finally:
        inputs.close()
        if poses is not None:
            poses.close()
