"""A node's drawn results: the colour frame with what the node found drawn on it, published only when asked for.

The convention (docs/guides/writing-nodes.md, "Drawn results"): a node offers ``--overlay`` (``add_argument``), off by
default and applied at the start (a read-only parameter, no live one). With it, the node publishes the image stream
``<camera>/<node>/overlay``: rgb8, the colour frame's size, stamp, frame id and camera_info (it is the same camera
image). Without it, no publisher exists: no topic, no info record. The node publishes its results first and then hands
the frame to ``Overlay.put``; a thread of its own draws (Pillow) and publishes, keeping one frame (a newer frame
replaces the one waiting), and draws only while somebody subscribes.

    overlay = Overlay(session, camera, "yolox", draw=lambda rgb, found: draw_boxes(rgb, found.boxes, texts, colours))
    ...
    results.publish(...)                                   # the results first
    overlay.put(frame.stamp_ns, frame.header.frame_id, rgb, found, camera_info=color.camera_info)
    overlay.check()                                        # the drawing thread's error, raised here
    ...
    overlay.close()

The drawing helpers take and return an (H, W, 3) uint8 RGB array: ``draw_boxes`` (boxes with labels),
``draw_masks`` (instances of a label image), ``draw_points`` (keypoints and a skeleton), ``draw_tracks`` (point
tracks), and ``depth_colours`` (depth in mm as colours: the map ``xblick view`` uses as well).
"""

from __future__ import annotations

import colorsys
import threading

import numpy as np

from .camera import CameraPublisher

OVERLAY = "overlay"  # the stream's last part: <camera>/<node>/overlay
DEPTH_NEAR_MM, DEPTH_FAR_MM = 300, 5000  # the depth colours' fixed range (depth_colours)


def add_argument(parser):
    """``--overlay``: the convention's option (off by default, applies at the start)."""
    parser.add_argument("--overlay", action="store_true",
                        help="also publish <camera>/<node>/overlay: the colour image with the results drawn on it "
                             "(rgb8; drawn only while somebody subscribes). Off by default; applies at the start")


class Overlay:
    """The stream ``<camera>/<node>/overlay`` and the thread that draws it (module docstring). ``draw(rgb, results)``
    returns the drawn (H, W, 3) uint8 RGB image. ``replaced``: frames that a newer one replaced before they were
    drawn; ``published``: overlays sent."""

    def __init__(self, session, camera: str, node: str, draw, source: str = ""):
        self.publisher = CameraPublisher(session, f"{camera}/{node}/{OVERLAY}", encoding="rgb8", frame_id="",
                                         source=source)
        self.draw = draw
        self.published = 0
        self.replaced = 0
        self._waiting = None
        self._error: BaseException | None = None
        self._stop = False
        self._cond = threading.Condition()
        self._thread = threading.Thread(target=self._run, name=f"overlay {camera}/{node}", daemon=True)
        self._thread.start()

    @property
    def base(self) -> str:
        return self.publisher.base

    def put(self, stamp_ns: int, frame_id: str, rgb: np.ndarray, results, camera_info=None):
        """The colour frame (``rgb``, its stamp, frame id and camera_info) and the node's results for it; never
        waits."""
        with self._cond:
            if self._waiting is not None:
                self.replaced += 1
            self._waiting = (stamp_ns, frame_id, rgb, results, camera_info)
            self._cond.notify()

    def _run(self):
        try:
            while True:
                with self._cond:
                    while self._waiting is None and not self._stop:
                        self._cond.wait()
                    if self._stop:
                        return
                    (stamp_ns, frame_id, rgb, results, camera_info), self._waiting = self._waiting, None
                if not self.publisher.subscribed:
                    continue
                image = self.draw(rgb, results)
                self.publisher.frame_id = frame_id
                self.publisher.camera_info = camera_info
                self.publisher.publish(image, stamp_ns=stamp_ns)
                self.published += 1
        except BaseException as error:  # noqa: BLE001 - handed to the node's loop by check()
            self._error = error

    def check(self):
        """Raises the error that ended the drawing thread, in the caller's thread."""
        if self._error is not None:
            raise RuntimeError(f"the overlay failed: {type(self._error).__name__}: {self._error}") from self._error

    def close(self):
        with self._cond:
            self._stop = True
            self._cond.notify()
        self._thread.join(timeout=5.0)
        self.publisher.close()


# ------------------------------------------------------------------------------------------------- drawing


def colour(index: int) -> tuple[int, int, int]:
    """A fixed colour per class or instance: hues spread by the golden ratio, so that neighbouring indices differ."""
    r, g, b = colorsys.hsv_to_rgb((index * 0.618033988749895) % 1.0, 0.85, 1.0)
    return round(r * 255), round(g * 255), round(b * 255)


def _font(h: int, w: int):
    from PIL import ImageFont

    return ImageFont.load_default(size=max(12, round(min(w, h) / 32)))


def _width(h: int, w: int) -> int:
    return max(2, round(min(w, h) / 240))  # 2 pixels at 480 lines


def draw_boxes(rgb: np.ndarray, boxes, texts, colours) -> np.ndarray:
    """A copy of the frame with each box (x1, y1, x2, y2 in pixels) outlined in its colour and its text (e.g.
    ``person 0.87``) in black on that colour above it (inside it at the top edge); the first box drawn last, so on
    top (sort the boxes best first)."""
    from PIL import Image, ImageDraw

    image = Image.fromarray(np.ascontiguousarray(rgb))
    pen = ImageDraw.Draw(image)
    h, w = rgb.shape[:2]
    width, font = _width(h, w), _font(h, w)
    for i in reversed(range(len(boxes))):
        x1, y1, x2, y2 = (float(v) for v in boxes[i])
        fill = tuple(colours[i])
        pen.rectangle((x1, y1, x2, y2), outline=fill, width=width)
        if texts[i]:
            left, top, right, bottom = pen.textbbox((0, 0), texts[i], font=font)
            tw, th = right - left, bottom - top
            ty = y1 - th - 2 * width if y1 - th - 2 * width >= 0 else y1
            pen.rectangle((x1, ty, x1 + tw + 2 * width, ty + th + 2 * width), fill=fill)
            pen.text((x1 + width - left, ty + width - top), texts[i], fill=(0, 0, 0), font=font)
    return np.asarray(image)


def draw_masks(rgb: np.ndarray, labels: np.ndarray, alpha: float = 0.5) -> np.ndarray:
    """A copy of the frame with each instance of the label image (H, W ints, 0 none) blended in ``colour(id)``."""
    out = rgb.astype(np.float32)
    for index in np.unique(labels):
        if index == 0:
            continue
        mask = labels == index
        out[mask] = (1 - alpha) * out[mask] + alpha * np.array(colour(int(index)), np.float32)
    return np.clip(out + 0.5, 0, 255).astype(np.uint8)


def draw_points(rgb: np.ndarray, points, edges=(), colour_: tuple[int, int, int] = (255, 255, 0),
                valid=None) -> np.ndarray:
    """A copy of the frame with keypoints (N, 2: x, y in pixels) as dots and the skeleton's ``edges`` ((i, j) pairs)
    as lines; ``valid``: N booleans, the invalid ones left out."""
    from PIL import Image, ImageDraw

    image = Image.fromarray(np.ascontiguousarray(rgb))
    pen = ImageDraw.Draw(image)
    h, w = rgb.shape[:2]
    width = _width(h, w)
    ok = np.ones(len(points), bool) if valid is None else np.asarray(valid, bool)
    for i, j in edges:
        if ok[i] and ok[j]:
            pen.line((*map(float, points[i]), *map(float, points[j])), fill=colour_, width=width)
    r = 2 * width
    for (x, y), shown in zip(points, ok):
        if shown:
            pen.ellipse((float(x) - r, float(y) - r, float(x) + r, float(y) + r), fill=colour_)
    return np.asarray(image)


def draw_tracks(rgb: np.ndarray, tracks) -> np.ndarray:
    """A copy of the frame with each track (a list of (x, y) pixels, oldest first) as a line in ``colour(i)`` and a
    dot at its newest point."""
    from PIL import Image, ImageDraw

    image = Image.fromarray(np.ascontiguousarray(rgb))
    pen = ImageDraw.Draw(image)
    h, w = rgb.shape[:2]
    width = _width(h, w)
    for i, track in enumerate(tracks):
        if len(track) == 0:
            continue
        fill = colour(i)
        if len(track) > 1:
            pen.line([tuple(map(float, p)) for p in track], fill=fill, width=width)
        x, y = map(float, track[-1])
        pen.ellipse((x - 2 * width, y - 2 * width, x + 2 * width, y + 2 * width), fill=fill)
    return np.asarray(image)


def depth_colours(depth_mm: np.ndarray, near_mm: float = DEPTH_NEAR_MM, far_mm: float = DEPTH_FAR_MM) -> np.ndarray:
    """Depth (H, W, mm; 0 none) as an (H, W, 3) uint8 RGB image: hue from red at ``near_mm`` through yellow and
    green to blue at ``far_mm`` (HSV hue 0 to 240 degrees, full saturation and value), clipped outside that range;
    no depth (0) black. The range is fixed: the same distance has the same colour in every frame."""
    d = depth_mm.astype(np.float32)
    x = np.clip((d - near_mm) / max(far_mm - near_mm, 1e-6), 0.0, 1.0)
    h6 = x * 4.0  # hue in sixths of the circle: 0 (red) to 4 (blue)
    i = np.floor(h6).astype(np.int32)
    f = h6 - i
    one, rise, fall, zero = np.ones_like(x), f, 1.0 - f, np.zeros_like(x)
    # HSV to RGB with S = V = 1 for the sectors 0..4 (red-yellow, yellow-green, green-cyan, cyan-blue, blue)
    r = np.choose(np.clip(i, 0, 4), [one, fall, zero, zero, zero])
    g = np.choose(np.clip(i, 0, 4), [rise, one, one, fall, zero])
    b = np.choose(np.clip(i, 0, 4), [zero, zero, rise, one, one])
    out = (np.stack([r, g, b], -1) * 255 + 0.5).astype(np.uint8)
    out[depth_mm == 0] = 0
    return out
