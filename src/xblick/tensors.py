"""Frames as PyTorch tensors on a device, for the nodes (torch is theirs: imported here only when called).

A frame from a CUDA ring on that device is taken as it is (no copy), a host frame (shared memory, the network) is
uploaded once. The results are the cheapest form: a view where one suffices (an RGBA frame's first three channels, a
grey frame repeated), so a caller that needs contiguous memory says ``.contiguous()``.

    rgb = tensors.rgb(frame, device)              # (H, W, 3) uint8 RGB
    depth = tensors.depth_metres(frame, device)   # (H, W) float32 metres, 0 where there is no depth
"""

from __future__ import annotations

COLOR_ENCODINGS = ("rgb8", "rgba8", "bgr8", "bgra8", "mono8")
DEPTH_ENCODINGS = ("16UC1", "32FC1")  # millimetres (REP 118), metres


def tensor(frame, device):
    """The frame's image as it is (its dtype and channels) on ``device``."""
    import torch

    return torch.as_tensor(frame.image, device=device)


def rgb(frame, device):
    """A colour frame as an (H, W, 3) uint8 RGB tensor on ``device``; a grey (mono8) frame in all three channels."""
    if frame.encoding not in COLOR_ENCODINGS:
        raise ValueError(f"a colour frame of encoding {frame.encoding!r}; one of {', '.join(COLOR_ENCODINGS)} expected")
    image = tensor(frame, device)
    if frame.encoding == "mono8":
        return (image if image.ndim == 3 else image[..., None]).expand(*image.shape[:2], 3)
    image = image[..., :3]
    return image.flip(-1) if frame.encoding.startswith("bgr") else image


def depth(frame, device):
    """(depth tensor as it came, its units per metre) of a depth frame: 16UC1 millimetres (1000), 32FC1 metres with
    NaN as 0 (1)."""
    if frame.encoding == "16UC1":
        return tensor(frame, device), 1000.0
    if frame.encoding == "32FC1":
        import torch

        return torch.nan_to_num(tensor(frame, device), nan=0.0), 1.0
    raise ValueError(f"a depth frame of encoding {frame.encoding!r}; one of {', '.join(DEPTH_ENCODINGS)} expected")


def depth_metres(frame, device):
    """A depth frame as an (H, W) float32 tensor of metres on ``device``, 0 where there is no depth."""
    values, per_metre = depth(frame, device)
    return values.float() / per_metre if per_metre != 1.0 else values.float()
