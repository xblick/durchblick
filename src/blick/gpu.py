"""Images in device memory, exchanged through ``__cuda_array_interface__`` (version 3), which PyTorch
(``torch.as_tensor(image, device="cuda")``), Warp, CuPy and Numba read without a copy. Durchblick itself depends on
none of them."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from . import _core
from .encodings import EncodingError, info


@dataclass(frozen=True)
class CudaArray:
    """What publish() needs of a device array: pointer, shape, dtype, and the stream its data is ready on."""

    ptr: int
    shape: tuple
    dtype: np.dtype
    # CUstream handle (0: the legacy default stream); None: not known (PyTorch's interface names none, and a tensor
    # may come from any of its streams), so the data is complete once all of the process's work on the device is
    stream: int | None

    @property
    def nbytes(self) -> int:
        return int(np.prod(self.shape)) * self.dtype.itemsize


def is_cuda_array(obj) -> bool:
    return hasattr(obj, "__cuda_array_interface__")


def stream_handle(stream) -> int:
    """A CUstream handle from an int or a stream object with one (``cuda_stream``: torch.cuda.Stream, CuPy, Warp)."""
    handle = getattr(stream, "cuda_stream", stream)
    if not isinstance(handle, int | np.integer):
        raise TypeError(f"stream {stream!r}: a CUstream handle (int) or an object with cuda_stream expected")
    return int(handle)


def cuda_array(obj, stream=None) -> CudaArray:
    """The device array behind ``obj`` (C-contiguous only), ready on ``stream`` (the stream whose queued work completes
    it: a handle or a stream object, stream_handle) or else the stream its interface names (v3; None there: complete
    already), or else on none known (PyTorch: CudaArray.stream None)."""
    iface = obj.__cuda_array_interface__
    shape = tuple(int(n) for n in iface["shape"])
    dtype = np.dtype(iface["typestr"])
    strides = iface.get("strides")
    if strides is not None:
        expected, step = [], dtype.itemsize
        for n in reversed(shape):
            expected.insert(0, step)
            step *= n
        if tuple(strides) != tuple(expected):
            raise EncodingError(f"device array with strides {strides} is not C-contiguous (shape {shape}); make it "
                                f"contiguous first")
    if iface.get("mask") is not None:
        raise EncodingError("masked device arrays are not images")
    if stream is not None:
        ready = stream_handle(stream)
    elif "stream" in iface:
        ready = 0 if iface["stream"] is None else int(iface["stream"])
    else:
        ready = None
    return CudaArray(ptr=int(iface["data"][0]), shape=shape, dtype=dtype, stream=ready)


def write_ring(ring, array: CudaArray) -> tuple[int, int]:
    """(slot, seq): ``array`` copied into the CUDA ring's next slot after the work that completes it (on its stream, or
    with the stream not known after all of the process's work on the device), and waited for: when this returns, the
    array may be freed or overwritten (e.g. by PyTorch's allocator handing its memory to the next frame)."""
    if array.stream is None:
        ring.synchronize_device()
    slot, seq = ring.write_device(array.ptr, array.nbytes, 0 if array.stream is None else array.stream)
    ring.wait_written(slot)
    return slot, seq


def check_cuda_array(array: CudaArray, encoding: str):
    enc = info(encoding)
    if array.dtype != enc.dtype:
        raise EncodingError(f"{encoding} needs {enc.dtype}, the device array is {array.dtype}")
    channels = 1 if len(array.shape) == 2 else (array.shape[2] if len(array.shape) == 3 else None)
    if channels != enc.channels:
        raise EncodingError(f"{encoding} has {enc.channels} channel(s), the device array has shape {array.shape}")


class CudaImage:
    """A received frame in device memory, H x W (x C). With ``owner`` a DeviceBuffer it is the reader's own copy;
    otherwise a view into the CUDA ring, valid while ``Frame.is_valid()`` holds."""

    def __init__(self, ptr: int, shape: tuple, dtype: np.dtype, device: int, owner=None, readonly: bool = True):
        self.ptr, self.shape, self.dtype, self.device = ptr, tuple(shape), np.dtype(dtype), device
        self._owner = owner
        self._readonly = readonly

    @property
    def nbytes(self) -> int:
        return int(np.prod(self.shape)) * self.dtype.itemsize

    @property
    def __cuda_array_interface__(self) -> dict:
        # stream None: the data is complete (reader copies are synchronous; ring views were waited for on the legacy
        # default stream, which every blocking stream follows)
        return {"shape": self.shape, "typestr": self.dtype.str, "data": (self.ptr, self._readonly), "version": 3,
                "strides": None, "stream": None}

    def numpy(self) -> np.ndarray:
        """A host copy (for checks and tests; a consumer on the CPU subscribes to image_raw or shm instead)."""
        if not isinstance(self._owner, _core.DeviceBuffer):
            raise RuntimeError("numpy() needs a copied frame (ImageSubscriber with copy=True)")
        out = np.empty(self.shape, self.dtype)
        self._owner.download(out)
        return out


def device_ordinal(device) -> int | None:
    """A CUDA device as an ordinal (as CUDA_VISIBLE_DEVICES numbers them): an int, ``"cuda"`` (0), ``"cuda:N"``, a
    torch.device; None: 0 where there is a CUDA device, else None; ``"cpu"``: None."""
    if device is None:
        return 0 if _core.cuda_device_count() > 0 else None
    if isinstance(device, int):
        return device
    text = str(device)
    if text == "cpu":
        return None
    if text == "cuda":
        return 0
    if text.startswith("cuda:") and text[5:].isdigit():
        return int(text[5:])
    raise ValueError(f"device {device!r}: an ordinal, 'cuda', 'cuda:N' or 'cpu'")


def device_uuid(ordinal: int | None) -> str:
    """The UUID of the CUDA device of ``ordinal`` (32 hex digits, as the info records' cuda_device_uuid); empty for
    None or without a CUDA driver."""
    return _core.cuda_device_uuid(ordinal) if ordinal is not None else ""
