"""Which FFmpeg input options exist, for ``xblick cam --option KEY=VALUE`` (cam.py).

FFmpeg's avformat_open_input takes the options it knows and leaves the others in its dictionary. PyAV discards
that rest for an input without a word (it warns about unused options only for an output container), so a misspelt
option would be ignored. This module therefore asks the FFmpeg inside PyAV's wheel, through ctypes and FFmpeg's
public API (av_opt_find with AV_OPT_SEARCH_FAKE_OBJ on an AVClass), whether an option exists in one of the places
that avformat_open_input applies options to:

* FFmpeg's generic input options (avformat_get_class: ``fflags``, ``probesize``, ``analyzeduration``, ...),
* the input format's own options (AVInputFormat.priv_class: ``rtsp_transport`` and ``timeout`` of ``rtsp``,
  ``input_format`` of ``video4linux2``),
* the options of the URL's protocol (avio_protocol_get_class: ``reconnect`` of ``http``, ``timeout`` of ``tcp``).

An option in none of them is refused. Not reachable through the public API, and therefore refused too: the options
of the protocols below another one (the ``tcp`` under ``rtsp``) and FFmpeg's generic URL options (``rw_timeout``).
"""

from __future__ import annotations

import ctypes
import glob
import os

AV_OPT_SEARCH_FAKE_OBJ = 2
PRIV_CLASS_OFFSET = 5 * ctypes.sizeof(ctypes.c_void_p)  # AVInputFormat: name, long_name, flags, extensions,
# codec_tag, then priv_class (libavformat/avformat.h; `int flags` is padded to a pointer's size)


class OptionError(ValueError):
    """An option FFmpeg does not have where it would apply it."""


class _FFmpeg:
    """libavformat and libavutil of PyAV's wheel (av.libs, beside the package av)."""

    def __init__(self):
        import av

        libs = os.path.normpath(os.path.join(os.path.dirname(av.__file__), os.pardir, "av.libs"))
        found = {name: sorted(glob.glob(os.path.join(libs, f"lib{name}-*.so*"))) for name in ("avformat", "avutil")}
        if not all(found.values()):
            raise OptionError(f"--option needs FFmpeg's libraries of PyAV's wheel in {libs}; they are not there (a "
                              "PyAV built against a system FFmpeg): --option cannot be checked")
        self.format = ctypes.CDLL(found["avformat"][0])
        self.util = ctypes.CDLL(found["avutil"][0])
        self.format.avformat_get_class.restype = ctypes.c_void_p
        self.format.av_find_input_format.restype = ctypes.c_void_p
        self.format.av_find_input_format.argtypes = [ctypes.c_char_p]
        self.format.avio_protocol_get_class.restype = ctypes.c_void_p
        self.format.avio_protocol_get_class.argtypes = [ctypes.c_char_p]
        self.util.av_opt_find.restype = ctypes.c_void_p
        self.util.av_opt_find.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_int,
                                          ctypes.c_int]
        generic = self.format.avformat_get_class()
        # the layout this module reads: checked on what every FFmpeg has
        if self.class_name(generic) != "AVFormatContext" or not self.has(generic, "fflags"):
            raise OptionError("this FFmpeg's AVFormatContext options cannot be read: --option cannot be checked")
        v4l2 = self.input_format("video4linux2")
        if v4l2 is not None and (self.class_name(v4l2[1]) != "V4L2 indev" or not self.has(v4l2[1], "input_format")):
            raise OptionError("this FFmpeg's input formats are not laid out as libavformat/avformat.h states: "
                              "--option cannot be checked")
        self.generic = generic

    @staticmethod
    def class_name(cls: int | None) -> str | None:
        if not cls:
            return None
        return ctypes.cast(ctypes.c_void_p.from_address(cls).value, ctypes.c_char_p).value.decode()

    def has(self, cls: int | None, name: str) -> bool:
        if not cls:
            return False
        holder = ctypes.c_void_p(cls)
        return bool(self.util.av_opt_find(ctypes.byref(holder), name.encode(), None, 0, AV_OPT_SEARCH_FAKE_OBJ))

    def input_format(self, name: str) -> tuple[str, int | None] | None:
        """(names, priv_class) of an input format, None when FFmpeg has none of that name."""
        pointer = self.format.av_find_input_format(name.encode())
        if not pointer:
            return None
        names = ctypes.cast(ctypes.c_void_p.from_address(pointer).value, ctypes.c_char_p).value.decode()
        return names, ctypes.c_void_p.from_address(pointer + PRIV_CLASS_OFFSET).value

    def protocol(self, name: str) -> int | None:
        return self.format.avio_protocol_get_class(name.encode()) or None


_ffmpeg: _FFmpeg | None = None


def check(options: dict, fmt: str | None, source: str):
    """OptionError naming every option of ``options`` that FFmpeg would not use for ``source`` opened with the
    input format ``fmt`` (None: a URL, its format the URL's scheme where FFmpeg has an input format of that name)."""
    global _ffmpeg
    if not options:
        return
    if _ffmpeg is None:
        _ffmpeg = _FFmpeg()
    ff = _ffmpeg
    scheme, sep, _ = source.partition("://")
    scheme = scheme.lower() if sep else ""
    places = [("FFmpeg's generic input options", ff.generic)]
    named = fmt or scheme
    found = ff.input_format(named) if named else None
    if found is not None:
        places.append((f"the input format {found[0]}", found[1]))
    if scheme and ff.protocol(scheme):
        places.append((f"the protocol {scheme}", ff.protocol(scheme)))
    unknown = [key for key in options if not any(ff.has(cls, key) for _, cls in places)]
    if unknown:
        raise OptionError(f"--option {', '.join(unknown)}: FFmpeg has no such option in "
                          + ", ".join(where for where, _ in places)
                          + ("" if found is not None
                             else ". For an option of the format FFmpeg detects, name it with --format"))
