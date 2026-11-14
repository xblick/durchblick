"""A node's model weights: their download at the node's start, checked, once for every process (standard library only).

A node keeps the list of its files (``Weight``: where it goes, where it comes from, its exact size, its SHA-256), the
terms notice of its model, the code that must be there first, and sources of its own; this module does the rest:

    STORE = weights.Store("xblick-vjepa2", root=REPO / "weights", terms=TERMS, required=(), repo=REPO)
    weights.add_arguments(parser)                 # --download-only, --no-download
    args = weights.parse_args(parser, argv)       # --download-only needs none of the start's required options
    if args.download_only:
        return STORE.download_only(needed(args.checkpoint))
    try:
        STORE.ensure(needed(args.checkpoint), download=not args.no_download)
    except weights.WeightsError as error:
        return STORE.fail(error)

``ensure`` touches neither the disk nor the network when every file is there. A missing file is downloaded under a
temporary name in its directory (``<name>.part``), checked and only then renamed, all under a file lock in the root,
so that two nodes started at once download once and never read half a file. The terms notice is printed before the
first byte. Ctrl-C and SIGTERM end a download at once and leave no partial file.

Sources: ``HuggingFace`` (a fixed revision; ``HF_ENDPOINT``, ``HF_TOKEN`` or the huggingface_hub login,
``HF_HUB_OFFLINE``), ``Url``, or a node's own with ``describe()`` and ``fetch(tmp, progress, store)`` returning the
SHA-256 of what it wrote (``store.http`` downloads, ``store.short`` names a path).
"""

from __future__ import annotations

import contextlib
import fcntl
import hashlib
import os
import signal
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass
from pathlib import Path

CHUNK = 1 << 20  # bytes read at a time
TIMEOUT_S = 60.0  # without a byte for this long, a download fails
LOCK = ".download.lock"  # in the root: one lock for every download of the node


class WeightsError(Exception):
    """Why the weights are not there: one line for the user, and the exit code (130 Ctrl-C, 143 SIGTERM)."""

    def __init__(self, message: str, code: int = 1):
        super().__init__(message)
        self.code = code


class _Terminated(BaseException):
    """SIGTERM during a download (a BaseException: no ``except Exception`` on the way swallows it)."""


@dataclass(frozen=True)
class HuggingFace:
    """A file of a Hugging Face repository at a fixed revision; ``gated``: the repository wants an accepted licence
    and a login."""

    repo: str
    revision: str
    filename: str
    gated: bool = False

    def describe(self) -> str:
        return f"Hugging Face {self.repo} at revision {self.revision}"

    def url(self) -> str:
        endpoint = os.environ.get("HF_ENDPOINT", "https://huggingface.co").rstrip("/")
        return f"{endpoint}/{self.repo}/resolve/{self.revision}/{urllib.parse.quote(self.filename)}"

    def fetch(self, tmp: Path, progress, store: Store) -> str:
        if os.environ.get("HF_HUB_OFFLINE", "").strip().lower() in ("1", "true", "yes", "on"):
            raise WeightsError(f"HF_HUB_OFFLINE is set, so {self.filename} is not downloaded from {self.repo}: unset "
                               "it, or copy the file from a PC that has it")
        denied = None
        if self.gated:
            denied = (f"{self.repo} is gated: accept its licence at https://huggingface.co/{self.repo} with your "
                      "Hugging Face account, then log in once with .venv/bin/hf auth login or set HF_TOKEN")
        return store.http(self.url(), tmp, progress, token=_hf_token(), denied=denied)


@dataclass(frozen=True)
class Url:
    """A file at a URL."""

    url: str

    def describe(self) -> str:
        return self.url

    def fetch(self, tmp: Path, progress, store: Store) -> str:
        return store.http(self.url, tmp, progress)


@dataclass(frozen=True)
class Weight:
    """One file the node reads: where it is (``path``), where it comes from (``source``), its exact size (``size``,
    bytes: announced, and a download of another length is refused) and, where known, its SHA-256."""

    path: Path
    source: object  # HuggingFace, Url or a source of the node's own: describe() and fetch(tmp, progress, store)
    size: int
    sha256: str | None = None


def add_arguments(parser):
    """``--download-only`` and ``--no-download``, which exclude each other."""
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--download-only", action="store_true",
                       help="download the missing weights, check them and end (for preparation scripts): no camera, "
                            "no Zenoh, no GPU; the options a start requires may be left out")
    group.add_argument("--no-download", action="store_true",
                       help="never download: end at once when weights are missing (for machines that must not start "
                            "a download on their own)")


def parse_args(parser, argv=None):
    """``parser.parse_args(argv)``. With ``--download-only`` (spelled out) the options a start requires (``--camera``
    ...) are not required: a preparation script names only what selects the weights."""
    tokens = list(sys.argv[1:] if argv is None else argv)
    options = tokens[:tokens.index("--")] if "--" in tokens else tokens
    if "--download-only" not in options:
        return parser.parse_args(argv)
    required = [action for action in parser._actions if action.required]  # argparse keeps no public list
    for action in required:
        action.required = False
    try:
        return parser.parse_args(argv)
    finally:
        for action in required:
            action.required = True


class Store:
    """The weights of one node: ``prog`` (its command: the lines' prefix and the downloads' User-Agent), ``root``
    (the weights' directory: the download lock goes there), ``terms`` (the notice printed before a download),
    ``required`` ((path, what): code that a setup script puts there first, checked before anything else) and
    ``repo`` (paths in messages are shown relative to it)."""

    def __init__(self, prog: str, root: Path, terms: str, required=(), repo: Path | None = None):
        self.prog, self.root, self.terms = prog, Path(root), terms
        self.required = tuple(required)
        self.repo = Path(repo) if repo is not None else None

    def download_only(self, weights: list[Weight], out=None) -> int:
        """``--download-only``: the missing weights downloaded, every one with a known SHA-256 checked (also those
        there already), one line at the end; the exit code."""
        out = out or sys.stdout
        try:
            got = self.ensure(weights, verify=True, out=out)
        except WeightsError as error:
            return self.fail(error)
        if not weights:
            print(f"{self.prog}: no weights to download for these options", file=out, flush=True)
        elif got:
            print(f"{self.prog}: weights complete: {len(got)} downloaded into {self.short(_root(weights))}", file=out,
                  flush=True)
        else:
            print(f"{self.prog}: weights complete in {self.short(_root(weights))}, nothing to download", file=out,
                  flush=True)
        return 0

    def fail(self, error: WeightsError) -> int:
        """The error's line on stderr; its exit code."""
        print(f"{self.prog}: {error}", file=sys.stderr, flush=True)
        return error.code

    def ensure(self, weights: list[Weight], *, download: bool = True, verify: bool = False, out=None) -> list[Weight]:
        """Every file of ``weights`` there, or WeightsError (also when a path of ``required`` is missing: code that
        comes with tools/setup.sh). All there: returns [] at once, no network, no file written (``verify``: those
        with a SHA-256 are read and checked). Missing ones (``download`` False: WeightsError naming them): under the
        lock, the model's terms and what comes, then each file downloaded, checked and renamed. Returns those
        downloaded."""
        out = out or sys.stdout
        for path, what in self.required:
            if not Path(path).exists():
                raise WeightsError(f"{self.short(path)} is missing ({what}): run tools/setup.sh")
        missing = [w for w in weights if not w.path.is_file()]
        with _stoppable():
            for w in weights if verify else []:
                if w not in missing and w.sha256 is not None:
                    digest = sha256_of(w.path)
                    if digest != w.sha256:
                        raise WeightsError(f"{self.short(w.path)}: wrong SHA-256 ({digest}, expected {w.sha256}): "
                                           "remove it, then download again")
        if not missing:
            return []
        names = ", ".join(self.short(w.path) for w in missing)
        if not download:
            raise WeightsError(f"weights missing: {names}; download them with {self.prog} --download-only or "
                               "tools/setup.sh (--no-download is set)")
        with _stoppable(), self._locked(out):
            missing = [w for w in missing if not w.path.is_file()]  # another process may have downloaded them
            if not missing:
                print(f"{self.prog}: another process downloaded the weights", file=out, flush=True)
                return []
            total = sum(w.size for w in missing)
            print(f"{self.prog}: to download: " + ", ".join(f"{self.short(w.path)} ({size_text(w.size)})"
                                                           for w in missing)
                  + f", {size_text(total)} in all", file=out)
            print(self.terms, file=out, flush=True)
            for w in missing:
                self._download(w, out)
        return missing

    def _download(self, w: Weight, out):
        """One file: into ``<name>.part``, its size and (where known) its SHA-256 checked, renamed. A failed check
        keeps the .part file for a look; any other failure or an interrupt removes it."""
        tmp = w.path.with_name(w.path.name + ".part")
        w.path.parent.mkdir(parents=True, exist_ok=True)
        print(f"{self.prog}: {w.path.name} ({size_text(w.size)}) from {w.source.describe()}", file=out, flush=True)
        progress = _Progress(self.prog, w.path.name, out)
        try:
            digest = w.source.fetch(tmp, progress, self)
        except BaseException:
            tmp.unlink(missing_ok=True)
            raise
        finally:
            progress.end()
        if (length := tmp.stat().st_size) != w.size:
            raise WeightsError(f"{self.short(tmp)}: {length} bytes, expected {w.size}: the source changed or the "
                               "download is damaged; remove it, then start again")
        if w.sha256 is not None and digest != w.sha256:
            raise WeightsError(f"{self.short(tmp)}: wrong SHA-256 ({digest}, expected {w.sha256}): the source changed "
                               "or the download is damaged; remove it, then start again")
        os.replace(tmp, w.path)
        print(f"{self.prog}: {self.short(w.path)}: " + ("size and SHA-256 checked" if w.sha256 else "size checked")
              + ", in place", file=out, flush=True)

    def http(self, url: str, tmp: Path, progress, token: str | None = None, denied: str | None = None) -> str:
        """GET ``url`` into ``tmp`` (redirects followed; ``token`` as a bearer to ``url``'s host only); the SHA-256
        of what came. ``denied``: the message for HTTP 401 and 403."""
        request = urllib.request.Request(url, headers={"User-Agent": self.prog})
        if token:
            request.add_unredirected_header("Authorization", f"Bearer {token}")
        host = urllib.parse.urlsplit(url).netloc
        digest, done = hashlib.sha256(), 0
        try:
            with urllib.request.urlopen(request, timeout=TIMEOUT_S) as response, open(tmp, "wb") as f:
                length = response.headers.get("Content-Length")
                total = int(length) if length is not None else None
                progress(0, total)
                while chunk := response.read(CHUNK):
                    f.write(chunk)
                    digest.update(chunk)
                    done += len(chunk)
                    progress(done, total)
                f.flush()
                os.fsync(f.fileno())
        except urllib.error.HTTPError as error:
            if error.code in (401, 403) and denied:
                raise WeightsError(denied) from None
            raise WeightsError(f"{url}: HTTP {error.code} {error.reason}") from None
        except (urllib.error.URLError, TimeoutError, ConnectionError) as error:
            reason = getattr(error, "reason", error)
            raise WeightsError(f"cannot download from {host} ({reason}): connect this PC to the internet, or run "
                               f"{self.prog} --download-only on one that is and copy {self.short(self.root)}") from None
        except OSError as error:
            raise WeightsError(f"{self.short(tmp)}: {error.strerror or error}") from None
        if total is not None and done != total:
            raise WeightsError(f"{url}: {done} of {total} bytes came: start again")
        return digest.hexdigest()

    def short(self, path) -> str:
        """``path`` relative to the repository where it is in it."""
        if self.repo is not None:
            try:
                return str(Path(path).relative_to(self.repo))
            except ValueError:
                pass
        return str(path)

    @contextlib.contextmanager
    def _locked(self, out):
        """The lock of this node's downloads (fcntl): another process downloading, wait for it."""
        path = self.root / LOCK
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "a") as f:
            try:
                fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                print(f"{self.prog}: another process downloads the weights; waiting for it", file=out, flush=True)
                fcntl.flock(f, fcntl.LOCK_EX)
            try:
                yield
            finally:
                fcntl.flock(f, fcntl.LOCK_UN)


def _hf_token() -> str | None:
    """The Hugging Face token: huggingface_hub's (HF_TOKEN, else the login's), else HF_TOKEN."""
    try:
        from huggingface_hub import get_token  # noqa: PLC0415 - only where a download needs it
    except ImportError:
        return os.environ.get("HF_TOKEN") or None
    return get_token()


class _Progress:
    """Progress of one file: on a terminal a line that updates in place, else (a log file) a line per tenth."""

    def __init__(self, prog: str, name: str, out):
        self.prog, self.name, self.out = prog, name, out
        self.tty = bool(getattr(out, "isatty", lambda: False)())
        self.start = self.shown = time.monotonic()
        self.step = -1  # the last tenth printed
        self.drawn = False

    def __call__(self, done: int, total: int | None):
        now = time.monotonic()
        rate = done / max(now - self.start, 1e-6) / 1e6
        text = f"{size_text(done)} of {size_text(total)}" if total else size_text(done)
        if self.tty:
            if now - self.shown >= 0.25 or (total and done == total):
                percent = f"{100 * done // total:3d}% " if total else ""
                self.out.write(f"\r{self.prog}: {self.name}: {percent}{text}, {rate:.1f} MB/s   ")
                self.out.flush()
                self.shown, self.drawn = now, True
            return
        step = 10 * done // total if total else done // (500 * 10**6)  # without a length: every 500 MB
        if step > self.step and done:
            self.step = step
            percent = f"{100 * done // total}% " if total else ""
            print(f"{self.prog}: {self.name}: {percent}({text}, {rate:.1f} MB/s)", file=self.out, flush=True)

    def end(self):
        if self.drawn:
            self.out.write("\n")
            self.out.flush()


@contextlib.contextmanager
def _stoppable():
    """Ctrl-C and SIGTERM end a download or a check at once: WeightsError, 130 or 143 (the .part file removed)."""
    main = threading.current_thread() is threading.main_thread()

    def terminate(signum, frame):
        raise _Terminated

    previous = signal.signal(signal.SIGTERM, terminate) if main else None
    try:
        yield
    except KeyboardInterrupt:
        raise WeightsError("stopped (Ctrl-C); no partial file kept", 130) from None
    except _Terminated:
        raise WeightsError("stopped (SIGTERM); no partial file kept", 143) from None
    finally:
        if main:
            signal.signal(signal.SIGTERM, previous)


def sha256_of(path: Path) -> str:
    """The SHA-256 of a file, read in CHUNK pieces."""
    digest = hashlib.sha256()
    with open(path, "rb") as f:
        while chunk := f.read(CHUNK):
            digest.update(chunk)
    return digest.hexdigest()


def _root(weights: list[Weight]) -> Path:
    return Path(os.path.commonpath([w.path.parent for w in weights]))


def size_text(n: int | None) -> str:
    """A size for the lines: ``1.5 GB``, ``14.9 kB``, ``17 B``; ``?`` when unknown."""
    if n is None:
        return "?"
    for unit, scale in (("GB", 1e9), ("MB", 1e6), ("kB", 1e3)):
        if n >= scale:
            return f"{n / scale:.1f} {unit}"
    return f"{n} B"
