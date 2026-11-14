"""A node's model weights: their download at the node's start, checked, once for every process (standard library only).

A node keeps the list of its files (``Weight``: its path below the store's root, where it comes from, its exact size,
its SHA-256), the terms notice of its model, the code that must be there first, and sources of its own; this module
does the rest:

    STORE = weights.Store("xblick-vjepa2", node="vjepa2", terms=TERMS, former=[REPO / "weights"])
    weights.add_arguments(parser)                 # --download-only, --no-download, --weights-dir
    args = weights.parse_args(parser, argv)       # --download-only needs none of the start's required options
    store = STORE.at(args.weights_dir)            # --weights-dir: another place than the node's cache directory
    options = ["--checkpoint", args.checkpoint]   # the options that select the weights, for the lines
    if args.download_only:
        return store.download_only(needed(args.checkpoint), options=options)
    try:
        store.ensure(needed(args.checkpoint), download=not args.no_download, options=options)
    except weights.WeightsError as error:
        return store.fail(error)
    model = load(store.file(CHECKPOINT))          # the file's path: below store.root

The root is the node's cache directory (``xblick.paths.cache_dir``: ``~/.cache/xblick/vjepa2``, ``$XBLICK_HOME/cache/
vjepa2``), the same for a checkout and a pip install; a node may pass its own ``root``. ``options`` (a list of
command-line words) go into the command that the lines name (``xblick-vjepa2 --download-only --checkpoint vitl``), so
that the printed command fetches exactly the missing files.

``former``: where the node kept its weights before (its checkout's ``weights/``). A file missing in the root but there
is never used from there. The store prints once, before anything else, the command that moves that directory to the
root, then goes on as for any missing file (a download, or the ``--no-download`` error).

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
import copy
import fcntl
import hashlib
import os
import shlex
import signal
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass
from pathlib import Path

from . import paths

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
    """One file the node reads: where it is (``path``: relative, below the store's root; absolute paths do not move
    with ``--weights-dir`` and have no former place), where it comes from (``source``), its exact size (``size``,
    bytes: announced, and a download of another length is refused) and, where known, its SHA-256."""

    path: Path
    source: object  # HuggingFace, Url or a source of the node's own: describe() and fetch(tmp, progress, store)
    size: int
    sha256: str | None = None


def add_arguments(parser):
    """``--download-only`` and ``--no-download``, which exclude each other, and ``--weights-dir`` (``Store.at``)."""
    parser.add_argument("--weights-dir", default=None, metavar="DIR",
                        help="where the weights are and go (default: the node's cache directory, "
                             "~/.cache/xblick/<node> or $XBLICK_HOME/cache/<node>)")
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
    (the weights' directory: the download lock goes there) or ``node`` (its short name: the root is
    ``paths.cache_dir(node)``), ``terms`` (the notice printed before a download), ``required`` ((path, what): code
    that a setup script puts there first, checked before anything else), ``repo`` (paths in it are shown relative to
    it) and ``former`` (directories where the node kept the weights before: see ``ensure``)."""

    def __init__(self, prog: str, root: Path | None = None, terms: str | None = None, required=(),
                 repo: Path | None = None, *, node: str | None = None, former=()):
        if (root is None) == (node is None):
            raise ValueError(f"{prog}: give the weights' root or the node's short name (its cache directory), "
                             "exactly one")
        if terms is None:
            raise ValueError(f"{prog}: the model's terms notice (terms) is missing")
        if isinstance(former, (str, Path)):
            raise TypeError(f"former: a list of directories, not {former!r}")
        self.prog, self.terms, self.node = prog, terms, node
        self.root = Path(root) if root is not None else paths.cache_dir(node)
        self.required = tuple(required)
        self.repo = Path(repo) if repo is not None else None
        self.former = tuple(Path(f) for f in former)
        self.moved = False  # True: the root is --weights-dir's (at)
        self._told = False  # the former place's message printed (once a process)
        self._fetch_command = self.command()  # the command named when a download fails (ensure sets it)

    def at(self, directory) -> Store:
        """This store with ``directory`` as its root (``--weights-dir``: ``${VAR}`` and ``~`` expanded, relative to
        the working directory), or this store itself for None."""
        if directory is None:
            return self
        if not str(directory).strip():
            raise ValueError("--weights-dir: an empty directory name")
        moved = copy.copy(self)
        moved.root, moved.moved = paths.expand_path(directory).absolute(), True
        return moved

    def file(self, w: Weight) -> Path:
        """Where ``w`` is: below the root, or its absolute path."""
        return w.path if w.path.is_absolute() else self.root / w.path

    def command(self, options=()) -> str:
        """``<prog> --download-only`` with the node's options that select the weights (a list of command-line words,
        quoted for a shell where needed): the command that fetches them."""
        if isinstance(options, str):
            raise TypeError(f"options: a list of command-line words, not the string {options!r}")
        return shlex.join([self.prog, "--download-only", *options])

    def download_only(self, weights: list[Weight], out=None, options=()) -> int:
        """``--download-only``: the missing weights downloaded, every one with a known SHA-256 checked (also those
        there already), one line at the end; the exit code. ``options``: as for ``ensure``."""
        out = out or sys.stdout
        try:
            got = self.ensure(weights, verify=True, out=out, options=options)
        except WeightsError as error:
            return self.fail(error)
        if not weights:
            print(f"{self.prog}: no weights to download for these options", file=out, flush=True)
        elif got:
            print(f"{self.prog}: weights complete: {len(got)} downloaded into {self.short(self._root(weights))}",
                  file=out, flush=True)
        else:
            print(f"{self.prog}: weights complete in {self.short(self._root(weights))}, nothing to download",
                  file=out, flush=True)
        return 0

    def fail(self, error: WeightsError) -> int:
        """The error's line on stderr; its exit code."""
        print(f"{self.prog}: {error}", file=sys.stderr, flush=True)
        return error.code

    def ensure(self, weights: list[Weight], *, download: bool = True, verify: bool = False, out=None,
               options=()) -> list[Weight]:
        """Every file of ``weights`` there, or WeightsError (also when a path of ``required`` is missing: code that
        comes with tools/setup.sh). All there: returns [] at once, no network, no file written (``verify``: those
        with a SHA-256 are read and checked). Missing ones (``download`` False: WeightsError naming them): under the
        lock, the model's terms and what comes, then each file downloaded, checked and renamed. Returns those
        downloaded. ``options``: the node's options that select ``weights`` (``["--model", "s"]``), named in the
        command that an error line gives (``command``)."""
        out = out or sys.stdout
        fetch_command = self.command(options)
        for w in weights:
            if w.path.is_absolute() and (self.moved or self.former):
                raise ValueError(f"{w.path}: an absolute path does not move with --weights-dir and has no former "
                                 "place; give the weight's path relative to the store's root")
        for path, what in self.required:
            if not Path(path).exists():
                raise WeightsError(f"{self.short(path)} is missing ({what}): run tools/setup.sh")
        missing = [w for w in weights if not self.file(w).is_file()]
        with _stoppable():
            for w in weights if verify else []:
                if w not in missing and w.sha256 is not None:
                    digest = sha256_of(self.file(w))
                    if digest != w.sha256:
                        raise WeightsError(f"{self.short(self.file(w))}: wrong SHA-256 ({digest}, expected "
                                           f"{w.sha256}): remove it, then download again")
        if not missing:
            return []
        self._tell_former(missing, out)
        names = ", ".join(self.short(self.file(w)) for w in missing)
        if not download:
            raise WeightsError(f"weights missing: {names}; download them with {fetch_command} or "
                               "tools/setup.sh (--no-download is set)")
        self._fetch_command = fetch_command
        try:
            return self._download_missing(missing, out)
        finally:
            self._fetch_command = self.command()

    def _tell_former(self, missing: list[Weight], out):
        """Once a process: the former places that hold a file missing in the root, with the command that moves each
        there. The file is not used from the former place (no fallback): a moved checkout would lose it."""
        places = [old for old in self.former if any((old / w.path).is_file() for w in missing)]
        if self._told or not places:
            return
        self._told = True
        lines = [f"{self.prog}: the weights are kept in {self.root} now, and some of them are still at the node's "
                 "former place. They are not used from there. Move them (then start again):"]
        for old in places:
            if self.root.exists():  # into a directory that is there: copy what is missing, then remove the old one
                lines.append(f"  cp -a -n {shlex.quote(str(old))}/. {shlex.quote(str(self.root))}/ && "
                             f"rm -r {shlex.quote(str(old))}")
            else:
                lines.append(f"  mkdir -p {shlex.quote(str(self.root.parent))} && "
                             f"mv {shlex.quote(str(old))} {shlex.quote(str(self.root))}")
        print("\n".join(lines), file=out, flush=True)

    def _download_missing(self, missing: list[Weight], out) -> list[Weight]:
        """``ensure``'s downloads, under the lock."""
        with _stoppable(), self._locked(out):
            missing = [w for w in missing if not self.file(w).is_file()]  # another process may have downloaded them
            if not missing:
                print(f"{self.prog}: another process downloaded the weights", file=out, flush=True)
                return []
            total = sum(w.size for w in missing)
            print(f"{self.prog}: to download: " + ", ".join(f"{self.short(self.file(w))} ({size_text(w.size)})"
                                                           for w in missing)
                  + f", {size_text(total)} in all", file=out)
            print(self.terms, file=out, flush=True)
            for w in missing:
                self._download(w, out)
        return missing

    def _download(self, w: Weight, out):
        """One file: into ``<name>.part``, its size and (where known) its SHA-256 checked, renamed. A failed check
        keeps the .part file for a look; any other failure or an interrupt removes it."""
        path = self.file(w)
        tmp = path.with_name(path.name + ".part")
        path.parent.mkdir(parents=True, exist_ok=True)
        print(f"{self.prog}: {path.name} ({size_text(w.size)}) from {w.source.describe()}", file=out, flush=True)
        progress = _Progress(self.prog, path.name, out)
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
        os.replace(tmp, path)
        print(f"{self.prog}: {self.short(path)}: " + ("size and SHA-256 checked" if w.sha256 else "size checked")
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
                               f"{self._fetch_command} on one that is and copy {self.short(self.root)}") from None
        except OSError as error:
            raise WeightsError(f"{self.short(tmp)}: {error.strerror or error}") from None
        if total is not None and done != total:
            raise WeightsError(f"{url}: {done} of {total} bytes came: start again")
        return digest.hexdigest()

    def _root(self, weights: list[Weight]) -> Path:
        """The directory that holds ``weights``."""
        return Path(os.path.commonpath([self.file(w).parent for w in weights]))

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


def size_text(n: int | None) -> str:
    """A size for the lines: ``1.5 GB``, ``14.9 kB``, ``17 B``; ``?`` when unknown."""
    if n is None:
        return "?"
    for unit, scale in (("GB", 1e9), ("MB", 1e6), ("kB", 1e3)):
        if n >= scale:
            return f"{n / scale:.1f} {unit}"
    return f"{n} B"
