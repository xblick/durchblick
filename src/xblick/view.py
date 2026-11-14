"""``xblick view``: streams as MJPEG over HTTP, shown in VLC or a browser, from Python's standard library.

    xblick view test/camera/color                 # serves it and starts VLC (one window per stream)
    xblick view a/color b/depth --browser         # the grid page of all streams, side by side, in the browser
    xblick view test/camera/color --no-player     # serves it and prints the addresses only

The command subscribes to each named stream's compressed transport and serves its JPEG frames unchanged, one URL per
stream (``/<stream>.mjpg``, ``multipart/x-mixed-replace``) and a page with a grid of all of them (``/``).
Depth (16-bit, RVL or PNG) is decoded, coloured by ``overlay.depth_colours`` (a fixed range: red at 300 mm through
green to blue at 5000 mm, no depth black) and encoded as JPEG; other frames that are no JPEG are encoded as JPEG.
Point clouds and streams without compressed transport are refused.

By default it starts VLC: the program ``vlc`` on PATH, one window per stream, ``--network-caching=100``. Without
``vlc`` on PATH it says so and, on a terminal, asks whether to open the browser view instead; without a terminal it
does not ask and keeps serving. ``--browser`` opens the grid page in the system's browser; ``--no-player`` starts
nothing. On a system without a display (Linux: neither DISPLAY nor WAYLAND_DISPLAY set) it starts no VLC and no
browser and asks nothing: it prints the addresses and how to open them from another PC (``--http``, or
``ssh -L PORT:127.0.0.1:PORT <this PC>``), and keeps serving.

While it serves on a terminal, the key v opens the VLC windows that are not open (any more) and b the browser view,
without Enter, also with --no-player; on a headless system they start nothing. Without a terminal nothing reads
stdin. The start lines give, per stream, the command to open it in VLC by hand.

Each client gets the newest frame whenever it can take one: a slow client skips frames and holds up neither the
subscriber nor the other clients. It listens on 127.0.0.1 by default (this PC only); ``--http 0.0.0.0:PORT`` opens
it to the network WITHOUT A PASSWORD: anyone who can reach the address can watch.
"""

from __future__ import annotations

import html
import http.server
import io
import os
import shutil
import socket
import subprocess
import sys
import threading
import time
import webbrowser

from . import topics
from .camera import find_streams, wait_for_info
from .codecs import JPEG, decompress, parse_format
from .msg import CompressedImage
from .session import why_nothing
from .overlay import DEPTH_FAR_MM, DEPTH_NEAR_MM, depth_colours

PORT = 8090  # the default port (fixed)
LISTEN = f"127.0.0.1:{PORT}"
BOUNDARY = "xblickframe"
VLC_CACHE_MS = 100  # VLC's --network-caching when xblick view starts it: little delay, a few frames of buffer
CLIENT_TIMEOUT_S = 10.0  # a client that takes no frame for this long is dropped
JPEG_QUALITY = 85
DEPTH_RANGE = (DEPTH_NEAR_MM, DEPTH_FAR_MM)  # depth colours: red at 300 mm through green to blue at 5000 mm (fixed)  # for frames xblick view encodes itself (depth, PNG, raw)


class ViewError(RuntimeError):
    """One line for the user: a stream that cannot be shown, a port that is taken, VLC that is missing."""


def parse_listen(text: str) -> tuple[str, int]:
    host, sep, port = text.rpartition(":")
    if not sep or not host or not port.isdigit():
        raise ViewError(f"--http {text!r}: ADDRESS:PORT, e.g. 0.0.0.0:{PORT}")
    return host.strip("[]"), int(port)


class _Feed:
    """The newest JPEG of one stream and its number; clients wait for a newer one."""

    def __init__(self, base: str):
        self.base = base
        self.jpeg: bytes | None = None
        self.number = 0
        self.cond = threading.Condition()
        self.error: BaseException | None = None

    def put(self, jpeg: bytes):
        with self.cond:
            self.jpeg, self.number = jpeg, self.number + 1
            self.cond.notify_all()

    def next_after(self, number: int, timeout: float) -> tuple[int, bytes | None]:
        with self.cond:
            self.cond.wait_for(lambda: self.number > number, timeout)
            return self.number, self.jpeg


def to_jpeg(message: CompressedImage, depth_range: tuple[float, float]) -> bytes:
    """A compressed frame as JPEG: a JPEG as it is (the same bytes), else decoded (depth coloured) and encoded."""
    if parse_format(message.format)[1] == JPEG:
        return bytes(message.data)
    from PIL import Image

    image, encoding = decompress(message)
    if encoding in ("16UC1", "mono16"):
        image = depth_colours(image, *depth_range)
    elif encoding in ("bgr8", "bgra8"):
        image = image[..., 2::-1]
    elif encoding == "rgba8":
        image = image[..., :3]
    elif encoding not in ("rgb8", "mono8", "8UC1"):
        raise ViewError(f"frames of {encoding} cannot be shown")
    out = io.BytesIO()
    Image.fromarray(image.copy()).save(out, "JPEG", quality=JPEG_QUALITY)
    return out.getvalue()


class View:
    """The streams ``bases`` served as MJPEG on ``listen`` (host, port) until :meth:`close`. ViewError for a stream
    that does not answer, has no compressed transport or is a point cloud, and for a port that is taken."""

    def __init__(self, session, bases: list[str], listen: tuple[str, int] = ("127.0.0.1", PORT),
                 depth_range: tuple[float, float] = DEPTH_RANGE, seconds: float = 2.0):
        if not bases:
            raise ViewError("name the streams to show, e.g. xblick view test/camera/color")
        self.session, self.depth_range = session, depth_range
        self.feeds: dict[str, _Feed] = {}
        self._subscribers = []
        self.clients = 0
        self._clients_lock = threading.Lock()
        self._stop = threading.Event()
        for base in bases:
            self._check(topics.check_name(base), seconds)
        for base in bases:
            feed = self.feeds[base] = _Feed(base)
            self._subscribers.append(session.declare_subscriber(topics.compressed(base),
                                                                lambda s, f=feed: self._receive(f, s)))
        try:
            self.server = self._serve(listen)
        except ViewError:
            for subscriber in self._subscribers:
                subscriber.undeclare()
            raise
        self.host, self.port = listen[0], self.server.server_address[1]
        threading.Thread(target=self.server.serve_forever, name="xblick view", daemon=True).start()

    def _check(self, base: str, seconds: float):
        records = wait_for_info(self.session, base, seconds)
        if not records:
            found = [b for b, r in find_streams(self.session, "", seconds) if r.get("kind") == "image"]
            why = "" if found else why_nothing(self.session)
            raise ViewError(f"{base}: no publisher answers within {seconds:g} s; image streams found: "
                            + (", ".join(found) or "none") + (f" ({why})" if why else ""))
        record = records[0]
        if record.get("kind") == "points":
            raise ViewError(f"{base}: a point cloud stream; xblick view shows images only (xblick rerun shows clouds)")
        if "compressed" not in record.get("transports", ()):
            raise ViewError(f"{base}: no compressed transport (its encoding {record.get('encoding')} has no codec), so "
                            "it cannot be served")

    def _receive(self, feed: _Feed, sample):
        try:
            feed.put(to_jpeg(CompressedImage.deserialize(sample.payload.to_bytes()), self.depth_range))
        except Exception as error:  # noqa: BLE001 - shown by the command's loop (check)
            feed.error = error

    def check(self):
        for feed in self.feeds.values():
            if feed.error is not None:
                raise ViewError(f"{feed.base}: {type(feed.error).__name__}: {feed.error}")

    def url(self, base: str | None = None) -> str:
        host = "127.0.0.1" if self.host in ("0.0.0.0", "") else self.host
        return f"http://{host}:{self.port}/" + (f"{base}.mjpg" if base else "")

    def index(self) -> bytes:
        cells = "\n".join(f'<figure><img src="/{html.escape(b)}.mjpg" alt="{html.escape(b)}">'
                          f"<figcaption>{html.escape(b)}</figcaption></figure>" for b in self.feeds)
        return ("<!doctype html><html><head><meta charset='utf-8'><title>xblick view</title><style>"
                "body{margin:0;background:#111;color:#ddd;font:14px sans-serif}"
                "main{display:grid;gap:8px;padding:8px;grid-template-columns:repeat(auto-fit,minmax(320px,1fr))}"
                "figure{margin:0}img{width:100%;height:auto;background:#000}figcaption{padding:4px 0}"
                f"</style></head><body><main>\n{cells}\n</main></body></html>\n").encode()

    def _serve(self, listen: tuple[str, int]) -> http.server.ThreadingHTTPServer:
        view = self

        class Handler(http.server.BaseHTTPRequestHandler):
            timeout = CLIENT_TIMEOUT_S  # a client that takes nothing for this long is dropped

            def log_message(self, *args):
                pass

            def do_GET(self):
                path = self.path.split("?", 1)[0]
                if path in ("/", "/index.html"):
                    body = view.index()
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                base = path.lstrip("/").removesuffix(".mjpg")
                if not path.endswith(".mjpg") or base not in view.feeds:
                    self.send_error(404, "no such stream: " + ", ".join(f"/{b}.mjpg" for b in view.feeds))
                    return
                view._stream(self, view.feeds[base])

        class Server(http.server.ThreadingHTTPServer):
            address_family = socket.AF_INET6 if ":" in listen[0] else socket.AF_INET
            allow_reuse_address = True  # a restart right after clients left (TIME_WAIT); a listening port stays an error
            allow_reuse_port = False
            daemon_threads = True

        try:
            server = Server(listen, Handler)
        except OSError as error:
            raise ViewError(f"cannot listen on {listen[0]}:{listen[1]} ({error.strerror or error}): the port is taken "
                            "or the address is not this PC's; give another one with --http") from None
        return server

    def _stream(self, handler, feed: _Feed):
        """One client: the newest frame whenever it can take one (frames in between are skipped, for it alone)."""
        with self._clients_lock:
            self.clients += 1
        try:
            handler.send_response(200)
            handler.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY}")
            handler.send_header("Cache-Control", "no-cache")
            handler.end_headers()
            number = 0
            while not self._stop.is_set():
                number, jpeg = feed.next_after(number, 0.5)
                if jpeg is None or number == 0:
                    continue
                handler.wfile.write(f"--{BOUNDARY}\r\nContent-Type: image/jpeg\r\nContent-Length: {len(jpeg)}"
                                    "\r\n\r\n".encode() + jpeg + b"\r\n")
                handler.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
            pass  # the client went
        finally:
            with self._clients_lock:
                self.clients -= 1

    def close(self):
        self._stop.set()
        for feed in self.feeds.values():
            with feed.cond:
                feed.cond.notify_all()
        for subscriber in self._subscribers:
            subscriber.undeclare()
        self.server.shutdown()
        self.server.server_close()


def start_vlc(vlc: str, urls: list[str]) -> list[subprocess.Popen]:
    """One VLC per stream (``vlc`` the program), with a small network cache (VLC_CACHE_MS)."""
    return [subprocess.Popen([vlc, f"--network-caching={VLC_CACHE_MS}", url], stdin=subprocess.DEVNULL,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) for url in urls]


def headless() -> bool:
    """No display: on Linux and the other Unix systems neither DISPLAY (X11) nor WAYLAND_DISPLAY (Wayland) is set,
    the variables every graphical program there needs (a fixed rule, not a probe). macOS and Windows always have one.
    On a headless system xblick view starts no VLC and no browser: Python's webbrowser would start a text browser
    (lynx, w3m) in this terminal."""
    return (sys.platform not in ("darwin", "win32") and not os.environ.get("DISPLAY")
            and not os.environ.get("WAYLAND_DISPLAY"))


def remote_hint(view: "View") -> str:
    """How to open the addresses from another PC."""
    host = socket.gethostname()
    if view.host in ("127.0.0.1", "localhost", "::1"):
        return (f"open them from another PC: start with --http 0.0.0.0:{view.port} (no password), or forward the port "
                f"with ssh -L {view.port}:127.0.0.1:{view.port} {host} and open http://127.0.0.1:{view.port}/ there")
    return f"open them from another PC as http://{host}:{view.port}/"


def open_browser(url: str, prog: str):
    """The grid page in the system's browser (not called on a headless system), or one line when none opens."""
    if not webbrowser.open(url):
        print(f"{prog}: no browser could be opened; open {url} yourself", flush=True)


def _ask(question: str) -> bool:
    """The question on stdout (input() would print it on stderr on a terminal), the answer from stdin; no answer
    (end of input) is no."""
    print(question, end="", flush=True)
    return sys.stdin.readline().strip().lower() in ("y", "yes")


NO_DISPLAY = "this system has no display (neither DISPLAY nor WAYLAND_DISPLAY is set)"


class _Players:
    """The VLC windows, one per stream; ``open`` starts those that are not open (any more)."""

    def __init__(self, view: View, prog: str):
        self.view, self.prog = view, prog
        self.processes: dict[str, subprocess.Popen] = {}
        self.closed_said = False

    def open_vlc(self, asked: bool):
        """The windows that are not open; ``asked``: by the key v (then one line also when nothing is to start)."""
        if headless():
            print(f"{self.prog}: {NO_DISPLAY}: nothing started", flush=True)
            return
        vlc = shutil.which("vlc")
        if vlc is None:
            print(f"{self.prog}: VLC is not installed (no vlc on PATH)", flush=True)
            return
        missing = [b for b in self.view.feeds if b not in self.processes or self.processes[b].poll() is not None]
        for base, process in zip(missing, start_vlc(vlc, [self.view.url(b) for b in missing])):
            self.processes[base] = process
        if missing or asked:
            print(f"{self.prog}: started VLC ({len(missing)} window(s))" if missing else
                  f"{self.prog}: the VLC windows are open", flush=True)
        self.closed_said = False

    def all_closed(self) -> bool:
        return bool(self.processes) and all(p.poll() is not None for p in self.processes.values())

    def close(self):
        for process in self.processes.values():
            if process.poll() is None:
                process.terminate()
        for process in self.processes.values():
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()


def browser(view: View, prog: str):
    if headless():
        print(f"{prog}: {NO_DISPLAY}: nothing started", flush=True)
    else:
        open_browser(view.url(), prog)


class _Keys:
    """Single keys from the terminal on stdin (cbreak: no Enter, no echo, Ctrl-C stays a signal); the terminal's mode
    restored by ``close``. Without a terminal on stdin nothing reads it."""

    def __init__(self):
        self.fd = sys.stdin.fileno() if sys.stdin is not None and sys.stdin.isatty() else None
        self.saved = None
        if self.fd is not None:
            import termios
            import tty

            self.saved = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)

    def get(self, timeout: float) -> str:
        """A key pressed within ``timeout`` seconds, else ""; without a terminal it only waits."""
        if self.fd is None:
            time.sleep(timeout)
            return ""
        import select

        if not select.select([self.fd], [], [], timeout)[0]:
            return ""
        return os.read(self.fd, 1).decode(errors="replace").lower()

    def close(self):
        if self.saved is not None:
            import termios

            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)
            self.saved = None


KEYS = "press v for VLC, b for the browser view, Ctrl-C to end"


def run(session, bases: list[str], listen: str = LISTEN, player: str = "vlc",
        depth_range: tuple[float, float] = DEPTH_RANGE, seconds: float = 2.0, prog: str = "xblick view") -> int:
    """The command: serve until Ctrl-C or SIGTERM (KeyboardInterrupt); the exit code. ``player``: "vlc" (the
    default: VLC if ``vlc`` is on PATH, else ask whether to open the browser instead; without a terminal never asks),
    "browser" (the grid page in the system's browser) or "none" (serve only). On a terminal the keys v (VLC windows
    that are not open) and b (the browser view) work in every case."""
    if player not in ("vlc", "browser", "none"):
        raise ValueError(f"player {player!r}: vlc, browser or none")
    try:
        view = View(session, bases, parse_listen(listen), depth_range, seconds)
    except ViewError as error:
        print(f"{prog}: {error}", file=sys.stderr, flush=True)
        return 1
    players = _Players(view, prog)
    keys = None
    try:
        local = view.host in ("127.0.0.1", "localhost", "::1")
        print(f"{prog}: serving {len(bases)} stream(s) on http://{view.host}:{view.port}/ "
              + ("(this PC only; --http 0.0.0.0:PORT opens it to the network)" if local else
                 "(open to the network: anyone who can reach this address can watch, there is no password)"),
              flush=True)
        for base in bases:
            print(f"{prog}: {base}: {view.url(base)}", flush=True)
        print(f"{prog}: all of them: {view.url()}", flush=True)
        for base in bases:
            print(f"{prog}: by hand: vlc --network-caching={VLC_CACHE_MS} {view.url(base)}", flush=True)
        terminal = sys.stdin is not None and sys.stdin.isatty()
        if player == "none":
            pass
        elif headless():
            print(f"{prog}: {NO_DISPLAY}, so no player is started; {remote_hint(view)}", flush=True)
        elif player == "browser":
            open_browser(view.url(), prog)
        elif shutil.which("vlc") is not None:
            players.open_vlc(asked=False)
        elif terminal and sys.stdout.isatty():
            print(f"{prog}: VLC is not installed (no vlc on PATH)", flush=True)
            if _ask(f"{prog}: open the browser view instead? [y/N] "):
                open_browser(view.url(), prog)
        else:
            print(f"{prog}: VLC is not installed (no vlc on PATH); --browser opens the browser view", flush=True)
        keys = _Keys()  # after the question: it reads a line
        print(f"{prog}: serving; " + (KEYS if keys.fd is not None else "Ctrl-C to stop"), flush=True)
        while True:
            key = keys.get(0.5)
            view.check()
            if key == "v":
                players.open_vlc(asked=True)
            elif key == "b":
                browser(view, prog)
            if players.all_closed() and not players.closed_said:
                players.closed_said = True
                print(f"{prog}: the VLC windows were closed; still serving. "
                      + (KEYS[0].upper() + KEYS[1:] + "." if keys.fd is not None else
                         f"At {view.url()} until Ctrl-C."), flush=True)
    except KeyboardInterrupt:
        return 0
    except ViewError as error:
        print(f"{prog}: {error}", file=sys.stderr, flush=True)
        return 1
    finally:
        if keys is not None:
            keys.close()
        players.close()
        view.close()
