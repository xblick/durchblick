"""xblick.weights, the download of a node's weights at its start, without the real weights or the internet.

Downloads go to a local HTTP server: URL files directly, Hugging Face files through HF_ENDPOINT (the request, its
redirect to a second path, the token only to the first). Where network use must not happen, any use fails the test.
Taken from the part that was the same in every node's test_weights.py (xblick-vjepa2, -tapnext, -gem-x, -sam3,
-boxer, -nemo); a node keeps the tests of its own files."""

import argparse
import contextlib
import hashlib
import http.server
import io
import os
import signal
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest

from xblick import weights

TERMS_LINE = "A model is the work of its authors"  # the stand-in terms notice

DATA = os.urandom(3 * 1024 * 1024 + 17)  # a stand-in weight file, larger than a few read chunks
DATA_SHA = hashlib.sha256(DATA).hexdigest()


class Server:
    """A local HTTP server of files: path -> bytes. ``/resolve/`` paths redirect to ``/cdn/`` (as Hugging Face's do);
    ``slow``: each MB after a pause; ``stall``: half the file, then nothing until the server stops. Records each GET
    and its Authorization header."""

    def __init__(self):
        self.files: dict[str, bytes] = {}
        self.gets: list[tuple[str, str | None]] = []
        self.slow = self.stall = False
        self.status: int | None = None  # answer every GET with this status
        self.stop = threading.Event()
        owner = self

        class Handler(http.server.BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_GET(self):
                owner.gets.append((self.path, self.headers.get("Authorization")))
                if owner.status:
                    self.send_error(owner.status)
                    return
                if "/resolve/" in self.path:
                    self.send_response(302)
                    self.send_header("Location", "/cdn/" + self.path.rsplit("/", 1)[1])
                    self.send_header("Content-Length", "0")
                    self.end_headers()
                    return
                key = self.path.removeprefix("/cdn")
                if key not in owner.files:
                    self.send_error(404)
                    return
                data = owner.files[key]
                self.send_response(200)
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                with contextlib.suppress(OSError):
                    if owner.stall:
                        self.wfile.write(data[:len(data) // 2])
                        self.wfile.flush()
                        owner.stop.wait(60)
                        return
                    for i in range(0, len(data), 1 << 20):
                        if owner.slow:
                            time.sleep(0.3)
                        self.wfile.write(data[i:i + (1 << 20)])

        self.httpd = http.server.ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.httpd.daemon_threads = True
        self.url = f"http://127.0.0.1:{self.httpd.server_address[1]}"
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

    def close(self):
        if not self.stop.is_set():
            self.stop.set()
            self.httpd.shutdown()
            self.httpd.server_close()


@pytest.fixture
def server():
    s = Server()
    yield s
    s.close()


@pytest.fixture
def no_network(monkeypatch):
    """Any network use of this process fails the test."""
    def refuse(*args, **kwargs):
        raise AssertionError(f"network use: {args}")

    monkeypatch.setattr(socket.socket, "connect", refuse)
    monkeypatch.setattr(socket.socket, "connect_ex", refuse)
    monkeypatch.setattr(socket, "create_connection", refuse)
    monkeypatch.setattr(socket, "getaddrinfo", refuse)


@pytest.fixture
def home(tmp_path, monkeypatch):
    """The weights' root in tmp_path (the lock goes there too); a token that is no login of this PC."""
    root = tmp_path / "weights"
    monkeypatch.setenv("HF_TOKEN", "secret")
    monkeypatch.delenv("HF_HUB_OFFLINE", raising=False)
    return root


@pytest.fixture
def store(home):
    return weights.Store("xblick-test", root=home, terms=TERMS_LINE + ", under its terms.", repo=home.parent)


@pytest.fixture
def stand_in(home, server, monkeypatch):
    """Two files of a Hugging Face repository (through HF_ENDPOINT) as stand-ins on the local server: (weights,
    {path: content})."""
    monkeypatch.setenv("HF_ENDPOINT", server.url)
    server.files["/config.json"] = b"{}"
    server.files["/model.safetensors"] = DATA
    ws = [weights.Weight(home / "m" / "config.json", weights.HuggingFace("org/m", "rev1", "config.json"), 2,
                         hashlib.sha256(b"{}").hexdigest()),
          weights.Weight(home / "m" / "model.safetensors", weights.HuggingFace("org/m", "rev1", "model.safetensors"),
                         len(DATA), DATA_SHA)]
    return ws, {home / "m" / "config.json": b"{}", home / "m" / "model.safetensors": DATA}


def url_weight(root: Path, server: Server, name="a.bin", sha=DATA_SHA) -> weights.Weight:
    server.files[f"/{name}"] = DATA
    return weights.Weight(root / name, weights.Url(f"{server.url}/{name}"), len(DATA), sha)


def place(files: dict[Path, bytes]):
    for path, data in files.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)


# ------------------------------------------------------------------------------------------------ all present


def test_all_present_touches_neither_network_nor_disk(home, store, stand_in, no_network):
    ws, files = stand_in
    place(files)
    before = sorted(home.rglob("*"))
    out = io.StringIO()
    assert store.ensure(ws, out=out) == [] and out.getvalue() == ""
    assert store.ensure(ws, download=False) == []
    assert sorted(home.rglob("*")) == before  # no lock file either


def test_download_only_with_all_present_prints_one_line(home, store, stand_in, no_network, capsys):
    ws, files = stand_in
    place(files)
    assert store.download_only(ws) == 0
    lines = capsys.readouterr().out.splitlines()
    assert len(lines) == 1 and "nothing to download" in lines[0]


# ---------------------------------------------------------------------------------------------------- downloads


def test_a_hugging_face_file_is_downloaded_checked_and_renamed(home, store, server, monkeypatch):
    monkeypatch.setenv("HF_ENDPOINT", server.url)
    server.files["/model.bin"] = DATA
    w = weights.Weight(home / "sub" / "model.bin", weights.HuggingFace("org/model", "abc123", "model.bin"), len(DATA))
    out = io.StringIO()
    assert store.ensure([w], out=out) == [w]
    assert w.path.read_bytes() == DATA and not list(home.rglob("*.part"))
    # the node's URL; the token to Hugging Face, not to the host it redirects to
    assert server.gets == [("/org/model/resolve/abc123/model.bin", "Bearer secret"), ("/cdn/model.bin", None)]
    text = out.getvalue()
    assert "to download: " in text and TERMS_LINE in text and "Hugging Face org/model at revision abc123" in text
    assert text.index(TERMS_LINE) < text.index("from Hugging Face")  # the terms before the download


def test_a_url_file_is_checked_against_its_sha256(home, store, server):
    w = url_weight(home, server)
    assert store.ensure([w], out=io.StringIO()) == [w]
    assert w.path.read_bytes() == DATA
    assert store.ensure([w], out=io.StringIO()) == [] and len(server.gets) == 1


def test_a_wrong_checksum_keeps_the_download_aside_and_says_to_remove_it(home, store, server):
    w = url_weight(home, server, sha="0" * 64)
    with pytest.raises(weights.WeightsError) as error:
        store.ensure([w], out=io.StringIO())
    assert "a.bin.part: wrong SHA-256" in str(error.value) and "remove it" in str(error.value)
    assert not w.path.exists() and (home / "a.bin.part").read_bytes() == DATA


def test_a_download_of_another_length_is_refused_and_kept_aside(home, store, server):
    w = url_weight(home, server, sha=None)
    w = weights.Weight(w.path, w.source, len(DATA) + 1)  # a size in code that the source no longer has
    with pytest.raises(weights.WeightsError, match=rf"a.bin.part: {len(DATA)} bytes, expected {len(DATA) + 1}.*remove"):
        store.ensure([w], out=io.StringIO())
    assert not w.path.exists() and (home / "a.bin.part").exists()


def test_download_only_checks_present_files_against_their_sha256(home, store, no_network):
    w = weights.Weight(home / "a.bin", weights.Url("http://127.0.0.1:9/a.bin"), 1, DATA_SHA)
    place({w.path: b"x"})
    with pytest.raises(weights.WeightsError, match="a.bin: wrong SHA-256.*remove it"):
        store.ensure([w], verify=True)
    assert w.path.exists()  # not deleted


def test_a_gated_repository_says_how_to_get_access(home, store, server, monkeypatch):
    monkeypatch.setenv("HF_ENDPOINT", server.url)
    gated = weights.Weight(home / "c.pt", weights.HuggingFace("facebook/sam3.1", "main", "c.pt", gated=True), 1)
    server.status = 401
    with pytest.raises(weights.WeightsError) as error:
        store.ensure([gated], out=io.StringIO())
    message = str(error.value)
    assert "https://huggingface.co/facebook/sam3.1" in message and "hf auth login" in message and "HF_TOKEN" in message


def test_without_a_network_download_only_ends_with_one_line(home, store, server, stand_in, capsys):
    ws, stand_in = stand_in
    server.close()  # no server any more: as without a network
    assert store.download_only(ws) == 1
    err = capsys.readouterr().err.strip().splitlines()
    assert len(err) == 1 and "cannot download from 127.0.0.1" in err[0] and "connect this PC" in err[0]
    assert "run xblick-test --download-only on one that is" in err[0]
    assert all(not path.exists() for path in stand_in) and not list(home.rglob("*.part"))
    assert store.download_only(ws, options=["--model", "s", "--dir", "a b"]) == 1  # the node's selecting options
    assert "run xblick-test --download-only --model s --dir 'a b' on one that is" in capsys.readouterr().err


def test_missing_code_from_the_setup_ends_with_one_line(home, stand_in, no_network, capsys):
    ws, _ = stand_in
    store = weights.Store("xblick-test", home, TERMS_LINE, required=((home.parent / "third_party" / "code",
                                                                      "the upstream code"),), repo=home.parent)
    with pytest.raises(weights.WeightsError, match=r"code is missing \(the upstream code\): run tools/setup.sh"):
        store.ensure(ws, out=io.StringIO())
    assert store.download_only(ws) == 1 and "run tools/setup.sh" in capsys.readouterr().err
    assert not home.exists()  # nothing written: no lock, no directory


def test_hf_hub_offline_is_kept(home, store, monkeypatch, no_network):
    monkeypatch.setenv("HF_HUB_OFFLINE", "1")
    w = weights.Weight(home / "m.bin", weights.HuggingFace("org/model", "abc123", "m.bin"), 1)
    with pytest.raises(weights.WeightsError, match="HF_HUB_OFFLINE"):
        store.ensure([w], out=io.StringIO())


def test_progress_in_a_log_file_has_lines_and_no_carriage_returns(home, store, server):
    w = url_weight(home, server)
    log = io.StringIO()  # no terminal
    store.ensure([w], out=log)
    progress = [line for line in log.getvalue().splitlines() if "a.bin: " in line and "%" in line]
    assert "\r" not in log.getvalue() and 3 <= len(progress) <= 11 and "100%" in progress[-1]

    class Terminal(io.StringIO):
        def isatty(self):
            return True

    w.path.unlink()
    terminal = Terminal()
    store.ensure([w], out=terminal)
    assert "\r" in terminal.getvalue()


# ----------------------------------------------------- interrupts and two processes at once (own processes)

CHILD = """
import signal
import sys
from pathlib import Path
from xblick.weights import Store, Url, Weight, WeightsError
signal.signal(signal.SIGINT, signal.default_int_handler)  # also where a background shell ignores it
root, url, sha, size = Path(sys.argv[1]), sys.argv[2], sys.argv[3], int(sys.argv[4])
store = Store("xblick-test", root, "terms")
try:
    store.ensure([Weight(root / "a.bin", Url(url), size, sha)])
except WeightsError as error:
    sys.exit(store.fail(error))
print("done", flush=True)
"""


def child(root: Path, server: Server) -> subprocess.Popen:
    return subprocess.Popen([sys.executable, "-c", CHILD, str(root), f"{server.url}/a.bin", DATA_SHA, str(len(DATA))],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)


def wait_for(condition, seconds=20.0):
    deadline = time.monotonic() + seconds
    while not condition():
        assert time.monotonic() < deadline
        time.sleep(0.05)


@pytest.mark.parametrize(("sig", "code"), [(signal.SIGTERM, 143), (signal.SIGINT, 130)])
def test_an_interrupted_download_leaves_no_file(home, server, sig, code):
    server.files["/a.bin"] = DATA
    server.stall = True
    process = child(home, server)
    try:
        wait_for(lambda: (home / "a.bin.part").exists() and (home / "a.bin.part").stat().st_size > 0)
        process.send_signal(sig)
        _, err = process.communicate(timeout=10)  # promptly: well before the server's stall ends
    finally:
        process.kill()
    assert process.returncode == code and "stopped (" in err and "Traceback" not in err
    assert not (home / "a.bin").exists() and not (home / "a.bin.part").exists()


def test_two_processes_at_once_download_once(home, server):
    server.files["/a.bin"] = DATA
    server.slow = True  # about a second: the second process starts while the first downloads
    first = child(home, server)
    wait_for(lambda: (home / "a.bin.part").exists())
    second = child(home, server)
    outputs = [p.communicate(timeout=60) for p in (first, second)]
    assert [p.returncode for p in (first, second)] == [0, 0], outputs
    assert [path for path, _ in server.gets] == ["/a.bin"]
    assert "another process downloaded the weights" in outputs[1][0]
    assert (home / "a.bin").read_bytes() == DATA


# ------------------------------------------------------------------------------------------- the command line


def parser():
    p = argparse.ArgumentParser(prog="xblick-test")
    p.add_argument("--camera", required=True)
    weights.add_arguments(p)
    return p


def test_download_only_needs_no_start_options_and_excludes_no_download(capsys):
    assert weights.parse_args(parser(), ["--download-only"]).download_only
    with pytest.raises(SystemExit) as exit_:
        weights.parse_args(parser(), ["--download-only", "--no-download"])
    assert exit_.value.code == 2 and "not allowed with" in capsys.readouterr().err
    with pytest.raises(SystemExit) as exit_:
        weights.parse_args(parser(), ["--no-download"])  # a start needs its required options
    assert exit_.value.code == 2 and "required" in capsys.readouterr().err
    p = parser()
    weights.parse_args(p, ["--download-only"])
    assert [a.required for a in p._actions if a.dest == "camera"] == [True]  # required again after


def test_download_only_downloads_and_exits_0(home, store, stand_in, capsys):
    ws, files = stand_in
    assert store.download_only(ws) == 0
    assert all(path.read_bytes() == data for path, data in files.items())
    assert f"weights complete: {len(files)} downloaded" in capsys.readouterr().out.splitlines()[-1]


def test_no_download_ends_naming_the_missing_files(home, store, stand_in, no_network):
    ws, files = stand_in
    with pytest.raises(weights.WeightsError) as error:
        store.ensure(ws, download=False)
    assert all(path.name in str(error.value) for path in files) and "xblick-test --download-only" in str(error.value)
    assert not any(path.exists() for path in files)
    with pytest.raises(weights.WeightsError) as error:
        store.ensure(ws, download=False, options=["--model", "s"])  # the printed command fetches these files
    assert "download them with xblick-test --download-only --model s or tools/setup.sh" in str(error.value)
    with pytest.raises(TypeError, match="a list of command-line words"):
        store.ensure(ws, download=False, options="--model s")


def test_a_source_of_the_nodes_own_gets_the_store(home, store):
    """A node's own source (an archive member, joined shards) writes ``tmp`` and returns its SHA-256; the store
    gives it its download (``store.http``) and its names (``store.short``)."""
    class Joined:
        def describe(self):
            return "two parts, joined"

        def fetch(self, tmp, progress, given):
            assert given is store and given.short(tmp).startswith("weights")
            tmp.write_bytes(b"ab")
            return hashlib.sha256(b"ab").hexdigest()

    w = weights.Weight(home / "j.bin", Joined(), 2, hashlib.sha256(b"ab").hexdigest())
    assert store.ensure([w], out=io.StringIO()) == [w] and w.path.read_bytes() == b"ab"


# --------------------------------------------- the node's cache directory, --weights-dir, the former place


@pytest.fixture
def user(tmp_path, monkeypatch):
    """A HOME in tmp_path, none of the variables that move the cache; the node's cache directory."""
    monkeypatch.setenv("HOME", str(tmp_path / "u"))
    for v in ("XBLICK_HOME", "XDG_CACHE_HOME", "XDG_DATA_HOME", "XDG_CONFIG_HOME"):
        monkeypatch.delenv(v, raising=False)
    return tmp_path / "u" / ".cache" / "xblick" / "yolox"


def node_store(**kwargs) -> weights.Store:
    return weights.Store("xblick-yolox", node="yolox", terms=TERMS_LINE, **kwargs)


def relative_weight(server: Server, name="a.bin") -> weights.Weight:
    server.files[f"/{name}"] = DATA
    return weights.Weight(Path(name), weights.Url(f"{server.url}/{name}"), len(DATA), DATA_SHA)


def test_the_default_root_is_the_nodes_cache_directory_and_nothing_is_made_before_a_download(user, server):
    store = node_store()
    assert store.root == user and not user.parent.parent.exists()  # asking made nothing
    w = relative_weight(server)
    out = io.StringIO()
    assert store.ensure([w], out=out) == [w]
    assert (user / "a.bin").read_bytes() == DATA and store.file(w) == user / "a.bin"
    assert f"{user}/a.bin: size and SHA-256 checked" in out.getvalue()  # the real path, not one relative to a repo


def test_xblick_home_moves_the_default_root(user, tmp_path, monkeypatch):
    monkeypatch.setenv("XBLICK_HOME", str(tmp_path / "shared"))
    assert node_store().root == tmp_path / "shared" / "cache" / "yolox"


def test_a_store_takes_a_root_or_a_node_and_a_short_name(user, tmp_path):
    with pytest.raises(ValueError, match="exactly one"):
        weights.Store("xblick-yolox", terms=TERMS_LINE)
    with pytest.raises(ValueError, match="exactly one"):
        weights.Store("xblick-yolox", tmp_path, TERMS_LINE, node="yolox")
    with pytest.raises(ValueError, match="node name"):
        weights.Store("xblick-yolox", node="xblick-yolox", terms=TERMS_LINE)
    with pytest.raises(ValueError, match="terms"):
        weights.Store("xblick-yolox", node="yolox")
    with pytest.raises(TypeError, match="a list of directories"):
        node_store(former=tmp_path)


def test_weights_dir_moves_the_root(user, server, tmp_path, monkeypatch):
    p = argparse.ArgumentParser(prog="xblick-yolox")
    weights.add_arguments(p)
    monkeypatch.setenv("SHARED", str(tmp_path / "shared"))
    args = weights.parse_args(p, ["--weights-dir", "${SHARED}/yolox", "--no-download"])
    store = node_store().at(args.weights_dir)
    assert store.root == tmp_path / "shared" / "yolox" and node_store().root == user  # the node's STORE unchanged
    assert node_store().at(weights.parse_args(p, []).weights_dir).root == user  # not given: the cache directory
    w = relative_weight(server)
    with pytest.raises(weights.WeightsError, match=f"weights missing: {tmp_path}/shared/yolox/a.bin"):
        store.ensure([w], download=False)
    assert store.ensure([w], out=io.StringIO()) == [w] and (tmp_path / "shared" / "yolox" / "a.bin").is_file()
    monkeypatch.chdir(tmp_path)
    assert node_store().at("rel").root == tmp_path / "rel"  # relative to the working directory, shown absolute
    with pytest.raises(ValueError, match="does not move with --weights-dir"):
        store.ensure([weights.Weight(tmp_path / "abs.bin", w.source, 1)], download=False)


def former_place(tmp_path: Path, server: Server):
    """A node's checkout with the weight in its weights/ (the former place)."""
    old = tmp_path / "checkout" / "weights"
    place({old / "a.bin": DATA})
    return old, relative_weight(server)


def test_a_file_only_at_the_former_place_is_not_used_and_the_move_is_printed_once(user, server, tmp_path):
    old, w = former_place(tmp_path, server)
    store = node_store(former=[old])
    with pytest.raises(weights.WeightsError, match="weights missing") as error:
        out = io.StringIO()
        store.ensure([w], download=False, out=out)
    lines = out.getvalue().splitlines()
    assert lines[0].startswith(f"xblick-yolox: the weights are kept in {user} now") and "not used" in lines[0]
    assert lines[1] == f"  mkdir -p {user.parent} && mv {old} {user}"
    assert "a.bin" in str(error.value)
    out = io.StringIO()
    assert store.ensure([w], out=out) == [w]  # then as for any missing file: downloaded, the message not again
    assert "former place" not in out.getvalue() and "to download:" in out.getvalue()
    assert (old / "a.bin").is_file()  # nothing moved by the node


def test_the_command_runs_and_into_a_root_that_is_there_it_copies(user, server, tmp_path):
    old, w = former_place(tmp_path, server)
    out = io.StringIO()
    node_store(former=[old]).download_only([w], out=out)  # --download-only prints it too
    assert f"mv {old} {user}" in out.getvalue()
    user.joinpath("a.bin").unlink()  # the root is there now (the lock), the file is not
    out = io.StringIO()
    with pytest.raises(weights.WeightsError):
        node_store(former=[old]).ensure([w], download=False, out=out)
    command = out.getvalue().splitlines()[1].strip()
    assert command == f"cp -a -n {old}/. {user}/ && rm -r {old}"
    subprocess.run(["bash", "-c", command], check=True)
    assert node_store(former=[old]).ensure([w], download=False) == [] and not old.exists()


def test_both_places_or_neither_print_nothing(user, server, tmp_path):
    old, w = former_place(tmp_path, server)
    place({user / "a.bin": DATA})
    out = io.StringIO()
    assert node_store(former=[old]).ensure([w], out=out) == [] and out.getvalue() == ""  # the new place is used
    user.joinpath("a.bin").unlink()
    (old / "a.bin").unlink()
    out = io.StringIO()
    assert node_store(former=[old]).ensure([w], out=out) == [w]
    assert "former place" not in out.getvalue()


def test_a_weights_dir_is_quoted_in_the_command(user, server, tmp_path):
    old, w = former_place(tmp_path, server)
    store = node_store(former=[old]).at(str(tmp_path / "a b"))
    out = io.StringIO()
    with pytest.raises(weights.WeightsError):
        store.ensure([w], download=False, out=out)
    assert f"mv {old} '{tmp_path}/a b'" in out.getvalue()
