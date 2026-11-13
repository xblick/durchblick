"""``xblick relay``: which relay program it starts (cli.relay_program) and that it passes the arguments unchanged.

Each case runs the command in a child process, since it replaces its process by the relay. Stub programs stand in
for the relay. A child that must not see an installed xblick-relay sets ``sys.modules["xblick_relay"] = None``."""

import os
import subprocess
import sys
import textwrap

HIDE_PACKAGE = "import sys; sys.modules['xblick_relay'] = None; "
RUN = "from xblick.cli import main; sys.exit(main())"


def stub(directory, name="xblick-relay"):
    """A program that writes its own path and its arguments, one per line, to $RELAY_OUT."""
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / name
    path.write_text('#!/bin/sh\nprintf "%s\\n" "$0" "$@" > "$RELAY_OUT"\n')
    path.chmod(0o755)
    return path


def fake_package(root, binary_body):
    """A package xblick_relay on PYTHONPATH whose binary() runs binary_body."""
    package = root / "xblick_relay"
    package.mkdir(parents=True)
    (package / "__init__.py").write_text("from pathlib import Path\n\n\ndef binary():\n"
                                         + textwrap.indent(binary_body, "    ") + "\n")


def xblick(args, tmp_path, path_dirs, prelude="", pythonpath=None):
    env = {"PATH": os.pathsep.join(str(d) for d in path_dirs), "RELAY_OUT": str(tmp_path / "out.txt")}
    if pythonpath is not None:
        env["PYTHONPATH"] = str(pythonpath)
    return subprocess.run([sys.executable, "-c", "import sys; " + prelude + RUN, "relay", *args], env=env,
                          capture_output=True, text=True, timeout=60)


def test_relay_on_path_gets_the_arguments_unchanged(tmp_path):
    program = stub(tmp_path / "bin")
    done = xblick(["--help", "--stream", "robot/a", "robot/b", "--cpu"], tmp_path, [tmp_path / "bin"], HIDE_PACKAGE)
    assert done.returncode == 0, done.stderr
    assert (tmp_path / "out.txt").read_text().splitlines() == [str(program), "--help", "--stream", "robot/a",
                                                                "robot/b", "--cpu"]


def test_no_relay_fails_and_names_both_ways_to_install_it(tmp_path):
    (tmp_path / "empty").mkdir()
    done = xblick(["--help"], tmp_path, [tmp_path / "empty"], HIDE_PACKAGE)
    assert done.returncode == 1
    assert "xblick-relay not found" in done.stderr
    assert "pip install 'xblick[relay]'" in done.stderr and "CMake" in done.stderr
    assert not (tmp_path / "out.txt").exists()


def test_the_package_comes_before_path(tmp_path):
    packaged = stub(tmp_path / "wheel")
    stub(tmp_path / "bin")
    fake_package(tmp_path / "site", f"return Path({str(packaged)!r})")
    done = xblick(["--cpu"], tmp_path, [tmp_path / "bin"], pythonpath=tmp_path / "site")
    assert done.returncode == 0, done.stderr
    assert (tmp_path / "out.txt").read_text().splitlines() == [str(packaged), "--cpu"]


def test_a_package_without_its_program_fails_without_trying_path(tmp_path):
    stub(tmp_path / "bin")
    fake_package(tmp_path / "site", "raise FileNotFoundError('xblick_relay/bin/xblick-relay is missing')")
    done = xblick(["--cpu"], tmp_path, [tmp_path / "bin"], pythonpath=tmp_path / "site")
    assert done.returncode == 1
    assert "xblick relay: xblick_relay/bin/xblick-relay is missing" in done.stderr
    assert not (tmp_path / "out.txt").exists()
