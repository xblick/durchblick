"""xblick.paths: the cache, data and config directories for every case of the environment (unset, XDG set, XBLICK_HOME
set, empty and relative values, ``~``), the node name's check, and the same cases through the C++ rule (paths.hpp,
called through xblick._core)."""

import pytest

from xblick import _core, paths

VARIABLES = ("XBLICK_HOME", "XDG_CACHE_HOME", "XDG_DATA_HOME", "XDG_CONFIG_HOME")

# (environment, cache_dir("yolox"), data_dir("nemo"), config_dir()), with HOME=/home/u
CASES = [
    ({}, "/home/u/.cache/xblick/yolox", "/home/u/.local/share/xblick/nemo", "/home/u/.config/xblick"),
    ({"XDG_CACHE_HOME": "/c", "XDG_DATA_HOME": "/d/", "XDG_CONFIG_HOME": "/e"},
     "/c/xblick/yolox", "/d/xblick/nemo", "/e/xblick"),
    ({"XDG_CACHE_HOME": "~/c", "XDG_DATA_HOME": "~", "XDG_CONFIG_HOME": "~/e"},
     "/home/u/c/xblick/yolox", "/home/u/xblick/nemo", "/home/u/e/xblick"),
    ({"XDG_CACHE_HOME": "relative", "XDG_DATA_HOME": "", "XDG_CONFIG_HOME": "./e"},  # ignored, as XDG says
     "/home/u/.cache/xblick/yolox", "/home/u/.local/share/xblick/nemo", "/home/u/.config/xblick"),
    ({"XDG_CACHE_HOME": "~other/c"},  # ~user is not expanded: relative, ignored
     "/home/u/.cache/xblick/yolox", "/home/u/.local/share/xblick/nemo", "/home/u/.config/xblick"),
    ({"XBLICK_HOME": "/srv/xblick", "XDG_CACHE_HOME": "/c", "XDG_CONFIG_HOME": "/e"},  # it wins; config stays
     "/srv/xblick/cache/yolox", "/srv/xblick/data/nemo", "/e/xblick"),
    ({"XBLICK_HOME": "~/x/"}, "/home/u/x/cache/yolox", "/home/u/x/data/nemo", "/home/u/.config/xblick"),
    ({"XBLICK_HOME": "", "XDG_CACHE_HOME": "/c"},  # empty: unset
     "/c/xblick/yolox", "/home/u/.local/share/xblick/nemo", "/home/u/.config/xblick"),
    ({"XBLICK_HOME": "/srv//a/./b/"}, "/srv/a/b/cache/yolox", "/srv/a/b/data/nemo", "/home/u/.config/xblick"),
]


@pytest.fixture
def env(monkeypatch):
    """HOME=/home/u and none of the variables; returns a function that sets some."""
    monkeypatch.setenv("HOME", "/home/u")
    for v in VARIABLES:
        monkeypatch.delenv(v, raising=False)

    def set_(values: dict):
        for k, v in values.items():
            monkeypatch.setenv(k, v)

    return set_


@pytest.mark.parametrize(("values", "cache", "data", "config"), CASES)
def test_the_directories_in_every_case_in_python_and_cpp(env, values, cache, data, config):
    env(values)
    assert (str(paths.cache_dir("yolox")), str(paths.data_dir("nemo")), str(paths.config_dir())) == \
        (cache, data, config)
    assert (_core.cpp_cache_dir("yolox"), _core.cpp_data_dir("nemo"), _core.cpp_config_dir()) == (cache, data, config)


@pytest.mark.parametrize("values", [{"XBLICK_HOME": "relative"}, {"XBLICK_HOME": "./x"}, {"HOME": "relative"},
                                    {"HOME": ""}])
def test_a_relative_xblick_home_or_home_is_refused_in_python_and_cpp(env, values):
    env(values)
    for ask in (paths.cache_dir, paths.data_dir, _core.cpp_cache_dir, _core.cpp_data_dir):
        with pytest.raises(ValueError, match="absolute path"):
            ask("yolox")


@pytest.mark.parametrize("name", ["", "xblick-yolox", "xblick_yolox", "xblick", "Yolox", "-a", "_a", "a/b", "..",
                                  ".", "a b", "a" * 65, "yolox\n"])
def test_a_node_name_that_is_not_a_short_name_is_refused_in_python_and_cpp(env, name):
    for ask in (paths.cache_dir, paths.data_dir, _core.cpp_cache_dir, _core.cpp_data_dir):
        with pytest.raises(ValueError, match="node name"):
            ask(name)


def test_short_names_and_the_hint_for_a_program_name(env):
    assert [paths.check_node_name(n) for n in ("yolox", "sam3", "gem-x", "k4abt", "a_1", "a" * 64)] == \
        ["yolox", "sam3", "gem-x", "k4abt", "a_1", "a" * 64]
    with pytest.raises(ValueError, match=r"without the program's 'xblick-' \(yolox\)"):
        paths.cache_dir("xblick-yolox")


def test_asking_creates_nothing(env, tmp_path, monkeypatch):
    monkeypatch.setenv("HOME", str(tmp_path))
    paths.cache_dir("yolox"), paths.data_dir("yolox"), paths.config_dir()
    env({"XBLICK_HOME": str(tmp_path / "x")})
    paths.cache_dir("yolox"), paths.data_dir("yolox")
    _core.cpp_cache_dir("yolox"), _core.cpp_data_dir("yolox"), _core.cpp_config_dir()
    assert list(tmp_path.iterdir()) == []


def test_without_home_the_password_database(env, monkeypatch):
    import pwd
    import os

    monkeypatch.delenv("HOME")
    home = pwd.getpwuid(os.getuid()).pw_dir
    assert str(paths.cache_dir("yolox")) == f"{home}/.cache/xblick/yolox" == _core.cpp_cache_dir("yolox")


def test_the_terminal_interfaces_presets_follow_config_dir(env):
    from xblick import tui

    env({"XDG_CONFIG_HOME": "/e", "XBLICK_HOME": "/srv"})
    assert str(tui.presets_path()) == "/e/xblick/rerun_presets.json"
