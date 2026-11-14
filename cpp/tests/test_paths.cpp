// paths.hpp: the cache, data and config directories for each case of the environment, and the node name's check.
// The comparison with Python's xblick.paths for the same cases is in tests/test_paths.py.
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>

#include "xblick/paths.hpp"

namespace
{
    int failures = 0;

    void check(bool ok, const std::string& what)
    {
        if (not ok)
        {
            ++failures;
            std::printf("FAIL: %s\n", what.c_str());
        }
    }

    void equal(const std::filesystem::path& got, const std::string& expected, const std::string& what)
    {
        check(got.string() == expected, what + ": " + got.string() + ", expected " + expected);
    }

    template <class F>
    void refused(F f, const std::string& what)
    {
        try
        {
            f();
            check(false, what + ": not refused");
        }
        catch (const std::invalid_argument&)
        {
        }
    }

    void clear()
    {
        for (const char* v : {"XBLICK_HOME", "XDG_CACHE_HOME", "XDG_DATA_HOME", "XDG_CONFIG_HOME"})
        {
            unsetenv(v);
        }
    }
}  // namespace

int main()
{
    using xblick::cache_dir, xblick::config_dir, xblick::data_dir;
    setenv("HOME", "/home/u", 1);
    clear();
    equal(cache_dir("yolox"), "/home/u/.cache/xblick/yolox", "unset: cache");
    equal(data_dir("nemo"), "/home/u/.local/share/xblick/nemo", "unset: data");
    equal(config_dir(), "/home/u/.config/xblick", "unset: config");

    setenv("XDG_CACHE_HOME", "/c", 1);
    setenv("XDG_DATA_HOME", "/d/", 1);
    setenv("XDG_CONFIG_HOME", "~/conf", 1);
    equal(cache_dir("k4abt"), "/c/xblick/k4abt", "XDG: cache");
    equal(data_dir("nemo"), "/d/xblick/nemo", "XDG: data");
    equal(config_dir(), "/home/u/conf/xblick", "XDG: config, ~ expanded");

    setenv("XDG_CACHE_HOME", "relative/c", 1);
    setenv("XDG_DATA_HOME", "", 1);
    equal(cache_dir("k4abt"), "/home/u/.cache/xblick/k4abt", "XDG relative: ignored");
    equal(data_dir("nemo"), "/home/u/.local/share/xblick/nemo", "XDG empty: ignored");

    setenv("XBLICK_HOME", "/srv/xblick", 1);
    equal(cache_dir("gem-x"), "/srv/xblick/cache/gem-x", "XBLICK_HOME: cache");
    equal(data_dir("nemo"), "/srv/xblick/data/nemo", "XBLICK_HOME: data");
    equal(config_dir(), "/home/u/conf/xblick", "XBLICK_HOME: config not moved");
    setenv("XBLICK_HOME", "~/x", 1);
    equal(cache_dir("sam3"), "/home/u/x/cache/sam3", "XBLICK_HOME with ~");
    setenv("XBLICK_HOME", "", 1);
    equal(cache_dir("sam3"), "/home/u/.cache/xblick/sam3", "XBLICK_HOME empty: unset");
    setenv("XBLICK_HOME", "rel", 1);
    refused([] { cache_dir("sam3"); }, "XBLICK_HOME relative");
    clear();

    for (const char* bad : {"", "xblick-yolox", "xblick", "Yolox", "-a", "a/b", "..", "a b"})
    {
        refused([bad] { xblick::check_node_name(bad); }, std::string("node name '") + bad + "'");
    }
    xblick::check_node_name("gem-x");
    xblick::check_node_name("k4abt_2");
    refused([] { xblick::check_node_name(std::string(65, 'a')); }, "node name of 65 characters");
    setenv("HOME", "relative", 1);
    refused([] { cache_dir("yolox"); }, "HOME relative");

    std::printf(failures ? "%d failures\n" : "paths: ok\n", failures);
    return failures ? 1 : 0;
}
