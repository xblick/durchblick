// Where nodes keep their files, the rule of Python's xblick.paths (docs/reference/files.md):
//
//   cache_dir("k4abt")  $XBLICK_HOME/cache/k4abt, else $XDG_CACHE_HOME/xblick/k4abt, else ~/.cache/xblick/k4abt
//   data_dir("nemo")    $XBLICK_HOME/data/nemo,   else $XDG_DATA_HOME/xblick/nemo,   else ~/.local/share/xblick/nemo
//   config_dir()        $XDG_CONFIG_HOME/xblick, else ~/.config/xblick (XBLICK_HOME does not move it)
//
// The cache holds what can be fetched or built again (weights, TensorRT engines), the data what a user made. An XDG
// variable that is empty or relative (after ~ is expanded) is ignored, as the XDG Base Directory Specification says.
// XBLICK_HOME empty counts as unset; relative, it is an error. Nothing is created. A node's short name: lower-case
// letters, digits, '-' and '_', starting with a letter or a digit, at most 64 characters, without the program's
// "xblick-". tests/test_paths.py checks every case against Python through xblick._core. In xblick::core.
#pragma once

#include <filesystem>
#include <string_view>

namespace xblick
{
    // std::invalid_argument unless `node` is a node's short name (yolox, not xblick-yolox).
    void check_node_name(std::string_view node);
    // std::invalid_argument for a relative XBLICK_HOME or HOME.
    std::filesystem::path cache_dir(std::string_view node);
    std::filesystem::path data_dir(std::string_view node);
    std::filesystem::path config_dir();
}  // namespace xblick
