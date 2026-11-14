// Where nodes keep their files (paths.hpp): the rule of Python's xblick.paths.
#include "xblick/paths.hpp"

#include <pwd.h>
#include <unistd.h>

#include <cstdlib>
#include <stdexcept>
#include <string>
#include <vector>

namespace xblick
{
    namespace
    {
        // without empty and "." parts, as Python's pathlib writes a path ("/a//b/./c/" is "/a/b/c")
        std::filesystem::path clean(const std::filesystem::path& path)
        {
            std::filesystem::path out = path.root_path();
            for (const auto& part : path.relative_path())
            {
                if (not part.empty() and part != ".")
                {
                    out /= part;
                }
            }
            return out;
        }

        std::string env(const char* name)
        {
            const char* value = std::getenv(name);
            return value ? value : "";
        }

        std::filesystem::path home()
        {
            if (const char* value = std::getenv("HOME"))
            {
                if (std::string(value).empty() or value[0] != '/')
                {
                    throw std::invalid_argument(std::string("HOME='") + value + "' is not an absolute path");
                }
                return value;
            }
            long size = sysconf(_SC_GETPW_R_SIZE_MAX);
            std::vector<char> buffer(size > 0 ? size : 16384);
            passwd entry{};
            passwd* found = nullptr;
            if (getpwuid_r(getuid(), &entry, buffer.data(), buffer.size(), &found) != 0 or found == nullptr)
            {
                throw std::runtime_error("HOME is not set and the password database has no entry for this user");
            }
            return entry.pw_dir;
        }

        // ~ and ~/... with the home expanded (~user stays as it is)
        std::string tilde(const std::string& value)
        {
            if (value == "~" or value.rfind("~/", 0) == 0)
            {
                return home().string() + value.substr(1);
            }
            return value;
        }

        std::filesystem::path xdg(const char* variable, const char* fallback)
        {
            const std::string value = tilde(env(variable));
            return not value.empty() and value[0] == '/' ? std::filesystem::path(value) : home() / fallback;
        }

        // XBLICK_HOME, or empty when it is unset or empty
        std::filesystem::path xblick_home()
        {
            const std::string value = tilde(env("XBLICK_HOME"));
            if (value.empty())
            {
                return {};
            }
            if (value[0] != '/')
            {
                throw std::invalid_argument("XBLICK_HOME='" + env("XBLICK_HOME") + "': give an absolute path");
            }
            return value;
        }

        std::filesystem::path node_dir(std::string_view node, const char* kind, const char* variable,
                                       const char* fallback)
        {
            check_node_name(node);
            const std::filesystem::path base = xblick_home();
            if (not base.empty())
            {
                return clean(base / kind / std::string(node));
            }
            return clean(xdg(variable, fallback) / "xblick" / std::string(node));
        }
    }  // namespace

    void check_node_name(std::string_view node)
    {
        const auto allowed = [](char c, bool first) {
            return (c >= 'a' and c <= 'z') or (c >= '0' and c <= '9') or (not first and (c == '-' or c == '_'));
        };
        bool ok = not node.empty() and node.size() <= 64;
        for (std::size_t i = 0; ok and i < node.size(); ++i)
        {
            ok = allowed(node[i], i == 0);
        }
        if (not ok)
        {
            throw std::invalid_argument("node name '" + std::string(node) + "': lower-case letters, digits, '-' and "
                                        "'_', starting with a letter or a digit, at most 64 characters (yolox, sam3, "
                                        "gem-x)");
        }
        if (node == "xblick" or node.rfind("xblick-", 0) == 0 or node.rfind("xblick_", 0) == 0)
        {
            throw std::invalid_argument("node name '" + std::string(node) + "': the node's short name, without the "
                                        "program's 'xblick-'");
        }
    }

    std::filesystem::path cache_dir(std::string_view node)
    {
        return node_dir(node, "cache", "XDG_CACHE_HOME", ".cache");
    }

    std::filesystem::path data_dir(std::string_view node)
    {
        return node_dir(node, "data", "XDG_DATA_HOME", ".local/share");
    }

    std::filesystem::path config_dir()
    {
        return clean(xdg("XDG_CONFIG_HOME", ".config") / "xblick");
    }
}  // namespace xblick
