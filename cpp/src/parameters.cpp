#include "durchblick/parameters.hpp"

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <set>
#include <sstream>

#include "durchblick/camera.hpp"
#include "durchblick/protocol.hpp"
#include "json.hpp"

namespace durchblick
{
    namespace p = protocol;

    // The parameter messages and their CDR are in ros2_cdr_msgs.hpp.

    // --------------------------------------------------------------------------------------------- checks

    namespace
    {
        // Python's format(v, "g")
        std::string g(double v)
        {
            char text[64];
            std::snprintf(text, sizeof(text), "%g", v);
            return text;
        }

        // Python's repr of a str (single quotes; no escapes beyond those Python makes for printable text)
        std::string repr(const std::string& s)
        {
            const bool single = s.find('\'') != std::string::npos and s.find('"') == std::string::npos;
            const char q = single ? '"' : '\'';
            std::string out(1, q);
            for (const char c : s)
            {
                if (c == q or c == '\\')
                {
                    out += '\\';
                }
                out += c;
            }
            return out + q;
        }

        const std::string kRestart = "read-only: applies at the next start";
        const std::string kReadOnlyAccess = "parameters are read only on this node (--params-read-only)";
    }  // namespace

    // ------------------------------------------------------------------------------------------ YAML (the part
    // params files use: block mappings, scalars, flow lists, comments), as Python's parameters.parse_yaml

    namespace
    {
        std::string trim(const std::string& s)
        {
            const auto b = s.find_first_not_of(" \t\r");
            if (b == std::string::npos)
            {
                return "";
            }
            const auto e = s.find_last_not_of(" \t\r");
            return s.substr(b, e - b + 1);
        }

        std::string strip_comment(const std::string& line)
        {
            char quote = 0;
            for (std::size_t i = 0; i < line.size(); ++i)
            {
                const char c = line[i];
                if (quote)
                {
                    if (c == quote)
                    {
                        quote = 0;
                    }
                }
                else if (c == '\'' or c == '"')
                {
                    quote = c;
                }
                else if (c == '#' and (i == 0 or line[i - 1] == ' ' or line[i - 1] == '\t'))
                {
                    return line.substr(0, i);
                }
            }
            return line;
        }

        std::vector<std::string> split_flow(const std::string& text)
        {
            std::vector<std::string> parts;
            int depth = 0;
            char quote = 0;
            std::size_t start = 0;
            for (std::size_t i = 0; i < text.size(); ++i)
            {
                const char c = text[i];
                if (quote)
                {
                    if (c == quote)
                    {
                        quote = 0;
                    }
                }
                else if (c == '\'' or c == '"')
                {
                    quote = c;
                }
                else if (c == '[')
                {
                    ++depth;
                }
                else if (c == ']')
                {
                    --depth;
                }
                else if (c == ',' and depth == 0)
                {
                    parts.push_back(trim(text.substr(start, i - start)));
                    start = i + 1;
                }
            }
            parts.push_back(trim(text.substr(start)));
            return parts;
        }

        bool is_quoted(const std::string& s)
        {
            return s.size() >= 2 and s.front() == s.back() and (s.front() == '\'' or s.front() == '"');
        }

        std::string unquote(const std::string& s)
        {
            if (not is_quoted(s))
            {
                return s;
            }
            const std::string inner = s.substr(1, s.size() - 2);
            std::string out;
            if (s.front() == '\'')
            {
                for (std::size_t i = 0; i < inner.size(); ++i)
                {
                    out += inner[i];
                    if (inner[i] == '\'' and i + 1 < inner.size() and inner[i + 1] == '\'')
                    {
                        ++i;
                    }
                }
                return out;
            }
            const json::Object o = json::parse("{\"s\":" + s + "}");  // a JSON string, as Python's json.loads
            return json::get<std::string>(o, "s").value_or("");
        }

        std::string lower(std::string s)
        {
            std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
            return s;
        }

        std::optional<std::int64_t> as_integer(const std::string& s)
        {
            std::size_t i = (not s.empty() and (s[0] == '-' or s[0] == '+')) ? 1 : 0;
            if (i == s.size() or (s[i] == '0' and s.size() > i + 1))
            {
                return std::nullopt;
            }
            for (std::size_t k = i; k < s.size(); ++k)
            {
                if (not std::isdigit(static_cast<unsigned char>(s[k])))
                {
                    return std::nullopt;
                }
            }
            std::int64_t v = 0;
            const char* begin = s.data() + (s[0] == '+' ? 1 : 0);
            const auto [end, error] = std::from_chars(begin, s.data() + s.size(), v);
            if (error != std::errc() or end != s.data() + s.size())
            {
                return std::nullopt;
            }
            return v;
        }

        std::optional<double> as_double(const std::string& text)
        {
            const std::string s = lower(text);
            if (s == ".inf" or s == "+.inf")
            {
                return std::numeric_limits<double>::infinity();
            }
            if (s == "-.inf")
            {
                return -std::numeric_limits<double>::infinity();
            }
            if (s == ".nan")
            {
                return std::numeric_limits<double>::quiet_NaN();
            }
            if (s.empty() or s.find_first_not_of("+-0123456789.e") != std::string::npos or
                s.find_first_of("0123456789") == std::string::npos)
            {
                return std::nullopt;
            }
            double v = 0;
            const char* begin = s.data() + (s[0] == '+' ? 1 : 0);
            const auto [end, error] = std::from_chars(begin, s.data() + s.size(), v);
            if (error != std::errc() or end != s.data() + s.size())
            {
                return std::nullopt;
            }
            return v;
        }

        // A plain scalar YAML would read as something else than this string (true, 3, [..], null)
        bool reads_as_other(const std::string& s)
        {
            const std::string l = lower(s);
            return l == "true" or l == "false" or l == "null" or l == "~" or as_integer(s) or as_double(s) or
                   (not s.empty() and s.front() == '[' and s.back() == ']');
        }

        struct YamlNode
        {
            std::string scalar;
            std::vector<std::pair<std::string, YamlNode>> children;
            bool mapping = false;
        };

        YamlNode parse_yaml(const std::string& text, const std::string& where)
        {
            YamlNode root;
            root.mapping = true;
            std::vector<std::pair<int, YamlNode*>> stack{{-1, &root}};
            std::istringstream in(text);
            std::string raw;
            int number = 0;
            while (std::getline(in, raw))
            {
                ++number;
                std::string line = strip_comment(raw);
                while (not line.empty() and (line.back() == ' ' or line.back() == '\r' or line.back() == '\t'))
                {
                    line.pop_back();
                }
                if (trim(line).empty() or trim(line) == "---")
                {
                    continue;
                }
                const std::size_t first = line.find_first_not_of(' ');
                if (line[first] == '\t')
                {
                    throw ParameterError(where + ":" + std::to_string(number) + ": tabs for indentation");
                }
                const int indent = static_cast<int>(first);
                const std::string body = line.substr(first);
                const auto colon = body.find(':');
                if (colon == std::string::npos or (colon + 1 < body.size() and body[colon + 1] != ' '))
                {
                    throw ParameterError(where + ":" + std::to_string(number) + ": 'key: value' expected, not '" +
                                         trim(raw) + "'");
                }
                std::string key = trim(body.substr(0, colon));
                if (is_quoted(key))
                {
                    key = unquote(key);
                }
                const std::string rest = trim(body.substr(colon + 1));
                while (indent <= stack.back().first)
                {
                    stack.pop_back();
                }
                YamlNode* parent = stack.back().second;
                if (not parent->mapping)
                {
                    throw ParameterError(where + ":" + std::to_string(number) + ": a mapping under a value");
                }
                parent->children.emplace_back(key, YamlNode{});
                YamlNode& child = parent->children.back().second;
                if (rest.empty())
                {
                    child.mapping = true;
                    stack.emplace_back(indent, &child);
                }
                else
                {
                    child.scalar = rest;
                }
            }
            return root;
        }

        void flatten(const YamlNode& node, const std::string& prefix,
                     std::vector<std::pair<std::string, std::string>>& out)
        {
            for (const auto& [k, child] : node.children)
            {
                const std::string name = prefix.empty() ? k : prefix + "." + k;
                if (child.mapping)
                {
                    flatten(child, name, out);
                }
                else
                {
                    out.emplace_back(name, child.scalar);
                }
            }
        }

        std::vector<std::string> path_parts(const std::string& s)
        {
            std::vector<std::string> parts;
            std::string part;
            for (const char c : s + "/")
            {
                if (c == '/')
                {
                    if (not part.empty())
                    {
                        parts.push_back(part);
                    }
                    part.clear();
                }
                else
                {
                    part += c;
                }
            }
            return parts;
        }

        bool matches(const std::vector<std::string>& p, std::size_t i, const std::vector<std::string>& n,
                     std::size_t j)
        {
            if (i == p.size())
            {
                return j == n.size();
            }
            if (p[i] == "**")
            {
                for (std::size_t k = j; k <= n.size(); ++k)
                {
                    if (matches(p, i + 1, n, k))
                    {
                        return true;
                    }
                }
                return false;
            }
            return j < n.size() and (p[i] == "*" or p[i] == n[j]) and matches(p, i + 1, n, j + 1);
        }

        std::string yaml_scalar(const ParameterValue& v)
        {
            namespace t = parameter_type;
            const auto number = [](double d) -> std::string {
                if (std::isnan(d))
                {
                    return ".nan";
                }
                if (std::isinf(d))
                {
                    return d > 0 ? ".inf" : "-.inf";
                }
                char text[64];
                const auto [end, error] = std::to_chars(text, text + sizeof(text), d);  // shortest, as Python's repr
                std::string s(text, end);
                if (s.find_first_of(".en") == std::string::npos)
                {
                    s += ".0";
                }
                return s;
            };
            const auto string = [](const std::string& s) -> std::string {
                const bool special = s.empty() or s != trim(s) or reads_as_other(s) or
                                     s.find_first_of(":#[]{},'\"") != std::string::npos;
                return special ? json::quote(s) : s;
            };
            const auto list = [](const auto& items, auto each) {
                std::string out = "[";
                for (std::size_t i = 0; i < items.size(); ++i)
                {
                    out += (i ? ", " : "") + each(items[i]);
                }
                return out + "]";
            };
            switch (v.type)
            {
                case t::kBool: return v.bool_value ? "true" : "false";
                case t::kInteger: return std::to_string(v.integer_value);
                case t::kDouble: return number(v.double_value);
                case t::kString: return string(v.string_value);
                case t::kByteArray: return list(v.byte_array_value, [](std::uint8_t b) { return std::to_string(b); });
                case t::kBoolArray:
                    return list(v.bool_array_value, [](bool b) { return std::string(b ? "true" : "false"); });
                case t::kIntegerArray:
                    return list(v.integer_array_value, [](std::int64_t i) { return std::to_string(i); });
                case t::kDoubleArray: return list(v.double_array_value, number);
                case t::kStringArray: return list(v.string_array_value, string);
                default: return "null";
            }
        }
    }  // namespace

    ParameterValue parameter_from_text(std::uint8_t type, const std::string& given)
    {
        namespace t = parameter_type;
        const std::string s = trim(given);
        const auto items = [&]() -> std::vector<std::string> {
            if (s.size() < 2 or s.front() != '[' or s.back() != ']')
            {
                throw ParameterError("'" + s + "': a list [...] expected");
            }
            const std::string inner = trim(s.substr(1, s.size() - 2));
            return inner.empty() ? std::vector<std::string>{} : split_flow(inner);
        };
        const auto boolean = [](const std::string& x) {
            const std::string l = lower(x);
            if (l != "true" and l != "false")
            {
                throw ParameterError("'" + x + "': true or false expected");
            }
            return l == "true";
        };
        const auto integer = [](const std::string& x) {
            const auto v = as_integer(x);
            if (not v)
            {
                throw ParameterError("'" + x + "': an integer expected");
            }
            return *v;
        };
        const auto real = [](const std::string& x) {
            const auto v = as_double(x);
            if (not v)
            {
                throw ParameterError("'" + x + "': a number expected");
            }
            return *v;
        };
        ParameterValue v;
        v.type = type;
        switch (type)
        {
            case t::kBool: v.bool_value = boolean(s); break;
            case t::kInteger: v.integer_value = integer(s); break;
            case t::kDouble: v.double_value = real(s); break;
            case t::kString: v.string_value = unquote(s); break;
            case t::kBoolArray:
                for (const auto& x : items()) v.bool_array_value.push_back(boolean(x));
                break;
            case t::kIntegerArray:
                for (const auto& x : items()) v.integer_array_value.push_back(integer(x));
                break;
            case t::kDoubleArray:
                for (const auto& x : items()) v.double_array_value.push_back(real(x));
                break;
            case t::kStringArray:
                for (const auto& x : items()) v.string_array_value.push_back(unquote(x));
                break;
            case t::kByteArray:
                for (const auto& x : items()) v.byte_array_value.push_back(static_cast<std::uint8_t>(integer(x)));
                break;
            default: throw ParameterError("a parameter of type " + std::to_string(type) + " has no text form");
        }
        return v;
    }

    std::vector<std::pair<std::string, std::string>> read_params_file(const std::filesystem::path& path,
                                                                      const std::string& node)
    {
        std::ifstream in(path);
        if (not in)
        {
            throw ParameterError(path.string() + ": cannot be read");
        }
        std::stringstream text;
        text << in.rdbuf();
        const YamlNode root = parse_yaml(text.str(), path.string());
        std::vector<std::pair<std::string, std::string>> values;
        for (const auto& [pattern, section] : root.children)
        {
            const auto it = std::find_if(section.children.begin(), section.children.end(),
                                         [](const auto& c) { return c.first == "ros__parameters"; });
            if (not section.mapping or it == section.children.end())
            {
                throw ParameterError(path.string() + ": " + pattern + ": 'ros__parameters:' expected below the node");
            }
            const auto parts = path_parts(pattern), node_parts = path_parts(node);
            // "<name>" alone: that node in any namespace; else /<namespace>/<name> (the / optional), * and **
            const bool bare = parts.size() == 1 and not pattern.starts_with("/") and parts[0] != "*" and
                              parts[0] != "**";
            if (bare ? (not node_parts.empty() and node_parts.back() == parts[0])
                     : matches(parts, 0, node_parts, 0))
            {
                flatten(it->second, "", values);
            }
        }
        return values;
    }

    std::string dump_params_file(const std::map<std::string, std::map<std::string, ParameterValue>>& nodes)
    {
        std::string out;
        for (const auto& [node, values] : nodes)
        {
            out += node + ":\n  ros__parameters:\n";
            std::vector<std::string> open;  // the nested mappings written last
            for (const auto& [name, value] : values)
            {
                std::vector<std::string> parts;
                std::stringstream split(name);
                for (std::string part; std::getline(split, part, '.');)
                {
                    parts.push_back(part);
                }
                std::size_t same = 0;
                while (same < open.size() and same + 1 < parts.size() and open[same] == parts[same])
                {
                    ++same;
                }
                open.resize(same);
                for (std::size_t k = same; k + 1 < parts.size(); ++k)
                {
                    out += std::string(4 + 2 * k, ' ') + parts[k] + ":\n";
                    open.push_back(parts[k]);
                }
                out += std::string(4 + 2 * (parts.size() - 1), ' ') + parts.back() + ": " + yaml_scalar(value) + "\n";
            }
        }
        return out;
    }

    ParameterStartup parameter_args(int argc, char** argv)
    {
        ParameterStartup s;
        for (int i = 1; i < argc; ++i)
        {
            const std::string a = argv[i];
            const auto next = [&]() -> std::string {
                if (i + 1 >= argc)
                {
                    throw ParameterError(a + " needs a value");
                }
                return argv[++i];
            };
            if (a == "--param")
            {
                s.params.push_back(next());
            }
            else if (a == "--params-file")
            {
                s.params_files.emplace_back(next());
            }
            else if (a == "--dump-params")
            {
                s.dump = next();
            }
            else if (a == "--params-read-only")
            {
                s.read_only_access = true;
            }
        }
        return s;
    }

    std::string parameter_text(const ParameterValue& value)
    {
        return yaml_scalar(value);
    }

    std::optional<std::string> check(const ParameterDescriptor& d, const ParameterValue& value,
                                     const std::vector<ParameterValue>& choices)
    {
        namespace t = parameter_type;
        if (value.type != d.type and not d.dynamic_typing)
        {
            return std::string(t::name(value.type)) + " given, the parameter is " + t::name(d.type);
        }
        if (not d.integer_range.empty() and value.type == t::kInteger)
        {
            const IntegerRange& r = d.integer_range[0];
            const std::int64_t v = value.integer_value;
            if (v < r.from_value or v > r.to_value)
            {
                return std::to_string(v) + " is outside " + std::to_string(r.from_value) + " to " +
                       std::to_string(r.to_value);
            }
            if (r.step != 0 and v != r.to_value and static_cast<std::uint64_t>(v - r.from_value) % r.step != 0)
            {
                return std::to_string(v) + " is not " + std::to_string(r.from_value) + " plus a multiple of " +
                       std::to_string(r.step);
            }
        }
        if (not d.floating_point_range.empty() and value.type == t::kDouble)
        {
            const FloatingPointRange& r = d.floating_point_range[0];
            const double v = value.double_value;
            const double tolerance = 1e-9 * std::max({1.0, std::abs(r.from_value), std::abs(r.to_value)});
            if (not(v >= r.from_value - tolerance and v <= r.to_value + tolerance))
            {
                return g(v) + " is outside " + g(r.from_value) + " to " + g(r.to_value);
            }
            if (r.step != 0.0 and std::abs(v - r.to_value) > tolerance)
            {
                const double k = std::round((v - r.from_value) / r.step);
                if (std::abs(r.from_value + k * r.step - v) > tolerance)
                {
                    return g(v) + " is not " + g(r.from_value) + " plus a multiple of " + g(r.step);
                }
            }
        }
        if (not choices.empty() and std::find(choices.begin(), choices.end(), value) == choices.end())
        {
            std::string list;
            for (const ParameterValue& c : choices)
            {
                list += (list.empty() ? "" : ", ") + parameter_text(c);
            }
            return parameter_text(value) + " is none of " + list;
        }
        return std::nullopt;
    }

    // ------------------------------------------------------------------------------------------------- the node

    namespace
    {
        bool valid_name(const std::string& name)
        {
            if (name.empty() or name.front() == '.' or name.back() == '.' or name.find("..") != std::string::npos)
            {
                return false;
            }
            bool start = true;
            for (const char c : name)
            {
                if (c == '.')
                {
                    start = true;
                    continue;
                }
                const bool letter = std::isalpha(static_cast<unsigned char>(c)) or c == '_';
                if (not letter and (start or not std::isdigit(static_cast<unsigned char>(c))))
                {
                    return false;
                }
                start = false;
            }
            return true;
        }

        std::string node_key(const std::string& ns, const std::string& name)
        {
            std::string trimmed = ns;
            while (not trimmed.empty() and trimmed.front() == '/') trimmed.erase(trimmed.begin());
            while (not trimmed.empty() and trimmed.back() == '/') trimmed.pop_back();
            return check_name(trimmed.empty() ? name : trimmed + "/" + name);
        }

        const std::vector<std::string_view>& services()
        {
            static const std::vector<std::string_view> s = {p::kListParameters,  p::kDescribeParameters,
                                                            p::kGetParameters,   p::kGetParameterTypes,
                                                            p::kSetParameters,   p::kSetParametersAtomically};
            return s;
        }

        bool under(const std::string& name, const std::vector<std::string>& prefixes, std::uint64_t depth)
        {
            const auto dots = [](const std::string& s) {
                return static_cast<std::uint64_t>(std::count(s.begin(), s.end(), '.'));
            };
            if (prefixes.empty())
            {
                return depth == 0 or dots(name) < depth;
            }
            for (const std::string& prefix : prefixes)
            {
                if (name == prefix or name.rfind(prefix + ".", 0) == 0)
                {
                    std::string rest = name.substr(prefix.size());
                    if (not rest.empty() and rest.front() == '.')
                    {
                        rest.erase(rest.begin());
                    }
                    return depth == 0 or dots(rest) < depth;
                }
            }
            return false;
        }

        std::string constraints_text(const ParameterOptions& o)
        {
            std::string out;
            if (not o.unit.empty())
            {
                out = "unit: " + o.unit;
            }
            if (not o.choices.empty())
            {
                std::string list;
                for (const ParameterValue& c : o.choices)
                {
                    list += (list.empty() ? "" : ", ") + parameter_text(c);
                }
                out += (out.empty() ? "" : "; ") + std::string("choices: ") + list;
            }
            return out;
        }

        std::string call_write(const Parameters::Writer& write, const ParameterValue& value)
        {
            try
            {
                return write(value);
            }
            catch (const std::exception& e)
            {
                return std::string("the node failed to take it: ") + e.what();
            }
        }
    }  // namespace

    Parameters::Parameters(zenoh::Session* session, const std::string& name, const std::string& ns,
                           bool read_only_access)
        : key_(node_key(ns, name)), name_("/" + key_), read_only_access_(read_only_access)
    {
        if (session == nullptr)
        {
            return;
        }
        zenoh::Session::PublisherOptions options;
        options.encoding = zenoh::Encoding(p::kParameterEventType.encoding());
        events_.emplace(session->declare_publisher(zenoh::KeyExpr(std::string(p::kParameterEvents)),
                                                   std::move(options)));
        for (const std::string_view service : services())
        {
            const std::string s(service);
            queryables_.push_back(session->declare_queryable(
                zenoh::KeyExpr(key_ + "/" + s), [this, s](const zenoh::Query& query) { answer(s, query); },
                zenoh::closures::none));
        }
        token_.emplace(
            session->liveliness_declare_token(zenoh::KeyExpr(key_ + "/" + std::string(p::kParametersToken))));
    }

    Parameters::~Parameters()
    {
        token_.reset();
        queryables_.clear();
        events_.reset();
    }

    void Parameters::configure(const ParameterStartup& startup)
    {
        std::lock_guard lock(mutex_);
        for (const auto& path : startup.params_files)
        {
            for (const auto& [name, text] : read_params_file(path, name_))
            {
                overrides_[name] = {text, path.string()};
            }
        }
        for (const std::string& item : startup.params)
        {
            add_override(item);
        }
        if (startup.dump)
        {
            dump_ = startup.dump;
        }
        read_only_access_ = read_only_access_ or startup.read_only_access;
    }

    void Parameters::add_override(const std::string& assignment, const std::string& where)
    {
        const auto sep = assignment.find(":=");
        if (sep == std::string::npos or not valid_name(assignment.substr(0, sep)))
        {
            throw ParameterError(where + " '" + assignment + "': NAME:=VALUE expected");
        }
        std::lock_guard lock(mutex_);
        overrides_[assignment.substr(0, sep)] = {assignment.substr(sep + 2), where};
    }

    ParameterValue Parameters::declare(const std::string& name, const ParameterValue& default_value,
                                       const ParameterOptions& options, Writer write, Reader read)
    {
        namespace t = parameter_type;
        if (not valid_name(name))
        {
            throw ParameterError("parameter '" + name + "': names of letters, digits and _, parts joined by dots");
        }
        ParameterDescriptor d;
        d.name = name;
        d.type = default_value.type;
        d.description = options.description;
        d.additional_constraints = constraints_text(options);
        d.read_only = options.read_only;
        if (options.floating_range)
        {
            if (d.type != t::kDouble)
            {
                throw ParameterError(name + ": a range is for integer and double parameters");
            }
            d.floating_point_range = {*options.floating_range};
        }
        if (options.integer_range)
        {
            if (d.type != t::kInteger)
            {
                throw ParameterError(name + ": a range is for integer and double parameters");
            }
            d.integer_range = {*options.integer_range};
        }
        if (const auto problem = check(d, default_value, options.choices))
        {
            throw ParameterError(name + ": its default: " + *problem);
        }
        std::lock_guard lock(mutex_);
        if (declared_.count(name))
        {
            throw ParameterError(name + " is declared already");
        }
        ParameterValue value = default_value;
        if (const auto it = overrides_.find(name); it != overrides_.end())
        {
            const auto& [text, where] = it->second;
            try
            {
                value = parameter_from_text(d.type, text);
            }
            catch (const ParameterError& e)
            {
                throw ParameterError(name + " := '" + text + "' (" + where + "): " + e.what());
            }
            if (const auto problem = check(d, value, options.choices))
            {
                throw ParameterError(name + " := '" + text + "' (" + where + "): " + *problem);
            }
        }
        Declared& entry = declared_[name] = Declared{d, value, options.choices, std::move(write), std::move(read)};
        order_.push_back(name);
        publish_event({Parameter{name, current(entry)}}, {});
        return value;
    }

    double Parameters::declare(const std::string& name, double default_value, const ParameterOptions& options,
                               Writer write, Reader read)
    {
        return declare(name, ParameterValue::of(default_value), options, std::move(write), std::move(read))
            .double_value;
    }

    std::int64_t Parameters::declare(const std::string& name, std::int64_t default_value,
                                     const ParameterOptions& options, Writer write, Reader read)
    {
        return declare(name, ParameterValue::of(default_value), options, std::move(write), std::move(read))
            .integer_value;
    }

    bool Parameters::declare(const std::string& name, bool default_value, const ParameterOptions& options,
                             Writer write, Reader read)
    {
        return declare(name, ParameterValue::of(default_value), options, std::move(write), std::move(read))
            .bool_value;
    }

    std::string Parameters::declare(const std::string& name, const std::string& default_value,
                                    const ParameterOptions& options, Writer write, Reader read)
    {
        return declare(name, ParameterValue::of(default_value), options, std::move(write), std::move(read))
            .string_value;
    }

    void Parameters::check_overrides()
    {
        std::lock_guard lock(mutex_);
        std::string unknown;
        for (const auto& [name, given] : overrides_)
        {
            if (not declared_.count(name))
            {
                unknown += (unknown.empty() ? "" : ", ") + name + " (given by " + given.second + ")";
            }
        }
        if (not unknown.empty())
        {
            std::string have;
            for (const auto& [name, _] : declared_)
            {
                have += (have.empty() ? "" : ", ") + name;
            }
            throw ParameterError(name_ + " has no parameter " + unknown + "; it has " +
                                 (have.empty() ? "none" : have));
        }
        if (dump_)
        {
            std::ofstream out(*dump_);
            out << dump();
            if (not out)
            {
                throw ParameterError(dump_->string() + ": cannot be written");
            }
        }
    }

    void Parameters::refresh()
    {
        std::lock_guard lock(mutex_);
        std::vector<Parameter> changed;
        for (const std::string& name : order_)
        {
            Declared& d = declared_.at(name);
            if (not d.read)
            {
                continue;
            }
            const ParameterValue now = current(d);
            if (not(now == d.value))
            {
                d.value = now;
                changed.push_back(Parameter{name, now});
            }
        }
        if (not changed.empty())
        {
            publish_event({}, changed);
        }
    }

    void Parameters::apply_all()
    {
        std::lock_guard lock(mutex_);
        std::string failed;
        for (const std::string& name : order_)  // as declared: a setting's auto_* after it, as its device wants
        {
            const Declared& d = declared_.at(name);
            if (d.write and not d.descriptor.read_only)
            {
                if (const std::string reason = call_write(d.write, d.value); not reason.empty())
                {
                    failed += (failed.empty() ? "" : "; ") + name + ": " + reason;
                }
            }
        }
        if (not failed.empty())
        {
            throw ParameterError(name_ + ": " + failed);
        }
    }

    ParameterValue Parameters::current(const Declared& d) const
    {
        return d.read ? d.read() : d.value;
    }

    ParameterValue Parameters::get(const std::string& name) const
    {
        std::lock_guard lock(mutex_);
        const auto it = declared_.find(name);
        if (it == declared_.end())
        {
            throw ParameterError(name_ + " has no parameter '" + name + "'");
        }
        return current(it->second);
    }

    bool Parameters::has(const std::string& name) const
    {
        std::lock_guard lock(mutex_);
        return declared_.count(name) != 0;
    }

    bool Parameters::overridden(const std::string& name) const
    {
        std::lock_guard lock(mutex_);
        return overrides_.count(name) != 0;
    }

    ParameterDescriptor Parameters::descriptor(const std::string& name) const
    {
        std::lock_guard lock(mutex_);
        const auto it = declared_.find(name);
        if (it == declared_.end())
        {
            throw ParameterError(name_ + " has no parameter '" + name + "'");
        }
        return it->second.descriptor;
    }

    std::vector<std::string> Parameters::names() const
    {
        std::lock_guard lock(mutex_);
        std::vector<std::string> out;
        for (const auto& [name, _] : declared_)
        {
            out.push_back(name);
        }
        return out;
    }

    SetParametersResult Parameters::set(const std::string& name, const ParameterValue& value)
    {
        return set_all({Parameter{name, value}}, false, true).front();
    }

    std::string Parameters::dump() const
    {
        std::lock_guard lock(mutex_);
        std::map<std::string, ParameterValue> values;
        for (const auto& [name, d] : declared_)
        {
            values[name] = current(d);
        }
        return dump_params_file({{name_, values}});
    }

    std::optional<std::string> Parameters::refusal(const Parameter& p, bool local) const
    {
        const auto it = declared_.find(p.name);
        if (it == declared_.end())
        {
            return "no parameter " + repr(p.name);
        }
        if (read_only_access_ and not local)
        {
            return kReadOnlyAccess;
        }
        if (it->second.descriptor.read_only)
        {
            return kRestart;
        }
        return check(it->second.descriptor, p.value, it->second.choices);
    }

    std::vector<SetParametersResult> Parameters::set_all(const std::vector<Parameter>& parameters, bool atomic,
                                                         bool local)
    {
        std::lock_guard lock(mutex_);
        std::vector<std::optional<std::string>> reasons;
        for (const Parameter& p : parameters)
        {
            reasons.push_back(refusal(p, local));
        }
        if (atomic and std::any_of(reasons.begin(), reasons.end(), [](const auto& r) { return r.has_value(); }))
        {
            std::string why;
            for (std::size_t i = 0; i < parameters.size(); ++i)
            {
                if (reasons[i])
                {
                    why += (why.empty() ? "" : "; ") + parameters[i].name + ": " + *reasons[i];
                }
            }
            return {SetParametersResult{false, why}};
        }
        std::map<std::string, ParameterValue> before;
        for (const auto& [name, d] : declared_)
        {
            before[name] = current(d);
        }
        std::vector<SetParametersResult> results;
        for (std::size_t i = 0; i < parameters.size(); ++i)
        {
            std::string reason = reasons[i].value_or("");
            if (reason.empty())
            {
                Declared& d = declared_.at(parameters[i].name);
                if (d.write)
                {
                    reason = call_write(d.write, parameters[i].value);
                }
                if (reason.empty())
                {
                    d.value = parameters[i].value;
                }
            }
            results.push_back(SetParametersResult{reason.empty(), reason});
        }
        // every value that differs now: a write may change others (a control's value switching its auto mode off);
        // those stored as they are now, so that apply_all writes what the device was left with
        std::vector<Parameter> changed;
        for (auto& [name, d] : declared_)
        {
            const ParameterValue now = current(d);
            if (not(now == before[name]))
            {
                changed.push_back(Parameter{name, now});
                d.value = now;
            }
        }
        if (not changed.empty())
        {
            publish_event({}, changed);
        }
        if (atomic)
        {
            std::string failed;
            for (std::size_t i = 0; i < parameters.size(); ++i)
            {
                if (not results[i].successful)
                {
                    failed += (failed.empty() ? "" : "; ") + parameters[i].name + ": " + results[i].reason;
                }
            }
            return {SetParametersResult{failed.empty(), failed}};
        }
        return results;
    }

    void Parameters::publish_event(const std::vector<Parameter>& added, const std::vector<Parameter>& changed)
    {
        if (not events_)
        {
            return;
        }
        ParameterEvent e;
        e.stamp = Time::from_ns(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                    std::chrono::system_clock::now().time_since_epoch())
                                    .count());
        e.node = name_;
        e.new_parameters = added;
        e.changed_parameters = changed;
        events_->put(zenoh::Bytes(e.serialize()));
    }

    ListParametersResponse Parameters::list(const ListParametersRequest& request) const
    {
        std::lock_guard lock(mutex_);
        ListParametersResponse out;
        std::set<std::string> prefixes;
        for (const auto& [name, _] : declared_)
        {
            if (under(name, request.prefixes, request.depth))
            {
                out.names.push_back(name);
                if (const auto dot = name.rfind('.'); dot != std::string::npos)
                {
                    prefixes.insert(name.substr(0, dot));
                }
            }
        }
        out.prefixes.assign(prefixes.begin(), prefixes.end());
        return out;
    }

    DescribeParametersResponse Parameters::describe(const NamesRequest& request) const
    {
        std::lock_guard lock(mutex_);
        DescribeParametersResponse out;
        for (const std::string& n : request.names)
        {
            ParameterDescriptor d;
            if (const auto it = declared_.find(n); it != declared_.end())
            {
                d = it->second.descriptor;
            }
            else
            {
                d.name = n;  // not declared: type not set
            }
            d.read_only = d.read_only or read_only_access_;
            out.descriptors.push_back(d);
        }
        return out;
    }

    GetParametersResponse Parameters::get_parameters(const NamesRequest& request) const
    {
        std::lock_guard lock(mutex_);
        GetParametersResponse out;
        for (const std::string& n : request.names)
        {
            const auto it = declared_.find(n);
            out.values.push_back(it != declared_.end() ? current(it->second) : ParameterValue{});
        }
        return out;
    }

    GetParameterTypesResponse Parameters::types(const NamesRequest& request) const
    {
        std::lock_guard lock(mutex_);
        GetParameterTypesResponse out;
        for (const std::string& n : request.names)
        {
            const auto it = declared_.find(n);
            out.types.push_back(it != declared_.end() ? it->second.descriptor.type : 0);
        }
        return out;
    }

    SetParametersResponse Parameters::set_parameters(const ParametersRequest& request)
    {
        return SetParametersResponse{set_all(request.parameters, false, false)};
    }

    SetParametersAtomicallyResponse Parameters::set_parameters_atomically(const ParametersRequest& request)
    {
        return SetParametersAtomicallyResponse{set_all(request.parameters, true, false).front()};
    }

    void Parameters::answer(const std::string& service, const zenoh::Query& query)
    {
        try
        {
            std::vector<std::uint8_t> request;
            if (const auto payload = query.get_payload())
            {
                request = payload->get().as_vector();
            }
            std::vector<std::uint8_t> response;
            p::MessageType type = p::kListParametersResponseType;
            const auto names = [&] { return request.empty() ? NamesRequest{} : NamesRequest::deserialize(request); };
            const auto parameters = [&] {
                return request.empty() ? ParametersRequest{} : ParametersRequest::deserialize(request);
            };
            if (service == p::kListParameters)
            {
                const ListParametersRequest r =
                    request.empty() ? ListParametersRequest{} : ListParametersRequest::deserialize(request);
                response = list(r).serialize();
            }
            else if (service == p::kDescribeParameters)
            {
                response = describe(names()).serialize();
                type = p::kDescribeParametersResponseType;
            }
            else if (service == p::kGetParameters)
            {
                response = get_parameters(names()).serialize();
                type = p::kGetParametersResponseType;
            }
            else if (service == p::kGetParameterTypes)
            {
                response = types(names()).serialize();
                type = p::kGetParameterTypesResponseType;
            }
            else if (service == p::kSetParameters)
            {
                response = set_parameters(parameters()).serialize();
                type = p::kSetParametersResponseType;
            }
            else
            {
                response = set_parameters_atomically(parameters()).serialize();
                type = p::kSetParametersAtomicallyResponseType;
            }
            zenoh::Query::ReplyOptions options;
            options.encoding = zenoh::Encoding(type.encoding());
            query.reply(query.get_keyexpr(), zenoh::Bytes(std::move(response)), std::move(options));
        }
        catch (const std::exception& e)
        {
            query.reply_err(zenoh::Bytes(name_ + "/" + service + ": " + e.what()));
        }
    }

    // ------------------------------------------------------------------------------------------------ the client

    ParameterClient::ParameterClient(zenoh::Session& session, std::string key, std::uint32_t timeout_ms)
        : session_(session), key_(check_name(std::move(key))), timeout_ms_(timeout_ms)
    {
    }

    std::vector<std::uint8_t> ParameterClient::call(const std::string& service,
                                                    const std::vector<std::uint8_t>& request)
    {
        std::vector<std::vector<std::uint8_t>> ok;
        for (const auto& reply : replies(session_, key_ + "/" + service,
                                         std::chrono::milliseconds(static_cast<std::int64_t>(timeout_ms_)), request))
        {
            if (not reply.is_ok())
            {
                throw ParameterError(reply.get_err().get_payload().as_string());
            }
            ok.push_back(reply.get_ok().get_payload().as_vector());
        }
        if (ok.empty())
        {
            throw ParameterError("no answer from " + key_ + "/" + service + " within " + std::to_string(timeout_ms_) +
                                 " ms");
        }
        if (ok.size() > 1)
        {
            throw ParameterError(std::to_string(ok.size()) + " nodes answer " + key_ + "/" + service +
                                 ": one name, one node");
        }
        return ok.front();
    }

    ListParametersResponse ParameterClient::list(const ListParametersRequest& request)
    {
        return ListParametersResponse::deserialize(call(std::string(p::kListParameters), request.serialize()));
    }

    DescribeParametersResponse ParameterClient::describe(const std::vector<std::string>& names)
    {
        return DescribeParametersResponse::deserialize(
            call(std::string(p::kDescribeParameters), NamesRequest{names}.serialize()));
    }

    GetParametersResponse ParameterClient::get(const std::vector<std::string>& names)
    {
        return GetParametersResponse::deserialize(
            call(std::string(p::kGetParameters), NamesRequest{names}.serialize()));
    }

    GetParameterTypesResponse ParameterClient::types(const std::vector<std::string>& names)
    {
        return GetParameterTypesResponse::deserialize(
            call(std::string(p::kGetParameterTypes), NamesRequest{names}.serialize()));
    }

    SetParametersResponse ParameterClient::set(const std::vector<Parameter>& parameters)
    {
        return SetParametersResponse::deserialize(
            call(std::string(p::kSetParameters), ParametersRequest{parameters}.serialize()));
    }

    SetParametersAtomicallyResponse ParameterClient::set_atomically(const std::vector<Parameter>& parameters)
    {
        return SetParametersAtomicallyResponse::deserialize(
            call(std::string(p::kSetParametersAtomically), ParametersRequest{parameters}.serialize()));
    }
}  // namespace durchblick
