// A program's command-line options as the node's parameters (options.hpp).
#include "xblick/options.hpp"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cstdlib>
#include <iostream>
#include <sstream>

namespace xblick
{
    namespace
    {
        namespace T = parameter_type;

        std::string default_metavar(std::uint8_t type)
        {
            switch (type)
            {
                case T::kInteger:
                    return "N";
                case T::kDouble:
                    return "X";
                default:
                    return "TEXT";
            }
        }

        ParameterDescriptor descriptor_of(std::uint8_t type, const ParameterOptions& options)
        {
            ParameterDescriptor d;
            d.type = type;
            if (options.integer_range)
            {
                d.integer_range = {*options.integer_range};
            }
            if (options.floating_range)
            {
                d.floating_point_range = {*options.floating_range};
            }
            return d;
        }
    }  // namespace

    Options::Options(std::string program, std::string summary)
        : program_(std::move(program)), summary_(std::move(summary))
    {
    }

    std::string Options::parameter_name(const std::string& option)
    {
        std::string out = option;
        std::replace(out.begin(), out.end(), '-', '_');
        return out;
    }

    Options::Option& Options::define(const std::string& name, Kind kind, ParameterValue value, OptionSpec spec,
                                     bool optional)
    {
        for (const std::string& n : [&] {
                 std::vector<std::string> all{name};
                 all.insert(all.end(), spec.aliases.begin(), spec.aliases.end());
                 return all;
             }())
        {
            if (n.empty() or n.rfind("-", 0) == 0)
            {
                throw std::logic_error("option '" + n + "': a name without leading dashes");
            }
            if (by_name_.contains(n))
            {
                throw std::logic_error("option --" + n + " defined twice");
            }
        }
        if (spec.live and optional)
        {
            throw std::logic_error("option --" + name + ": a live one needs a default (add, not add_optional)");
        }
        options_.push_back(Option{name, kind, std::move(value), optional, false, false, std::move(spec)});
        const std::size_t index = options_.size() - 1;
        by_name_[name] = index;
        for (const std::string& alias : options_.back().spec.aliases)
        {
            by_name_[alias] = index;
        }
        if (not optional and kind != Kind::list and kind != Kind::flag)
        {
            check_value(options_.back(), options_.back().value);  // a default outside its own range is a bug
        }
        return options_.back();
    }

    Options& Options::add(const std::string& name, const ParameterValue& default_value, OptionSpec spec)
    {
        switch (default_value.type)
        {
            case T::kBool:
                if (default_value.bool_value)
                {
                    throw std::logic_error("option --" + name + ": a flag is false unless given (name it no-...)");
                }
                define(name, Kind::flag, default_value, std::move(spec), false);
                break;
            case T::kInteger:
                define(name, Kind::integer, default_value, std::move(spec), false);
                break;
            case T::kDouble:
                define(name, Kind::number, default_value, std::move(spec), false);
                break;
            case T::kString:
                define(name, Kind::text, default_value, std::move(spec), false);
                break;
            case T::kStringArray:
                define(name, Kind::list, default_value, std::move(spec), false);
                break;
            default:
                throw std::logic_error("option --" + name + ": a bool, integer, double, string or string list");
        }
        return *this;
    }

    Options& Options::add(const std::string& name, std::int64_t default_value, OptionSpec spec)
    {
        return add(name, ParameterValue::of(default_value), std::move(spec));
    }

    Options& Options::add(const std::string& name, double default_value, OptionSpec spec)
    {
        return add(name, ParameterValue::of(default_value), std::move(spec));
    }

    Options& Options::add(const std::string& name, const std::string& default_value, OptionSpec spec)
    {
        return add(name, ParameterValue::of(default_value), std::move(spec));
    }

    Options& Options::add(const std::string& name, const char* default_value, OptionSpec spec)
    {
        return add(name, ParameterValue::of(std::string(default_value)), std::move(spec));
    }

    Options& Options::add_flag(const std::string& name, OptionSpec spec)
    {
        return add(name, ParameterValue::of(false), std::move(spec));
    }

    Options& Options::add_list(const std::string& name, std::vector<std::string> default_value, OptionSpec spec)
    {
        return add(name, ParameterValue::of(default_value), std::move(spec));
    }

    Options& Options::add_optional(const std::string& name, std::uint8_t type, OptionSpec spec)
    {
        const Kind kind = type == T::kInteger  ? Kind::integer
                          : type == T::kDouble ? Kind::number
                          : type == T::kString ? Kind::text
                                               : throw std::logic_error("option --" + name +
                                                                        ": an optional one is an integer, double or "
                                                                        "string");
        Option& o = define(name, kind, ParameterValue{}, std::move(spec), true);  // not set: its kind says its type
        o.spec.metavar = o.spec.metavar.empty() ? default_metavar(type) : o.spec.metavar;
        return *this;
    }

    Options& Options::add_session_options()
    {
        add_list("connect", {}, {.help = "Zenoh endpoints to connect to: host, host:port or a locator like "
                                         "tcp/192.0.2.10:7447 (default, without --listen: this PC's router)",
                                 .metavar = "ENDPOINT"});
        add_list("listen", {}, {.help = "Zenoh endpoints to listen on, e.g. 0.0.0.0:7448", .metavar = "ENDPOINT"});
        add("zenoh-mode", "peer",
            {.help = "peer, or client (everything through the router --connect names)",
             .parameter = {.choices = {ParameterValue::of("peer"), ParameterValue::of("client")}}});
        add("zenoh-config", "", {.help = "a Zenoh JSON5 configuration file instead of the endpoints", .metavar = "FILE"});
        for (const char* name : {"connect", "listen", "zenoh-mode", "zenoh-config"})
        {
            options_[by_name_.at(name)].internal = true;
        }
        return *this;
    }

    Options& Options::add_parameter_options()
    {
        add_list("param", {}, {.help = "a parameter's value at the start, as ROS's -p (repeat for more)",
                               .metavar = "NAME:=VALUE"});
        add_list("params-file", {}, {.help = "a ROS params file (<node>: ros__parameters: ...); repeated, the later win",
                                     .metavar = "FILE"});
        add("dump-params", "", {.help = "write the parameters in effect at the start to this params file",
                                .metavar = "FILE"});
        add_flag("params-read-only", {.help = "parameters can be read over Zenoh, not changed"});
        for (const char* name : {"param", "params-file", "dump-params", "params-read-only"})
        {
            options_[by_name_.at(name)].internal = true;
        }
        return *this;
    }

    Options& Options::alias(const std::string& name, const std::string& alias)
    {
        const auto it = by_name_.find(name);
        if (it == by_name_.end())
        {
            throw std::logic_error("alias " + alias + ": no option --" + name);
        }
        if (by_name_.contains(alias) or alias.empty() or alias.rfind("-", 0) == 0)
        {
            throw std::logic_error("alias --" + alias + ": taken or malformed");
        }
        by_name_[alias] = it->second;
        options_[it->second].spec.aliases.push_back(alias);
        return *this;
    }

    // ---------------------------------------------------------------------------------------------------- parsing

    ParameterValue Options::convert(const Option& option, const std::string& text) const
    {
        const std::string flag = "--" + option.name;
        switch (option.kind)
        {
            case Kind::integer:
            {
                std::int64_t v = 0;
                const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), v);
                if (error != std::errc() or end != text.data() + text.size())
                {
                    throw OptionsError(flag + " " + text + ": an integer expected");
                }
                return ParameterValue::of(v);
            }
            case Kind::number:
            {
                char* end = nullptr;
                const double v = std::strtod(text.c_str(), &end);
                if (text.empty() or end != text.c_str() + text.size())
                {
                    throw OptionsError(flag + " " + text + ": a number expected");
                }
                return ParameterValue::of(v);
            }
            case Kind::text:
                return ParameterValue::of(text);
            default:
                throw std::logic_error("convert: " + flag);
        }
    }

    void Options::check_value(const Option& option, const ParameterValue& value) const
    {
        const auto problem =
            check(descriptor_of(value.type, option.spec.parameter), value, option.spec.parameter.choices);
        if (problem)
        {
            throw OptionsError("--" + option.name + ": " + *problem);
        }
    }

    void Options::parse(int argc, const char* const* argv)
    {
        try
        {
            parse_or_throw(argc, argv);
        }
        catch (const OptionsError& error)
        {
            std::cerr << program_ << ": " << error.what() << "\n" << usage() << std::flush;
            std::exit(2);
        }
    }

    void Options::parse_or_throw(int argc, const char* const* argv)
    {
        Option* list = nullptr;  // the list option that takes the values that follow
        for (int i = 1; i < argc; ++i)
        {
            std::string arg = argv[i];
            const bool negative_number = arg.size() > 1 and arg[0] == '-' and
                                         (std::isdigit(static_cast<unsigned char>(arg[1])) or arg[1] == '.');
            if (arg.rfind("--", 0) != 0 or arg == "--")
            {
                if (arg == "-h")
                {
                    std::cout << help() << std::flush;
                    std::exit(0);
                }
                if (list == nullptr or (arg.rfind("-", 0) == 0 and not negative_number))
                {
                    throw OptionsError("unexpected argument '" + arg + "'");
                }
                list->value.string_array_value.push_back(arg);
                continue;
            }
            list = nullptr;
            std::optional<std::string> inline_value;
            if (const auto eq = arg.find('='); eq != std::string::npos)
            {
                inline_value = arg.substr(eq + 1);
                arg = arg.substr(0, eq);
            }
            const std::string name = arg.substr(2);
            if (name == "help")
            {
                std::cout << help() << std::flush;
                std::exit(0);
            }
            const auto it = by_name_.find(name);
            if (it == by_name_.end())
            {
                throw OptionsError("unknown option " + arg);
            }
            Option& o = options_[it->second];
            if (o.kind == Kind::list)
            {
                if (not o.given)
                {
                    o.value.string_array_value.clear();  // given: its values replace the default
                }
                o.given = true;
                if (inline_value)
                {
                    o.value.string_array_value.push_back(*inline_value);
                }
                list = &o;
                continue;
            }
            if (o.given)
            {
                throw OptionsError("--" + o.name + " given twice");
            }
            o.given = true;
            if (o.kind == Kind::flag)
            {
                if (inline_value)
                {
                    throw OptionsError("--" + o.name + " takes no value");
                }
                o.value = ParameterValue::of(true);
                continue;
            }
            std::string text;
            if (inline_value)
            {
                text = *inline_value;
            }
            else
            {
                if (i + 1 >= argc)
                {
                    throw OptionsError("--" + o.name + " needs a value");
                }
                const std::string next = argv[i + 1];
                const bool next_negative = next.size() > 1 and next[0] == '-' and
                                           (std::isdigit(static_cast<unsigned char>(next[1])) or next[1] == '.');
                if (next.rfind("-", 0) == 0 and not next_negative)
                {
                    throw OptionsError("--" + o.name + " needs a value");
                }
                text = argv[++i];
            }
            o.value = convert(o, text);
            check_value(o, o.value);
        }
        for (const Option& o : options_)
        {
            if (o.spec.required and (not o.given or (o.kind == Kind::list and o.value.string_array_value.empty())))
            {
                throw OptionsError("--" + o.name + " is required");
            }
        }
    }

    // ------------------------------------------------------------------------------------------- the parameters

    std::vector<std::string> Options::declare(Parameters& params, const std::map<std::string, Parameters::Writer>& writers,
                                              const std::vector<std::string>& leave_out)
    {
        for (const auto& [name, _] : writers)
        {
            const auto it = std::find_if(options_.begin(), options_.end(),
                                         [&](const Option& o) { return parameter_name(o.name) == name; });
            if (it == options_.end() or not it->spec.live)
            {
                throw std::logic_error("a writer for " + name + ", which is no live option of " + program_);
            }
        }
        std::vector<std::string> declared;
        for (Option& o : options_)
        {
            const std::string name = parameter_name(o.name);
            const bool left_out = std::find(leave_out.begin(), leave_out.end(), o.name) != leave_out.end() or
                                  std::find(leave_out.begin(), leave_out.end(), name) != leave_out.end();
            if (o.internal or left_out or params.has(name))
            {
                continue;
            }
            ParameterOptions p = o.spec.parameter;
            p.description = "--" + o.name + (o.spec.help.empty() ? "" : ": " + o.spec.help);
            p.read_only = not o.spec.live;
            const bool unset = o.optional;  // its parameter: a string, empty when not given
            const ParameterValue start = unset ? ParameterValue::of(o.value.type == T::kNotSet ? std::string()
                                                                                                 : parameter_text(o.value))
                                               : o.value;
            if (unset)
            {
                p.choices.clear();
                p.integer_range.reset();
                p.floating_range.reset();
            }
            const auto writer = writers.find(name);
            params.declare(name, start, p, writer != writers.end() ? writer->second : Parameters::Writer{});
            declared.push_back(name);
            const ParameterValue now = params.get(name);  // a value given at the start, else the option's
            if (unset)
            {
                if (now.string_value.empty())
                {
                    o.value = ParameterValue{};  // not set (also when a params file empties one given)
                }
                else if (o.value.type == T::kNotSet or now.string_value != parameter_text(o.value))
                {
                    o.value = convert(o, now.string_value);
                    check_value(o, o.value);
                }
            }
            else
            {
                o.value = now;
            }
        }
        return declared;
    }

    // ----------------------------------------------------------------------------------------------- the values

    const Options::Option& Options::find(const std::string& name) const
    {
        const auto it = by_name_.find(name);
        if (it == by_name_.end())
        {
            throw std::logic_error(program_ + ": no option --" + name);
        }
        return options_[it->second];
    }

    const Options::Option& Options::checked(const std::string& name, Kind kind) const
    {
        const Option& o = find(name);
        if (o.kind != kind)
        {
            throw std::logic_error(program_ + ": --" + name + " is of another kind");
        }
        if (o.optional and o.value.type == T::kNotSet)
        {
            throw std::logic_error(program_ + ": --" + name + " has no value (has_value first)");
        }
        return o;
    }

    bool Options::flag(const std::string& name) const { return checked(name, Kind::flag).value.bool_value; }
    std::int64_t Options::integer(const std::string& name) const { return checked(name, Kind::integer).value.integer_value; }
    double Options::number(const std::string& name) const { return checked(name, Kind::number).value.double_value; }
    std::string Options::text(const std::string& name) const { return checked(name, Kind::text).value.string_value; }
    std::vector<std::string> Options::list(const std::string& name) const
    {
        return checked(name, Kind::list).value.string_array_value;
    }
    bool Options::has_value(const std::string& name) const { return find(name).value.type != T::kNotSet; }
    bool Options::given(const std::string& name) const { return find(name).given; }
    ParameterValue Options::value(const std::string& name) const { return find(name).value; }

    SessionOptions Options::session_options() const
    {
        SessionOptions s;
        s.connect = list("connect");
        s.listen = list("listen");
        s.mode = text("zenoh-mode");
        s.config_file = text("zenoh-config");
        return s;
    }

    ParameterStartup Options::parameter_startup() const
    {
        ParameterStartup s;
        for (const std::string& path : list("params-file"))
        {
            s.params_files.emplace_back(path);
        }
        s.params = list("param");
        if (const std::string dump = text("dump-params"); not dump.empty())
        {
            s.dump = dump;
        }
        s.read_only_access = flag("params-read-only");
        return s;
    }

    // ------------------------------------------------------------------------------------------------- the help

    std::string Options::usage() const
    {
        std::ostringstream out;
        out << "usage: " << program_;
        for (const Option& o : options_)
        {
            if (o.spec.required)
            {
                out << " --" << o.name << " " << (o.spec.metavar.empty() ? "VALUE" : o.spec.metavar)
                    << (o.kind == Kind::list ? " ..." : "");
            }
        }
        out << " [options] (--help: all)\n";
        return out.str();
    }

    std::string Options::help() const
    {
        std::ostringstream out;
        out << usage();
        if (not summary_.empty())
        {
            out << "\n" << summary_ << "\n";
        }
        out << "\noptions (each also a parameter NAME with _ for -, read-only but those marked live):\n";
        for (const Option& o : options_)
        {
            std::string flags = "--" + o.name;
            for (const std::string& alias : o.spec.aliases)
            {
                flags += ", --" + alias;
            }
            if (o.kind != Kind::flag)
            {
                flags += " " + (o.spec.metavar.empty() ? default_metavar(o.value.type) : o.spec.metavar) +
                         (o.kind == Kind::list ? " ..." : "");
            }
            out << "  " << flags << "\n      " << o.spec.help;
            if (o.kind != Kind::flag and o.kind != Kind::list and not o.optional)
            {
                out << " (default " << parameter_text(o.value) << ")";
            }
            if (not o.spec.parameter.unit.empty())
            {
                out << " [" << o.spec.parameter.unit << "]";
            }
            if (o.spec.live)
            {
                out << " (live)";
            }
            out << "\n";
        }
        return out.str();
    }
}  // namespace xblick
