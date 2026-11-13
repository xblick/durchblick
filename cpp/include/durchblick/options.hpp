// A program's command-line options, each defined once (name, default and so type, help, live or not), parsed, and
// offered as the node's parameters: Python's argparse with Parameters.declare_options in one
// (docs/reference/parameters.md, "A node's options as parameters").
//
//   durchblick::Options options("durchblick-relay", "relays image streams of other PCs into this PC's rings");
//   options.add_list("stream", {}, {.help = "the streams", .metavar = "NAME", .required = true});
//   options.add("slots", std::int64_t{32}, {.help = "ring slots", .parameter = {.integer_range = IntegerRange{2, 256, 1}}});
//   options.add("idle-ms", std::int64_t{2000}, {.help = "...", .live = true, .parameter = {.unit = "ms"}});
//   options.add_flag("prefer-raw", {.help = "..."});
//   options.add_optional("cuda-device", parameter_type::kInteger, {.help = "the GPU"});   // no default
//   options.add_session_options();     // --connect, --listen, --zenoh-mode, --zenoh-config
//   options.add_parameter_options();   // --param, --params-file, --dump-params, --params-read-only
//   options.parse(argc, argv);         // --help: the help, exit 0; a wrong option: why and the usage, exit 2
//   zenoh::Session session = open_session(options.session_options());
//   Parameters params(session, "relay", "robot");
//   params.configure(options.parameter_startup());
//   options.declare(params);           // every option a parameter: read-only, but the live ones
//   params.check_overrides();
//   options.integer("slots");          // its value (given at the start, also by --param or a params file)
//   params.get("idle_ms")              // a live one's value now
//
// A parameter is named after its option with '_' for '-' (--idle-ms: idle_ms). Kinds: a flag (bool, no value), an
// integer, a number (double), a text, a list of texts (zero or more values; repeated, it adds), and each of the first
// four without a default (add_optional: not set until given; its parameter a read-only string, empty when not given).
// An option given twice (but a list), with a value it does not take, or unknown is an error, never ignored.
#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "durchblick/camera.hpp"
#include "durchblick/parameters.hpp"

namespace durchblick
{
    struct OptionSpec
    {
        std::string help{};
        std::string metavar{};               // the value's name in the help (default: the kind's, e.g. N or TEXT)
        bool required = false;             // given or an error (a list: at least one value)
        bool live = false;                 // its parameter changes while the node runs (read where it applies)
        ParameterOptions parameter{};      // unit, choices, range of its parameter (also checked at parse)
        std::vector<std::string> aliases{};  // other names (without --), e.g. {"streams"}
    };

    class Options
    {
    public:
        explicit Options(std::string program, std::string summary = "");

        // An option of the type of `default_value`: bool (a flag: false unless given), integer, double, string.
        Options& add(const std::string& name, const ParameterValue& default_value, OptionSpec spec = {});
        Options& add(const std::string& name, std::int64_t default_value, OptionSpec spec = {});
        Options& add(const std::string& name, double default_value, OptionSpec spec = {});
        Options& add(const std::string& name, const std::string& default_value, OptionSpec spec = {});
        Options& add(const std::string& name, const char* default_value, OptionSpec spec = {});
        Options& add_flag(const std::string& name, OptionSpec spec = {});
        Options& add_list(const std::string& name, std::vector<std::string> default_value, OptionSpec spec = {});
        // An option without a default (integer, double or string): not set until given.
        Options& add_optional(const std::string& name, std::uint8_t type, OptionSpec spec = {});
        // --connect ENDPOINT..., --listen ENDPOINT..., --zenoh-mode peer|client, --zenoh-config FILE: the Zenoh
        // session's (session_options); not parameters.
        Options& add_session_options();
        // --param NAME:=VALUE (repeated), --params-file FILE (repeated), --dump-params FILE, --params-read-only
        // (parameter_startup); not parameters.
        Options& add_parameter_options();
        // Another name of an option defined already (e.g. a program's older --mode for --zenoh-mode).
        Options& alias(const std::string& name, const std::string& alias);

        // Reads the command line. --help prints help() and ends the program (0); a wrong command line prints why
        // and the usage and ends it (2). parse_or_throw: OptionsError instead (tests, programs with their own exit).
        void parse(int argc, const char* const* argv);
        void parse_or_throw(int argc, const char* const* argv);

        // Every option but the session's and the parameters' own as a parameter of `params` (after its configure):
        // read-only, but the live ones (with their writer from `writers`, by parameter name); the description is the
        // option's help. A value given at the start (--param, a params file) becomes the option's value. Left out:
        // the names in `leave_out` (option or parameter names) and parameters declared already. Returns the names.
        std::vector<std::string> declare(Parameters& params, const std::map<std::string, Parameters::Writer>& writers = {},
                                         const std::vector<std::string>& leave_out = {});

        bool flag(const std::string& name) const;
        std::int64_t integer(const std::string& name) const;
        double number(const std::string& name) const;
        std::string text(const std::string& name) const;
        std::vector<std::string> list(const std::string& name) const;
        bool has_value(const std::string& name) const;  // an add_optional one given (or set by a parameter)
        bool given(const std::string& name) const;      // on the command line
        ParameterValue value(const std::string& name) const;

        SessionOptions session_options() const;
        ParameterStartup parameter_startup() const;
        std::string help() const;
        std::string usage() const;

        static std::string parameter_name(const std::string& option);  // idle-ms -> idle_ms

    private:
        enum class Kind
        {
            flag,
            integer,
            number,
            text,
            list
        };
        struct Option
        {
            std::string name;
            Kind kind;
            ParameterValue value;  // kNotSet: an optional one not given
            bool optional = false;
            bool given = false;
            bool internal = false;  // the session's and the parameters' own: no parameter
            OptionSpec spec;
        };
        Option& define(const std::string& name, Kind kind, ParameterValue value, OptionSpec spec, bool optional);
        const Option& find(const std::string& name) const;
        const Option& checked(const std::string& name, Kind kind) const;
        ParameterValue convert(const Option& option, const std::string& text) const;
        void check_value(const Option& option, const ParameterValue& value) const;

        std::string program_, summary_;
        std::vector<Option> options_;
        std::map<std::string, std::size_t> by_name_;  // names and aliases -> index
    };

    class OptionsError : public std::runtime_error
    {
    public:
        using std::runtime_error::runtime_error;
    };
}  // namespace durchblick
