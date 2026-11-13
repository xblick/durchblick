// durchblick::Options (options.hpp): a command line parsed, wrong ones refused with why, and the options as a node's
// parameters (read-only but the live ones, a value given at the start written back), without a Zenoh session.
#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

#include "durchblick/options.hpp"

namespace
{
    int failures = 0;

    void expect(bool condition, const char* what)
    {
        if (not condition)
        {
            std::fprintf(stderr, "FAILED: %s\n", what);
            ++failures;
        }
    }

    durchblick::Options relay_like()
    {
        using namespace durchblick;
        Options o("durchblick-test", "a test program");
        o.add_list("stream", {}, {.help = "the streams", .metavar = "NAME", .required = true, .aliases = {"streams"}});
        o.add("slots", std::int64_t{32}, {.help = "ring slots", .parameter = {.integer_range = IntegerRange{2, 256, 1}}});
        o.add("idle-ms", std::int64_t{2000}, {.help = "silence before idle", .live = true, .parameter = {.unit = "ms"}});
        o.add("gain", 1.5, {.help = "a gain", .live = true});
        o.add("mode", "fast", {.help = "how", .parameter = {.choices = {ParameterValue::of("fast"),
                                                                         ParameterValue::of("exact")}}});
        o.add_flag("prefer-raw", {.help = "raw frames"});
        o.add_optional("cuda-device", parameter_type::kInteger, {.help = "the GPU"});
        o.add_session_options();
        o.add_parameter_options();
        return o;
    }

    bool refused(const std::vector<const char*>& argv, const std::string& part)
    {
        auto o = relay_like();
        try
        {
            o.parse_or_throw(static_cast<int>(argv.size()), argv.data());
        }
        catch (const durchblick::OptionsError& error)
        {
            if (std::string(error.what()).find(part) == std::string::npos)
            {
                std::fprintf(stderr, "refused with '%s', expected '%s'\n", error.what(), part.c_str());
                return false;
            }
            return true;
        }
        return false;
    }
}  // namespace

int main()
{
    using namespace durchblick;
    {
        auto o = relay_like();
        const std::vector<const char*> argv{"x", "--stream", "a/b", "c/d", "--slots", "64", "--prefer-raw",
                                            "--connect", "h1", "h2", "--param", "gain:=2.5", "--zenoh-mode=client",
                                            "--cuda-device", "1", "--params-file", "/tmp/none.yaml"};
        o.parse_or_throw(static_cast<int>(argv.size()), argv.data());
        expect(o.list("stream") == std::vector<std::string>{"a/b", "c/d"}, "a list takes the values that follow");
        expect(o.integer("slots") == 64 and o.flag("prefer-raw") and o.text("mode") == "fast", "values and defaults");
        expect(o.has_value("cuda-device") and o.integer("cuda-device") == 1, "an optional one given");
        const SessionOptions s = o.session_options();
        expect(s.connect == std::vector<std::string>{"h1", "h2"} and s.mode == "client", "the session's options");
        const ParameterStartup start = o.parameter_startup();
        expect(start.params == std::vector<std::string>{"gain:=2.5"} and start.params_files.size() == 1,
               "the parameters' own options");

        Parameters params(nullptr, "test", "robot/cam");
        ParameterStartup only_param;
        only_param.params = start.params;
        params.configure(only_param);
        const auto names = o.declare(params, {}, {"stream"});
        expect(std::find(names.begin(), names.end(), "idle_ms") != names.end() and
                   std::find(names.begin(), names.end(), "stream") == names.end() and
                   std::find(names.begin(), names.end(), "connect") == names.end() and
                   std::find(names.begin(), names.end(), "param") == names.end(),
               "the options as parameters: named with _, the session's and the parameters' own left out");
        params.check_overrides();
        expect(o.number("gain") == 2.5, "a value given with --param written back into the option");
        expect(params.descriptor("slots").read_only and not params.descriptor("idle_ms").read_only and
                   not params.descriptor("gain").read_only,
               "read-only but the live ones");
        expect(params.descriptor("slots").description == "--slots: ring slots", "the help as description");
        expect(params.get("cuda_device").string_value == "1" and params.descriptor("cuda_device").read_only,
               "an optional one: a read-only string");
        expect(params.set("idle_ms", ParameterValue::of(std::int64_t{500})).successful and
                   params.get("idle_ms").integer_value == 500,
               "a live one changes");
        expect(not params.set("slots", ParameterValue::of(std::int64_t{8})).successful, "a read-only one does not");
    }
    {
        auto o = relay_like();
        const std::vector<const char*> argv{"x", "--streams", "a/b"};
        o.parse_or_throw(static_cast<int>(argv.size()), argv.data());
        expect(not o.has_value("cuda-device") and o.list("stream").size() == 1, "an alias; an optional one not given");
        Parameters params(nullptr, "test", "");
        ParameterStartup with_device;
        with_device.params = {"cuda_device:=3"};
        params.configure(with_device);
        o.declare(params);
        expect(o.has_value("cuda-device") and o.integer("cuda-device") == 3,
               "an optional one set at the start by --param, converted to its type");
    }
    expect(refused({"x"}, "--stream is required"), "a required one missing");
    expect(refused({"x", "--stream", "a", "--slots", "1"}, "--slots"), "outside its range");
    expect(refused({"x", "--stream", "a", "--slots", "many"}, "an integer expected"), "not an integer");
    expect(refused({"x", "--stream", "a", "--mode", "slow"}, "--mode"), "not one of its choices");
    expect(refused({"x", "--stream", "a", "--slot", "3"}, "unknown option --slot"), "a typo");
    expect(refused({"x", "--stream", "a", "--slots", "3", "--slots", "4"}, "given twice"), "given twice");
    expect(refused({"x", "--stream", "a", "--prefer-raw=yes"}, "takes no value"), "a value for a flag");
    expect(refused({"x", "--stream", "a", "--slots"}, "needs a value"), "no value");
    expect(refused({"x", "stray", "--stream", "a"}, "unexpected argument"), "a stray argument");
    {
        auto o = relay_like();
        const std::vector<const char*> argv{"x", "--stream", "a", "--gain", "-0.5"};
        o.parse_or_throw(static_cast<int>(argv.size()), argv.data());
        expect(o.number("gain") == -0.5, "a negative number is a value, not an option");
        expect(o.help().find("--idle-ms N") != std::string::npos and o.help().find("(live)") != std::string::npos and
                   o.help().find("(default 32)") != std::string::npos,
               "the help lists the options, their defaults and which are live");
    }
    bool live_optional_refused = false;
    try
    {
        Options o("x");
        o.add_optional("gpu", parameter_type::kInteger, {.live = true});
    }
    catch (const std::logic_error&)
    {
        live_optional_refused = true;
    }
    expect(live_optional_refused, "a live option needs a default");
    if (failures == 0)
    {
        std::puts("options: all passed");
    }
    return failures == 0 ? 0 : 1;
}
