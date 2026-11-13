// Tests of the C++ parameters (parameters.hpp): the rcl_interfaces messages against bytes Python writes (msg.py,
// itself compared with rosbags by tests/test_cdr.py; tests/test_cdr.py::test_cpp_parameter_golden_bytes keeps these
// constants Python's), the checks, values from text, params files and their dump.
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "xblick/parameters.hpp"

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

    std::string hex(const std::vector<std::uint8_t>& bytes)
    {
        static const char* digits = "0123456789abcdef";
        std::string out;
        for (auto b : bytes)
        {
            out += digits[b >> 4];
            out += digits[b & 15];
        }
        return out;
    }

    std::vector<std::uint8_t> unhex(const std::string& text)
    {
        std::vector<std::uint8_t> out;
        for (std::size_t i = 0; i + 1 < text.size(); i += 2)
        {
            out.push_back(static_cast<std::uint8_t>(std::stoi(text.substr(i, 2), nullptr, 16)));
        }
        return out;
    }

    const char* kParameterEvent = "000100000c00000022000000150000002f726f626f742f63616d6572612f63616d65726100000000020000000c0000006578706f737572655f6d73000300000000000000000000000000000000000000000021400100000000000000000000000000000000000000000000000000000008000000656e61626c6564000101000000000000000000000000000000000000000000000100000000000000000000000000000000000000000000000000000002000000050000006d6f6465000400000000000000000000000000000000000000000000050000006661737400000000000000000000000000000000000000000000000004000000696473000700000000000000000000000000000000000000000000000100000000000000000000000000000002000000000000000100000000000000feffffffffffffff000000000000000000000000";
    const char* kDescribe = "00010000020000000c0000006578706f737572655f6d7300030000000e0000006578706f737572652074696d650000000f0000007b22756e6974223a20226d73227d00000000000001000000000000000000e03f0000000000804040000000000000e03f00000000050000006761696e000200000100000000000000010000000001000000000000010000000000000000000000ff000000000000000100000000000000";
    const char* kSetResults = "00010000020000000100000001000000000000000a00000072656164206f6e6c7900";
}  // namespace

int main()
{
    using namespace xblick;
    ParameterEvent e;
    e.stamp = Time{12, 34};
    e.node = "/robot/camera/camera";
    e.new_parameters = {{"exposure_ms", ParameterValue::of(8.5)}, {"enabled", ParameterValue::of(true)}};
    e.changed_parameters = {{"mode", ParameterValue::of("fast")},
                            {"ids", ParameterValue::of(std::vector<std::int64_t>{1, -2})}};
    expect(hex(e.serialize()) == kParameterEvent, "ParameterEvent bytes");
    const ParameterEvent back = ParameterEvent::deserialize(unhex(kParameterEvent));
    expect(back.node == e.node and back.changed_parameters == e.changed_parameters, "ParameterEvent read back");

    DescribeParametersResponse d;
    ParameterDescriptor exposure{"exposure_ms", parameter_type::kDouble, "exposure time", "{\"unit\": \"ms\"}"};
    exposure.floating_point_range = {FloatingPointRange{0.5, 33.0, 0.5}};
    ParameterDescriptor gain{"gain", parameter_type::kInteger};
    gain.read_only = true;
    gain.integer_range = {IntegerRange{0, 255, 1}};
    d.descriptors = {exposure, gain};
    expect(hex(d.serialize()) == kDescribe, "DescribeParameters_Response bytes");
    expect(DescribeParametersResponse::deserialize(unhex(kDescribe)).descriptors == d.descriptors, "descriptors back");
    SetParametersResponse s{{SetParametersResult{true, ""}, SetParametersResult{false, "read only"}}};
    expect(hex(s.serialize()) == kSetResults, "SetParameters_Response bytes");

    // the checks, in Python's words
    expect(check(exposure, ParameterValue::of(8.5)) == std::nullopt, "in range, on a step");
    expect(check(exposure, ParameterValue::of(8.25)).value_or("") == "8.25 is not 0.5 plus a multiple of 0.5",
           "off the step");
    expect(check(exposure, ParameterValue::of(40.0)).value_or("") == "40 is outside 0.5 to 33", "outside");
    expect(check(exposure, ParameterValue::of(std::int64_t{8})).value_or("") ==
               "integer given, the parameter is double", "type");
    expect(check(gain, ParameterValue::of(std::int64_t{256})).value_or("") == "256 is outside 0 to 255", "int range");
    ParameterDescriptor mode{"mode", parameter_type::kString};
    expect(check(mode, ParameterValue::of("slow"), {ParameterValue::of("fast"), ParameterValue::of("accurate")})
                   .value_or("") == "slow is none of fast, accurate", "choices");

    // values from text, a params file, the dump
    expect(parameter_from_text(parameter_type::kDouble, "3").double_value == 3.0, "an integer for a double");
    expect(parameter_from_text(parameter_type::kString, "'a b'").string_value == "a b", "a quoted string");
    expect(parameter_from_text(parameter_type::kStringArray, "[cup, \"a, b\"]").string_array_value ==
               std::vector<std::string>{"cup", "a, b"}, "a string list");
    const auto path = std::filesystem::temp_directory_path() / "xblick_test_parameters.yaml";
    std::ofstream(path) << "/**:\n  ros__parameters:\n    enabled: false  # every node\n"
                           "/robot/camera/camera:\n  ros__parameters:\n    exposure_ms: 12.5\n"
                           "    color:\n      gain: 7\n";
    const auto values = read_params_file(path, "/robot/camera/camera");
    expect(values == std::vector<std::pair<std::string, std::string>>{{"enabled", "false"}, {"exposure_ms", "12.5"},
                                                                     {"color.gain", "7"}}, "params file");
    expect(read_params_file(path, "/robot/other").size() == 1, "only the wildcard for another node");
    std::filesystem::remove(path);
    const std::string dumped = dump_params_file({{"/n", {{"a.b", ParameterValue::of(0.3)},
                                                         {"a.c", ParameterValue::of(std::int64_t{3})},
                                                         {"s", ParameterValue::of("x: y")},
                                                         {"t", ParameterValue::of(true)}}}});
    expect(dumped == "/n:\n  ros__parameters:\n    a:\n      b: 0.3\n      c: 3\n    s: \"x: y\"\n    t: true\n",
           "dump");
    {  // apply_all in declaration order: the value (which switches to manual) before its auto_* bool
        bool auto_mode = false;
        std::int64_t exposure = 0;
        std::vector<std::string> written;
        Parameters params(nullptr, "camera", "t");
        params.declare("color.exposure_time_us", std::int64_t{8000}, {},
                       [&](const ParameterValue& v) {
                           written.push_back("color.exposure_time_us");
                           exposure = v.integer_value;
                           auto_mode = false;
                           return std::string();
                       },
                       [&] { return ParameterValue::of(exposure); });
        params.declare("color.auto_exposure", true, {}, [&](const ParameterValue& v) {
            written.push_back("color.auto_exposure");
            auto_mode = v.bool_value;
            return std::string();
        }, [&] { return ParameterValue::of(auto_mode); });
        params.check_overrides();
        params.apply_all();
        expect(written == std::vector<std::string>{"color.exposure_time_us", "color.auto_exposure"},
               "apply_all in declaration order");
        expect(auto_mode and exposure == 8000, "apply_all: auto after the value");
        // a value set later switches to manual; the auto bool, changed by that write, is stored as it is now
        expect(params.set("color.exposure_time_us", ParameterValue::of(std::int64_t{9000})).successful, "set");
        expect(not auto_mode and not params.get("color.auto_exposure").bool_value, "manual after setting the value");
        params.apply_all();
        expect(not auto_mode and exposure == 9000, "apply_all keeps manual: the bool's stored value followed");
        exposure = 7000;  // the device changed it by itself: refresh stores it
        params.refresh();
        exposure = 0;
        params.apply_all();
        expect(exposure == 7000, "refresh stores what the readers say");
    }
    if (failures == 0)
    {
        std::printf("parameters: all passed\n");
    }
    return failures == 0 ? 0 : 1;
}
