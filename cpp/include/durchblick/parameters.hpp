// A node's parameters, as ROS 2's: rcl_interfaces messages in CDR (byte-identical to ROS 2), ROS 2's parameter
// services as Zenoh queryables below the node, ParameterEvents on change, and values at the start from --param
// name:=value and ROS params files. The same model, keys and checks as Python's blick.parameters (its module
// docstring and docs/reference/parameters.md are the reference):
//
//   durchblick::Parameters params(session, "camera", "robot/camera");   // the node <namespace>/<name>
//   params.configure(durchblick::parameter_args(argc, argv));                // --param, --params-file, ...
//   params.declare("color.exposure_time_us", std::int64_t{8000},
//       {.description = "exposure time", .unit = "us", .integer_range = durchblick::IntegerRange{500, 33000, 1}},
//       [&](const durchblick::ParameterValue& v) { return device.set_exposure(v.integer_value); },  // "" or why not
//       [&] { return durchblick::ParameterValue::of(device.exposure()); });  // what the device uses
//   params.declare("fps", std::int64_t{30}, {.choices = {ParameterValue::of(5), ParameterValue::of(30)},
//                                            .read_only = true});           // applies at the next start
//   params.check_overrides();   // all declared: unknown --param entries refused, --dump-params written
//
// Keys: <namespace>/<name>/{list_parameters, describe_parameters, get_parameters, get_parameter_types,
// set_parameters, set_parameters_atomically} (queryables: request in the query's payload, response in the reply),
// parameter_events (every node's ParameterEvent, as ROS's /parameter_events), liveliness token
// <namespace>/<name>/parameters. A change is checked against the descriptor (type, range and step, choices,
// read_only: "read-only: applies at the next start"), then given to the parameter's writer, which may fail with a
// reason; the event has every value that differs after it (a write may change another parameter).
#pragma once

#include <cstdint>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <zenoh.hxx>

#include "durchblick/msg.hpp"

namespace durchblick
{
    class ParameterError : public std::runtime_error
    {
    public:
        using std::runtime_error::runtime_error;
    };

    // A value's text, as YAML's scalar (the constraints, the dumps, the messages): true, 3, 0.5, "a: b", [1, 2].
    std::string parameter_text(const ParameterValue& value);

    // Why `value` does not fit `descriptor` (type, range and step, `choices`), or empty; the same rules and words as
    // Python's parameters.check.
    std::optional<std::string> check(const ParameterDescriptor& descriptor, const ParameterValue& value,
                                     const std::vector<ParameterValue>& choices = {});

    // A value given as text (--param) or as a params file's YAML scalar, for a parameter of `type`.
    ParameterValue parameter_from_text(std::uint8_t type, const std::string& text);

    // The options of a declaration: the descriptor's fields (unit and choices go to additional_constraints as
    // "unit: us; choices: 5, 15, 30").
    struct ParameterOptions
    {
        // (each with its initializer: a designated initializer may leave any out without -Wmissing-field-initializers)
        std::string description{};
        std::string unit{};
        std::vector<ParameterValue> choices{};  // the values it takes (of its type); empty: any (in range)
        std::optional<FloatingPointRange> floating_range{};
        std::optional<IntegerRange> integer_range{};
        bool read_only = false;  // applies at the next start (set then: --param, a params file)
    };

    // The values given at the start (parameter_args reads them from a command line).
    struct ParameterStartup
    {
        std::vector<std::filesystem::path> params_files;  // ROS params files, the later win
        std::vector<std::string> params;                  // "name:=value", over the files
        std::optional<std::filesystem::path> dump;        // the parameters in effect written here (check_overrides)
        bool read_only_access = false;                    // read over Zenoh, not changed
    };

    // --param NAME:=VALUE, --params-file FILE, --dump-params FILE, --params-read-only of a command line (the other
    // arguments are left alone).
    ParameterStartup parameter_args(int argc, char** argv);

    class Parameters
    {
    public:
        // The value in effect (e.g. a device's, which may change by itself); empty function: the stored value is it.
        using Reader = std::function<ParameterValue()>;
        // Applies a checked change: "" when done, else why it failed (refused then, nothing stored). Empty function:
        // nothing to apply (the stored value is all). It runs in Zenoh's thread for a change over Zenoh.
        using Writer = std::function<std::string(const ParameterValue&)>;

        // The node <ns>/<name>; without a session (nullptr) no services (the node's own use, tests).
        Parameters(zenoh::Session* session, const std::string& name, const std::string& ns,
                   bool read_only_access = false);
        Parameters(zenoh::Session& session, const std::string& name, const std::string& ns,
                   bool read_only_access = false)
            : Parameters(&session, name, ns, read_only_access)
        {
        }
        ~Parameters();
        Parameters(const Parameters&) = delete;
        Parameters& operator=(const Parameters&) = delete;

        void configure(const ParameterStartup& startup);
        void add_override(const std::string& assignment, const std::string& where = "--param");  // "name:=value"

        // A parameter of the type of `default_value`; its value the one given at the start (checked), else the
        // default. ParameterError for a declaration that does not fit (a value at the start outside the range, off
        // the step, not a choice, of another type, too).
        ParameterValue declare(const std::string& name, const ParameterValue& default_value,
                               const ParameterOptions& options = {}, Writer write = {}, Reader read = {});
        double declare(const std::string& name, double default_value, const ParameterOptions& options = {},
                       Writer write = {}, Reader read = {});
        std::int64_t declare(const std::string& name, std::int64_t default_value, const ParameterOptions& options = {},
                             Writer write = {}, Reader read = {});
        bool declare(const std::string& name, bool default_value, const ParameterOptions& options = {},
                     Writer write = {}, Reader read = {});
        std::string declare(const std::string& name, const std::string& default_value,
                            const ParameterOptions& options = {}, Writer write = {}, Reader read = {});
        // Every parameter declared: a value given at the start for a parameter the node does not have is an error
        // (a typo; ROS would ignore it); --dump-params gets the parameters in effect.
        void check_overrides();
        // Every writable parameter's writer with its stored value (at the start: a device keeps its last values
        // otherwise), in the order they were declared: a setting declared before its auto_* bool ends as the bool
        // says (writing the value switches to manual). ParameterError with the reasons of those that failed.
        void apply_all();
        // The values that changed by themselves (their readers': a device's) stored and announced in one
        // ParameterEvent; call it when the node's state may have changed (e.g. once a second).
        void refresh();

        ParameterValue get(const std::string& name) const;  // the value in effect (the reader's, else the stored)
        bool has(const std::string& name) const;
        bool overridden(const std::string& name) const;  // a value for it was given at the start
        ParameterDescriptor descriptor(const std::string& name) const;
        std::vector<std::string> names() const;
        // A change by the node itself: checked, then its writer; an event if values changed (read-only access does
        // not apply).
        SetParametersResult set(const std::string& name, const ParameterValue& value);
        std::string dump() const;  // the parameters now as this node's params file

        // What the services answer (also without a session).
        ListParametersResponse list(const ListParametersRequest& request) const;
        DescribeParametersResponse describe(const NamesRequest& request) const;
        GetParametersResponse get_parameters(const NamesRequest& request) const;
        GetParameterTypesResponse types(const NamesRequest& request) const;
        SetParametersResponse set_parameters(const ParametersRequest& request);
        SetParametersAtomicallyResponse set_parameters_atomically(const ParametersRequest& request);

        const std::string& key() const { return key_; }    // <namespace>/<name>
        const std::string& name() const { return name_; }  // /<namespace>/<name>

    private:
        struct Declared
        {
            ParameterDescriptor descriptor;
            ParameterValue value;  // stored
            std::vector<ParameterValue> choices;
            Writer write;
            Reader read;
        };
        ParameterValue current(const Declared& d) const;
        std::vector<SetParametersResult> set_all(const std::vector<Parameter>& parameters, bool atomic, bool local);
        std::optional<std::string> refusal(const Parameter& p, bool local) const;
        void publish_event(const std::vector<Parameter>& added, const std::vector<Parameter>& changed);
        void answer(const std::string& service, const zenoh::Query& query);

        std::string key_, name_;
        bool read_only_access_;
        mutable std::recursive_mutex mutex_;
        std::map<std::string, Declared> declared_;
        std::vector<std::string> order_;  // the names as declared: apply_all's order
        std::map<std::string, std::pair<std::string, std::string>> overrides_;  // name -> (text, where from)
        std::optional<std::filesystem::path> dump_;
        std::optional<zenoh::Publisher> events_;
        std::vector<zenoh::Queryable<void>> queryables_;
        std::optional<zenoh::LivelinessToken> token_;
    };

    // Another node's parameters over Zenoh (key: its <namespace>/<name>): each call queries <key>/<service>; throws
    // if none or several answer within the timeout, or the node replies an error.
    class ParameterClient
    {
    public:
        ParameterClient(zenoh::Session& session, std::string key, std::uint32_t timeout_ms = 2000);
        ListParametersResponse list(const ListParametersRequest& request = {});
        DescribeParametersResponse describe(const std::vector<std::string>& names);
        GetParametersResponse get(const std::vector<std::string>& names);
        GetParameterTypesResponse types(const std::vector<std::string>& names);
        SetParametersResponse set(const std::vector<Parameter>& parameters);
        SetParametersAtomicallyResponse set_atomically(const std::vector<Parameter>& parameters);

    private:
        std::vector<std::uint8_t> call(const std::string& service, const std::vector<std::uint8_t>& request);
        zenoh::Session& session_;
        std::string key_;
        std::uint32_t timeout_ms_;
    };

    // Params files: a ROS params file's values for the node `node` (fully qualified), as the YAML scalars' text, in
    // file order; and a params file of nodes' values.
    std::vector<std::pair<std::string, std::string>> read_params_file(const std::filesystem::path& path,
                                                                      const std::string& node);
    std::string dump_params_file(const std::map<std::string, std::map<std::string, ParameterValue>>& nodes);
}  // namespace durchblick
