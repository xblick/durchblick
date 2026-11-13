// ROS 2 messages in CDR, header only: the messages of a robot's controller on Zenoh (its command, state, status and
// parameters) and what they are made of. serialize() writes CDR as ROS 2 does (XCDR1 little endian, 4-byte
// encapsulation header, alignment counted after it); deserialize() reads it and refuses anything else with CdrError.
//
//   builtin_interfaces/Time, std_msgs/Header, geometry_msgs/{Vector3, Quaternion, Transform, TransformStamped, Twist,
//   Wrench, WrenchStamped}, tf2_msgs/TFMessage, diagnostic_msgs/{KeyValue, DiagnosticStatus, DiagnosticArray},
//   sensor_msgs/{JointState, MultiDOFJointState}, moveit_msgs/RobotState (no attached collision objects),
//   std_msgs/{MultiArrayDimension, MultiArrayLayout, Float64MultiArray}, rcl_interfaces' parameter messages and the
//   requests and responses of its parameter services, and Durchblick's xblick_msgs/srv/TorqueStep (a controller's
//   bring-up torque step, its version in kTorqueStepVersion)
//
// Versions: 1 (Durchblick 0.3.45) the above without TorqueStep; 2 (0.3.46) CdrWriter/CdrReader u16, TorqueStep.
//
// SHARED FILE: xblick/msg.hpp includes it (its other types are there). A controller that must not depend on
// Durchblick may carry a copy of this file. Change such a copy together with this file and raise the version with
// every change of a declaration: the inline namespace v<version> keeps two versions apart in one process (a program
// that links both copies), and the same version must mean the same declarations.
#pragma once

#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#define ROS2_CDR_MSGS_VERSION 2

namespace ros2_cdr
{
    inline namespace v2
    {
        static_assert(std::endian::native == std::endian::little, "CDR little endian is written by copying");

        class CdrError : public std::runtime_error
        {
        public:
            using std::runtime_error::runtime_error;
        };

        namespace detail
        {
            inline constexpr std::uint8_t kEncapsulation[4] = {0x00, 0x01, 0x00, 0x00};  // CDR_LE
            inline constexpr std::int64_t kNsPerS = 1'000'000'000;
        }  // namespace detail

        // ------------------------------------------------------------------------------------------- writer

        class CdrWriter
        {
        public:
            CdrWriter() : buffer_(detail::kEncapsulation, detail::kEncapsulation + 4) {}
            void reserve(std::size_t n) { buffer_.reserve(n); }
            void u8(std::uint8_t v) { buffer_.push_back(v); }
            void u16(std::uint16_t v) { put(v); }
            void u32(std::uint32_t v) { put(v); }
            void i32(std::int32_t v) { put(v); }
            void u64(std::uint64_t v) { put(v); }
            void i64(std::int64_t v) { put(v); }
            void f64(double v) { put(v); }

            void string(const std::string& v)
            {
                if (v.find('\0') != std::string::npos)
                {
                    throw CdrError("string with a NUL byte cannot be serialized");
                }
                u32(static_cast<std::uint32_t>(v.size() + 1));
                buffer_.insert(buffer_.end(), v.begin(), v.end());
                buffer_.push_back(0);
            }

            // sequence<uint8>: count, then the bytes
            void bytes(const std::uint8_t* data, std::size_t n)
            {
                u32(static_cast<std::uint32_t>(n));
                buffer_.insert(buffer_.end(), data, data + n);
            }

            // fixed array: no count
            void f64_array(const double* v, std::size_t n)
            {
                align(8);
                const auto* p = reinterpret_cast<const std::uint8_t*>(v);
                buffer_.insert(buffer_.end(), p, p + 8 * n);
            }

            void f64_sequence(const std::vector<double>& v)
            {
                u32(static_cast<std::uint32_t>(v.size()));
                if (not v.empty())
                {
                    f64_array(v.data(), v.size());
                }
            }

            std::vector<std::uint8_t> take() { return std::move(buffer_); }

        private:
            void align(std::size_t n)
            {
                const std::size_t pad = (n - (buffer_.size() - 4) % n) % n;
                buffer_.insert(buffer_.end(), pad, 0);
            }

            template <class T>
            void put(T v)
            {
                align(sizeof(T));
                const auto* p = reinterpret_cast<const std::uint8_t*>(&v);
                buffer_.insert(buffer_.end(), p, p + sizeof(T));
            }

            std::vector<std::uint8_t> buffer_;
        };

        // ------------------------------------------------------------------------------------------- reader

        class CdrReader
        {
        public:
            explicit CdrReader(std::span<const std::uint8_t> data) : data_(data)
            {
                if (data_.size() < 4)
                {
                    throw CdrError(std::to_string(data_.size()) + " bytes are no CDR message");
                }
                if (std::memcmp(data_.data(), detail::kEncapsulation, 4) != 0)
                {
                    throw CdrError("encapsulation is not CDR little endian (00 01 00 00)");
                }
            }

            std::uint8_t u8() { return take(1)[0]; }
            std::uint16_t u16() { return get<std::uint16_t>(); }
            std::uint32_t u32() { return get<std::uint32_t>(); }
            std::int32_t i32() { return get<std::int32_t>(); }
            std::uint64_t u64() { return get<std::uint64_t>(); }
            std::int64_t i64() { return get<std::int64_t>(); }
            double f64() { return get<double>(); }

            std::string string()
            {
                const std::uint32_t n = u32();
                if (n == 0)
                {
                    return "";  // some writers send an empty string without the NUL
                }
                auto raw = take(n);
                if (raw[n - 1] != 0)
                {
                    throw CdrError("string without terminating NUL");
                }
                return std::string(reinterpret_cast<const char*>(raw.data()), n - 1);
            }

            std::span<const std::uint8_t> bytes() { return take(u32()); }  // a view into the payload

            /// A sequence's length, refused (CdrError) when its elements, of at least `elementBytes` each, cannot fit
            /// in what is left: a forged length must not allocate more than the message holds.
            std::uint32_t count(std::size_t elementBytes)
            {
                const std::uint32_t n = u32();
                if (static_cast<std::uint64_t>(n) * elementBytes > data_.size() - pos_)
                {
                    throw CdrError("a sequence of " + std::to_string(n) + " elements does not fit in the " +
                                   std::to_string(data_.size() - pos_) + " bytes left");
                }
                return n;
            }

            void f64_array(double* out, std::size_t n)
            {
                align(8);
                std::memcpy(out, take(8 * n).data(), 8 * n);
            }

            std::vector<double> f64_sequence()
            {
                const std::uint32_t n = count(8);
                std::vector<double> v(n);
                if (n != 0)
                {
                    f64_array(v.data(), n);
                }
                return v;
            }

            void expect_end() const
            {
                // trailing bytes up to the next 4-byte boundary are padding some writers add
                if (data_.size() - pos_ >= 4)
                {
                    throw CdrError(std::to_string(data_.size() - pos_) + " bytes left after the message");
                }
            }

        private:
            void align(std::size_t n) { pos_ += (n - (pos_ - 4) % n) % n; }

            std::span<const std::uint8_t> take(std::size_t n)
            {
                if (pos_ + n > data_.size())
                {
                    throw CdrError("truncated message: needs " + std::to_string(pos_ + n) + " bytes, has " +
                                   std::to_string(data_.size()));
                }
                auto chunk = data_.subspan(pos_, n);
                pos_ += n;
                return chunk;
            }

            template <class T>
            T get()
            {
                align(sizeof(T));
                T v;
                std::memcpy(&v, take(sizeof(T)).data(), sizeof(T));
                return v;
            }

            std::span<const std::uint8_t> data_;
            std::size_t pos_ = 4;
        };

        // ------------------------------------------------------------------------------------------- messages

        struct Time
        {
            std::int32_t sec = 0;
            std::uint32_t nanosec = 0;

            static Time from_ns(std::int64_t ns)
            {
                std::int64_t sec = ns / detail::kNsPerS;
                std::int64_t rest = ns % detail::kNsPerS;
                if (rest < 0)
                {
                    rest += detail::kNsPerS;
                    --sec;
                }
                return {static_cast<std::int32_t>(sec), static_cast<std::uint32_t>(rest)};
            }

            std::int64_t to_ns() const { return static_cast<std::int64_t>(sec) * detail::kNsPerS + nanosec; }
            bool operator==(const Time&) const = default;
        };

        struct Header
        {
            Time stamp;
            std::string frame_id;
            bool operator==(const Header&) const = default;
        };

        // geometry_msgs/msg/Vector3, Quaternion, Transform, TransformStamped and tf2_msgs/msg/TFMessage. As in ROS, a
        // TransformStamped is the pose of child_frame_id in header.frame_id: it maps a point from the child frame to
        // the parent frame.
        struct Vector3
        {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            bool operator==(const Vector3&) const = default;
        };

        struct Quaternion
        {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            double w = 1.0;
            bool operator==(const Quaternion&) const = default;
        };

        struct Transform
        {
            Vector3 translation;
            Quaternion rotation;

            // From a rotation matrix (row-major, orthonormal) and a translation: p_parent = R p_child + t.
            static Transform from_matrix(const std::array<double, 9>& m, const Vector3& translation)
            {
                // the largest of w, x, y, z first, so that the division is by a number far from zero
                Quaternion q;
                const double trace = m[0] + m[4] + m[8];
                if (trace > 0.0)
                {
                    const double s = 2.0 * std::sqrt(1.0 + trace);
                    q = {(m[7] - m[5]) / s, (m[2] - m[6]) / s, (m[3] - m[1]) / s, 0.25 * s};
                }
                else if (m[0] > m[4] and m[0] > m[8])
                {
                    const double s = 2.0 * std::sqrt(1.0 + m[0] - m[4] - m[8]);
                    q = {0.25 * s, (m[1] + m[3]) / s, (m[2] + m[6]) / s, (m[7] - m[5]) / s};
                }
                else if (m[4] > m[8])
                {
                    const double s = 2.0 * std::sqrt(1.0 + m[4] - m[0] - m[8]);
                    q = {(m[1] + m[3]) / s, 0.25 * s, (m[5] + m[7]) / s, (m[2] - m[6]) / s};
                }
                else
                {
                    const double s = 2.0 * std::sqrt(1.0 + m[8] - m[0] - m[4]);
                    q = {(m[2] + m[6]) / s, (m[5] + m[7]) / s, 0.25 * s, (m[3] - m[1]) / s};
                }
                const double sign = q.w < 0.0 ? -1.0 : 1.0;  // one of the two equal quaternions: w >= 0
                const double n = sign / std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
                return {translation, {q.x * n, q.y * n, q.z * n, q.w * n}};
            }

            bool operator==(const Transform&) const = default;
        };

        struct TransformStamped
        {
            Header header;  // frame_id: the parent frame
            std::string child_frame_id;
            Transform transform;
            bool operator==(const TransformStamped&) const = default;
        };

        struct TFMessage
        {
            std::vector<TransformStamped> transforms;

            std::vector<std::uint8_t> serialize() const;
            static TFMessage deserialize(std::span<const std::uint8_t> payload);
            bool operator==(const TFMessage&) const = default;
        };

        // diagnostic_msgs/msg/KeyValue, DiagnosticStatus, DiagnosticArray: how a publisher is doing, as ROS's
        // /diagnostics: several publishers, one status each, told apart by name.
        struct KeyValue
        {
            std::string key;
            std::string value;
            bool operator==(const KeyValue&) const = default;
        };

        struct DiagnosticStatus
        {
            static constexpr std::uint8_t kOk = 0, kWarn = 1, kError = 2, kStale = 3;
            std::uint8_t level = kOk;
            std::string name;
            std::string message;
            std::string hardware_id;
            std::vector<KeyValue> values;
            bool operator==(const DiagnosticStatus&) const = default;
        };

        struct DiagnosticArray
        {
            Header header;
            std::vector<DiagnosticStatus> status;

            std::vector<std::uint8_t> serialize() const;
            static DiagnosticArray deserialize(std::span<const std::uint8_t> payload);
            bool operator==(const DiagnosticArray&) const = default;
        };

        // position (rad, m), velocity and effort (Nm, N) of the joints in name, each as long as name or empty (the
        // source does not have that quantity), as ROS allows; serialize() throws std::invalid_argument for other
        // lengths.
        struct JointState
        {
            Header header;
            std::vector<std::string> name;
            std::vector<double> position;
            std::vector<double> velocity;
            std::vector<double> effort;

            std::vector<std::uint8_t> serialize() const;
            static JointState deserialize(std::span<const std::uint8_t> payload);
            bool operator==(const JointState&) const = default;
        };

        struct Twist  // geometry_msgs/msg/Twist: linear (m/s) and angular (rad/s) velocity
        {
            Vector3 linear;
            Vector3 angular;
            bool operator==(const Twist&) const = default;
        };

        struct Wrench  // geometry_msgs/msg/Wrench: force (N) and torque (Nm)
        {
            Vector3 force;
            Vector3 torque;
            bool operator==(const Wrench&) const = default;
        };

        struct WrenchStamped  // a force-torque sensor's measurement in its frame (header.frame_id)
        {
            Header header;
            Wrench wrench;

            std::vector<std::uint8_t> serialize() const;
            static WrenchStamped deserialize(std::span<const std::uint8_t> payload);
            bool operator==(const WrenchStamped&) const = default;
        };

        // sensor_msgs/msg/MultiDOFJointState: per joint in joint_names a transform, and its twist and wrench (each as
        // long as joint_names or empty, as ROS allows); in a controller command the joints are the controlled frames
        // (TCPs) and header.frame_id the frame of their poses.
        struct MultiDOFJointState
        {
            Header header;
            std::vector<std::string> joint_names;
            std::vector<Transform> transforms;
            std::vector<Twist> twists;
            std::vector<Wrench> wrenches;
            bool operator==(const MultiDOFJointState&) const = default;
        };

        // moveit_msgs/msg/RobotState: a controller's command, one message: joint_state (joint targets: postures,
        // hands) and multi_dof_joint_state (the controlled frames' pose targets), both with the command's stamp. No
        // attached collision objects: serialize() writes none, deserialize() refuses a message with some
        // (std::invalid_argument). serialize() checks the lengths as JointState's and MultiDOFJointState's.
        struct RobotState
        {
            JointState joint_state;
            MultiDOFJointState multi_dof_joint_state;
            bool is_diff = false;

            std::vector<std::uint8_t> serialize() const;
            static RobotState deserialize(std::span<const std::uint8_t> payload);
            bool operator==(const RobotState&) const = default;
        };

        struct MultiArrayDimension  // std_msgs/msg/MultiArrayDimension
        {
            std::string label;
            std::uint32_t size = 0;
            std::uint32_t stride = 0;
            bool operator==(const MultiArrayDimension&) const = default;
        };

        struct MultiArrayLayout  // std_msgs/msg/MultiArrayLayout
        {
            std::vector<MultiArrayDimension> dim;
            std::uint32_t data_offset = 0;
            bool operator==(const MultiArrayLayout&) const = default;
        };

        // std_msgs/msg/Float64MultiArray: numbers of a fixed layout, e.g. a controller's state records (records x
        // values; the values' names elsewhere)
        struct Float64MultiArray
        {
            MultiArrayLayout layout;
            std::vector<double> data;

            std::vector<std::uint8_t> serialize() const;
            static Float64MultiArray deserialize(std::span<const std::uint8_t> payload);
            bool operator==(const Float64MultiArray&) const = default;
        };

        // ------------------------------------------------------------------------ parameters (rcl_interfaces)

        namespace parameter_type  // rcl_interfaces/msg/ParameterType
        {
            inline constexpr std::uint8_t kNotSet = 0, kBool = 1, kInteger = 2, kDouble = 3, kString = 4,
                                          kByteArray = 5, kBoolArray = 6, kIntegerArray = 7, kDoubleArray = 8,
                                          kStringArray = 9;

            inline const char* name(std::uint8_t type)
            {
                static const char* names[] = {"not set",    "bool",       "integer",       "double",
                                              "string",     "byte array", "bool array",    "integer array",
                                              "double array", "string array"};
                return type < 10 ? names[type] : "unknown";
            }
        }  // namespace parameter_type

        // rcl_interfaces/msg/ParameterValue: `type` says which field holds the value
        struct ParameterValue
        {
            std::uint8_t type = parameter_type::kNotSet;
            bool bool_value = false;
            std::int64_t integer_value = 0;
            double double_value = 0.0;
            std::string string_value;
            std::vector<std::uint8_t> byte_array_value;
            std::vector<bool> bool_array_value;
            std::vector<std::int64_t> integer_array_value;
            std::vector<double> double_array_value;
            std::vector<std::string> string_array_value;

            static ParameterValue of(bool v)
            {
                ParameterValue p;
                p.type = parameter_type::kBool;
                p.bool_value = v;
                return p;
            }

            static ParameterValue of(std::int64_t v)
            {
                ParameterValue p;
                p.type = parameter_type::kInteger;
                p.integer_value = v;
                return p;
            }

            static ParameterValue of(int v) { return of(static_cast<std::int64_t>(v)); }

            static ParameterValue of(double v)
            {
                ParameterValue p;
                p.type = parameter_type::kDouble;
                p.double_value = v;
                return p;
            }

            static ParameterValue of(const std::string& v)
            {
                ParameterValue p;
                p.type = parameter_type::kString;
                p.string_value = v;
                return p;
            }

            static ParameterValue of(const char* v) { return of(std::string(v)); }

            static ParameterValue of(const std::vector<double>& v)
            {
                ParameterValue p;
                p.type = parameter_type::kDoubleArray;
                p.double_array_value = v;
                return p;
            }

            static ParameterValue of(const std::vector<std::int64_t>& v)
            {
                ParameterValue p;
                p.type = parameter_type::kIntegerArray;
                p.integer_array_value = v;
                return p;
            }

            static ParameterValue of(const std::vector<std::string>& v)
            {
                ParameterValue p;
                p.type = parameter_type::kStringArray;
                p.string_array_value = v;
                return p;
            }

            static ParameterValue of(const std::vector<bool>& v)
            {
                ParameterValue p;
                p.type = parameter_type::kBoolArray;
                p.bool_array_value = v;
                return p;
            }

            bool operator==(const ParameterValue&) const = default;
        };

        struct Parameter  // rcl_interfaces/msg/Parameter
        {
            std::string name;
            ParameterValue value;
            bool operator==(const Parameter&) const = default;
        };

        struct FloatingPointRange  // rcl_interfaces/msg/FloatingPointRange: step 0 for any value between
        {
            double from_value = 0.0;
            double to_value = 0.0;
            double step = 0.0;
            bool operator==(const FloatingPointRange&) const = default;
        };

        struct IntegerRange  // rcl_interfaces/msg/IntegerRange
        {
            std::int64_t from_value = 0;
            std::int64_t to_value = 0;
            std::uint64_t step = 0;
            bool operator==(const IntegerRange&) const = default;
        };

        // rcl_interfaces/msg/ParameterDescriptor; the ranges are sequences of at most one
        struct ParameterDescriptor
        {
            std::string name;
            std::uint8_t type = parameter_type::kNotSet;
            std::string description;
            std::string additional_constraints;
            bool read_only = false;
            bool dynamic_typing = false;
            std::vector<FloatingPointRange> floating_point_range;
            std::vector<IntegerRange> integer_range;
            bool operator==(const ParameterDescriptor&) const = default;
        };

        struct SetParametersResult  // rcl_interfaces/msg/SetParametersResult
        {
            bool successful = false;
            std::string reason;
            bool operator==(const SetParametersResult&) const = default;
        };

        struct ParameterEvent  // rcl_interfaces/msg/ParameterEvent
        {
            Time stamp;
            std::string node;  // the node's fully qualified name
            std::vector<Parameter> new_parameters, changed_parameters, deleted_parameters;

            std::vector<std::uint8_t> serialize() const;
            static ParameterEvent deserialize(std::span<const std::uint8_t> payload);
        };

        // The services' requests and responses (rcl_interfaces/srv/<Service>_Request, _Response).
        struct ListParametersRequest
        {
            std::vector<std::string> prefixes;
            std::uint64_t depth = 0;  // 0: every level
            std::vector<std::uint8_t> serialize() const;
            static ListParametersRequest deserialize(std::span<const std::uint8_t> payload);
        };

        struct ListParametersResponse  // its ListParametersResult
        {
            std::vector<std::string> names, prefixes;
            std::vector<std::uint8_t> serialize() const;
            static ListParametersResponse deserialize(std::span<const std::uint8_t> payload);
        };

        struct NamesRequest  // DescribeParameters, GetParameters, GetParameterTypes
        {
            std::vector<std::string> names;
            std::vector<std::uint8_t> serialize() const;
            static NamesRequest deserialize(std::span<const std::uint8_t> payload);
        };

        struct DescribeParametersResponse
        {
            std::vector<ParameterDescriptor> descriptors;
            std::vector<std::uint8_t> serialize() const;
            static DescribeParametersResponse deserialize(std::span<const std::uint8_t> payload);
        };

        struct GetParametersResponse
        {
            std::vector<ParameterValue> values;
            std::vector<std::uint8_t> serialize() const;
            static GetParametersResponse deserialize(std::span<const std::uint8_t> payload);
        };

        struct GetParameterTypesResponse
        {
            std::vector<std::uint8_t> types;
            std::vector<std::uint8_t> serialize() const;
            static GetParameterTypesResponse deserialize(std::span<const std::uint8_t> payload);
        };

        struct ParametersRequest  // SetParameters, SetParametersAtomically
        {
            std::vector<Parameter> parameters;
            std::vector<std::uint8_t> serialize() const;
            static ParametersRequest deserialize(std::span<const std::uint8_t> payload);
        };

        struct SetParametersResponse
        {
            std::vector<SetParametersResult> results;
            std::vector<std::uint8_t> serialize() const;
            static SetParametersResponse deserialize(std::span<const std::uint8_t> payload);
        };

        struct SetParametersAtomicallyResponse
        {
            SetParametersResult result;
            std::vector<std::uint8_t> serialize() const;
            static SetParametersAtomicallyResponse deserialize(std::span<const std::uint8_t> payload);
        };

        // ----------------------------------------------------------------------- xblick_msgs/srv/TorqueStep
        // A controller's bring-up torque step (its service <controller>/torque_step): `torque` [Nm] on the joint
        // `joint` for `duration` [s]; the controller checks joint and bounds. Response: accepted or why not.

        inline constexpr std::uint32_t kTorqueStepVersion = 1;  // the schema's @<version>

        struct TorqueStepRequest
        {
            std::string joint;
            double torque = 0.0;
            double duration = 0.0;

            std::vector<std::uint8_t> serialize() const
            {
                CdrWriter w;
                w.string(joint);
                w.f64(torque);
                w.f64(duration);
                return w.take();
            }

            static TorqueStepRequest deserialize(std::span<const std::uint8_t> payload)
            {
                CdrReader r(payload);
                TorqueStepRequest m;
                m.joint = r.string();
                m.torque = r.f64();
                m.duration = r.f64();
                r.expect_end();
                return m;
            }
        };

        struct TorqueStepResponse
        {
            bool ok = false;
            std::string message;  // empty when ok; else why not

            std::vector<std::uint8_t> serialize() const
            {
                CdrWriter w;
                w.u8(ok ? 1 : 0);
                w.string(message);
                return w.take();
            }

            static TorqueStepResponse deserialize(std::span<const std::uint8_t> payload)
            {
                CdrReader r(payload);
                TorqueStepResponse m;
                m.ok = r.u8() != 0;
                m.message = r.string();
                r.expect_end();
                return m;
            }
        };

        // ------------------------------------------------------------------------------------ the CDR layouts

        namespace detail
        {
            inline void write(CdrWriter& w, const Header& h)
            {
                w.i32(h.stamp.sec);
                w.u32(h.stamp.nanosec);
                w.string(h.frame_id);
            }

            inline Header read_header(CdrReader& r)
            {
                Header h;
                h.stamp.sec = r.i32();
                h.stamp.nanosec = r.u32();
                h.frame_id = r.string();
                return h;
            }

            inline void write_vector3(CdrWriter& w, const Vector3& v)
            {
                w.f64(v.x);
                w.f64(v.y);
                w.f64(v.z);
            }

            inline Vector3 read_vector3(CdrReader& r)
            {
                Vector3 v;
                v.x = r.f64();
                v.y = r.f64();
                v.z = r.f64();
                return v;
            }

            inline void write_joint_state(CdrWriter& w, const JointState& m)
            {
                for (const std::vector<double>* values : {&m.position, &m.velocity, &m.effort})
                {
                    if (not values->empty() and values->size() != m.name.size())
                    {
                        throw std::invalid_argument("JointState: " + std::to_string(values->size()) +
                                                    " values for " + std::to_string(m.name.size()) +
                                                    " joints (that many or none)");
                    }
                }
                write(w, m.header);
                w.u32(static_cast<std::uint32_t>(m.name.size()));
                for (const std::string& n : m.name)
                {
                    w.string(n);
                }
                w.f64_sequence(m.position);
                w.f64_sequence(m.velocity);
                w.f64_sequence(m.effort);
            }

            inline JointState read_joint_state(CdrReader& r)
            {
                JointState m;
                m.header = read_header(r);
                const std::uint32_t n = r.u32();
                for (std::uint32_t i = 0; i < n; ++i)
                {
                    m.name.push_back(r.string());
                }
                m.position = r.f64_sequence();
                m.velocity = r.f64_sequence();
                m.effort = r.f64_sequence();
                return m;
            }

            inline void write_strings(CdrWriter& w, const std::vector<std::string>& v)
            {
                w.u32(static_cast<std::uint32_t>(v.size()));
                for (const std::string& s : v)
                {
                    w.string(s);
                }
            }

            inline std::vector<std::string> read_strings(CdrReader& r)
            {
                std::vector<std::string> v(r.count(4));
                for (std::string& s : v)
                {
                    s = r.string();
                }
                return v;
            }

            inline void write(CdrWriter& w, const ParameterValue& v)
            {
                w.u8(v.type);
                w.u8(v.bool_value ? 1 : 0);
                w.i64(v.integer_value);
                w.f64(v.double_value);
                w.string(v.string_value);
                w.bytes(v.byte_array_value.data(), v.byte_array_value.size());
                w.u32(static_cast<std::uint32_t>(v.bool_array_value.size()));
                for (const bool b : v.bool_array_value)
                {
                    w.u8(b ? 1 : 0);
                }
                w.u32(static_cast<std::uint32_t>(v.integer_array_value.size()));
                for (const std::int64_t i : v.integer_array_value)
                {
                    w.i64(i);
                }
                w.f64_sequence(v.double_array_value);
                write_strings(w, v.string_array_value);
            }

            inline ParameterValue read_value(CdrReader& r)
            {
                ParameterValue v;
                v.type = r.u8();
                v.bool_value = r.u8() != 0;
                v.integer_value = r.i64();
                v.double_value = r.f64();
                v.string_value = r.string();
                const auto bytes = r.bytes();
                v.byte_array_value.assign(bytes.begin(), bytes.end());
                v.bool_array_value.resize(r.count(1));
                for (std::size_t i = 0; i < v.bool_array_value.size(); ++i)
                {
                    v.bool_array_value[i] = r.u8() != 0;
                }
                v.integer_array_value.resize(r.count(8));
                for (std::int64_t& i : v.integer_array_value)
                {
                    i = r.i64();
                }
                v.double_array_value = r.f64_sequence();
                v.string_array_value = read_strings(r);
                return v;
            }

            inline void write(CdrWriter& w, const Parameter& p)
            {
                w.string(p.name);
                write(w, p.value);
            }

            inline Parameter read_parameter(CdrReader& r)
            {
                Parameter p;
                p.name = r.string();
                p.value = read_value(r);
                return p;
            }

            inline void write(CdrWriter& w, const ParameterDescriptor& d)
            {
                w.string(d.name);
                w.u8(d.type);
                w.string(d.description);
                w.string(d.additional_constraints);
                w.u8(d.read_only ? 1 : 0);
                w.u8(d.dynamic_typing ? 1 : 0);
                w.u32(static_cast<std::uint32_t>(d.floating_point_range.size()));
                for (const FloatingPointRange& r : d.floating_point_range)
                {
                    w.f64(r.from_value);
                    w.f64(r.to_value);
                    w.f64(r.step);
                }
                w.u32(static_cast<std::uint32_t>(d.integer_range.size()));
                for (const IntegerRange& r : d.integer_range)
                {
                    w.i64(r.from_value);
                    w.i64(r.to_value);
                    w.u64(r.step);
                }
            }

            inline ParameterDescriptor read_descriptor(CdrReader& r)
            {
                ParameterDescriptor d;
                d.name = r.string();
                d.type = r.u8();
                d.description = r.string();
                d.additional_constraints = r.string();
                d.read_only = r.u8() != 0;
                d.dynamic_typing = r.u8() != 0;
                d.floating_point_range.resize(r.count(24));
                for (FloatingPointRange& f : d.floating_point_range)
                {
                    f.from_value = r.f64();
                    f.to_value = r.f64();
                    f.step = r.f64();
                }
                d.integer_range.resize(r.count(24));
                for (IntegerRange& i : d.integer_range)
                {
                    i.from_value = r.i64();
                    i.to_value = r.i64();
                    i.step = r.u64();
                }
                return d;
            }

            inline void write(CdrWriter& w, const SetParametersResult& s)
            {
                w.u8(s.successful ? 1 : 0);
                w.string(s.reason);
            }

            inline SetParametersResult read_result(CdrReader& r)
            {
                SetParametersResult s;
                s.successful = r.u8() != 0;
                s.reason = r.string();
                return s;
            }

            template <typename T>
            void write_list(CdrWriter& w, const std::vector<T>& items)
            {
                w.u32(static_cast<std::uint32_t>(items.size()));
                for (const T& item : items)
                {
                    write(w, item);
                }
            }

            template <typename T, typename F>
            std::vector<T> read_list(CdrReader& r, F read)
            {
                std::vector<T> items(r.count(1));
                for (T& item : items)
                {
                    item = read(r);
                }
                return items;
            }
        }  // namespace detail

        inline std::vector<std::uint8_t> TFMessage::serialize() const
        {
            CdrWriter w;
            w.u32(static_cast<std::uint32_t>(transforms.size()));
            for (const TransformStamped& t : transforms)
            {
                detail::write(w, t.header);
                w.string(t.child_frame_id);
                const Transform& f = t.transform;
                for (double v : {f.translation.x, f.translation.y, f.translation.z, f.rotation.x, f.rotation.y,
                                 f.rotation.z, f.rotation.w})
                {
                    w.f64(v);
                }
            }
            return w.take();
        }

        inline TFMessage TFMessage::deserialize(std::span<const std::uint8_t> payload)
        {
            CdrReader r(payload);
            TFMessage m;
            const std::uint32_t n = r.u32();
            for (std::uint32_t i = 0; i < n; ++i)
            {
                TransformStamped t;
                t.header = detail::read_header(r);
                t.child_frame_id = r.string();
                Transform& f = t.transform;
                f.translation = {r.f64(), r.f64(), r.f64()};
                f.rotation = {r.f64(), r.f64(), r.f64(), r.f64()};
                m.transforms.push_back(std::move(t));
            }
            r.expect_end();
            return m;
        }

        inline std::vector<std::uint8_t> DiagnosticArray::serialize() const
        {
            CdrWriter w;
            detail::write(w, header);
            w.u32(static_cast<std::uint32_t>(status.size()));
            for (const DiagnosticStatus& s : status)
            {
                w.u8(s.level);
                w.string(s.name);
                w.string(s.message);
                w.string(s.hardware_id);
                w.u32(static_cast<std::uint32_t>(s.values.size()));
                for (const KeyValue& kv : s.values)
                {
                    w.string(kv.key);
                    w.string(kv.value);
                }
            }
            return w.take();
        }

        inline DiagnosticArray DiagnosticArray::deserialize(std::span<const std::uint8_t> payload)
        {
            CdrReader r(payload);
            DiagnosticArray m;
            m.header = detail::read_header(r);
            const std::uint32_t n = r.u32();
            for (std::uint32_t i = 0; i < n; ++i)
            {
                DiagnosticStatus s;
                s.level = r.u8();
                s.name = r.string();
                s.message = r.string();
                s.hardware_id = r.string();
                const std::uint32_t k = r.u32();
                for (std::uint32_t j = 0; j < k; ++j)
                {
                    KeyValue kv;
                    kv.key = r.string();
                    kv.value = r.string();
                    s.values.push_back(std::move(kv));
                }
                m.status.push_back(std::move(s));
            }
            r.expect_end();
            return m;
        }

        inline std::vector<std::uint8_t> JointState::serialize() const
        {
            CdrWriter w;
            detail::write_joint_state(w, *this);
            return w.take();
        }

        inline JointState JointState::deserialize(std::span<const std::uint8_t> payload)
        {
            CdrReader r(payload);
            JointState m = detail::read_joint_state(r);
            r.expect_end();
            return m;
        }

        inline std::vector<std::uint8_t> WrenchStamped::serialize() const
        {
            CdrWriter w;
            detail::write(w, header);
            const Vector3 &f = wrench.force, &t = wrench.torque;
            for (double v : {f.x, f.y, f.z, t.x, t.y, t.z})
            {
                w.f64(v);
            }
            return w.take();
        }

        inline WrenchStamped WrenchStamped::deserialize(std::span<const std::uint8_t> payload)
        {
            CdrReader r(payload);
            WrenchStamped m;
            m.header = detail::read_header(r);
            m.wrench.force = {r.f64(), r.f64(), r.f64()};
            m.wrench.torque = {r.f64(), r.f64(), r.f64()};
            r.expect_end();
            return m;
        }

        inline std::vector<std::uint8_t> RobotState::serialize() const
        {
            const MultiDOFJointState& d = multi_dof_joint_state;
            const std::size_t n = d.joint_names.size();
            if (d.transforms.size() != n or (not d.twists.empty() and d.twists.size() != n) or
                (not d.wrenches.empty() and d.wrenches.size() != n))
            {
                throw std::invalid_argument("MultiDOFJointState: " + std::to_string(d.transforms.size()) +
                                            " transforms, " + std::to_string(d.twists.size()) + " twists, " +
                                            std::to_string(d.wrenches.size()) + " wrenches for " +
                                            std::to_string(n) +
                                            " joints (a transform each; twists and wrenches that many or none)");
            }
            CdrWriter w;
            detail::write_joint_state(w, joint_state);
            detail::write(w, d.header);
            w.u32(static_cast<std::uint32_t>(n));
            for (const std::string& name : d.joint_names)
            {
                w.string(name);
            }
            w.u32(static_cast<std::uint32_t>(n));
            for (const Transform& t : d.transforms)
            {
                detail::write_vector3(w, t.translation);
                for (double v : {t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w})
                {
                    w.f64(v);
                }
            }
            w.u32(static_cast<std::uint32_t>(d.twists.size()));
            for (const Twist& t : d.twists)
            {
                detail::write_vector3(w, t.linear);
                detail::write_vector3(w, t.angular);
            }
            w.u32(static_cast<std::uint32_t>(d.wrenches.size()));
            for (const Wrench& t : d.wrenches)
            {
                detail::write_vector3(w, t.force);
                detail::write_vector3(w, t.torque);
            }
            w.u32(0);  // attached_collision_objects: none
            w.u8(is_diff ? 1 : 0);
            return w.take();
        }

        inline RobotState RobotState::deserialize(std::span<const std::uint8_t> payload)
        {
            CdrReader r(payload);
            RobotState m;
            m.joint_state = detail::read_joint_state(r);
            MultiDOFJointState& d = m.multi_dof_joint_state;
            d.header = detail::read_header(r);
            for (std::uint32_t i = 0, n = r.u32(); i < n; ++i)
            {
                d.joint_names.push_back(r.string());
            }
            for (std::uint32_t i = 0, n = r.u32(); i < n; ++i)
            {
                Transform& t = d.transforms.emplace_back();
                t.translation = detail::read_vector3(r);
                t.rotation.x = r.f64();
                t.rotation.y = r.f64();
                t.rotation.z = r.f64();
                t.rotation.w = r.f64();
            }
            for (std::uint32_t i = 0, n = r.u32(); i < n; ++i)
            {
                Twist& t = d.twists.emplace_back();
                t.linear = detail::read_vector3(r);
                t.angular = detail::read_vector3(r);
            }
            for (std::uint32_t i = 0, n = r.u32(); i < n; ++i)
            {
                Wrench& t = d.wrenches.emplace_back();
                t.force = detail::read_vector3(r);
                t.torque = detail::read_vector3(r);
            }
            if (const std::uint32_t attached = r.u32(); attached != 0)
            {
                throw std::invalid_argument("RobotState with " + std::to_string(attached) +
                                            " attached collision objects: none are taken");
            }
            m.is_diff = r.u8() != 0;
            r.expect_end();
            const std::size_t n = d.joint_names.size();
            if (d.transforms.size() != n or (not d.twists.empty() and d.twists.size() != n) or
                (not d.wrenches.empty() and d.wrenches.size() != n))
            {
                throw std::invalid_argument(
                    "MultiDOFJointState: transforms, twists or wrenches do not match its joints");
            }
            return m;
        }

        inline std::vector<std::uint8_t> Float64MultiArray::serialize() const
        {
            CdrWriter w;
            w.u32(static_cast<std::uint32_t>(layout.dim.size()));
            for (const MultiArrayDimension& d : layout.dim)
            {
                w.string(d.label);
                w.u32(d.size);
                w.u32(d.stride);
            }
            w.u32(layout.data_offset);
            w.f64_sequence(data);
            return w.take();
        }

        inline Float64MultiArray Float64MultiArray::deserialize(std::span<const std::uint8_t> payload)
        {
            CdrReader r(payload);
            Float64MultiArray m;
            const std::uint32_t n = r.u32();
            for (std::uint32_t i = 0; i < n; ++i)
            {
                MultiArrayDimension& d = m.layout.dim.emplace_back();
                d.label = r.string();
                d.size = r.u32();
                d.stride = r.u32();
            }
            m.layout.data_offset = r.u32();
            m.data = r.f64_sequence();
            r.expect_end();
            return m;
        }

        inline std::vector<std::uint8_t> ParameterEvent::serialize() const
        {
            CdrWriter w;
            w.i32(stamp.sec);
            w.u32(stamp.nanosec);
            w.string(node);
            detail::write_list(w, new_parameters);
            detail::write_list(w, changed_parameters);
            detail::write_list(w, deleted_parameters);
            return w.take();
        }

        inline ParameterEvent ParameterEvent::deserialize(std::span<const std::uint8_t> payload)
        {
            CdrReader r(payload);
            ParameterEvent e;
            e.stamp.sec = r.i32();
            e.stamp.nanosec = r.u32();
            e.node = r.string();
            e.new_parameters = detail::read_list<Parameter>(r, detail::read_parameter);
            e.changed_parameters = detail::read_list<Parameter>(r, detail::read_parameter);
            e.deleted_parameters = detail::read_list<Parameter>(r, detail::read_parameter);
            r.expect_end();
            return e;
        }

        inline std::vector<std::uint8_t> ListParametersRequest::serialize() const
        {
            CdrWriter w;
            detail::write_strings(w, prefixes);
            w.u64(depth);
            return w.take();
        }

        inline ListParametersRequest ListParametersRequest::deserialize(std::span<const std::uint8_t> payload)
        {
            CdrReader r(payload);
            ListParametersRequest m;
            m.prefixes = detail::read_strings(r);
            m.depth = r.u64();
            r.expect_end();
            return m;
        }

        inline std::vector<std::uint8_t> ListParametersResponse::serialize() const
        {
            CdrWriter w;
            detail::write_strings(w, names);
            detail::write_strings(w, prefixes);
            return w.take();
        }

        inline ListParametersResponse ListParametersResponse::deserialize(std::span<const std::uint8_t> payload)
        {
            CdrReader r(payload);
            ListParametersResponse m;
            m.names = detail::read_strings(r);
            m.prefixes = detail::read_strings(r);
            r.expect_end();
            return m;
        }

        inline std::vector<std::uint8_t> NamesRequest::serialize() const
        {
            CdrWriter w;
            detail::write_strings(w, names);
            return w.take();
        }

        inline NamesRequest NamesRequest::deserialize(std::span<const std::uint8_t> payload)
        {
            CdrReader r(payload);
            NamesRequest m;
            m.names = detail::read_strings(r);
            r.expect_end();
            return m;
        }

        inline std::vector<std::uint8_t> DescribeParametersResponse::serialize() const
        {
            CdrWriter w;
            detail::write_list(w, descriptors);
            return w.take();
        }

        inline DescribeParametersResponse DescribeParametersResponse::deserialize(
            std::span<const std::uint8_t> payload)
        {
            CdrReader r(payload);
            DescribeParametersResponse m;
            m.descriptors = detail::read_list<ParameterDescriptor>(r, detail::read_descriptor);
            r.expect_end();
            return m;
        }

        inline std::vector<std::uint8_t> GetParametersResponse::serialize() const
        {
            CdrWriter w;
            detail::write_list(w, values);
            return w.take();
        }

        inline GetParametersResponse GetParametersResponse::deserialize(std::span<const std::uint8_t> payload)
        {
            CdrReader r(payload);
            GetParametersResponse m;
            m.values = detail::read_list<ParameterValue>(r, detail::read_value);
            r.expect_end();
            return m;
        }

        inline std::vector<std::uint8_t> GetParameterTypesResponse::serialize() const
        {
            CdrWriter w;
            w.bytes(types.data(), types.size());
            return w.take();
        }

        inline GetParameterTypesResponse GetParameterTypesResponse::deserialize(std::span<const std::uint8_t> payload)
        {
            CdrReader r(payload);
            GetParameterTypesResponse m;
            const auto bytes = r.bytes();
            m.types.assign(bytes.begin(), bytes.end());
            r.expect_end();
            return m;
        }

        inline std::vector<std::uint8_t> ParametersRequest::serialize() const
        {
            CdrWriter w;
            detail::write_list(w, parameters);
            return w.take();
        }

        inline ParametersRequest ParametersRequest::deserialize(std::span<const std::uint8_t> payload)
        {
            CdrReader r(payload);
            ParametersRequest m;
            m.parameters = detail::read_list<Parameter>(r, detail::read_parameter);
            r.expect_end();
            return m;
        }

        inline std::vector<std::uint8_t> SetParametersResponse::serialize() const
        {
            CdrWriter w;
            detail::write_list(w, results);
            return w.take();
        }

        inline SetParametersResponse SetParametersResponse::deserialize(std::span<const std::uint8_t> payload)
        {
            CdrReader r(payload);
            SetParametersResponse m;
            m.results = detail::read_list<SetParametersResult>(r, detail::read_result);
            r.expect_end();
            return m;
        }

        inline std::vector<std::uint8_t> SetParametersAtomicallyResponse::serialize() const
        {
            CdrWriter w;
            detail::write(w, result);
            return w.take();
        }

        inline SetParametersAtomicallyResponse SetParametersAtomicallyResponse::deserialize(
            std::span<const std::uint8_t> payload)
        {
            CdrReader r(payload);
            SetParametersAtomicallyResponse m;
            m.result = detail::read_result(r);
            r.expect_end();
            return m;
        }
    }  // namespace v2
}  // namespace ros2_cdr
