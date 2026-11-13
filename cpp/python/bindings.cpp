// blick._core: the C++ rings for Python. Buffers cross without a copy: host slots as NumPy arrays (views into the
// shared memory, kept alive by the ring object), device buffers as pointers that blick.gpu wraps with
// __cuda_array_interface__. Copies and waits release the GIL.
#include <algorithm>
#include <cstring>
#include <vector>

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include "durchblick/cuda_ring.hpp"
#include "durchblick/pairing.hpp"
#include "durchblick/protocol.hpp"
#include "durchblick/ray_cloud.hpp"
#include "durchblick/rectify.hpp"
#include "durchblick/ring.hpp"
#include "durchblick/rvl.hpp"
#include "durchblick/version.hpp"

namespace nb = nanobind;
using namespace nb::literals;
using durchblick::Written;

namespace
{
    using HostArray = nb::ndarray<nb::device::cpu, nb::c_contig>;          // written to (out)
    using InArray = nb::ndarray<nb::ro, nb::device::cpu, nb::c_contig>;    // only read: read-only arrays are fine
    using ByteView = nb::ndarray<nb::numpy, std::uint8_t, nb::ndim<1>>;
    using ReadOnlyByteView = nb::ndarray<nb::numpy, const std::uint8_t, nb::ndim<1>>;

    template <typename Array>
    std::uint64_t nbytes_of(const Array& a)
    {
        return static_cast<std::uint64_t>(a.size()) * a.itemsize();
    }

    nb::tuple pair(const Written& w)
    {
        return nb::make_tuple(w.slot, w.seq);
    }
}  // namespace

NB_MODULE(_core, m)
{
    m.doc() = "Durchblick's C++ core: shared-memory and CUDA rings (cpp/include/durchblick/ring.hpp describes them)";
    nb::exception<durchblick::RingError>(m, "RingError", PyExc_RuntimeError);
    nb::exception<durchblick::rvl::Error>(m, "RvlError", PyExc_ValueError);

    m.def("host_id", &durchblick::host_id, "boot id of this host; descriptors carry it");
    m.def("rvl_encode", [](InArray pixels) {
              if (pixels.ndim() != 2 or pixels.dtype() != nb::dtype<std::uint16_t>())
              {
                  throw nb::value_error("rvl_encode takes a 2-D uint16 array");
              }
              const auto bytes = durchblick::rvl::encode(pixels.data(), static_cast<std::uint32_t>(pixels.shape(0)),
                                                         static_cast<std::uint32_t>(pixels.shape(1)));
              return nb::bytes(reinterpret_cast<const char*>(bytes.data()), bytes.size());
          }, "pixels"_a, "a 16-bit image as ROS's compressedDepth rvl writes it (rvl.hpp)");
    m.def("rvl_shape", [](InArray data) {
              const auto [height, width] =
                  durchblick::rvl::shape({static_cast<const std::uint8_t*>(data.data()), data.nbytes()});
              return std::make_tuple(height, width);
          }, "data"_a, "(height, width) of rvl_encode's bytes");
    m.def("rvl_decode", [](InArray data, HostArray out) {
              const std::span<const std::uint8_t> bytes{static_cast<const std::uint8_t*>(data.data()), data.nbytes()};
              const auto [height, width] = durchblick::rvl::shape(bytes);
              if (out.dtype() != nb::dtype<std::uint16_t>() or out.nbytes() != std::size_t{height} * width * 2)
              {
                  throw nb::value_error("rvl_decode: out is not a uint16 array of rvl_shape's size");
              }
              durchblick::rvl::decode(bytes, out.data());
          }, "data"_a, "out"_a, "decodes rvl_encode's bytes into out, a uint16 array of rvl_shape(data)");
    // cloud/compressed (ray_cloud.hpp): points and rays as bytes, x, y, z at their offsets in a point
    m.def("ray_cloud_ranges", [](InArray points, std::uint32_t point_step, std::uint32_t x, std::uint32_t y,
                                 std::uint32_t z, InArray rays, double range_step, HostArray steps) {
              const std::size_t count = steps.nbytes() / 2;
              if (steps.dtype() != nb::dtype<std::uint16_t>() or rays.dtype() != nb::dtype<float>()
                  or rays.nbytes() != count * 12 or points.nbytes() != count * point_step
                  or std::max({x, y, z}) + 4 > point_step)
              {
                  throw nb::value_error("ray_cloud_ranges: points, rays (float32, 3 per point) and steps (uint16) "
                                        "are not of one size");
              }
              const auto fit = durchblick::ray_cloud::ranges(
                  points.data(), count, {point_step, x, y, z}, static_cast<const float*>(rays.data()), range_step,
                  static_cast<std::uint16_t*>(steps.data()));
              return std::make_tuple(fit.valid, fit.low, fit.high, fit.deviation, fit.limit);
          }, "points"_a, "point_step"_a, "x"_a, "y"_a, "z"_a, "rays"_a, "range_step"_a, "steps"_a,
          "writes the points' ranges on their rays to steps; (valid, low, high, deviation, limit)");
    m.def("ray_cloud_points", [](InArray steps, InArray rays, double range_step, HostArray points,
                                 std::uint32_t point_step, std::uint32_t x, std::uint32_t y, std::uint32_t z) {
              const std::size_t count = steps.nbytes() / 2;
              if (steps.dtype() != nb::dtype<std::uint16_t>() or rays.dtype() != nb::dtype<float>()
                  or rays.nbytes() != count * 12 or points.nbytes() != count * point_step
                  or std::max({x, y, z}) + 4 > point_step)
              {
                  throw nb::value_error("ray_cloud_points: steps (uint16), rays (float32, 3 per point) and points "
                                        "are not of one size");
              }
              durchblick::ray_cloud::points(static_cast<const std::uint16_t*>(steps.data()), count,
                                            static_cast<const float*>(rays.data()), range_step,
                                            {point_step, x, y, z}, points.data());
          }, "steps"_a, "rays"_a, "range_step"_a, "points"_a, "point_step"_a, "x"_a, "y"_a, "z"_a,
          "writes x, y, z of the points of these ranges on their rays (NaN for 0)");
    // rectification (rectify.hpp): the map of a camera, as cv2.initUndistortRectifyMap gives it
    m.def("rectify_map", [](std::uint32_t width, std::uint32_t height, InArray k, const std::string& model, InArray d,
                            InArray r, InArray p, HostArray x, HostArray y) {
              const auto doubles = [](const InArray& a, std::size_t n, const char* what) {
                  if (a.dtype() != nb::dtype<double>() or (n != 0 and a.nbytes() != n * sizeof(double)))
                  {
                      throw nb::value_error((std::string("rectify_map: ") + what + " is not float64 of its size")
                                                .c_str());
                  }
                  const auto* v = static_cast<const double*>(a.data());
                  return std::vector<double>(v, v + a.nbytes() / sizeof(double));
              };
              durchblick::RectifyCamera c;
              c.width = width;
              c.height = height;
              const auto kv = doubles(k, 9, "k"), rv = doubles(r, 9, "r"), pv = doubles(p, 12, "p");
              std::copy(kv.begin(), kv.end(), c.k.begin());
              std::copy(rv.begin(), rv.end(), c.r.begin());
              std::copy(pv.begin(), pv.end(), c.p.begin());
              c.distortion_model = model;
              c.d = doubles(d, 0, "d");
              const std::size_t n = std::size_t{width} * height;
              if (x.dtype() != nb::dtype<float>() or y.dtype() != nb::dtype<float>() or x.nbytes() != n * 4
                  or y.nbytes() != n * 4)
              {
                  throw nb::value_error("rectify_map: x and y are not float32 of height x width");
              }
              durchblick::RectifyMap map;
              try
              {
                  map = durchblick::rectify_map(c);
              }
              catch (const std::invalid_argument& e)
              {
                  throw nb::value_error(e.what());
              }
              std::memcpy(x.data(), map.x.data(), n * 4);
              std::memcpy(y.data(), map.y.data(), n * 4);
          }, "width"_a, "height"_a, "k"_a, "distortion_model"_a, "d"_a, "r"_a, "p"_a, "x"_a, "y"_a,
          "writes the rectification map of the camera (rectify.hpp) into x and y (float32, height x width)");
    // pairing by stamp (pairing.hpp): history.PartnerHistory pairs with these
    m.def("partner_period_ns", &durchblick::partner_period_ns, "kept"_a,
          "the median step of the kept stamps, each once; None before two different stamps");
    m.def("partner_of", [](const std::vector<std::int64_t>& kept, std::int64_t stamp_ns,
                           std::optional<std::int64_t> max_offset_ns) {
              try
              {
                  return durchblick::partner_of(kept, stamp_ns, max_offset_ns);
              }
              catch (const std::invalid_argument& e)
              {
                  throw nb::value_error(e.what());
              }
          }, "kept"_a, "stamp_ns"_a, "max_offset_ns"_a.none(),
          "the stamp of the partner sample of the primary frame stamped stamp_ns (pairing.hpp); None if none (yet)");
    m.def("cuda_device_count", &durchblick::cuda_device_count, "CUDA devices; 0 without driver or device");
    m.def("cuda_device_uuid", &durchblick::cuda_device_uuid, "ordinal"_a,
          "the UUID of the CUDA device of this ordinal (32 hex digits); empty without a driver");
    m.def("cuda_device_ordinal", &durchblick::cuda_device_ordinal, "uuid"_a,
          "ordinal of the CUDA device with this UUID; -1 if this process does not see it");
    m.attr("MAX_SLOTS") = durchblick::kMaxSlots;
    m.attr("__version__") = DURCHBLICK_VERSION;

    namespace p = durchblick::protocol;
    nb::module_ protocol = m.def_submodule("protocol", "the wire protocol's names and versions (protocol.hpp)");
    protocol.attr("IMAGE_RAW") = std::string(p::kImageRaw);
    protocol.attr("COMPRESSED") = std::string(p::kCompressed);
    protocol.attr("SHM") = std::string(p::kShm);
    protocol.attr("CUDA") = std::string(p::kCuda);
    protocol.attr("CAMERA_INFO") = std::string(p::kCameraInfo);
    protocol.attr("INFO") = std::string(p::kInfo);
    protocol.attr("WANT") = std::string(p::kWant);
    protocol.attr("TF_STATIC") = std::string(p::kTfStatic);
    protocol.attr("TF") = std::string(p::kTf);
    protocol.attr("IMU") = std::string(p::kImu);
    protocol.attr("DIAGNOSTICS") = std::string(p::kDiagnostics);
    protocol.attr("JOINT_STATES") = std::string(p::kJointStates);
    protocol.attr("ODOM") = std::string(p::kOdom);
    protocol.attr("WRENCH") = std::string(p::kWrench);
    protocol.attr("CLOUD") = std::string(p::kCloud);
    protocol.attr("CLOUD_SHM") = std::string(p::kCloudShm);
    protocol.attr("CLOUD_CUDA") = std::string(p::kCloudCuda);
    protocol.attr("CLOUD_COMPRESSED") = std::string(p::kCloudCompressed);
    protocol.attr("CLOUD_RAYS") = std::string(p::kCloudRays);
    protocol.attr("SENT") = std::string(p::kSent);
    protocol.attr("CLOCK") = std::string(p::kClock);
    protocol.attr("EVENTS") = std::string(p::kEvents);
    protocol.attr("COMMAND") = std::string(p::kCommand);
    protocol.attr("CONTROLLER") = std::string(p::kController);
    protocol.attr("RECORD") = std::string(p::kRecord);
    protocol.attr("STATUS") = std::string(p::kStatus);
    protocol.attr("TORQUE_STEP") = std::string(p::kTorqueStep);
    protocol.attr("CONTROLLER_MANAGER") = std::string(p::kControllerManager);
    protocol.attr("LIST_CONTROLLERS") = std::string(p::kListControllers);
    protocol.attr("SWITCH_CONTROLLER") = std::string(p::kSwitchController);
    protocol.attr("UNLOAD_CONTROLLER") = std::string(p::kUnloadController);
    protocol.attr("CREATE_CONTROLLER") = std::string(p::kCreateController);
    protocol.attr("GET_CONTROLLER_CONFIG") = std::string(p::kGetControllerConfig);
    protocol.attr("GET_CONFIG_FILE") = std::string(p::kGetConfigFile);
    protocol.attr("LIST_PARAMETERS") = std::string(p::kListParameters);
    protocol.attr("DESCRIBE_PARAMETERS") = std::string(p::kDescribeParameters);
    protocol.attr("GET_PARAMETERS") = std::string(p::kGetParameters);
    protocol.attr("GET_PARAMETER_TYPES") = std::string(p::kGetParameterTypes);
    protocol.attr("SET_PARAMETERS") = std::string(p::kSetParameters);
    protocol.attr("SET_PARAMETERS_ATOMICALLY") = std::string(p::kSetParametersAtomically);
    protocol.attr("PARAMETER_EVENTS") = std::string(p::kParameterEvents);
    protocol.attr("PARAMETERS_TOKEN") = std::string(p::kParametersToken);
    protocol.attr("ATTACHMENT_VERSION") = p::kAttachmentVersion;
    protocol.attr("INFO_VERSION") = p::kInfoVersion;
    protocol.attr("SENT_VERSION") = p::kSentVersion;
    protocol.attr("RING_VERSION") = durchblick::kVersion;
    protocol.attr("DEFAULT_PORT") = p::kDefaultPort;
    nb::dict types;  // type name -> (schema, Zenoh encoding)
    for (const p::MessageType& t : {p::kImageType, p::kCompressedImageType, p::kCameraInfoType, p::kTFMessageType,
                                    p::kImuType, p::kDiagnosticArrayType, p::kShmImageType, p::kCudaImageType,
                                    p::kPointCloud2Type, p::kShmPointCloud2Type, p::kCudaPointCloud2Type,
                                    p::kCompressedPointCloud2Type, p::kRaysType,
                                    p::kDetection2DArrayType, p::kDetection3DArrayType, p::kClassificationType,
                                    p::kJointStateType, p::kOdometryType, p::kWrenchStampedType,
                                    p::kClockType, p::kFloat64MultiArrayType, p::kRobotStateType, p::kParameterEventType,
                                    p::kListParametersRequestType,
                                    p::kListParametersResponseType, p::kDescribeParametersRequestType,
                                    p::kDescribeParametersResponseType, p::kGetParametersRequestType,
                                    p::kGetParametersResponseType, p::kGetParameterTypesRequestType,
                                    p::kGetParameterTypesResponseType, p::kSetParametersRequestType,
                                    p::kSetParametersResponseType, p::kSetParametersAtomicallyRequestType,
                                    p::kSetParametersAtomicallyResponseType, p::kTorqueStepRequestType,
                                    p::kTorqueStepResponseType, p::kListControllersRequestType,
                                    p::kListControllersResponseType, p::kSwitchControllerRequestType,
                                    p::kSwitchControllerResponseType, p::kUnloadControllerRequestType,
                                    p::kUnloadControllerResponseType, p::kCreateControllerRequestType,
                                    p::kCreateControllerResponseType, p::kGetControllerConfigRequestType,
                                    p::kGetControllerConfigResponseType, p::kGetConfigFileRequestType,
                                    p::kGetConfigFileResponseType})
    {
        types[nb::str(std::string(t.name).c_str())] = nb::make_tuple(t.schema(), t.encoding());
    }
    protocol.attr("TYPES") = types;

    nb::class_<durchblick::HostRingWriter>(m, "HostRingWriter")
        .def(nb::init<std::uint32_t, std::uint64_t>(), "slot_count"_a, "slot_size"_a)
        .def(
            "write",
            [](durchblick::HostRingWriter& w, const InArray& a) {
                Written out{};
                {
                    nb::gil_scoped_release release;
                    out = w.write(a.data(), nbytes_of(a));
                }
                return pair(out);
            },
            "array"_a, "Copy a C-contiguous array into the next slot; (slot, seq).")
        .def(
            "loan",
            [](nb::handle self) {
                auto& w = nb::cast<durchblick::HostRingWriter&>(self);
                std::uint8_t* p = w.loan();
                return ByteView(p, {static_cast<std::size_t>(w.slot_size())}, self);
            },
            "The next slot as a writable uint8 array, to fill in place; then commit(nbytes).")
        .def("commit", [](durchblick::HostRingWriter& w, std::uint64_t n) { return pair(w.commit(n)); }, "nbytes"_a)
        .def_prop_ro("name", &durchblick::HostRingWriter::name)
        .def_prop_ro("slot_count", &durchblick::HostRingWriter::slot_count)
        .def_prop_ro("slot_size", &durchblick::HostRingWriter::slot_size);

    nb::class_<durchblick::HostRingReader>(m, "HostRingReader")
        .def(nb::init<const std::string&>(), "name"_a)
        .def(
            "read",
            [](const durchblick::HostRingReader& r, std::uint32_t slot, std::uint64_t seq, HostArray out) {
                nb::gil_scoped_release release;
                return r.read(slot, seq, out.data(), nbytes_of(out));
            },
            "slot"_a, "seq"_a, "out"_a, "Copy frame (slot, seq) into out; False if it was overwritten.")
        .def(
            "view",
            [](nb::handle self, std::uint32_t slot, std::uint64_t nbytes) {
                auto& r = nb::cast<const durchblick::HostRingReader&>(self);
                if (nbytes > r.slot_size())
                {
                    throw durchblick::RingError("view of " + std::to_string(nbytes) + " bytes, slot of " +
                                                std::to_string(r.slot_size()));
                }
                return ReadOnlyByteView(r.view(slot), {static_cast<std::size_t>(nbytes)}, self);
            },
            "slot"_a, "nbytes"_a, "The slot's bytes without a copy (read-only); check validate() after use.")
        .def("acquire", &durchblick::HostRingReader::acquire, "slot"_a, "seq"_a)
        .def("validate", &durchblick::HostRingReader::validate, "slot"_a, "seq"_a)
        .def("nbytes", &durchblick::HostRingReader::nbytes, "slot"_a)
        .def_prop_ro("name", &durchblick::HostRingReader::name)
        .def_prop_ro("slot_count", &durchblick::HostRingReader::slot_count)
        .def_prop_ro("slot_size", &durchblick::HostRingReader::slot_size);

    nb::class_<durchblick::DeviceBuffer>(m, "DeviceBuffer")
        .def_prop_ro("ptr", &durchblick::DeviceBuffer::ptr)
        .def_prop_ro("nbytes", &durchblick::DeviceBuffer::nbytes)
        .def_prop_ro("device", &durchblick::DeviceBuffer::device)
        .def(
            "download",
            [](const durchblick::DeviceBuffer& b, HostArray out) {
                if (nbytes_of(out) < b.nbytes())
                {
                    throw durchblick::RingError("download of " + std::to_string(b.nbytes()) + " bytes into " +
                                                std::to_string(nbytes_of(out)));
                }
                nb::gil_scoped_release release;
                b.download(out.data());
            },
            "out"_a)
        .def(
            "upload",
            [](durchblick::DeviceBuffer& b, const InArray& a) {
                if (nbytes_of(a) != b.nbytes())
                {
                    throw durchblick::RingError("upload of " + std::to_string(nbytes_of(a)) + " bytes into " +
                                                std::to_string(b.nbytes()));
                }
                nb::gil_scoped_release release;
                b.upload(a.data());
            },
            "array"_a, "Copy a C-contiguous array of nbytes bytes into the buffer.");
    m.def("make_device_buffer", &durchblick::make_device_buffer, "device"_a, "nbytes"_a,
          "A device buffer of its own (e.g. for a driver that uploads frames, or tests).");

    nb::class_<durchblick::CudaRingWriter>(m, "CudaRingWriter")
        .def(nb::init<int, std::uint32_t, std::uint64_t>(), "device"_a, "slot_count"_a, "slot_size"_a)
        .def(
            "write_device",
            [](durchblick::CudaRingWriter& w, std::uint64_t ptr, std::uint64_t nbytes, std::uint64_t stream) {
                Written out{};
                {
                    nb::gil_scoped_release release;
                    out = w.write_device(ptr, nbytes, stream);
                }
                return pair(out);
            },
            "ptr"_a, "nbytes"_a, "stream"_a = 0)
        .def(
            "wait_written",
            [](durchblick::CudaRingWriter& w, std::uint32_t slot) {
                nb::gil_scoped_release release;
                w.wait_written(slot);
            },
            "slot"_a, "Wait until the frame written into slot is there (write_device's source may then be freed).")
        .def(
            "synchronize_device",
            [](durchblick::CudaRingWriter& w) {
                nb::gil_scoped_release release;
                w.synchronize_device();
            },
            "Wait for all work of this process on the ring's device.")
        .def(
            "write_host",
            [](durchblick::CudaRingWriter& w, const InArray& a) {
                Written out{};
                {
                    nb::gil_scoped_release release;
                    out = w.write_host(a.data(), nbytes_of(a));
                }
                return pair(out);
            },
            "array"_a)
        .def(
            "read_host",
            [](durchblick::CudaRingWriter& w, std::uint32_t slot, HostArray out) {
                nb::gil_scoped_release release;
                w.read_host(slot, out.data(), nbytes_of(out));
            },
            "slot"_a, "out"_a)
        .def_prop_ro("endpoint", &durchblick::CudaRingWriter::endpoint)
        .def_prop_ro("device_uuid", &durchblick::CudaRingWriter::device_uuid)
        .def_prop_ro("device", &durchblick::CudaRingWriter::device)
        .def_prop_ro("slot_count", &durchblick::CudaRingWriter::slot_count)
        .def_prop_ro("slot_size", &durchblick::CudaRingWriter::slot_size);

    nb::class_<durchblick::CudaRingReader>(m, "CudaRingReader")
        .def(nb::init<const std::string&, int>(), "endpoint"_a, "target_device"_a = -1)
        .def(
            "copy",
            [](durchblick::CudaRingReader& r, std::uint32_t slot, std::uint64_t seq) {
                nb::gil_scoped_release release;
                return r.copy(slot, seq);
            },
            "slot"_a, "seq"_a, "A DeviceBuffer with the frame, or None if it was overwritten.")
        .def("slot_ptr", &durchblick::CudaRingReader::slot_ptr, "slot"_a)
        .def("acquire", &durchblick::CudaRingReader::acquire, "slot"_a, "seq"_a)
        .def("validate", &durchblick::CudaRingReader::validate, "slot"_a, "seq"_a)
        .def(
            "wait",
            [](const durchblick::CudaRingReader& r, std::uint32_t slot, std::uint64_t stream) {
                nb::gil_scoped_release release;
                r.wait(slot, stream);
            },
            "slot"_a, "stream"_a = 0)
        .def("nbytes", &durchblick::CudaRingReader::nbytes, "slot"_a)
        .def_prop_ro("device", &durchblick::CudaRingReader::device)
        .def_prop_ro("ring_device", &durchblick::CudaRingReader::ring_device)
        .def_prop_ro("direct", &durchblick::CudaRingReader::direct)
        .def_prop_ro("slot_count", &durchblick::CudaRingReader::slot_count);
}
