// xblick-cpp-probe: a C++ subscriber that prints one JSON line per frame (encoding, size, sequence number,
// checksum of the pixels), for checking C++ against Python and as an example of the C++ API.
//
//   xblick-cpp-probe --connect host:7447 --base robot/test_camera/color --transport auto|raw|compressed|shm|cuda
//                        [--view] [--frames 10] [--timeout 10] [--camera-info] [--points [--local cuda]]
//                        [--max-hz 5]
//   xblick-cpp-probe --connect host:7447 --params <namespace>/<name> [--set NAME:=VALUE]
//
// --params: a node's parameters (parameters.hpp, ParameterClient): with --set that change first (a line with its
// result), then a line per parameter (name, type, value, read_only, additional_constraints).
// --points: a point cloud stream (points.hpp); a line per cloud with its fields instead of the encoding.
#include <chrono>
#include <map>

#include "xblick/camera.hpp"
#include "xblick/parameters.hpp"
#include "xblick/points.hpp"
#include "tools.hpp"

namespace
{
    // The bytes of a received frame or cloud in host memory (a device copy is downloaded; a device view is not read).
    std::string sum_of(const std::uint8_t* host, std::uint64_t device_ptr, const std::shared_ptr<const void>& owner,
                       bool copy, std::uint32_t height, std::uint32_t row, std::uint32_t step)
    {
        using namespace xblick;
        if (device_ptr == 0)
        {
            return std::to_string(tools::checksum(host, height, row, step));
        }
        if (not copy)  // a view's owner is the ring, a copy's its DeviceBuffer
        {
            return "null";
        }
        auto buffer = std::static_pointer_cast<const DeviceBuffer>(owner);
        std::vector<std::uint8_t> bytes(buffer->nbytes());
        buffer->download(bytes.data());
        return std::to_string(tools::checksum(bytes.data(), height, row, row));
    }

    int probe_points(zenoh::Session& session, const xblick::tools::Args& args, xblick::Transport transport,
                     bool copy)
    {
        using namespace xblick;
        CloudSubscriberOptions options;
        options.transport = transport;
        options.local = args.get("local", "shm") == "cuda" ? Transport::Cuda : Transport::Shm;
        options.copy = copy;
        options.max_hz = std::stod(args.get("max-hz", "0"));
        PointCloudSubscriber clouds(session, args.get("base", "robot/test_camera/points"), options);
        const int wanted = std::stoi(args.get("frames", "10"));
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::duration<double>(std::stod(args.get("timeout", "10")));
        int got = 0;
        while (got < wanted and std::chrono::steady_clock::now() < deadline)
        {
            auto cloud = clouds.latest(true, std::chrono::milliseconds(100));
            if (not cloud)
            {
                continue;
            }
            ++got;
            const std::string sum = sum_of(cloud->data(), cloud->device_ptr, cloud->owner, copy, cloud->height,
                                           cloud->row_step, cloud->row_step);
            std::string fields;
            for (const PointField& f : cloud->fields)
            {
                fields += (fields.empty() ? "\"" : ",\"") + f.name + "\"";
            }
            const auto t = clouds.transport();
            std::cout << "{\"seq\":" << cloud->seq.value_or(-1) << ",\"fields\":[" << fields
                      << "],\"point_step\":" << cloud->point_step << ",\"height\":" << cloud->height
                      << ",\"width\":" << cloud->width << ",\"is_dense\":" << (cloud->is_dense ? "true" : "false")
                      << ",\"transport\":\"" << (t ? to_string(*t) : "?") << "\",\"checksum\":" << sum
                      << ",\"source\":\"" << cloud->source << "\",\"frame_id\":\"" << cloud->header.frame_id
                      << "\",\"valid\":" << (cloud->is_valid() ? "true" : "false") << ",\"age_at_send_ns\":"
                      << (cloud->age_at_send_ns() ? std::to_string(*cloud->age_at_send_ns()) : "null") << "}"
                      << std::endl;
        }
        std::cerr << "xblick-cpp-probe: " << got << " clouds, stale " << clouds.stale() << std::endl;
        return got == wanted ? 0 : 2;
    }

    std::string quoted(const std::string& s)
    {
        std::string out = "\"";
        for (const char c : s)
        {
            out += (c == '"' or c == '\\') ? std::string("\\") + c : std::string(1, c);
        }
        return out + "\"";
    }

    int probe_parameters(zenoh::Session& session, const xblick::tools::Args& args)
    {
        using namespace xblick;
        ParameterClient client(session, args.get("params", ""));
        if (args.flag("set"))
        {
            const std::string assignment = args.get("set", "");
            const auto sep = assignment.find(":=");
            const std::string name = assignment.substr(0, sep);
            const auto types = client.types({name});
            const auto result = client.set({Parameter{name, parameter_from_text(types.types.at(0),
                                                                                assignment.substr(sep + 2))}});
            std::cout << "{\"set\":" << quoted(name) << ",\"successful\":"
                      << (result.results.at(0).successful ? "true" : "false")
                      << ",\"reason\":" << quoted(result.results.at(0).reason) << "}" << std::endl;
        }
        const auto names = client.list().names;
        const auto descriptors = client.describe(names).descriptors;
        const auto values = client.get(names).values;
        for (std::size_t i = 0; i < names.size(); ++i)
        {
            const ParameterDescriptor& d = descriptors[i];
            std::cout << "{\"name\":" << quoted(names[i]) << ",\"type\":" << int(d.type)
                      << ",\"value\":" << quoted(parameter_text(values[i]))
                      << ",\"read_only\":" << (d.read_only ? "true" : "false")
                      << ",\"constraints\":" << quoted(d.additional_constraints) << "}" << std::endl;
        }
        return 0;
    }
}  // namespace

int main(int argc, char** argv)
{
    using namespace xblick;
    tools::Args args(argc, argv);
    try
    {
        zenoh::Session session = open_session({.connect = args.list("connect"), .listen = args.list("listen")});
        const std::string name = args.get("transport", "auto");
        const std::map<std::string, Transport> transports = {{"auto", Transport::Auto}, {"raw", Transport::Raw},
                                                             {"compressed", Transport::Compressed},
                                                             {"shm", Transport::Shm}, {"cuda", Transport::Cuda}};
        if (not transports.count(name))
        {
            throw std::invalid_argument("--transport " + name);
        }
        if (args.flag("points"))
        {
            return probe_points(session, args, transports.at(name), not args.flag("view"));
        }
        if (args.flag("params"))
        {
            return probe_parameters(session, args);
        }
        SubscriberOptions options;
        options.transport = transports.at(name);
        options.copy = not args.flag("view");
        options.camera_info = args.flag("camera-info");
        options.max_hz = std::stod(args.get("max-hz", "0"));
        ImageSubscriber images(session, args.get("base", "robot/test_camera/color"), options);
        const int frames = std::stoi(args.get("frames", "10"));
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(std::stod(args.get("timeout", "10")));
        int got = 0;
        while (got < frames and std::chrono::steady_clock::now() < deadline)
        {
            auto frame = images.latest(true, std::chrono::milliseconds(100));
            if (not frame)
            {
                continue;
            }
            ++got;
            const std::uint32_t row = frame->width * static_cast<std::uint32_t>(encoding_info(frame->encoding).pixel_bytes());
            const std::string sum = sum_of(frame->data(), frame->device_ptr, frame->owner, options.copy,
                                           frame->height, row, frame->step);
            std::string info = "null";
            if (options.camera_info)
            {
                if (auto ci = images.camera_info())
                {
                    info = "{\"width\":" + std::to_string(ci->width) + ",\"fx\":" + std::to_string(ci->k[0]) + "}";
                }
            }
            const auto t = images.transport();
            std::cout << "{\"seq\":" << frame->seq.value_or(-1) << ",\"encoding\":\"" << frame->encoding
                      << "\",\"height\":" << frame->height << ",\"width\":" << frame->width
                      << ",\"transport\":\"" << (t ? to_string(*t) : "?") << "\",\"checksum\":" << sum
                      << ",\"source\":\"" << frame->source << "\",\"stamp_ns\":" << frame->stamp_ns()
                      << ",\"age_at_send_ns\":"
                      << (frame->age_at_send_ns() ? std::to_string(*frame->age_at_send_ns()) : "null")
                      << ",\"valid\":" << (frame->is_valid() ? "true" : "false") << ",\"camera_info\":" << info << "}"
                      << std::endl;
        }
        std::cerr << "xblick-cpp-probe: " << got << " frames, stale " << images.stale() << ", skipped for max-hz "
                  << images.skipped_max_hz() << std::endl;
        return got == frames ? 0 : 2;
    }
    catch (const std::exception& e)
    {
        std::cerr << "xblick-cpp-probe: " << e.what() << std::endl;
        return 1;
    }
}
