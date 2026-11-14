// xblick-cpp-camera: a C++ publisher of a test pattern (tools.hpp), for checking C++ against Python and as an
// example of the C++ API.
//
//   xblick-cpp-camera --base test/camera/color --encoding rgb8 --size 640 480
//                         [--fps 30] [--seconds 10] [--codec png] [--cuda-device 0 [--device-frames]] [--jpeg-frames]
//                         [--points] [--diagnostics] [--transports shm,cuda] [--relayed-from HOST --origin-seq N]
//
// --transports: offer only these representations (PublisherOptions::transports). --relayed-from: as a relay of HOST's
// stream (the info record's relayed_from); with --origin-seq every frame goes out with Origin seq N, N+1, ... and source
// "origin", frame_id "OriginCamera" (a relay's pass-through of the source's header and attachment).
//
// --diagnostics: also a DiagnosticArray (status "cpp_camera") on <camera>/diagnostics once a second, <camera> the base
// without its last part.
//
// --params: parameters (parameters.hpp) as the node <camera>/cpp_camera: fps (changes the rate), pattern.shift and
// pattern.auto_shift (setting the shift turns the automatic shift off), codec (read-only); --param, --params-file,
// --dump-params, --params-read-only as every node.
//
// --points: a point cloud stream (points.hpp) of --size points, layout xyzrgb, test pattern tools::cloud_pattern.
// --rays (with --points): the clouds lie on a camera's rays (tools::ray_cloud_pattern), with cloud/compressed.
//
// --jpeg-frames: like an MJPEG camera, every frame also exists as a JPEG (written here, with the comment segment
// "xblick-test-jpeg <seq>" so that a subscriber can tell these bytes from a JPEG the publisher wrote); it is
// decoded (here: the pattern itself) only if somebody takes the pixels.
// --connect, --listen, --zenoh-mode peer|client: the Zenoh session (SessionOptions); none of them: the PC's router.
#include <atomic>
#include <chrono>
#include <optional>
#include <csignal>
#include <thread>

#include "xblick/camera.hpp"
#include "xblick/parameters.hpp"
#include "xblick/points.hpp"
#include "tools.hpp"

namespace
{
    volatile std::sig_atomic_t stop = 0;
}

int main(int argc, char** argv)
{
    using namespace xblick;
    tools::Args args(argc, argv);
    std::signal(SIGINT, [](int) { stop = 1; });
    std::signal(SIGTERM, [](int) { stop = 1; });
    try
    {
        zenoh::Session session = open_session({.connect = args.list("connect"), .listen = args.list("listen"),
                                                   .mode = args.get("zenoh-mode", "peer"), .program = "xblick-cpp-camera"});
        const std::string encoding = args.get("encoding", "rgb8");
        const auto size = args.list("size");
        const std::uint32_t width = size.size() == 2 ? std::stoul(size[0]) : 640;
        const std::uint32_t height = size.size() == 2 ? std::stoul(size[1]) : 480;
        PublisherOptions options{.encoding = encoding, .frame_id = "TestCamera", .source = "test"};
        options.codec = args.get("codec", kJpeg);
        options.camera_info = CameraInfo::pinhole(width, height, 500.0, 500.0, (width - 1) / 2.0, (height - 1) / 2.0,
                                                  options.frame_id);
        if (args.flag("cuda-device"))
        {
            options.cuda_device = std::stoi(args.get("cuda-device", "0"));
        }
        if (args.flag("transports"))
        {
            for (const std::string& t : args.list("transports"))
            {
                std::size_t start = 0;
                while (start <= t.size())
                {
                    const std::size_t end = t.find(',', start);
                    const std::string part = t.substr(start, end == std::string::npos ? std::string::npos : end - start);
                    if (not part.empty())
                    {
                        options.transports.push_back(part);
                    }
                    if (end == std::string::npos)
                    {
                        break;
                    }
                    start = end + 1;
                }
            }
        }
        options.relayed_from = args.get("relayed-from", "");
        if (not options.relayed_from.empty())
        {
            options.camera_info.reset();  // a relay's: camera_info comes from the source
        }
        const std::optional<std::int64_t> origin_seq =
            args.flag("origin-seq") ? std::optional<std::int64_t>(std::stoll(args.get("origin-seq", "0"))) : std::nullopt;
        const bool device_frames = args.flag("device-frames");
        const bool jpeg_frames = args.flag("jpeg-frames");
        const double fps = std::stod(args.get("fps", "30"));
        const double seconds = std::stod(args.get("seconds", "0"));
        if (args.flag("points"))
        {
            const bool rays = args.flag("rays");
            CloudPublisherOptions cloud_options{.layout = xyzrgb_layout(), .frame_id = options.frame_id,
                                                .source = options.source, .cuda_device = options.cuda_device};
            if (rays)  // the clouds lie on a camera's rays: cloud/compressed, with exact colours
            {
                cloud_options.rays = CloudRays{height, width, tools::ray_cloud_rays(width, height)};
                cloud_options.color_codec = "png";
            }
            PointCloudPublisher points(session, args.get("base", "test/camera/points"), cloud_options);
            std::shared_ptr<DeviceBuffer> device;
            if (device_frames)
            {
                device = make_device_buffer(*options.cuda_device, static_cast<std::uint64_t>(width) * height * 16);
            }
            std::cout << "ready" << std::endl;
            const auto start = std::chrono::steady_clock::now();
            auto next = start;
            std::uint64_t seq = 0;
            while (not stop and (seconds <= 0 or std::chrono::steady_clock::now() - start <
                                                     std::chrono::duration<double>(seconds)))
            {
                next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(1.0 / fps));
                if (points.subscribed())
                {
                    ++seq;
                    const std::vector<std::uint8_t> cloud =
                        rays ? tools::ray_cloud_pattern(width, height, seq) : tools::cloud_pattern(width, height, seq);
                    if (device)
                    {
                        device->upload(cloud.data());
                        points.publish(DeviceCloud{device->ptr(), height, width, 0, false});
                    }
                    else
                    {
                        points.publish(HostCloud{cloud.data(), height, width, false});
                    }
                }
                std::this_thread::sleep_until(next);
            }
            return 0;
        }
        const std::string base = args.get("base", "test/camera/color");
        std::atomic<double> rate{fps};
        std::atomic<std::int64_t> shift{0};
        std::atomic<bool> auto_shift{true};
        std::optional<Parameters> params;
        if (args.flag("params"))
        {
            params.emplace(session, "cpp_camera", base.substr(0, base.rfind('/')));
            params->configure(parameter_args(argc, argv));
            rate = params->declare("fps", fps, {.description = "frames a second", .unit = "Hz",
                                                .floating_range = FloatingPointRange{1.0, 60.0, 0.5}},
                                   [&](const ParameterValue& v) {
                                       rate = v.double_value;
                                       return std::string();
                                   },
                                   [&] { return ParameterValue::of(rate.load()); });
            params->declare("pattern.shift", std::int64_t{0}, {.integer_range = IntegerRange{0, 255, 1}},
                            [&](const ParameterValue& v) {
                                if (v.integer_value == 13)
                                {
                                    return std::string("the camera refused 13");
                                }
                                shift = v.integer_value;
                                auto_shift = false;  // a value given: manual, as a camera control
                                return std::string();
                            },
                            [&] { return ParameterValue::of(shift.load()); });
            params->declare("pattern.auto_shift", true, {},
                            [&](const ParameterValue& v) {
                                auto_shift = v.bool_value;
                                return std::string();
                            },
                            [&] { return ParameterValue::of(auto_shift.load()); });
            params->declare("codec", options.codec, {.choices = {ParameterValue::of("jpeg"), ParameterValue::of("png"), ParameterValue::of("rvl")},
                                                     .read_only = true});
            params->check_overrides();
        }
        CameraPublisher camera(session, base, options);
        std::optional<DiagnosticsPublisher> diagnostics;
        if (args.flag("diagnostics"))
        {
            diagnostics.emplace(session, base.substr(0, base.rfind('/')));
        }
        auto reported = std::chrono::steady_clock::now() - std::chrono::seconds(1);
        std::shared_ptr<DeviceBuffer> device;
        if (device_frames)
        {
            device = make_device_buffer(*options.cuda_device,
                                        static_cast<std::uint64_t>(width) * height * encoding_info(encoding).pixel_bytes());
        }
        std::cout << "ready" << std::endl;
        const auto start = std::chrono::steady_clock::now();
        auto next = start;
        for (std::uint64_t seq = 1; not stop; ++seq)
        {
            if (seconds > 0 and std::chrono::steady_clock::now() - start > std::chrono::duration<double>(seconds))
            {
                break;
            }
            next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(1.0 / rate.load()));
            if (diagnostics and std::chrono::steady_clock::now() - reported >= std::chrono::seconds(1))
            {
                reported = std::chrono::steady_clock::now();
                const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                    std::chrono::system_clock::now().time_since_epoch()).count();
                diagnostics->publish({Header{Time::from_ns(ns), ""},
                                      {DiagnosticStatus{DiagnosticStatus::kOk, "cpp_camera", "publishing", "", {}}}});
            }
            if (camera.subscribed())
            {
                // seq is the publisher's frame counter, so the pattern of frame n is pattern(n) on both sides
                const std::vector<std::uint8_t> frame = tools::pattern(encoding, width, height, seq);
                std::vector<std::uint8_t> jpeg;
                if (jpeg_frames)
                {
                    jpeg = compress(HostImage{frame.data(), height, width, 0}, encoding, kJpeg, 90, Header{}).data;
                    const std::string comment = "xblick-test-jpeg " + std::to_string(seq);
                    const auto length = static_cast<std::uint16_t>(comment.size() + 2);
                    std::vector<std::uint8_t> segment = {0xFF, 0xFE, static_cast<std::uint8_t>(length >> 8),
                                                         static_cast<std::uint8_t>(length & 0xFF)};
                    segment.insert(segment.end(), comment.begin(), comment.end());
                    jpeg.insert(jpeg.begin() + 2, segment.begin(), segment.end());  // after the start marker
                }
                if (origin_seq)
                {
                    const Origin origin{Header{Time::from_ns(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                                                 std::chrono::system_clock::now().time_since_epoch())
                                                                 .count()),
                                               "OriginCamera"},
                                        *origin_seq + static_cast<std::int64_t>(seq) - 1, "origin"};
                    if (device)
                    {
                        device->upload(frame.data());
                        camera.publish(DeviceImage{device->ptr(), height, width, 0}, origin);
                    }
                    else
                    {
                        camera.publish(HostImage{frame.data(), height, width, 0}, origin);
                    }
                }
                else if (jpeg_frames and not camera.needs_pixels())
                {
                    camera.publish_jpeg(jpeg);
                }
                else if (device)
                {
                    device->upload(frame.data());
                    camera.publish(DeviceImage{device->ptr(), height, width, 0}, std::nullopt, jpeg);
                }
                else
                {
                    camera.publish(HostImage{frame.data(), height, width, 0}, std::nullopt, jpeg);
                }
            }
            else
            {
                --seq;  // frames are counted only when published
            }
            std::this_thread::sleep_until(next);
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "xblick-cpp-camera: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
