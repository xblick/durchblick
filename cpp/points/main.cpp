// durchblick-points: the point cloud node (point_cloud_node.hpp) as a program.
//
//   durchblick-points --depth robot/camera/depth [--color robot/camera/color] [--camera <camera>]
//                     [--output <camera>/points] [--cuda-device 0] [--max-offset-ms 10] [--remote compressed|raw]
//                     [--max-depth M] [--max-hz HZ] [--source robot] [--connect host[:port] ...]
//                     [--range-step 0.001] [--jpeg-quality 95]   (cloud/compressed, for readers on other hosts)
//                     [--listen host:port ...]
//
// --max-hz: at most so many clouds a second, of the newest depth frames; the frames between are not decoded; 0 or
// none: every frame it can take (the default).
//
// It prints how it is doing once a second (and publishes it on <camera>/diagnostics). Run it on the camera's host:
// the inputs then come from the camera's CUDA rings. Without --connect and --listen it uses the PC's Zenoh router
// (docs/concepts/sessions.md, "Zenoh routers"), through which readers on other hosts reach it.
#include <chrono>
#include <csignal>
#include <iostream>

#include "../tools/tools.hpp"
#include "durchblick/camera.hpp"
#include "durchblick/point_cloud_node.hpp"

namespace
{
    volatile std::sig_atomic_t stop = 0;
}

int main(int argc, char** argv)
{
    using namespace durchblick;
    tools::Args args(argc, argv);
    std::signal(SIGINT, [](int) { stop = 1; });
    std::signal(SIGTERM, [](int) { stop = 1; });
    try
    {
        const std::string remote = args.get("remote", "compressed");
        if (remote != "compressed" and remote != "raw")
        {
            throw std::invalid_argument("--remote " + remote + ": compressed or raw");
        }
        PointCloudNodeOptions options;
        options.depth = args.get("depth", "");
        options.color = args.get("color", "");
        options.camera = args.get("camera", "");
        options.output = args.get("output", "");
        options.cuda_device = std::stoi(args.get("cuda-device", "0"));
        options.source = args.get("source", "");
        options.max_offset = std::chrono::microseconds(
            static_cast<std::int64_t>(1000.0 * std::stod(args.get("max-offset-ms", "10"))));
        options.max_depth = std::stof(args.get("max-depth", "0"));
        options.range_step = std::stod(args.get("range-step", "0.001"));
        options.jpeg_quality = std::stoi(args.get("jpeg-quality", "95"));
        const std::string max_hz = args.get("max-hz", "0");  // the node's default: every frame
        options.max_hz = max_hz == "none" ? 0.0 : std::stod(max_hz);
        if (not (options.max_hz >= 0))
        {
            throw std::invalid_argument("--max-hz " + max_hz + ": a rate in Hz, or 0 or none for every frame");
        }
        options.remote = remote == "raw" ? Transport::Raw : Transport::Compressed;
        if (options.depth.empty())
        {
            throw std::invalid_argument("--depth <base of the depth stream> is needed");
        }
        zenoh::Session session = open_session({.connect = args.list("connect"), .listen = args.list("listen")});
        PointCloudNode node(session, options);
        std::cerr << "durchblick-points: " << options.depth << (options.color.empty() ? "" : " + " + options.color)
                  << " -> " << node.output() << " on CUDA device " << options.cuda_device << ", "
                  << rate_text(options.max_hz) << std::endl;
        auto next_report = std::chrono::steady_clock::now() + std::chrono::seconds(1);
        while (not stop)
        {
            node.step(std::chrono::milliseconds(100));
            if (std::chrono::steady_clock::now() >= next_report)
            {
                const DiagnosticStatus status = node.report();
                std::cerr << "durchblick-points: " << status.message;
                for (const KeyValue& kv : status.values)
                {
                    std::cerr << ", " << kv.key << " " << kv.value;
                }
                std::cerr << std::endl;
                next_report += std::chrono::seconds(1);
            }
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "durchblick-points: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
