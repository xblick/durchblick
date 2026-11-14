// The point cloud node (xblick::points, built with XBLICK_POINTS; needs a CUDA toolkit): organized point
// clouds of a depth stream, coloured from a colour stream, computed once per depth frame on the GPU for all readers
// and published as a point cloud stream (points.hpp). It works with any camera whose streams carry camera_info and
// whose colour camera is placed by <camera>/tf_static (or is in the depth camera's frame); the program
// xblick-points and other C++ programs run it.
//
//   xblick::PointCloudNode node(session, {.depth = "test/camera/depth",
//                                             .color = "test/camera/color"});
//   while (running) { node.step(std::chrono::milliseconds(100)); if (second_passed) node.report(); }
//
// The inputs are taken only while somebody takes the points: from the CUDA rings on the camera's host (no copy through
// the CPU), from the compressed streams elsewhere (lossless RVL or PNG depth, JPEG colour). A point is
// (x, y, z) in metres in the depth camera's frame (the frame_id of its camera_info), NaN without depth (or beyond
// max_depth); its colour is
// the colour pixel its projection meets (black where the colour camera does not see it; no test for occlusion, so at
// an object's edge a point the colour camera cannot see takes the colour of what is in front of it).
#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <zenoh.hxx>

#include "xblick/camera.hpp"
#include "xblick/msg.hpp"

namespace xblick
{
    struct PointCloudNodeOptions
    {
        std::string depth;   // base of the depth stream: 16UC1 or mono16 in millimetres, or 32FC1 in metres
        std::string color;   // base of the colour stream (rgb8, bgr8, rgba8, bgra8); empty: points without colour
        std::string camera;  // where tf_static and diagnostics are; empty: the depth stream's parent
        std::string output;  // base of the point cloud stream; empty: <camera>/points
        int cuda_device = 0;
        std::string source;  // in every cloud's attachment
        // largest difference of the depth and the colour frame's stamps that pairs them
        std::chrono::microseconds max_offset{10000};
        // metres: points farther away (z) are NaN; 0: no limit
        float max_depth = 0.0f;
        std::uint32_t slots = 4;
        // cloud/compressed, what a reader on another host takes (points.hpp): the points' z in steps of range_step
        // metres (16 bits: up to 65.5 m at 1 mm; a point farther away stops the node: set max_depth or a larger
        // step) and their colours as JPEG of this quality
        double range_step = 0.001;
        int jpeg_quality = 95;
        Transport remote = Transport::Compressed;  // how inputs of a camera on another host come (or Raw)
        // At most so many clouds a second, of the newest depth frames: the depth subscribed with this max_hz
        // (SubscriberOptions), the frames between skipped before they are decoded or read from their ring; the colour
        // stream is the depth's partner, its samples kept undecoded and only the one of a depth frame taken decoded
        // (no pair is lost to the limit). 0: every frame.
        double max_hz = 0.0;
    };

    // The effective rate in words, as the Python nodes say it: "max 30 Hz" or "every frame" (max_hz 0).
    std::string rate_text(double max_hz);

    struct PointCloudNodeStats
    {
        std::uint64_t clouds = 0;
        std::uint64_t depth_frames = 0;
        std::uint64_t color_frames = 0;  // decoded: one per depth frame paired with colour
        std::uint64_t unpaired = 0;  // depth frames dropped without a colour frame of their time
        std::uint64_t replaced = 0;  // depth frames dropped while waiting for a colour frame, a newer one came
        double ms = 0.0;             // host time of the published clouds' steps (input copies, kernel launch, sends)
    };

    class PointCloudNode
    {
    public:
        PointCloudNode(zenoh::Session& session, PointCloudNodeOptions options);
        ~PointCloudNode();
        PointCloudNode(const PointCloudNode&) = delete;
        PointCloudNode& operator=(const PointCloudNode&) = delete;

        // Waits up to timeout for a depth frame (only while somebody takes the points, else it drops its input
        // subscriptions and waits), makes its cloud and publishes it. True if a cloud went out.
        bool step(std::chrono::milliseconds timeout);
        // How the node is doing since the last report; also published on <camera>/diagnostics (status
        // "point_cloud").
        DiagnosticStatus report();

        const PointCloudNodeStats& stats() const;
        const std::string& output() const;
        const std::string& camera() const;

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}  // namespace xblick
