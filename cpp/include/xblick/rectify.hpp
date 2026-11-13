// A camera's image rectified, as ROS's image_proc and OpenCV's initUndistortRectifyMap compute it: without the lens
// distortion, turned by the CameraInfo's R into the camera that its P describes. For one camera R is the identity and
// the rectification only undistorts; for the two cameras of a stereo pair calibrated with cv::stereoRectify (R1, P1
// and R2, P2) the rectified images share their rows. Python's rgbd.Undistorter computes the same map with OpenCV.
//
// One map per calibration (rectify_map) gives every rectified pixel the point of the camera's image that it shows;
// remap() applies it to a frame on the CPU. 8-bit images are interpolated bilinearly, 16-bit images (depth, mono16)
// take the nearest pixel, because blending would make up values between a surface and what lies behind it. A
// rectified pixel whose point lies outside the camera's image is 0. Pixel centres are at integer coordinates.
//
// The map's x and y are what nppiRemap and cv::remap take, so a GPU node can apply it there
// (xblick-rectify does). In xblick::core, without the messages: rectify_camera() and rectified_info() take a
// CameraInfo (camera API, msg.cpp).
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace xblick
{
    struct CameraInfo;

    // The parts of a CameraInfo that the map depends on.
    struct RectifyCamera
    {
        std::uint32_t width = 0, height = 0;
        std::array<double, 9> k{};
        std::string distortion_model;  // plumb_bob (5), rational_polynomial (8), equidistant (4); none: all zeros
        std::vector<double> d;
        std::array<double, 9> r{1, 0, 0, 0, 1, 0, 0, 0, 1};
        std::array<double, 12> p{};
    };

    struct RectifyMap
    {
        std::uint32_t width = 0, height = 0;
        std::vector<float> x, y;  // per rectified pixel: where in the camera's image (-1 behind the camera)
        // the same for remap(), precomputed: the top-left neighbour's pixel index (-1: outside), the weights of the
        // right and the lower neighbour in 1/256, whether those exist (bit 0: right, 1: lower; at the last column or
        // row the point lies on it and its weight is 0), and the nearest pixel's index (-1: outside)
        std::vector<std::int32_t> index;
        std::vector<std::uint16_t> wx, wy;
        std::vector<std::uint8_t> neighbours;
        std::vector<std::int32_t> nearest;
    };

    // The map of a camera: R a rotation (the identity for one camera); P without skew, its fourth column free (a
    // stereo pair's baseline); P all zeros takes K. Throws std::invalid_argument for anything else.
    RectifyMap rectify_map(const RectifyCamera& camera);

    // How remap() treats a frame of an encoding: bytes per channel (1 bilinear, 2 nearest) and channels (1, 3, 4).
    struct PixelLayout
    {
        int bytes = 0;
        int channels = 0;  // 0: the encoding is not rectified
        int pixel_bytes() const { return bytes * channels; }
        bool supported() const { return channels > 0; }
    };
    PixelLayout rectify_layout(std::string_view encoding);
    // The encodings that rectify_layout takes, for messages.
    std::string rectify_encodings();

    // A frame of the map's size and of `layout`: src with rows of src_step bytes, dst with rows without padding.
    void remap(const RectifyMap& map, const PixelLayout& layout, const std::uint8_t* src, std::size_t src_step,
               std::uint8_t* dst);

    // Camera API (msg.cpp): the parts of a CameraInfo, and the CameraInfo of the rectified image: K = P's left 3x3
    // (K where P is all zeros), no distortion (plumb_bob, zeros), R = I, P as given (the right camera of a pair keeps
    // P[3] = -f b, which stereo readers take the baseline from), the same header.
    RectifyCamera rectify_camera(const CameraInfo& info);
    CameraInfo rectified_info(const CameraInfo& info);
}  // namespace xblick
