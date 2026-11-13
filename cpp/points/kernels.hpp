// The GPU work of the point cloud node (kernels.cu): one thread per depth pixel.
#pragma once

#include <cstdint>

#include <cuda_runtime_api.h>

#include "camera_model.hpp"

namespace xblick::points
{
    enum class DepthFormat
    {
        Millimetres16,  // 16UC1, mono16 (ROS REP 118)
        Metres32,       // 32FC1
    };

    // Channel order of the colour image.
    enum class ColorFormat
    {
        Rgb8,
        Bgr8,
        Rgba8,
        Bgra8,
    };

    struct ColorInput
    {
        const std::uint8_t* pixels = nullptr;  // device memory, rows without padding
        ColorFormat format = ColorFormat::Rgb8;
        CameraModel camera;
        float rotation[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};  // p_color = R p_depth + t (metres)
        float translation[3] = {0, 0, 0};
    };

    // Points (x, y, z, rgb as PCL packs it; 16 bytes) of a depth image, written to `points` on `stream`: NaN where the
    // depth is 0 or not finite or beyond max_depth (metres; 0: no limit) or the pixel has no ray; rgb from the colour
    // pixel nearest to the point's projection into the colour camera (0, black, where the colour camera does not see
    // it); without colour, rgb is 0.
    void depth_to_points(const void* depth, DepthFormat format, int width, int height, const float* rays,
                         float max_depth, const ColorInput* color, void* points, cudaStream_t stream);
}  // namespace xblick::points
