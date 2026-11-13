#pragma once
// A cloud whose points lie on fixed rays, as one range per point: the arithmetic of cloud/compressed
// (durchblick_msgs/msg/CompressedPointCloud2; Python's points.py describes the transport). A point is its ray's
// direction times its range; the range is sent as a 16-bit number of steps of `range_step` metres, 0 for no point.
// A camera's rays are (x/z, y/z, 1) of each pixel, so that the range is z; a scanner's are its beams' unit vectors.

#include <cstddef>
#include <cstdint>

namespace durchblick::ray_cloud
{
    // Where x, y and z (float32) are in a point of `point_step` bytes.
    struct Layout
    {
        std::uint32_t point_step = 16;
        std::uint32_t x = 0;
        std::uint32_t y = 4;
        std::uint32_t z = 8;
    };

    // How a cloud fits its rays. A cloud can be sent if it has no valid point, or if 1 <= low, high <= 65535 and
    // deviation <= limit.
    struct Fit
    {
        std::size_t valid = 0;  // points with finite x, y and z
        double low = 0;         // the smallest and the largest range of a valid point, in steps (rounded)
        double high = 0;
        double deviation = 0;   // the farthest a valid point is from its rounded place on its ray, per axis (metres)
        double limit = 0;       // what rounding alone explains: range_step times the largest component of a ray
    };

    // The ranges of `count` points on `rays` (3 float32 per point) in steps of `range_step`, written to `steps`
    // (0 for a point that is not finite, and for a range outside 1..65535, which Fit reports).
    Fit ranges(const void* points, std::size_t count, const Layout& layout, const float* rays, double range_step,
               std::uint16_t* steps);

    // The reverse: writes x, y and z of `count` points (NaN where the range is 0); the points' other bytes stay.
    void points(const std::uint16_t* steps, std::size_t count, const float* rays, double range_step,
                const Layout& layout, void* points);
}  // namespace durchblick::ray_cloud
