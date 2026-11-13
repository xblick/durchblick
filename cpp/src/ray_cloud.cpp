#include "durchblick/ray_cloud.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace durchblick::ray_cloud
{
    Fit ranges(const void* points, std::size_t count, const Layout& layout, const float* rays, double range_step,
               std::uint16_t* steps)
    {
        Fit fit;
        fit.low = std::numeric_limits<double>::infinity();
        fit.high = -std::numeric_limits<double>::infinity();
        const auto* bytes = static_cast<const std::uint8_t*>(points);
        const auto step = static_cast<float>(range_step);
        float largest = 1.0F;  // of a ray's components, among the valid points
        float deviation = 0.0F;
        for (std::size_t i = 0; i < count; ++i, bytes += layout.point_step, rays += 3)
        {
            float p[3];
            std::memcpy(&p[0], bytes + layout.x, 4);
            std::memcpy(&p[1], bytes + layout.y, 4);
            std::memcpy(&p[2], bytes + layout.z, 4);
            if (not(std::isfinite(p[0]) and std::isfinite(p[1]) and std::isfinite(p[2])))
            {
                steps[i] = 0;
                continue;
            }
            const float along = (p[0] * rays[0] + p[1] * rays[1] + p[2] * rays[2])
                                / ((rays[0] * rays[0] + rays[1] * rays[1] + rays[2] * rays[2]) * step);
            const float rounded = std::nearbyint(along);
            ++fit.valid;
            fit.low = std::min<double>(fit.low, rounded);
            fit.high = std::max<double>(fit.high, rounded);
            steps[i] = rounded >= 1.0F and rounded <= 65535.0F ? static_cast<std::uint16_t>(rounded) : 0;
            const float range = rounded * step;
            for (int a = 0; a < 3; ++a)
            {
                deviation = std::max(deviation, std::fabs(rays[a] * range - p[a]));
                largest = std::max(largest, std::fabs(rays[a]));
            }
        }
        if (fit.valid == 0)
        {
            fit.low = fit.high = 0;
        }
        fit.deviation = deviation;
        fit.limit = range_step * largest;
        return fit;
    }

    void points(const std::uint16_t* steps, std::size_t count, const float* rays, double range_step,
                const Layout& layout, void* points)
    {
        auto* bytes = static_cast<std::uint8_t*>(points);
        const auto step = static_cast<float>(range_step);
        const float nan = std::numeric_limits<float>::quiet_NaN();
        for (std::size_t i = 0; i < count; ++i, bytes += layout.point_step, rays += 3)
        {
            const float range = steps[i] ? static_cast<float>(steps[i]) * step : nan;
            const float p[3] = {rays[0] * range, rays[1] * range, rays[2] * range};
            std::memcpy(bytes + layout.x, &p[0], 4);
            std::memcpy(bytes + layout.y, &p[1], 4);
            std::memcpy(bytes + layout.z, &p[2], 4);
        }
    }
}  // namespace durchblick::ray_cloud
