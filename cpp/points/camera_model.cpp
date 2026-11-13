#include "camera_model.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <string>

#include "xblick/msg.hpp"

namespace xblick::points
{
    CameraModel camera_model(const CameraInfo& info)
    {
        CameraModel m;
        m.fx = static_cast<float>(info.k[0]);
        m.fy = static_cast<float>(info.k[4]);
        m.cx = static_cast<float>(info.k[2]);
        m.cy = static_cast<float>(info.k[5]);
        m.width = static_cast<int>(info.width);
        m.height = static_cast<int>(info.height);
        if (not(m.fx > 0.0f and m.fy > 0.0f) or m.width <= 0 or m.height <= 0)
        {
            throw std::invalid_argument("camera_info of " + info.header.frame_id + ": no focal length or size");
        }
        if (info.binning_x > 1 or info.binning_y > 1 or info.roi.width != 0 or info.roi.height != 0)
        {
            throw std::invalid_argument("camera_info of " + info.header.frame_id +
                                        ": binning and regions of interest are not supported");
        }
        const std::vector<double>& d = info.d;
        const auto all_zero = [&d] {
            for (double v : d)
            {
                if (v != 0.0)
                {
                    return false;
                }
            }
            return true;
        };
        if (info.distortion_model == "plumb_bob" or info.distortion_model == "rational_polynomial")
        {
            const std::size_t n = info.distortion_model == "plumb_bob" ? 5 : 8;
            if (d.size() != n)
            {
                throw std::invalid_argument("camera_info of " + info.header.frame_id + ": " + info.distortion_model +
                                            " has " + std::to_string(n) + " coefficients, not " +
                                            std::to_string(d.size()));
            }
            m.k1 = static_cast<float>(d[0]);
            m.k2 = static_cast<float>(d[1]);
            m.p1 = static_cast<float>(d[2]);
            m.p2 = static_cast<float>(d[3]);
            m.k3 = static_cast<float>(d[4]);
            if (n == 8)
            {
                m.k4 = static_cast<float>(d[5]);
                m.k5 = static_cast<float>(d[6]);
                m.k6 = static_cast<float>(d[7]);
            }
        }
        else if (not(info.distortion_model.empty() and all_zero()))
        {
            throw std::invalid_argument("camera_info of " + info.header.frame_id + ": distortion model '" +
                                        info.distortion_model + "' is not supported (plumb_bob, rational_polynomial)");
        }
        return m;
    }

    std::vector<float> rays(const CameraModel& m)
    {
        constexpr int kIterations = 100;
        const double tolerance = 1e-3 / std::max(m.fx, m.fy);  // 1e-3 px in normalized coordinates
        std::vector<float> out(2 * static_cast<std::size_t>(m.width) * m.height);
        for (int v = 0; v < m.height; ++v)
        {
            for (int u = 0; u < m.width; ++u)
            {
                // fixed-point iteration as OpenCV's undistortPoints, until the distorted ray meets the pixel
                const double x0 = (u - double(m.cx)) / m.fx;
                const double y0 = (v - double(m.cy)) / m.fy;
                double x = x0, y = y0;
                bool converged = false;
                for (int i = 0; i < kIterations and not converged; ++i)
                {
                    double xd, yd;
                    m.distort(x, y, xd, yd);
                    const double ex = xd - x0, ey = yd - y0;
                    if (std::abs(ex) < tolerance and std::abs(ey) < tolerance)
                    {
                        converged = true;
                        break;
                    }
                    // x0 = x * radial + tangential(x)  ->  x = (x0 - tangential) / radial, with the current x
                    const double r2 = x * x + y * y;
                    const double radial = (1.0 + r2 * (m.k1 + r2 * (m.k2 + r2 * double(m.k3)))) /
                                          (1.0 + r2 * (m.k4 + r2 * (m.k5 + r2 * double(m.k6))));
                    const double dx = 2.0 * m.p1 * x * y + m.p2 * (r2 + 2.0 * x * x);
                    const double dy = m.p1 * (r2 + 2.0 * y * y) + 2.0 * m.p2 * x * y;
                    x = (x0 - dx) / radial;
                    y = (y0 - dy) / radial;
                    if (not std::isfinite(x) or not std::isfinite(y))
                    {
                        break;
                    }
                }
                const std::size_t i = 2 * (static_cast<std::size_t>(v) * m.width + u);
                const float nan = std::numeric_limits<float>::quiet_NaN();
                out[i] = converged ? static_cast<float>(x) : nan;
                out[i + 1] = converged ? static_cast<float>(y) : nan;
            }
        }
        return out;
    }
}  // namespace xblick::points
