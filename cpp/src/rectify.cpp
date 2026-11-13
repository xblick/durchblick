#include "xblick/rectify.hpp"

#include <cmath>
#include <cstring>
#include <stdexcept>

namespace xblick
{
    namespace
    {
        enum class Model
        {
            None,
            PlumbBob,      // k1 k2 p1 p2 k3
            Rational,      // k1 k2 p1 p2 k3 k4 k5 k6
            Equidistant,   // k1 k2 k3 k4 (OpenCV's fisheye)
        };

        Model model_of(const RectifyCamera& c)
        {
            bool zeros = true;
            for (const double v : c.d)
            {
                zeros = zeros and v == 0.0;
            }
            if (c.distortion_model == "rational_polynomial" and c.d.size() == 8)
            {
                return Model::Rational;
            }
            if (c.distortion_model == "plumb_bob" and c.d.size() == 5)
            {
                return Model::PlumbBob;
            }
            if (c.distortion_model == "equidistant" and c.d.size() == 4)
            {
                return Model::Equidistant;
            }
            if (zeros)
            {
                return Model::None;
            }
            throw std::invalid_argument("rectification: distortion model '" + c.distortion_model + "' with " +
                                        std::to_string(c.d.size()) +
                                        " coefficients; plumb_bob (5), rational_polynomial (8), equidistant (4) or "
                                        "none");
        }

        void check_rotation(const std::array<double, 9>& r)
        {
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    const double dot = r[3 * i] * r[3 * j] + r[3 * i + 1] * r[3 * j + 1] + r[3 * i + 2] * r[3 * j + 2];
                    if (std::abs(dot - (i == j ? 1.0 : 0.0)) > 1e-6)
                    {
                        throw std::invalid_argument("rectification: the camera_info's R is no rotation");
                    }
                }
            }
            const double det = r[0] * (r[4] * r[8] - r[5] * r[7]) - r[1] * (r[3] * r[8] - r[5] * r[6]) +
                               r[2] * (r[3] * r[7] - r[4] * r[6]);
            if (std::abs(det - 1.0) > 1e-6)
            {
                throw std::invalid_argument("rectification: the camera_info's R is a reflection, no rotation");
            }
        }

        // P, or K as a P where P is all zeros (a CameraInfo without rectification)
        std::array<double, 12> projection_of(const RectifyCamera& c)
        {
            bool zeros = true;
            for (const double v : c.p)
            {
                zeros = zeros and v == 0.0;
            }
            if (not zeros)
            {
                return c.p;
            }
            const auto& k = c.k;
            return {k[0], k[1], k[2], 0.0, k[3], k[4], k[5], 0.0, k[6], k[7], k[8], 0.0};
        }
    }  // namespace

    RectifyMap rectify_map(const RectifyCamera& c)
    {
        const Model model = model_of(c);
        if (c.width == 0 or c.height == 0)
        {
            throw std::invalid_argument("rectification: the camera_info has no image size");
        }
        check_rotation(c.r);
        const std::array<double, 12> p = projection_of(c);
        if (p[1] != 0.0 or p[4] != 0.0 or p[8] != 0.0 or p[9] != 0.0 or p[10] != 1.0 or p[11] != 0.0 or
            not(p[0] > 0.0) or not(p[5] > 0.0))
        {
            throw std::invalid_argument("rectification: P must be [fx' 0 cx' Tx; 0 fy' cy' Ty; 0 0 1 0]");
        }
        const auto& k = c.k;
        const auto& r = c.r;
        const auto& d = c.d;
        const auto coefficient = [&d](std::size_t i) { return i < d.size() ? d[i] : 0.0; };
        RectifyMap map;
        map.width = c.width;
        map.height = c.height;
        const std::size_t n = std::size_t(c.width) * c.height;
        map.x.resize(n);
        map.y.resize(n);
        for (std::uint32_t v = 0; v < c.height; ++v)
        {
            for (std::uint32_t u = 0; u < c.width; ++u)
            {
                // the ray of the rectified pixel, turned back into the camera by R^T, then the camera's distortion
                const double xr = (u - p[2]) / p[0];
                const double yr = (v - p[6]) / p[5];
                const double X = r[0] * xr + r[3] * yr + r[6];
                const double Y = r[1] * xr + r[4] * yr + r[7];
                const double Z = r[2] * xr + r[5] * yr + r[8];
                const std::size_t i = std::size_t(v) * c.width + u;
                if (not(Z > 0.0))
                {
                    map.x[i] = map.y[i] = -1.0f;  // behind the camera: outside
                    continue;
                }
                const double x = X / Z, y = Y / Z;
                double xd = x, yd = y;
                if (model == Model::Equidistant)
                {
                    const double radius = std::sqrt(x * x + y * y);
                    const double theta = std::atan(radius), t2 = theta * theta;
                    const double k1 = coefficient(0), k2 = coefficient(1), k3 = coefficient(2), k4 = coefficient(3);
                    const double theta_d = theta * (1 + t2 * (k1 + t2 * (k2 + t2 * (k3 + t2 * k4))));
                    const double scale = radius > 1e-12 ? theta_d / radius : 1.0;
                    xd = x * scale;
                    yd = y * scale;
                }
                else if (model != Model::None)
                {
                    // OpenCV's rational model; plumb_bob without k4..k6
                    const double k1 = coefficient(0), k2 = coefficient(1), p1 = coefficient(2), p2 = coefficient(3);
                    const double k3 = coefficient(4);
                    const double k4 = model == Model::Rational ? coefficient(5) : 0.0;
                    const double k5 = model == Model::Rational ? coefficient(6) : 0.0;
                    const double k6 = model == Model::Rational ? coefficient(7) : 0.0;
                    const double r2 = x * x + y * y;
                    const double radial =
                        (1 + r2 * (k1 + r2 * (k2 + r2 * k3))) / (1 + r2 * (k4 + r2 * (k5 + r2 * k6)));
                    xd = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
                    yd = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;
                }
                map.x[i] = float(k[0] * xd + k[1] * yd + k[2]);
                map.y[i] = float(k[4] * yd + k[5]);
            }
        }
        const int w = int(c.width), h = int(c.height);
        map.index.assign(n, -1);
        map.wx.assign(n, 0);
        map.wy.assign(n, 0);
        map.neighbours.assign(n, 0);
        map.nearest.assign(n, -1);
        for (std::size_t i = 0; i < n; ++i)
        {
            const float sx = map.x[i], sy = map.y[i];
            if (not(sx >= 0.0f and sy >= 0.0f and sx <= float(w - 1) and sy <= float(h - 1)))
            {
                continue;
            }
            const int x0 = int(sx), y0 = int(sy);
            map.index[i] = y0 * w + x0;
            map.wx[i] = std::uint16_t(std::lround((sx - float(x0)) * 256.0f));
            map.wy[i] = std::uint16_t(std::lround((sy - float(y0)) * 256.0f));
            map.neighbours[i] = std::uint8_t((x0 < w - 1 ? 1 : 0) | (y0 < h - 1 ? 2 : 0));
            map.nearest[i] = std::int32_t(std::lround(sy)) * w + std::int32_t(std::lround(sx));
        }
        return map;
    }

    PixelLayout rectify_layout(std::string_view encoding)
    {
        if (encoding == "mono8" or encoding == "8UC1")
        {
            return {1, 1};
        }
        if (encoding == "rgb8" or encoding == "bgr8" or encoding == "8UC3")
        {
            return {1, 3};
        }
        if (encoding == "rgba8" or encoding == "bgra8" or encoding == "8UC4")
        {
            return {1, 4};
        }
        if (encoding == "mono16" or encoding == "16UC1")
        {
            return {2, 1};
        }
        return {};
    }

    std::string rectify_encodings()
    {
        return "mono8, rgb8, bgr8, rgba8, bgra8 (bilinear), mono16 and 16UC1 (nearest pixel)";
    }

    void remap(const RectifyMap& map, const PixelLayout& layout, const std::uint8_t* src, std::size_t src_step,
               std::uint8_t* dst)
    {
        if (not layout.supported())
        {
            throw std::invalid_argument("rectification: an encoding it does not take (" + rectify_encodings() + ")");
        }
        const std::size_t n = map.index.size();
        if (layout.bytes == 2)
        {
            // the nearest pixel: a depth or another 16-bit value is never blended with its neighbours
            for (std::size_t i = 0; i < n; ++i)
            {
                std::uint8_t* out = dst + 2 * i;
                if (map.nearest[i] < 0)
                {
                    out[0] = out[1] = 0;
                    continue;
                }
                const std::size_t x = std::size_t(map.nearest[i]) % map.width;
                const std::size_t y = std::size_t(map.nearest[i]) / map.width;
                std::memcpy(out, src + y * src_step + x * 2, 2);
            }
            return;
        }
        const std::size_t c = std::size_t(layout.channels);
        for (std::size_t i = 0; i < n; ++i)
        {
            std::uint8_t* out = dst + c * i;
            if (map.index[i] < 0)
            {
                std::memset(out, 0, c);
                continue;
            }
            const std::size_t x0 = std::size_t(map.index[i]) % map.width, y0 = std::size_t(map.index[i]) / map.width;
            const std::uint8_t* a = src + y0 * src_step + x0 * c;
            const std::uint8_t* b = a + ((map.neighbours[i] & 1) ? c : 0);
            const std::uint8_t* e = a + ((map.neighbours[i] & 2) ? src_step : 0);
            const std::uint8_t* f = e + (b - a);
            const std::uint32_t wx = map.wx[i], wy = map.wy[i];
            const std::uint32_t wa = (256 - wx) * (256 - wy), wb = wx * (256 - wy), we = (256 - wx) * wy, wf = wx * wy;
            for (std::size_t ch = 0; ch < c; ++ch)
            {
                out[ch] = std::uint8_t((a[ch] * wa + b[ch] * wb + e[ch] * we + f[ch] * wf + 32768) >> 16);
            }
        }
    }
}  // namespace xblick
