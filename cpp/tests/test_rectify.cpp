// rectify.hpp without messages: the map of an ideal camera is the identity, the remap is exact on a linear image,
// a stereo P's baseline does not move the map, and a CameraInfo the map cannot take is refused. The comparison with
// OpenCV for every distortion model is in tests/test_rectify.py.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

#include "xblick/rectify.hpp"

namespace
{
    int failures = 0;

    void check(bool ok, const std::string& what)
    {
        if (not ok)
        {
            ++failures;
            std::printf("FAIL: %s\n", what.c_str());
        }
    }

    xblick::RectifyCamera ideal(std::uint32_t w, std::uint32_t h)
    {
        xblick::RectifyCamera c;
        c.width = w;
        c.height = h;
        c.k = {100, 0, 31.5, 0, 100, 23.5, 0, 0, 1};
        c.distortion_model = "plumb_bob";
        c.d = {0, 0, 0, 0, 0};
        c.p = {100, 0, 31.5, 0, 0, 100, 23.5, 0, 0, 0, 1, 0};
        return c;
    }

    bool throws(xblick::RectifyCamera c)
    {
        try
        {
            xblick::rectify_map(c);
        }
        catch (const std::invalid_argument&)
        {
            return true;
        }
        return false;
    }
}  // namespace

int main()
{
    using namespace xblick;
    const std::uint32_t w = 64, h = 48;

    // an ideal camera: the identity; remap returns the image as it is, 8 and 16 bit
    const RectifyMap identity = rectify_map(ideal(w, h));
    bool same = true;
    for (std::uint32_t v = 0; v < h; ++v)
    {
        for (std::uint32_t u = 0; u < w; ++u)
        {
            same = same and std::abs(identity.x[v * w + u] - float(u)) < 1e-4f and
                   std::abs(identity.y[v * w + u] - float(v)) < 1e-4f;
        }
    }
    check(same, "an ideal camera's map is the identity");
    std::vector<std::uint8_t> rgb(w * h * 3), out(rgb.size());
    for (std::size_t i = 0; i < rgb.size(); ++i)
    {
        rgb[i] = std::uint8_t(i * 7 % 251);
    }
    remap(identity, rectify_layout("rgb8"), rgb.data(), w * 3, out.data());
    check(out == rgb, "rgb8 through the identity map");
    std::vector<std::uint16_t> depth(w * h), depth_out(depth.size());
    for (std::size_t i = 0; i < depth.size(); ++i)
    {
        depth[i] = std::uint16_t(1000 + i * 13);
    }
    remap(identity, rectify_layout("16UC1"), reinterpret_cast<const std::uint8_t*>(depth.data()), w * 2,
          reinterpret_cast<std::uint8_t*>(depth_out.data()));
    check(depth_out == depth, "16UC1 through the identity map");

    // P's principal point half a pixel to the left: each rectified pixel lies half a pixel right of its own; on a
    // linear image (value 2x + 2y, below 256) the bilinear remap gives the exact mean
    RectifyCamera shifted = ideal(w, h);
    shifted.p[2] -= 0.5;
    const RectifyMap half = rectify_map(shifted);
    std::vector<std::uint8_t> ramp(w * h), ramp_out(ramp.size());
    for (std::uint32_t v = 0; v < h; ++v)
    {
        for (std::uint32_t u = 0; u < w; ++u)
        {
            ramp[v * w + u] = std::uint8_t(2 * u + 2 * v);
        }
    }
    remap(half, rectify_layout("mono8"), ramp.data(), w, ramp_out.data());
    bool mean = true;
    for (std::uint32_t v = 0; v < h; ++v)
    {
        for (std::uint32_t u = 0; u + 1 < w; ++u)
        {
            mean = mean and ramp_out[v * w + u] == std::uint8_t(2 * u + 2 * v + 1);
        }
    }
    check(mean, "a half-pixel shift gives the mean of the two neighbours");
    check(half.index[w - 1] == -1 and ramp_out[w - 1] == 0, "a point beyond the last column is 0");

    // the right camera of a stereo pair: P[3] = -f b does not move the map
    RectifyCamera right = ideal(w, h);
    right.p[3] = -100 * 0.176;
    check(rectify_map(right).x == identity.x, "a stereo P's baseline does not move the map");

    // P all zeros: K is taken (a CameraInfo without rectification)
    RectifyCamera no_p = ideal(w, h);
    no_p.p = {};
    check(rectify_map(no_p).x == identity.x, "P all zeros takes K");

    // refused: R no rotation, P with skew, an unknown model with coefficients, no size
    RectifyCamera bad = ideal(w, h);
    bad.r = {1, 0, 0, 0, 2, 0, 0, 0, 1};
    check(throws(bad), "R that is no rotation is refused");
    bad = ideal(w, h);
    bad.r = {1, 0, 0, 0, 1, 0, 0, 0, -1};
    check(throws(bad), "a reflection is refused");
    bad = ideal(w, h);
    bad.p[1] = 3;
    check(throws(bad), "P with skew is refused");
    bad = ideal(w, h);
    bad.distortion_model = "kannala_brandt";
    bad.d = {0.1, 0, 0, 0};
    check(throws(bad), "an unknown model with coefficients is refused");
    bad = ideal(0, h);
    check(throws(bad), "no image size is refused");
    check(not rectify_layout("32FC1").supported() and rectify_layout("bgra8").channels == 4 and
              rectify_layout("mono16").bytes == 2,
          "the layouts of the encodings");

    if (failures == 0)
    {
        std::printf("test_rectify: all passed\n");
        return 0;
    }
    std::printf("test_rectify: %d failed\n", failures);
    return 1;
}
