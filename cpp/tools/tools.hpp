// Shared by the command-line tools durchblick-cpp-camera and durchblick-cpp-probe: a tiny argument parser and the
// test patterns (frames, clouds) that tests/test_cpp_interop.py computes the same way in Python.
#pragma once

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace durchblick::tools
{
    // Pixel (x, y) of frame `seq`: rgb8 channel c = (x + 2y + 3c + seq) mod 256; 16UC1 = (7x + 13y + 5 seq) mod 65536.
    inline std::vector<std::uint8_t> pattern(const std::string& encoding, std::uint32_t width, std::uint32_t height,
                                             std::uint64_t seq)
    {
        if (encoding == "rgb8")
        {
            std::vector<std::uint8_t> out(static_cast<std::size_t>(width) * height * 3);
            for (std::uint32_t y = 0; y < height; ++y)
                for (std::uint32_t x = 0; x < width; ++x)
                    for (std::uint32_t c = 0; c < 3; ++c)
                        out[(static_cast<std::size_t>(y) * width + x) * 3 + c] =
                            static_cast<std::uint8_t>((x + 2 * y + 3 * c + seq) & 0xFF);
            return out;
        }
        if (encoding == "16UC1")
        {
            std::vector<std::uint8_t> out(static_cast<std::size_t>(width) * height * 2);
            auto* d = reinterpret_cast<std::uint16_t*>(out.data());
            for (std::uint32_t y = 0; y < height; ++y)
                for (std::uint32_t x = 0; x < width; ++x)
                    d[static_cast<std::size_t>(y) * width + x] =
                        static_cast<std::uint16_t>((7 * x + 13 * y + 5 * seq) & 0xFFFF);
            return out;
        }
        throw std::invalid_argument("the test pattern is rgb8 or 16UC1, not " + encoding);
    }

    // A camera's rays for the compressed transport of clouds: (x/z, y/z, 1) of each pixel of a pinhole camera.
    inline std::vector<float> ray_cloud_rays(std::uint32_t width, std::uint32_t height)
    {
        std::vector<float> rays(static_cast<std::size_t>(width) * height * 3);
        for (std::uint32_t y = 0; y < height; ++y)
        {
            for (std::uint32_t x = 0; x < width; ++x)
            {
                float* d = rays.data() + 3 * (static_cast<std::size_t>(y) * width + x);
                d[0] = (static_cast<float>(x) - static_cast<float>(width) / 2) / 60.0F;
                d[1] = (static_cast<float>(y) - static_cast<float>(height) / 2) / 60.0F;
                d[2] = 1.0F;
            }
        }
        return rays;
    }

    // Cloud `seq` on ray_cloud_rays (layout xyzrgb): the depth 800 + 3x + 5y + seq mod 100 millimetres, no point
    // where x + 2y + seq is a multiple of 11; the colour r = x + y, g = y, b = x + seq (mod 256).
    inline std::vector<std::uint8_t> ray_cloud_pattern(std::uint32_t width, std::uint32_t height, std::uint64_t seq)
    {
        const std::vector<float> rays = ray_cloud_rays(width, height);
        std::vector<std::uint8_t> out(static_cast<std::size_t>(width) * height * 16);
        for (std::uint32_t y = 0; y < height; ++y)
        {
            for (std::uint32_t x = 0; x < width; ++x)
            {
                const std::size_t i = static_cast<std::size_t>(y) * width + x;
                const bool hole = (x + 2 * y + seq) % 11 == 0;
                const float z = hole ? std::numeric_limits<float>::quiet_NaN()
                                     : static_cast<float>(800 + 3 * x + 5 * y + seq % 100) * 0.001F;
                const float point[3] = {rays[3 * i] * z, rays[3 * i + 1] * z, rays[3 * i + 2] * z};
                std::memcpy(out.data() + 16 * i, point, 12);
                const std::uint8_t packed[4] = {static_cast<std::uint8_t>(x + seq), static_cast<std::uint8_t>(y),
                                                static_cast<std::uint8_t>(x + y), 0};
                std::memcpy(out.data() + 16 * i + 12, packed, 4);
            }
        }
        return out;
    }

    // Point i (row-major) of cloud `seq`, 16 bytes per point (layout xyzrgb): byte k = (7i + 3k + seq) mod 256, for
    // the transport tests (the values need not be meaningful points).
    inline std::vector<std::uint8_t> cloud_pattern(std::uint32_t width, std::uint32_t height, std::uint64_t seq)
    {
        std::vector<std::uint8_t> out(static_cast<std::size_t>(width) * height * 16);
        for (std::size_t i = 0; i < static_cast<std::size_t>(width) * height; ++i)
            for (std::size_t k = 0; k < 16; ++k)
                out[16 * i + k] = static_cast<std::uint8_t>((7 * i + 3 * k + seq) & 0xFF);
        return out;
    }

    // Sum of all bytes mod 2^32 (rows without padding): what the tests compare across languages.
    inline std::uint32_t checksum(const std::uint8_t* data, std::uint32_t height, std::uint32_t row, std::uint32_t step)
    {
        std::uint64_t sum = 0;
        for (std::uint32_t y = 0; y < height; ++y)
            for (std::uint32_t i = 0; i < row; ++i)
                sum += data[static_cast<std::size_t>(y) * step + i];
        return static_cast<std::uint32_t>(sum & 0xFFFFFFFFu);
    }

    class Args
    {
    public:
        Args(int argc, char** argv) : args_(argv + 1, argv + argc) {}

        // Values of --name: all words up to the next option.
        std::vector<std::string> list(const std::string& name) const
        {
            std::vector<std::string> out;
            for (std::size_t i = 0; i < args_.size(); ++i)
            {
                if (args_[i] == "--" + name)
                {
                    for (std::size_t j = i + 1; j < args_.size() and args_[j].rfind("--", 0) != 0; ++j)
                    {
                        out.push_back(args_[j]);
                    }
                }
            }
            return out;
        }
        std::string get(const std::string& name, const std::string& fallback) const
        {
            auto v = list(name);
            return v.empty() ? fallback : v.front();
        }
        bool flag(const std::string& name) const
        {
            for (const auto& a : args_)
            {
                if (a == "--" + name)
                {
                    return true;
                }
            }
            return false;
        }

    private:
        std::vector<std::string> args_;
    };
}  // namespace durchblick::tools
