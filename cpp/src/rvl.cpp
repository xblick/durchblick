#include "durchblick/rvl.hpp"

#include <bit>
#include <cstring>
#include <string>

namespace durchblick::rvl
{
    static_assert(std::endian::native == std::endian::little, "RVL's words are written as the host has them");

    namespace
    {
        constexpr std::uint64_t kMaxPixels = std::uint64_t{1} << 28;  // 16384 x 16384; no camera here comes near

        struct Writer
        {
            std::uint8_t* out;
            std::uint32_t word = 0;
            int nibbles = 0;

            void put(std::uint32_t value)
            {
                do
                {
                    std::uint32_t nibble = value & 0x7;
                    value >>= 3;
                    if (value)
                    {
                        nibble |= 0x8;  // more groups follow
                    }
                    word = (word << 4) | nibble;
                    if (++nibbles == 8)
                    {
                        std::memcpy(out, &word, 4);
                        out += 4;
                        nibbles = 0;
                        word = 0;
                    }
                } while (value);
            }

            void finish()
            {
                if (nibbles)
                {
                    word <<= 4 * (8 - nibbles);
                    std::memcpy(out, &word, 4);
                    out += 4;
                }
            }
        };

        struct Reader
        {
            const std::uint8_t* in;
            const std::uint8_t* end;
            std::uint32_t word = 0;
            int nibbles = 0;

            std::uint32_t get()
            {
                std::uint32_t value = 0;
                std::uint32_t nibble;
                int shift = 0;
                do
                {
                    if (!nibbles)
                    {
                        if (end - in < 4)
                        {
                            throw Error("RVL: the bytes end before the image does");
                        }
                        std::memcpy(&word, in, 4);
                        in += 4;
                        nibbles = 8;
                    }
                    nibble = word >> 28;
                    word <<= 4;
                    --nibbles;
                    if (shift > 27)  // kMaxPixels has 29 bits, a difference 17
                    {
                        throw Error("RVL: a number of more than 30 bits");
                    }
                    value |= (nibble & 0x7) << shift;
                    shift += 3;
                } while (nibble & 0x8);
                return value;
            }
        };

        inline std::uint16_t load(const std::uint8_t* p)
        {
            std::uint16_t v;
            std::memcpy(&v, p, 2);
            return v;
        }
    }  // namespace

    std::vector<std::uint8_t> encode(const void* pixels, std::uint32_t height, std::uint32_t width)
    {
        const std::uint64_t count = std::uint64_t{height} * width;
        if (count == 0 or count > kMaxPixels)
        {
            throw Error("RVL: no image of " + std::to_string(width) + " x " + std::to_string(height));
        }
        // the most a pixel takes is 6 groups (a difference of 17 bits) and a run's two lengths: under 3 bytes
        std::vector<std::uint8_t> out(kHeaderBytes + 3 * count + 16);
        const std::uint32_t zero = 0;
        std::memcpy(out.data(), &zero, 4);  // ConfigHeader.format: INV_DEPTH
        std::memcpy(out.data() + 4, &zero, 4);
        std::memcpy(out.data() + 8, &zero, 4);
        std::memcpy(out.data() + 12, &width, 4);
        std::memcpy(out.data() + 16, &height, 4);
        Writer w{out.data() + kHeaderBytes};
        const auto* in = static_cast<const std::uint8_t*>(pixels);
        const std::uint8_t* const end = in + 2 * count;
        std::int32_t previous = 0;
        while (in != end)
        {
            std::uint32_t zeros = 0;
            for (; in != end and load(in) == 0; in += 2, ++zeros)
            {
            }
            w.put(zeros);
            std::uint32_t nonzeros = 0;
            for (const std::uint8_t* p = in; p != end and load(p) != 0; p += 2, ++nonzeros)
            {
            }
            w.put(nonzeros);
            for (std::uint32_t i = 0; i < nonzeros; ++i, in += 2)
            {
                const std::int32_t current = load(in);
                const std::int32_t delta = current - previous;
                w.put((static_cast<std::uint32_t>(delta) << 1) ^ static_cast<std::uint32_t>(delta >> 31));
                previous = current;
            }
        }
        w.finish();
        out.resize(static_cast<std::size_t>(w.out - out.data()));
        return out;
    }

    std::pair<std::uint32_t, std::uint32_t> shape(std::span<const std::uint8_t> data)
    {
        if (data.size() < kHeaderBytes)
        {
            throw Error("RVL: " + std::to_string(data.size()) + " bytes are fewer than the header's "
                        + std::to_string(kHeaderBytes));
        }
        std::uint32_t width;
        std::uint32_t height;
        std::memcpy(&width, data.data() + 12, 4);
        std::memcpy(&height, data.data() + 16, 4);
        const std::uint64_t count = std::uint64_t{height} * width;
        // a run of zeros of any length takes a few groups, so the bytes do not bound the size; kMaxPixels does
        if (count == 0 or count > kMaxPixels)
        {
            throw Error("RVL: the header states " + std::to_string(width) + " x " + std::to_string(height)
                        + ", no image");
        }
        return {height, width};
    }

    void decode(std::span<const std::uint8_t> data, void* pixels)
    {
        const auto [height, width] = shape(data);
        std::uint64_t left = std::uint64_t{height} * width;
        Reader r{data.data() + kHeaderBytes, data.data() + data.size()};
        auto* out = static_cast<std::uint8_t*>(pixels);
        std::uint16_t previous = 0;
        while (left)
        {
            const std::uint32_t zeros = r.get();
            if (zeros > left)
            {
                throw Error("RVL: a run of zeros passes the image's end");
            }
            std::memset(out, 0, 2 * std::size_t{zeros});
            out += 2 * std::size_t{zeros};
            left -= zeros;
            const std::uint32_t nonzeros = r.get();
            if (nonzeros > left or (zeros == 0 and nonzeros == 0))
            {
                throw Error("RVL: a run passes the image's end or is empty");
            }
            left -= nonzeros;
            for (std::uint32_t i = 0; i < nonzeros; ++i, out += 2)
            {
                const std::uint32_t positive = r.get();
                const auto delta = static_cast<std::int32_t>(positive >> 1) ^ -static_cast<std::int32_t>(positive & 1);
                previous = static_cast<std::uint16_t>(previous + delta);
                std::memcpy(out, &previous, 2);
            }
        }
    }
}  // namespace durchblick::rvl
