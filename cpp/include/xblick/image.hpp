// Image encodings (ROS sensor_msgs/image_encodings.hpp) and sensor_msgs/CompressedImage as ROS image_transport's
// compressed plugin writes it, as Python's encodings.py and codecs.py do: format "<original>; <jpeg|png> compressed
// <target>", JPEG for 8-bit colour and grey, PNG for 8-bit colour (with alpha) and 8- or 16-bit grey, lossless.
// Decoding gives back the original encoding where the file keeps it (colour order, 16-bit grey).
#pragma once

#include <cstdint>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include "xblick/msg.hpp"

namespace xblick
{
    class EncodingError : public std::invalid_argument
    {
    public:
        using std::invalid_argument::invalid_argument;
    };

    struct EncodingInfo
    {
        int channels;
        int bytes_per_channel;
        char kind;  // 'u' unsigned, 's' signed, 'f' float
        int pixel_bytes() const { return channels * bytes_per_channel; }
    };

    // Channels and channel type of a ROS encoding (rgb8, mono16, 16UC1, 32FC1, ...); EncodingError if unknown.
    EncodingInfo encoding_info(const std::string& encoding);

    // Pixels in host memory, row after row (rows may be padded: step >= width * pixel bytes).
    struct HostImage
    {
        const void* data = nullptr;
        std::uint32_t height = 0;
        std::uint32_t width = 0;
        std::uint32_t step = 0;  // 0: rows without padding
    };

    // A decoded image that owns its pixels (rows without padding).
    struct DecodedImage
    {
        std::vector<std::uint8_t> pixels;
        std::uint32_t height = 0;
        std::uint32_t width = 0;
        std::uint32_t step = 0;
        std::string encoding;
    };

    inline constexpr const char* kJpeg = "jpeg";
    inline constexpr const char* kPng = "png";
    // 16-bit grey and depth only, lossless, much faster than PNG: ROS's compressedDepth "rvl" (rvl.hpp); the format
    // is "<encoding>; compressedDepth rvl"
    inline constexpr const char* kRvl = "rvl";

    CompressedImage compress(const HostImage& image, const std::string& encoding, const std::string& codec,
                             int jpeg_quality, const Header& header);
    // A frame that is JPEG already (an MJPEG camera) as the message compress() would write for it: the bytes as they
    // are, the format of a JPEG of `encoding` (8-bit colour or grey). Checks the JPEG's start marker only.
    CompressedImage jpeg_message(std::span<const std::uint8_t> jpeg, const std::string& encoding,
                                 const Header& header);
    DecodedImage decompress(const CompressedImage& message);
}  // namespace xblick
