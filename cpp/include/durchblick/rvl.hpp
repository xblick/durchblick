#pragma once
// RVL, the lossless codec for 16-bit depth images (Wilson, "Fast Lossless Depth Image Compression", 2017): runs of
// zeros by their length, the other pixels as the difference to the pixel before in 3-bit groups. On Kinect depth it
// encodes about nine times as fast as PNG at level 1 and is a fifth smaller (docs/concepts/transports.md).
//
// The bytes are those of ROS's compressed_depth_image_transport for a 16UC1 image with the format "rvl" (4.0.0,
// 2024), the `data` of a sensor_msgs/CompressedImage with the format "16UC1; compressedDepth rvl":
//
//   int32   0            ROS's ConfigHeader: the format (INV_DEPTH) ...
//   float32 0, 0         ... and its depth parameters, which a 16-bit image does not use
//   uint32  width, height
//   int32   ...          the RVL words, 8 groups of 4 bits each, the first group in the highest bits
//
// All little endian, as ROS writes them on the PCs it runs on. Unlike ROS's decoder, decode() checks every length:
// the bytes come from the network.

#include <cstddef>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

namespace durchblick::rvl
{
    inline constexpr std::size_t kHeaderBytes = 20;  // ConfigHeader (12), width, height

    // A message whose bytes are not an RVL image of the stated size.
    struct Error : std::runtime_error
    {
        using std::runtime_error::runtime_error;
    };

    // `pixels`: height x width 16-bit pixels, rows without padding (may be unaligned).
    std::vector<std::uint8_t> encode(const void* pixels, std::uint32_t height, std::uint32_t width);

    // (height, width) as the bytes state them. Throws Error for bytes too short or a size no image has.
    std::pair<std::uint32_t, std::uint32_t> shape(std::span<const std::uint8_t> data);

    // Writes shape(data)'s height x width pixels to `pixels` (may be unaligned). Throws Error for bytes that end
    // early or runs that pass the image's end; `pixels` then holds a part of the image.
    void decode(std::span<const std::uint8_t> data, void* pixels);
}  // namespace durchblick::rvl
