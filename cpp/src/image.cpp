#include "durchblick/image.hpp"

#include <csetjmp>
#include <cstdio>
#include <cstring>
#include <map>
#include <regex>

#include <jpeglib.h>
#include <png.h>

#include "durchblick/rvl.hpp"

namespace durchblick
{
    // ------------------------------------------------------------------------------------------- encodings

    EncodingInfo encoding_info(const std::string& encoding)
    {
        static const std::map<std::string, EncodingInfo> named = {
            {"rgb8", {3, 1, 'u'}},         {"bgr8", {3, 1, 'u'}},         {"rgba8", {4, 1, 'u'}},
            {"bgra8", {4, 1, 'u'}},        {"rgb16", {3, 2, 'u'}},        {"bgr16", {3, 2, 'u'}},
            {"rgba16", {4, 2, 'u'}},       {"bgra16", {4, 2, 'u'}},       {"mono8", {1, 1, 'u'}},
            {"mono16", {1, 2, 'u'}},       {"bayer_rggb8", {1, 1, 'u'}},  {"bayer_bggr8", {1, 1, 'u'}},
            {"bayer_gbrg8", {1, 1, 'u'}},  {"bayer_grbg8", {1, 1, 'u'}},  {"bayer_rggb16", {1, 2, 'u'}},
            {"bayer_bggr16", {1, 2, 'u'}}, {"bayer_gbrg16", {1, 2, 'u'}}, {"bayer_grbg16", {1, 2, 'u'}},
        };
        if (auto it = named.find(encoding); it != named.end())
        {
            return it->second;
        }
        static const std::regex generic("^(8|16|32|64)(U|S|F)C([0-9]+)$");
        std::smatch m;
        if (std::regex_match(encoding, m, generic))
        {
            const int bits = std::stoi(m[1]);
            const char kind = m[2].str()[0] == 'U' ? 'u' : (m[2].str()[0] == 'S' ? 's' : 'f');
            const bool known = (kind == 'u' and (bits == 8 or bits == 16)) or (kind == 's' and bits <= 32) or
                               (kind == 'f' and bits >= 16);
            if (known)
            {
                return {std::stoi(m[3]), bits / 8, kind};
            }
        }
        throw EncodingError("unknown image encoding '" + encoding + "'");
    }

    namespace
    {
        bool grey8(const std::string& e)
        {
            return e == "mono8" or e == "8UC1";
        }
        bool grey16(const std::string& e)
        {
            return e == "mono16" or e == "16UC1";
        }

        std::uint32_t step_of(const HostImage& image, int pixel_bytes)
        {
            const std::uint32_t row = image.width * static_cast<std::uint32_t>(pixel_bytes);
            const std::uint32_t step = image.step == 0 ? row : image.step;
            if (step < row)
            {
                throw EncodingError("step " + std::to_string(step) + " is smaller than a row (" + std::to_string(row) +
                                    " bytes)");
            }
            return step;
        }

        // --------------------------------------------------------------------------------------- JPEG

        struct JpegError
        {
            jpeg_error_mgr mgr;
            std::jmp_buf jump;
            char message[JMSG_LENGTH_MAX];
        };

        void jpeg_fail(j_common_ptr info)
        {
            auto* e = reinterpret_cast<JpegError*>(info->err);
            (*info->err->format_message)(info, e->message);
            std::longjmp(e->jump, 1);
        }

        std::vector<std::uint8_t> encode_jpeg(const std::uint8_t* pixels, std::uint32_t height, std::uint32_t width,
                                              std::uint32_t step, J_COLOR_SPACE space, int components, int quality)
        {
            jpeg_compress_struct c{};
            JpegError error{};
            c.err = jpeg_std_error(&error.mgr);
            error.mgr.error_exit = jpeg_fail;
            unsigned char* out = nullptr;
            unsigned long size = 0;
            if (setjmp(error.jump))
            {
                jpeg_destroy_compress(&c);
                std::free(out);
                throw EncodingError(std::string("JPEG encoding failed: ") + error.message);
            }
            jpeg_create_compress(&c);
            jpeg_mem_dest(&c, &out, &size);
            c.image_width = width;
            c.image_height = height;
            c.input_components = components;
            c.in_color_space = space;
            jpeg_set_defaults(&c);
            jpeg_set_quality(&c, quality, TRUE);
            jpeg_start_compress(&c, TRUE);
            while (c.next_scanline < c.image_height)
            {
                auto* row = const_cast<JSAMPROW>(pixels + static_cast<std::size_t>(c.next_scanline) * step);
                jpeg_write_scanlines(&c, &row, 1);
            }
            jpeg_finish_compress(&c);
            std::vector<std::uint8_t> data(out, out + size);
            jpeg_destroy_compress(&c);
            std::free(out);
            return data;
        }

        DecodedImage decode_jpeg(const std::vector<std::uint8_t>& file, bool bgr)
        {
            jpeg_decompress_struct d{};
            JpegError error{};
            d.err = jpeg_std_error(&error.mgr);
            error.mgr.error_exit = jpeg_fail;
            DecodedImage out;
            if (setjmp(error.jump))
            {
                jpeg_destroy_decompress(&d);
                throw EncodingError(std::string("JPEG decoding failed: ") + error.message);
            }
            jpeg_create_decompress(&d);
            jpeg_mem_src(&d, file.data(), file.size());
            jpeg_read_header(&d, TRUE);
            const bool grey = d.num_components == 1;
            d.out_color_space = grey ? JCS_GRAYSCALE : (bgr ? JCS_EXT_BGR : JCS_RGB);
            jpeg_start_decompress(&d);
            out.width = d.output_width;
            out.height = d.output_height;
            out.step = d.output_width * d.output_components;
            out.pixels.resize(static_cast<std::size_t>(out.step) * out.height);
            while (d.output_scanline < d.output_height)
            {
                JSAMPROW row = out.pixels.data() + static_cast<std::size_t>(d.output_scanline) * out.step;
                jpeg_read_scanlines(&d, &row, 1);
            }
            jpeg_finish_decompress(&d);
            jpeg_destroy_decompress(&d);
            out.encoding = grey ? "mono8" : (bgr ? "bgr8" : "rgb8");
            return out;
        }

        // ---------------------------------------------------------------------------------------- PNG

        struct PngError
        {
            char message[256] = "";
        };

        void png_fail(png_structp png, png_const_charp text)
        {
            auto* e = static_cast<PngError*>(png_get_error_ptr(png));
            std::snprintf(e->message, sizeof(e->message), "%s", text);
            png_longjmp(png, 1);
        }

        void png_warn(png_structp, png_const_charp)
        {
        }

        void png_write_to(png_structp png, png_bytep data, png_size_t n)
        {
            auto* out = static_cast<std::vector<std::uint8_t>*>(png_get_io_ptr(png));
            out->insert(out->end(), data, data + n);
        }

        void png_flush(png_structp)
        {
        }

        struct PngSource
        {
            const std::uint8_t* data;
            std::size_t size;
            std::size_t pos;
        };

        void png_read_from(png_structp png, png_bytep out, png_size_t n)
        {
            auto* s = static_cast<PngSource*>(png_get_io_ptr(png));
            if (s->pos + n > s->size)
            {
                png_error(png, "truncated PNG");
            }
            std::memcpy(out, s->data + s->pos, n);
            s->pos += n;
        }

        // color_type: PNG_COLOR_TYPE_RGB/RGBA/GRAY; bgr: swap colour order; depth 8 or 16 (16: little-endian input)
        std::vector<std::uint8_t> encode_png(const std::uint8_t* pixels, std::uint32_t height, std::uint32_t width,
                                             std::uint32_t step, int color_type, int depth, bool bgr)
        {
            std::vector<std::uint8_t> out;
            PngError error;
            png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, &error, png_fail, png_warn);
            png_infop info = png ? png_create_info_struct(png) : nullptr;
            if (png == nullptr or info == nullptr)
            {
                png_destroy_write_struct(&png, &info);
                throw EncodingError("PNG: out of memory");
            }
            if (setjmp(png_jmpbuf(png)))
            {
                png_destroy_write_struct(&png, &info);
                throw EncodingError(std::string("PNG encoding failed: ") + error.message);
            }
            png_set_write_fn(png, &out, png_write_to, png_flush);
            png_set_compression_level(png, 1);  // fast; PNG is lossless at every level
            png_set_IHDR(png, info, width, height, depth, color_type, PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT,
                         PNG_FILTER_TYPE_DEFAULT);
            png_write_info(png, info);
            if (bgr)
            {
                png_set_bgr(png);
            }
            if (depth == 16)
            {
                png_set_swap(png);  // PNG stores 16-bit samples big endian
            }
            for (std::uint32_t y = 0; y < height; ++y)
            {
                png_write_row(png, const_cast<png_bytep>(pixels + static_cast<std::size_t>(y) * step));
            }
            png_write_end(png, nullptr);
            png_destroy_write_struct(&png, &info);
            return out;
        }

        // original: the encoding the publisher had (decides colour order and the grey encoding's name)
        DecodedImage decode_png(const std::vector<std::uint8_t>& file, const std::string& original)
        {
            PngError error;
            png_structp png = png_create_read_struct(PNG_LIBPNG_VER_STRING, &error, png_fail, png_warn);
            png_infop info = png ? png_create_info_struct(png) : nullptr;
            if (png == nullptr or info == nullptr)
            {
                png_destroy_read_struct(&png, &info, nullptr);
                throw EncodingError("PNG: out of memory");
            }
            DecodedImage out;
            PngSource source{file.data(), file.size(), 0};
            if (setjmp(png_jmpbuf(png)))
            {
                png_destroy_read_struct(&png, &info, nullptr);
                throw EncodingError(std::string("PNG decoding failed: ") + error.message);
            }
            png_set_read_fn(png, &source, png_read_from);
            png_read_info(png, info);
            const int color = png_get_color_type(png, info);
            const int depth = png_get_bit_depth(png, info);
            if (color == PNG_COLOR_TYPE_PALETTE)
            {
                png_set_palette_to_rgb(png);
            }
            if (color == PNG_COLOR_TYPE_GRAY and depth < 8)
            {
                png_set_expand_gray_1_2_4_to_8(png);
            }
            if (png_get_valid(png, info, PNG_INFO_tRNS))
            {
                png_set_tRNS_to_alpha(png);
            }
            const bool grey = color == PNG_COLOR_TYPE_GRAY;
            if (color == PNG_COLOR_TYPE_GRAY_ALPHA)  // as Pillow's convert("RGB")
            {
                png_set_strip_alpha(png);
                png_set_gray_to_rgb(png);
            }
            if (depth == 16 and not grey)
            {
                png_set_strip_16(png);
            }
            if (depth == 16 and grey)
            {
                png_set_swap(png);
            }
            png_read_update_info(png, info);
            const int channels = png_get_channels(png, info);
            const int out_depth = png_get_bit_depth(png, info);
            const bool bgr_out = channels >= 3 and (original == "bgr8" or original == "bgra8");
            if (bgr_out)
            {
                png_set_bgr(png);
                png_read_update_info(png, info);
            }
            out.width = png_get_image_width(png, info);
            out.height = png_get_image_height(png, info);
            out.step = static_cast<std::uint32_t>(png_get_rowbytes(png, info));
            out.pixels.resize(static_cast<std::size_t>(out.step) * out.height);
            for (std::uint32_t y = 0; y < out.height; ++y)
            {
                png_read_row(png, out.pixels.data() + static_cast<std::size_t>(y) * out.step, nullptr);
            }
            png_read_end(png, nullptr);
            png_destroy_read_struct(&png, &info, nullptr);
            if (channels == 1)
            {
                out.encoding = out_depth == 16 ? (grey16(original) ? original : "mono16")
                                               : (grey8(original) ? original : "mono8");
            }
            else if (channels == 4)
            {
                out.encoding = original == "bgra8" ? "bgra8" : "rgba8";
            }
            else
            {
                out.encoding = bgr_out ? "bgr8" : "rgb8";
            }
            return out;
        }

        std::string lower(std::string s)
        {
            for (char& c : s)
            {
                c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            }
            return s;
        }

        std::string trim(const std::string& s)
        {
            const auto a = s.find_first_not_of(" \t");
            const auto b = s.find_last_not_of(" \t");
            return a == std::string::npos ? "" : s.substr(a, b - a + 1);
        }
    }  // namespace

    CompressedImage compress(const HostImage& image, const std::string& encoding, const std::string& codec,
                             int jpeg_quality, const Header& header)
    {
        const EncodingInfo enc = encoding_info(encoding);
        const std::uint32_t step = step_of(image, enc.pixel_bytes());
        const auto* pixels = static_cast<const std::uint8_t*>(image.data);
        CompressedImage out;
        out.header = header;
        std::string target;
        if (codec == kJpeg)
        {
            J_COLOR_SPACE space;
            int components;
            if (encoding == "rgb8" or encoding == "rgba8")
            {
                space = encoding == "rgb8" ? JCS_RGB : JCS_EXT_RGBA;
                components = enc.channels;
                target = "bgr8";
            }
            else if (encoding == "bgr8" or encoding == "bgra8")
            {
                space = encoding == "bgr8" ? JCS_EXT_BGR : JCS_EXT_BGRA;
                components = enc.channels;
                target = "bgr8";
            }
            else if (grey8(encoding))
            {
                space = JCS_GRAYSCALE;
                components = 1;
                target = "mono8";
            }
            else
            {
                throw EncodingError("JPEG takes 8-bit colour or grey images, not " + encoding + " (depth: use png)");
            }
            out.data = encode_jpeg(pixels, image.height, image.width, step, space, components, jpeg_quality);
        }
        else if (codec == kPng)
        {
            if (encoding == "rgb8" or encoding == "bgr8")
            {
                out.data = encode_png(pixels, image.height, image.width, step, PNG_COLOR_TYPE_RGB, 8,
                                      encoding == "bgr8");
                target = "bgr8";
            }
            else if (encoding == "rgba8" or encoding == "bgra8")
            {
                out.data = encode_png(pixels, image.height, image.width, step, PNG_COLOR_TYPE_RGBA, 8,
                                      encoding == "bgra8");
                target = "bgra8";
            }
            else if (grey8(encoding))
            {
                out.data = encode_png(pixels, image.height, image.width, step, PNG_COLOR_TYPE_GRAY, 8, false);
                target = "mono8";
            }
            else if (grey16(encoding))
            {
                out.data = encode_png(pixels, image.height, image.width, step, PNG_COLOR_TYPE_GRAY, 16, false);
                target = encoding;
            }
            else
            {
                throw EncodingError("PNG takes 8-bit colour, 8- or 16-bit grey images, not " + encoding);
            }
        }
        else if (codec == kRvl)
        {
            if (not grey16(encoding))
            {
                throw EncodingError("RVL takes 16-bit grey and depth images, not " + encoding);
            }
            if (step == image.width * 2)
            {
                out.data = rvl::encode(pixels, image.height, image.width);
            }
            else  // rows with padding
            {
                std::vector<std::uint8_t> packed(static_cast<std::size_t>(image.height) * image.width * 2);
                for (std::uint32_t y = 0; y < image.height; ++y)
                {
                    std::memcpy(packed.data() + static_cast<std::size_t>(y) * image.width * 2,
                                pixels + static_cast<std::size_t>(y) * step, static_cast<std::size_t>(image.width) * 2);
                }
                out.data = rvl::encode(packed.data(), image.height, image.width);
            }
            out.format = encoding + "; compressedDepth rvl";
            return out;
        }
        else
        {
            throw EncodingError("unknown codec '" + codec + "' (jpeg, png, rvl)");
        }
        out.format = encoding + "; " + codec + " compressed " + target;
        return out;
    }

    CompressedImage jpeg_message(std::span<const std::uint8_t> jpeg, const std::string& encoding,
                                 const Header& header)
    {
        std::string target;
        if (encoding == "rgb8" or encoding == "rgba8" or encoding == "bgr8" or encoding == "bgra8")
        {
            target = "bgr8";
        }
        else if (grey8(encoding))
        {
            target = "mono8";
        }
        else
        {
            throw EncodingError("JPEG takes 8-bit colour or grey images, not " + encoding);
        }
        if (jpeg.size() < 4 or jpeg[0] != 0xFF or jpeg[1] != 0xD8)
        {
            throw EncodingError("not a JPEG file (no start-of-image marker)");
        }
        CompressedImage out;
        out.header = header;
        out.data = std::vector<std::uint8_t>(jpeg.begin(), jpeg.end());
        out.format = encoding + "; " + std::string(kJpeg) + " compressed " + target;
        return out;
    }

    DecodedImage decompress(const CompressedImage& message)
    {
        std::string original;
        std::string codec;
        const auto semicolon = message.format.find(';');
        if (semicolon != std::string::npos)
        {
            original = trim(message.format.substr(0, semicolon));
            const std::string rest = trim(message.format.substr(semicolon + 1));
            codec = lower(rest.substr(0, rest.find(' ')));
        }
        else
        {
            codec = lower(trim(message.format));
            if (codec == "jpg")
            {
                codec = kJpeg;
            }
        }
        if (codec == kJpeg)
        {
            DecodedImage out = decode_jpeg(message.data, original == "bgr8" or original == "bgra8");
            if (out.encoding == "mono8" and grey8(original))
            {
                out.encoding = original;
            }
            return out;
        }
        if (codec == kPng)
        {
            return decode_png(message.data, original);
        }
        if (codec == "compresseddepth")  // ROS's compressedDepth plugin: "<encoding>; compressedDepth rvl"
        {
            if (lower(trim(message.format.substr(semicolon + 1))) != "compresseddepth rvl" or not grey16(original))
            {
                throw EncodingError("compressed format '" + message.format
                                    + "': of ROS's compressedDepth only rvl of a 16-bit image is read");
            }
            try
            {
                const auto [height, width] = rvl::shape(message.data);
                DecodedImage out;
                out.height = height;
                out.width = width;
                out.step = width * 2;
                out.encoding = original;
                out.pixels.resize(static_cast<std::size_t>(height) * width * 2);
                rvl::decode(message.data, out.pixels.data());
                return out;
            }
            catch (const rvl::Error& error)
            {
                throw EncodingError(error.what());
            }
        }
        throw EncodingError("compressed format '" + message.format + "' is not one of image_transport's");
    }
}  // namespace durchblick
