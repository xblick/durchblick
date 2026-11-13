#include "kernels.hpp"

#include <stdexcept>
#include <string>

namespace xblick::points
{
    namespace
    {
        __device__ float depth_at(const void* depth, DepthFormat format, int i)
        {
            if (format == DepthFormat::Millimetres16)
            {
                return 0.001f * static_cast<const std::uint16_t*>(depth)[i];
            }
            return static_cast<const float*>(depth)[i];
        }

        // PCL's packed colour: the uint32 0x00RRGGBB, stored as the bits of a float
        __device__ float colour_at(const ColorInput& c, float x, float y, float z)
        {
            const float* r = c.rotation;
            const float xc = r[0] * x + r[1] * y + r[2] * z + c.translation[0];
            const float yc = r[3] * x + r[4] * y + r[5] * z + c.translation[1];
            const float zc = r[6] * x + r[7] * y + r[8] * z + c.translation[2];
            float u, v;
            if (not c.camera.project(xc, yc, zc, u, v))
            {
                return 0.0f;
            }
            const int ui = __float2int_rn(u);
            const int vi = __float2int_rn(v);
            if (ui < 0 or vi < 0 or ui >= c.camera.width or vi >= c.camera.height)
            {
                return 0.0f;
            }
            const bool alpha = c.format == ColorFormat::Rgba8 or c.format == ColorFormat::Bgra8;
            const std::uint8_t* p = c.pixels + (static_cast<std::size_t>(vi) * c.camera.width + ui) * (alpha ? 4 : 3);
            const bool bgr = c.format == ColorFormat::Bgr8 or c.format == ColorFormat::Bgra8;
            const std::uint32_t red = bgr ? p[2] : p[0];
            const std::uint32_t blue = bgr ? p[0] : p[2];
            return __uint_as_float((red << 16) | (std::uint32_t(p[1]) << 8) | blue);
        }

        __global__ void depth_to_points_kernel(const void* depth, DepthFormat format, int width, int height,
                                               const float2* rays, float max_depth, ColorInput color, bool with_color,
                                               float4* points)
        {
            const int u = blockIdx.x * blockDim.x + threadIdx.x;
            const int v = blockIdx.y * blockDim.y + threadIdx.y;
            if (u >= width or v >= height)
            {
                return;
            }
            const int i = v * width + u;
            const float z = depth_at(depth, format, i);
            const float2 ray = rays[i];
            if (not(z > 0.0f) or isinf(z) or isnan(ray.x) or (max_depth > 0.0f and z > max_depth))
            {
                const float nan = __int_as_float(0x7fc00000);
                points[i] = make_float4(nan, nan, nan, 0.0f);
                return;
            }
            const float x = ray.x * z;
            const float y = ray.y * z;
            points[i] = make_float4(x, y, z, with_color ? colour_at(color, x, y, z) : 0.0f);
        }
    }  // namespace

    void depth_to_points(const void* depth, DepthFormat format, int width, int height, const float* rays,
                         float max_depth, const ColorInput* color, void* points, cudaStream_t stream)
    {
        const dim3 block(32, 8);
        const dim3 grid((width + block.x - 1) / block.x, (height + block.y - 1) / block.y);
        depth_to_points_kernel<<<grid, block, 0, stream>>>(depth, format, width, height,
                                                           reinterpret_cast<const float2*>(rays), max_depth,
                                                           color ? *color : ColorInput{}, color != nullptr,
                                                           static_cast<float4*>(points));
        const cudaError_t error = cudaGetLastError();
        if (error != cudaSuccess)
        {
            throw std::runtime_error(std::string("depth_to_points kernel: ") + cudaGetErrorString(error));
        }
    }
}  // namespace xblick::points
