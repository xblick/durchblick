// The pinhole camera with ROS's distortion models (sensor_msgs/CameraInfo: plumb_bob, rational_polynomial), for the
// point cloud node: on the CPU, the ray of every pixel of the depth camera (once per calibration); on the GPU, the
// projection of a point into the colour camera. Pixel centres are at integer coordinates, as in ROS and OpenCV.
#pragma once

#include <cmath>
#include <vector>

#ifdef __CUDACC__
#define DURCHBLICK_HD __host__ __device__
#else
#define DURCHBLICK_HD
#endif

namespace durchblick
{
    struct CameraInfo;
}

namespace durchblick::points
{
    // K and D of a CameraInfo (skew and the rectification R, P are not used: points are in the camera's own frame).
    struct CameraModel
    {
        float fx = 0, fy = 0, cx = 0, cy = 0;
        float k1 = 0, k2 = 0, p1 = 0, p2 = 0, k3 = 0, k4 = 0, k5 = 0, k6 = 0;  // k4..k6: rational_polynomial only
        int width = 0, height = 0;

        // Normalized image coordinates (x/z, y/z) of a point to distorted ones: ROS's / OpenCV's model (float on the
        // GPU, double for the rays).
        template <typename T>
        DURCHBLICK_HD void distort(T x, T y, T& xd, T& yd) const
        {
            const T r2 = x * x + y * y;
            const T radial = (T(1) + r2 * (T(k1) + r2 * (T(k2) + r2 * T(k3)))) /
                             (T(1) + r2 * (T(k4) + r2 * (T(k5) + r2 * T(k6))));
            xd = x * radial + T(2) * T(p1) * x * y + T(p2) * (r2 + T(2) * x * x);
            yd = y * radial + T(p1) * (r2 + T(2) * y * y) + T(2) * T(p2) * x * y;
        }

        // The pixel of a point in this camera's frame; false if it is behind the camera.
        DURCHBLICK_HD bool project(float x, float y, float z, float& u, float& v) const
        {
            if (not(z > 0.0f))
            {
                return false;
            }
            float xd, yd;
            distort(x / z, y / z, xd, yd);
            u = fx * xd + cx;
            v = fy * yd + cy;
            return true;
        }
    };

    // The model of a CameraInfo; throws std::invalid_argument for a distortion model it does not know.
    CameraModel camera_model(const CameraInfo& info);

    // The ray of every pixel (row-major): x/z and y/z of the points it sees, interleaved; NaN where the distortion
    // cannot be inverted (the iteration does not reach 1e-3 px, e.g. outside the lens's calibrated field of view).
    std::vector<float> rays(const CameraModel& model);
}  // namespace durchblick::points
