#include "durchblick/point_cloud_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <thread>

#include <cuda_runtime_api.h>

#include "camera_model.hpp"
#include "durchblick/image.hpp"
#include "durchblick/pairing.hpp"
#include "durchblick/points.hpp"
#include "kernels.hpp"

namespace durchblick
{
    namespace
    {
        void check(cudaError_t error, const char* what)
        {
            if (error != cudaSuccess)
            {
                throw std::runtime_error(std::string(what) + ": " + cudaGetErrorString(error));
            }
        }

        // Device memory of the node's own, grown as needed.
        class DeviceMemory
        {
        public:
            DeviceMemory() = default;
            DeviceMemory(const DeviceMemory&) = delete;
            DeviceMemory& operator=(const DeviceMemory&) = delete;
            ~DeviceMemory()
            {
                if (ptr_ != nullptr)
                {
                    cudaFree(ptr_);
                }
            }

            void* ptr() const { return ptr_; }

            void* ensure(std::size_t n)
            {
                if (n > size_)
                {
                    if (ptr_ != nullptr)
                    {
                        check(cudaFree(ptr_), "cudaFree");
                        ptr_ = nullptr;
                    }
                    check(cudaMalloc(&ptr_, n), "cudaMalloc");
                    size_ = n;
                }
                return ptr_;
            }

        private:
            void* ptr_ = nullptr;
            std::size_t size_ = 0;
        };

        std::string parent_of(const std::string& base)
        {
            const std::string name = check_name(base);
            const std::size_t slash = name.rfind('/');
            if (slash == std::string::npos)
            {
                throw std::invalid_argument("depth stream '" + base + "' has no parent (the camera): give the camera");
            }
            return name.substr(0, slash);
        }

        bool same_calibration(const CameraInfo& a, const CameraInfo& b)
        {
            return a.width == b.width and a.height == b.height and a.distortion_model == b.distortion_model and
                   a.d == b.d and a.k == b.k;
        }

        points::DepthFormat depth_format(const std::string& encoding)
        {
            if (encoding == "16UC1" or encoding == "mono16")
            {
                return points::DepthFormat::Millimetres16;
            }
            if (encoding == "32FC1")
            {
                return points::DepthFormat::Metres32;
            }
            throw std::invalid_argument("depth encoding " + encoding +
                                        ": 16UC1, mono16 (millimetres) or 32FC1 (metres)");
        }

        points::ColorFormat color_format(const std::string& encoding)
        {
            if (encoding == "rgb8") return points::ColorFormat::Rgb8;
            if (encoding == "bgr8") return points::ColorFormat::Bgr8;
            if (encoding == "rgba8") return points::ColorFormat::Rgba8;
            if (encoding == "bgra8") return points::ColorFormat::Bgra8;
            throw std::invalid_argument("colour encoding " + encoding + ": rgb8, bgr8, rgba8 or bgra8");
        }

        std::string number(double v)
        {
            char text[32];
            std::snprintf(text, sizeof(text), "%.2f", v);
            return text;
        }
    }  // namespace

    struct PointCloudNode::Impl
    {
        zenoh::Session& session;
        PointCloudNodeOptions o;
        std::string camera, output;
        std::unique_ptr<PointCloudPublisher> publisher;
        std::unique_ptr<ImageSubscriber> depth, color;
        std::unique_ptr<StaticTransformSubscriber> tf;
        std::unique_ptr<DiagnosticsPublisher> diagnostics;
        cudaStream_t stream = nullptr;

        std::optional<CameraInfo> depth_calibration;  // the one the rays are of
        points::CameraModel depth_model;
        DeviceMemory rays, depth_upload, color_upload;
        std::string frame_id;

        std::optional<Frame> pending;  // the newest depth frame, waiting for the colour frame of its time
        std::chrono::steady_clock::time_point pending_since;
        std::optional<Frame> pending_color;  // its colour frame, decoded once found
        std::string waiting_for;   // what the pending frame waits for (camera_info, tf_static), for the report

        PointCloudNodeStats stats, reported_stats;
        std::uint64_t skipped_total = 0, subscriptions_skipped = 0, reported_skipped = 0;  // for max_hz
        std::chrono::steady_clock::time_point reported = std::chrono::steady_clock::now();

        Impl(zenoh::Session& s, PointCloudNodeOptions options) : session(s), o(std::move(options))
        {
        }

        bool subscribed() const { return publisher->subscribed(); }

        void subscribe()
        {
            if (depth)
            {
                return;
            }
            SubscriberOptions options;
            options.transport = Transport::Auto;
            options.local = Transport::Cuda;
            options.remote = o.remote;
            options.camera_info = true;
            // the depth frames drive the clouds: only they are limited; the colour stream is their partner, its
            // samples kept undecoded and only the one of a depth frame's time decoded (no pair lost to the limit)
            options.max_hz = o.max_hz;
            depth = std::make_unique<ImageSubscriber>(session, o.depth, options);
            if (not o.color.empty())
            {
                options.max_hz = 0.0;
                options.keep_recent = 8;
                color = std::make_unique<ImageSubscriber>(session, o.color, options);
            }
        }

        // nobody takes the points: the camera need not send the inputs
        void unsubscribe()
        {
            depth.reset();
            color.reset();
            pending.reset();
            pending_color.reset();
        }

        // The colour frame of the pending depth frame's time, decoded once: the nearest colour sample kept, within
        // max_offset, once no nearer one can still come (partner_of, pairing.hpp); nullptr if there is none (yet).
        const Frame* match()
        {
            if (pending_color)
            {
                return &*pending_color;
            }
            const std::int64_t stamp = pending->stamp_ns();
            const std::int64_t max_offset = std::chrono::duration_cast<std::chrono::nanoseconds>(o.max_offset).count();
            const std::optional<std::int64_t> best = partner_of(color->recent_stamps(), stamp, max_offset);
            if (not best)
            {
                return nullptr;  // none, or a nearer one may still come
            }
            pending_color = color->at(*best, std::chrono::nanoseconds(0));
            if (pending_color)
            {
                ++stats.color_frames;
            }
            return pending_color ? &*pending_color : nullptr;
        }

        // Device pointer of a frame's pixels: the frame's own (cuda transport) or uploaded into `upload`.
        const void* on_device(const Frame& f, std::size_t pixel_bytes, DeviceMemory& upload)
        {
            const std::size_t nbytes = static_cast<std::size_t>(f.width) * f.height * pixel_bytes;
            if (f.device_ptr != 0)
            {
                if (f.device != o.cuda_device)
                {
                    throw std::runtime_error(f.key + ": the frame is on CUDA device " + std::to_string(f.device) +
                                             ", the node computes on " + std::to_string(o.cuda_device));
                }
                return reinterpret_cast<const void*>(f.device_ptr);
            }
            if (f.step != f.width * pixel_bytes)
            {
                throw std::runtime_error(f.key + ": rows with padding are not supported");
            }
            void* ptr = upload.ensure(nbytes);
            check(cudaMemcpyAsync(ptr, f.data(), nbytes, cudaMemcpyHostToDevice, stream), "upload of a frame");
            return ptr;
        }

        void calibrate_depth(const CameraInfo& info, const Frame& f)
        {
            if (info.width != f.width or info.height != f.height)
            {
                throw std::runtime_error(o.depth + ": camera_info is " + std::to_string(info.width) + " x " +
                                         std::to_string(info.height) + ", the frame " + std::to_string(f.width) +
                                         " x " + std::to_string(f.height));
            }
            if (depth_calibration and same_calibration(*depth_calibration, info))
            {
                return;
            }
            depth_model = points::camera_model(info);
            const std::vector<float> table = points::rays(depth_model);
            void* ptr = rays.ensure(table.size() * sizeof(float));
            check(cudaMemcpy(ptr, table.data(), table.size() * sizeof(float), cudaMemcpyHostToDevice), "rays");
            // cloud/compressed: a point is (x/z, y/z, 1) times its z. A pixel whose ray is NaN never has a point; its
            // direction only has to be one.
            CloudRays directions{info.height, info.width, {}};
            directions.directions.resize(table.size() / 2 * 3);
            for (std::size_t i = 0; i < table.size() / 2; ++i)
            {
                const bool none = std::isnan(table[2 * i]) or std::isnan(table[2 * i + 1]);
                directions.directions[3 * i] = none ? 0.0f : table[2 * i];
                directions.directions[3 * i + 1] = none ? 0.0f : table[2 * i + 1];
                directions.directions[3 * i + 2] = 1.0f;
            }
            publisher->set_rays(std::move(directions));
            depth_calibration = info;
            const std::string id = info.header.frame_id.empty() ? f.header.frame_id : info.header.frame_id;
            if (id != frame_id)
            {
                frame_id = id;
                publisher->set_frame_id(id);
            }
        }

        // Makes and publishes the cloud of the pending depth frame (with colour frame c); false if a calibration it
        // needs has not come yet.
        bool publish(const Frame* c)
        {
            const auto start = std::chrono::steady_clock::now();
            const Frame& d = *pending;
            const std::optional<CameraInfo> depth_info = depth->camera_info();
            if (not depth_info)
            {
                waiting_for = "the depth stream's camera_info";
                return false;
            }
            if (depth_info->width != d.width or depth_info->height != d.height)
            {
                // the camera came back with another size: its new camera_info has not come yet, or the frame is old
                waiting_for = o.depth + "'s camera_info for its frames' size " + std::to_string(d.width) + " x " +
                              std::to_string(d.height) + " (the last one is " + std::to_string(depth_info->width) +
                              " x " + std::to_string(depth_info->height) + ")";
                return false;
            }
            std::optional<points::ColorInput> color_input;
            if (c != nullptr)
            {
                const std::optional<CameraInfo> color_info = color->camera_info();
                if (not color_info)
                {
                    waiting_for = "the colour stream's camera_info";
                    return false;
                }
                const std::optional<TFMessage> transforms = tf->latest();
                const std::string depth_frame = depth_info->header.frame_id;
                const std::string color_frame = color_info->header.frame_id;
                std::optional<RigidTransform> pose;
                if (depth_frame == color_frame)
                {
                    pose = RigidTransform{};  // one camera (a simulation's, the Azure Kinect's depth_to_color)
                }
                else if (transforms)
                {
                    // the depth camera in the colour camera's frame: p_color = R p_depth + t
                    pose = lookup(*transforms, color_frame, depth_frame);
                }
                if (not pose)
                {
                    waiting_for = camera + "/tf_static with " + depth_frame + " and " + color_frame;
                    return false;
                }
                points::ColorInput in;
                in.format = color_format(c->encoding);
                in.camera = points::camera_model(*color_info);
                if (static_cast<int>(c->width) != in.camera.width or static_cast<int>(c->height) != in.camera.height)
                {
                    waiting_for = o.color + "'s camera_info for its frames' size " + std::to_string(c->width) +
                                  " x " + std::to_string(c->height) + " (the last one is " +
                                  std::to_string(in.camera.width) + " x " + std::to_string(in.camera.height) + ")";
                    return false;
                }
                const bool alpha = in.format == points::ColorFormat::Rgba8 or in.format == points::ColorFormat::Bgra8;
                in.pixels = static_cast<const std::uint8_t*>(on_device(*c, alpha ? 4 : 3, color_upload));
                for (int i = 0; i < 9; ++i)
                {
                    in.rotation[i] = static_cast<float>(pose->rotation[i]);
                }
                in.translation[0] = static_cast<float>(pose->translation.x);
                in.translation[1] = static_cast<float>(pose->translation.y);
                in.translation[2] = static_cast<float>(pose->translation.z);
                color_input = in;
            }
            calibrate_depth(*depth_info, d);
            const points::DepthFormat format = depth_format(d.encoding);
            const void* depth_ptr =
                on_device(d, format == points::DepthFormat::Millimetres16 ? 2 : 4, depth_upload);
            const std::uint64_t out = publisher->loan(d.height, d.width);
            try
            {
                points::depth_to_points(depth_ptr, format, static_cast<int>(d.width), static_cast<int>(d.height),
                                        static_cast<const float*>(rays.ptr()), o.max_depth,
                                        color_input ? &*color_input : nullptr, reinterpret_cast<void*>(out), stream);
            }
            catch (...)
            {
                publisher->commit(reinterpret_cast<std::uint64_t>(stream), false, d.stamp_ns());
                throw;
            }
            // is_dense false, as ROS's depth_image_proc: a depth image has pixels without depth
            publisher->commit(reinterpret_cast<std::uint64_t>(stream), false, d.stamp_ns());
            ++stats.clouds;
            stats.ms += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
            waiting_for.clear();
            return true;
        }
    };

    PointCloudNode::PointCloudNode(zenoh::Session& session, PointCloudNodeOptions options)
        : impl_(std::make_unique<Impl>(session, std::move(options)))
    {
        Impl& m = *impl_;
        if (m.o.depth.empty())
        {
            throw std::invalid_argument("the point cloud node needs a depth stream");
        }
        if (m.o.remote != Transport::Compressed and m.o.remote != Transport::Raw)
        {
            throw std::invalid_argument("remote: Compressed or Raw");
        }
        if (not(m.o.max_depth >= 0.0f) or std::isinf(m.o.max_depth))
        {
            throw std::invalid_argument("max_depth " + std::to_string(m.o.max_depth) + ": metres, 0 for no limit");
        }
        if (m.o.max_offset.count() < 0)
        {
            throw std::invalid_argument("max_offset " + std::to_string(m.o.max_offset.count()) + " us: not negative");
        }
        m.camera = m.o.camera.empty() ? parent_of(m.o.depth) : check_name(m.o.camera);
        m.output = m.o.output.empty() ? m.camera + "/points" : check_name(m.o.output);
        check(cudaSetDevice(m.o.cuda_device), "cudaSetDevice");
        // a blocking stream: the frames' DeviceBuffers are freed on the legacy default stream, after its work
        check(cudaStreamCreate(&m.stream), "cudaStreamCreate");
        m.publisher = std::make_unique<PointCloudPublisher>(
            session, m.output,
            CloudPublisherOptions{.layout = m.o.color.empty() ? xyz_layout() : xyzrgb_layout(),
                                  .frame_id = "",  // the depth camera's, from its camera_info (calibrate_depth)
                                  .source = m.o.source,
                                  .slots = m.o.slots,
                                  .cuda_device = m.o.cuda_device,
                                  // cloud/compressed for readers on other hosts; the depth camera's rays follow
                                  // with its calibration (calibrate_depth), before the first cloud
                                  .rays = CloudRays{1, 1, {0.0f, 0.0f, 1.0f}},
                                  .range_step = m.o.range_step,
                                  .jpeg_quality = m.o.jpeg_quality});
        if (not m.o.color.empty())
        {
            m.tf = std::make_unique<StaticTransformSubscriber>(session, m.camera, std::chrono::hours(24 * 365));
        }
        m.diagnostics = std::make_unique<DiagnosticsPublisher>(session, m.camera);
    }

    PointCloudNode::~PointCloudNode()
    {
        Impl& m = *impl_;
        m.unsubscribe();
        m.publisher.reset();
        if (m.stream != nullptr)
        {
            cudaStreamSynchronize(m.stream);
            cudaStreamDestroy(m.stream);
        }
    }

    bool PointCloudNode::step(std::chrono::milliseconds timeout)
    {
        Impl& m = *impl_;
        check(cudaSetDevice(m.o.cuda_device), "cudaSetDevice");  // the calling thread may be another one each time
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        if (not m.subscribed())
        {
            m.unsubscribe();
            std::this_thread::sleep_until(deadline);
            return false;
        }
        m.subscribe();
        const auto slice = std::chrono::milliseconds(5);
        while (true)
        {
            const auto now = std::chrono::steady_clock::now();
            const auto remaining = deadline > now
                                       ? std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now)
                                       : std::chrono::milliseconds(0);
            const auto wait = m.pending ? std::chrono::milliseconds(0) : std::min(slice, remaining);
            if (auto d = m.depth->latest(true, wait))
            {
                ++m.stats.depth_frames;
                if (m.pending)
                {
                    ++m.stats.replaced;
                }
                m.pending = std::move(*d);
                m.pending_color.reset();
                m.pending_since = std::chrono::steady_clock::now();
            }
            if (m.pending)
            {
                const Frame* c = m.color ? m.match() : nullptr;
                if ((not m.color or c != nullptr) and m.publish(c))
                {
                    m.pending.reset();
                    m.pending_color.reset();
                    return true;
                }
                const auto max_offset = std::chrono::duration_cast<std::chrono::nanoseconds>(m.o.max_offset).count();
                bool passed = false;
                if (m.color and c == nullptr)
                {
                    const std::vector<std::int64_t> stamps = m.color->recent_stamps();
                    passed = not stamps.empty() and stamps.back() > m.pending->stamp_ns() + max_offset;
                }
                if (passed or std::chrono::steady_clock::now() - m.pending_since > std::chrono::seconds(1))
                {
                    ++m.stats.unpaired;  // no colour frame of its time will come, or a calibration is missing
                    m.pending.reset();
                    m.pending_color.reset();
                }
            }
            if (std::chrono::steady_clock::now() >= deadline)
            {
                return false;
            }
            if (m.pending)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));  // the colour frame of its time may come
            }
        }
    }

    std::string rate_text(double max_hz)
    {
        if (max_hz <= 0)
        {
            return "every frame";
        }
        std::ostringstream text;
        text << "max " << max_hz << " Hz";  // as Python's :g
        return text.str();
    }

    DiagnosticStatus PointCloudNode::report()
    {
        Impl& m = *impl_;
        const auto now = std::chrono::steady_clock::now();
        const double seconds = std::max(1e-3, std::chrono::duration<double>(now - m.reported).count());
        const PointCloudNodeStats& a = m.stats;
        const PointCloudNodeStats& b = m.reported_stats;
        const std::uint64_t clouds = a.clouds - b.clouds;
        const std::uint64_t unpaired = a.unpaired - b.unpaired;
        // the subscriptions count it (dropped when nobody takes the points, so counted here as it goes)
        const std::uint64_t now_skipped = (m.depth ? m.depth->skipped_max_hz() : 0) +
                                          (m.color ? m.color->skipped_max_hz() : 0);
        if (now_skipped < m.subscriptions_skipped)  // subscribed anew: they count from 0
        {
            m.subscriptions_skipped = 0;
        }
        m.skipped_total += now_skipped - m.subscriptions_skipped;
        m.subscriptions_skipped = now_skipped;
        const std::uint64_t skipped = m.skipped_total;
        DiagnosticStatus status;
        status.name = "point_cloud";
        status.hardware_id = m.output;
        const auto transport = [](const std::unique_ptr<ImageSubscriber>& s) -> std::string {
            if (not s)
            {
                return "-";
            }
            const auto t = s->transport();
            return t ? to_string(*t) : "?";
        };
        status.values = {{"clouds/s", number(double(clouds) / seconds)},
                         {"depth/s", number(double(a.depth_frames - b.depth_frames) / seconds)},
                         {"colour/s", number(double(a.color_frames - b.color_frames) / seconds)},
                         {"unpaired", std::to_string(unpaired)},
                         {"replaced", std::to_string(a.replaced - b.replaced)},
                         {"cloud ms", number(clouds ? (a.ms - b.ms) / double(clouds) : 0.0)},
                         {"stale frames", std::to_string((m.depth ? m.depth->stale() : 0) +
                                                         (m.color ? m.color->stale() : 0))},
                         {"max-hz", rate_text(m.o.max_hz)},
                         {"skipped for max-hz", std::to_string(skipped - m.reported_skipped)},
                         {"depth transport", transport(m.depth)},
                         {"colour transport", transport(m.color)},
                         {"cuda device", std::to_string(m.o.cuda_device)}};
        if (not m.depth)
        {
            status.message = "idle: nobody takes the points";
        }
        else if (a.depth_frames == b.depth_frames)
        {
            status.level = DiagnosticStatus::kError;
            status.message = "no depth frames from " + m.o.depth;
        }
        else if (unpaired != 0)
        {
            status.level = DiagnosticStatus::kWarn;
            status.message = std::to_string(unpaired) + " depth frames without a cloud" +
                             (m.waiting_for.empty() ? " (no colour frame of their time)"
                                                    : " (waiting for " + m.waiting_for + ")");
        }
        else
        {
            status.message = number(double(clouds) / seconds) + " clouds/s";
        }
        if (m.o.max_hz > 0 and skipped > m.reported_skipped)
        {
            status.message += ", " + std::to_string(skipped - m.reported_skipped) + " skipped for max-hz";
        }
        DiagnosticArray array;
        array.header.stamp = Time::from_ns(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                               std::chrono::system_clock::now().time_since_epoch())
                                               .count());
        array.status = {status};
        m.diagnostics->publish(array);
        m.reported = now;
        m.reported_stats = a;
        m.reported_skipped = skipped;
        return status;
    }

    const PointCloudNodeStats& PointCloudNode::stats() const
    {
        return impl_->stats;
    }

    const std::string& PointCloudNode::output() const
    {
        return impl_->output;
    }

    const std::string& PointCloudNode::camera() const
    {
        return impl_->camera;
    }
}  // namespace durchblick
