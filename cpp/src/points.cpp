#include "xblick/points.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <thread>
#include <mutex>
#include <set>

#include "xblick/image.hpp"
#include "xblick/protocol.hpp"
#include "xblick/ray_cloud.hpp"
#include "xblick/version.hpp"
#include "json.hpp"
#include "stream.hpp"

namespace xblick
{
    namespace p = protocol;
    using detail::declare;
    using detail::key;
    using detail::matching;
    using detail::now_ns;
    using detail::put;

    PointLayout xyz_layout()
    {
        return {{{"x", 0, PointField::kFloat32, 1},
                 {"y", 4, PointField::kFloat32, 1},
                 {"z", 8, PointField::kFloat32, 1}},
                16};
    }

    PointLayout xyzrgb_layout()
    {
        PointLayout layout = xyz_layout();
        layout.fields.push_back({"rgb", 12, PointField::kFloat32, 1});
        return layout;
    }

    void check_layout(const PointLayout& layout)
    {
        if (layout.fields.empty() or layout.point_step == 0)
        {
            throw std::invalid_argument("a point layout needs fields and a point_step");
        }
        std::set<std::string> names;
        for (const PointField& f : layout.fields)
        {
            std::uint32_t size = 0;
            try
            {
                size = point_field_size(f.datatype);
            }
            catch (const CdrError& e)
            {
                throw std::invalid_argument("field " + f.name + ": " + e.what());
            }
            if (f.count == 0 or f.offset + size * f.count > layout.point_step)
            {
                throw std::invalid_argument("field " + f.name + " does not fit a point of " +
                                            std::to_string(layout.point_step) + " bytes");
            }
            if (not names.insert(f.name).second)
            {
                throw std::invalid_argument("field " + f.name + " is there twice");
            }
        }
    }

    const PointField* Cloud::field(const std::string& name) const
    {
        const auto it = std::find_if(fields.begin(), fields.end(), [&](const PointField& f) { return f.name == name; });
        return it == fields.end() ? nullptr : &*it;
    }

    // ---------------------------------------------------------------------------------- compressed clouds

    namespace
    {
        // Where x, y, z and (if the layout has it) rgb are in a point: what cloud/compressed holds.
        struct RaysLayout
        {
            ray_cloud::Layout xyz;
            std::optional<std::uint32_t> rgb;
        };

        RaysLayout rays_layout(const std::vector<PointField>& fields, std::uint32_t point_step)
        {
            RaysLayout out;
            out.xyz.point_step = point_step;
            int found = 0;
            for (const PointField& f : fields)
            {
                const bool colour = f.name == "rgb";
                if (f.name != "x" and f.name != "y" and f.name != "z" and not colour)
                {
                    throw std::invalid_argument("cloud/compressed holds the fields x, y, z and rgb, not " + f.name);
                }
                if (f.count != 1 or
                    (f.datatype != PointField::kFloat32 and not(colour and f.datatype == PointField::kUint32)))
                {
                    throw std::invalid_argument("cloud/compressed: the field " + f.name + " is not a float32" +
                                                (colour ? " or uint32" : ""));
                }
                if (colour)
                {
                    out.rgb = f.offset;
                    continue;
                }
                (f.name == "x" ? out.xyz.x : f.name == "y" ? out.xyz.y : out.xyz.z) = f.offset;
                ++found;
            }
            if (found != 3)
            {
                throw std::invalid_argument("cloud/compressed holds the fields x, y, z and rgb: one of x, y, z is missing");
            }
            return out;
        }

        // The rays a publisher sends: the table, its name and the message that answers cloud/rays.
        struct RaysState
        {
            CloudRays rays;
            std::string id;
            std::vector<std::uint8_t> message;
        };

        std::shared_ptr<const RaysState> rays_state(CloudRays rays)
        {
            const std::size_t count = static_cast<std::size_t>(rays.height) * rays.width;
            if (count == 0 or rays.directions.size() != count * 3)
            {
                throw std::invalid_argument("rays: " + std::to_string(rays.directions.size()) + " numbers for " +
                                            std::to_string(rays.width) + " x " + std::to_string(rays.height) +
                                            " points (3 each)");
            }
            for (std::size_t i = 0; i < count; ++i)
            {
                const float* d = rays.directions.data() + 3 * i;
                if (not(std::isfinite(d[0]) and std::isfinite(d[1]) and std::isfinite(d[2])) or
                    d[0] * d[0] + d[1] * d[1] + d[2] * d[2] <= 0.0F)
                {
                    throw std::invalid_argument("rays: finite directions, none of them 0");
                }
            }
            auto state = std::make_shared<RaysState>();
            // the name: FNV-1a (64 bit) of the size and the bytes; a subscriber only compares names
            std::uint64_t hash = 14695981039346656037ULL;
            const auto mix = [&hash](const void* data, std::size_t n) {
                const auto* bytes = static_cast<const std::uint8_t*>(data);
                for (std::size_t i = 0; i < n; ++i)
                {
                    hash = (hash ^ bytes[i]) * 1099511628211ULL;
                }
            };
            mix(&rays.height, 4);
            mix(&rays.width, 4);
            mix(rays.directions.data(), rays.directions.size() * 4);
            char name[17];
            std::snprintf(name, sizeof(name), "%016llx", static_cast<unsigned long long>(hash));
            state->id = name;
            Rays message;
            message.id = state->id;
            message.height = rays.height;
            message.width = rays.width;
            message.directions = {reinterpret_cast<const std::uint8_t*>(rays.directions.data()),
                                  rays.directions.size() * 4};
            state->message = message.serialize();
            state->rays = std::move(rays);
            return state;
        }
    }  // namespace

    // ------------------------------------------------------------------------------------------ publisher

    struct PointCloudPublisher::Impl
    {
        CloudPublisherOptions o;
        std::string host;
        std::optional<zenoh::Publisher> raw, shm, cuda, compressed;
        std::string shm_key, cuda_key;  // the rings' keys: with the host, the CUDA ring's with its GPU too
        RaysLayout compressed_layout;
        std::shared_ptr<const RaysState> rays;  // cloud/compressed: null for a publisher without rays
        mutable std::mutex rays_mutex;          // the rays queryable reads them from Zenoh's thread
        std::vector<std::uint16_t> steps;       // of the cloud being compressed
        std::vector<std::uint8_t> colours;
        detail::RingWriters rings;
        std::int64_t seq = 0;
        std::optional<std::pair<std::uint32_t, std::uint32_t>> loaned;  // height, width
        mutable std::mutex frame_mutex;  // frame_id: the info queryable reads it from Zenoh's thread
        // Last, so destroyed first, also when the constructor throws: Zenoh's threads answer <base>/info (and rays)
        // from the members above, and undeclaring waits for a callback that runs (zenoh-c 1.10). Declared before them,
        // a query during the destruction read freed members (a CUDA ring's server takes up to 200 ms to stop;
        // xblick-relay 0.2.1 stopped with std::length_error).
        std::optional<zenoh::Queryable<void>> info_queryable, rays_queryable;
        std::optional<zenoh::LivelinessToken> alive;  // as CameraPublisher's

        std::string frame_id() const
        {
            std::lock_guard lock(frame_mutex);
            return o.frame_id;
        }

        template <typename Descriptor>
        Descriptor descriptor(const Header& header, std::uint32_t height, std::uint32_t width, bool is_dense,
                              const Written& w) const
        {
            Descriptor d;
            d.header = header;
            d.height = height;
            d.width = width;
            d.fields = o.layout.fields;
            d.point_step = o.layout.point_step;
            d.row_step = width * o.layout.point_step;
            d.is_dense = is_dense;
            d.host = host;
            d.slot = w.slot;
            d.seq = w.seq;
            return d;
        }

        std::uint64_t nbytes(std::uint32_t height, std::uint32_t width) const
        {
            return static_cast<std::uint64_t>(height) * width * o.layout.point_step;
        }

        std::shared_ptr<const RaysState> current_rays() const
        {
            std::lock_guard lock(rays_mutex);
            return rays;
        }

        // cloud/compressed of a cloud in host memory: its ranges on the rays (RVL) and its colours.
        void publish_compressed(const std::string& base, const std::uint8_t* points, std::uint32_t height,
                                std::uint32_t width, bool is_dense, const Header& header,
                                const detail::FrameAttachment& attachment, std::vector<std::string>& sent)
        {
            const std::shared_ptr<const RaysState> state = current_rays();
            if (state->rays.height != height or state->rays.width != width)
            {
                throw std::invalid_argument(base + ": a cloud of " + std::to_string(width) + " x " +
                                            std::to_string(height) + " points, rays for " +
                                            std::to_string(state->rays.width) + " x " +
                                            std::to_string(state->rays.height));
            }
            const std::size_t count = static_cast<std::size_t>(height) * width;
            steps.resize(count);
            const ray_cloud::Fit fit = ray_cloud::ranges(points, count, compressed_layout.xyz,
                                                         state->rays.directions.data(), o.range_step, steps.data());
            if (fit.valid != 0)
            {
                if (fit.low < 1 or fit.high > 65535)
                {
                    throw std::invalid_argument(base + ": cloud/compressed: ranges of " +
                                                std::to_string(fit.low * o.range_step) + " to " +
                                                std::to_string(fit.high * o.range_step) + " m (along the rays) do not "
                                                "fit 1 to 65535 steps of " + std::to_string(o.range_step) +
                                                " m: choose another range_step");
                }
                if (fit.deviation > fit.limit)  // a point is at most half a step along its ray from its rounded place
                {
                    throw std::invalid_argument(base + ": cloud/compressed: the cloud does not lie on its rays (a "
                                                "point is " + std::to_string(fit.deviation) + " m off, more than " +
                                                std::to_string(fit.limit) + " m): the rays are those of another "
                                                "camera or frame");
                }
            }
            const CompressedImage range = compress(HostImage{steps.data(), height, width, 0}, "16UC1", kRvl, 0, {});
            CompressedImage colour;
            if (compressed_layout.rgb)
            {
                colours.resize(count * 3);  // PCL's packed rgb is 0x00RRGGBB: the bytes b, g, r
                const std::uint8_t* packed = points + *compressed_layout.rgb;
                for (std::size_t i = 0; i < count; ++i, packed += o.layout.point_step)
                {
                    colours[3 * i] = packed[2];
                    colours[3 * i + 1] = packed[1];
                    colours[3 * i + 2] = packed[0];
                }
                colour = compress(HostImage{colours.data(), height, width, 0}, "rgb8", o.color_codec, o.jpeg_quality, {});
            }
            CompressedPointCloud2 m;
            m.header = header;
            m.height = height;
            m.width = width;
            m.fields = o.layout.fields;
            m.point_step = o.layout.point_step;
            m.is_dense = is_dense;
            m.rays = state->id;
            m.range_step = o.range_step;
            m.range_format = range.format;
            m.range = range.data;
            m.color_format = colour.format;
            m.color = colour.data;
            put(*compressed, m.serialize(), attachment);
            sent.push_back(key(base, p::kCloudCompressed));
        }

        // A cloud written into CUDA ring slot w: its descriptor, and the host representations from one download.
        std::vector<std::string> publish_device(const std::string& base, const Written& w, std::uint32_t height,
                                                std::uint32_t width, bool is_dense,
                                                std::optional<std::int64_t> stamp_ns)
        {
            const std::uint64_t n = nbytes(height, width);
            CudaRingWriter& ring = *rings.cuda;
            const Header header{Time::from_ns(stamp_ns.value_or(now_ns())), frame_id()};
            ++seq;
            const detail::FrameAttachment attachment = detail::frame_attachment(seq, o.source);
            std::vector<std::string> sent;
            if (matching(cuda))
            {
                CudaPointCloud2 d = descriptor<CudaPointCloud2>(header, height, width, is_dense, w);
                d.endpoint = ring.endpoint();
                d.device_uuid = ring.device_uuid();
                put(*cuda, d.serialize(), attachment);
                sent.push_back(cuda_key);
            }
            if (matching(shm))
            {
                // one download, straight into the host ring's next slot; raw reads it there
                HostRingWriter& host_ring = rings.pinned_host_ring(o.slots, n);
                std::uint8_t* slot = host_ring.loan();
                try
                {
                    ring.read_host(w.slot, slot, n);
                }
                catch (...)
                {
                    host_ring.commit(0);
                    throw;
                }
                const Written in_shm = host_ring.commit(n);
                publish_host(base, slot, height, width, is_dense, header, attachment, sent, &in_shm);
            }
            else if (matching(raw) or matching(compressed))
            {
                std::vector<std::uint8_t> host(n);
                ring.read_host(w.slot, host.data(), n);
                publish_host(base, host.data(), height, width, is_dense, header, attachment, sent);
            }
            return sent;
        }

        // The host representations (shm, raw) of a cloud in host memory; `in_shm`: it is in that host ring slot.
        void publish_host(const std::string& base, const std::uint8_t* points, std::uint32_t height,
                          std::uint32_t width, bool is_dense, const Header& header,
                          const detail::FrameAttachment& attachment, std::vector<std::string>& sent,
                          const Written* in_shm = nullptr)
        {
            const std::uint64_t n = nbytes(height, width);
            if (in_shm or matching(shm))
            {
                const Written w = in_shm ? *in_shm : rings.host_ring(o.slots, n).write(points, n);
                ShmPointCloud2 d = descriptor<ShmPointCloud2>(header, height, width, is_dense, w);
                d.segment = rings.host->name();
                put(*shm, d.serialize(), attachment);
                sent.push_back(shm_key);
            }
            if (matching(raw))
            {
                PointCloud2 m;
                m.header = header;
                m.height = height;
                m.width = width;
                m.fields = o.layout.fields;
                m.point_step = o.layout.point_step;
                m.row_step = width * o.layout.point_step;
                m.data = std::span<const std::uint8_t>(points, n);
                m.is_dense = is_dense;
                put(*raw, m.serialize(), attachment);
                sent.push_back(key(base, p::kCloud));
            }
            if (matching(compressed))
            {
                publish_compressed(base, points, height, width, is_dense, header, attachment, sent);
            }
        }
    };

    PointCloudPublisher::PointCloudPublisher(zenoh::Session& session, const std::string& base,
                                             CloudPublisherOptions options)
        : base_(check_name(base)), impl_(std::make_unique<Impl>())
    {
        Impl& m = *impl_;
        m.o = std::move(options);
        check_layout(m.o.layout);
        m.host = host_id();
        m.raw = declare(session, key(base_, p::kCloud), p::kPointCloud2Type);
        m.shm_key = detail::ring_key(base_, p::kCloudShm, m.host);
        m.shm = declare(session, m.shm_key, p::kShmPointCloud2Type);
        if (m.o.cuda_device)
        {
            m.cuda_key = detail::ring_key(base_, p::kCloudCuda, m.host, detail::ring_gpu(base_, *m.o.cuda_device));
            m.cuda = declare(session, m.cuda_key, p::kCudaPointCloud2Type);
        }
        if (m.o.rays)
        {
            if (not(m.o.range_step > 0))
            {
                throw std::invalid_argument("range_step: a positive number of metres");
            }
            m.compressed_layout = rays_layout(m.o.layout.fields, m.o.layout.point_step);
            m.rays = rays_state(std::move(*m.o.rays));
            m.o.rays.reset();
            m.compressed = declare(session, key(base_, p::kCloudCompressed), p::kCompressedPointCloud2Type);
            const std::string rays_key = key(base_, p::kCloudRays);
            m.rays_queryable = session.declare_queryable(
                zenoh::KeyExpr(rays_key),
                [this, rays_key](const zenoh::Query& query) {
                    zenoh::Query::ReplyOptions options;
                    options.encoding = zenoh::Encoding(p::kRaysType.encoding());
                    query.reply(zenoh::KeyExpr(rays_key), zenoh::Bytes(impl_->current_rays()->message),
                                std::move(options));
                },
                zenoh::closures::none);
        }
        const std::string info_key = key(base_, p::kInfo);
        m.info_queryable = session.declare_queryable(
            zenoh::KeyExpr(info_key),
            [this, info_key](const zenoh::Query& query) { query.reply(zenoh::KeyExpr(info_key), info_record()); },
            zenoh::closures::none);
        m.alive.emplace(session.liveliness_declare_token(zenoh::KeyExpr(detail::alive_key(base_, m.host))));
    }

    PointCloudPublisher::~PointCloudPublisher() = default;  // Impl's queryables and token go first (its last members)

    bool PointCloudPublisher::subscribed() const
    {
        const Impl& m = *impl_;
        return matching(m.raw) or matching(m.shm) or matching(m.cuda) or matching(m.compressed);
    }

    std::string PointCloudPublisher::info_record() const
    {
        const Impl& m = *impl_;
        std::vector<std::string> transports = {"raw", "shm"};
        if (m.cuda)
        {
            transports.push_back("cuda");
        }
        if (m.compressed)
        {
            transports.push_back("compressed");
        }
        std::vector<std::string> fields;
        for (const PointField& f : m.o.layout.fields)
        {
            fields.push_back(f.name);
        }
        // the ring's GPU, also before a reader made the ring exist (a subscriber's local Auto compares it with its own)
        std::string cuda_uuid = m.rings.uuid();
        if (cuda_uuid.empty() and m.o.cuda_device)
        {
            cuda_uuid = cuda_device_uuid(*m.o.cuda_device);
        }
        const json::Value uuid = cuda_uuid.empty() ? json::Value(nullptr) : json::Value(cuda_uuid);
        return json::write({{"v", static_cast<std::int64_t>(p::kInfoVersion)},
                            {"kind", std::string("points")},
                            {"xblick", std::string(XBLICK_VERSION)},
                            {"host", m.host},
                            {"transports", transports},
                            {"fields", fields},
                            {"frame_id", m.frame_id()},
                            {"source", m.o.source},
                            {"cuda_device_uuid", uuid}});
    }

    void PointCloudPublisher::set_frame_id(const std::string& frame_id)
    {
        std::lock_guard lock(impl_->frame_mutex);
        impl_->o.frame_id = frame_id;
    }

    void PointCloudPublisher::set_rays(CloudRays rays)
    {
        Impl& m = *impl_;
        if (not m.compressed)
        {
            throw std::invalid_argument(base_ + ": a publisher made without rays offers no cloud/compressed");
        }
        std::shared_ptr<const RaysState> state = rays_state(std::move(rays));
        std::lock_guard lock(m.rays_mutex);
        m.rays = std::move(state);
    }

    std::vector<std::string> PointCloudPublisher::publish(const HostCloud& cloud, std::optional<std::int64_t> stamp_ns)
    {
        Impl& m = *impl_;
        if (m.loaned)
        {
            throw std::logic_error("publish: a loaned cloud has not been committed");
        }
        const Header header{Time::from_ns(stamp_ns.value_or(now_ns())), m.frame_id()};
        ++m.seq;
        const detail::FrameAttachment attachment = detail::frame_attachment(m.seq, m.o.source);
        std::vector<std::string> sent;
        const auto* points = static_cast<const std::uint8_t*>(cloud.data);
        if (matching(m.cuda))
        {
            const std::uint64_t n = m.nbytes(cloud.height, cloud.width);
            const Written w = m.rings.cuda_ring(*m.o.cuda_device, m.o.slots, n).write_host(points, n);
            CudaPointCloud2 d = m.descriptor<CudaPointCloud2>(header, cloud.height, cloud.width, cloud.is_dense, w);
            d.endpoint = m.rings.cuda->endpoint();
            d.device_uuid = m.rings.cuda->device_uuid();
            put(*m.cuda, d.serialize(), attachment);
            sent.push_back(m.cuda_key);
        }
        m.publish_host(base_, points, cloud.height, cloud.width, cloud.is_dense, header, attachment, sent);
        return sent;
    }

    std::uint64_t PointCloudPublisher::loan(std::uint32_t height, std::uint32_t width)
    {
        Impl& m = *impl_;
        if (not m.o.cuda_device)
        {
            throw std::invalid_argument("loan: needs a publisher with cuda_device");
        }
        CudaRingWriter& ring = m.rings.cuda_ring(*m.o.cuda_device, m.o.slots, m.nbytes(height, width));  // grows
        const std::uint64_t ptr = ring.loan();
        m.loaned = {height, width};
        return ptr;
    }

    std::vector<std::string> PointCloudPublisher::commit(std::uint64_t stream, bool is_dense,
                                                         std::optional<std::int64_t> stamp_ns)
    {
        Impl& m = *impl_;
        if (not m.loaned)
        {
            throw std::logic_error("commit: no cloud is loaned");
        }
        const auto [height, width] = *m.loaned;
        m.loaned.reset();
        const Written w = m.rings.cuda->commit(m.nbytes(height, width), stream);
        return m.publish_device(base_, w, height, width, is_dense, stamp_ns);
    }

    std::vector<std::string> PointCloudPublisher::publish(const DeviceCloud& cloud,
                                                          std::optional<std::int64_t> stamp_ns)
    {
        Impl& m = *impl_;
        if (not m.o.cuda_device)
        {
            throw std::invalid_argument("a device cloud needs a publisher with cuda_device");
        }
        if (m.loaned)
        {
            throw std::logic_error("publish: a loaned cloud has not been committed");
        }
        const std::uint64_t n = m.nbytes(cloud.height, cloud.width);
        const Written w = m.rings.cuda_ring(*m.o.cuda_device, m.o.slots, n).write_device(cloud.ptr, n, cloud.stream);
        return m.publish_device(base_, w, cloud.height, cloud.width, cloud.is_dense, stamp_ns);
    }

    // ----------------------------------------------------------------------------------------- subscriber

    namespace
    {
        const p::MessageType& cloud_type_of(Transport t)
        {
            switch (t)
            {
                case Transport::Raw: return p::kPointCloud2Type;
                case Transport::Shm: return p::kShmPointCloud2Type;
                case Transport::Cuda: return p::kCudaPointCloud2Type;
                case Transport::Compressed: return p::kCompressedPointCloud2Type;
                case Transport::Auto: break;
            }
            throw std::logic_error("no point cloud type for " + std::string(to_string(t)));
        }

        std::string_view cloud_suffix_of(Transport t)
        {
            switch (t)
            {
                case Transport::Raw: return p::kCloud;
                case Transport::Shm: return p::kCloudShm;
                case Transport::Cuda: return p::kCloudCuda;
                case Transport::Compressed: return p::kCloudCompressed;
                case Transport::Auto: break;
            }
            throw std::logic_error("no point cloud topic for " + std::string(to_string(t)));
        }

        // The points of a cloud must be little endian, as long as the layout says and inside the points; rows of a
        // ring cloud have no padding.
        void check_cloud(const std::string& base, const RingPointCloud2& d)
        {
            if (d.is_bigendian)
            {
                throw std::runtime_error(base + ": big-endian clouds are not supported");
            }
            check_layout({d.fields, d.point_step});
            if (d.row_step != d.width * d.point_step)
            {
                throw std::runtime_error(base + ": ring clouds have rows without padding (row_step " +
                                         std::to_string(d.row_step) + ", width * point_step " +
                                         std::to_string(d.width * d.point_step) + ")");
            }
        }
    }  // namespace

    struct PointCloudSubscriber::Impl
    {
        CloudSubscriberOptions o;
        detail::Subscription subscription;
        detail::RingReaders rings;
        std::atomic<std::uint64_t> stale{0};
        zenoh::Session& session;
        std::string rays_id;            // of the rays held: asked at the first compressed cloud that names them
        std::vector<float> directions;
        std::uint32_t rays_height = 0, rays_width = 0;

        // Auto from another host: compressed where every source offers it (has rays), else raw
        Impl(zenoh::Session& session, const std::string& base, CloudSubscriberOptions options)
            : o(options),
              subscription(session, base, "points", o.transport, o.local,
                           o.remote == Transport::Raw ? Transport::Raw : Transport::Compressed, cloud_suffix_of,
                           o.cuda_device,
                           o.remote == Transport::Auto ? std::optional<Transport>(Transport::Raw) : std::nullopt),
              session(session)
        {
        }

        // The rays of this name, asked from the publisher.
        void fetch_rays(const std::string& name)
        {
            const std::string& base = subscription.base();
            const std::string rays_key = key(base, p::kCloudRays);
            // asked again until these rays come (another publisher's, or older ones, may answer first) or 5 s passed
            using clock = std::chrono::steady_clock;
            const auto end = clock::now() + std::chrono::milliseconds(5000);
            while (clock::now() < end)
            {
                const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(end - clock::now());
                for (const zenoh::Reply& reply : ask(session, rays_key, std::max(left, std::chrono::milliseconds(10))))
                {
                    if (not reply.is_ok())
                    {
                        continue;
                    }
                    const std::vector<std::uint8_t> payload = reply.get_ok().get_payload().as_vector();
                    const Rays rays = Rays::deserialize(payload);
                    if (rays.id != name)
                    {
                        continue;  // another publisher of the stream, or rays that changed meanwhile
                    }
                    if (rays.directions.size() != static_cast<std::size_t>(rays.height) * rays.width * 12)
                    {
                        throw ProtocolError(base + ": the rays " + name + " do not have their size");
                    }
                    directions.resize(rays.directions.size() / 4);
                    std::memcpy(directions.data(), rays.directions.data(), rays.directions.size());
                    rays_height = rays.height;
                    rays_width = rays.width;
                    rays_id = name;
                    return;
                }
                std::this_thread::sleep_for(
                    std::min<clock::duration>(std::chrono::milliseconds(250), end - clock::now()));
            }
            throw ProtocolError(base + ": no publisher answered the rays " + name + " of its compressed clouds within "
                                "5 s (" + rays_key + ")");
        }
    };

    PointCloudSubscriber::PointCloudSubscriber(zenoh::Session& session, const std::string& base,
                                               CloudSubscriberOptions options)
    {
        if (options.remote != Transport::Auto and options.remote != Transport::Raw and
            options.remote != Transport::Compressed)
        {
            throw std::invalid_argument("remote: Auto, Raw or Compressed");
        }
        if (options.local != Transport::Auto and options.local != Transport::Shm and options.local != Transport::Cuda)
        {
            throw std::invalid_argument("local: Auto, Shm or Cuda");
        }
        if (not options.copy and (options.transport == Transport::Raw or options.transport == Transport::Compressed))
        {
            throw std::invalid_argument("copy = false is for the shm and cuda transports (and auto)");
        }
        impl_ = std::make_unique<Impl>(session, check_name(base), options);
        impl_->subscription.limit_rate(options.max_hz);
    }

    PointCloudSubscriber::~PointCloudSubscriber() = default;

    std::optional<Transport> PointCloudSubscriber::transport() const
    {
        return impl_->subscription.transport();
    }

    std::string PointCloudSubscriber::transport_reason() const
    {
        return impl_->subscription.transport_reason();
    }

    std::uint64_t PointCloudSubscriber::received() const
    {
        return impl_->subscription.received();
    }

    std::uint64_t PointCloudSubscriber::overwritten() const
    {
        return impl_->subscription.overwritten();
    }

    std::uint64_t PointCloudSubscriber::stale() const
    {
        return impl_->stale.load();
    }

    std::uint64_t PointCloudSubscriber::skipped_max_hz() const
    {
        return impl_->subscription.skipped_max_hz();
    }

    std::optional<Cloud> PointCloudSubscriber::latest(bool consume, std::optional<std::chrono::milliseconds> timeout)
    {
        Impl& m = *impl_;
        const auto taken = m.subscription.take(consume, timeout);
        if (not taken)
        {
            return std::nullopt;
        }
        const detail::Item& item = *taken->item;
        const Transport t = taken->transport;
        detail::check_encoding(item, cloud_type_of(t));

        Cloud c;
        c.key = item.key;
        c.receive_ns = item.receive_ns;
        const detail::Meta meta = detail::parse_attachment(item);
        c.seq = meta.seq;
        c.publish_ns = meta.publish_ns;
        c.source = meta.source;
        const std::string& base = m.subscription.base();

        if (t == Transport::Compressed)
        {
            const CompressedPointCloud2 message = CompressedPointCloud2::deserialize(item.payload);
            check_layout({message.fields, message.point_step});
            const RaysLayout layout = rays_layout(message.fields, message.point_step);
            if (m.rays_id != message.rays)
            {
                m.fetch_rays(message.rays);
            }
            DecodedImage steps;
            DecodedImage colours;
            try
            {
                steps = decompress(CompressedImage{{}, message.range_format, {message.range.begin(), message.range.end()}});
                if (layout.rgb)
                {
                    colours = decompress(
                        CompressedImage{{}, message.color_format, {message.color.begin(), message.color.end()}});
                }
            }
            catch (const EncodingError& error)
            {
                throw ProtocolError(base + ": a compressed cloud's images: " + error.what());
            }
            const std::size_t count = static_cast<std::size_t>(message.height) * message.width;
            if (steps.height != message.height or steps.width != message.width or steps.pixels.size() != count * 2 or
                m.rays_height != message.height or m.rays_width != message.width or
                (layout.rgb and (colours.height != message.height or colours.width != message.width or
                                 colours.encoding != "rgb8" or colours.pixels.size() != count * 3)))
            {
                throw ProtocolError(base + ": a compressed cloud of " + std::to_string(message.width) + " x " +
                                    std::to_string(message.height) + " points whose ranges, colours or rays have "
                                    "another size");
            }
            c.points.assign(count * message.point_step, 0);
            ray_cloud::points(reinterpret_cast<const std::uint16_t*>(steps.pixels.data()), count, m.directions.data(),
                              message.range_step, layout.xyz, c.points.data());
            if (layout.rgb)
            {
                std::uint8_t* packed = c.points.data() + *layout.rgb;
                for (std::size_t i = 0; i < count; ++i, packed += message.point_step)
                {
                    packed[0] = colours.pixels[3 * i + 2];
                    packed[1] = colours.pixels[3 * i + 1];
                    packed[2] = colours.pixels[3 * i];
                }
            }
            c.header = message.header;
            c.height = message.height;
            c.width = message.width;
            c.fields = message.fields;
            c.point_step = message.point_step;
            c.row_step = message.width * message.point_step;
            c.is_dense = message.is_dense;
            return c;
        }

        if (t == Transport::Raw)
        {
            const PointCloud2 message = PointCloud2::deserialize(item.payload);
            if (message.is_bigendian)
            {
                throw std::runtime_error(base + ": big-endian clouds are not supported");
            }
            check_layout({message.fields, message.point_step});
            if (message.row_step < message.width * message.point_step or
                message.data.size() < static_cast<std::size_t>(message.row_step) * message.height)
            {
                throw std::runtime_error(base + ": the cloud's data does not fit its size and row_step");
            }
            c.header = message.header;
            c.height = message.height;
            c.width = message.width;
            c.fields = message.fields;
            c.point_step = message.point_step;
            c.row_step = message.row_step;
            c.is_dense = message.is_dense;
            if (message.row_step == message.width * message.point_step)
            {
                c.view = message.data.data();  // into the payload, which the cloud keeps
                c.owner = taken->item;
            }
            else  // rows with padding: packed, as every cloud this API hands out
            {
                const std::size_t row = static_cast<std::size_t>(message.width) * message.point_step;
                c.points.resize(row * message.height);
                for (std::uint32_t y = 0; y < message.height; ++y)
                {
                    std::copy_n(message.data.data() + static_cast<std::size_t>(y) * message.row_step, row,
                                c.points.data() + y * row);
                }
                c.row_step = static_cast<std::uint32_t>(row);
            }
            return c;
        }

        RingPointCloud2* d = nullptr;
        ShmPointCloud2 shm;
        CudaPointCloud2 cuda;
        if (t == Transport::Shm)
        {
            shm = ShmPointCloud2::deserialize(item.payload);
            d = &shm;
        }
        else
        {
            cuda = CudaPointCloud2::deserialize(item.payload);
            d = &cuda;
        }
        if (d->host != m.subscription.host())
        {
            m.subscription.moved(d->host, t);
            return std::nullopt;
        }
        check_cloud(base, *d);
        c.header = d->header;
        c.height = d->height;
        c.width = d->width;
        c.fields = d->fields;
        c.point_step = d->point_step;
        c.row_step = d->row_step;
        c.is_dense = d->is_dense;
        const std::uint64_t nbytes = static_cast<std::uint64_t>(d->row_step) * d->height;
        std::optional<detail::RingData> data = t == Transport::Shm
                                                   ? m.rings.host(shm.segment, d->slot, d->seq, nbytes, m.o.copy)
                                                   : m.rings.cuda(cuda.endpoint, d->slot, d->seq, m.o.copy, m.o.cuda_device);
        if (not data)
        {
            ++m.stale;
            return std::nullopt;
        }
        c.points = std::move(data->bytes);
        c.view = data->view;
        c.device_ptr = data->device_ptr;
        c.device = data->device;
        c.owner = std::move(data->owner);
        c.valid = std::move(data->valid);
        return c;
    }
}  // namespace xblick
