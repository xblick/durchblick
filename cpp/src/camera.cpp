#include "xblick/camera.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <set>
#include <regex>

#include "xblick/protocol.hpp"
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

    namespace
    {
        std::string json_list(const std::vector<std::string>& items)
        {
            std::string out = "[";
            for (std::size_t i = 0; i < items.size(); ++i)
            {
                out += (i ? "," : "") + json::quote(items[i]);
            }
            return out + "]";
        }
    }  // namespace

    // ------------------------------------------------------------------------------------------- sessions

    std::string endpoint(const std::string& host, std::uint16_t port)
    {
        if (host.find('/') != std::string::npos)
        {
            return host;
        }
        if (host.find(':') != std::string::npos)
        {
            return "tcp/" + host;
        }
        return "tcp/" + host + ":" + std::to_string(port);
    }

    zenoh::Session open_session(const SessionOptions& o)
    {
        if (not o.config_file.empty())
        {
            if (not o.connect.empty() or not o.listen.empty())
            {
                throw std::invalid_argument("give either a Zenoh config file or connect/listen endpoints, not both");
            }
            return zenoh::Session::open(zenoh::Config::from_file(o.config_file));
        }
        if (o.mode != "peer" and o.mode != "client")
        {
            throw std::invalid_argument("mode '" + o.mode + "': peer or client");
        }
        std::vector<std::string> connect = o.connect, listen = o.listen;
        if (o.local_router and connect.empty() and listen.empty())
        {
            connect = {"tcp/127.0.0.1:" + std::to_string(p::kDefaultPort)};
            if (o.mode == "peer")
            {
                listen = {"tcp/127.0.0.1:0"};
            }
        }
        if (o.mode == "client" and connect.empty())
        {
            throw std::invalid_argument("client mode needs an endpoint to connect to (a router or a peer)");
        }
        auto endpoints = [](const std::vector<std::string>& hosts) {
            std::vector<std::string> out;
            for (const auto& h : hosts)
            {
                out.push_back(endpoint(h, p::kDefaultPort));
            }
            return json_list(out);
        };
        zenoh::Config config = zenoh::Config::create_default();
        config.insert_json5("mode", json::quote(o.mode));
        config.insert_json5("scouting/multicast/enabled", o.multicast_scouting ? "true" : "false");
        config.insert_json5("connect/endpoints", endpoints(connect));
        if (o.mode == "peer")
        {
            config.insert_json5("listen/endpoints", endpoints(listen));
        }
        return zenoh::Session::open(std::move(config));
    }

    std::string check_name(const std::string& path)
    {
        static const std::regex token("^[A-Za-z_][A-Za-z0-9_]*$");
        std::string trimmed = path;
        trimmed.erase(0, trimmed.find_first_not_of('/'));
        trimmed.erase(trimmed.find_last_not_of('/') + 1);
        std::size_t start = 0;
        while (true)
        {
            const std::size_t end = trimmed.find('/', start);
            const std::string part = trimmed.substr(start, end == std::string::npos ? std::string::npos : end - start);
            if (not std::regex_match(part, token))
            {
                throw std::invalid_argument("'" + path + "': '" + part +
                                            "' is not a valid ROS name token ([A-Za-z_][A-Za-z0-9_]*)");
            }
            if (end == std::string::npos)
            {
                return trimmed;
            }
            start = end + 1;
        }
    }

    std::string stream_base(const std::string& ns, const std::string& camera, const std::string& stream)
    {
        return check_name(ns + "/" + camera + "/" + stream);
    }

    // ------------------------------------------------------------------------------------------ publisher

    namespace
    {
        // PublisherOptions::transports checked; all (cuda with cuda_device) if it is empty
        std::vector<std::string> offered_transports(const PublisherOptions& o)
        {
            if (o.transports.empty())
            {
                std::vector<std::string> all = {"raw", "compressed", "shm"};
                if (o.cuda_device)
                {
                    all.push_back("cuda");
                }
                return all;
            }
            std::vector<std::string> out;
            for (const std::string& t : o.transports)
            {
                if (t != "raw" and t != "compressed" and t != "shm" and t != "cuda")
                {
                    throw std::invalid_argument("transport '" + t + "': raw, compressed, shm or cuda");
                }
                if (t == "cuda" and not o.cuda_device)
                {
                    throw std::invalid_argument("transport cuda needs cuda_device");
                }
                if (std::find(out.begin(), out.end(), t) != out.end())
                {
                    throw std::invalid_argument("transport '" + t + "' given twice");
                }
                out.push_back(t);
            }
            return out;
        }
    }  // namespace

    struct CameraPublisher::Impl
    {
        PublisherOptions o;
        EncodingInfo enc{};
        std::string host;
        std::optional<zenoh::Publisher> raw, compressed, shm, cuda, camera_info;
        std::string shm_key, cuda_key;  // the rings' keys: with the host, the CUDA ring's with its GPU too
        detail::RingWriters rings;
        std::int64_t seq = 0;
        std::vector<std::string> offered;  // PublisherOptions::transports, checked (all if it was empty)
        // Last, so destroyed first, also when the constructor throws: Zenoh's threads answer <base>/info (and rays)
        // from the members above, and undeclaring waits for a callback that runs (zenoh-c 1.10). Declared before them,
        // a query during the destruction read freed members (a CUDA ring's server takes up to 200 ms to stop;
        // xblick-relay 0.2.1 stopped with std::length_error).
        std::optional<zenoh::Queryable<void>> info_queryable;
        std::optional<zenoh::LivelinessToken> alive;  // tells subscribers that this publisher came (and went)

        // each put takes it anew: each transport's publish_ns after its encoding or ring copy
        detail::FrameAttachment attachment() const
        {
            return detail::frame_attachment(seq, o.source);
        }

        // Host representations of a frame (rows packed if the image has padding where a ring needs none). With
        // `in_shm`, the frame is already in the host ring's slot `*in_shm` (image points there): only its descriptor
        // goes out.
        void publish_host(const std::string& base, const HostImage& image, const Header& header,
                          const detail::FrameAttachment& attachment, std::vector<std::string>& sent,
                          std::span<const std::uint8_t> jpeg, const Written* in_shm = nullptr)
        {
            const std::uint32_t row = image.width * static_cast<std::uint32_t>(enc.pixel_bytes());
            const std::uint32_t step = image.step == 0 ? row : image.step;
            if (step < row)
            {
                throw EncodingError("step " + std::to_string(step) + " is smaller than a row of " +
                                    std::to_string(row) + " bytes");
            }
            const auto* pixels = static_cast<const std::uint8_t*>(image.data);
            if (in_shm)
            {
                ShmImage d;
                fill(d, header, image.height, image.width, row, *in_shm);
                d.segment = rings.host->name();
                put(*shm, d.serialize(), attachment);
                sent.push_back(shm_key);
            }
            else if (matching(shm))
            {
                std::vector<std::uint8_t> packed;
                const std::uint8_t* tight = pixels;
                if (step != row)
                {
                    packed.resize(static_cast<std::size_t>(row) * image.height);
                    for (std::uint32_t y = 0; y < image.height; ++y)
                    {
                        std::memcpy(packed.data() + static_cast<std::size_t>(y) * row,
                                    pixels + static_cast<std::size_t>(y) * step, row);
                    }
                    tight = packed.data();
                }
                const std::uint64_t nbytes = static_cast<std::uint64_t>(row) * image.height;
                const Written w = rings.host_ring(o.slots, nbytes).write(tight, nbytes);
                ShmImage d;
                fill(d, header, image.height, image.width, row, w);
                d.segment = rings.host->name();
                put(*shm, d.serialize(), attachment);
                sent.push_back(shm_key);
            }
            if (matching(raw))
            {
                Image m;
                m.header = header;
                m.height = image.height;
                m.width = image.width;
                m.encoding = o.encoding;
                m.step = step;
                m.data = std::span<const std::uint8_t>(pixels, static_cast<std::size_t>(step) * image.height);
                put(*raw, m.serialize(), attachment);
                sent.push_back(key(base, p::kImageRaw));
            }
            if (matching(compressed))
            {
                put(*compressed,
                    (jpeg.empty() ? compress(image, o.encoding, o.codec, o.jpeg_quality, header)
                                  : jpeg_message(jpeg, o.encoding, header))
                        .serialize(),
                    attachment);
                sent.push_back(key(base, p::kCompressed));
            }
        }

        void check_jpeg(std::span<const std::uint8_t> jpeg) const
        {
            if (not jpeg.empty() and o.codec != kJpeg)
            {
                throw std::invalid_argument("a JPEG frame needs a publisher with codec jpeg, not " + o.codec);
            }
        }

        void fill(RingImage& d, const Header& header, std::uint32_t height, std::uint32_t width, std::uint32_t step,
                  const Written& w) const
        {
            d.header = header;
            d.height = height;
            d.width = width;
            d.encoding = o.encoding;
            d.step = step;
            d.host = host;
            d.slot = w.slot;
            d.seq = w.seq;
        }

        std::vector<std::string> publish_host_frame(const std::string& base, const HostImage& image,
                                                    const Header& header, const detail::FrameAttachment& attachment,
                                                    std::span<const std::uint8_t> jpeg);
        std::vector<std::string> publish_device_frame(const std::string& base, const DeviceImage& image,
                                                      const Header& header, const detail::FrameAttachment& attachment,
                                                      std::span<const std::uint8_t> jpeg);

        CudaRingWriter& cuda_ring(std::uint64_t nbytes)
        {
            return rings.cuda_ring(*o.cuda_device, o.slots, nbytes);
        }

        void publish_cuda_descriptor(const Written& w, std::uint32_t height,
                                     std::uint32_t width, const Header& header,
                                     const detail::FrameAttachment& attachment, std::vector<std::string>& sent)
        {
            CudaImage d;
            fill(d, header, height, width, width * static_cast<std::uint32_t>(enc.pixel_bytes()), w);
            d.endpoint = rings.cuda->endpoint();
            d.device_uuid = rings.cuda->device_uuid();
            put(*cuda, d.serialize(), attachment);
            sent.push_back(cuda_key);
        }

        void publish_camera_info(const std::string& base, const Header& header,
                                 const detail::FrameAttachment& attachment, std::vector<std::string>& sent)
        {
            if (o.camera_info and matching(camera_info))
            {
                CameraInfo info = *o.camera_info;
                info.header = header;
                put(*camera_info, info.serialize(), attachment);
                sent.push_back(key(base, p::kCameraInfo));
            }
        }
    };

    CameraPublisher::CameraPublisher(zenoh::Session& session, const std::string& base, PublisherOptions options)
        : base_(check_name(base)), impl_(std::make_unique<Impl>())
    {
        Impl& m = *impl_;
        m.o = std::move(options);
        m.enc = encoding_info(m.o.encoding);
        m.host = host_id();
        m.offered = offered_transports(m.o);
        const auto offers = [&m](const char* t) {
            return std::find(m.offered.begin(), m.offered.end(), t) != m.offered.end();
        };
        if (offers("raw"))
        {
            m.raw = declare(session, key(base_, p::kImageRaw), p::kImageType);
        }
        if (offers("compressed"))
        {
            m.compressed = declare(session, key(base_, p::kCompressed), p::kCompressedImageType);
        }
        if (offers("shm"))
        {
            m.shm_key = detail::ring_key(base_, p::kShm, m.host);
            m.shm = declare(session, m.shm_key, p::kShmImageType);
        }
        if (offers("cuda"))
        {
            m.cuda_key = detail::ring_key(base_, p::kCuda, m.host, detail::ring_gpu(base_, *m.o.cuda_device));
            m.cuda = declare(session, m.cuda_key, p::kCudaImageType);
        }
        if (m.o.camera_info)
        {
            m.camera_info = declare(session, key(base_, p::kCameraInfo), p::kCameraInfoType);
        }
        const std::string info_key = key(base_, p::kInfo);
        m.info_queryable = session.declare_queryable(
            zenoh::KeyExpr(info_key),
            [this, info_key](const zenoh::Query& query) { query.reply(zenoh::KeyExpr(info_key), info_record()); },
            zenoh::closures::none);
        m.alive.emplace(session.liveliness_declare_token(zenoh::KeyExpr(detail::alive_key(base_, m.host))));
    }

    CameraPublisher::~CameraPublisher() = default;  // Impl's queryable and token go first (its last members)

    ImuPublisher::ImuPublisher(zenoh::Session& session, const std::string& camera)
        : publisher_(declare(session, check_name(camera) + "/" + std::string(p::kImu), p::kImuType))
    {
    }

    bool ImuPublisher::subscribed() const
    {
        return publisher_.get_matching_status().matching;
    }

    void ImuPublisher::publish(const Imu& sample)
    {
        publisher_.put(zenoh::Bytes(sample.serialize()));
    }

    DiagnosticsPublisher::DiagnosticsPublisher(zenoh::Session& session, const std::string& camera)
        : publisher_(
              declare(session, check_name(camera) + "/" + std::string(p::kDiagnostics), p::kDiagnosticArrayType)),
          zid_(session.get_zid().to_string())
    {
    }

    void DiagnosticsPublisher::publish(const DiagnosticArray& diagnostics)
    {
        put(publisher_, diagnostics.serialize(), detail::attachment(++seq_, "", zid_));
    }

    MessagePublisher::MessagePublisher(zenoh::Session& session, const std::string& key,
                                       const protocol::MessageType& type)
        : publisher_(declare(session, check_name(key), type))
    {
    }

    void MessagePublisher::put(const std::vector<std::uint8_t>& payload)
    {
        publisher_.put(zenoh::Bytes(payload));
    }

    TransformPublisher::TransformPublisher(zenoh::Session& session, const std::string& camera)
        : publisher_(declare(session, check_name(camera) + "/" + std::string(p::kTf), p::kTFMessageType))
    {
    }

    void TransformPublisher::publish(const TFMessage& transforms)
    {
        publisher_.put(zenoh::Bytes(transforms.serialize()));
    }

    RobotStatePublisher::RobotStatePublisher(zenoh::Session& session, const std::string& robot)
        : session_(session),
          robot_(check_name(robot)),
          joint_states_(declare(session, robot_ + "/" + std::string(p::kJointStates), p::kJointStateType)),
          tf_(declare(session, robot_ + "/" + std::string(p::kTf), p::kTFMessageType)),
          odom_(declare(session, robot_ + "/" + std::string(p::kOdom), p::kOdometryType))
    {
    }

    void RobotStatePublisher::joint_states(const JointState& message)
    {
        joint_states_.put(zenoh::Bytes(message.serialize()));
    }

    void RobotStatePublisher::tf(const TFMessage& message)
    {
        tf_.put(zenoh::Bytes(message.serialize()));
    }

    void RobotStatePublisher::odom(const Odometry& message)
    {
        odom_.put(zenoh::Bytes(message.serialize()));
    }

    void RobotStatePublisher::wrench(const std::string& sensor, const WrenchStamped& message)
    {
        auto it = wrenches_.find(sensor);
        if (it == wrenches_.end())
        {
            const std::string key = check_name(robot_ + "/" + std::string(p::kWrench) + "/" + sensor);
            it = wrenches_.emplace(sensor, declare(session_, key, p::kWrenchStampedType)).first;
        }
        it->second.put(zenoh::Bytes(message.serialize()));
    }

    StaticTransformPublisher::StaticTransformPublisher(zenoh::Session& session, const std::string& camera,
                                                       TFMessage transforms)
        : key_(check_name(camera) + "/" + std::string(p::kTfStatic)), payload_(transforms.serialize())
    {
        // the queryable first: a subscriber that starts after the put asks for it
        queryable_ = session.declare_queryable(
            zenoh::KeyExpr(key_),
            [this](const zenoh::Query& query)
            {
                zenoh::Query::ReplyOptions options;
                options.encoding = zenoh::Encoding(p::kTFMessageType.encoding());
                query.reply(zenoh::KeyExpr(key_), zenoh::Bytes(payload_), std::move(options));
            },
            zenoh::closures::none);
        zenoh::Session::PutOptions options;
        options.encoding = zenoh::Encoding(p::kTFMessageType.encoding());
        session.put(zenoh::KeyExpr(key_), zenoh::Bytes(payload_), std::move(options));
    }

    struct StaticTransformSubscriber::Impl
    {
        std::string key;
        mutable std::mutex mutex;
        std::optional<TFMessage> transforms;
        std::exception_ptr error;
        std::uint64_t received = 0;
        std::optional<zenoh::Subscriber<void>> subscriber;
        std::atomic<bool> closed{false};
        std::thread asker;

        void deliver(const std::string& encoding, std::vector<std::uint8_t> payload)
        {
            std::lock_guard lock(mutex);
            try
            {
                if (encoding != p::kTFMessageType.encoding())
                {
                    throw ProtocolError(key + ": the publisher sends '" + encoding + "', this subscriber reads '" +
                                        p::kTFMessageType.encoding() + "'");
                }
                transforms = TFMessage::deserialize(payload);
                ++received;
            }
            catch (...)
            {
                error = std::current_exception();
            }
        }
    };

    StaticTransformSubscriber::StaticTransformSubscriber(zenoh::Session& session, const std::string& camera,
                                                         std::chrono::milliseconds timeout)
        : impl_(std::make_unique<Impl>())
    {
        Impl& m = *impl_;
        m.key = check_name(camera) + "/" + std::string(p::kTfStatic);
        Impl* self = &m;
        m.subscriber.emplace(session.declare_subscriber(
            zenoh::KeyExpr(m.key),
            [self](const zenoh::Sample& s) {
                self->deliver(s.get_encoding().as_string(), s.get_payload().as_vector());
            },
            zenoh::closures::none));
        m.asker = std::thread([self, &session, timeout] {
            const auto deadline = std::chrono::steady_clock::now() + timeout;
            while (not self->closed.load() and std::chrono::steady_clock::now() < deadline)
            {
                bool answered = false;
                for (const auto& reply : replies(session, self->key, std::chrono::milliseconds(500)))
                {
                    if (reply.is_ok())
                    {
                        answered = true;
                        self->deliver(reply.get_ok().get_encoding().as_string(),
                                      reply.get_ok().get_payload().as_vector());
                    }
                }
                if (answered)
                {
                    return;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        });
    }

    StaticTransformSubscriber::~StaticTransformSubscriber()
    {
        impl_->closed.store(true);
        if (impl_->asker.joinable())
        {
            impl_->asker.join();
        }
        impl_->subscriber.reset();
    }

    std::optional<TFMessage> StaticTransformSubscriber::latest() const
    {
        std::lock_guard lock(impl_->mutex);
        if (impl_->error)
        {
            std::rethrow_exception(impl_->error);
        }
        return impl_->transforms;
    }

    std::uint64_t StaticTransformSubscriber::received() const
    {
        std::lock_guard lock(impl_->mutex);
        return impl_->received;
    }

    namespace
    {
        // p_a = R p_b + t composed: (R1, t1) after (R2, t2)
        RigidTransform compose(const RigidTransform& a, const RigidTransform& b)
        {
            RigidTransform out;
            const auto& r1 = a.rotation;
            const auto& r2 = b.rotation;
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    out.rotation[3 * i + j] =
                        r1[3 * i] * r2[j] + r1[3 * i + 1] * r2[3 + j] + r1[3 * i + 2] * r2[6 + j];
                }
            }
            const Vector3& t2 = b.translation;
            out.translation = {r1[0] * t2.x + r1[1] * t2.y + r1[2] * t2.z + a.translation.x,
                               r1[3] * t2.x + r1[4] * t2.y + r1[5] * t2.z + a.translation.y,
                               r1[6] * t2.x + r1[7] * t2.y + r1[8] * t2.z + a.translation.z};
            return out;
        }

        RigidTransform inverse(const RigidTransform& a)
        {
            RigidTransform out;
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    out.rotation[3 * i + j] = a.rotation[3 * j + i];
                }
            }
            const Vector3& t = a.translation;
            const auto& r = out.rotation;
            out.translation = {-(r[0] * t.x + r[1] * t.y + r[2] * t.z), -(r[3] * t.x + r[4] * t.y + r[5] * t.z),
                               -(r[6] * t.x + r[7] * t.y + r[8] * t.z)};
            return out;
        }

        RigidTransform pose_of(const Transform& t)
        {
            const Quaternion& q = t.rotation;
            const double n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
            const double x = q.x / n, y = q.y / n, z = q.z / n, w = q.w / n;
            RigidTransform out;
            out.rotation = {1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w),
                            2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
                            2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)};
            out.translation = t.translation;
            return out;
        }

        // The pose of `frame` in the root of its tree (the frame without a parent), and that root.
        std::pair<RigidTransform, std::string> in_root(const TFMessage& transforms, const std::string& frame)
        {
            RigidTransform pose;
            std::string current = frame;
            for (std::size_t hops = 0; hops <= transforms.transforms.size(); ++hops)
            {
                const auto parent =
                    std::find_if(transforms.transforms.begin(), transforms.transforms.end(),
                                 [&](const TransformStamped& t) { return t.child_frame_id == current; });
                if (parent == transforms.transforms.end())
                {
                    return {pose, current};
                }
                pose = compose(pose_of(parent->transform), pose);
                current = parent->header.frame_id;
            }
            throw std::runtime_error("tf_static: the transforms of " + frame + " form a cycle");
        }
    }  // namespace

    std::optional<RigidTransform> lookup(const TFMessage& transforms, const std::string& parent,
                                         const std::string& child)
    {
        const auto [parent_pose, parent_root] = in_root(transforms, parent);
        const auto [child_pose, child_root] = in_root(transforms, child);
        if (parent_root != child_root)
        {
            return std::nullopt;
        }
        return compose(inverse(parent_pose), child_pose);
    }

    bool CameraPublisher::subscribed() const
    {
        const Impl& m = *impl_;
        return matching(m.raw) or matching(m.compressed) or matching(m.shm) or matching(m.cuda);
    }

    std::string CameraPublisher::info_record() const
    {
        const Impl& m = *impl_;
        const std::vector<std::string>& transports = m.offered;
        // the ring's GPU, also before a reader made the ring exist (a subscriber's local Auto compares it with its own)
        std::string cuda_uuid = m.rings.uuid();
        if (cuda_uuid.empty() and m.o.cuda_device)
        {
            cuda_uuid = cuda_device_uuid(*m.o.cuda_device);
        }
        const json::Value uuid = cuda_uuid.empty() ? json::Value(nullptr) : json::Value(cuda_uuid);
        return json::write({{"v", static_cast<std::int64_t>(p::kInfoVersion)},
                            {"kind", std::string("image")},
                            {"xblick", std::string(XBLICK_VERSION)},
                            {"host", m.host},
                            {"transports", transports},
                            {"encoding", m.o.encoding},
                            {"frame_id", m.o.frame_id},
                            {"source", m.o.source},
                            {"cuda_device_uuid", uuid},
                            {"relayed_from", m.o.relayed_from.empty() ? json::Value(nullptr)
                                                                       : json::Value(m.o.relayed_from)}});
    }

    std::vector<std::string> CameraPublisher::publish(const HostImage& image, std::optional<std::int64_t> stamp_ns)
    {
        return publish(image, stamp_ns, {});
    }

    std::vector<std::string> CameraPublisher::publish(const HostImage& image, std::optional<std::int64_t> stamp_ns,
                                                      std::span<const std::uint8_t> jpeg)
    {
        Impl& m = *impl_;
        m.check_jpeg(jpeg);
        const Header header{Time::from_ns(stamp_ns.value_or(now_ns())), m.o.frame_id};
        ++m.seq;
        return m.publish_host_frame(base_, image, header, m.attachment(), jpeg);
    }

    std::vector<std::string> CameraPublisher::publish(const HostImage& image, const Origin& origin)
    {
        Impl& m = *impl_;
        m.seq = origin.seq;
        return m.publish_host_frame(base_, image, origin.header, detail::frame_attachment(origin.seq, origin.source),
                                    {});
    }

    std::vector<std::string> CameraPublisher::Impl::publish_host_frame(const std::string& base_,
                                                                       const HostImage& image, const Header& header,
                                                                       const detail::FrameAttachment& attachment,
                                                                       std::span<const std::uint8_t> jpeg)
    {
        Impl& m = *this;
        std::vector<std::string> sent;
        if (matching(m.cuda))
        {
            const std::uint32_t row = image.width * static_cast<std::uint32_t>(m.enc.pixel_bytes());
            if (image.step != 0 and image.step != row)
            {
                throw EncodingError("frames for the CUDA ring need rows without padding");
            }
            const std::uint64_t nbytes = static_cast<std::uint64_t>(row) * image.height;
            const Written w = m.cuda_ring(nbytes).write_host(image.data, nbytes);
            m.publish_cuda_descriptor(w, image.height, image.width, header, attachment, sent);
        }
        m.publish_host(base_, image, header, attachment, sent, jpeg);
        m.publish_camera_info(base_, header, attachment, sent);
        return sent;
    }

    std::vector<std::string> CameraPublisher::publish(const DeviceImage& image, std::optional<std::int64_t> stamp_ns)
    {
        return publish(image, stamp_ns, {});
    }

    std::vector<std::string> CameraPublisher::publish(const DeviceImage& image, std::optional<std::int64_t> stamp_ns,
                                                      std::span<const std::uint8_t> jpeg)
    {
        Impl& m = *impl_;
        m.check_jpeg(jpeg);
        if (not m.o.cuda_device)
        {
            throw std::invalid_argument("a device image needs a publisher with cuda_device");
        }
        const Header header{Time::from_ns(stamp_ns.value_or(now_ns())), m.o.frame_id};
        ++m.seq;
        return m.publish_device_frame(base_, image, header, m.attachment(), jpeg);
    }

    std::vector<std::string> CameraPublisher::publish(const DeviceImage& image, const Origin& origin)
    {
        Impl& m = *impl_;
        if (not m.o.cuda_device)
        {
            throw std::invalid_argument("a device image needs a publisher with cuda_device");
        }
        m.seq = origin.seq;
        return m.publish_device_frame(base_, image, origin.header,
                                      detail::frame_attachment(origin.seq, origin.source), {});
    }

    std::vector<std::string> CameraPublisher::Impl::publish_device_frame(const std::string& base_,
                                                                         const DeviceImage& image,
                                                                         const Header& header,
                                                                         const detail::FrameAttachment& attachment,
                                                                         std::span<const std::uint8_t> jpeg)
    {
        Impl& m = *this;
        std::vector<std::string> sent;
        const std::uint64_t nbytes =
            static_cast<std::uint64_t>(image.width) * image.height * static_cast<std::uint64_t>(m.enc.pixel_bytes());
        CudaRingWriter& ring = m.cuda_ring(nbytes);
        const Written w = ring.write_device(image.ptr, nbytes, image.stream);
        if (matching(m.cuda))
        {
            m.publish_cuda_descriptor(w, image.height, image.width, header, attachment, sent);
        }
        if (matching(m.shm))
        {
            // one download, straight into the host ring's next slot; the other CPU representations read it there
            HostRingWriter& host_ring = m.rings.pinned_host_ring(m.o.slots, nbytes);
            std::uint8_t* slot = host_ring.loan();
            try
            {
                ring.read_host(w.slot, slot, nbytes);
            }
            catch (...)
            {
                host_ring.commit(0);
                throw;
            }
            const Written in_shm = host_ring.commit(nbytes);
            m.publish_host(base_, HostImage{slot, image.height, image.width, 0}, header, attachment, sent, jpeg,
                           &in_shm);
        }
        else if (matching(m.raw) or (matching(m.compressed) and jpeg.empty()))
        {
            std::vector<std::uint8_t> host(nbytes);  // one download for every CPU representation
            ring.read_host(w.slot, host.data(), nbytes);
            m.publish_host(base_, HostImage{host.data(), image.height, image.width, 0}, header, attachment, sent,
                           jpeg);
        }
        else if (matching(m.compressed))
        {
            put(*m.compressed, jpeg_message(jpeg, m.o.encoding, header).serialize(), attachment);
            sent.push_back(key(base_, p::kCompressed));
        }
        m.publish_camera_info(base_, header, attachment, sent);
        return sent;
    }

    bool CameraPublisher::needs_pixels() const
    {
        const Impl& m = *impl_;
        return matching(m.raw) or matching(m.shm) or matching(m.cuda);
    }

    std::vector<std::string> CameraPublisher::publish_jpeg(std::span<const std::uint8_t> jpeg,
                                                           std::optional<std::int64_t> stamp_ns)
    {
        Impl& m = *impl_;
        if (jpeg.empty())
        {
            throw std::invalid_argument("publish_jpeg needs the JPEG bytes");
        }
        m.check_jpeg(jpeg);
        const Header header{Time::from_ns(stamp_ns.value_or(now_ns())), m.o.frame_id};
        ++m.seq;
        const detail::FrameAttachment attachment = m.attachment();
        std::vector<std::string> sent;
        if (matching(m.compressed))
        {
            put(*m.compressed, jpeg_message(jpeg, m.o.encoding, header).serialize(), attachment);
            sent.push_back(key(base_, p::kCompressed));
        }
        m.publish_camera_info(base_, header, attachment, sent);
        return sent;
    }

    // ----------------------------------------------------------------------------------------- subscriber

    const char* to_string(Transport transport)
    {
        switch (transport)
        {
            case Transport::Auto: return "auto";
            case Transport::Compressed: return "compressed";
            case Transport::Raw: return "raw";
            case Transport::Shm: return "shm";
            case Transport::Cuda: return "cuda";
        }
        return "?";
    }

    namespace
    {
        const p::MessageType& type_of(Transport t)
        {
            switch (t)
            {
                case Transport::Compressed: return p::kCompressedImageType;
                case Transport::Raw: return p::kImageType;
                case Transport::Shm: return p::kShmImageType;
                case Transport::Cuda: return p::kCudaImageType;
                case Transport::Auto: break;
            }
            throw std::logic_error("no message type for auto");
        }

        std::string_view suffix_of(Transport t)
        {
            switch (t)
            {
                case Transport::Compressed: return p::kCompressed;
                case Transport::Raw: return p::kImageRaw;
                case Transport::Shm: return p::kShm;
                case Transport::Cuda: return p::kCuda;
                case Transport::Auto: break;
            }
            throw std::logic_error("no topic for auto");
        }
    }  // namespace

    namespace
    {
        std::atomic<std::uint64_t> not_finalized{0};
        std::mutex said_mutex;
        std::set<std::string> said;  // selectors said once
    }

    std::uint64_t queries_not_finalized()
    {
        return not_finalized.load();
    }

    std::vector<zenoh::Reply> replies(const zenoh::Session& session, const std::string& selector,
                                      std::chrono::milliseconds timeout,
                                      const std::optional<std::vector<std::uint8_t>>& payload)
    {
        struct State
        {
            std::mutex mutex;
            std::condition_variable cv;
            std::vector<zenoh::Reply> replies;
            bool done = false;
        };
        auto state = std::make_shared<State>();  // the callbacks may come after a deadline: they keep it
        zenoh::Session::GetOptions options;
        options.timeout_ms = static_cast<std::uint64_t>(timeout.count());
        // every reply, not one per key: several publishers answer a stream's info key (a source and its relays),
        // several nodes may answer one service key (a caller that expects one counts them)
        options.consolidation = zenoh::QueryConsolidation(Z_CONSOLIDATION_MODE_NONE);
        if (payload)
        {
            options.payload = zenoh::Bytes(*payload);
        }
        session.get(
            zenoh::KeyExpr(selector), "",
            [state](zenoh::Reply& reply) {
                std::lock_guard lock(state->mutex);
                state->replies.push_back(std::move(reply));
            },
            [state] {
                std::lock_guard lock(state->mutex);
                state->done = true;
                state->cv.notify_all();
            },
            std::move(options));
        const auto deadline = timeout + std::max(std::chrono::milliseconds(500), timeout);
        std::unique_lock lock(state->mutex);
        if (not state->cv.wait_for(lock, deadline, [&] { return state->done; }))
        {
            ++not_finalized;
            std::lock_guard said_lock(said_mutex);
            if (said.insert(selector).second)
            {
                std::fprintf(stderr, "xblick: query %s not finalized by Zenoh after %lld ms (its timeout %lld ms): "
                                     "going on with the %zu replies that came\n",
                             selector.c_str(), static_cast<long long>(deadline.count()),
                             static_cast<long long>(timeout.count()), state->replies.size());
            }
        }
        return std::move(state->replies);
    }

    std::vector<std::string> query_info(zenoh::Session& session, const std::string& base,
                                        std::chrono::milliseconds timeout)
    {
        std::vector<std::string> records;
        for (const auto& reply : replies(session, key(check_name(base), p::kInfo), timeout))
        {
            if (reply.is_ok())
            {
                records.push_back(reply.get_ok().get_payload().as_string());
            }
        }
        return records;
    }

    bool StreamInfo::offers(const std::string& transport) const
    {
        return std::find(transports.begin(), transports.end(), transport) != transports.end();
    }

    std::vector<StreamInfo> stream_infos(zenoh::Session& session, const std::string& base,
                                         std::chrono::milliseconds timeout)
    {
        std::vector<StreamInfo> out;
        for (const std::string& text : query_info(session, base, timeout))
        {
            const json::Object record = json::parse(text);
            if (json::get<std::int64_t>(record, "v") != static_cast<std::int64_t>(p::kInfoVersion))
            {
                throw ProtocolError(base + ": info record " + text.substr(0, 120) + " is not of version " +
                                    std::to_string(p::kInfoVersion) + "; run the same Durchblick version on both sides");
            }
            StreamInfo s;
            s.kind = json::get<std::string>(record, "kind").value_or("");
            s.xblick = json::get<std::string>(record, "xblick").value_or("");
            s.host = json::get<std::string>(record, "host").value_or("");
            s.transports = json::get<std::vector<std::string>>(record, "transports").value_or(std::vector<std::string>{});
            s.encoding = json::get<std::string>(record, "encoding").value_or("");
            s.frame_id = json::get<std::string>(record, "frame_id").value_or("");
            s.source = json::get<std::string>(record, "source").value_or("");
            s.cuda_device_uuid = json::get<std::string>(record, "cuda_device_uuid").value_or("");
            s.relayed_from = json::get<std::string>(record, "relayed_from").value_or("");
            out.push_back(std::move(s));
        }
        return out;
    }

    FrameMeta parse_frame_attachment(const std::string& attachment)
    {
        detail::Item item;
        item.attachment = attachment;
        item.key = "attachment";
        const detail::Meta meta = detail::parse_attachment(item);
        return FrameMeta{meta.seq, meta.publish_ns, meta.source};
    }

    struct ImageSubscriber::Impl
    {
        SubscriberOptions o;
        detail::Subscription subscription;
        detail::RingReaders rings;
        std::atomic<std::uint64_t> stale{0};
        std::atomic<std::uint64_t> decoded{0};
        std::shared_ptr<detail::Latest> info_latest;
        std::optional<zenoh::Subscriber<void>> info_subscriber;

        Impl(zenoh::Session& session, const std::string& base, SubscriberOptions options)
            : o(options), subscription(session, base, "image", o.transport, o.local, o.remote, suffix_of, o.cuda_device)
        {
        }
    };

    ImageSubscriber::ImageSubscriber(zenoh::Session& session, const std::string& base, SubscriberOptions options)
    {
        if (options.remote != Transport::Compressed and options.remote != Transport::Raw)
        {
            throw std::invalid_argument("remote: Compressed or Raw");
        }
        if (options.local != Transport::Auto and options.local != Transport::Shm and options.local != Transport::Cuda)
        {
            throw std::invalid_argument("local: Auto, Shm or Cuda");
        }
        if (not options.copy and (options.transport == Transport::Compressed or options.transport == Transport::Raw))
        {
            throw std::invalid_argument("copy = false is for the shm and cuda transports (and auto)");
        }
        impl_ = std::make_unique<Impl>(session, check_name(base), options);
        Impl& m = *impl_;
        m.subscription.limit_rate(options.max_hz);
        m.subscription.keep_recent(options.keep_recent);
        if (options.camera_info)
        {
            m.info_latest = std::make_shared<detail::Latest>();
            auto target = m.info_latest;
            m.info_subscriber.emplace(session.declare_subscriber(
                zenoh::KeyExpr(key(m.subscription.base(), p::kCameraInfo)),
                [target](const zenoh::Sample& s) { target->put(s, Transport::Auto, 0); }, zenoh::closures::none));
        }
    }

    ImageSubscriber::~ImageSubscriber()
    {
        impl_->info_subscriber.reset();
    }

    std::optional<Transport> ImageSubscriber::transport() const
    {
        return impl_->subscription.transport();
    }

    std::string ImageSubscriber::transport_reason() const
    {
        return impl_->subscription.transport_reason();
    }

    std::uint64_t ImageSubscriber::received() const
    {
        return impl_->subscription.received();
    }

    std::uint64_t ImageSubscriber::overwritten() const
    {
        return impl_->subscription.overwritten();
    }

    std::uint64_t ImageSubscriber::stale() const
    {
        return impl_->stale.load();
    }

    std::uint64_t ImageSubscriber::skipped_max_hz() const
    {
        return impl_->subscription.skipped_max_hz();
    }

    std::optional<CameraInfo> ImageSubscriber::camera_info() const
    {
        if (not impl_->info_latest)
        {
            throw std::logic_error("this subscriber was created without camera_info = true");
        }
        auto item = impl_->info_latest->take(false, std::nullopt);
        if (not item)
        {
            return std::nullopt;
        }
        detail::check_encoding(*item, p::kCameraInfoType);
        return CameraInfo::deserialize(item->payload);
    }

    std::optional<Frame> ImageSubscriber::latest(bool consume, std::optional<std::chrono::milliseconds> timeout)
    {
        Impl& m = *impl_;
        const auto taken = m.subscription.take(consume, timeout);
        if (not taken)
        {
            return std::nullopt;
        }
        return decode(taken->item, taken->transport);
    }

    std::optional<Frame> ImageSubscriber::at(std::int64_t stamp_ns, std::chrono::nanoseconds max_offset)
    {
        const auto recent = impl_->subscription.recent();
        if (not recent)
        {
            return std::nullopt;
        }
        std::shared_ptr<const detail::Item> best;
        std::int64_t best_offset = 0;
        for (const auto& item : recent->first)
        {
            const std::int64_t offset = std::abs(detail::stamp_of(*item) - stamp_ns);
            if (offset <= max_offset.count() and (not best or offset < best_offset))
            {
                best = item;
                best_offset = offset;
            }
        }
        if (not best)
        {
            return std::nullopt;
        }
        return decode(best, best->transport);
    }

    std::vector<std::int64_t> ImageSubscriber::recent_stamps() const
    {
        std::vector<std::int64_t> stamps;
        if (const auto recent = impl_->subscription.recent())
        {
            for (const auto& item : recent->first)
            {
                stamps.push_back(detail::stamp_of(*item));
            }
        }
        return stamps;
    }

    std::uint64_t ImageSubscriber::decoded() const
    {
        return impl_->decoded.load();
    }

    std::optional<Frame> ImageSubscriber::decode(const std::shared_ptr<const detail::Item>& held, Transport t)
    {
        Impl& m = *impl_;
        const detail::Item& item = *held;
        detail::check_encoding(item, type_of(t));

        Frame f;
        f.key = item.key;
        f.receive_ns = item.receive_ns;
        const detail::Meta meta = detail::parse_attachment(item);
        f.seq = meta.seq;
        f.publish_ns = meta.publish_ns;
        f.source = meta.source;

        if (t == Transport::Compressed)
        {
            const CompressedImage message = CompressedImage::deserialize(item.payload);
            DecodedImage d = decompress(message);
            f.header = message.header;
            f.encoding = d.encoding;
            f.height = d.height;
            f.width = d.width;
            f.step = d.step;
            f.pixels = std::move(d.pixels);
            ++m.decoded;
            return f;
        }
        if (t == Transport::Raw)
        {
            const Image message = Image::deserialize(item.payload);
            if (message.is_bigendian)
            {
                throw EncodingError(f.key + ": big-endian images are not supported in C++");
            }
            const EncodingInfo enc = encoding_info(message.encoding);
            if (message.step < message.width * static_cast<std::uint32_t>(enc.pixel_bytes()) or
                message.data.size() < static_cast<std::size_t>(message.step) * message.height)
            {
                throw EncodingError(f.key + ": the image's data does not fit its size and step");
            }
            f.header = message.header;
            f.encoding = message.encoding;
            f.height = message.height;
            f.width = message.width;
            f.step = message.step;
            f.view = message.data.data();  // into the payload, which the frame keeps
            f.owner = held;
            ++m.decoded;
            return f;
        }

        // ring transports: the descriptor says where the frame is
        RingImage* d = nullptr;
        ShmImage shm;
        CudaImage cuda;
        if (t == Transport::Shm)
        {
            shm = ShmImage::deserialize(item.payload);
            d = &shm;
        }
        else
        {
            cuda = CudaImage::deserialize(item.payload);
            d = &cuda;
        }
        if (d->host != m.subscription.host())
        {
            m.subscription.moved(d->host, t);
            return std::nullopt;
        }
        const EncodingInfo enc = encoding_info(d->encoding);
        const std::uint32_t row = d->width * static_cast<std::uint32_t>(enc.pixel_bytes());
        if (d->step != row or d->is_bigendian)
        {
            throw std::runtime_error(m.subscription.base() + ": ring frames have little-endian rows without padding");
        }
        f.header = d->header;
        f.encoding = d->encoding;
        f.height = d->height;
        f.width = d->width;
        f.step = d->step;
        const std::uint64_t nbytes = static_cast<std::uint64_t>(row) * d->height;
        std::optional<detail::RingData> data = t == Transport::Shm
                                                   ? m.rings.host(shm.segment, d->slot, d->seq, nbytes, m.o.copy)
                                                   : m.rings.cuda(cuda.endpoint, d->slot, d->seq, m.o.copy, m.o.cuda_device);
        if (not data)
        {
            ++m.stale;
            return std::nullopt;
        }
        f.pixels = std::move(data->bytes);
        f.view = data->view;
        f.device_ptr = data->device_ptr;
        f.device = data->device;
        f.owner = std::move(data->owner);
        f.valid = std::move(data->valid);
        ++m.decoded;
        return f;
    }
}  // namespace xblick
