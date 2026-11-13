#include "stream.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <random>
#include <stdexcept>

#include "durchblick/version.hpp"
#include "json.hpp"

namespace durchblick::detail
{
    namespace p = protocol;

    std::int64_t now_ns()
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::system_clock::now().time_since_epoch())
            .count();
    }

    std::string key(const std::string& base, std::string_view suffix)
    {
        return base + "/" + std::string(suffix);
    }

    std::string ring_key(const std::string& base, std::string_view suffix, const std::string& host)
    {
        return key(base, suffix) + "/" + host;
    }

    std::string ring_key(const std::string& base, std::string_view suffix, const std::string& host,
                         std::string_view gpu)
    {
        return ring_key(base, suffix, host) + "/" + std::string(gpu);
    }

    namespace
    {
        std::string random_id()  // 16 hex digits
        {
            static const char* digits = "0123456789abcdef";
            std::random_device random;
            std::string id;
            for (int i = 0; i < 4; ++i)
            {
                const std::uint32_t value = random();
                for (int shift = 28; shift >= 0; shift -= 4)
                {
                    id += digits[(value >> shift) & 0xF];
                }
            }
            return id;
        }
    }  // namespace

    std::string alive_key(const std::string& base, const std::string& host)
    {
        return key(base, p::kInfo) + "/" + host + "/" + random_id();
    }

    std::string want_key(const std::string& base, const std::string& host)
    {
        return key(base, p::kWant) + "/" + host + "/" + random_id();
    }

    std::string ring_gpu(const std::string& base, int cuda_device)
    {
        const std::string uuid = cuda_device_uuid(cuda_device);
        if (uuid.empty())
        {
            throw std::runtime_error(base + ": a publisher with cuda_device " + std::to_string(cuda_device) +
                                     " needs the CUDA driver (libcuda.so.1)");
        }
        return uuid;
    }

    zenoh::Publisher declare(zenoh::Session& session, const std::string& key_expr, const p::MessageType& type)
    {
        zenoh::Session::PublisherOptions options;
        options.encoding = zenoh::Encoding(type.encoding());
        options.congestion_control = Z_CONGESTION_CONTROL_DROP;  // latest-value data: drop, never queue
        return session.declare_publisher(zenoh::KeyExpr(key_expr), std::move(options));
    }

    bool matching(const std::optional<zenoh::Publisher>& publisher)
    {
        return publisher.has_value() and publisher->get_matching_status().matching;
    }

    void put(zenoh::Publisher& publisher, std::vector<std::uint8_t>&& payload, const std::string& attachment)
    {
        zenoh::Publisher::PutOptions options;
        options.attachment = zenoh::Bytes(attachment);
        publisher.put(zenoh::Bytes(std::move(payload)), std::move(options));
    }

    std::string attachment(std::int64_t seq, const std::string& source)
    {
        return json::write({{"v", static_cast<std::int64_t>(p::kAttachmentVersion)},
                            {"seq", seq},
                            {"publish_ns", now_ns()},
                            {"source", source}});
    }

    std::string attachment(std::int64_t seq, const std::string& source, const std::string& zid)
    {
        return json::write({{"v", static_cast<std::int64_t>(p::kAttachmentVersion)},
                            {"seq", seq},
                            {"publish_ns", now_ns()},
                            {"source", source},
                            {"zid", zid}});
    }

    std::string FrameAttachment::now() const
    {
        return json::write({{"v", static_cast<std::int64_t>(p::kAttachmentVersion)},
                            {"seq", seq},
                            {"publish_ns", now_ns()},
                            {"source", source},
                            {"publish_called_ns", called_ns}});
    }

    FrameAttachment frame_attachment(std::int64_t seq, const std::string& source)
    {
        return FrameAttachment{seq, source, now_ns()};
    }

    void put(zenoh::Publisher& publisher, std::vector<std::uint8_t>&& payload, const FrameAttachment& attachment)
    {
        put(publisher, std::move(payload), attachment.now());
    }

    // ----------------------------------------------------------------------------------------------- Latest

    void Latest::move(int to)
    {
        std::lock_guard lock(mutex);
        if (route != 0)
        {
            old = route;
        }
        route = to;
    }

    bool Latest::wait_moved(std::chrono::milliseconds timeout)
    {
        std::unique_lock lock(mutex);
        return cv.wait_for(lock, timeout, [&] { return not old.has_value(); });
    }

    void Latest::leave_old()
    {
        std::lock_guard lock(mutex);
        leave_old_locked();
    }

    void Latest::leave_old_locked()
    {
        if (old)
        {
            old.reset();
            compare_until = std::chrono::steady_clock::now() + kHandover;
            cv.notify_all();
        }
    }

    void Latest::put(const zenoh::Sample& sample, Transport transport, int from)
    {
        auto made = std::make_shared<Item>();
        Item& i = *made;
        i.transport = transport;
        i.payload = sample.get_payload().as_vector();
        if (auto a = sample.get_attachment())
        {
            i.attachment = a->get().as_string();
        }
        i.key = std::string(sample.get_keyexpr().as_string_view());
        i.encoding = sample.get_encoding().as_string();
        i.receive_ns = now_ns();
        offer(std::move(made), from);
    }

    void Latest::offer(std::shared_ptr<Item> made, int from)
    {
        // a sample without a header is refused when it is read
        const std::optional<std::int64_t> stamp =
            made->payload.size() >= 12 ? std::optional<std::int64_t>(stamp_of(*made)) : std::nullopt;
        std::lock_guard lock(mutex);
        if (from != route and from != old)
        {
            return;  // of a route left already: its frames come by the new route
        }
        if (from == route)
        {
            leave_old_locked();
        }
        const bool compare = old or (compare_until and std::chrono::steady_clock::now() < *compare_until);
        constexpr std::int64_t window = std::chrono::duration_cast<std::chrono::nanoseconds>(kHandover).count();
        // a restart (far back) only by the route that is kept: the old one's frames come again by the new one
        if (compare and stamp and newest_stamp and *stamp <= *newest_stamp and
            (from != route or *newest_stamp - window <= *stamp))
        {
            ++duplicates;  // the same frame by the other route
            return;
        }
        if (from == route)
        {
            // newer than the frames of the old route, or a restart: none of its later samples is one of them
            compare_until.reset();
        }
        newest_stamp = stamp;
        if (fresh and keep == 0)  // with keep a replaced sample stays in recent, available to at(): not lost
        {
            ++overwritten;
        }
        if (keep > 0)
        {
            recent.push_back(made);
            while (recent.size() > keep)
            {
                recent.pop_front();
            }
        }
        item = std::move(made);
        fresh = true;
        ++received;
        cv.notify_all();
    }

    std::int64_t stamp_of(const Item& item)
    {
        if (item.payload.size() < 12)
        {
            throw ProtocolError(item.key + ": a sample of " + std::to_string(item.payload.size()) +
                                " bytes has no header");
        }
        std::int32_t sec;
        std::uint32_t nanosec;
        std::memcpy(&sec, item.payload.data() + 4, 4);
        std::memcpy(&nanosec, item.payload.data() + 8, 4);
        return static_cast<std::int64_t>(sec) * 1'000'000'000 + nanosec;
    }

    std::shared_ptr<const Item> Latest::take(bool consume, std::optional<std::chrono::milliseconds> timeout)
    {
        std::unique_lock lock(mutex);
        if (timeout and not fresh)
        {
            cv.wait_for(lock, *timeout, [this] { return fresh; });
        }
        if (not item or (consume and not fresh))
        {
            return nullptr;
        }
        if (consume)
        {
            fresh = false;
        }
        return item;
    }

    void check_encoding(const Item& item, const p::MessageType& type)
    {
        if (item.encoding != type.encoding())
        {
            throw ProtocolError(item.key + ": the publisher sends '" + item.encoding + "', this subscriber reads '" +
                                type.encoding() + "' (Durchblick " DURCHBLICK_VERSION
                                "); run the same Durchblick version on both sides");
        }
    }

    Meta parse_attachment(const Item& item)
    {
        Meta meta;
        if (item.attachment.empty())
        {
            return meta;
        }
        const json::Object o = json::parse(item.attachment);
        if (json::get<std::int64_t>(o, "v") != static_cast<std::int64_t>(p::kAttachmentVersion))
        {
            throw ProtocolError(item.key + ": attachment " + item.attachment.substr(0, 80) +
                                " is not a Durchblick attachment of version " + std::to_string(p::kAttachmentVersion));
        }
        meta.seq = json::get<std::int64_t>(o, "seq");
        meta.publish_ns = json::get<std::int64_t>(o, "publish_ns");
        meta.source = json::get<std::string>(o, "source").value_or("");
        return meta;
    }

    // ----------------------------------------------------------------------------------------- Subscription

    namespace
    {
        constexpr int kSettleRounds = 3;  // info queries after a publisher came or went (its queryable can lag)
    }

    Subscription::Subscription(zenoh::Session& session, std::string base, std::string kind, Transport requested,
                               Transport local, Transport remote, Suffix suffix, std::optional<int> cuda_device,
                               std::optional<Transport> remote_else)
        : session_(session),
          base_(std::move(base)),
          kind_(std::move(kind)),
          requested_(requested),
          local_(local),
          remote_(remote),
          remote_else_(remote_else),
          suffix_(std::move(suffix)),
          cuda_device_(cuda_device),
          host_(host_id())
    {
        if (requested_ == Transport::Auto)
        {
            zenoh::Session::LivelinessSubscriberOptions options;
            options.history = true;
            alive_.emplace(session_.liveliness_declare_subscriber(
                zenoh::KeyExpr(key(base_, p::kInfo) + "/**"), [this](const zenoh::Sample&) { recheck(); },
                zenoh::closures::none, std::move(options)));
            chooser_ = std::thread([this] { choose(); });
        }
        else
        {
            use(requested_, std::string(p::kAnyGpu));
        }
    }

    Subscription::~Subscription()
    {
        closed_.store(true);
        recheck();
        if (chooser_.joinable())
        {
            chooser_.join();
        }
        alive_.reset();
        want_.reset();
        std::lock_guard lock(mutex_);
        subscriber_.reset();
    }

    void Subscription::recheck()
    {
        {
            std::lock_guard lock(recheck_mutex_);
            recheck_ = true;
        }
        recheck_cv_.notify_all();
    }

    std::string Subscription::key_of(Transport t, std::string_view gpu) const
    {
        if (t == Transport::Shm)
        {
            return ring_key(base_, suffix_(t), host_);
        }
        if (t == Transport::Cuda)
        {
            return ring_key(base_, suffix_(t), host_, gpu);
        }
        return key(base_, suffix_(t));
    }

    // Reads the stream by `t` from now on. With a transport in use already, both are read until the new one delivered
    // its first frame (at most kHandover: a stream that sends nothing loses nothing), each frame handed out once.
    void Subscription::use(Transport t, const std::string& gpu)
    {
        std::optional<zenoh::Subscriber<void>> old;
        int route = 0;
        {
            std::lock_guard lock(mutex_);
            route = ++route_;
            if (subscriber_)
            {
                old.emplace(std::move(*subscriber_));
                subscriber_.reset();
            }
            latest_->move(route);
            auto target = latest_;
            subscriber_.emplace(session_.declare_subscriber(
                zenoh::KeyExpr(key_of(t, gpu)), [target, t, route](const zenoh::Sample& s) { target->put(s, t, route); },
                zenoh::closures::none));
            transport_ = t;
            using_ = std::make_pair(t, gpu);
        }
        chosen_cv_.notify_all();
        if (not old)
        {
            return;
        }
        const auto end = std::chrono::steady_clock::now() + kHandover;
        while (not closed_.load() and std::chrono::steady_clock::now() < end and
               not latest_->wait_moved(std::chrono::milliseconds(50)))
        {
        }
        // before the subscriber goes: Zenoh can still hand over a sample of the old one while it is undeclared
        latest_->leave_old();
        old.reset();
    }

    // The chooser's thread: chooses when a publisher is found and again whenever one comes or goes (recheck()).
    // Without a publisher it asks every 0.5 s (a publisher without a liveliness token is found this way).
    void Subscription::choose()
    {
        try
        {
            int settle = 0;
            while (not closed_.load())
            {
                {
                    std::lock_guard lock(recheck_mutex_);
                    recheck_ = false;
                }
                bool found = false;
                bool local = false;
                bool cuda = false;
                bool remote_missing = false;  // a source (not a relay) offers no `remote_` form
                bool source_ring_here = false;  // a source (not a relay) has a ring on this host
                std::string remote_host;
                std::string ring_uuid;  // the GPU of the publisher's CUDA ring
                for (const std::string& text : query_info(session_, base_))
                {
                    const json::Object record = json::parse(text);
                    if (json::get<std::int64_t>(record, "v") != static_cast<std::int64_t>(p::kInfoVersion))
                    {
                        throw ProtocolError(base_ + ": info record " + text.substr(0, 120) + " is not of version " +
                                            std::to_string(p::kInfoVersion) +
                                            "; run the same Durchblick version on both sides");
                    }
                    const auto kind = json::get<std::string>(record, "kind");
                    if (kind != kind_)
                    {
                        throw ProtocolError(base_ + " is a stream of kind '" + kind.value_or("") + "', this subscriber "
                                            "reads '" + kind_ + "'");
                    }
                    found = true;
                    const auto transports =
                        json::get<std::vector<std::string>>(record, "transports").value_or(std::vector<std::string>{});
                    const auto offers = [&](const char* t) {
                        return std::find(transports.begin(), transports.end(), t) != transports.end();
                    };
                    if (json::get<std::string>(record, "host") == host_ and offers("shm"))
                    {
                        local = true;
                        cuda = offers("cuda");
                        ring_uuid = json::get<std::string>(record, "cuda_device_uuid").value_or("");
                        source_ring_here =
                            source_ring_here or json::get<std::string>(record, "relayed_from").value_or("").empty();
                    }
                    // a relay (durchblick-relay) offers only its own host's rings: the network forms are the source's
                    if (json::get<std::string>(record, "relayed_from").value_or("").empty() and
                        not offers(to_string(remote_)))
                    {
                        remote_missing = true;
                        remote_host = json::get<std::string>(record, "host").value_or("");
                    }
                }
                if (found and not closed_.load())
                {
                    // the want token while the stream has no source with a ring on this host: this host's relay
                    // (durchblick-relay, where one runs) then brings the stream into rings here
                    if (not source_ring_here and not want_)
                    {
                        want_.emplace(session_.liveliness_declare_token(zenoh::KeyExpr(want_key(base_, host_))));
                    }
                    else if (source_ring_here and want_)
                    {
                        want_.reset();
                    }
                    std::string why;
                    Transport t = remote_;
                    if (not local)
                    {
                        if (remote_missing and remote_else_)
                        {
                            t = *remote_else_;
                        }
                        else if (remote_missing)
                        {
                            // as Python's ImageSubscriber: say so instead of waiting for frames that never come
                            throw ProtocolError(base_ + ": its publisher on host " + remote_host.substr(0, 8) +
                                                " offers no " + to_string(remote_) + " transport (a stream without "
                                                "that form: subscribe with " +
                                                (remote_ == Transport::Compressed ? "raw" : "compressed") + ")");
                        }
                        why = "publisher on another host";
                    }
                    else if (local_ == Transport::Shm)
                    {
                        t = Transport::Shm, why = "asked for";
                    }
                    else if (local_ == Transport::Cuda)  // the CUDA ring where the publisher offers one
                    {
                        t = cuda ? Transport::Cuda : Transport::Shm;
                        why = cuda ? "asked for" : "publisher offers no CUDA ring";
                    }
                    else  // Auto: the CUDA ring if this process sees its GPU
                    {
                        const std::string mine = cuda and cuda_device_ ? cuda_device_uuid(*cuda_device_) : "";
                        // the ring on another GPU of this host: copied from GPU to GPU if this process sees that GPU
                        const bool same = not mine.empty() and mine == ring_uuid;
                        const bool reachable = not mine.empty() and (same or cuda_device_ordinal(ring_uuid) >= 0);
                        t = cuda and reachable ? Transport::Cuda : Transport::Shm;
                        why = not cuda_device_  ? "this subscriber works on the CPU"
                              : not cuda        ? "publisher offers no CUDA ring"
                              : mine.empty()    ? "no CUDA driver here"
                              : same            ? "publisher on this GPU"
                              : reachable       ? "copied from the publisher's GPU " + ring_uuid.substr(0, 8)
                                                : "publisher's ring is on GPU " + ring_uuid.substr(0, 8) +
                                                      ", which this process does not see";
                    }
                    const std::string gpu =
                        t == Transport::Cuda and not ring_uuid.empty() ? ring_uuid : std::string(p::kAnyGpu);
                    if (not using_ or *using_ != std::make_pair(t, gpu))
                    {
                        {
                            std::lock_guard lock(mutex_);
                            reason_ = std::string(to_string(t)) + " (" + why + ")";
                        }
                        use(t, gpu);
                    }
                }
                std::unique_lock lock(recheck_mutex_);
                const auto woken = [this] { return recheck_ or closed_.load(); };
                bool event = true;
                if (not found or settle > 0)
                {
                    event = recheck_cv_.wait_for(lock, std::chrono::milliseconds(500), woken);
                }
                else
                {
                    recheck_cv_.wait(lock, woken);
                }
                if (event)
                {
                    settle = kSettleRounds;
                }
                else if (settle > 0)
                {
                    --settle;
                }
            }
        }
        catch (...)
        {
            std::lock_guard lock(mutex_);
            error_ = std::current_exception();
            chosen_cv_.notify_all();
        }
    }

    std::optional<Subscription::Taken> Subscription::take(bool consume,
                                                          std::optional<std::chrono::milliseconds> timeout)
    {
        const auto start = std::chrono::steady_clock::now();
        std::shared_ptr<Latest> source;
        {
            std::unique_lock lock(mutex_);
            const auto ready = [this] { return transport_.has_value() or error_ != nullptr; };
            if (not ready())
            {
                if (not timeout or not chosen_cv_.wait_for(lock, *timeout, ready))
                {
                    return std::nullopt;
                }
            }
            if (error_)
            {
                std::rethrow_exception(error_);
            }
            source = latest_;
        }
        while (true)
        {
            std::optional<std::chrono::milliseconds> remaining;
            if (timeout)
            {
                const auto spent =
                    std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start);
                remaining = std::max(std::chrono::milliseconds(0), *timeout - spent);
            }
            std::shared_ptr<const Item> item = source->take(consume, remaining);
            if (not item)
            {
                return std::nullopt;
            }
            if (consume and period_ns_ > 0)
            {
                const std::int64_t period = stamp_of(*item) / period_ns_;  // the stamp, not decoded
                if (last_period_ == period)
                {
                    ++skipped_max_hz_;  // not decoded, its ring not read: the next within the timeout
                    continue;
                }
                last_period_ = period;
            }
            const Transport t = item->transport;
            return Taken{std::move(item), t};
        }
    }

    void Subscription::keep_recent(std::size_t n)
    {
        std::lock_guard lock(mutex_);
        keep_ = n;
        std::lock_guard inner(latest_->mutex);
        latest_->keep = n;
    }

    std::optional<std::pair<std::vector<std::shared_ptr<const Item>>, Transport>> Subscription::recent() const
    {
        std::lock_guard lock(mutex_);
        if (error_)
        {
            std::rethrow_exception(error_);
        }
        if (not transport_)
        {
            return std::nullopt;
        }
        std::lock_guard inner(latest_->mutex);
        return std::make_pair(std::vector<std::shared_ptr<const Item>>(latest_->recent.begin(), latest_->recent.end()),
                              *transport_);
    }

    void Subscription::limit_rate(double max_hz)
    {
        if (max_hz < 0 or std::isnan(max_hz))
        {
            throw std::invalid_argument("max_hz " + std::to_string(max_hz) + ": a rate above 0, or 0 for every frame");
        }
        period_ns_ = max_hz > 0 ? static_cast<std::int64_t>(std::llround(1e9 / max_hz)) : 0;
    }

    bool Subscription::moved(const std::string& host, Transport transport)
    {
        if (requested_ == Transport::Auto)  // cannot be by the rings' keys (they name the host): choose again
        {
            recheck();
            return true;
        }
        throw std::runtime_error(base_ + ": the data is in a ring on host " + host + ", this is " + host_ + "; the " +
                                 to_string(transport) + " transport works on one host only");
    }

    std::optional<Transport> Subscription::transport() const
    {
        std::lock_guard lock(mutex_);
        return transport_;
    }

    std::string Subscription::transport_reason() const
    {
        std::lock_guard lock(mutex_);
        return reason_;
    }

    std::uint64_t Subscription::received() const
    {
        std::lock_guard lock(mutex_);
        std::lock_guard inner(latest_->mutex);
        return latest_->received;
    }

    std::uint64_t Subscription::overwritten() const
    {
        std::lock_guard lock(mutex_);
        std::lock_guard inner(latest_->mutex);
        return latest_->overwritten;
    }

    // ------------------------------------------------------------------------------------------ RingReaders

    std::optional<RingData> RingReaders::host(const std::string& segment, std::uint32_t slot, std::uint64_t seq,
                                              std::uint64_t nbytes, bool copy)
    {
        if (segment != host_name_)  // first frame, or the publisher was restarted with a new ring
        {
            try
            {
                host_ring_ = std::make_shared<HostRingReader>(segment);
            }
            catch (const RingError&)
            {
                // a descriptor that came before its publisher stopped, read after it removed its ring (a camera's
                // restart): the frame is stale (counted), not an error that ends the reader
                return std::nullopt;
            }
            host_name_ = segment;
        }
        auto ring = host_ring_;
        RingData d;
        if (copy)
        {
            d.bytes.resize(nbytes);
            if (not ring->read(slot, seq, d.bytes.data(), nbytes))
            {
                return std::nullopt;
            }
            return d;
        }
        if (not ring->acquire(slot, seq))
        {
            return std::nullopt;
        }
        d.view = ring->view(slot);
        d.owner = ring;
        d.valid = [ring, slot, seq] { return ring->validate(slot, seq); };
        return d;
    }

    std::optional<RingData> RingReaders::cuda(const std::string& endpoint, std::uint32_t slot, std::uint64_t seq,
                                              bool copy, std::optional<int> target)
    {
        if (endpoint != cuda_name_)
        {
            try
            {
                cuda_ring_ = std::make_shared<CudaRingReader>(endpoint, target.value_or(-1));
            }
            catch (const RingError&)
            {
                return std::nullopt;  // its publisher stopped (as host())
            }
            cuda_name_ = endpoint;
        }
        auto ring = cuda_ring_;
        RingData d;
        d.device = ring->device();
        // a ring on another device than the subscriber's: a view of it would be on the wrong device, so a copy
        if (copy or ring->device() != ring->ring_device())
        {
            std::shared_ptr<DeviceBuffer> buffer = ring->copy(slot, seq);
            if (not buffer)
            {
                return std::nullopt;
            }
            d.device_ptr = buffer->ptr();
            d.owner = buffer;
            return d;
        }
        if (not ring->acquire(slot, seq))
        {
            return std::nullopt;
        }
        ring->wait(slot, 0);  // the legacy default stream waits for the slot's copy; blocking streams follow it
        d.device_ptr = ring->slot_ptr(slot);
        d.owner = ring;
        d.valid = [ring, slot, seq] { return ring->validate(slot, seq); };
        return d;
    }

    // ------------------------------------------------------------------------------------------ RingWriters

    // The rings are made with the first frame and anew for a larger one (a camera back with another size): readers
    // follow the new ring by its name.
    HostRingWriter& RingWriters::host_ring(std::uint32_t slots, std::uint64_t nbytes)
    {
        if (not host or nbytes > host->slot_size())
        {
            host = std::make_unique<HostRingWriter>(slots, nbytes);
        }
        return *host;
    }

    CudaRingWriter& RingWriters::cuda_ring(int device, std::uint32_t slots, std::uint64_t nbytes)
    {
        if (not cuda or nbytes > cuda->slot_size())
        {
            cuda = std::make_unique<CudaRingWriter>(device, slots, nbytes);
            std::lock_guard lock(uuid_mutex);
            cuda_uuid = cuda->device_uuid();
        }
        return *cuda;
    }

    HostRingWriter& RingWriters::pinned_host_ring(std::uint32_t slots, std::uint64_t nbytes)
    {
        if (not host or nbytes > host->slot_size())
        {
            host = std::make_unique<HostRingWriter>(slots, nbytes);
            // page-locked: the GPU writes the frames into it directly (read_host of a 4K RGB frame, RTX PRO 6000:
            // 0.7 ms instead of 2.6 ms)
            cuda->pin_host(host->slots().data(), host->slots().size());
        }
        if (nbytes > host->slot_size())
        {
            throw RingError(std::to_string(nbytes) + " bytes do not fit a slot of " +
                            std::to_string(host->slot_size()));
        }
        return *host;
    }

    std::string RingWriters::uuid() const
    {
        std::lock_guard lock(uuid_mutex);
        return cuda_uuid;
    }
}  // namespace durchblick::detail
