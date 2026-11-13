// What the image and the point cloud streams share (camera.cpp, points.cpp; not installed): declaring publishers,
// sending a sample with its attachment, keeping the newest received sample, choosing a transport from the publisher's
// info record, and reading a frame or cloud out of a host or CUDA ring.
#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <zenoh.hxx>

#include "xblick/camera.hpp"
#include "xblick/cuda_ring.hpp"
#include "xblick/protocol.hpp"
#include "xblick/ring.hpp"

namespace xblick::detail
{
    std::int64_t now_ns();
    std::string key(const std::string& base, std::string_view suffix);
    // The key of a ring: its place after the representation's suffix (protocol.hpp): the host, a CUDA ring's GPU too.
    std::string ring_key(const std::string& base, std::string_view suffix, const std::string& host);
    std::string ring_key(const std::string& base, std::string_view suffix, const std::string& host,
                         std::string_view gpu);
    // The key of a stream publisher's liveliness token, <base>/info/<host>/<an id of its own> (protocol.hpp).
    std::string alive_key(const std::string& base, const std::string& host);
    // The key of a reader's want token, <base>/want/<host>/<an id of its own> (protocol.hpp).
    std::string want_key(const std::string& base, const std::string& host);
    // The UUID of the GPU of a publisher's CUDA ring (in its key and info record); throws without a CUDA driver.
    std::string ring_gpu(const std::string& base, int cuda_device);
    // A publisher of latest-value data: drops under congestion, never queues.
    zenoh::Publisher declare(zenoh::Session& session, const std::string& key_expr, const protocol::MessageType& type);
    bool matching(const std::optional<zenoh::Publisher>& publisher);
    void put(zenoh::Publisher& publisher, std::vector<std::uint8_t>&& payload, const std::string& attachment);
    // The attachment of a frame or cloud (protocol.hpp).
    std::string attachment(std::int64_t seq, const std::string& source);
    // with "zid": the publishing session's id (diagnostics: several sessions publish on one key)
    std::string attachment(std::int64_t seq, const std::string& source, const std::string& zid);

    // The attachment of one frame's or cloud's samples, made anew at each put (put below): each transport's
    // publish_ns is when that sample went out, after its encoding or its copy into a ring; publish_called_ns when
    // publish was called.
    struct FrameAttachment
    {
        std::int64_t seq = 0;
        std::string source;
        std::int64_t called_ns = 0;

        std::string now() const;
    };
    FrameAttachment frame_attachment(std::int64_t seq, const std::string& source);
    // The payload is made (encoded, serialized) when this is called: the attachment's publish_ns is taken here.
    void put(zenoh::Publisher& publisher, std::vector<std::uint8_t>&& payload, const FrameAttachment& attachment);

    struct Item
    {
        std::vector<std::uint8_t> payload;
        std::string attachment;
        std::string key;
        std::string encoding;
        std::int64_t receive_ns = 0;
        Transport transport = Transport::Auto;  // the transport it came by
    };

    // How long a subscription that changes its transport keeps the old one for the new one's first frame; after
    // that, as long again the new route's samples are compared with the newest frame kept, and a stamp at most as
    // far behind it is the same frame.
    inline constexpr std::chrono::milliseconds kHandover{2000};

    // The newest sample of a stream, kept as received; take() hands it out once. The samples come by routes (a
    // subscription's subscribers, numbered); those of `route` are kept. When a subscription changes its transport
    // (move), it reads the old and the new route side by side until the new one delivers its first sample (or
    // leave_old). From then on it reads only the new one: a sample of the old route is left out, also one that Zenoh
    // hands over while the old subscriber is undeclared. While both routes are read, and after that for kHandover or
    // until the new route passes the newest frame kept (a relay's ring lags the network by a frame or more), a sample
    // whose header stamp is not newer than the newest one kept is the same frame by the other route and is left out
    // (duplicates). Only on the new route, a stamp more than kHandover of stamp time behind the newest one is a
    // restart of the stream (a camera or a simulation): it is kept and ends the window. The old route's frames of a
    // restart come again by the new route. So the stamps handed out during a move only grow, except after a restart
    // of the stream. What remains:
    // - copies on the new route more than kHandover behind (a relay that queued frames while it started) are taken
    //   for a restart and handed out again: stamps alone do not tell the two apart;
    // - a new route that stops for longer than kHandover after a first sample behind the newest one hands out older
    //   frames again;
    // - frames that carry one stamp on purpose, or a restart less than kHandover back, are not handed out during the
    //   window;
    // - frames are lost when the new route is ahead of the old one (those the old one had still to bring).
    // As Python's _Latest.
    struct Latest
    {
        std::mutex mutex;
        std::condition_variable cv;
        std::shared_ptr<const Item> item;  // shared with the frames made from it: no copy of the payload
        bool fresh = false;
        std::uint64_t received = 0;
        std::uint64_t overwritten = 0;
        std::size_t keep = 0;  // also the last so many samples, not decoded (Subscription::keep_recent)
        std::deque<std::shared_ptr<const Item>> recent;
        std::uint64_t duplicates = 0;
        std::optional<std::int64_t> newest_stamp;
        int route = 0;           // the route whose samples are kept
        std::optional<int> old;  // the route moved from, read until the new one delivers
        // after the move: the new route's samples are compared with newest_stamp until then
        std::optional<std::chrono::steady_clock::time_point> compare_until;

        void put(const zenoh::Sample& sample, Transport transport, int from);
        // Keeps `made`, a sample of route `from`, unless it is left out.
        void offer(std::shared_ptr<Item> made, int from);
        // The samples of `to` are kept from now on; with a route before it, that one's too until `to` delivers its
        // first sample. Before its subscriber is declared.
        void move(int to);
        // True once the new route delivered its first sample (waits at most timeout).
        bool wait_moved(std::chrono::milliseconds timeout);
        // Leaves the old route out from now on, also before the new one delivered: before its subscriber goes.
        void leave_old();
        std::shared_ptr<const Item> take(bool consume, std::optional<std::chrono::milliseconds> timeout);

    private:
        void leave_old_locked();  // with `mutex` held
    };

    // The header stamp of a stream's sample, not decoded (every representation starts with std_msgs/Header).
    std::int64_t stamp_of(const Item& item);

    // ProtocolError unless the sample is of this type and version.
    void check_encoding(const Item& item, const protocol::MessageType& type);

    // seq, publish_ns and source of a sample's attachment (empty if it has none); ProtocolError for another version.
    struct Meta
    {
        std::optional<std::int64_t> seq;
        std::optional<std::int64_t> publish_ns;
        std::string source;
    };
    Meta parse_attachment(const Item& item);

    // The subscription of one stream: a Zenoh subscriber on the key of the transport in use, keeping the newest sample.
    // With Transport::Auto the transport is chosen in a thread of its own from the publishers' info records (whose
    // "kind" must be `kind`): `local` on a publisher's host, `remote` elsewhere. The thread chooses again whenever a
    // publisher of the stream comes or goes (their liveliness tokens), so the order in which programs start does not
    // matter. While the subscription moves to another transport it reads both routes and hands out each frame once.
    class Subscription
    {
    public:
        using Suffix = std::function<std::string_view(Transport)>;

        Subscription(zenoh::Session& session, std::string base, std::string kind, Transport requested, Transport local,
                     Transport remote, Suffix suffix, std::optional<int> cuda_device = std::nullopt,
                     std::optional<Transport> remote_else = std::nullopt);  // where a source lacks `remote`
        ~Subscription();
        Subscription(const Subscription&) = delete;
        Subscription& operator=(const Subscription&) = delete;

        struct Taken
        {
            std::shared_ptr<const Item> item;
            Transport transport;
        };
        // The newest sample and the transport it came by; nullopt if there is none (yet), as latest() describes.
        std::optional<Taken> take(bool consume, std::optional<std::chrono::milliseconds> timeout);
        // A ring descriptor named another host (cannot be by the rings' keys): with Auto choose again (true), else
        // throw.
        bool moved(const std::string& host, Transport transport);
        // At most max_hz samples a second taken (camera.py's MaxRate): a sample is taken when its header stamp lies
        // in a later period of 1/max_hz s than the last one taken (periods from the epoch: the streams of one camera
        // take the same frames); the others are skipped before they are decoded. 0: every sample.
        void limit_rate(double max_hz);
        std::uint64_t skipped_max_hz() const { return skipped_max_hz_.load(); }
        // Keep the last n samples as they came, not decoded: a partner stream, read at the stamps of another's
        // (ImageSubscriber::at).
        void keep_recent(std::size_t n);
        // The samples kept and the transport they came by (empty before a transport was chosen).
        std::optional<std::pair<std::vector<std::shared_ptr<const Item>>, Transport>> recent() const;

        std::optional<Transport> transport() const;
        // Why Auto took the transport it took, e.g. "cuda (publisher on this GPU)"; empty before it chose.
        std::string transport_reason() const;
        std::uint64_t received() const;
        std::uint64_t overwritten() const;
        const std::string& base() const { return base_; }
        const std::string& host() const { return host_; }

    private:
        std::string key_of(Transport t, std::string_view gpu) const;
        void use(Transport t, const std::string& gpu);
        void choose();
        void recheck();

        zenoh::Session& session_;
        std::string base_;
        std::string kind_;
        Transport requested_, local_, remote_;
        std::optional<Transport> remote_else_;
        Suffix suffix_;
        std::optional<int> cuda_device_;
        std::string reason_;
        std::string host_;
        mutable std::mutex mutex_;
        std::condition_variable chosen_cv_;
        std::optional<Transport> transport_;
        std::optional<zenoh::Subscriber<void>> subscriber_;
        std::optional<zenoh::Subscriber<void>> alive_;  // the publishers' liveliness tokens
        std::optional<zenoh::LivelinessToken> want_;  // asks this host's relay for the stream (chooser's thread)
        std::optional<std::pair<Transport, std::string>> using_;  // transport and GPU of the subscriber in use
        int route_ = 0;
        std::mutex recheck_mutex_;
        std::condition_variable recheck_cv_;
        bool recheck_ = false;  // a publisher came or went: choose again
        const std::shared_ptr<Latest> latest_ = std::make_shared<Latest>();
        std::exception_ptr error_;
        std::atomic<bool> closed_{false};
        std::thread chooser_;
        std::size_t keep_ = 0;        // keep_recent
        std::int64_t period_ns_ = 0;  // limit_rate; 0 none
        std::optional<std::int64_t> last_period_;
        std::atomic<std::uint64_t> skipped_max_hz_{0};
    };

    // The bytes of a frame or cloud read out of a ring: a copy (bytes, or a DeviceBuffer behind device_ptr) or, with
    // copy = false, a view that stays right while valid() holds.
    struct RingData
    {
        std::vector<std::uint8_t> bytes;
        const std::uint8_t* view = nullptr;
        std::uint64_t device_ptr = 0;
        int device = -1;
        std::shared_ptr<const void> owner;
        std::function<bool()> valid;
    };

    // The readers of the rings a subscriber reads from, opened again when the publisher's ring changes (restart).
    class RingReaders
    {
    public:
        // nullopt: the frame (slot, seq) was overwritten before or while it was read.
        std::optional<RingData> host(const std::string& segment, std::uint32_t slot, std::uint64_t seq,
                                     std::uint64_t nbytes, bool copy);
        // target: the device of the subscriber (copies arrive there); nullopt: the ring's device.
        std::optional<RingData> cuda(const std::string& endpoint, std::uint32_t slot, std::uint64_t seq, bool copy,
                                     std::optional<int> target = std::nullopt);

    private:
        std::shared_ptr<HostRingReader> host_ring_;
        std::shared_ptr<CudaRingReader> cuda_ring_;
        std::string host_name_, cuda_name_;
    };

    // The ring writers of a publisher, created at the first frame or cloud a reader asks for and sized for it, and the
    // descriptors' common work.
    struct RingWriters
    {
        std::unique_ptr<HostRingWriter> host;
        std::unique_ptr<CudaRingWriter> cuda;  // after host: destroyed first, it unpins host's slots
        mutable std::mutex uuid_mutex;  // the info queryable reads the uuid from Zenoh's thread
        std::string cuda_uuid;

        HostRingWriter& host_ring(std::uint32_t slots, std::uint64_t nbytes);
        CudaRingWriter& cuda_ring(int device, std::uint32_t slots, std::uint64_t nbytes);
        // The host ring for downloads from the CUDA ring: page-locked, so that the GPU writes into it directly.
        HostRingWriter& pinned_host_ring(std::uint32_t slots, std::uint64_t nbytes);
        std::string uuid() const;
    };
}  // namespace xblick::detail
