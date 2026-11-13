// Publishing and receiving one point cloud stream in C++ (protocol.hpp: cloud, cloud/shm, cloud/cuda), as Python's
// points.py, in the manner of camera.hpp's image streams: a cloud is written once into a ring, readers on the same
// host get a descriptor (cloud/shm: host memory, cloud/cuda: device memory, no copy needed); sensor_msgs/PointCloud2
// over the network (cloud) works too, but costs the full size per cloud (a 640x576 cloud of 16-byte points: 5.9 MB).
//
// cloud/compressed is the network form of a cloud whose points lie on fixed rays (a camera's, a scanner's): a point is
// its ray's direction times its range, so the cloud goes out as an image of ranges (16 bit, RVL) and one of colours
// (JPEG), about a twentieth of its size; the directions go once to each subscriber (cloud/rays). The publisher gets
// them as CloudPublisherOptions::rays and knows nothing else about the cloud's origin (Python's points.py and
// docs/concepts/transports.md describe the transport).
//
//   durchblick::PointCloudPublisher points(session, "robot/camera/points",
//                                          {.layout = durchblick::xyzrgb_layout(),
//                                           .frame_id = "AzureKinectDepthCamera", .cuda_device = 0});
//   if (points.subscribed()) {
//       const std::uint64_t ptr = points.loan(576, 640);   // fill it on `stream`, e.g. with a kernel
//       points.commit(stream, false, capture_ns);
//   }
//
//   durchblick::PointCloudSubscriber cloud(session, "robot/camera/points",
//                                          {.local = durchblick::Transport::Cuda});
//   if (auto c = cloud.latest(true, std::chrono::milliseconds(100))) { use(c->device_ptr, c->height, c->width, ...); }
#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <zenoh.hxx>

#include "durchblick/camera.hpp"
#include "durchblick/msg.hpp"

namespace durchblick
{
    // What a point holds: its fields and its size in bytes (point_step, at least the fields' extent).
    struct PointLayout
    {
        std::vector<PointField> fields;
        std::uint32_t point_step = 0;
    };

    // x, y, z as float32 in metres, 16 bytes per point (PCL's PointXYZ, padded for aligned loads on a GPU).
    PointLayout xyz_layout();
    // x, y, z and PCL's packed colour "rgb" at offset 12 (float32 whose bits are the uint32 0x00RRGGBB; PCL's
    // PointXYZRGB, 16 bytes per point).
    PointLayout xyzrgb_layout();
    // Checks the layout (known datatypes, fields inside point_step, no field twice); throws std::invalid_argument.
    void check_layout(const PointLayout& layout);

    // Points in host memory, rows without padding: height * width * point_step bytes.
    struct HostCloud
    {
        const void* data = nullptr;
        std::uint32_t height = 0;
        std::uint32_t width = 0;
        bool is_dense = false;  // true if there are no invalid (NaN) points
    };

    // Points in device memory (rows without padding), ready on stream (a CUstream; 0 = the legacy default stream).
    struct DeviceCloud
    {
        std::uint64_t ptr = 0;
        std::uint32_t height = 0;
        std::uint32_t width = 0;
        std::uint64_t stream = 0;
        bool is_dense = false;
    };

    // The direction of each point's ray, for cloud/compressed: 3 float32 per point, rows first. A camera's node
    // passes (x/z, y/z, 1) of each pixel (the range is then z), a scanner's the beams' unit vectors.
    struct CloudRays
    {
        std::uint32_t height = 0;
        std::uint32_t width = 0;
        std::vector<float> directions;
    };

    struct CloudPublisherOptions
    {
        PointLayout layout;
        std::string frame_id;
        std::string source;  // e.g. "robot" or "sim": in every cloud's attachment
        std::uint32_t slots = 4;  // ring slots: a reader has slots - 1 cloud periods for a cloud
        std::optional<int> cuda_device;  // offer cloud/cuda with a CUDA ring on this device
        // offer cloud/compressed: the points lie on these rays (the layout: x, y, z and at most rgb). Every cloud is
        // checked against them; one that does not lie on them throws std::invalid_argument.
        std::optional<CloudRays> rays;
        double range_step = 0.001;         // metres per step of a range (16 bits: 65.5 m at 1 mm)
        std::string color_codec = "jpeg";  // of the colours: jpeg, or png (exact and slower)
        int jpeg_quality = 95;
    };

    class PointCloudPublisher
    {
    public:
        PointCloudPublisher(zenoh::Session& session, const std::string& base, CloudPublisherOptions options);
        ~PointCloudPublisher();
        PointCloudPublisher(const PointCloudPublisher&) = delete;
        PointCloudPublisher& operator=(const PointCloudPublisher&) = delete;

        // True if anybody takes clouds of this stream (any representation): compute them only then.
        bool subscribed() const;
        // Publish one cloud captured at stamp_ns (nanoseconds since the epoch; default: now). Returns the topics it
        // went to.
        std::vector<std::string> publish(const HostCloud& cloud, std::optional<std::int64_t> stamp_ns = {});
        // A cloud in device memory of the producer's own (needs cuda_device): copied into the CUDA ring on the GPU.
        std::vector<std::string> publish(const DeviceCloud& cloud, std::optional<std::int64_t> stamp_ns = {});
        // Zero copy on the GPU (needs cuda_device): loan() returns the device memory of the next CUDA ring slot for a
        // cloud of height x width points (rows without padding); fill it with work on a stream, then commit() it with
        // that stream. Host representations (shm, raw) download it once, only if somebody takes them. The first loan
        // sizes the rings; a larger cloud gets new ones (its readers follow them by name).
        std::uint64_t loan(std::uint32_t height, std::uint32_t width);
        std::vector<std::string> commit(std::uint64_t stream, bool is_dense, std::optional<std::int64_t> stamp_ns = {});
        // What <base>/info answers (JSON).
        std::string info_record() const;
        // The frame of the clouds from now on (e.g. once a producer knows it from its input's camera_info).
        void set_frame_id(const std::string& frame_id);
        // Other rays for the clouds from now on (a camera back with another size); only for a publisher made with rays.
        void set_rays(CloudRays rays);

        const std::string& base() const { return base_; }

    private:
        struct Impl;
        std::string base_;
        std::unique_ptr<Impl> impl_;
    };

    struct CloudSubscriberOptions
    {
        // Auto, Raw (sensor_msgs/PointCloud2), Compressed (ranges on fixed rays), Shm or Cuda
        Transport transport = Transport::Auto;
        // What Auto takes from a publisher on another host: Auto (Compressed where every publisher offers it, that is
        // where it has rays, else Raw), Compressed or Raw.
        Transport remote = Transport::Auto;
        // What Auto takes on the publisher's host: Auto (the publisher's CUDA ring if it is on cuda_device, else Shm),
        // Shm, or Cuda (the CUDA ring where the publisher offers one, else Shm); elsewhere it takes `remote`.
        Transport local = Transport::Auto;
        // with local Auto: the GPU this subscriber works on (an ordinal, as CUDA numbers them); unset: the CPU, so
        // Auto takes shared memory (a CUDA ring only for a subscriber that works on a GPU)
        std::optional<int> cuda_device;
        bool copy = true;  // false: shm/cuda clouds are views into the ring (check is_valid)
        double max_hz = 0.0;  // at most so many clouds a second, the newest (SubscriberOptions::max_hz); 0: all
    };

    struct Cloud
    {
        Header header;
        std::uint32_t height = 0;
        std::uint32_t width = 0;
        std::vector<PointField> fields;
        std::uint32_t point_step = 0;
        std::uint32_t row_step = 0;
        bool is_dense = false;
        std::string key;
        std::int64_t receive_ns = 0;
        std::optional<std::int64_t> seq;
        std::string source;
        std::optional<std::int64_t> publish_ns;

        // Host points (every transport but cuda), rows without padding (row_step = width * point_step):
        // valid while the cloud exists (copies), or while is_valid() holds (shm views).
        const std::uint8_t* data() const { return view != nullptr ? view : points.data(); }
        // Device points (cuda): a pointer on device `device`.
        std::uint64_t device_ptr = 0;
        int device = -1;

        // False once the publisher has begun overwriting a zero-copy cloud; always true for copies.
        bool is_valid() const { return valid ? valid() : true; }
        // The field of this name, or nullptr.
        const PointField* field(const std::string& name) const;

        std::vector<std::uint8_t> points;
        const std::uint8_t* view = nullptr;
        std::shared_ptr<const void> owner;  // keeps the payload, a copy's DeviceBuffer or a view's ring alive
        std::function<bool()> valid;

        std::int64_t stamp_ns() const { return header.stamp.to_ns(); }
        // From capture to the put of this cloud's sample (Frame::age_at_send_ns).
        std::optional<std::int64_t> age_at_send_ns() const
        {
            if (not publish_ns or stamp_ns() == 0)
            {
                return std::nullopt;
            }
            return *publish_ns - stamp_ns();
        }
    };

    class PointCloudSubscriber
    {
    public:
        PointCloudSubscriber(zenoh::Session& session, const std::string& base, CloudSubscriberOptions options = {});
        ~PointCloudSubscriber();
        PointCloudSubscriber(const PointCloudSubscriber&) = delete;
        PointCloudSubscriber& operator=(const PointCloudSubscriber&) = delete;

        // The newest cloud; with consume each cloud is returned once. With a timeout, wait that long for a new one.
        // Empty as well for a cloud overwritten in its ring before it could be read (counted in stale()).
        std::optional<Cloud> latest(bool consume = true,
                                    std::optional<std::chrono::milliseconds> timeout = std::nullopt);
        // The transport in use; empty while Auto has not found the publisher yet.
        std::optional<Transport> transport() const;
        // Why it took that transport, e.g. "cuda (publisher on this GPU)"; empty before it chose.
        std::string transport_reason() const;
        std::uint64_t received() const;
        std::uint64_t overwritten() const;
        std::uint64_t stale() const;
        std::uint64_t skipped_max_hz() const;  // clouds left out for CloudSubscriberOptions::max_hz

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}  // namespace durchblick
