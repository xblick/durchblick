#include "xblick/msg.hpp"

#include "xblick/rectify.hpp"

#include <bit>
#include <cmath>
#include <cstring>

namespace xblick
{
    // The CDR writer and reader and the shared messages are in ros2_cdr_msgs.hpp.

    namespace
    {
        void write(CdrWriter& w, const Header& h)
        {
            w.i32(h.stamp.sec);
            w.u32(h.stamp.nanosec);
            w.string(h.frame_id);
        }

        Header read_header(CdrReader& r)
        {
            Header h;
            h.stamp.sec = r.i32();
            h.stamp.nanosec = r.u32();
            h.frame_id = r.string();
            return h;
        }

        void write_image_fields(CdrWriter& w, const RingImage& m)
        {
            write(w, m.header);
            w.u32(m.height);
            w.u32(m.width);
            w.string(m.encoding);
            w.u8(m.is_bigendian);
            w.u32(m.step);
            w.string(m.host);
        }

        void read_image_fields(CdrReader& r, RingImage& m)
        {
            m.header = read_header(r);
            m.height = r.u32();
            m.width = r.u32();
            m.encoding = r.string();
            m.is_bigendian = r.u8();
            m.step = r.u32();
            m.host = r.string();
        }
    }  // namespace

    std::vector<std::uint8_t> Image::serialize() const
    {
        CdrWriter w;
        w.reserve(64 + header.frame_id.size() + encoding.size() + data.size());
        write(w, header);
        w.u32(height);
        w.u32(width);
        w.string(encoding);
        w.u8(is_bigendian);
        w.u32(step);
        w.bytes(data.data(), data.size());
        return w.take();
    }

    Image Image::deserialize(std::span<const std::uint8_t> payload)
    {
        CdrReader r(payload);
        Image m;
        m.header = read_header(r);
        m.height = r.u32();
        m.width = r.u32();
        m.encoding = r.string();
        m.is_bigendian = r.u8();
        m.step = r.u32();
        m.data = r.bytes();
        r.expect_end();
        return m;
    }

    std::vector<std::uint8_t> CompressedImage::serialize() const
    {
        CdrWriter w;
        w.reserve(64 + header.frame_id.size() + format.size() + data.size());
        write(w, header);
        w.string(format);
        w.bytes(data.data(), data.size());
        return w.take();
    }

    CompressedImage CompressedImage::deserialize(std::span<const std::uint8_t> payload)
    {
        CdrReader r(payload);
        CompressedImage m;
        m.header = read_header(r);
        m.format = r.string();
        auto bytes = r.bytes();
        m.data.assign(bytes.begin(), bytes.end());
        r.expect_end();
        return m;
    }

    CameraInfo CameraInfo::pinhole(std::uint32_t width, std::uint32_t height, double fx, double fy, double cx,
                                   double cy, const std::string& frame_id)
    {
        CameraInfo info;
        info.header.frame_id = frame_id;
        info.width = width;
        info.height = height;
        info.distortion_model = "plumb_bob";
        info.d.assign(5, 0.0);
        info.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
        info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
        info.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
        return info;
    }

    std::vector<std::uint8_t> CameraInfo::serialize() const
    {
        CdrWriter w;
        write(w, header);
        w.u32(height);
        w.u32(width);
        w.string(distortion_model);
        w.f64_sequence(d);
        w.f64_array(k.data(), 9);
        w.f64_array(r.data(), 9);
        w.f64_array(p.data(), 12);
        w.u32(binning_x);
        w.u32(binning_y);
        w.u32(roi.x_offset);
        w.u32(roi.y_offset);
        w.u32(roi.height);
        w.u32(roi.width);
        w.u8(roi.do_rectify ? 1 : 0);
        return w.take();
    }

    CameraInfo CameraInfo::deserialize(std::span<const std::uint8_t> payload)
    {
        CdrReader rd(payload);
        CameraInfo m;
        m.header = read_header(rd);
        m.height = rd.u32();
        m.width = rd.u32();
        m.distortion_model = rd.string();
        m.d = rd.f64_sequence();
        rd.f64_array(m.k.data(), 9);
        rd.f64_array(m.r.data(), 9);
        rd.f64_array(m.p.data(), 12);
        m.binning_x = rd.u32();
        m.binning_y = rd.u32();
        m.roi.x_offset = rd.u32();
        m.roi.y_offset = rd.u32();
        m.roi.height = rd.u32();
        m.roi.width = rd.u32();
        m.roi.do_rectify = rd.u8() != 0;
        rd.expect_end();
        return m;
    }

    std::vector<std::uint8_t> Imu::serialize() const
    {
        CdrWriter w;
        write(w, header);
        for (double v : {orientation.x, orientation.y, orientation.z, orientation.w})
        {
            w.f64(v);
        }
        w.f64_array(orientation_covariance.data(), 9);
        for (double v : {angular_velocity.x, angular_velocity.y, angular_velocity.z})
        {
            w.f64(v);
        }
        w.f64_array(angular_velocity_covariance.data(), 9);
        for (double v : {linear_acceleration.x, linear_acceleration.y, linear_acceleration.z})
        {
            w.f64(v);
        }
        w.f64_array(linear_acceleration_covariance.data(), 9);
        return w.take();
    }

    Imu Imu::deserialize(std::span<const std::uint8_t> payload)
    {
        CdrReader r(payload);
        Imu m;
        m.header = read_header(r);
        m.orientation = {r.f64(), r.f64(), r.f64(), r.f64()};
        r.f64_array(m.orientation_covariance.data(), 9);
        m.angular_velocity = {r.f64(), r.f64(), r.f64()};
        r.f64_array(m.angular_velocity_covariance.data(), 9);
        m.linear_acceleration = {r.f64(), r.f64(), r.f64()};
        r.f64_array(m.linear_acceleration_covariance.data(), 9);
        r.expect_end();
        return m;
    }

    std::vector<std::uint8_t> ShmImage::serialize() const
    {
        CdrWriter w;
        write_image_fields(w, *this);
        w.string(segment);
        w.u32(slot);
        w.u64(seq);
        return w.take();
    }

    ShmImage ShmImage::deserialize(std::span<const std::uint8_t> payload)
    {
        CdrReader r(payload);
        ShmImage m;
        read_image_fields(r, m);
        m.segment = r.string();
        m.slot = r.u32();
        m.seq = r.u64();
        r.expect_end();
        return m;
    }

    std::uint32_t point_field_size(std::uint8_t datatype)
    {
        switch (datatype)
        {
            case PointField::kInt8:
            case PointField::kUint8: return 1;
            case PointField::kInt16:
            case PointField::kUint16: return 2;
            case PointField::kInt32:
            case PointField::kUint32:
            case PointField::kFloat32: return 4;
            case PointField::kFloat64: return 8;
        }
        throw CdrError("unknown PointField datatype " + std::to_string(datatype));
    }

    namespace
    {
        void write_fields(CdrWriter& w, const std::vector<PointField>& fields)
        {
            w.u32(static_cast<std::uint32_t>(fields.size()));
            for (const PointField& f : fields)
            {
                w.string(f.name);
                w.u32(f.offset);
                w.u8(f.datatype);
                w.u32(f.count);
            }
        }

        std::vector<PointField> read_fields(CdrReader& r)
        {
            std::vector<PointField> fields(r.u32());
            for (PointField& f : fields)
            {
                f.name = r.string();
                f.offset = r.u32();
                f.datatype = r.u8();
                f.count = r.u32();
            }
            return fields;
        }

        void write_cloud_fields(CdrWriter& w, const RingPointCloud2& m)
        {
            write(w, m.header);
            w.u32(m.height);
            w.u32(m.width);
            write_fields(w, m.fields);
            w.u8(m.is_bigendian);
            w.u32(m.point_step);
            w.u32(m.row_step);
            w.u8(m.is_dense);
            w.string(m.host);
        }

        void read_cloud_fields(CdrReader& r, RingPointCloud2& m)
        {
            m.header = read_header(r);
            m.height = r.u32();
            m.width = r.u32();
            m.fields = read_fields(r);
            m.is_bigendian = r.u8() != 0;
            m.point_step = r.u32();
            m.row_step = r.u32();
            m.is_dense = r.u8() != 0;
            m.host = r.string();
        }
    }  // namespace

    std::vector<std::uint8_t> PointCloud2::serialize() const
    {
        CdrWriter w;
        w.reserve(128 + header.frame_id.size() + 32 * fields.size() + data.size());
        write(w, header);
        w.u32(height);
        w.u32(width);
        write_fields(w, fields);
        w.u8(is_bigendian);
        w.u32(point_step);
        w.u32(row_step);
        w.bytes(data.data(), data.size());
        w.u8(is_dense);
        return w.take();
    }

    PointCloud2 PointCloud2::deserialize(std::span<const std::uint8_t> payload)
    {
        CdrReader r(payload);
        PointCloud2 m;
        m.header = read_header(r);
        m.height = r.u32();
        m.width = r.u32();
        m.fields = read_fields(r);
        m.is_bigendian = r.u8() != 0;
        m.point_step = r.u32();
        m.row_step = r.u32();
        m.data = r.bytes();
        m.is_dense = r.u8() != 0;
        r.expect_end();
        return m;
    }

    std::vector<std::uint8_t> CompressedPointCloud2::serialize() const
    {
        CdrWriter w;
        w.reserve(256 + header.frame_id.size() + 32 * fields.size() + range.size() + color.size());
        write(w, header);
        w.u32(height);
        w.u32(width);
        write_fields(w, fields);
        w.u32(point_step);
        w.u8(is_dense);
        w.string(rays);
        w.f64(range_step);
        w.string(range_format);
        w.bytes(range.data(), range.size());
        w.string(color_format);
        w.bytes(color.data(), color.size());
        return w.take();
    }

    CompressedPointCloud2 CompressedPointCloud2::deserialize(std::span<const std::uint8_t> payload)
    {
        CdrReader r(payload);
        CompressedPointCloud2 m;
        m.header = read_header(r);
        m.height = r.u32();
        m.width = r.u32();
        m.fields = read_fields(r);
        m.point_step = r.u32();
        m.is_dense = r.u8() != 0;
        m.rays = r.string();
        m.range_step = r.f64();
        m.range_format = r.string();
        m.range = r.bytes();
        m.color_format = r.string();
        m.color = r.bytes();
        r.expect_end();
        return m;
    }

    std::vector<std::uint8_t> Rays::serialize() const
    {
        CdrWriter w;
        w.reserve(64 + directions.size());
        w.string(id);
        w.u32(height);
        w.u32(width);
        w.bytes(directions.data(), directions.size());
        return w.take();
    }

    Rays Rays::deserialize(std::span<const std::uint8_t> payload)
    {
        CdrReader r(payload);
        Rays m;
        m.id = r.string();
        m.height = r.u32();
        m.width = r.u32();
        m.directions = r.bytes();
        r.expect_end();
        return m;
    }

    std::vector<std::uint8_t> ShmPointCloud2::serialize() const
    {
        CdrWriter w;
        write_cloud_fields(w, *this);
        w.string(segment);
        w.u32(slot);
        w.u64(seq);
        return w.take();
    }

    ShmPointCloud2 ShmPointCloud2::deserialize(std::span<const std::uint8_t> payload)
    {
        CdrReader r(payload);
        ShmPointCloud2 m;
        read_cloud_fields(r, m);
        m.segment = r.string();
        m.slot = r.u32();
        m.seq = r.u64();
        r.expect_end();
        return m;
    }

    std::vector<std::uint8_t> CudaPointCloud2::serialize() const
    {
        CdrWriter w;
        write_cloud_fields(w, *this);
        w.string(endpoint);
        w.string(device_uuid);
        w.u32(slot);
        w.u64(seq);
        return w.take();
    }

    CudaPointCloud2 CudaPointCloud2::deserialize(std::span<const std::uint8_t> payload)
    {
        CdrReader r(payload);
        CudaPointCloud2 m;
        read_cloud_fields(r, m);
        m.endpoint = r.string();
        m.device_uuid = r.string();
        m.slot = r.u32();
        m.seq = r.u64();
        r.expect_end();
        return m;
    }

    std::vector<std::uint8_t> CudaImage::serialize() const
    {
        CdrWriter w;
        write_image_fields(w, *this);
        w.string(endpoint);
        w.string(device_uuid);
        w.u32(slot);
        w.u64(seq);
        return w.take();
    }

    CudaImage CudaImage::deserialize(std::span<const std::uint8_t> payload)
    {
        CdrReader r(payload);
        CudaImage m;
        read_image_fields(r, m);
        m.endpoint = r.string();
        m.device_uuid = r.string();
        m.slot = r.u32();
        m.seq = r.u64();
        r.expect_end();
        return m;
    }

    // ------------------------------------------------------------------------------------------- vision_msgs

    namespace
    {
        void write_pose(CdrWriter& w, const Pose& p)
        {
            for (double v : {p.position.x, p.position.y, p.position.z, p.orientation.x, p.orientation.y,
                             p.orientation.z, p.orientation.w})
            {
                w.f64(v);
            }
        }

        Pose read_pose(CdrReader& r)
        {
            Pose p;
            p.position = {r.f64(), r.f64(), r.f64()};
            p.orientation = {r.f64(), r.f64(), r.f64(), r.f64()};
            return p;
        }

        void write_results(CdrWriter& w, const std::vector<ObjectHypothesisWithPose>& results)
        {
            w.u32(static_cast<std::uint32_t>(results.size()));
            for (const ObjectHypothesisWithPose& h : results)
            {
                w.string(h.hypothesis.class_id);
                w.f64(h.hypothesis.score);
                write_pose(w, h.pose.pose);
                w.f64_array(h.pose.covariance.data(), h.pose.covariance.size());
            }
        }

        std::vector<ObjectHypothesisWithPose> read_results(CdrReader& r)
        {
            // element by element: a corrupt count ends at the payload's end, not in an allocation
            std::vector<ObjectHypothesisWithPose> results;
            const std::uint32_t n = r.u32();
            for (std::uint32_t i = 0; i < n; ++i)
            {
                ObjectHypothesisWithPose h;
                h.hypothesis.class_id = r.string();
                h.hypothesis.score = r.f64();
                h.pose.pose = read_pose(r);
                r.f64_array(h.pose.covariance.data(), h.pose.covariance.size());
                results.push_back(std::move(h));
            }
            return results;
        }
    }  // namespace

    std::vector<std::uint8_t> Detection2DArray::serialize() const
    {
        CdrWriter w;
        write(w, header);
        w.u32(static_cast<std::uint32_t>(detections.size()));
        for (const Detection2D& d : detections)
        {
            write(w, d.header);
            write_results(w, d.results);
            for (double v : {d.bbox.center.position.x, d.bbox.center.position.y, d.bbox.center.theta, d.bbox.size_x,
                             d.bbox.size_y})
            {
                w.f64(v);
            }
            w.string(d.id);
        }
        return w.take();
    }

    Detection2DArray Detection2DArray::deserialize(std::span<const std::uint8_t> payload)
    {
        CdrReader r(payload);
        Detection2DArray m;
        m.header = read_header(r);
        const std::uint32_t n = r.u32();
        for (std::uint32_t i = 0; i < n; ++i)
        {
            Detection2D& d = m.detections.emplace_back();
            d.header = read_header(r);
            d.results = read_results(r);
            d.bbox.center.position = {r.f64(), r.f64()};
            d.bbox.center.theta = r.f64();
            d.bbox.size_x = r.f64();
            d.bbox.size_y = r.f64();
            d.id = r.string();
        }
        r.expect_end();
        return m;
    }

    std::vector<std::uint8_t> Detection3DArray::serialize() const
    {
        CdrWriter w;
        write(w, header);
        w.u32(static_cast<std::uint32_t>(detections.size()));
        for (const Detection3D& d : detections)
        {
            write(w, d.header);
            write_results(w, d.results);
            write_pose(w, d.bbox.center);
            for (double v : {d.bbox.size.x, d.bbox.size.y, d.bbox.size.z})
            {
                w.f64(v);
            }
            w.string(d.id);
        }
        return w.take();
    }

    Detection3DArray Detection3DArray::deserialize(std::span<const std::uint8_t> payload)
    {
        CdrReader r(payload);
        Detection3DArray m;
        m.header = read_header(r);
        const std::uint32_t n = r.u32();
        for (std::uint32_t i = 0; i < n; ++i)
        {
            Detection3D& d = m.detections.emplace_back();
            d.header = read_header(r);
            d.results = read_results(r);
            d.bbox.center = read_pose(r);
            d.bbox.size = {r.f64(), r.f64(), r.f64()};
            d.id = r.string();
        }
        r.expect_end();
        return m;
    }

    std::vector<std::uint8_t> Classification::serialize() const
    {
        CdrWriter w;
        write(w, header);
        w.u32(static_cast<std::uint32_t>(results.size()));
        for (const ObjectHypothesis& h : results)
        {
            w.string(h.class_id);
            w.f64(h.score);
        }
        return w.take();
    }

    Classification Classification::deserialize(std::span<const std::uint8_t> payload)
    {
        CdrReader r(payload);
        Classification m;
        m.header = read_header(r);
        const std::uint32_t n = r.u32();
        for (std::uint32_t i = 0; i < n; ++i)
        {
            ObjectHypothesis& h = m.results.emplace_back();
            h.class_id = r.string();
            h.score = r.f64();
        }
        r.expect_end();
        return m;
    }

    // ------------------------------------------------------------------------------------------- robot state

    std::vector<std::uint8_t> Odometry::serialize() const
    {
        CdrWriter w;
        write(w, header);
        w.string(child_frame_id);
        write_pose(w, pose.pose);
        w.f64_array(pose.covariance.data(), pose.covariance.size());
        const Twist& t = twist.twist;
        for (double v : {t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z})
        {
            w.f64(v);
        }
        w.f64_array(twist.covariance.data(), twist.covariance.size());
        return w.take();
    }

    Odometry Odometry::deserialize(std::span<const std::uint8_t> payload)
    {
        CdrReader r(payload);
        Odometry m;
        m.header = read_header(r);
        m.child_frame_id = r.string();
        m.pose.pose = read_pose(r);
        r.f64_array(m.pose.covariance.data(), m.pose.covariance.size());
        m.twist.twist.linear = {r.f64(), r.f64(), r.f64()};
        m.twist.twist.angular = {r.f64(), r.f64(), r.f64()};
        r.f64_array(m.twist.covariance.data(), m.twist.covariance.size());
        r.expect_end();
        return m;
    }

    // ------------------------------------------------------------------------------- the controller manager

    namespace
    {
        void write_strings(CdrWriter& w, const std::vector<std::string>& v)
        {
            w.u32(static_cast<std::uint32_t>(v.size()));
            for (const std::string& s : v)
            {
                w.string(s);
            }
        }

        std::vector<std::string> read_strings(CdrReader& r)
        {
            std::vector<std::string> v(r.count(4));
            for (std::string& s : v)
            {
                s = r.string();
            }
            return v;
        }
    }  // namespace

    std::vector<std::uint8_t> ListControllersRequest::serialize() const
    {
        CdrWriter w;
        w.u8(0);  // rosidl's structure_needs_at_least_one_member
        return w.take();
    }

    ListControllersRequest ListControllersRequest::deserialize(std::span<const std::uint8_t> payload)
    {
        CdrReader r(payload);
        if (payload.size() > 4)
        {
            r.u8();
        }
        r.expect_end();
        return {};
    }

    std::vector<std::uint8_t> ListControllersResponse::serialize() const
    {
        CdrWriter w;
        w.u32(static_cast<std::uint32_t>(controller.size()));
        for (const ControllerState& s : controller)
        {
            w.string(s.name);
            w.string(s.state);
            w.string(s.type);
            w.u8(s.is_async ? 1 : 0);
            w.u16(s.update_rate);
            write_strings(w, s.claimed_interfaces);
            write_strings(w, s.required_command_interfaces);
            write_strings(w, s.required_state_interfaces);
            w.u8(s.is_chainable ? 1 : 0);
            w.u8(s.is_chained ? 1 : 0);
            write_strings(w, s.exported_state_interfaces);
            write_strings(w, s.reference_interfaces);
            w.u32(static_cast<std::uint32_t>(s.chain_connections.size()));
            for (const ChainConnection& c : s.chain_connections)
            {
                w.string(c.name);
                write_strings(w, c.reference_interfaces);
            }
        }
        return w.take();
    }

    ListControllersResponse ListControllersResponse::deserialize(std::span<const std::uint8_t> payload)
    {
        CdrReader r(payload);
        ListControllersResponse m;
        m.controller.resize(r.count(1));
        for (ControllerState& s : m.controller)
        {
            s.name = r.string();
            s.state = r.string();
            s.type = r.string();
            s.is_async = r.u8() != 0;
            s.update_rate = r.u16();
            s.claimed_interfaces = read_strings(r);
            s.required_command_interfaces = read_strings(r);
            s.required_state_interfaces = read_strings(r);
            s.is_chainable = r.u8() != 0;
            s.is_chained = r.u8() != 0;
            s.exported_state_interfaces = read_strings(r);
            s.reference_interfaces = read_strings(r);
            s.chain_connections.resize(r.count(1));
            for (ChainConnection& c : s.chain_connections)
            {
                c.name = r.string();
                c.reference_interfaces = read_strings(r);
            }
        }
        r.expect_end();
        return m;
    }

    std::vector<std::uint8_t> SwitchControllerRequest::serialize() const
    {
        CdrWriter w;
        write_strings(w, activate_controllers);
        write_strings(w, deactivate_controllers);
        w.i32(strictness);
        w.u8(activate_asap ? 1 : 0);
        w.i32(timeout.sec);
        w.u32(timeout.nanosec);
        return w.take();
    }

    SwitchControllerRequest SwitchControllerRequest::deserialize(std::span<const std::uint8_t> payload)
    {
        CdrReader r(payload);
        SwitchControllerRequest m;
        m.activate_controllers = read_strings(r);
        m.deactivate_controllers = read_strings(r);
        m.strictness = r.i32();
        m.activate_asap = r.u8() != 0;
        m.timeout.sec = r.i32();
        m.timeout.nanosec = r.u32();
        r.expect_end();
        return m;
    }

    std::vector<std::uint8_t> SwitchControllerResponse::serialize() const
    {
        CdrWriter w;
        w.u8(ok ? 1 : 0);
        w.string(message);
        return w.take();
    }

    SwitchControllerResponse SwitchControllerResponse::deserialize(std::span<const std::uint8_t> payload)
    {
        CdrReader r(payload);
        SwitchControllerResponse m;
        m.ok = r.u8() != 0;
        m.message = r.string();
        r.expect_end();
        return m;
    }

    std::vector<std::uint8_t> UnloadControllerRequest::serialize() const
    {
        CdrWriter w;
        w.string(name);
        return w.take();
    }

    UnloadControllerRequest UnloadControllerRequest::deserialize(std::span<const std::uint8_t> payload)
    {
        CdrReader r(payload);
        UnloadControllerRequest m;
        m.name = r.string();
        r.expect_end();
        return m;
    }

    std::vector<std::uint8_t> OkResponse::serialize() const
    {
        CdrWriter w;
        w.u8(ok ? 1 : 0);
        return w.take();
    }

    OkResponse OkResponse::deserialize(std::span<const std::uint8_t> payload)
    {
        CdrReader r(payload);
        OkResponse m;
        m.ok = r.u8() != 0;
        r.expect_end();
        return m;
    }

    std::vector<std::uint8_t> CreateControllerRequest::serialize() const
    {
        CdrWriter w;
        w.string(name);
        w.string(type);
        w.string(config);
        return w.take();
    }

    CreateControllerRequest CreateControllerRequest::deserialize(std::span<const std::uint8_t> payload)
    {
        CdrReader r(payload);
        CreateControllerRequest m;
        m.name = r.string();
        m.type = r.string();
        m.config = r.string();
        r.expect_end();
        return m;
    }

    std::vector<std::uint8_t> MessageResponse::serialize() const
    {
        CdrWriter w;
        w.u8(ok ? 1 : 0);
        w.string(message);
        return w.take();
    }

    MessageResponse MessageResponse::deserialize(std::span<const std::uint8_t> payload)
    {
        CdrReader r(payload);
        MessageResponse m;
        m.ok = r.u8() != 0;
        m.message = r.string();
        r.expect_end();
        return m;
    }

    std::vector<std::uint8_t> NameRequest::serialize() const
    {
        CdrWriter w;
        w.string(name);
        return w.take();
    }

    NameRequest NameRequest::deserialize(std::span<const std::uint8_t> payload)
    {
        CdrReader r(payload);
        NameRequest m;
        m.name = r.string();
        r.expect_end();
        return m;
    }

    std::vector<std::uint8_t> GetConfigFileRequest::serialize() const
    {
        CdrWriter w;
        w.string(type);
        w.string(name);
        return w.take();
    }

    GetConfigFileRequest GetConfigFileRequest::deserialize(std::span<const std::uint8_t> payload)
    {
        CdrReader r(payload);
        GetConfigFileRequest m;
        m.type = r.string();
        m.name = r.string();
        r.expect_end();
        return m;
    }

    std::vector<std::uint8_t> ConfigResponse::serialize() const
    {
        CdrWriter w;
        w.u8(ok ? 1 : 0);
        w.string(message);
        w.string(config);
        return w.take();
    }

    ConfigResponse ConfigResponse::deserialize(std::span<const std::uint8_t> payload)
    {
        CdrReader r(payload);
        ConfigResponse m;
        m.ok = r.u8() != 0;
        m.message = r.string();
        m.config = r.string();
        r.expect_end();
        return m;
    }

    // --------------------------------------------------------------------------------- rectification (rectify.hpp)

    RectifyCamera rectify_camera(const CameraInfo& info)
    {
        RectifyCamera c;
        c.width = info.width;
        c.height = info.height;
        c.k = info.k;
        c.distortion_model = info.distortion_model;
        c.d = info.d;
        c.r = info.r;
        bool zeros = true;
        for (const double v : info.r)
        {
            zeros = zeros and v == 0.0;
        }
        if (zeros)
        {
            c.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};  // a CameraInfo without R: no rotation
        }
        c.p = info.p;
        return c;
    }

    CameraInfo rectified_info(const CameraInfo& info)
    {
        bool zeros = true;
        for (const double v : info.p)
        {
            zeros = zeros and v == 0.0;
        }
        const auto& k = info.k;
        CameraInfo out = zeros ? CameraInfo::pinhole(info.width, info.height, k[0], k[4], k[2], k[5])
                               : CameraInfo::pinhole(info.width, info.height, info.p[0], info.p[5], info.p[2],
                                                     info.p[6]);
        out.header = info.header;
        if (not zeros)
        {
            out.p = info.p;
        }
        return out;
    }
}  // namespace xblick
