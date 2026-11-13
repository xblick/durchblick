// CUDA ring of Durchblick: image buffers in device memory, shared between processes on one host through CUDA virtual
// memory management (cuMemCreate with a POSIX file descriptor handle). The layout of the control block and the
// sequence lock are those of ring.hpp (kind 1: the control block holds no pixels).
//
// Device memory: one allocation of slot_count * slot_stride bytes (rounded up to the allocation granularity); slot i
// starts at i * slot_stride. Per slot an interprocess CUDA event is recorded on the writer's stream after the slot's
// copy, so a reader's stream waits for exactly that copy (cuStreamWaitEvent) instead of the whole device.
//
// Handshake: the writer listens on an abstract Unix socket "\0xblick-<16 hex>" (the descriptor's endpoint). A
// reader connects; the writer accepts only a peer of its own user id (SO_PEERCRED) and sends one message: the handshake
// record below plus the allocation's file descriptor (SCM_RIGHTS). The reader imports, maps and keeps it; the socket is
// closed after the handshake.
//
//   offset  size  field (little endian)
//        0     8  magic "DBLKCUDA"
//        8     4  version (1)
//       12     4  slot_count
//       16     8  slot_stride
//       24     8  allocation size
//       32    16  device UUID (cuDeviceGetUuid)
//       48    64  control block name, NUL-padded
//      112  64 n  interprocess event handle of slot i (CUipcEventHandle), n = slot_count
#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "xblick/ring.hpp"

namespace xblick
{
    struct DevicePool;  // a reader's stream-ordered memory pool (cuda_ring.cpp)

    // A device buffer owned by a reader (a copy of one frame), from the reader's memory pool. Freed when the last
    // reference goes, ordered on the legacy default stream: after the work of every blocking stream (PyTorch's default
    // stream among them). Work on a non-blocking stream that uses the buffer must be complete before it is released.
    class DeviceBuffer
    {
    public:
        DeviceBuffer(std::shared_ptr<DevicePool> pool, std::uint64_t nbytes, std::uint64_t stream);
        ~DeviceBuffer();
        DeviceBuffer(const DeviceBuffer&) = delete;
        DeviceBuffer& operator=(const DeviceBuffer&) = delete;

        // Copy the buffer to host memory (nbytes() bytes); returns when done.
        void download(void* dst) const;
        // Copy nbytes() bytes from host memory into the buffer; returns when done.
        void upload(const void* src);

        std::uint64_t ptr() const { return ptr_; }
        std::uint64_t nbytes() const { return nbytes_; }
        int device() const;

    private:
        friend std::shared_ptr<DeviceBuffer> make_device_buffer(int device, std::uint64_t nbytes);
        std::shared_ptr<DevicePool> pool_;  // keeps the pool and its context alive: the buffer may outlive the reader
        std::uint64_t ptr_ = 0;
        std::uint64_t nbytes_;
    };

    class CudaRingWriter
    {
    public:
        // device: CUDA ordinal of the device that holds the ring (where the frames are produced).
        CudaRingWriter(int device, std::uint32_t slot_count, std::uint64_t slot_size);
        ~CudaRingWriter();
        CudaRingWriter(const CudaRingWriter&) = delete;
        CudaRingWriter& operator=(const CudaRingWriter&) = delete;

        // Copy nbytes of device memory at src into the next slot, ordered after the work on stream (a CUstream handle;
        // 0 = the legacy default stream, which PyTorch uses by default). Does not wait for the copy.
        Written write_device(std::uint64_t src, std::uint64_t nbytes, std::uint64_t stream);
        // Wait until the frame written into slot is there (its copy or the work commit() named is done): the source
        // of write_device may then be freed or reused.
        void wait_written(std::uint32_t slot);
        // Wait for all work of this process on the ring's device (every stream of its primary context): for a source
        // whose stream is not known.
        void synchronize_device();
        // Copy nbytes of host memory into the next slot; returns when the copy is done (src may be reused).
        Written write_host(const void* src, std::uint64_t nbytes);
        // Zero-copy production: loan() returns the device pointer of the next slot (slot_size() bytes) to fill in
        // place, e.g. by a kernel; commit() publishes it once the work that fills it is enqueued on stream (a CUstream
        // handle, 0 = the legacy default stream). Does not wait for that work: readers' streams wait for it.
        std::uint64_t loan();
        Written commit(std::uint64_t nbytes, std::uint64_t stream);
        // Copy slot (as last written) to host memory; returns when done.
        void read_host(std::uint32_t slot, void* dst, std::uint64_t nbytes);
        // Page-lock host memory that read_host copies into (a host ring's slots), so that the GPU writes it directly
        // instead of through the driver's staging buffer; unlocked when this ring is destroyed, which must happen
        // before the memory is unmapped. Once per ring.
        void pin_host(void* ptr, std::uint64_t nbytes);

        const std::string& endpoint() const { return endpoint_; }
        std::string device_uuid() const;  // 32 hex digits
        int device() const { return device_; }
        std::uint32_t slot_count() const { return control_.slot_count(); }
        std::uint64_t slot_size() const { return control_.slot_size(); }

    private:
        struct Impl;
        int device_;
        ControlWriter control_;
        std::string endpoint_;
        std::int64_t loaned_ = -1;
        std::unique_ptr<Impl> impl_;
    };

    class CudaRingReader
    {
    public:
        // target_device: the CUDA ordinal on which copy() delivers its buffers; -1: the ring's device. With another
        // device than the ring's, copy() copies from GPU to GPU (cuMemcpyPeerAsync: over PCIe where the two devices
        // reach each other, else through host memory). The process must see the ring's device in either case.
        explicit CudaRingReader(const std::string& endpoint, int target_device = -1);
        ~CudaRingReader();
        CudaRingReader(const CudaRingReader&) = delete;
        CudaRingReader& operator=(const CudaRingReader&) = delete;

        // A copy of the frame (slot, seq) in a buffer of its own, or nullptr if the frame was overwritten before or
        // while copying. Synchronous: the buffer is complete when this returns.
        std::shared_ptr<DeviceBuffer> copy(std::uint32_t slot, std::uint64_t seq);

        // Zero copy: device pointer of the slot. Make stream wait for the slot's copy with wait(); after using it,
        // validate(slot, seq) tells whether the writer had begun overwriting it meanwhile.
        std::uint64_t slot_ptr(std::uint32_t slot) const;
        bool acquire(std::uint32_t slot, std::uint64_t seq) const;
        void wait(std::uint32_t slot, std::uint64_t stream) const;
        bool validate(std::uint32_t slot, std::uint64_t seq) const;
        std::uint64_t nbytes(std::uint32_t slot) const;

        // The device of copy()'s buffers (the target device).
        int device() const { return target_ < 0 ? device_ : target_; }
        // The device that holds the ring: slot_ptr() and wait() are of this device.
        int ring_device() const { return device_; }
        // With another target device: true if the two devices reach each other directly (copies go over PCIe), false
        // if the driver copies through host memory. False without another target device.
        bool direct() const { return direct_; }
        std::uint32_t slot_count() const;

    private:
        struct Impl;
        int device_ = -1;
        int target_ = -1;
        bool direct_ = false;
        std::unique_ptr<Impl> impl_;
    };

    // Number of CUDA devices; 0 if there is no driver (libcuda.so.1) or no device.
    int cuda_device_count();
    // The UUID (32 hex digits, as the info records' cuda_device_uuid) of the CUDA device of this ordinal (as
    // CUDA_VISIBLE_DEVICES numbers them; the UUID is the GPU's own); empty without a driver; RingError for an
    // ordinal there is no device of.
    std::string cuda_device_uuid(int ordinal);
    // The ordinal of the CUDA device with this UUID (32 hex digits), or -1 if this process sees no such device
    // (CUDA_VISIBLE_DEVICES hides it, or there is no driver).
    int cuda_device_ordinal(const std::string& uuid);

    // A device buffer of its own (e.g. for a driver that uploads frames, or tests).
    std::shared_ptr<DeviceBuffer> make_device_buffer(int device, std::uint64_t nbytes);
}  // namespace xblick
