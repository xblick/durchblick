#include "xblick/cuda_ring.hpp"

#include <cuda.h>
#include <dlfcn.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <array>
#include <bit>
#include <cstddef>
#include <atomic>
#include <cerrno>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

namespace xblick
{
    namespace
    {
        // ------------------------------------------------------------------------------------- driver API
        // libcuda.so.1 is opened at run time, so that the library loads on hosts without a GPU; the functions are
        // looked up with cuGetProcAddress for the CUDA 12.0 ABI (drivers from 525 on).

        constexpr int kDriverAbi = 12000;

#define XBLICK_CUDA_FUNCTIONS(X)                                                                                    \
    X(cuInit)                                                                                                        \
    X(cuGetErrorName)                                                                                                \
    X(cuGetErrorString)                                                                                              \
    X(cuDeviceGet)                                                                                                   \
    X(cuDeviceGetCount)                                                                                              \
    X(cuDeviceGetAttribute)                                                                                          \
    X(cuDeviceGetUuid)                                                                                               \
    X(cuDevicePrimaryCtxRetain)                                                                                      \
    X(cuDevicePrimaryCtxRelease)                                                                                     \
    X(cuCtxPushCurrent)                                                                                              \
    X(cuCtxPopCurrent)                                                                                               \
    X(cuCtxSynchronize)                                                                                               \
    X(cuMemGetAllocationGranularity)                                                                                 \
    X(cuMemCreate)                                                                                                   \
    X(cuMemRelease)                                                                                                  \
    X(cuMemAddressReserve)                                                                                           \
    X(cuMemAddressFree)                                                                                              \
    X(cuMemMap)                                                                                                      \
    X(cuMemUnmap)                                                                                                    \
    X(cuMemSetAccess)                                                                                                \
    X(cuMemExportToShareableHandle)                                                                                  \
    X(cuMemImportFromShareableHandle)                                                                                \
    X(cuMemAlloc)                                                                                                    \
    X(cuMemFree)                                                                                                     \
    X(cuMemPoolCreate)                                                                                               \
    X(cuMemPoolDestroy)                                                                                              \
    X(cuMemPoolSetAttribute)                                                                                         \
    X(cuMemAllocFromPoolAsync)                                                                                       \
    X(cuMemFreeAsync)                                                                                                \
    X(cuCtxEnablePeerAccess)                                                                                              \
    X(cuMemcpyPeerAsync)                                                                                                  \
    X(cuMemcpyDtoDAsync)                                                                                             \
    X(cuMemcpyHtoDAsync)                                                                                             \
    X(cuMemcpyDtoHAsync)                                                                                             \
    X(cuMemHostRegister)                                                                                             \
    X(cuMemHostUnregister)                                                                                           \
    X(cuEventCreate)                                                                                                 \
    X(cuEventDestroy)                                                                                                \
    X(cuEventRecord)                                                                                                 \
    X(cuIpcGetEventHandle)                                                                                           \
    X(cuIpcOpenEventHandle)                                                                                          \
    X(cuStreamCreate)                                                                                                \
    X(cuStreamDestroy)                                                                                               \
    X(cuStreamSynchronize)                                                                                           \
    X(cuStreamWaitEvent)

        struct Driver
        {
#define XBLICK_MEMBER(name) decltype(&::name) name = nullptr;
            XBLICK_CUDA_FUNCTIONS(XBLICK_MEMBER)
#undef XBLICK_MEMBER
            std::string error;  // why the driver is not available
        };

        Driver load_driver()
        {
            Driver d;
            void* lib = ::dlopen("libcuda.so.1", RTLD_NOW | RTLD_LOCAL);
            if (lib == nullptr)
            {
                d.error = std::string("cannot load the CUDA driver libcuda.so.1: ") + ::dlerror();
                return d;
            }
            using GetProcAddress = CUresult (*)(const char*, void**, int, cuuint64_t, CUdriverProcAddressQueryResult*);
            auto get = reinterpret_cast<GetProcAddress>(::dlsym(lib, "cuGetProcAddress_v2"));
            if (get == nullptr)
            {
                d.error = "the CUDA driver has no cuGetProcAddress_v2 (needs a driver for CUDA 12 or newer)";
                return d;
            }
#define XBLICK_LOOKUP(name)                                                                                           \
    {                                                                                                                 \
        void* fn = nullptr;                                                                                           \
        CUdriverProcAddressQueryResult status{};                                                                      \
        if (get(#name, &fn, kDriverAbi, CU_GET_PROC_ADDRESS_DEFAULT, &status) != CUDA_SUCCESS or fn == nullptr)       \
        {                                                                                                             \
            d.error = "the CUDA driver lacks " #name;                                                                 \
            return d;                                                                                                 \
        }                                                                                                             \
        d.name = reinterpret_cast<decltype(d.name)>(fn);                                                              \
    }
            XBLICK_CUDA_FUNCTIONS(XBLICK_LOOKUP)
#undef XBLICK_LOOKUP
            if (d.cuInit(0) != CUDA_SUCCESS)
            {
                d = Driver{};
                d.error = "cuInit failed (no usable CUDA device?)";
            }
            return d;
        }

        const Driver& driver_or_error(std::string* error)
        {
            static const Driver d = load_driver();
            if (error != nullptr)
            {
                *error = d.error;
            }
            return d;
        }

        const Driver& driver()
        {
            std::string error;
            const Driver& d = driver_or_error(&error);
            if (not error.empty())
            {
                throw RingError(error);
            }
            return d;
        }

        void check(CUresult result, const char* what)
        {
            if (result == CUDA_SUCCESS)
            {
                return;
            }
            const char* name = nullptr;
            const char* text = nullptr;
            driver().cuGetErrorName(result, &name);
            driver().cuGetErrorString(result, &text);
            throw RingError(std::string(what) + ": " + (name ? name : "?") + " (" + (text ? text : "?") + ")");
        }

#define CU(call) check(driver().call, #call)

        // The device's primary context (shared with PyTorch, Warp and the runtime API), current while it exists.
        class ContextGuard
        {
        public:
            explicit ContextGuard(CUcontext context)
            {
                CU(cuCtxPushCurrent(context));
            }
            ~ContextGuard()
            {
                CUcontext popped = nullptr;
                driver().cuCtxPopCurrent(&popped);
            }
        };

        class PrimaryContext
        {
        public:
            explicit PrimaryContext(int ordinal)
            {
                CU(cuDeviceGet(&device, ordinal));
                CU(cuDevicePrimaryCtxRetain(&context, device));
            }
            ~PrimaryContext()
            {
                driver().cuDevicePrimaryCtxRelease(device);
            }
            CUdevice device{};
            CUcontext context{};
        };

        std::array<unsigned char, 16> uuid_of(CUdevice device)
        {
            CUuuid uuid{};
            CU(cuDeviceGetUuid(&uuid, device));
            std::array<unsigned char, 16> bytes{};
            std::memcpy(bytes.data(), uuid.bytes, 16);
            return bytes;
        }

        std::string hex(const unsigned char* data, std::size_t n)
        {
            static const char* digits = "0123456789abcdef";
            std::string s;
            for (std::size_t i = 0; i < n; ++i)
            {
                s += digits[data[i] >> 4];
                s += digits[data[i] & 0xF];
            }
            return s;
        }

        std::size_t round_up(std::size_t value, std::size_t multiple)
        {
            return (value + multiple - 1) / multiple * multiple;
        }

        // ------------------------------------------------------------------------------------- handshake

        constexpr char kHandshakeMagic[8] = {'D', 'B', 'L', 'K', 'C', 'U', 'D', 'A'};
        constexpr std::size_t kHandshakeHeader = 112;
        constexpr std::size_t kEventHandleSize = sizeof(CUipcEventHandle);
        static_assert(kEventHandleSize == 64);

        sockaddr_un abstract_address(const std::string& endpoint, socklen_t* length)
        {
            sockaddr_un address{};
            address.sun_family = AF_UNIX;
            if (endpoint.size() + 1 > sizeof(address.sun_path))
            {
                throw RingError("endpoint name too long: " + endpoint);
            }
            address.sun_path[0] = '\0';  // abstract namespace: no file, gone with the socket
            std::memcpy(address.sun_path + 1, endpoint.data(), endpoint.size());
            *length = static_cast<socklen_t>(offsetof(sockaddr_un, sun_path) + 1 + endpoint.size());
            return address;
        }

        void put_u32(std::vector<unsigned char>& b, std::size_t at, std::uint32_t v)
        {
            std::memcpy(b.data() + at, &v, 4);
        }
        void put_u64(std::vector<unsigned char>& b, std::size_t at, std::uint64_t v)
        {
            std::memcpy(b.data() + at, &v, 8);
        }
        std::uint32_t get_u32(const unsigned char* b)
        {
            std::uint32_t v;
            std::memcpy(&v, b, 4);
            return v;
        }
        std::uint64_t get_u64(const unsigned char* b)
        {
            std::uint64_t v;
            std::memcpy(&v, b, 8);
            return v;
        }

        static_assert(std::endian::native == std::endian::little, "the ring layout is little endian");
    }  // namespace

    int cuda_device_count()
    {
        std::string error;
        const Driver& d = driver_or_error(&error);
        if (not error.empty())
        {
            return 0;
        }
        int count = 0;
        return d.cuDeviceGetCount(&count) == CUDA_SUCCESS ? count : 0;
    }

    std::string cuda_device_uuid(int ordinal)
    {
        std::string error;
        driver_or_error(&error);
        if (not error.empty())
        {
            return "";
        }
        CUdevice device;
        CU(cuDeviceGet(&device, ordinal));
        const auto uuid = uuid_of(device);
        return hex(uuid.data(), uuid.size());
    }

    int cuda_device_ordinal(const std::string& uuid)
    {
        const int count = cuda_device_count();
        for (int i = 0; i < count; ++i)
        {
            if (cuda_device_uuid(i) == uuid)
            {
                return i;
            }
        }
        return -1;
    }

    // ------------------------------------------------------------------------------------------ DeviceBuffer

    struct DevicePool
    {
        explicit DevicePool(int ordinal) : ordinal(ordinal), primary(ordinal)
        {
            ContextGuard guard(primary.context);
            CUmemPoolProps props{};
            props.allocType = CU_MEM_ALLOCATION_TYPE_PINNED;
            props.location.type = CU_MEM_LOCATION_TYPE_DEVICE;
            props.location.id = primary.device;
            CU(cuMemPoolCreate(&pool, &props));
            // keep freed memory in the pool: frames of one stream have one size, so it is reused at once
            cuuint64_t keep = ~cuuint64_t{0};
            CU(cuMemPoolSetAttribute(pool, CU_MEMPOOL_ATTR_RELEASE_THRESHOLD, &keep));
        }
        ~DevicePool()
        {
            ContextGuard guard(primary.context);
            driver().cuMemPoolDestroy(pool);  // deferred by the driver until the last buffer's free has run
        }
        int ordinal;
        PrimaryContext primary;
        CUmemoryPool pool{};
    };

    DeviceBuffer::DeviceBuffer(std::shared_ptr<DevicePool> pool, std::uint64_t nbytes, std::uint64_t stream)
        : pool_(std::move(pool)), nbytes_(nbytes)
    {
        ContextGuard guard(pool_->primary.context);
        CUdeviceptr ptr = 0;
        CU(cuMemAllocFromPoolAsync(&ptr, nbytes == 0 ? 1 : nbytes, pool_->pool, reinterpret_cast<CUstream>(stream)));
        ptr_ = ptr;
    }

    DeviceBuffer::~DeviceBuffer()
    {
        if (ptr_ != 0)
        {
            ContextGuard guard(pool_->primary.context);
            driver().cuMemFreeAsync(ptr_, CU_STREAM_LEGACY);
        }
    }

    int DeviceBuffer::device() const
    {
        return pool_->ordinal;
    }

    void DeviceBuffer::upload(const void* src)
    {
        ContextGuard guard(pool_->primary.context);
        CU(cuMemcpyHtoDAsync(ptr_, src, nbytes_, CU_STREAM_LEGACY));
        CU(cuStreamSynchronize(CU_STREAM_LEGACY));
    }

    std::shared_ptr<DeviceBuffer> make_device_buffer(int device, std::uint64_t nbytes)
    {
        auto buffer = std::make_shared<DeviceBuffer>(std::make_shared<DevicePool>(device), nbytes,
                                                     reinterpret_cast<std::uint64_t>(CU_STREAM_LEGACY));
        ContextGuard guard(buffer->pool_->primary.context);
        CU(cuStreamSynchronize(CU_STREAM_LEGACY));  // the allocation is complete before anybody uses it
        return buffer;
    }

    void DeviceBuffer::download(void* dst) const
    {
        ContextGuard guard(pool_->primary.context);
        CU(cuMemcpyDtoHAsync(dst, ptr_, nbytes_, CU_STREAM_LEGACY));
        CU(cuStreamSynchronize(CU_STREAM_LEGACY));
    }

    // ---------------------------------------------------------------------------------------------- writer

    struct CudaRingWriter::Impl
    {
        explicit Impl(int ordinal) : primary(ordinal)
        {
        }

        PrimaryContext primary;
        CUmemGenericAllocationHandle allocation{};
        CUdeviceptr base = 0;
        std::size_t size = 0;
        std::uint64_t stride = 0;
        int export_fd = -1;
        std::vector<CUevent> events;
        std::vector<CUipcEventHandle> event_handles;
        CUstream stream{};  // for host copies
        void* pinned = nullptr;  // host memory page-locked by pin_host
        std::array<unsigned char, 16> uuid{};
        std::vector<unsigned char> handshake;

        int listen_fd = -1;
        std::atomic<bool> stop{false};
        std::thread server;

        void serve()
        {
            while (not stop.load())
            {
                pollfd p{listen_fd, POLLIN, 0};
                if (::poll(&p, 1, 200) <= 0)
                {
                    continue;
                }
                const int connection = ::accept4(listen_fd, nullptr, nullptr, SOCK_CLOEXEC);
                if (connection < 0)
                {
                    continue;
                }
                ucred peer{};
                socklen_t length = sizeof(peer);
                if (::getsockopt(connection, SOL_SOCKET, SO_PEERCRED, &peer, &length) == 0 and peer.uid == ::getuid())
                {
                    iovec io{handshake.data(), handshake.size()};
                    alignas(cmsghdr) char control[CMSG_SPACE(sizeof(int))] = {};
                    msghdr message{};
                    message.msg_iov = &io;
                    message.msg_iovlen = 1;
                    message.msg_control = control;
                    message.msg_controllen = sizeof(control);
                    cmsghdr* c = CMSG_FIRSTHDR(&message);
                    c->cmsg_level = SOL_SOCKET;
                    c->cmsg_type = SCM_RIGHTS;
                    c->cmsg_len = CMSG_LEN(sizeof(int));
                    std::memcpy(CMSG_DATA(c), &export_fd, sizeof(int));
                    ::sendmsg(connection, &message, MSG_NOSIGNAL);
                }
                ::close(connection);  // a peer of another user gets nothing
            }
        }
    };

    CudaRingWriter::CudaRingWriter(int device, std::uint32_t slot_count, std::uint64_t slot_size)
        : device_(device),
          control_(RingKind::Cuda, slot_count, round_up(slot_size, 512), 0),
          endpoint_(random_name("xblick-")),
          impl_(std::make_unique<Impl>(device))
    {
        Impl& m = *impl_;
        ContextGuard guard(m.primary.context);
        int vmm = 0;
        int posix_fd = 0;
        CU(cuDeviceGetAttribute(&vmm, CU_DEVICE_ATTRIBUTE_VIRTUAL_MEMORY_MANAGEMENT_SUPPORTED, m.primary.device));
        CU(cuDeviceGetAttribute(&posix_fd, CU_DEVICE_ATTRIBUTE_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR_SUPPORTED,
                                m.primary.device));
        if (not vmm or not posix_fd)
        {
            throw RingError("CUDA device " + std::to_string(device) + " does not support virtual memory management "
                            "with POSIX file descriptors");
        }
        m.uuid = uuid_of(m.primary.device);

        CUmemAllocationProp prop{};
        prop.type = CU_MEM_ALLOCATION_TYPE_PINNED;
        prop.location.type = CU_MEM_LOCATION_TYPE_DEVICE;
        prop.location.id = m.primary.device;
        prop.requestedHandleTypes = CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR;
        std::size_t granularity = 0;
        CU(cuMemGetAllocationGranularity(&granularity, &prop, CU_MEM_ALLOC_GRANULARITY_RECOMMENDED));
        m.stride = control_.slot_size();
        m.size = round_up(m.stride * slot_count, granularity);
        CU(cuMemCreate(&m.allocation, m.size, &prop, 0));
        CU(cuMemAddressReserve(&m.base, m.size, granularity, 0, 0));
        CU(cuMemMap(m.base, m.size, 0, m.allocation, 0));
        CUmemAccessDesc access{};
        access.location = prop.location;
        access.flags = CU_MEM_ACCESS_FLAGS_PROT_READWRITE;
        CU(cuMemSetAccess(m.base, m.size, &access, 1));
        CU(cuMemExportToShareableHandle(&m.export_fd, m.allocation, CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR, 0));

        m.events.resize(slot_count);
        m.event_handles.resize(slot_count);
        for (std::uint32_t i = 0; i < slot_count; ++i)
        {
            CU(cuEventCreate(&m.events[i], CU_EVENT_DISABLE_TIMING | CU_EVENT_INTERPROCESS));
            CU(cuIpcGetEventHandle(&m.event_handles[i], m.events[i]));
        }
        CU(cuStreamCreate(&m.stream, CU_STREAM_NON_BLOCKING));

        m.handshake.assign(kHandshakeHeader + kEventHandleSize * slot_count, 0);
        std::memcpy(m.handshake.data(), kHandshakeMagic, 8);
        put_u32(m.handshake, 8, kVersion);
        put_u32(m.handshake, 12, slot_count);
        put_u64(m.handshake, 16, m.stride);
        put_u64(m.handshake, 24, m.size);
        std::memcpy(m.handshake.data() + 32, m.uuid.data(), 16);
        if (control_.name().size() >= 64)
        {
            throw RingError("control block name too long: " + control_.name());
        }
        std::memcpy(m.handshake.data() + 48, control_.name().data(), control_.name().size());
        for (std::uint32_t i = 0; i < slot_count; ++i)
        {
            std::memcpy(m.handshake.data() + kHandshakeHeader + kEventHandleSize * i, &m.event_handles[i],
                        kEventHandleSize);
        }

        m.listen_fd = ::socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
        if (m.listen_fd < 0)
        {
            throw RingError(std::string("socket: ") + std::strerror(errno));
        }
        socklen_t length = 0;
        const sockaddr_un address = abstract_address(endpoint_, &length);
        if (::bind(m.listen_fd, reinterpret_cast<const sockaddr*>(&address), length) != 0 or
            ::listen(m.listen_fd, 16) != 0)
        {
            throw RingError("cannot listen on @" + endpoint_ + ": " + std::strerror(errno));
        }
        m.server = std::thread([&m] { m.serve(); });
    }

    CudaRingWriter::~CudaRingWriter()
    {
        Impl& m = *impl_;
        m.stop.store(true);
        if (m.server.joinable())
        {
            m.server.join();
        }
        if (m.listen_fd >= 0)
        {
            ::close(m.listen_fd);
        }
        ContextGuard guard(m.primary.context);
        if (m.stream != nullptr)
        {
            driver().cuStreamSynchronize(m.stream);
            driver().cuStreamDestroy(m.stream);
        }
        if (m.pinned != nullptr)
        {
            driver().cuMemHostUnregister(m.pinned);
        }
        for (CUevent e : m.events)
        {
            driver().cuEventDestroy(e);
        }
        if (m.export_fd >= 0)
        {
            ::close(m.export_fd);
        }
        if (m.base != 0)
        {
            driver().cuMemUnmap(m.base, m.size);
            driver().cuMemAddressFree(m.base, m.size);
        }
        if (m.allocation != 0)
        {
            driver().cuMemRelease(m.allocation);  // readers that imported it keep the memory until they unmap
        }
    }

    std::string CudaRingWriter::device_uuid() const
    {
        return hex(impl_->uuid.data(), 16);
    }

    Written CudaRingWriter::write_device(std::uint64_t src, std::uint64_t nbytes, std::uint64_t stream)
    {
        if (nbytes > control_.slot_size())
        {
            throw RingError(std::to_string(nbytes) + " bytes do not fit a slot of " +
                            std::to_string(control_.slot_size()));
        }
        Impl& m = *impl_;
        ContextGuard guard(m.primary.context);
        const std::uint32_t slot = control_.begin();
        auto s = reinterpret_cast<CUstream>(stream);
        try
        {
            CU(cuMemcpyDtoDAsync(m.base + slot * m.stride, src, nbytes, s));
            CU(cuEventRecord(m.events[slot], s));
        }
        catch (...)
        {
            control_.commit(0);
            throw;
        }
        return {slot, control_.commit(nbytes)};
    }

    Written CudaRingWriter::write_host(const void* src, std::uint64_t nbytes)
    {
        if (nbytes > control_.slot_size())
        {
            throw RingError(std::to_string(nbytes) + " bytes do not fit a slot of " +
                            std::to_string(control_.slot_size()));
        }
        Impl& m = *impl_;
        ContextGuard guard(m.primary.context);
        const std::uint32_t slot = control_.begin();
        try
        {
            CU(cuMemcpyHtoDAsync(m.base + slot * m.stride, src, nbytes, m.stream));
            CU(cuEventRecord(m.events[slot], m.stream));
            CU(cuStreamSynchronize(m.stream));
        }
        catch (...)
        {
            control_.commit(0);
            throw;
        }
        return {slot, control_.commit(nbytes)};
    }

    std::uint64_t CudaRingWriter::loan()
    {
        if (loaned_ >= 0)
        {
            throw RingError("loan: the slot loaned before has not been committed");
        }
        loaned_ = control_.begin();
        return impl_->base + static_cast<std::uint64_t>(loaned_) * impl_->stride;
    }

    Written CudaRingWriter::commit(std::uint64_t nbytes, std::uint64_t stream)
    {
        if (loaned_ < 0)
        {
            throw RingError("commit: no slot is loaned");
        }
        const auto slot = static_cast<std::uint32_t>(loaned_);
        loaned_ = -1;
        if (nbytes > control_.slot_size())
        {
            control_.commit(0);
            throw RingError(std::to_string(nbytes) + " bytes do not fit a slot of " +
                            std::to_string(control_.slot_size()));
        }
        Impl& m = *impl_;
        ContextGuard guard(m.primary.context);
        try
        {
            CU(cuEventRecord(m.events[slot], reinterpret_cast<CUstream>(stream)));
        }
        catch (...)
        {
            control_.commit(0);
            throw;
        }
        return {slot, control_.commit(nbytes)};
    }

    void CudaRingWriter::pin_host(void* ptr, std::uint64_t nbytes)
    {
        Impl& m = *impl_;
        if (m.pinned != nullptr)
        {
            throw RingError("pin_host: this ring has pinned host memory already");
        }
        ContextGuard guard(m.primary.context);
        CU(cuMemHostRegister(ptr, nbytes, CU_MEMHOSTREGISTER_PORTABLE));
        m.pinned = ptr;
    }

    void CudaRingWriter::wait_written(std::uint32_t slot)
    {
        if (slot >= control_.slot_count())
        {
            throw RingError("wait_written: slot " + std::to_string(slot));
        }
        Impl& m = *impl_;
        ContextGuard guard(m.primary.context);
        CU(cuStreamWaitEvent(m.stream, m.events[slot], 0));
        CU(cuStreamSynchronize(m.stream));
    }

    void CudaRingWriter::synchronize_device()
    {
        ContextGuard guard(impl_->primary.context);
        CU(cuCtxSynchronize());
    }

    void CudaRingWriter::read_host(std::uint32_t slot, void* dst, std::uint64_t nbytes)
    {
        if (slot >= control_.slot_count() or nbytes > control_.slot_size())
        {
            throw RingError("read_host: slot " + std::to_string(slot) + ", " + std::to_string(nbytes) + " bytes");
        }
        Impl& m = *impl_;
        ContextGuard guard(m.primary.context);
        CU(cuStreamWaitEvent(m.stream, m.events[slot], 0));
        CU(cuMemcpyDtoHAsync(dst, m.base + slot * m.stride, nbytes, m.stream));
        CU(cuStreamSynchronize(m.stream));
    }

    // ---------------------------------------------------------------------------------------------- reader

    struct CudaRingReader::Impl
    {
        std::unique_ptr<PrimaryContext> primary;
        std::shared_ptr<DevicePool> pool;
        std::unique_ptr<ControlReader> control;
        CUmemGenericAllocationHandle allocation{};
        CUdeviceptr base = 0;
        std::size_t size = 0;
        std::uint64_t stride = 0;
        std::uint32_t slot_count = 0;
        std::vector<CUevent> events;
        CUstream stream{};
        CUstream target_stream{};  // of the target device's context, for its allocations (another device only)
        std::mutex mutex;  // copy() from several threads shares the stream
    };

    CudaRingReader::CudaRingReader(const std::string& endpoint, int target_device)
        : target_(target_device), impl_(std::make_unique<Impl>())
    {
        Impl& m = *impl_;
        const int fd = ::socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
        if (fd < 0)
        {
            throw RingError(std::string("socket: ") + std::strerror(errno));
        }
        socklen_t length = 0;
        const sockaddr_un address = abstract_address(endpoint, &length);
        if (::connect(fd, reinterpret_cast<const sockaddr*>(&address), length) != 0)
        {
            const std::string message = std::string("cannot connect to the CUDA ring @") + endpoint + ": " +
                                        std::strerror(errno) + " (is the publisher on this host and still running?)";
            ::close(fd);
            throw RingError(message);
        }
        std::vector<unsigned char> buffer(kHandshakeHeader + kEventHandleSize * kMaxSlots);
        iovec io{buffer.data(), buffer.size()};
        alignas(cmsghdr) char control[CMSG_SPACE(sizeof(int))] = {};
        msghdr message{};
        message.msg_iov = &io;
        message.msg_iovlen = 1;
        message.msg_control = control;
        message.msg_controllen = sizeof(control);
        const ssize_t received = ::recvmsg(fd, &message, MSG_CMSG_CLOEXEC);
        ::close(fd);
        int memory_fd = -1;
        for (cmsghdr* c = CMSG_FIRSTHDR(&message); c != nullptr; c = CMSG_NXTHDR(&message, c))
        {
            if (c->cmsg_level == SOL_SOCKET and c->cmsg_type == SCM_RIGHTS)
            {
                std::memcpy(&memory_fd, CMSG_DATA(c), sizeof(int));
            }
        }
        if (received < static_cast<ssize_t>(kHandshakeHeader) or memory_fd < 0)
        {
            if (memory_fd >= 0)
            {
                ::close(memory_fd);
            }
            throw RingError("no handshake from the CUDA ring @" + endpoint + " (a publisher of another user?)");
        }
        struct FdCloser
        {
            int fd;
            ~FdCloser() { ::close(fd); }
        } closer{memory_fd};

        const unsigned char* b = buffer.data();
        if (std::memcmp(b, kHandshakeMagic, 8) != 0 or get_u32(b + 8) != kVersion)
        {
            throw RingError("@" + endpoint + " is not a Durchblick CUDA ring of version " + std::to_string(kVersion));
        }
        m.slot_count = get_u32(b + 12);
        m.stride = get_u64(b + 16);
        m.size = get_u64(b + 24);
        if (m.slot_count < 2 or m.slot_count > kMaxSlots or
            static_cast<std::size_t>(received) != kHandshakeHeader + kEventHandleSize * m.slot_count)
        {
            throw RingError("@" + endpoint + ": malformed handshake");
        }
        std::array<unsigned char, 16> uuid{};
        std::memcpy(uuid.data(), b + 32, 16);
        const std::string control_name(reinterpret_cast<const char*>(b + 48),
                                       ::strnlen(reinterpret_cast<const char*>(b + 48), 64));

        int count = 0;
        CU(cuDeviceGetCount(&count));
        for (int i = 0; i < count and device_ < 0; ++i)
        {
            CUdevice d{};
            CU(cuDeviceGet(&d, i));
            if (uuid_of(d) == uuid)
            {
                device_ = i;
            }
        }
        if (device_ < 0)
        {
            throw RingError("the CUDA ring @" + endpoint + " is on device " + hex(uuid.data(), 16) +
                            ", which this process does not see (CUDA_VISIBLE_DEVICES?)");
        }
        m.primary = std::make_unique<PrimaryContext>(device_);
        ContextGuard guard(m.primary->context);
        CU(cuMemImportFromShareableHandle(&m.allocation, reinterpret_cast<void*>(static_cast<std::intptr_t>(memory_fd)),
                                          CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR));
        CU(cuMemAddressReserve(&m.base, m.size, 0, 0, 0));
        CU(cuMemMap(m.base, m.size, 0, m.allocation, 0));
        CUmemAccessDesc access{};
        access.location.type = CU_MEM_LOCATION_TYPE_DEVICE;
        access.location.id = m.primary->device;
        access.flags = CU_MEM_ACCESS_FLAGS_PROT_READ;
        CU(cuMemSetAccess(m.base, m.size, &access, 1));
        m.events.resize(m.slot_count);
        for (std::uint32_t i = 0; i < m.slot_count; ++i)
        {
            CUipcEventHandle handle{};
            std::memcpy(&handle, b + kHandshakeHeader + kEventHandleSize * i, kEventHandleSize);
            CU(cuIpcOpenEventHandle(&m.events[i], handle));
        }
        CU(cuStreamCreate(&m.stream, CU_STREAM_NON_BLOCKING));
        if (target_ == device_)
        {
            target_ = -1;  // the ring's own device: no copy between devices
        }
        m.pool = std::make_shared<DevicePool>(device());
        if (target_ >= 0)
        {
            // Direct access between the two devices, in both directions, so that copy() goes from GPU to GPU over
            // PCIe. Where the devices cannot reach each other (CUDA_ERROR_PEER_ACCESS_UNSUPPORTED), the driver copies
            // through host memory instead: slower, the same result.
            const auto enable = [](CUcontext peer) {
                const CUresult r = driver().cuCtxEnablePeerAccess(peer, 0);
                if (r != CUDA_ERROR_PEER_ACCESS_ALREADY_ENABLED and r != CUDA_ERROR_PEER_ACCESS_UNSUPPORTED)
                {
                    check(r, "cuCtxEnablePeerAccess");
                }
                return r != CUDA_ERROR_PEER_ACCESS_UNSUPPORTED;
            };
            direct_ = enable(m.pool->primary.context);  // the ring's context is current
            {
                ContextGuard target(m.pool->primary.context);
                direct_ = enable(m.primary->context) and direct_;
                CU(cuStreamCreate(&m.target_stream, CU_STREAM_NON_BLOCKING));
            }
            if (direct_)  // the mapping of the ring is readable from the target device as well
            {
                CUmemAccessDesc peer{};
                peer.location.type = CU_MEM_LOCATION_TYPE_DEVICE;
                peer.location.id = m.pool->primary.device;
                peer.flags = CU_MEM_ACCESS_FLAGS_PROT_READ;
                CU(cuMemSetAccess(m.base, m.size, &peer, 1));
            }
        }
        m.control = std::make_unique<ControlReader>(control_name, RingKind::Cuda);
        if (m.control->slot_count() != m.slot_count or m.control->slot_size() != m.stride)
        {
            throw RingError("@" + endpoint + ": control block and handshake disagree");
        }
    }

    CudaRingReader::~CudaRingReader()
    {
        Impl& m = *impl_;
        if (m.primary == nullptr)
        {
            return;
        }
        if (m.target_stream != nullptr)
        {
            ContextGuard target(m.pool->primary.context);
            driver().cuStreamSynchronize(m.target_stream);
            driver().cuStreamDestroy(m.target_stream);
        }
        ContextGuard guard(m.primary->context);
        if (m.stream != nullptr)
        {
            driver().cuStreamSynchronize(m.stream);
            driver().cuStreamDestroy(m.stream);
        }
        for (CUevent e : m.events)
        {
            driver().cuEventDestroy(e);
        }
        if (m.base != 0)
        {
            driver().cuMemUnmap(m.base, m.size);
            driver().cuMemAddressFree(m.base, m.size);
        }
        if (m.allocation != 0)
        {
            driver().cuMemRelease(m.allocation);
        }
    }

    std::uint32_t CudaRingReader::slot_count() const
    {
        return impl_->slot_count;
    }

    std::uint64_t CudaRingReader::slot_ptr(std::uint32_t slot) const
    {
        if (slot >= impl_->slot_count)
        {
            throw RingError("slot " + std::to_string(slot) + " of " + std::to_string(impl_->slot_count));
        }
        return impl_->base + slot * impl_->stride;
    }

    bool CudaRingReader::acquire(std::uint32_t slot, std::uint64_t seq) const
    {
        return impl_->control->acquire(slot, seq);
    }

    bool CudaRingReader::validate(std::uint32_t slot, std::uint64_t seq) const
    {
        return impl_->control->validate(slot, seq);
    }

    std::uint64_t CudaRingReader::nbytes(std::uint32_t slot) const
    {
        return impl_->control->nbytes(slot);
    }

    void CudaRingReader::wait(std::uint32_t slot, std::uint64_t stream) const
    {
        ContextGuard guard(impl_->primary->context);
        CU(cuStreamWaitEvent(reinterpret_cast<CUstream>(stream), impl_->events.at(slot), 0));
    }

    std::shared_ptr<DeviceBuffer> CudaRingReader::copy(std::uint32_t slot, std::uint64_t seq)
    {
        Impl& m = *impl_;
        const std::uint64_t src = slot_ptr(slot);
        if (not m.control->acquire(slot, seq))
        {
            return nullptr;
        }
        const std::uint64_t n = m.control->nbytes(slot);
        if (n > m.stride)
        {
            return nullptr;  // nbytes of a newer frame being written: validate would fail
        }
        std::lock_guard lock(m.mutex);
        if (target_ >= 0)  // onto another device: the buffer is of that device's context, the copy of the ring's
        {
            auto buffer = std::make_shared<DeviceBuffer>(m.pool, n, reinterpret_cast<std::uint64_t>(m.target_stream));
            {
                ContextGuard target(m.pool->primary.context);
                CU(cuStreamSynchronize(m.target_stream));  // the allocation is complete before the other context writes
            }
            ContextGuard guard(m.primary->context);
            CU(cuStreamWaitEvent(m.stream, m.events[slot], 0));
            CU(cuMemcpyPeerAsync(buffer->ptr(), m.pool->primary.context, src, m.primary->context, n, m.stream));
            CU(cuStreamSynchronize(m.stream));
            return m.control->validate(slot, seq) ? buffer : nullptr;
        }
        ContextGuard guard(m.primary->context);
        auto buffer = std::make_shared<DeviceBuffer>(m.pool, n, reinterpret_cast<std::uint64_t>(m.stream));
        // the slot's event was recorded after the frame's copy; if the writer re-recorded it for a newer frame,
        // validate() below fails anyway
        CU(cuStreamWaitEvent(m.stream, m.events[slot], 0));
        CU(cuMemcpyDtoDAsync(buffer->ptr(), src, n, m.stream));
        CU(cuStreamSynchronize(m.stream));
        if (not m.control->validate(slot, seq))
        {
            return nullptr;
        }
        return buffer;
    }
}  // namespace xblick
