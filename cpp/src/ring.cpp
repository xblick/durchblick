#include "xblick/ring.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <fstream>
#include <random>
#include <utility>

namespace xblick
{
    namespace
    {
        std::string errno_text(const std::string& what)
        {
            return what + ": " + std::strerror(errno);
        }

        std::size_t round_up(std::size_t value, std::size_t multiple)
        {
            return (value + multiple - 1) / multiple * multiple;
        }
    }  // namespace

    // ------------------------------------------------------------------------------------------------ Mapping

    Mapping Mapping::create(const std::string& name, std::size_t size)
    {
        const int fd = ::shm_open(name.c_str(), O_CREAT | O_EXCL | O_RDWR, 0600);
        if (fd < 0)
        {
            throw RingError(errno_text("shm_open(" + name + ") for writing"));
        }
        if (::ftruncate(fd, static_cast<off_t>(size)) != 0)
        {
            const std::string message = errno_text("ftruncate(" + name + ", " + std::to_string(size) + ")");
            ::close(fd);
            ::shm_unlink(name.c_str());
            throw RingError(message);
        }
        void* data = ::mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        ::close(fd);
        if (data == MAP_FAILED)
        {
            const std::string message = errno_text("mmap(" + name + ")");
            ::shm_unlink(name.c_str());
            throw RingError(message);
        }
        Mapping m;
        m.name_ = name;
        m.data_ = static_cast<std::uint8_t*>(data);
        m.size_ = size;
        m.owner_ = true;
        return m;
    }

    Mapping Mapping::open_readonly(const std::string& name)
    {
        const int fd = ::shm_open(name.c_str(), O_RDONLY, 0);
        if (fd < 0)
        {
            throw RingError(errno_text("shm_open(" + name + ") for reading (is the publisher on this host and "
                                       "still running?)"));
        }
        struct stat st{};
        if (::fstat(fd, &st) != 0)
        {
            const std::string message = errno_text("fstat(" + name + ")");
            ::close(fd);
            throw RingError(message);
        }
        const auto size = static_cast<std::size_t>(st.st_size);
        if (size < kControlSize)
        {
            ::close(fd);
            throw RingError(name + ": " + std::to_string(size) + " bytes, smaller than a control block");
        }
        void* data = ::mmap(nullptr, size, PROT_READ, MAP_SHARED, fd, 0);
        ::close(fd);
        if (data == MAP_FAILED)
        {
            throw RingError(errno_text("mmap(" + name + ")"));
        }
        Mapping m;
        m.name_ = name;
        m.data_ = static_cast<std::uint8_t*>(data);
        m.size_ = size;
        return m;
    }

    Mapping::Mapping(Mapping&& other) noexcept
        : name_(std::move(other.name_)),
          data_(std::exchange(other.data_, nullptr)),
          size_(std::exchange(other.size_, 0)),
          owner_(std::exchange(other.owner_, false))
    {
    }

    Mapping& Mapping::operator=(Mapping&& other) noexcept
    {
        if (this != &other)
        {
            this->~Mapping();
            new (this) Mapping(std::move(other));
        }
        return *this;
    }

    Mapping::~Mapping()
    {
        if (data_ != nullptr)
        {
            ::munmap(data_, size_);
        }
        if (owner_)
        {
            ::shm_unlink(name_.c_str());
        }
    }

    std::string random_name(const std::string& prefix)
    {
        std::random_device device;
        std::uint64_t value = (static_cast<std::uint64_t>(device()) << 32) ^ device();
        static const char* hex = "0123456789abcdef";
        std::string suffix(16, '0');
        for (int i = 15; i >= 0; --i, value >>= 4)
        {
            suffix[i] = hex[value & 0xF];
        }
        return prefix + suffix;
    }

    std::string host_id()
    {
        std::ifstream file("/proc/sys/kernel/random/boot_id");
        std::string id;
        if (not(file >> id) or id.empty())
        {
            throw RingError("cannot read /proc/sys/kernel/random/boot_id");
        }
        return id;
    }

    // ----------------------------------------------------------------------------------------- ControlWriter

    ControlWriter::ControlWriter(RingKind kind, std::uint32_t slot_count, std::uint64_t slot_size,
                                 std::size_t data_bytes)
    {
        if (slot_count < 2 or slot_count > kMaxSlots)
        {
            throw RingError("slot count " + std::to_string(slot_count) + ": 2 to " + std::to_string(kMaxSlots));
        }
        mapping_ = Mapping::create(random_name("/xblick-"), kControlSize + data_bytes);
        auto* h = header();
        std::memcpy(h->magic, kMagic, sizeof(kMagic));
        h->version = kVersion;
        h->slot_count = slot_count;
        h->slot_size = slot_size;
        h->data_offset = kind == RingKind::Host ? kControlSize : 0;
        h->kind = static_cast<std::uint32_t>(kind);
        for (std::uint32_t i = 0; i < slot_count; ++i)
        {
            new (slot(i)) SlotHeader{};
            slot(i)->seq.store(0, std::memory_order_relaxed);
            slot(i)->nbytes.store(0, std::memory_order_relaxed);
        }
        std::atomic_thread_fence(std::memory_order_release);
    }

    SlotHeader* ControlWriter::slot(std::uint32_t i) const
    {
        return reinterpret_cast<SlotHeader*>(mapping_.data() + kSlotHeaderOffset + i * kSlotHeaderSize);
    }

    std::uint32_t ControlWriter::begin()
    {
        if (open_ >= 0)
        {
            throw RingError("a slot is being written already (commit it first)");
        }
        const std::uint32_t i = next_;
        next_ = (next_ + 1) % header()->slot_count;
        SlotHeader* s = slot(i);
        const std::uint64_t seq = s->seq.load(std::memory_order_relaxed);
        s->seq.store(seq + 1, std::memory_order_relaxed);
        // the odd seq must be visible before any byte of the slot changes
        std::atomic_thread_fence(std::memory_order_release);
        open_ = i;
        return i;
    }

    std::uint64_t ControlWriter::commit(std::uint64_t nbytes)
    {
        if (open_ < 0)
        {
            throw RingError("commit without a slot begun");
        }
        if (nbytes > header()->slot_size)
        {
            throw RingError(std::to_string(nbytes) + " bytes do not fit a slot of " +
                            std::to_string(header()->slot_size));
        }
        SlotHeader* s = slot(static_cast<std::uint32_t>(open_));
        s->nbytes.store(nbytes, std::memory_order_relaxed);
        const std::uint64_t seq = s->seq.load(std::memory_order_relaxed) + 1;
        s->seq.store(seq, std::memory_order_release);  // the bytes before it are visible to an acquiring reader
        open_ = -1;
        return seq;
    }

    // ----------------------------------------------------------------------------------------- ControlReader

    ControlReader::ControlReader(const std::string& name, RingKind kind) : mapping_(Mapping::open_readonly(name))
    {
        const auto* h = header();
        if (std::memcmp(h->magic, kMagic, sizeof(kMagic)) != 0)
        {
            throw RingError(name + " is not a Durchblick ring");
        }
        if (h->version != kVersion)
        {
            throw RingError(name + ": ring version " + std::to_string(h->version) + ", this reader knows " +
                            std::to_string(kVersion));
        }
        if (h->kind != static_cast<std::uint32_t>(kind))
        {
            throw RingError(name + ": ring kind " + std::to_string(h->kind) + ", expected " +
                            std::to_string(static_cast<std::uint32_t>(kind)));
        }
        if (h->slot_count < 2 or h->slot_count > kMaxSlots)
        {
            throw RingError(name + ": slot count " + std::to_string(h->slot_count));
        }
        if (kind == RingKind::Host and mapping_.size() < kControlSize + h->slot_count * h->slot_size)
        {
            throw RingError(name + ": " + std::to_string(mapping_.size()) + " bytes, too small for its slots");
        }
    }

    const SlotHeader* ControlReader::slot(std::uint32_t i) const
    {
        if (i >= header()->slot_count)
        {
            throw RingError("slot " + std::to_string(i) + " of " + std::to_string(header()->slot_count));
        }
        return reinterpret_cast<const SlotHeader*>(mapping_.data() + kSlotHeaderOffset + i * kSlotHeaderSize);
    }

    bool ControlReader::acquire(std::uint32_t i, std::uint64_t seq) const
    {
        return slot(i)->seq.load(std::memory_order_acquire) == seq;
    }

    bool ControlReader::validate(std::uint32_t i, std::uint64_t seq) const
    {
        // the reads of the slot's bytes must complete before seq is read again
        std::atomic_thread_fence(std::memory_order_acquire);
        return slot(i)->seq.load(std::memory_order_relaxed) == seq;
    }

    std::uint64_t ControlReader::nbytes(std::uint32_t i) const
    {
        return slot(i)->nbytes.load(std::memory_order_relaxed);
    }

    // ------------------------------------------------------------------------------------------- host ring

    HostRingWriter::HostRingWriter(std::uint32_t slot_count, std::uint64_t slot_size)
        : control_(RingKind::Host, slot_count, round_up(slot_size, 4096),
                   static_cast<std::size_t>(slot_count) * round_up(slot_size, 4096))
    {
        if (slot_size == 0)
        {
            throw RingError("slot size 0");
        }
    }

    Written HostRingWriter::write(const void* src, std::uint64_t nbytes)
    {
        if (nbytes > control_.slot_size())
        {
            throw RingError(std::to_string(nbytes) + " bytes do not fit a slot of " +
                            std::to_string(control_.slot_size()));
        }
        std::uint8_t* dst = loan();
        std::memcpy(dst, src, nbytes);
        return commit(nbytes);
    }

    std::uint8_t* HostRingWriter::loan()
    {
        const std::uint32_t i = control_.begin();
        loaned_ = i;
        return control_.data() + i * control_.slot_size();
    }

    Written HostRingWriter::commit(std::uint64_t nbytes)
    {
        if (loaned_ < 0)
        {
            throw RingError("commit without a loaned slot");
        }
        const auto slot = static_cast<std::uint32_t>(loaned_);
        loaned_ = -1;
        return {slot, control_.commit(nbytes)};
    }

    HostRingReader::HostRingReader(const std::string& name) : control_(name, RingKind::Host)
    {
    }

    void HostRingReader::check_slot(std::uint32_t slot) const
    {
        if (slot >= control_.slot_count())
        {
            throw RingError("slot " + std::to_string(slot) + " of " + std::to_string(control_.slot_count()));
        }
    }

    bool HostRingReader::read(std::uint32_t slot, std::uint64_t seq, void* dst, std::uint64_t capacity) const
    {
        check_slot(slot);
        if (not control_.acquire(slot, seq))
        {
            return false;
        }
        const std::uint64_t n = control_.nbytes(slot);
        if (n > capacity)
        {
            if (not control_.validate(slot, seq))
            {
                return false;  // nbytes of a newer frame
            }
            throw RingError("frame of " + std::to_string(n) + " bytes, buffer of " + std::to_string(capacity));
        }
        std::memcpy(dst, view(slot), n);
        return control_.validate(slot, seq);
    }

    const std::uint8_t* HostRingReader::view(std::uint32_t slot) const
    {
        check_slot(slot);
        return control_.data() + slot * control_.slot_size();
    }
}  // namespace xblick
