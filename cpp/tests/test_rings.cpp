// Tests of the C++ core with writer and reader in separate processes, as they are used (CUDA's interprocess events
// cannot be opened in the process that created them). ctest runs "test_rings host" and "test_rings cuda"; the CUDA
// test needs a GPU with virtual memory management and is skipped (exit code 77) without one.
#include <sys/wait.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "xblick/cuda_ring.hpp"
#include "xblick/ring.hpp"

namespace
{
    int failures = 0;

    void expect(bool condition, const char* what)
    {
        if (not condition)
        {
            std::fprintf(stderr, "FAILED: %s\n", what);
            ++failures;
        }
    }

    std::vector<unsigned char> frame(std::size_t n, unsigned char value)
    {
        return std::vector<unsigned char>(n, value);
    }

    // One line through a pipe: "<name> <slot> <seq> ..." from the writer to the reader process.
    struct Pipe
    {
        int fd[2];
        Pipe() { expect(::pipe(fd) == 0, "pipe"); }
        void send(const std::string& line) { expect(::write(fd[1], line.c_str(), line.size() + 1) > 0, "write"); }
        std::string receive()
        {
            std::string line;
            char c;
            while (::read(fd[0], &c, 1) == 1 and c != '\0')
            {
                line += c;
            }
            return line;
        }
    };

    int wait_child(pid_t child)
    {
        int status = 0;
        ::waitpid(child, &status, 0);
        return WIFEXITED(status) ? WEXITSTATUS(status) : 99;
    }

    int test_host()
    {
        constexpr std::size_t n = 640 * 480 * 3;
        Pipe to_reader, to_writer;
        const pid_t child = ::fork();
        if (child == 0)
        {
            char name[128];
            unsigned slot, slot2;
            unsigned long long seq, seq2;
            std::sscanf(to_reader.receive().c_str(), "%127s %u %llu %u %llu", name, &slot, &seq, &slot2, &seq2);
            xblick::HostRingReader reader(name);
            std::vector<unsigned char> out(n);
            expect(reader.read(slot, seq, out.data(), out.size()), "read the first frame");
            expect(out == frame(n, 7), "content of the first frame");
            expect(reader.read(slot2, seq2, out.data(), out.size()), "read the second frame");
            expect(out == frame(n, 8), "content of the second frame");
            to_writer.send("read");
            // the writer has now overwritten every slot: the first frame must be refused
            to_reader.receive();
            expect(not reader.read(slot, seq, out.data(), out.size()), "an overwritten frame is refused");
            expect(not reader.acquire(slot, seq), "acquire refuses an overwritten frame");
            std::exit(failures);
        }
        xblick::HostRingWriter writer(3, n);
        const auto a = writer.write(frame(n, 7).data(), n);
        const auto b = writer.write(frame(n, 8).data(), n);
        expect(a.slot != b.slot and a.seq == 2 and b.seq == 2, "slots and sequence numbers");
        to_reader.send(writer.name() + " " + std::to_string(a.slot) + " " + std::to_string(a.seq) + " " +
                       std::to_string(b.slot) + " " + std::to_string(b.seq));
        to_writer.receive();
        for (int i = 0; i < 3; ++i)
        {
            writer.write(frame(n, 9).data(), n);
        }
        to_reader.send("overwritten");
        expect(wait_child(child) == 0, "reader process");
        return failures;
    }

    int test_cuda()
    {
        constexpr std::size_t n = 1920 * 1080 * 3;
        Pipe to_reader, to_writer;
        // fork before any CUDA call: a CUDA context does not survive fork
        const pid_t child = ::fork();
        if (child == 0)
        {
            char endpoint[128];
            unsigned slot;
            unsigned long long seq;
            std::sscanf(to_reader.receive().c_str(), "%127s %u %llu", endpoint, &slot, &seq);
            xblick::CudaRingReader reader(endpoint);
            auto buffer = reader.copy(slot, seq);
            expect(buffer != nullptr, "copy of the frame");
            if (buffer)
            {
                std::vector<unsigned char> out(n);
                buffer->download(out.data());
                expect(out == frame(n, 42), "content of the frame on the GPU");
            }
            if (xblick::cuda_device_count() >= 2)      // a reader on another GPU: the copy arrives there
            {
                xblick::CudaRingReader other(endpoint, 1);
                expect(other.ring_device() == 0 and other.device() == 1, "devices of a reader on another GPU");
                auto copied = other.copy(slot, seq);
                expect(copied != nullptr and copied->device() == 1, "copy onto another GPU");
                if (copied)
                {
                    std::vector<unsigned char> out(n);
                    copied->download(out.data());
                    expect(out == frame(n, 42), "content of the frame on the other GPU");
                }
                xblick::CudaRingReader same(endpoint, 0);
                expect(same.device() == 0 and not same.direct(), "the ring's own device as the target");
            }
            to_writer.send("read");
            to_reader.receive();
            expect(reader.copy(slot, seq) == nullptr, "an overwritten frame is refused");
            std::exit(failures);
        }
        if (xblick::cuda_device_count() == 0)
        {
            std::fprintf(stderr, "SKIPPED: no CUDA device\n");
            ::kill(child, SIGKILL);
            wait_child(child);
            return 77;  // ctest's SKIP_RETURN_CODE
        }
        // made before the CUDA ring, so that it is destroyed after it: the CUDA ring unpins it
        xblick::HostRingWriter host(2, n);
        xblick::CudaRingWriter writer(0, 2, n);
        const auto a = writer.write_host(frame(n, 42).data(), n);
        std::vector<unsigned char> back(n);
        writer.read_host(a.slot, back.data(), n);
        expect(back == frame(n, 42), "read_host of the writer");
        // straight into a page-locked host ring slot (CameraPublisher's download for shm subscribers)
        writer.pin_host(host.slots().data(), host.slots().size());
        bool refused = false;
        try
        {
            writer.pin_host(host.slots().data(), host.slots().size());
        }
        catch (const xblick::RingError&)
        {
            refused = true;
        }
        expect(refused, "a second pin_host is refused");
        std::uint8_t* slot = host.loan();
        writer.read_host(a.slot, slot, n);
        host.commit(n);
        expect(std::vector<unsigned char>(slot, slot + n) == frame(n, 42), "read_host into a pinned host slot");
        // loan and commit come in pairs
        expect(writer.loan() != 0, "loan gives a device pointer");
        bool twice = false;
        try
        {
            writer.loan();
        }
        catch (const xblick::RingError&)
        {
            twice = true;
        }
        expect(twice, "a second loan before commit is refused");
        const auto loaned = writer.commit(0, 0);
        bool unloaned = false;
        try
        {
            writer.commit(0, 0);
        }
        catch (const xblick::RingError&)
        {
            unloaned = true;
        }
        expect(unloaned, "a commit without loan is refused");
        expect(loaned.slot != a.slot, "the loaned slot is the next one");
        const auto again = writer.write_host(frame(n, 42).data(), n);  // the reader below gets this frame
        to_reader.send(writer.endpoint() + " " + std::to_string(again.slot) + " " + std::to_string(again.seq));
        to_writer.receive();
        writer.write_host(frame(n, 1).data(), n);
        writer.write_host(frame(n, 2).data(), n);
        to_reader.send("overwritten");
        expect(wait_child(child) == 0, "reader process");
        return failures;
    }
}  // namespace

int main(int argc, char** argv)
{
    const std::string which = argc > 1 ? argv[1] : "";
    try
    {
        if (which == "host")
        {
            return test_host();
        }
        if (which == "cuda")
        {
            return test_cuda();
        }
    }
    catch (const std::exception& e)
    {
        std::fprintf(stderr, "FAILED: %s\n", e.what());
        return 1;
    }
    std::fprintf(stderr, "usage: test_rings host|cuda\n");
    return 2;
}
