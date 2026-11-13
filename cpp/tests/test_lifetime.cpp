// A publisher (camera or point cloud) destroyed while another thread queries its <base>/info without pause: Zenoh's thread answers the query
// from the publisher's members, so the queryable must go before them. A CUDA ring's server takes up to 200 ms to stop,
// which left a query enough time to read freed members (durchblick-relay 0.2.1 stopped with std::length_error). The
// shared-memory case runs everywhere, the CUDA case only with a GPU. Built with -fsanitize=address, the use after free
// shows as such; without it, a freed string can still throw or crash.
#include <atomic>
#include <chrono>
#include <cstdio>
#include <memory>
#include <thread>
#include <vector>

#include "durchblick/camera.hpp"
#include "durchblick/points.hpp"

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

    template <typename Condition>
    bool wait_until(Condition condition, std::chrono::seconds timeout = std::chrono::seconds(20))
    {
        const auto end = std::chrono::steady_clock::now() + timeout;
        while (not condition())
        {
            if (std::chrono::steady_clock::now() > end)
            {
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return true;
    }

    // Creates and destroys a publisher of `base` `rounds` times, each after one frame into its ring, while a thread
    // queries <base>/info all the time. Returns the number of answers (some must come, or nothing was tested).
    template <typename MakePublisher, typename MakeSubscriber, typename Publish>
    std::size_t destroy_while_queried(zenoh::Session& session, const std::string& base, MakePublisher make_publisher,
                                      MakeSubscriber make_subscriber, Publish publish, int rounds)
    {
        using namespace std::chrono_literals;
        std::atomic<bool> stop{false};
        std::atomic<std::size_t> answers{0};
        std::thread asker([&] {
            while (not stop.load())
            {
                for (const auto& reply : durchblick::replies(session, base + "/info", 100ms))
                {
                    if (reply.is_ok())
                    {
                        ++answers;
                    }
                }
            }
        });
        for (int i = 0; i < rounds; ++i)
        {
            auto publisher = make_publisher();
            auto subscriber = make_subscriber();
            expect(wait_until([&] { return publisher->subscribed() and subscriber->transport(); }), "subscribed");
            expect(not publish(*publisher).empty(), "a frame into the ring");
            const std::size_t before = answers.load();
            wait_until([&] { return answers.load() > before; }, std::chrono::seconds(5));
            publisher.reset();  // with the asker's queries arriving
        }
        stop.store(true);
        asker.join();
        return answers.load();
    }
}

int main()
{
    using namespace durchblick;
    zenoh::Session session = open_session({.local_router = false});
    std::vector<std::uint8_t> pixels(64 * 48, 7);
    const auto camera = [&](const std::string& base, PublisherOptions options) {
        return [&session, base, options] { return std::make_unique<CameraPublisher>(session, base, options); };
    };
    const auto images = [&](const std::string& base, SubscriberOptions options) {
        return [&session, base, options] { return std::make_unique<ImageSubscriber>(session, base, options); };
    };
    const std::size_t shm = destroy_while_queried(
        session, "t/lifetime/shm", camera("t/lifetime/shm", {.encoding = "mono8", .frame_id = "cam"}),
        images("t/lifetime/shm", {.transport = Transport::Shm}),
        [&](CameraPublisher& p) { return p.publish(HostImage{pixels.data(), 48, 64, 0}); }, 10);
    expect(shm > 0, "shm: the info queries were answered");
    std::vector<float> points(64 * 48 * 4, 1.0f);  // xyz_layout: x, y, z and padding, 16 bytes per point
    const std::size_t clouds = destroy_while_queried(
        session, "t/lifetime/points",
        [&] {
            return std::make_unique<PointCloudPublisher>(session, "t/lifetime/points",
                                                         CloudPublisherOptions{.layout = xyz_layout(), .frame_id = "cam"});
        },
        [&] {
            return std::make_unique<PointCloudSubscriber>(session, "t/lifetime/points",
                                                          CloudSubscriberOptions{.transport = Transport::Shm});
        },
        [&](PointCloudPublisher& p) { return p.publish(HostCloud{points.data(), 48, 64, true}); }, 10);
    expect(clouds > 0, "points: the info queries were answered");
    if (cuda_device_count() > 0)
    {
        auto buffer = make_device_buffer(0, pixels.size());
        buffer->upload(pixels.data());
        const std::size_t cuda = destroy_while_queried(
            session, "t/lifetime/cuda",
            camera("t/lifetime/cuda", {.encoding = "mono8", .frame_id = "cam", .cuda_device = 0}),
            images("t/lifetime/cuda", {.cuda_device = 0}),
            [&](CameraPublisher& p) { return p.publish(DeviceImage{buffer->ptr(), 48, 64, 0}); }, 10);
        expect(cuda > 0, "cuda: the info queries were answered");
        auto cloud = make_device_buffer(0, points.size() * sizeof(float));
        cloud->upload(points.data());
        const std::size_t cuda_clouds = destroy_while_queried(
            session, "t/lifetime/cuda_points",
            [&] {
                return std::make_unique<PointCloudPublisher>(
                    session, "t/lifetime/cuda_points",
                    CloudPublisherOptions{.layout = xyz_layout(), .frame_id = "cam", .cuda_device = 0});
            },
            [&] {
                return std::make_unique<PointCloudSubscriber>(session, "t/lifetime/cuda_points",
                                                              CloudSubscriberOptions{.cuda_device = 0});
            },
            [&](PointCloudPublisher& p) { return p.publish(DeviceCloud{cloud->ptr(), 48, 64, 0, true}); }, 10);
        expect(cuda_clouds > 0, "cuda points: the info queries were answered");
    }
    else
    {
        std::printf("lifetime: no CUDA device, the CUDA ring case skipped\n");
    }
    if (failures == 0)
    {
        std::printf("lifetime: all passed\n");
    }
    return failures == 0 ? 0 : 1;
}
