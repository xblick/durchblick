// A partner subscriber (SubscriberOptions::keep_recent) against a plain one, in one Zenoh session that reaches
// nobody else: frames the plain one never took count as overwritten, the partner's stay available to at() and do not.
// In the same session, replies(): a query answered, one nobody answers (ended by Zenoh by its timeout). Before that,
// partner_of (pairing.hpp) on the cases of Python's tests/test_camera.py::test_the_partner_rule.
#include <chrono>
#include <cstdio>
#include <optional>
#include <stdexcept>
#include <thread>
#include <vector>

#include "durchblick/camera.hpp"

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

    // The cases of tests/test_camera.py::test_the_partner_rule: from a steady partner, a sample at most a quarter of its
    // smallest step before the primary stamp is taken at once, with the measured and with an explicit offset.
    void pairing_rule()
    {
        using durchblick::partner_of;
        constexpr std::int64_t ms = 1'000'000;
        const std::vector<std::int64_t> steady{0, 40 * ms, 80 * ms, 120 * ms, 160 * ms};  // 25 Hz, newest 160 ms
        const std::vector<std::int64_t> one{160 * ms};
        const std::vector<std::int64_t> irregular{0, 10 * ms, 20 * ms, 160 * ms};  // median step 10 ms
        const std::optional<std::int64_t> measured;
        expect(partner_of(steady, 170 * ms, measured) == 160 * ms, "a quarter period before: taken at once");
        expect(not partner_of(steady, 170 * ms + 1, measured), "more than a quarter period before: waits");
        expect(partner_of(steady, 162 * ms, 5 * ms) == 160 * ms, "explicit offset: taken at once as well");
        expect(not partner_of(steady, 171 * ms, 15 * ms), "explicit, more than a quarter period before: waits");
        expect(not partner_of(steady, 166 * ms, 5 * ms), "beyond the offset");
        expect(partner_of(steady, 158 * ms, 5 * ms) == 160 * ms, "the nearest after the stamp: taken");
        expect(partner_of(steady, 100 * ms, 20 * ms) == 80 * ms, "equally near: the first kept");
        expect(not partner_of(one, 162 * ms, 5 * ms), "one sample, period unmeasured: waits");
        expect(not partner_of(one, 160 * ms, measured), "one sample, measured offset: no offset yet");
        expect(not partner_of(irregular, 162 * ms, 5 * ms), "not steady: the next one may come at any time");
        constexpr std::int64_t camera = 33'333'333;  // a 30 Hz camera, of which the partner has frames 0, 4, 7, 11, 15
        const std::vector<std::int64_t> skipping{0, 4 * camera, 7 * camera, 11 * camera, 15 * camera};
        expect(not partner_of(skipping, 16 * camera, measured), "skips camera frames: frame 16's own may still come");
        const std::vector<std::int64_t> jitter{0, 66'900'000, 133'100'000, 200'200'000, 266'400'000};  // 15 Hz
        expect(partner_of(jitter, 266'640'000, 5 * ms) == 266'400'000, "steady within a tenth: taken at once");
        expect(durchblick::steady_step_ns(jitter) == 66'200'000 and not durchblick::steady_step_ns(skipping),
               "steady step: the smallest of a steady partner");
        expect(not partner_of(steady, 160 * ms + 1, 0), "offset 0: equal stamps only");
        expect(partner_of(steady, 160 * ms, 0) == 160 * ms and partner_of(steady, 80 * ms, 0) == 80 * ms,
               "offset 0: an equal stamp, at once");
        expect(not durchblick::partner_period_ns({}) and not durchblick::partner_period_ns({5}) and
                   not durchblick::partner_period_ns({5, 5}),
               "period: none before two different stamps");
        expect(durchblick::partner_period_ns(irregular) == 10 * ms and durchblick::partner_period_ns({30, 0, 10}) == 20,
               "period: the median step (of two middle ones the larger)");
        bool refused = false;
        try
        {
            partner_of(steady, 0, -1);
        }
        catch (const std::invalid_argument&)
        {
            refused = true;
        }
        expect(refused, "a negative offset is refused");
    }
}

int main()
{
    using namespace durchblick;
    using namespace std::chrono_literals;
    pairing_rule();
    zenoh::Session session = open_session({.local_router = false});
    CameraPublisher publisher(session, "t/partner/depth", {.encoding = "mono8", .frame_id = "cam"});
    ImageSubscriber plain(session, "t/partner/depth", {.transport = Transport::Raw});
    ImageSubscriber partner(session, "t/partner/depth", {.transport = Transport::Raw, .keep_recent = 4});
    expect(wait_until([&] { return publisher.subscribed(); }), "the subscribers found");
    std::vector<std::uint8_t> pixels(8 * 8, 7);
    const std::int64_t t0 = 1'000'000'000'000;
    for (int i = 0; i < 5; ++i)
    {
        publisher.publish(HostImage{pixels.data(), 8, 8, 0}, t0 + i * 33'000'000);
    }
    expect(wait_until([&] { return plain.received() == 5 and partner.received() == 5; }), "5 frames each");
    expect(plain.overwritten() == 4, "the plain subscriber: 4 replaced before it took them");
    expect(partner.overwritten() == 0, "the partner: none lost, its frames kept for at()");
    expect(partner.recent_stamps().size() == 4, "the partner keeps 4");
    const auto paired = partner.at(t0 + 2 * 33'000'000, 1ms);
    expect(paired and paired->header.stamp.to_ns() == t0 + 2 * 33'000'000, "a kept frame paired by its stamp");
    {
        auto queryable = session.declare_queryable(
            zenoh::KeyExpr("t/q"), [](const zenoh::Query& q) { q.reply(zenoh::KeyExpr("t/q"), "yes"); },
            zenoh::closures::none);
        const auto answers = replies(session, "t/q", 1000ms);
        expect(answers.size() == 1 and answers[0].is_ok() and answers[0].get_ok().get_payload().as_string() == "yes",
               "replies: the answer");
        const auto started = std::chrono::steady_clock::now();
        expect(replies(session, "t/nobody", 200ms).empty(), "replies: none from nobody");
        expect(std::chrono::steady_clock::now() - started < 600ms, "replies: ended by Zenoh, before the deadline");
        expect(queries_not_finalized() == 0, "replies: every query finalized");
        std::move(queryable).undeclare();
    }
    if (cuda_device_count() > 0)  // local Auto: the CUDA ring on this subscriber's GPU (only the choice here)
    {
        CameraPublisher gpu(session, "t/partner/gpu", {.encoding = "mono8", .frame_id = "cam", .cuda_device = 0});
        ImageSubscriber cpu(session, "t/partner/gpu", {});
        ImageSubscriber automatic(session, "t/partner/gpu", {.cuda_device = 0});
        ImageSubscriber shm(session, "t/partner/gpu", {.local = Transport::Shm});
        expect(wait_until([&] { return cpu.transport() and automatic.transport() and shm.transport(); }),
               "auto chose");
        expect(cpu.transport_reason() == "shm (this subscriber works on the CPU)", "auto without a device: shm");
        expect(automatic.transport_reason() == "cuda (publisher on this GPU)", "auto: the CUDA ring on this GPU");
        expect(shm.transport_reason() == "shm (asked for)", "shm as asked");
    }
    if (failures == 0)
    {
        std::printf("partner: all passed\n");
    }
    return failures == 0 ? 0 : 1;
}
