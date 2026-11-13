// detail::Latest while a subscription changes its transport, driven by hand (no Zenoh): which samples of the old and
// the new route are handed out, and which are left out as the same frame by the other route. The same cases as the
// _Latest tests in tests/test_camera.py.
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <thread>
#include <vector>

#include "stream.hpp"

namespace
{
    using xblick::detail::Item;
    using xblick::detail::Latest;

    int failures = 0;

    void expect(bool condition, const char* what)
    {
        if (not condition)
        {
            std::fprintf(stderr, "FAILED: %s\n", what);
            ++failures;
        }
    }

    constexpr std::int64_t kT0 = 1'000'000'000'000;  // 1000 s
    constexpr std::int64_t kPeriod = 33'333'333;     // 30 Hz

    std::int64_t frame(int n)
    {
        return kT0 + n * kPeriod;
    }

    // Offers a sample of `stamp` by `route` and returns the stamp handed out then (-1: none).
    std::int64_t offer(Latest& latest, int route, std::int64_t stamp)
    {
        auto item = std::make_shared<Item>();
        item->payload.assign(16, 0);  // CDR encapsulation, then the header's stamp
        const auto sec = static_cast<std::int32_t>(stamp / 1'000'000'000);
        const auto nanosec = static_cast<std::uint32_t>(stamp % 1'000'000'000);
        std::memcpy(item->payload.data() + 4, &sec, 4);
        std::memcpy(item->payload.data() + 8, &nanosec, 4);
        latest.offer(std::move(item), route);
        const auto taken = latest.take(true, std::nullopt);
        return taken ? xblick::detail::stamp_of(*taken) : -1;
    }

    // Offers `stamps` by `route` and appends those handed out to `out`.
    void offer_all(Latest& latest, int route, const std::vector<std::int64_t>& stamps, std::vector<std::int64_t>& out)
    {
        for (const auto s : stamps)
        {
            if (const auto got = offer(latest, route, s); got >= 0)
            {
                out.push_back(got);
            }
        }
    }

    std::vector<std::int64_t> frames(int first, int last)
    {
        std::vector<std::int64_t> out;
        for (int n = first; n <= last; ++n)
        {
            out.push_back(frame(n));
        }
        return out;
    }

    // The network hands out 1..10; the ring (route 2) lags by two frames: 8, 9, 10 are frames handed out already, and
    // the network's 11, still delivered while its subscriber is undeclared, comes after the ring's first sample.
    void lagging_new_route()
    {
        Latest latest;
        std::vector<std::int64_t> out;
        latest.move(1);
        offer_all(latest, 1, frames(1, 10), out);
        latest.move(2);
        offer_all(latest, 2, {frame(8)}, out);
        offer_all(latest, 1, {frame(11)}, out);  // the old route, left at the new route's first sample
        offer_all(latest, 2, {frame(9), frame(10), frame(11), frame(12)}, out);
        expect(out == frames(1, 12), "lagging new route: every frame once, in order");
        expect(latest.duplicates == 3, "lagging new route: 8, 9, 10 by the ring left out");
    }

    // A restart (a simulation's stamps start again) far behind the newest frame: kept at once, while both routes are
    // read and after the old one was left.
    void restart_far_back()
    {
        Latest latest;
        std::vector<std::int64_t> out;
        latest.move(1);
        offer_all(latest, 1, frames(1, 100), out);
        latest.move(2);
        offer_all(latest, 2, {frame(100)}, out);  // the ring's copy: the old route is left
        expect(offer(latest, 2, 1 * kPeriod) == 1 * kPeriod, "restart after the move: its first frame at once");
        expect(offer(latest, 2, 2 * kPeriod) == 2 * kPeriod, "restart after the move: its next frame");
    }

    // The source restarts while both routes are read, the network first: its restarted frame is left out (its frames
    // come again by the ring), the ring's lagging copies of 99 and 100 too, and the ring's restarted frames are kept.
    void restart_by_the_old_route()
    {
        Latest latest;
        std::vector<std::int64_t> out;
        latest.move(1);
        offer_all(latest, 1, frames(1, 100), out);
        latest.move(2);
        offer_all(latest, 1, {1 * kPeriod}, out);
        offer_all(latest, 2, {frame(99), frame(100), 1 * kPeriod, 2 * kPeriod}, out);
        auto expected = frames(1, 100);
        expected.insert(expected.end(), {1 * kPeriod, 2 * kPeriod});
        expect(out == expected, "restart by the old route: every frame once");
    }

    // The mirror case: the ring brings the restart as its first sample while the network still delivers frames of
    // before: the restart is kept and ends the handover, the network's later frames are left out with its route.
    void restart_by_the_new_route_first()
    {
        Latest latest;
        std::vector<std::int64_t> out;
        latest.move(1);
        offer_all(latest, 1, frames(1, 100), out);
        latest.move(2);
        offer_all(latest, 2, {1 * kPeriod}, out);
        offer_all(latest, 1, {frame(101), 1 * kPeriod}, out);
        offer_all(latest, 2, {2 * kPeriod}, out);
        auto expected = frames(1, 100);
        expected.insert(expected.end(), {1 * kPeriod, 2 * kPeriod});
        expect(out == expected, "restart by the new route first: every frame once");
    }

    // On the new route a stamp up to kHandover behind the newest one is the same frame, one ns further a restart.
    void restart_boundary()
    {
        constexpr std::int64_t window =
            std::chrono::duration_cast<std::chrono::nanoseconds>(xblick::detail::kHandover).count();
        for (const std::int64_t behind : {-1, 0, 1})
        {
            Latest latest;
            latest.move(1);
            offer(latest, 1, frame(100));
            latest.move(2);
            const std::int64_t stamp = frame(100) - window - behind;
            const bool kept = offer(latest, 2, stamp) == stamp;
            expect(kept == (behind == 1), "restart boundary: kept only more than kHandover behind");
        }
    }

    // Frames that carry one stamp on purpose, or a stamp less than kHandover behind: left out during the window.
    void equal_stamps_in_window()
    {
        Latest latest;
        latest.move(1);
        expect(offer(latest, 1, frame(5)) == frame(5), "equal stamps: the first frame");
        latest.move(2);
        expect(offer(latest, 2, frame(5)) < 0, "equal stamps: the ring's copy left out");
        expect(offer(latest, 2, frame(5)) < 0, "equal stamps: left out during the window");
        expect(offer(latest, 2, frame(5) - 1'000'000'000) < 0, "1 s behind: left out during the window");
    }

    // The window ends after kHandover: a ring that stops longer after a first sample behind the newest one hands out
    // older frames again (a remaining case, named in Latest's comment).
    void window_ends_by_time()
    {
        Latest latest;
        std::vector<std::int64_t> out;
        latest.move(1);
        offer_all(latest, 1, frames(1, 10), out);
        latest.move(2);
        expect(offer(latest, 2, frame(5)) < 0, "window: the ring's lagging first sample left out");
        std::this_thread::sleep_for(xblick::detail::kHandover + std::chrono::milliseconds(100));
        expect(offer(latest, 2, frame(6)) == frame(6), "window: after kHandover an older frame is handed out");
    }

    // The new route never delivers: leave_old (the end of the handover, or close) leaves the old route all the same.
    void new_route_never_delivers()
    {
        Latest latest;
        std::vector<std::int64_t> out;
        latest.move(1);
        offer_all(latest, 1, frames(1, 5), out);
        latest.move(2);
        expect(not latest.wait_moved(std::chrono::milliseconds(10)), "silent new route: not moved");
        latest.leave_old();
        expect(latest.wait_moved(std::chrono::milliseconds(0)), "leave_old: moved");
        offer_all(latest, 1, {frame(6)}, out);  // the old route, left
        offer_all(latest, 2, {frame(5), frame(6), frame(7)}, out);
        expect(out == frames(1, 7), "silent new route: old route left, the new one's copy of 5 left out");
    }

    // Two moves in a row (the second before the route of the first delivered): the route before is read until the
    // newest one delivers, an older one not at all.
    void two_moves_in_a_row()
    {
        Latest latest;
        std::vector<std::int64_t> out;
        latest.move(1);
        offer_all(latest, 1, frames(1, 5), out);
        latest.move(2);
        latest.leave_old();
        latest.move(3);
        offer_all(latest, 1, {frame(6)}, out);  // left with the first move
        offer_all(latest, 2, {frame(6)}, out);  // read until route 3 delivers
        offer_all(latest, 3, {frame(6)}, out);  // the same frame: left out, route 2 left
        offer_all(latest, 2, {frame(7)}, out);
        offer_all(latest, 3, {frame(7)}, out);
        expect(out == frames(1, 7), "two moves: every frame once");
    }
}

int main()
{
    lagging_new_route();
    restart_far_back();
    restart_by_the_old_route();
    restart_by_the_new_route_first();
    restart_boundary();
    equal_stamps_in_window();
    window_ends_by_time();
    new_route_never_delivers();
    two_moves_in_a_row();
    if (failures)
    {
        std::fprintf(stderr, "%d failed\n", failures);
        return 1;
    }
    std::puts("latest: ok");
    return 0;
}
