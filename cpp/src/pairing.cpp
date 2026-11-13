#include "xblick/pairing.hpp"

#include <algorithm>
#include <cstdlib>
#include <stdexcept>
#include <string>

namespace xblick
{
    namespace
    {
        // The steps between the kept stamps, each stamp counted once, sorted.
        std::vector<std::int64_t> steps_of(const std::vector<std::int64_t>& kept)
        {
            std::vector<std::int64_t> stamps = kept;
            std::sort(stamps.begin(), stamps.end());
            stamps.erase(std::unique(stamps.begin(), stamps.end()), stamps.end());
            std::vector<std::int64_t> steps;
            for (std::size_t i = 1; i < stamps.size(); ++i)
            {
                steps.push_back(stamps[i] - stamps[i - 1]);
            }
            std::sort(steps.begin(), steps.end());
            return steps;
        }
    }

    std::optional<std::int64_t> partner_period_ns(const std::vector<std::int64_t>& kept)
    {
        const std::vector<std::int64_t> steps = steps_of(kept);
        if (steps.empty())
        {
            return std::nullopt;
        }
        return steps[steps.size() / 2];
    }

    std::optional<std::int64_t> steady_step_ns(const std::vector<std::int64_t>& kept)
    {
        const std::vector<std::int64_t> steps = steps_of(kept);
        if (steps.empty())
        {
            return std::nullopt;
        }
        // a partner that skips camera frames (a node that takes the newest frame when it is free) has steps of k and
        // k + 1 camera periods: they differ by a tenth of the median only from k = 10 on. Clock jitter of a steady
        // camera stays far below (a millisecond of 33 or 67).
        if (steps.back() - steps.front() > steps[steps.size() / 2] / 10)
        {
            return std::nullopt;
        }
        return steps.front();
    }

    std::optional<std::int64_t> partner_of(const std::vector<std::int64_t>& kept, std::int64_t stamp_ns,
                                           std::optional<std::int64_t> max_offset_ns)
    {
        if (max_offset_ns and *max_offset_ns < 0)
        {
            throw std::invalid_argument("max_offset_ns " + std::to_string(*max_offset_ns) +
                                        ": not negative (none: half the partner's frame period)");
        }
        if (kept.empty())
        {
            return std::nullopt;
        }
        const std::optional<std::int64_t> period = partner_period_ns(kept);
        std::optional<std::int64_t> offset = max_offset_ns;
        if (not offset and period)
        {
            offset = *period / 2;
        }
        std::int64_t best = kept.front();
        for (const std::int64_t s : kept)
        {
            if (std::abs(s - stamp_ns) < std::abs(best - stamp_ns))
            {
                best = s;
            }
        }
        if (not offset or std::abs(best - stamp_ns) > *offset)
        {
            return std::nullopt;
        }
        const std::int64_t newest = *std::max_element(kept.begin(), kept.end());
        const std::int64_t nearer_until = stamp_ns + (stamp_ns - best);  // a later sample stamped before this is nearer
        // a steady partner sends its next sample about a step after the newest, at least half its smallest step later
        const std::optional<std::int64_t> step = steady_step_ns(kept);
        if (best < stamp_ns and newest < nearer_until and (not step or newest + *step / 2 < nearer_until))
        {
            return std::nullopt;  // one stamped before nearer_until may still come, nearer
        }
        return best;
    }
}
