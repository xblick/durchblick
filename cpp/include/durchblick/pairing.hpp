// Pairing by stamp: which sample of a partner stream (the depth of a colour frame) belongs to a frame of the primary
// stream. The partner's samples are kept as they came (SubscriberOptions::keep_recent, recent_stamps()); a primary
// frame takes the nearest of them within a maximum offset, once no nearer one can still come. Python's
// history.PartnerHistory calls these functions through blick._core, so both languages pair alike. In
// durchblick::core, without Zenoh and the messages.
//
// Stamps only grow, so a later sample is nearer than the nearest kept one (`best`, before the primary stamp) only if
// it is stamped before stamp + (stamp - best). A steady partner (its kept steps within a tenth of their median) sends
// its next sample about one step after the newest kept: it can be nearer only by coming more than half a step early.
// A nearest sample at most a quarter of the smallest step before the primary stamp (a camera of a pair that stamps a
// little earlier) is therefore taken at once. A partner that is not steady, or has fewer than two samples, may send
// its next sample at any time (a node that skips camera frames): the rule waits for a sample at or after that bound.
#pragma once

#include <cstdint>
#include <optional>
#include <vector>

namespace durchblick
{
    // The partner's frame period: the median step of the kept stamps, each stamp counted once (of an even number of
    // steps the larger middle one). Empty before two different stamps.
    std::optional<std::int64_t> partner_period_ns(const std::vector<std::int64_t>& kept);

    // The smallest step of the kept stamps of a steady partner: one whose steps (each stamp counted once) lie within a
    // tenth of their median. Empty for a partner that is not steady, and before two different stamps.
    std::optional<std::int64_t> steady_step_ns(const std::vector<std::int64_t>& kept);

    // The stamp of the partner sample of the primary frame stamped stamp_ns, of the kept stamps (in arrival order;
    // of two equally near the first): the nearest, at most max_offset_ns away, once no nearer one can still come.
    // Empty if there is none (yet). max_offset_ns empty: half the measured period (empty while it is unmeasured);
    // 0 asks for equal stamps. Throws std::invalid_argument for a negative max_offset_ns.
    std::optional<std::int64_t> partner_of(const std::vector<std::int64_t>& kept, std::int64_t stamp_ns,
                                           std::optional<std::int64_t> max_offset_ns);
}
