#pragma once

#include "path_manager/risk_zone.h"
#include "path_manager/route_search_backend.h"
#include <cstddef>
#include <cstdint>
#include <vector>

namespace path_manager {

// Owned policy provenance shared by planning, evaluation and execution.
// Shape identity and generation/epoch must be validated by the environment
// contact provider. A nonempty list is not evidence of measured policy.
using ZoneDisposition = RouteZoneDisposition;
struct ZonePolicySnapshot {
    // DATA generation (zone list / terrain re-derivation) AND policy
    // epoch (per front-end search) — dispositions are PER-SEARCH state
    // (endpoint containment, unavoidable-soft set, final pass), so a
    // re-plan with the SAME zone list still invalidates an older
    // snapshot (review find: generation alone missed that).
    uint64_t generation{0};
    uint64_t epoch{0};
    // False when the backend did not return measured canonical policy,
    // no search has run since the last zone/terrain change, or the
    // epoch's front-end ran MORE than one segment
    // search (multi-waypoint mission: each leg rewrites the whole
    // policy state, so the buffers describe only the LAST leg —
    // publishing that as plan-wide policy is fail-open; review find).
    // Valid means exactly: ONE single-goal search ran on current
    // data. Contract 2 requires that; an invalid snapshot is
    // fail-closed at every consumer.
    bool valid{true};
    struct Entry {
        RiskZone zone;  // SHAPE COPY — identity, not an index
        ZoneDisposition disposition{ZoneDisposition::HARD_AVOID};
    };
    std::vector<Entry> zones;
};

// Captured immediately after each leg search, in contiguous search order.
struct LegPolicySnapshot {
    size_t leg{0};
    // Monotone per search within an epoch. A missing or repeated serial
    // means the capture sequence has a hole or a double-write, which is
    // a programming error and is refused rather than averaged over.
    uint64_t search_serial{0};
    ZonePolicySnapshot policy;
};

// Unknown policy is INVALID; a known policy with mismatched provenance is
// STALE. Neither result is equivalent to CLEAR.
enum class ZoneContactResult { CLEAR, CONTACT, STALE, INVALID };

}  // namespace path_manager
