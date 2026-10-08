#include "path_manager/route_search_backend.h"

#include <algorithm>
#include <atomic>

namespace path_manager {

uint64_t nextRouteSearchIdentity() {
  static std::atomic<uint64_t> sequence{0};
  return sequence.fetch_add(1, std::memory_order_relaxed) + 1;
}

bool routeSearchGeometryUsable(const RouteSearchResult &result) {
  return result.status == RouteSearchStatus::SUCCESS && result.points.size() >= 2 &&
      std::all_of(result.points.begin(), result.points.end(),
                  [](const Eigen::Vector3d &point) { return point.allFinite(); });
}

bool routeSearchPolicyMeasured(const RouteSearchProblem &problem,
                               const RouteSearchPolicy &policy) {
  if (policy.knowledge != RoutePolicyKnowledge::CANONICAL_DISPOSITIONS) return false;
  const auto &a = problem.identity;
  const auto &b = policy.identity;
  if (a.request_id == 0 || a.request_id != b.request_id ||
      a.context_id != b.context_id || a.environment_revision != b.environment_revision ||
      a.zone_generation != b.zone_generation || a.policy_epoch != b.policy_epoch ||
      a.search_serial != b.search_serial || a.leg != b.leg) return false;
  const size_t count = problem.environment.zones.size();
  if (policy.dispositions.size() != count || policy.legacy_pass < 0 ||
      policy.legacy_pass > 3) return false;
  for (const auto disposition : policy.dispositions) {
    switch (disposition) {
      case RouteZoneDisposition::HARD_AVOID:
      case RouteZoneDisposition::SOFT_UNAVOIDABLE:
      case RouteZoneDisposition::SOFT_ENDPOINT:
      case RouteZoneDisposition::SOFT_FALLBACK: break;
      default: return false;
    }
  }
  const auto flags_valid = [count](const std::vector<char> &flags) {
    return (flags.empty() || flags.size() == count) &&
        std::all_of(flags.begin(), flags.end(), [](char flag) {
          return flag == 0 || flag == 1;
        });
  };
  if (!flags_valid(policy.endpoint_exempt) || !flags_valid(policy.soft_override))
    return false;
  if (policy.legacy_pass == 0) {
    // External canonical policy has no raw FM2 pass to justify subordinate
    // flags. Optional diagnostics must therefore agree directly with the
    // authoritative disposition, rather than certifying contradictory data.
    for (size_t i = 0; i < count; ++i) {
      if (!policy.endpoint_exempt.empty() &&
          bool(policy.endpoint_exempt[i]) !=
              (policy.dispositions[i] == RouteZoneDisposition::SOFT_ENDPOINT))
        return false;
      if (!policy.soft_override.empty() &&
          bool(policy.soft_override[i]) !=
              (policy.dispositions[i] == RouteZoneDisposition::SOFT_UNAVOIDABLE))
        return false;
    }
  } else {
    if (policy.endpoint_exempt.size() != count ||
        policy.soft_override.size() != count ||
        policy.suppress_crossing_exempt != (policy.legacy_pass == 1)) return false;
    for (size_t i = 0; i < count; ++i) {
      const auto expected = policy.endpoint_exempt[i]
          ? RouteZoneDisposition::SOFT_ENDPOINT
          : policy.legacy_pass == 2 ? RouteZoneDisposition::SOFT_FALLBACK
          : policy.soft_override[i] ? RouteZoneDisposition::SOFT_UNAVOIDABLE
                                    : RouteZoneDisposition::HARD_AVOID;
      if (policy.dispositions[i] != expected) return false;
    }
  }
  return true;
}

}  // namespace path_manager
