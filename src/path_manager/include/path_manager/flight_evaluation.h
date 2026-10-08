#ifndef PATH_MANAGER_FLIGHT_EVALUATION_H
#define PATH_MANAGER_FLIGHT_EVALUATION_H

#include <string>

namespace path_manager {

// [S13] Which JUDGMENT applies to a span of the flight. The evaluator
// selects the model by this CONTRACT TYPE — never by parsing the display
// name (review point: labels are for output; a typo in a string must not
// silently change physics).
//   CRUISE     full cruise-model envelope judgment (today's audit)
//   TERMINAL   prescribed helix — judged under the cruise model today
//              (distinct kind so a dedicated judgment can attach later)
//   TRANSITION outside the cruise model BY DEFINITION: excluded from the
//              cruise envelope statistics and MEASURED only — its own
//              gate arrives with the transition evaluator (ADR-0002
//              freezes the v0 limits as experimental, not acceptance).
//              Terrain/zone/risk checks still apply: those are
//              model-agnostic.
enum class PhaseKind { CRUISE, TERMINAL, TRANSITION };
struct PhaseSpan {
  double t_end{0.0};
  PhaseKind kind{PhaseKind::CRUISE};
  std::string name;  // display only
};

struct FlightVerdict {
  bool evaluated{false};  // false: degenerate input, nothing was judged
  bool clean{true};
  bool underground{false};
  bool no_cruise{false};
  double viol_pct{0.0};
  double util_peak{0.0};
  // Zone policy of the DELIVERED flight, sampled at 0.1 s. hard counts
  // contact with a HARD_AVOID volume only; soft counts crossings the
  // 3-pass policy chose (SOFT_UNAVOIDABLE / _ENDPOINT / _FALLBACK) and is
  // reported, not judged. policy_measurable is false when zones exist but
  // the snapshot was stale or invalid — "could not check" must never read
  // as "clear".
  // zone_hard_n counts samples inside the volume the MISSION AUTHORED.
  // zone_standoff_n counts samples in the 1.0-1.05 routing standoff shell
  // outside it. Refusing the second was the original defect: it fails a
  // flight half a kilometre clear of anything the mission declared, on the
  // exact surface where the optimizer's only restoring force is zero by
  // construction, so identical missions flipped between CLEAN and FAILED.
  // The shell is reported and degrades; the authored volume refuses.
  // WHY the flight is not clean, separately. `clean` collapses three
  // different causes into one bit, and the shared degrade helper then
  // reported all of them as STITCHED_ENVELOPE_BUDGET — so a flight whose
  // ZONE POLICY could not be evaluated came back with a machine-readable
  // reason about the airframe's limits, and the enum priority buried the
  // reason that was true. The detail string carried both; nothing a
  // program reads did.
  bool envelope_bad{false};
  int zone_hard_n{0};
  int zone_standoff_n{0};
  int zone_soft_n{0};
  // [LEG-POLICY] Leg handovers examined, and how many of them sat inside an
  // authored volume under the stricter of the two adjoining policies. A
  // handover is an instant, so it is counted rather than timed — but it
  // refuses the flight exactly as a sampled contact does.
  int zone_junction_n{0};
  int zone_junction_hard_n{0};
  bool policy_measurable{true};
  // The conditions no trajectory may ever fly with, whatever produced it.
  // Envelope over-utilization is graded separately: it is a margin the
  // caller may knowingly spend. Terrain is not, and neither is a hard
  // zone — the whole point of HARD_AVOID is that it is not traded against
  // distance, so it cannot be a DEGRADED reason the caller absorbs.
  // An unjudgeable zone policy joins them for the same reason a rejected
  // terrain map does: the flight was not checked, so it is not cleared.
  bool unflyable() const {
    return evaluated && (underground || no_cruise || zone_hard_n > 0 ||
                         zone_junction_hard_n > 0 || !policy_measurable);
  }
  // The unflyable conditions this verdict actually MEASURED, without the
  // "could not check" one. For callers whose mission shape puts zone
  // policy outside what the snapshot can express at all — a multi-leg
  // front end — !policy_measurable is a statement about scope rather than
  // about the flight, and treating it as a hazard refused every such
  // mission that had a zone anywhere. Those callers refuse on this and
  // degrade on the missing scope, so "we did not check" still never reads
  // as "it is clear".
  bool unflyableMeasured() const {
    return evaluated && (underground || no_cruise || zone_hard_n > 0 ||
                         zone_junction_hard_n > 0);
  }
};

}  // namespace path_manager

#endif  // PATH_MANAGER_FLIGHT_EVALUATION_H
