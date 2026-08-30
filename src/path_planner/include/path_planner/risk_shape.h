#ifndef _RISK_SHAPE_H_
#define _RISK_SHAPE_H_

#include <cmath>

// The single definition of the risk attenuation shape.
//
// It lives in path_planner because that is the lowest package: the front-end
// searcher, the optimizer, and the manager all depend on it, so one definition
// can reach every layer. Before this header the same expression existed as
// four independent transcriptions plus a hand-written derivative, and a
// mutation of ONE of them passed the entire regression -- 86/0/0, 71/71,
// 154/0 -- while materially changing the route (ledger PA-1).
//
// WHAT IS CONTRACTUAL (independent of the exponent):
//   1. maximum at the centre, zero at the boundary:
//      shape(0) == 1, and shape(q) == 0 for every q >= 1
//   2. monotonically non-increasing in q
//   3. finite and non-negative everywhere
//   4. every layer prices risk through THIS function
//   5. shapeDeriv() is exactly d(shape)/dq -- value and analytic derivative
//      always agree
//   6. changing the shape obliges you to re-derive
//      optimization/weight_Risk_barrier (25000.0; its written derivation
//      assumes the rim contact slope) and to re-validate the 154-case
//      scenario set
//
// WHAT IS NOT CONTRACTUAL: the exponent. The current implementation is
// quadratic. That is an implementation choice, audited 2026-08-19 and found to
// have no requirement behind it: no tracked document specifies it, the flight
// refusal verdict never reads it, and the optimizer used a CUBIC decay until
// 47ce135 unified it to the quadratic for field sharing -- not for the value 2.
// Do not quote the exponent to callers as a guarantee.
namespace mmp {
namespace risk {

// Per-zone moat ceiling. One definition; it used to be three identical
// constexpr locals whose agreement nothing checked.
constexpr double kMoatCap = 1.0 - 1e-3;

// Attenuation as a function of the normalised ellipsoidal radius q >= 0.
// Returns 0 outside the envelope, which is also what a NaN q yields: the
// comparison is written so that any unordered result takes the zero branch.
inline double shape(double q) {
  if (!(q < 1.0)) return 0.0;
  const double u = 1.0 - q;
  return u * u;
}

// d(shape)/dq. MUST stay the analytic derivative of shape() -- property 5.
// Non-positive by property 2, and 0 outside the envelope.
inline double shapeDeriv(double q) {
  if (!(q < 1.0)) return 0.0;
  const double u = 1.0 - q;
  return -2.0 * u;
}

// Admissible per-zone peak. Both the struct declaration and the operator
// config state peak MUST be in (0, 1]; nothing enforced it, so the yaml
// loader accepted anything and the runtime loader silently DROPPED offending
// zones -- which removes a hazard from the field and is fail-open.
//
// The rule callers must follow is ATOMIC REJECTION: a set containing one
// inadmissible zone takes effect nowhere. A caller that filters with this
// predicate and keeps the survivors has reimplemented the drop, which deletes
// a declared hazard from the field.
//
// Note this is narrower than system-level fail-closed: an initial config that
// is rejected leaves ZERO zones, and whether planning should continue with no
// risk information at all is an open policy question, not something this
// predicate settles.
inline bool isValidPeak(double peak) {
  return std::isfinite(peak) && peak > 0.0 && peak <= 1.0;
}

}  // namespace risk
}  // namespace mmp

#endif  // _RISK_SHAPE_H_
