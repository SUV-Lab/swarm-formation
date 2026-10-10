#include "path_manager/flight_evaluator.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <utility>

namespace path_manager {

FlightEvaluationIssue validateFlightPhases(
    const poly_traj::Trajectory &flight, const std::vector<PhaseSpan> &spans,
    size_t *issue_span)
{
  if (issue_span) *issue_span = 0;
  if (flight.getPieceNum() == 0)
    return FlightEvaluationIssue::DEGENERATE_INPUT;
  for (int i = 0; i < flight.getPieceNum(); ++i) {
    const auto &piece = flight[i];
    if (!std::isfinite(piece.getDuration()) || piece.getDuration() <= 0.0 ||
        !piece.getCoeffMat().allFinite())
      return FlightEvaluationIssue::INVALID_TRAJECTORY;
  }
  const double T = flight.getTotalDuration();
  if (!std::isfinite(T)) return FlightEvaluationIssue::INVALID_TRAJECTORY;
  if (T <= 1e-9 || spans.empty()) {
    return FlightEvaluationIssue::DEGENERATE_INPUT;
  }
  // [S13] Span input contract, fail-closed to UNEVALUATED (the direct
  // fallback gate treats that as FAILED; the stitched paths build their
  // own spans, so a violation here is a programming error to surface):
  //  - every t_end finite and strictly increasing
  //  - the last t_end covers the whole flight
  //  - TRANSITION only as the LEADING contiguous spans (a transition in
  //    the middle of a cruise flight is not a thing this contract knows)
  //  - TERMINAL only as the last span
  {
    double prev = 0.0;
    bool cruise_seen = false;
    for (size_t i = 0; i < spans.size(); ++i) {
      const PhaseSpan &sp = spans[i];
      if (!std::isfinite(sp.t_end) || sp.t_end <= prev + 1e-9) {
        if (issue_span) *issue_span = i;
        return FlightEvaluationIssue::SPAN_TIME;
      }
      prev = sp.t_end;
      if (sp.kind == PhaseKind::TRANSITION) {
        if (cruise_seen) {
          if (issue_span) *issue_span = i;
          return FlightEvaluationIssue::TRANSITION_ORDER;
        }
      } else {
        cruise_seen = true;
      }
      if (sp.kind == PhaseKind::TERMINAL && i + 1 != spans.size()) {
        if (issue_span) *issue_span = i;
        return FlightEvaluationIssue::TERMINAL_ORDER;
      }
    }
    if (std::abs(spans.back().t_end - T) > 1e-6) {
      return FlightEvaluationIssue::SPAN_DURATION;
    }
  }
  return FlightEvaluationIssue::NONE;
}

FlightEvaluationReport evaluateFlight(const FlightEvaluationInput &input)
{
  FlightEvaluationReport report;
  const auto &flight = input.trajectory;
  const auto &spans = input.phases;
  const double T = flight.getTotalDuration();
  report.duration = T;
  report.piece_count = flight.getPieceNum();
  report.phases = spans;
  report.issue = validateFlightPhases(flight, spans, &report.issue_span);
  if (report.issue != FlightEvaluationIssue::NONE) return report;

  const auto &settings = input.settings;
  if (!std::isfinite(settings.unit_xy_m) || settings.unit_xy_m <= 0.0 ||
      !std::isfinite(settings.unit_z_m) || settings.unit_z_m <= 0.0 ||
      !std::isfinite(settings.obstacle_clearance) ||
      !std::isfinite(settings.envelope_violation_max) ||
      !std::isfinite(settings.envelope_peak_max)) {
    report.issue = FlightEvaluationIssue::INVALID_SETTINGS;
    return report;
  }

  const auto &environment = input.environment;
  if (environment.zone_count > 0 &&
      (!environment.effective_risk || !environment.ellipsoid_radius ||
       !environment.risk_visibility || !environment.zone_contact ||
       !environment.authored_contact)) {
    report.issue = FlightEvaluationIssue::MISSING_ZONE_QUERIES;
    return report;
  }
  const auto &model = input.model;
  report.model_enabled = model != nullptr;
  const double clr_band = input.settings.obstacle_clearance;
  const Eigen::Vector3d S(input.settings.unit_xy_m, input.settings.unit_xy_m,
                          input.settings.unit_z_m);
  report.phase_metrics.resize(spans.size());
  auto &st = report.phase_metrics;
  std::vector<bool> previous_hard_contact(spans.size(), false);
  const double dt = report.sample_dt;
  const size_t nz = environment.zone_count;
  const auto &eval_snap = input.plan_policy;

  // [LEG-POLICY] A multi-leg mission has no plan-wide policy — the front end
  // runs one search per leg and the same zone can be HARD_AVOID on one and
  // SOFT_ENDPOINT on the next, so eval_snap above is INVALID for it and every
  // sample reads as unmeasurable. It is measurable; it just has to be asked
  // per piece. Route edge i became MINCO piece i, so the piece a sample falls
  // in names the leg that was flying there, and that leg's captured policy is
  // the one in force.
  //
  // Every condition below is a refusal, not a repair: a map that does not
  // describe THIS trajectory, a leg with no capture, a capture that is not
  // valid, a leg list with a hole or a repeat, or a policy that does not cover
  // the current zones. Any of them and this falls back to eval_snap, which
  // then refuses in the usual way. Notably a STITCHED flight lands here — the
  // manager's map describes the last segment only, and the piece counts
  // disagree, which is exactly the disagreement that must not be papered over.
  static const std::vector<size_t> kNoPieceLeg;
  const auto &piece_leg = input.piece_leg ? *input.piece_leg : kNoPieceLeg;
  const auto &leg_pols = input.leg_policies;
  bool per_piece = !piece_leg.empty() && !leg_pols.empty() &&
                   piece_leg.size() ==
                       static_cast<size_t>(flight.getPieceNum()) &&
                   // The map and the captures must come from the SAME
                   // front-end run. Sizes agreeing is not that: a map minted
                   // one epoch ago names legs of a different search, and
                   // nothing in a vector of indices says which.
                   input.piece_epoch != 0 && input.piece_epoch == input.policy_epoch;
  if (per_piece) {
    // ORDER. A route is walked once, front to back, so its legs appear in
    // flight order and each hands over to the NEXT one. A map that goes
    // backwards, jumps a leg, or lets a leg reappear after another has taken
    // over is not a route this planner can have produced — and every one of
    // those shapes would still pass a size-and-range check while attributing
    // stretches of the flight to the wrong policy.
    //
    // The first entry is NOT required to be leg 0: a slice of a committed
    // route legitimately starts mid-route, so its tags begin at whichever leg
    // the cut landed on.
    for (size_t i = 1; i < piece_leg.size() && per_piece; ++i) {
      const size_t prev = piece_leg[i - 1], cur = piece_leg[i];
      if (cur != prev && cur != prev + 1) per_piece = false;
    }
    for (size_t i = 0; i < leg_pols.size() && per_piece; ++i) {
      // leg i at index i, and serial exactly i + 1. The searcher increments
      // its per-epoch counter once per leg search and captures with the new
      // value, so a clean run is 1, 2, 3, ... — CONTIGUOUS. Testing only that
      // serials increase accepts [1, 3], which is a set with a hole in the
      // middle, and this code's whole promise is that it refuses those. A
      // hole or a double-write is a programming error, and "some legs" is not
      // a set this may average over.
      if (leg_pols[i].leg != i || !leg_pols[i].policy.valid ||
          leg_pols[i].policy.zones.size() != nz ||
          leg_pols[i].search_serial != i + 1)
        per_piece = false;
    }
    for (size_t leg : piece_leg)
      if (leg >= leg_pols.size()) per_piece = false;
  }

  report.per_piece_policy = per_piece;
  report.missing_policy = nz > 0 && !per_piece && !eval_snap.valid;
  report.piece_leg_count = piece_leg.size();
  report.leg_policy_count = leg_pols.size();

  // The policy in force at time t, for the ordinary 0.1 s samples: whichever
  // piece the sample falls in.
  //
  // This deliberately does NOT try to detect boundaries. locatePieceIdx
  // advances only while `t > duration` (poly_traj_utils.hpp), so at exactly a
  // seam it returns the PREVIOUS piece with the remainder equal to that
  // piece's whole duration — not the next piece at remainder zero. A boundary
  // test written against the second (wrong) reading is dead code, which is
  // what the first version of this was: `rem <= 1e-9` never held. And a
  // 0.1 s grid lands on an exact seam only by coincidence, so even a correct
  // test here would go unexercised. Seams are handled exhaustively below
  // instead, where their times are known exactly rather than sampled for.
  const auto policyAt =
      [&](double t) -> const ZonePolicySnapshot & {
    if (!per_piece) return eval_snap;
    double rem = t;
    const int pi = flight.locatePieceIdx(rem);
    // Out of range is a refusal, not a clamp. Pinning it to 0 or to the last
    // index hands those samples leg 0's or the LAST leg's policy — which is
    // the exact failure this whole mechanism exists to prevent, arriving
    // through the back door. eval_snap is INVALID for a multi-leg mission, so
    // returning it here refuses in the usual way.
    if (pi < 0 || pi >= static_cast<int>(piece_leg.size())) return eval_snap;
    return leg_pols[piece_leg[static_cast<size_t>(pi)]].policy;
  };
  // Same counting doctrine as the solver's [CONV-REJECT] audit, latch
  // included: the pre-cruise ramp (a rest-start mission legitimately
  // begins below stall) stays out of the statistics, but sub-min-speed
  // dwell AFTER cruise entry counts into BOTH counters — dropping it from
  // both made the evaluator blind to mid-flight speed collapse, the exact
  // ~0%-scored failure the solver's latch was built to kill (review find:
  // a sub-stall arrival contract fed a 300 s helix that scored "0.0%").
  bool reached_cruise = false;
  for (double t = 0.0; t < T; t += dt) {
    size_t ph = 0;
    while (ph + 1 < spans.size() && t >= spans[ph].t_end) ++ph;
    FlightPhaseMetrics &s = st[ph];
    // Judgment selection by CONTRACT TYPE, never by the display name. A
    // TRANSITION span is outside the cruise model by definition: its
    // samples never enter the cruise envelope statistics (its own
    // evaluator attaches with the generator, per ADR-0002 — v0 limits are
    // experimental, not gates). Terrain/zone/risk below stay: those are
    // model-agnostic.
    const bool cruise_judged = spans[ph].kind != PhaseKind::TRANSITION;
    const Eigen::Vector3d p = flight.getPos(t);
    double g = 0.0;
    if (environment.terrain_elevation)
      environment.terrain_elevation(p.x(), p.y(), &g);  // false: sea level 0
    const double agl = p.z() - g;
    if (agl < s.min_agl) { s.min_agl = agl; s.min_agl_t = t; }
    s.agl_sum += agl;
    ++s.n;
    if (agl < clr_band) ++s.n_below_band;
    if (agl < 0.0) ++s.n_below_ground;
    if (model && cruise_judged) {
      const auto ev = model->evaluate(
          S.cwiseProduct(p), S.cwiseProduct(flight.getVel(t)),
          S.cwiseProduct(flight.getAcc(t)));
      if (ev.valid && model->domain(ev).cruise) {
        reached_cruise = true;
        ++s.env_n;
        mmp_vehicle_dynamics::EnvelopeLimit lim;
        const double u =
            model->utilization(ev, &lim);
        if (!model->withinEnvelope(ev)) ++s.env_viol;
        if (u > s.util_peak) { s.util_peak = u; s.peak_limit = lim; }
      } else if (reached_cruise) {
        ++s.env_n;
        ++s.env_viol;
      }
    }
    if (nz > 0) {
      double keep = 1.0;
      for (size_t zi = 0; zi < nz; ++zi)
        keep *= 1.0 - environment.effective_risk(zi, p);
      const double r = 1.0 - keep;
      s.risk_max = std::max(s.risk_max, r);
      s.risk_int += r * dt;

      bool hard = false, standoff = false, soft = false;
      int hard_zi = -1;
      double hard_q = 1e9;
      // [LEG-POLICY] The policy that was in force WHERE THIS SAMPLE FLIES.
      // Identical to eval_snap when there is one plan-wide policy to have.
      const ZonePolicySnapshot &snap = policyAt(t);
      // Closest approach to a HARD_AVOID zone, tracked on EVERY sample and
      // not only on contact. "No contact" is a threshold answer; how close
      // the flight came is the measurement, and without it two products on
      // the same route cannot be compared at all — which is exactly the
      // question a contact on one and not the other raises.
      for (size_t zi = 0; zi < nz; ++zi) {
        if (zi >= snap.zones.size() ||
            snap.zones[zi].disposition !=
                ZoneDisposition::HARD_AVOID)
          continue;
        const double q = environment.ellipsoid_radius(zi, p);
        if (q < s.zone_clear_qmin) {
          s.zone_clear_qmin = q;
          s.zone_clear_p = p;
          s.zone_clear_t = t;
          s.zone_clear_zone = static_cast<int>(zi);
        }
      }
      for (size_t zi = 0; zi < nz; ++zi) {
        switch (environment.zone_contact(snap, zi, p)) {
          case ZoneContactResult::CONTACT:
            if (zi < snap.zones.size() &&
                snap.zones[zi].disposition ==
                    ZoneDisposition::HARD_AVOID) {
              // WHICH BAND. zoneContact() reports the 1.05x standoff volume,
              // which is the surface a ROUTE is kept out of; only the
              // authored volume refuses a finished flight.
              if (environment.authored_contact(snap, zi, p)) hard = true;
              else standoff = true;
              // Only on contact, so the deep-sample cost never touches the
              // common path.
              const double q = environment.ellipsoid_radius(zi, p);
              if (q < hard_q) { hard_q = q; hard_zi = static_cast<int>(zi); }
            } else {
              soft = true;
            }
            break;
          case ZoneContactResult::STALE:
          case ZoneContactResult::INVALID:
            // A snapshot that cannot be judged must not read as "clear" —
            // that is how a zero contact count becomes a claim nobody
            // checked. The verdict below refuses CLEAN on this.
            s.zone_contact_measurable = false;
            break;
          case ZoneContactResult::CLEAR:
            break;
        }
      }
      if (standoff) { ++s.zone_standoff_n; s.zone_standoff_s += dt; }
      // The contact-detail fields track EITHER band, so the emitted line can
      // say which one the deepest sample was in; the counters above are what
      // decide refusal.
      if (hard || standoff) {
        if (hard) { ++s.zone_hard_n; s.zone_hard_s += dt; }
        if (s.zone_hard_t_first < 0.0) s.zone_hard_t_first = t;
        s.zone_hard_t_last = t;
        if (!previous_hard_contact[ph]) ++s.zone_hard_runs;
        if (hard_q < s.zone_hard_qmin) {
          s.zone_hard_qmin = hard_q;
          s.zone_hard_qmin_p = p;
          s.zone_hard_qmin_zone = hard_zi;
          s.zone_hard_qmin_vis =
              hard_zi >= 0 ? environment.risk_visibility(hard_zi, p) : -1.0;
        }
      }
      previous_hard_contact[ph] = hard || standoff;
      if (soft) { ++s.zone_soft_n; s.zone_soft_s += dt; }
    }
  }

  // [LEG-POLICY] Leg handovers, audited exhaustively and off the grid.
  //
  // A seam between two legs is one instant. The 0.1 s sampler will land on it
  // only by coincidence, so the sampler cannot be the thing that enforces the
  // handover rule — the times have to be enumerated. They are known exactly:
  // the cumulative duration at every piece i where piece_leg[i-1] differs
  // from piece_leg[i].
  //
  // At that instant the point belongs to both legs, so it is judged under the
  // STRICTER of the two policies. Stricter means HARD_AVOID and nothing else:
  // that is the only distinction drawn below, and inventing an order among
  // the SOFT_* kinds would be an ordering no consumer reads. The effect is
  // that an exemption earned on one leg — a start or goal containment — stops
  // at the junction instead of leaking into a leg that never had it.
  //
  // These feed their own counters. Folding them into the 0.1 s totals would
  // add dwell seconds to an instant and make `n` mean two different things.
  if (per_piece && nz > 0) {
    ZonePolicySnapshot merged;
    double t_seam = 0.0;
    for (size_t i = 0; i + 1 < piece_leg.size(); ++i) {
      t_seam += flight[static_cast<int>(i)].getDuration();
      const size_t a = piece_leg[i], b = piece_leg[i + 1];
      if (a == b) continue;
      size_t ph = 0;
      while (ph + 1 < spans.size() && t_seam >= spans[ph].t_end) ++ph;
      FlightPhaseMetrics &s = st[ph];
      ++s.zone_junction_n;

      merged = leg_pols[b].policy;
      const auto &other = leg_pols[a].policy;
      for (size_t zi = 0; zi < merged.zones.size() && zi < other.zones.size();
           ++zi) {
        if (other.zones[zi].disposition ==
            ZoneDisposition::HARD_AVOID)
          merged.zones[zi].disposition =
              ZoneDisposition::HARD_AVOID;
      }

      const Eigen::Vector3d p = flight.getPos(t_seam);
      bool jhard = false;
      for (size_t zi = 0; zi < nz; ++zi) {
        switch (environment.zone_contact(merged, zi, p)) {
          case ZoneContactResult::CONTACT:
            if (zi < merged.zones.size() &&
                merged.zones[zi].disposition ==
                    ZoneDisposition::HARD_AVOID &&
                environment.authored_contact(merged, zi, p))
              jhard = true;
            break;
          case ZoneContactResult::STALE:
          case ZoneContactResult::INVALID:
            s.zone_contact_measurable = false;
            break;
          case ZoneContactResult::CLEAR:
            break;
        }
      }
      if (jhard) {
        ++s.zone_junction_hard_n;
        report.hard_handovers.push_back({a, b, t_seam});
      }
    }
  }

  auto &tot = report.total;
  for (size_t ph = 0; ph < spans.size(); ++ph) {
    const FlightPhaseMetrics &s = st[ph];
    if (s.min_agl < tot.min_agl) {
      tot.min_agl = s.min_agl;
      tot.min_agl_t = s.min_agl_t;
    }
    tot.n += s.n;
    tot.agl_sum += s.agl_sum;
    tot.n_below_band += s.n_below_band;
    tot.n_below_ground += s.n_below_ground;
    tot.env_n += s.env_n;
    tot.env_viol += s.env_viol;
    tot.risk_max = std::max(tot.risk_max, s.risk_max);
    tot.risk_int += s.risk_int;
    tot.zone_hard_n += s.zone_hard_n;
    tot.zone_hard_s += s.zone_hard_s;
    tot.zone_standoff_n += s.zone_standoff_n;
    tot.zone_standoff_s += s.zone_standoff_s;
    tot.zone_hard_runs += s.zone_hard_runs;
    if (s.zone_hard_t_first >= 0.0 &&
        (tot.zone_hard_t_first < 0.0 ||
         s.zone_hard_t_first < tot.zone_hard_t_first))
      tot.zone_hard_t_first = s.zone_hard_t_first;
    if (s.zone_hard_t_last > tot.zone_hard_t_last)
      tot.zone_hard_t_last = s.zone_hard_t_last;
    if (s.zone_hard_qmin < tot.zone_hard_qmin) {
      tot.zone_hard_qmin = s.zone_hard_qmin;
      tot.zone_hard_qmin_p = s.zone_hard_qmin_p;
      tot.zone_hard_qmin_vis = s.zone_hard_qmin_vis;
      tot.zone_hard_qmin_zone = s.zone_hard_qmin_zone;
    }
    if (s.zone_clear_qmin < tot.zone_clear_qmin) {
      tot.zone_clear_qmin = s.zone_clear_qmin;
      tot.zone_clear_p = s.zone_clear_p;
      tot.zone_clear_t = s.zone_clear_t;
      tot.zone_clear_zone = s.zone_clear_zone;
    }
    tot.zone_soft_n += s.zone_soft_n;
    tot.zone_soft_s += s.zone_soft_s;
    tot.zone_junction_n += s.zone_junction_n;
    tot.zone_junction_hard_n += s.zone_junction_hard_n;
    tot.zone_contact_measurable &= s.zone_contact_measurable;
    if (s.util_peak > tot.util_peak) {
      tot.util_peak = s.util_peak;
      tot.peak_limit = s.peak_limit;
    }
  }
  const double viol_pct =
      tot.env_n ? 100.0 * tot.env_viol / tot.env_n : 0.0;
  // A flight that never once reaches cruise is unflyable for this airframe
  // — the solver's own degenerate rule (viol := 1.0), mirrored here so
  // no-data can never read as clean. The envelope threshold is the SAME
  // configurable gate the per-solve audit uses, not a second number.
  const double viol_max_pct =
      100.0 * input.settings.envelope_violation_max;
  const bool no_cruise = model != nullptr && tot.env_n == 0;
  // [PHASE-DEP] peak joins the verdict: 0.7% violation TIME hid a 299%
  // instantaneous thrust demand as CLEAN (review find). Same parameter the
  // departure acceptance gate uses.
  const double peak_chk =
      input.settings.envelope_peak_max;
  const bool peak_bad = tot.util_peak > peak_chk;
  // A flight that entered a HARD_AVOID volume, or one whose zone policy could
  // not be judged at all, must not read CLEAN. Leaving these out of the
  // condition is what let the new metric be printed beside a CLEAN verdict it
  // had no influence over — the number was reported, not enforced. An
  // unjudgeable snapshot is treated like a contact rather than like clear:
  // "we could not check" is not evidence of safety.
  // [LEG-POLICY] A handover that sits inside an authored volume under the
  // stricter of the two policies refuses the flight exactly as a sampled
  // contact does. It is one instant rather than 0.1 s of dwell, which is why
  // it is counted apart — not why it is treated more leniently.
  const bool zone_bad = tot.zone_hard_n > 0 || tot.zone_junction_hard_n > 0 ||
                        !tot.zone_contact_measurable;
  // Envelope alone, kept apart from the zone reasons: a caller that has to
  // NAME the cause cannot recover it from `clean`.
  const bool envelope_bad = viol_pct >= viol_max_pct || peak_bad;
  const bool clean = tot.n_below_ground == 0 && !no_cruise &&
                     !envelope_bad && !zone_bad;
  FlightVerdict v;
  v.evaluated = true;
  v.clean = clean;
  v.envelope_bad = envelope_bad;
  v.underground = tot.n_below_ground != 0;
  v.no_cruise = no_cruise;
  v.zone_hard_n = tot.zone_hard_n;
  v.zone_standoff_n = tot.zone_standoff_n;
  v.zone_soft_n = tot.zone_soft_n;
  v.zone_junction_n = tot.zone_junction_n;
  v.zone_junction_hard_n = tot.zone_junction_hard_n;
  v.policy_measurable = tot.zone_contact_measurable;
  v.viol_pct = viol_pct;
  v.util_peak = tot.util_peak;
  report.verdict = v;
  report.envelope_violation_max_pct = viol_max_pct;
  return report;
}

namespace {
// Formatting is intentionally outside evaluation. A returned report can be
// inspected or serialized without formatting or emitting any log line.
class Lines {
 public:
  template <class... Args>
  void append(FlightEvaluationLogLevel level, const char *fmt, Args... args) {
    const int size = std::snprintf(nullptr, 0, fmt, args...);
    if (size < 0) return;
    std::vector<char> buffer(static_cast<size_t>(size) + 1);
    std::snprintf(buffer.data(), buffer.size(), fmt, args...);
    values.push_back({level, std::string(buffer.data(), static_cast<size_t>(size))});
  }
  template <class... Args> void infof(const char *fmt, Args... args) {
    append(FlightEvaluationLogLevel::INFO, fmt, args...);
  }
  template <class... Args> void warnf(const char *fmt, Args... args) {
    append(FlightEvaluationLogLevel::WARNING, fmt, args...);
  }
  template <class... Args> void errorf(const char *fmt, Args... args) {
    append(FlightEvaluationLogLevel::ERROR, fmt, args...);
  }
  std::vector<FlightEvaluationLogLine> values;
};
}  // namespace

std::vector<FlightEvaluationLogLine> formatFlightEvaluation(
    const FlightEvaluationReport &report)
{
  Lines lines;
  const auto &spans = report.phases;
  switch (report.issue) {
    case FlightEvaluationIssue::NONE: break;
    case FlightEvaluationIssue::DEGENERATE_INPUT: return {};
    case FlightEvaluationIssue::SPAN_TIME:
      lines.errorf("[FINAL-EVAL] span contract: t_end not finite/strictly "
                   "increasing at span %zu", report.issue_span);
      return std::move(lines.values);
    case FlightEvaluationIssue::TRANSITION_ORDER:
      lines.errorf("[FINAL-EVAL] span contract: TRANSITION after a "
                   "non-transition span (%zu)", report.issue_span);
      return std::move(lines.values);
    case FlightEvaluationIssue::TERMINAL_ORDER:
      lines.errorf("[FINAL-EVAL] span contract: TERMINAL not last (%zu)",
                   report.issue_span);
      return std::move(lines.values);
    case FlightEvaluationIssue::SPAN_DURATION:
      lines.errorf("[FINAL-EVAL] span contract: last t_end %.6f != flight "
                   "duration %.6f", spans.back().t_end, report.duration);
      return std::move(lines.values);
    case FlightEvaluationIssue::INVALID_TRAJECTORY:
      lines.errorf("%s", "[FINAL-EVAL] input contract: trajectory requires "
                   "finite positive piece durations, finite total duration "
                   "and finite coefficients");
      return std::move(lines.values);
    case FlightEvaluationIssue::INVALID_SETTINGS:
      lines.errorf("%s", "[FINAL-EVAL] input contract: unit scales must be "
                   "finite and positive; clearance and envelope thresholds "
                   "must be finite");
      return std::move(lines.values);
    case FlightEvaluationIssue::MISSING_ZONE_QUERIES:
      lines.errorf("%s", "[FINAL-EVAL] input contract: zone queries missing");
      return std::move(lines.values);
  }
  // A default-constructed report has nothing to report.
  if (!report.verdict.evaluated) return {};
  if (report.missing_policy)
    lines.warnf("[LEG-POLICY] no per-piece attribution (%zu tags, %d pieces, "
                "%zu legs) and no valid plan-wide policy — this flight cannot "
                "be cleared", report.piece_leg_count, report.piece_count,
                report.leg_policy_count);
  else if (report.per_piece_policy)
    lines.infof("[LEG-POLICY] audit reads policy per piece: %zu pieces over "
                "%zu legs", report.piece_leg_count, report.leg_policy_count);
  for (const auto &handover : report.hard_handovers)
    lines.warnf("[LEG-POLICY] leg %zu -> %zu handover at t=%.3f s is "
                "inside an authored volume under the stricter of the two "
                "policies", handover.from_leg, handover.to_leg, handover.time);
  lines.infof("[FINAL-EVAL] ===== whole-flight evaluation: %.1f s, %d "
              "pieces, %zu phase(s) =====", report.duration,
              report.piece_count, spans.size());
  double t0 = 0.0;
  const bool model = report.model_enabled;
  for (size_t ph = 0; ph < spans.size(); ++ph) {
    const FlightPhaseMetrics &s = report.phase_metrics[ph];
    if (s.n > 0) {
      // A phase with dynamics on but ZERO cruise-domain samples has no
      // measurement — "0.0%" would print no-data as a clean reading.
      char env[96];
      if (!model) {
        snprintf(env, sizeof env, "env model off");
      } else if (spans[ph].kind == PhaseKind::TRANSITION) {
        snprintf(env, sizeof env,
                 "env n/a (TRANSITION — cruise judgment not applicable)");
      } else if (s.env_n == 0) {
        snprintf(env, sizeof env, "env NO CRUISE DATA (pre-cruise ramp)");
      } else {
        snprintf(env, sizeof env, "env viol %5.1f%% peak %5.1f%% (%s)",
                 100.0 * s.env_viol / s.env_n, 100.0 * s.util_peak,
                 mmp_vehicle_dynamics::envelopeLimitName(s.peak_limit));
      }
      lines.infof(
          "[FINAL-EVAL] %-9s %6.1f s | AGL min %7.3f@%.0fs mean %6.2f u, "
          "<band %4.1f%%, underground %s | %s | risk max %.3f, "
          "exposure %.1f s, zone contact hard %d (%.1f s) soft %d (%.1f s) "
          "@0.1s sampling%s",
          spans[ph].name.c_str(), spans[ph].t_end - t0, s.min_agl,
          s.min_agl_t, s.agl_sum / s.n, 100.0 * s.n_below_band / s.n,
          s.n_below_ground ? "YES" : "no", env, s.risk_max, s.risk_int,
          s.zone_hard_n, s.zone_hard_s, s.zone_soft_n, s.zone_soft_s,
          s.zone_contact_measurable ? "" : " [UNMEASURABLE: stale/invalid "
                                           "zone snapshot]");
    }
    t0 = spans[ph].t_end;
  }
  const auto &tot = report.total;
  const auto &v = report.verdict;
  const bool clean = v.clean;
  const bool no_cruise = v.no_cruise;
  const double viol_pct = v.viol_pct;
  const double viol_max_pct = report.envelope_violation_max_pct;
  const double dt = report.sample_dt;
  if (clean) {
    lines.infof("[FINAL-EVAL] verdict: CLEAN — AGL min %.3f u, env viol "
                "%.1f%% (peak %.1f%% %s), risk exposure %.1f s, "
                "zone contact hard 0 standoff %d (%.1f s) soft %d (%.1f s) "
                "@0.1s sampling",
                tot.min_agl, viol_pct, 100.0 * tot.util_peak,
                mmp_vehicle_dynamics::envelopeLimitName(tot.peak_limit),
                tot.risk_int, tot.zone_standoff_n, tot.zone_standoff_s,
                tot.zone_soft_n, tot.zone_soft_s);
  } else {
    lines.warnf("[FINAL-EVAL] verdict: CHECK — %s%s%s%s%s%s(AGL min %.3f u "
                "@%.0fs, env viol %.1f%%) — the stitched flight carries "
                "hazards no per-solve audit saw",
                tot.n_below_ground ? "TERRAIN OVERLAP " : "",
                no_cruise ? "NEVER REACHES CRUISE " : "",
                tot.zone_hard_n > 0 ? "AUTHORED ZONE ENTERED " : "",
                tot.zone_junction_hard_n > 0 ? "LEG HANDOVER IN ZONE " : "",
                !tot.zone_contact_measurable ? "ZONE POLICY UNEVALUATED " : "",
                !no_cruise && viol_pct >= viol_max_pct ? "ENVELOPE " : "",
                tot.min_agl, tot.min_agl_t, viol_pct);
  }
  // Machine-readable, always emitted, one line. A reader (and the A/B
  // parser) must not have to infer zone safety from the ABSENCE of a
  // warning word — that is fail-open by construction: a reworded log or a
  // dropped call both read as "measurable, no contact".
  lines.infof("[ZONE-AUDIT] hard=%d hard_s=%.1f standoff=%d standoff_s=%.1f "
              "soft=%d soft_s=%.1f measurable=%s sample_dt=%.2f "
              "risk_max=%.4f risk_exposure_s=%.1f junctions=%d "
              "junction_hard=%d",
              v.zone_hard_n, tot.zone_hard_s,
              v.zone_standoff_n, tot.zone_standoff_s,
              v.zone_soft_n, tot.zone_soft_s,
              v.policy_measurable ? "true" : "false", dt,
              tot.risk_max, tot.risk_int, tot.zone_junction_n,
              tot.zone_junction_hard_n);
  // The count above says a breach happened; this says WHICH breach. Without
  // it, a 2 m graze of the 1.05x standoff shell and a traverse of the
  // authored volume are the same line — and they are not the same event.
  // q_min < 1.00 means the flight was inside the zone the mission authored;
  // 1.00 <= q_min < 1.05 means it stayed in the routing standoff margin,
  // where the optimizer's risk term is zero by construction and risk_max
  // above is therefore 0.0000 whatever the flight did.
  // runs= is summed per phase, so a contact straddling a phase boundary
  // reads as two.
  // Emitted for every flight that had a HARD_AVOID zone to stay clear of,
  // whether or not it touched anything. q_min is on the same scale the two
  // thresholds use: < 1.00 entered the authored volume, < 1.05 entered the
  // routing standoff, and anything above is the margin that was actually
  // kept.
  if (tot.zone_clear_zone >= 0) {
    lines.infof("[ZONE-CLEARANCE] zone=%d q_min=%.5f t=%.2fs "
                "at (%.2f, %.2f, %.2f)",
                tot.zone_clear_zone, tot.zone_clear_qmin, tot.zone_clear_t,
                tot.zone_clear_p.x(), tot.zone_clear_p.y(),
                tot.zone_clear_p.z());
  }
  if (tot.zone_hard_n > 0 || tot.zone_standoff_n > 0) {
    lines.infof("[ZONE-CONTACT] zone=%d t=[%.2f, %.2f]s runs=%d q_min=%.5f "
                "%s vis=%.3f at (%.2f, %.2f, %.2f)",
                tot.zone_hard_qmin_zone, tot.zone_hard_t_first,
                tot.zone_hard_t_last, tot.zone_hard_runs, tot.zone_hard_qmin,
                tot.zone_hard_qmin < 1.0 ? "INSIDE-AUTHORED" : "standoff-shell",
                tot.zone_hard_qmin_vis, tot.zone_hard_qmin_p.x(),
                tot.zone_hard_qmin_p.y(), tot.zone_hard_qmin_p.z());
  }
  return std::move(lines.values);
}

}  // namespace path_manager
