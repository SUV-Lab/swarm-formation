#pragma once

#include "path_manager/flight_evaluation.h"
#include "path_manager/zone_policy.h"
#include <mmp_vehicle_dynamics/vehicle_model.hpp>
#include "path_optimizer/poly_traj_utils.hpp"
#include <functional>
#include <limits>
#include <string>
#include <vector>

namespace path_manager {

// Existing planner coordinates (u), seconds, and global-z altitude. Only the
// PVA passed into VehicleModel is scaled to SI. These defaults are the legacy
// evaluator defaults; the ROS adapter reads the existing parameter names.
// Unit scales must be finite and positive; clearance and envelope thresholds
// must be finite. Their existing sign/comparison semantics are unchanged.
struct FlightEvaluationSettings {
  double unit_xy_m{100.0};
  double unit_z_m{100.0};
  double obstacle_clearance{0.3};
  double envelope_violation_max{0.25};  // fraction, compared with >=
  double envelope_peak_max{1.30};      // utilization, compared with >
};

// Providers are borrowed for ONE synchronous call. They must remain alive
// and stable until it returns; evaluation and environment updates are serial.
// No provider is retained in the result. Model const methods follow the
// VehicleModel concurrency contract. Successful numeric queries must return
// finite canonical data. Exceptions propagate to the caller; this boundary
// does not validate or repair a provider's implementation.
struct FlightEvaluationEnvironment {
  size_t zone_count{0};
  // Missing terrain provider means no terrain: sea-level zero, as before.
  // On a false return the provider must leave the initialized output alone.
  std::function<bool(double, double, double *)> terrain_elevation;
  // Required together whenever zone_count > 0. Use the existing canonical
  // primitives: routing contact q < 1.05 / visibility > 0.35, authored
  // contact q < 1.0 at the SAME visibility floor. Contact validates policy
  // generation, epoch, ordered shape identity and valid flag, returning
  // STALE/INVALID explicitly. Do not invent measured policy for a backend
  // that returned UNKNOWN. Risk is the existing untapered effective field.
  std::function<double(size_t, const Eigen::Vector3d &)> effective_risk;
  std::function<double(size_t, const Eigen::Vector3d &)> ellipsoid_radius;
  std::function<double(size_t, const Eigen::Vector3d &)> risk_visibility;
  std::function<ZoneContactResult(const ZonePolicySnapshot &, size_t,
                                  const Eigen::Vector3d &)> zone_contact;
  std::function<bool(const ZonePolicySnapshot &, size_t,
                     const Eigen::Vector3d &)> authored_contact;
};

// A trajectory and phase partition supplied by the caller, including an
// externally generated trajectory. Every piece has a finite positive duration
// and finite coefficients, and total duration must be finite. Numerically
// malformed input is unevaluated before any environment/model callback.
// References/pointers are borrowed only for
// evaluateFlight(). The final span ends at total duration; transitions lead
// contiguously and a terminal span is last. Sample spacing stays fixed at
// 0.1 s, matching the existing whole-flight gate (it is not continuous CCD).
struct FlightEvaluationInput {
  FlightEvaluationInput(const poly_traj::Trajectory &flight,
                        const std::vector<PhaseSpan> &spans)
      : trajectory(flight), phases(spans) {
    // External callers have not supplied measured policy by constructing an
    // evaluation request. The no-zone case still needs no policy at all.
    plan_policy.valid = false;
  }
  const poly_traj::Trajectory &trajectory;
  const std::vector<PhaseSpan> &phases;
  FlightEvaluationSettings settings;
  FlightEvaluationEnvironment environment;
  mmp_vehicle_dynamics::VehicleModel::Ptr model;  // null: dynamics disabled
  ZonePolicySnapshot plan_policy;
  std::vector<LegPolicySnapshot> leg_policies;
  const std::vector<size_t> *piece_leg{nullptr};
  uint64_t piece_epoch{0};
  uint64_t policy_epoch{0};
};

struct FlightPhaseMetrics {
  double min_agl{std::numeric_limits<double>::infinity()};
  double min_agl_t{0.0};
  double agl_sum{0.0};
  int n{0}, n_below_band{0}, n_below_ground{0};
  int env_n{0}, env_viol{0};
  double util_peak{0.0};
  mmp_vehicle_dynamics::EnvelopeLimit peak_limit{
      mmp_vehicle_dynamics::EnvelopeLimit::None};
  double risk_max{0.0}, risk_int{0.0};
  // Zone contact of the delivered flight. Containment alone is NOT the
  // question — policy lives in the per-zone DISPOSITION, exactly as the
  // transition candidate filter reads it: only HARD_AVOID contact is a
  // violation, while SOFT_UNAVOIDABLE / SOFT_ENDPOINT / SOFT_FALLBACK are
  // crossings the 3-pass policy deliberately allowed. Counting every
  // containment as "hard" reported 3 hard contacts on an r5 flight whose
  // final field was pass-2 all-soft — a permitted crossing dressed up as a
  // safety breach. The two are kept apart and both are reported.
  // Sampled at the dt below (0.1 s), so these are SAMPLE counts.
  int zone_hard_n{0};       // inside the AUTHORED volume (q < 1.0)
  double zone_hard_s{0.0};
  int zone_standoff_n{0};   // in the 1.0-1.05 routing standoff shell
  double zone_standoff_s{0.0};
  int zone_soft_n{0};
  double zone_soft_s{0.0};
  bool zone_contact_measurable{true};
  // [ZONE-CONTACT] WHERE and HOW DEEP, not just how many. A count alone
  // cannot separate a 2 m graze of the 1.05x standoff shell from a
  // traverse of the authored volume, and those two call for opposite
  // responses. q < 1 is inside the authored ellipsoid; the gate fires at
  // q < 1.05, so q_min is the number that says which one happened.
  double zone_hard_t_first{-1.0};
  double zone_hard_t_last{-1.0};
  int zone_hard_runs{0};       // contiguous contact intervals
  // Closest approach to any HARD_AVOID zone over the whole span, contact
  // or not. Distinct from zone_hard_qmin, which only exists when a contact
  // occurred.
  double zone_clear_qmin{1e9};
  Eigen::Vector3d zone_clear_p{Eigen::Vector3d::Zero()};
  double zone_clear_t{-1.0};
  int zone_clear_zone{-1};
  double zone_hard_qmin{1e9};
  Eigen::Vector3d zone_hard_qmin_p{Eigen::Vector3d::Zero()};
  double zone_hard_qmin_vis{-1.0};
  int zone_hard_qmin_zone{-1};
  // [LEG-POLICY] Piece boundaries where the leg changes hands, audited
  // EXHAUSTIVELY and separately from the 0.1 s grid. They are instants,
  // not dwell: they contribute counts and the refusal, never seconds.
  int zone_junction_n{0};        // leg-change boundaries examined
  int zone_junction_hard_n{0};   // ...of which entered an authored volume
};

enum class FlightEvaluationIssue {
  NONE,
  DEGENERATE_INPUT,
  SPAN_TIME,
  TRANSITION_ORDER,
  TERMINAL_ORDER,
  SPAN_DURATION,
  MISSING_ZONE_QUERIES,
  INVALID_TRAJECTORY,
  INVALID_SETTINGS
};
// Validate trajectory representation and the legacy phase contract. Shared
// with adapters so malformed trajectories/phases do not trigger environment
// queries or parameter declarations before refusal. issue_span is used only
// for the phase-specific issues; numeric trajectory errors have no phase.
FlightEvaluationIssue validateFlightPhases(
    const poly_traj::Trajectory &flight, const std::vector<PhaseSpan> &phases,
    size_t *issue_span = nullptr);

struct FlightLegHandoverContact {
  size_t from_leg{0}, to_leg{0};
  double time{0.0};
};

// Fully owned measurements and diagnostics, independent of input/provider
// lifetimes. No execution ID, publication or stored-trajectory mutation.
// Success/degraded/refused PlanResult policy remains with the coordinator;
// FlightVerdict retains the original measured conditions and predicates.
struct FlightEvaluationReport {
  FlightVerdict verdict;
  FlightEvaluationIssue issue{FlightEvaluationIssue::NONE};
  size_t issue_span{0};
  double duration{0.0};
  int piece_count{0};
  double sample_dt{0.1};
  bool model_enabled{false};
  bool per_piece_policy{false};
  bool missing_policy{false};
  size_t piece_leg_count{0}, leg_policy_count{0};
  double envelope_violation_max_pct{0.0};
  std::vector<PhaseSpan> phases;
  std::vector<FlightPhaseMetrics> phase_metrics;
  FlightPhaseMetrics total;
  std::vector<FlightLegHandoverContact> hard_handovers;
};

FlightEvaluationReport evaluateFlight(const FlightEvaluationInput &input);

// Rendering is separate from measurement: the caller chooses whether/where
// to emit these legacy-compatible lines. It needs no live environment/ROS.
enum class FlightEvaluationLogLevel { INFO, WARNING, ERROR };
struct FlightEvaluationLogLine {
  FlightEvaluationLogLevel level{FlightEvaluationLogLevel::INFO};
  std::string text;
};
std::vector<FlightEvaluationLogLine> formatFlightEvaluation(
    const FlightEvaluationReport &report);

}  // namespace path_manager
