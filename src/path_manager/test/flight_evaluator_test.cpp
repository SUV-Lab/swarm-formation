// Compile the production evaluator without ROS or PathManager. Analytic
// trajectories and explicit providers pin sampling, provenance and ownership.
#include "path_manager/flight_evaluator.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace pm = path_manager;
namespace vd = mmp_vehicle_dynamics;
namespace {
int failures = 0;
void check(bool condition, const char *message) {
  std::cout << (condition ? "[OK] " : "[FAIL] ") << message << '\n';
  if (!condition) ++failures;
}
bool near(double a, double b) { return std::abs(a - b) < 1e-12; }
poly_traj::Trajectory line(double duration = 0.25, double x = 0.0,
                           double z = 2.0) {
  poly_traj::CoefficientMat c = poly_traj::CoefficientMat::Zero();
  c.col(5) = Eigen::Vector3d(x, 0.2, z);
  c(0, 4) = 1.0;
  return poly_traj::Trajectory({duration}, {c});
}
std::vector<pm::PhaseSpan> phases(const poly_traj::Trajectory &trajectory) {
  return {{trajectory.getTotalDuration(), pm::PhaseKind::CRUISE, "external"}};
}

// A deterministic envelope: at x = 10 m cruise is reached, at x = 20 m it
// is lost again. SI checks make wrong axis/unit conversion observable.
class DomainVehicle final : public vd::VehicleModel {
 public:
  double peak{0.8};
  bool valid() const override { return true; }
  vd::VehicleLimits limits() const override { return {}; }
  vd::Evaluation evaluate(const Eigen::Vector3d &p, const Eigen::Vector3d &v,
                           const Eigen::Vector3d &a) const override {
    vd::Evaluation e;
    e.valid = near(p.y(), 20.0) && near(p.z(), 6.0) &&
              near(v.x(), 100.0) && a.isZero();
    e.speed_mps = p.x();
    return e;
  }
  vd::CostGradient penalty(const Eigen::Vector3d &, const Eigen::Vector3d &,
                            const Eigen::Vector3d &, double) const override {
    return {};
  }
  vd::EnvelopeDomain domain(const vd::Evaluation &e) const override {
    return {true, e.speed_mps >= 9.0 && e.speed_mps < 19.0, false};
  }
  double utilization(const vd::Evaluation &, vd::EnvelopeLimit *limit) const override {
    if (limit) *limit = vd::EnvelopeLimit::MaximumThrust;
    return peak;
  }
  bool withinEnvelope(const vd::Evaluation &) const override { return true; }
  vd::HandoffVerdict velocityVerdict(const Eigen::Vector3d &, double, double) const override {
    return {};
  }
  vd::HandoffVerdict handoffVerdict(const Eigen::Vector3d &, const Eigen::Vector3d &,
                                   const Eigen::Vector3d &, double, double) const override {
    return {};
  }
  vd::PlanningGeometry planningGeometry(double, double, double) const override {
    return {};
  }
};

pm::ZonePolicySnapshot policy(pm::ZoneDisposition disposition) {
  pm::ZonePolicySnapshot p;
  p.generation = 7;
  p.epoch = 3;
  p.zones.push_back({{Eigen::Vector3d::Zero(), 1.0, 0.5}, disposition});
  return p;
}
pm::FlightEvaluationEnvironment zoneEnvironment(
    std::function<double(const Eigen::Vector3d &)> radius) {
  pm::FlightEvaluationEnvironment e;
  e.zone_count = 1;
  e.effective_risk = [](size_t, const Eigen::Vector3d &) { return 0.2; };
  e.ellipsoid_radius = [radius](size_t, const Eigen::Vector3d &p) { return radius(p); };
  e.risk_visibility = [](size_t, const Eigen::Vector3d &) { return 1.0; };
  e.zone_contact = [radius](const pm::ZonePolicySnapshot &s, size_t i,
                            const Eigen::Vector3d &p) {
    if (!s.valid) return pm::ZoneContactResult::INVALID;
    if (s.generation != 7 || s.epoch != 3 || i >= s.zones.size() ||
        s.zones[i].zone.center != Eigen::Vector3d::Zero() ||
        s.zones[i].zone.reach != 1.0 || s.zones[i].zone.peak != 0.5)
      return pm::ZoneContactResult::STALE;
    return radius(p) < 1.05 ? pm::ZoneContactResult::CONTACT
                            : pm::ZoneContactResult::CLEAR;
  };
  e.authored_contact = [radius](const pm::ZonePolicySnapshot &, size_t,
                                const Eigen::Vector3d &p) { return radius(p) < 1.0; };
  return e;
}

void measurementsAndLifetime() {
  pm::FlightEvaluationReport retained;
  std::weak_ptr<int> lifetime;
  {
    auto flight = line();
    auto spans = phases(flight);
    pm::FlightEvaluationInput input{flight, spans};
    auto owner = std::make_shared<int>(0);
    lifetime = owner;
    input.environment.terrain_elevation = [owner](double, double, double *z) {
      ++*owner; *z = 0.5; return true;
    };
    retained = pm::evaluateFlight(input);
    check(retained.verdict.evaluated && retained.verdict.clean && *owner == 3,
          "0.1 second samples exclude the final instant and call terrain once each");
    check(retained.total.n == 3 && retained.total.min_agl == 1.5 &&
              retained.total.agl_sum == 4.5 && retained.total.n_below_band == 0,
          "AGL measurements use planner global z and retain phase/total sums");
    check(flight[0].getCoeffMat()(0, 4) == 1.0 && spans[0].name == "external",
          "evaluation does not mutate trajectory or phase input");
  }
  check(lifetime.expired() && retained.phases[0].name == "external" &&
            retained.phase_metrics[0].min_agl == 1.5,
        "owned report outlives trajectory, phases and borrowed provider owner");
  auto lines = pm::formatFlightEvaluation(retained);
  check(lines.size() == 4 &&
            lines.front().text == "[FINAL-EVAL] ===== whole-flight evaluation: 0.2 s, 1 pieces, 1 phase(s) =====" &&
            lines[2].text.find("verdict: CLEAN") != std::string::npos &&
            lines[3].text.find("measurable=true sample_dt=0.10") != std::string::npos,
        "legacy logs format later from owned report without any provider");
  auto flight = line(0.25, 0.0, -0.01);
  auto spans = phases(flight);
  pm::FlightEvaluationInput input{flight, spans};
  const auto underground = pm::evaluateFlight(input);
  check(underground.verdict.underground && underground.verdict.unflyable() &&
            underground.total.n_below_ground == 3,
        "missing terrain provider uses sea level and detects underground flight");
  input.environment.terrain_elevation = [](double, double, double *) { return false; };
  check(pm::evaluateFlight(input).total.min_agl == underground.total.min_agl,
        "unavailable terrain query retains the established zero fallback");
  input.environment.terrain_elevation = [](double, double, double *) -> bool {
    throw std::runtime_error("provider failure");
  };
  bool propagated = false;
  try { pm::evaluateFlight(input); } catch (const std::runtime_error &) { propagated = true; }
  check(propagated, "provider exception propagates synchronously without execution state");
}

void malformedPhases() {
  auto flight = line();
  int queries = 0;
  const auto checkIssue = [&](std::vector<pm::PhaseSpan> spans,
                              pm::FlightEvaluationIssue expected) {
    pm::FlightEvaluationInput input{flight, spans};
    input.environment.terrain_elevation = [&](double, double, double *) { ++queries; return false; };
    auto report = pm::evaluateFlight(input);
    check(!report.verdict.evaluated && report.issue == expected && queries == 0,
          "invalid phase partition remains unevaluated and does not query environment");
  };
  checkIssue({}, pm::FlightEvaluationIssue::DEGENERATE_INPUT);
  checkIssue({{0.0, pm::PhaseKind::CRUISE, "zero"}}, pm::FlightEvaluationIssue::SPAN_TIME);
  checkIssue({{0.1, pm::PhaseKind::CRUISE, "a"}, {0.1, pm::PhaseKind::CRUISE, "b"}},
             pm::FlightEvaluationIssue::SPAN_TIME);
  checkIssue({{0.1, pm::PhaseKind::CRUISE, "a"}, {0.25, pm::PhaseKind::TRANSITION, "b"}},
             pm::FlightEvaluationIssue::TRANSITION_ORDER);
  checkIssue({{0.1, pm::PhaseKind::TERMINAL, "a"}, {0.25, pm::PhaseKind::CRUISE, "b"}},
             pm::FlightEvaluationIssue::TERMINAL_ORDER);
  checkIssue({{0.24, pm::PhaseKind::CRUISE, "short"}}, pm::FlightEvaluationIssue::SPAN_DURATION);
  checkIssue({{std::numeric_limits<double>::quiet_NaN(), pm::PhaseKind::CRUISE, "nan"}},
             pm::FlightEvaluationIssue::SPAN_TIME);
}

void malformedNumerics() {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const auto reject = [&](const poly_traj::Trajectory &flight,
                          const pm::FlightEvaluationSettings &settings,
                          pm::FlightEvaluationIssue expected,
                          const char *message) {
    const auto spans = phases(flight);
    pm::FlightEvaluationInput input{flight, spans};
    input.settings = settings;
    int queries = 0;
    input.environment.terrain_elevation = [&](double, double, double *) {
      ++queries;
      return false;
    };
    const auto report = pm::evaluateFlight(input);
    const auto logs = pm::formatFlightEvaluation(report);
    check(!report.verdict.evaluated && report.issue == expected &&
              queries == 0 && report.phase_metrics.empty() &&
              logs.size() == 1 &&
              logs[0].level == pm::FlightEvaluationLogLevel::ERROR,
          message);
  };
  const auto valid = line();
  for (double bad : {nan, inf, -inf}) {
    auto coefficients = valid[0].getCoeffMat();
    coefficients(1, 0) = bad;
    const poly_traj::Trajectory flight({0.25}, {coefficients});
    reject(flight, {}, pm::FlightEvaluationIssue::INVALID_TRAJECTORY,
           "nonfinite trajectory coefficient is unevaluated before callbacks");
  }
  for (double bad : {nan, inf, -inf, 0.0, -0.1}) {
    const poly_traj::Trajectory flight({bad}, {valid[0].getCoeffMat()});
    reject(flight, {}, pm::FlightEvaluationIssue::INVALID_TRAJECTORY,
           "nonfinite or nonpositive piece duration is unevaluated before callbacks");
  }
  const double huge = std::numeric_limits<double>::max();
  const poly_traj::Trajectory overflow(
      {huge, huge}, {valid[0].getCoeffMat(), valid[0].getCoeffMat()});
  reject(overflow, {}, pm::FlightEvaluationIssue::INVALID_TRAJECTORY,
         "overflow of otherwise finite piece-duration sum is refused before sampling");
  const std::vector<pm::PhaseSpan> no_spans;
  const poly_traj::Trajectory empty;
  const auto tiny = line(1e-10);
  const auto tiny_spans = phases(tiny);
  check(pm::evaluateFlight({empty, no_spans}).issue ==
            pm::FlightEvaluationIssue::DEGENERATE_INPUT &&
            pm::evaluateFlight({tiny, tiny_spans}).issue ==
            pm::FlightEvaluationIssue::DEGENERATE_INPUT,
        "empty and finite tiny positive trajectories retain the degenerate outcome");

  using Field = double pm::FlightEvaluationSettings::*;
  const std::vector<Field> unit_fields{
      &pm::FlightEvaluationSettings::unit_xy_m,
      &pm::FlightEvaluationSettings::unit_z_m};
  for (Field field : unit_fields) {
    for (double bad : {nan, inf, -inf, 0.0, -1.0}) {
      pm::FlightEvaluationSettings settings;
      settings.*field = bad;
      reject(valid, settings, pm::FlightEvaluationIssue::INVALID_SETTINGS,
             "nonfinite or nonpositive unit scale is unevaluated before callbacks");
    }
  }
  const std::vector<Field> threshold_fields{
      &pm::FlightEvaluationSettings::obstacle_clearance,
      &pm::FlightEvaluationSettings::envelope_violation_max,
      &pm::FlightEvaluationSettings::envelope_peak_max};
  for (Field field : threshold_fields) {
    for (double bad : {nan, inf, -inf}) {
      pm::FlightEvaluationSettings settings;
      settings.*field = bad;
      reject(valid, settings, pm::FlightEvaluationIssue::INVALID_SETTINGS,
             "nonfinite clearance or envelope threshold is unevaluated before callbacks");
    }
  }
  const auto valid_spans = phases(valid);
  pm::FlightEvaluationInput signs{valid, valid_spans};
  signs.settings.obstacle_clearance = -1.0;
  signs.settings.envelope_violation_max = -1.0;
  signs.settings.envelope_peak_max = -1.0;
  const auto signed_report = pm::evaluateFlight(signs);
  check(signed_report.verdict.evaluated && signed_report.verdict.envelope_bad &&
            signed_report.total.n_below_band == 0,
        "finite negative thresholds and clearance retain their existing comparison semantics");
}

void dynamics() {
  auto flight = line();
  auto spans = phases(flight);
  pm::FlightEvaluationInput input{flight, spans};
  auto model = std::make_shared<DomainVehicle>();
  input.model = model;
  input.settings.unit_z_m = 3.0;
  auto report = pm::evaluateFlight(input);
  check(report.total.env_n == 2 && report.total.env_viol == 1 &&
            report.verdict.viol_pct == 50.0 && report.verdict.envelope_bad &&
            !report.verdict.no_cruise && !report.verdict.unflyable(),
        "SI conversion and cruise latch exclude pre-cruise but count later loss");
  input.settings.envelope_violation_max = 0.5;
  check(pm::evaluateFlight(input).verdict.envelope_bad,
        "violation fraction equal to the configured threshold still fails envelope");
  input.settings.envelope_violation_max = 0.51;
  check(pm::evaluateFlight(input).verdict.clean,
        "envelope budget affects clean verdict without changing sampled metrics");
  model->peak = 1.3;
  check(pm::evaluateFlight(input).verdict.clean,
        "peak utilization equal to configured threshold remains allowed");
  model->peak = 1.31;
  check(pm::evaluateFlight(input).verdict.envelope_bad,
        "peak utilization above threshold changes envelope verdict");
  spans[0].kind = pm::PhaseKind::TRANSITION;
  report = pm::evaluateFlight(input);
  check(report.total.env_n == 0 && report.verdict.no_cruise && report.verdict.unflyable() &&
            report.total.n == 3,
        "transition is excluded from cruise judgment but retains terrain samples");
  spans[0].kind = pm::PhaseKind::CRUISE;
  spans[0].name = "transition";
  check(pm::evaluateFlight(input).total.env_n == 2,
        "phase enum determines physics independently of display name");
  input.model.reset();
  check(pm::evaluateFlight(input).verdict.clean,
        "explicitly disabled dynamics does not invent a never-cruise refusal");
}

void zonesAndProvenance() {
  auto flight = line();
  auto spans = phases(flight);
  pm::FlightEvaluationInput input{flight, spans};
  input.environment = zoneEnvironment([](const Eigen::Vector3d &) { return 0.9; });
  check(!pm::evaluateFlight(input).verdict.policy_measurable,
        "external input defaults to unknown policy when zones exist");
  input.plan_policy = policy(pm::ZoneDisposition::HARD_AVOID);
  auto report = pm::evaluateFlight(input);
  check(report.verdict.zone_hard_n == 3 && report.verdict.unflyable() &&
            near(report.total.zone_hard_s, 0.3) && near(report.total.risk_int, 0.06),
        "authored contact refuses while exposure preserves original per-sample integration");
  input.environment = zoneEnvironment([](const Eigen::Vector3d &) { return 1.02; });
  report = pm::evaluateFlight(input);
  check(report.verdict.clean && !report.verdict.unflyable() &&
            report.verdict.zone_standoff_n == 3 && report.verdict.zone_hard_n == 0,
        "routing standoff stays distinct from authored volume and leaves degrade to coordinator");
  input.plan_policy.zones[0].disposition = pm::ZoneDisposition::SOFT_ENDPOINT;
  input.environment = zoneEnvironment([](const Eigen::Vector3d &) { return 0.9; });
  report = pm::evaluateFlight(input);
  check(report.verdict.zone_soft_n == 3 && report.verdict.zone_hard_n == 0 && report.verdict.clean,
        "measured soft policy permits crossing and still reports exposure");
  input.plan_policy.valid = false;
  report = pm::evaluateFlight(input);
  check(!report.verdict.policy_measurable && report.verdict.unflyable() && report.missing_policy,
        "unknown policy is unmeasurable rather than a zero-contact clearance claim");
  input.plan_policy.valid = true;
  input.plan_policy.epoch = 2;
  check(!pm::evaluateFlight(input).verdict.policy_measurable,
        "stale policy epoch remains unmeasurable through explicit contact provider");
  input.plan_policy = policy(pm::ZoneDisposition::HARD_AVOID);
  input.plan_policy.zones[0].zone.reach = 2.0;
  check(!pm::evaluateFlight(input).verdict.policy_measurable,
        "changed ordered zone shape remains unmeasurable");
  input.environment.zone_contact = {};
  report = pm::evaluateFlight(input);
  check(!report.verdict.evaluated && report.issue == pm::FlightEvaluationIssue::MISSING_ZONE_QUERIES,
        "incomplete external zone providers cannot produce evaluated clearance");

  auto split = line(0.15);
  split.append(line(0.10, 0.15));
  auto split_spans = phases(split);
  pm::FlightEvaluationInput legs{split, split_spans};
  legs.environment = zoneEnvironment([](const Eigen::Vector3d &p) {
    return std::abs(p.x() - 0.15) * 100.0;
  });
  legs.plan_policy = policy(pm::ZoneDisposition::HARD_AVOID);
  legs.plan_policy.valid = false;
  legs.leg_policies = {{0, 1, policy(pm::ZoneDisposition::SOFT_ENDPOINT)},
                       {1, 2, policy(pm::ZoneDisposition::HARD_AVOID)}};
  std::vector<size_t> attribution{0, 1};
  legs.piece_leg = &attribution;
  legs.piece_epoch = legs.policy_epoch = 3;
  report = pm::evaluateFlight(legs);
  check(report.per_piece_policy && report.verdict.policy_measurable &&
            report.verdict.zone_hard_n == 0 && report.verdict.zone_junction_n == 1 &&
            report.verdict.zone_junction_hard_n == 1 && report.verdict.unflyable() &&
            report.total.zone_hard_s == 0.0 && report.hard_handovers[0].time == 0.15,
        "off-grid leg handover uses stricter policy and counts an instant without dwell");
  check(pm::formatFlightEvaluation(report)[1].text.find("handover at t=0.150 s") != std::string::npos,
        "structured handover diagnostic preserves legacy log order and content");
  legs.piece_epoch = 2;
  report = pm::evaluateFlight(legs);
  check(!report.per_piece_policy && !report.verdict.policy_measurable && report.hard_handovers.empty(),
        "piece attribution from another search epoch cannot borrow valid leg policies");
  legs.piece_epoch = 3;
  legs.leg_policies[1].search_serial = 3;
  check(!pm::evaluateFlight(legs).per_piece_policy,
        "hole in per-leg capture serials refuses per-piece policy attribution");
  legs.leg_policies[1].search_serial = 2;
  attribution = {1, 0};
  check(!pm::evaluateFlight(legs).per_piece_policy,
        "backward leg attribution refuses even when all tags are in range");
  attribution = {0};
  check(!pm::evaluateFlight(legs).per_piece_policy,
        "piece-count mismatch cannot accidentally reuse the last solve attribution");
}
}  // namespace

int main() {
  measurementsAndLifetime();
  malformedPhases();
  malformedNumerics();
  dynamics();
  zonesAndProvenance();
  return failures ? 1 : 0;
}
