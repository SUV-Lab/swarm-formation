// Pins the RISK CONTRACT — not the risk formula.
//
// Audit PA-1 (2026-08-19) found that changing the searcher's attenuation from
// u^2 to u passed the entire regression (86/0/0, 71/71, 154/0) while moving
// nine of fourteen routes. The exponent was then examined as a contract
// question and NOT approved as one: no tracked document requires it, the
// flight-refusal verdict never reads it, and the optimizer used a cubic decay
// until 47ce135 unified it to the quadratic for field sharing.
//
// So this file deliberately does not assert the exponent. It asserts the six
// properties that were approved, each of which holds for any u^k with k > 0:
//
//   1. maximum at the centre, zero at the boundary
//   2. monotonically non-increasing with radius
//   3. finite and non-negative
//   4. every layer prices risk through the one shared function
//   5. value and analytic derivative always agree
//   6. peak is admissible only in (0, 1], and a set containing an
//      inadmissible zone is rejected ATOMICALLY  (PA-7)
//
// "Atomic rejection" is the precise claim, and it is narrower than
// system-level fail-closed: a rejected initial config leaves zero zones and
// planning continues. Whether it should is an open policy question.
//
// Property 4 is the one that kills the original mutation: re-inlining u*u at a
// single call site separates that layer from mmp::risk::shape, and the
// cross-layer checks below read both layers at the same points.

#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>

#include "path_manager/path_manager.h"
#include "path_planner/risk_shape.h"

namespace {

int g_pass = 0;
int g_fail = 0;

void check(bool ok, const std::string &what) {
  if (ok) {
    ++g_pass;
    std::printf("  [OK]   %s\n", what.c_str());
  } else {
    ++g_fail;
    std::printf("  [FAIL] %s\n", what.c_str());
  }
}

// ── property 1/2/3: the shape itself ────────────────────────────────────
void testShapeProperties() {
  std::printf("\n[1-3] shape properties (exponent-agnostic)\n");

  check(std::abs(mmp::risk::shape(0.0) - 1.0) < 1e-15,
        "maximum at the centre: shape(0) == 1");

  bool boundary_zero = true;
  for (double q : {1.0, 1.0 + 1e-12, 1.5, 10.0, 1e9}) {
    if (mmp::risk::shape(q) != 0.0) boundary_zero = false;
  }
  check(boundary_zero, "zero at and beyond the boundary: shape(q >= 1) == 0");

  // NaN must not leak a value. The comparison is written so an unordered
  // result takes the zero branch; a naive `if (q >= 1.0) return 0` would
  // return the NaN-derived u*u instead.
  check(mmp::risk::shape(std::numeric_limits<double>::quiet_NaN()) == 0.0,
        "NaN radius yields 0, not a NaN moat");

  bool monotone = true, bounded = true, finite = true;
  double prev = mmp::risk::shape(0.0);
  for (int i = 1; i <= 20000; ++i) {
    const double q = i * (1.2 / 20000.0);
    const double v = mmp::risk::shape(q);
    if (v > prev + 1e-15) monotone = false;
    if (v < 0.0 || v > 1.0) bounded = false;
    if (!std::isfinite(v)) finite = false;
    prev = v;
  }
  check(monotone, "monotonically non-increasing in q (20000 samples)");
  check(bounded, "stays within [0, 1]");
  check(finite, "finite everywhere sampled");

  // The centre value must not depend on the shape at all: it is peak*visibility
  // for ANY exponent, which is why it is safe to pin as an exact number.
  check(std::abs(mmp::risk::shape(0.0) * 0.8 - 0.8) < 1e-15,
        "centre value is peak exactly, independent of the exponent");
}

// ── property 5: value and analytic derivative agree ─────────────────────
void testDerivativeAgreement() {
  std::printf("\n[5] analytic derivative matches the value it differentiates\n");

  bool agrees = true;
  double worst = 0.0, worst_q = -1.0;
  const double h = 1e-6;
  for (int i = 1; i < 1000; ++i) {
    const double q = i * (1.0 / 1000.0);
    if (q - h <= 0.0 || q + h >= 1.0) continue;
    const double numeric =
        (mmp::risk::shape(q + h) - mmp::risk::shape(q - h)) / (2.0 * h);
    const double analytic = mmp::risk::shapeDeriv(q);
    const double err = std::abs(numeric - analytic);
    if (err > worst) { worst = err; worst_q = q; }
    if (err > 1e-7) agrees = false;
  }
  std::printf("         worst |numeric - analytic| = %.3e at q=%.3f\n",
              worst, worst_q);
  check(agrees, "central difference matches shapeDeriv within 1e-7");

  bool non_positive = true;
  for (int i = 0; i <= 1200; ++i) {
    if (mmp::risk::shapeDeriv(i * 0.001) > 0.0) non_positive = false;
  }
  check(non_positive, "derivative is non-positive everywhere (property 2)");
  check(mmp::risk::shapeDeriv(1.0) == 0.0 &&
            mmp::risk::shapeDeriv(2.0) == 0.0,
        "derivative is 0 outside the envelope");
}

// ── property 6 (PA-7): admissible peak ──────────────────────────────────
void testPeakValidation() {
  std::printf("\n[6] PA-7: peak is admissible only in (0, 1]\n");

  const double inf = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();

  bool accepts_valid = true;
  for (double p : {1e-9, 0.1, 0.5, 0.8, 1.0}) {
    if (!mmp::risk::isValidPeak(p)) accepts_valid = false;
  }
  check(accepts_valid, "accepts (0, 1]: 1e-9, 0.1, 0.5, 0.8, 1.0");

  struct Case { double v; const char *name; };
  const Case bad[] = {
      {0.0, "zero"},        {-0.0, "negative zero"}, {-1.0, "negative"},
      {-1e-9, "tiny negative"}, {1.0 + 1e-12, "just above 1"},
      {1.5, "> 1"},         {100.0, "legacy risk level 100"},
      {inf, "+Inf"},        {-inf, "-Inf"},          {nan, "NaN"},
  };
  bool rejects_all = true;
  for (const auto &c : bad) {
    if (mmp::risk::isValidPeak(c.v)) {
      rejects_all = false;
      std::printf("         accepted %s (%.3g) — must not\n", c.name, c.v);
    }
  }
  check(rejects_all,
        "rejects 0, -0, negative, >1, +/-Inf, NaN (10 cases)");
}

// ── property 4: every layer prices risk through one function ────────────
//
// Reads the FRONT END's composed field and the MANAGER's per-zone value at the
// same points and requires both to equal the shared shape. Re-inlining u*u at
// either site separates it from the other two and this dies.
void testCrossLayerIdentity(const char *yaml) {
  std::printf("\n[4] cross-layer field identity\n");

  // The SHIPPED parameter set, not code defaults: optimization/weight_Risk
  // declares 0.0 in code and 10000.0 in the yaml, so a run without the file
  // evaluates a back end whose risk term is switched off — every cost is 0 and
  // the gradient check passes vacuously. (Measured: it did.)
  rclcpp::NodeOptions opts;
  opts.arguments({"--ros-args", "--params-file", yaml});
  auto node = std::make_shared<rclcpp::Node>("risk_contract_test", opts);
  node->declare_parameter("drone_id", 1);
  path_manager::PathManager manager(node);
  auto *pm = &manager;

  path_manager::RiskZone z;
  z.center = Eigen::Vector3d(60.0, 60.0, 8.0);
  z.reach = 20.0;
  z.peak = 0.8;
  const bool accepted = pm->setRiskZonesRuntime({z});
  check(accepted, "a well-formed zone is accepted");

  // Sample along +x from the centre so q sweeps 0 -> past the rim. Staying on
  // the zone's own z keeps the vertical term out of it, which matters because
  // the layers derive the vertical semi-axis independently.
  int compared = 0;
  double worst_layers = 0.0, worst_shape = 0.0;
  bool inside_positive = true, outside_zero = true;
  for (int i = 0; i <= 40; ++i) {
    const Eigen::Vector3d p(z.center.x() + i * (z.reach * 1.25 / 40.0),
                            z.center.y(), z.center.z());
    const double q = pm->getZoneEllipsoidRadius(0, p);
    const double vis = pm->getRiskVisibility(0, p);

    const double fe = pm->getFrontEndRisk(p);
    const double mg = pm->getEffectiveRisk(0, p);
    const double ref =
        std::min(z.peak * mmp::risk::shape(q) * vis, mmp::risk::kMoatCap);

    worst_layers = std::max(worst_layers, std::abs(fe - mg));
    worst_shape = std::max(worst_shape, std::abs(mg - ref));
    ++compared;

    if (q < 0.95 && vis > 0.0 && !(mg > 0.0)) inside_positive = false;
    if (q >= 1.0 && (mg != 0.0 || fe != 0.0)) outside_zero = false;
  }
  std::printf("         %d points; max |front-end - manager| = %.3e, "
              "max |manager - shape| = %.3e\n",
              compared, worst_layers, worst_shape);

  check(worst_layers < 1e-12,
        "front end and manager agree pointwise (single-copy mutation dies)");
  check(worst_shape < 1e-12,
        "both equal peak * mmp::risk::shape(q) * visibility");
  check(inside_positive, "risk is positive inside the envelope");
  check(outside_zero, "risk is exactly zero at and beyond the rim");

  // Adding a zone must never lower the composed field (OR-composition).
  path_manager::RiskZone z2 = z;
  z2.center = Eigen::Vector3d(75.0, 60.0, 8.0);
  const Eigen::Vector3d probe(68.0, 60.0, 8.0);
  const double one = pm->getFrontEndRisk(probe);
  check(pm->setRiskZonesRuntime({z, z2}), "a second well-formed zone is accepted");
  const double two = pm->getFrontEndRisk(probe);
  std::printf("         composed risk at probe: 1 zone %.6f -> 2 zones %.6f\n",
              one, two);
  check(two >= one - 1e-15, "adding a zone never lowers the composed risk");

  // PA-7 at the boundary: a malformed set must change nothing.
  path_manager::RiskZone bad = z;
  bad.peak = 1.5;
  const bool rejected = !pm->setRiskZonesRuntime({z, bad});
  const double after = pm->getFrontEndRisk(probe);
  check(rejected, "a set containing peak=1.5 is rejected");
  check(std::abs(after - two) < 1e-15,
        "the rejected update left the active field untouched (atomic "
        "rejection, not a partial adopt)");

  // ── the BACK END, on the cost L-BFGS actually minimises ───────────────
  //
  // The front-end/manager comparison above cannot see the optimizer: its copy
  // lives inside the cost loop, so a mutation there survives a hook that reads
  // a parallel path. Measured: re-inlining the optimizer's shape call alone
  // left this file at 23/0 before this block existed.
  //
  // Differentiating the REAL cost numerically and comparing against the
  // analytic gradient the solver uses closes it. A value mutated away from
  // shape() while the gradient still comes from shapeDeriv() (or the reverse)
  // separates the two immediately.
  // The optimizer is built lazily, so a bare PathManager has none and every
  // back-end read would return false — which is indistinguishable from "the
  // checks passed" unless something asserts it. Build it, then assert it.
  pm->initOptimizer();
  check(pm->setRiskZonesRuntime({z}), "back end: single zone re-installed");
  {
    const Eigen::Vector3d vel(12.0, 0.0, 0.0);
    const double h = 1e-5;
    bool grad_ok = true, any_cost = false, reachable = false;
    double worst_rel = 0.0;
    int checked = 0;
    for (int i = 1; i <= 24; ++i) {
      const Eigen::Vector3d p(z.center.x() + i * (z.reach * 1.1 / 24.0),
                              z.center.y() + 1.0, z.center.z() + 0.5);
      Eigen::Vector3d gp, gv;
      double c = 0.0;
      // false means "this point contributes nothing" OR "there is no
      // optimizer". Only the second is a broken fixture, so reachability is
      // asserted from the points that DID answer.
      if (!pm->getBackEndRiskCost(p, vel, gp, gv, c)) continue;
      reachable = true;
      if (c > 0.0) any_cost = true;
      for (int ax = 0; ax < 3; ++ax) {
        Eigen::Vector3d pp = p, pm_ = p;
        pp[ax] += h; pm_[ax] -= h;
        Eigen::Vector3d g1, g2;
        double cp = 0.0, cm = 0.0;
        pm->getBackEndRiskCost(pp, vel, g1, g2, cp);
        pm->getBackEndRiskCost(pm_, vel, g1, g2, cm);
        const double numeric = (cp - cm) / (2.0 * h);
        const double denom = std::max(1.0, std::abs(numeric));
        const double rel = std::abs(numeric - gp[ax]) / denom;
        if (rel > worst_rel) worst_rel = rel;
        if (rel > 1e-4) grad_ok = false;
        ++checked;
      }
    }
    std::printf("         %d directional derivatives; worst relative error "
                "= %.3e\n", checked, worst_rel);
    check(reachable, "back end: the optimizer exists and answers");
    check(any_cost, "back end: the probe points actually cost something");
    check(checked > 0 && grad_ok,
          "back end: analytic gradient matches the numeric derivative of the "
          "cost it belongs to");

    Eigen::Vector3d gp, gv;
    double far_cost = -1.0;
    pm->getBackEndRiskCost(
        Eigen::Vector3d(z.center.x() + 10.0 * z.reach, z.center.y(),
                        z.center.z()),
        vel, gp, gv, far_cost);
    check(far_cost == 0.0 && gp.norm() == 0.0,
          "back end: zero cost and zero gradient far outside the envelope");
  }

  check(pm->setRiskZonesRuntime({z, z2}), "two zones re-installed for the "
        "atomic-rejection checks");
  path_manager::RiskZone nan_zone = z;
  nan_zone.peak = std::numeric_limits<double>::quiet_NaN();
  check(!pm->setRiskZonesRuntime({nan_zone}), "a NaN peak is rejected");
  check(std::abs(pm->getFrontEndRisk(probe) - two) < 1e-15,
        "the NaN-peak update also left the field untouched");
}

}  // namespace

int main(int argc, char **argv) {
  if (argc < 2) {
    std::fprintf(stderr,
                 "usage: risk_contract_test <optimizer_params.yaml>\n"
                 "  The yaml is REQUIRED: code defaults leave the back end's\n"
                 "  risk weight at 0, which makes its checks vacuous.\n");
    return 2;
  }
  rclcpp::init(argc, argv);

  testShapeProperties();
  testDerivativeAgreement();
  testPeakValidation();
  testCrossLayerIdentity(argv[1]);

  std::printf("\n==== risk_contract_test: passed=%d failed=%d ====\n",
              g_pass, g_fail);
  rclcpp::shutdown();
  return g_fail == 0 ? 0 : 1;
}
