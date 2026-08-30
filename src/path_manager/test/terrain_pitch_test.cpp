// The repairer and the judge must walk the route with the SAME stride.
//
// They did not. The terrain-lift sweep sampled at half the DEM cell — 0.50 u,
// 50 m on a 100 m corridor — while the final terrain gate sampled at half the
// finest voxel, 0.05 u. Ten times coarser. So the sweep walked with big
// strides, found nothing wrong at its own footsteps, and reported the route
// clean; the gate then walked the same route with small steps and found a
// point 30 cm short of the 60 m clearance, sitting between two of the sweep's
// samples. In fm2 there is no A* fallback, so that refusal ends the mission.
//
// Measured on r19_land_water_alternation, 2026-08-20:
//   [FM2-FINAL-CLEAR] agl=0.597 required=0.600 deficit=0.003
//                     lifted=4 sweep_clean=true
//
// The fixture below is that failure in miniature: a rise narrow enough to fall
// between 50 m samples and wide enough to be caught at 5 m. Nothing here
// asserts the exponent of any cost or the shape of any route — only that the
// two sides agree, and that the repair is what the judge is judging.

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>

#include "path_planner/dyn_a_star.h"

using path_planner::search::PathSearcher;

namespace {

int g_pass = 0, g_fail = 0;

void check(bool ok, const std::string &what) {
  if (ok) { ++g_pass; std::printf("  [OK]   %s\n", what.c_str()); }
  else    { ++g_fail; std::printf("  [FAIL] %s\n", what.c_str()); }
}

// Frame units: 1 u = 100 m. The corridor this reproduces is 100 m per cell,
// so the DEM-cell floor lands at 0.50 u and the voxel pitch at 0.05 u.
constexpr double kCellU = 1.0;      // 100 m DEM cell
constexpr double kVoxelZ = 0.1;     // 10 m z voxel, as the corridor SDF builds
constexpr double kMargin = 0.60;    // 60 m required clearance

// A rise centred between two 50 m samples. A coarse walk lands on multiples of
// 0.5 u, so a bump centred at 0.25 u with a half-width of 0.12 u (12 m) falls
// entirely between its footsteps. Smooth rather than a step, because a real
// DEM is bilinear and a cliff would make the chord sag pathological for
// reasons that have nothing to do with the stride.
double narrowRise(double x, double /*y*/) {
  const double period = 0.5;
  const double phase = std::fmod(x, period);
  const double d = std::abs(phase - 0.25) / 0.12;
  return (d < 1.0) ? 1.0 * (1.0 - d * d) : 0.0;   // up to 100 m of ground
}

PathSearcher makeSearcher() {
  PathSearcher a;
  a.setSDF(nullptr, Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(50, 50, 50),
           kCellU, kVoxelZ);
  a.setObstacleMargin(kMargin);
  a.setTerrainHeightmap(
      [](double x, double y) { return static_cast<float>(narrowRise(x, y)); },
      kCellU);
  return a;
}

// A straight run at a height that clears the flat ground but NOT the rise:
// ground is 0 or 1.0 u, the route sits at 1.0 u, so over flat ground the
// clearance is 1.0 (fine) and over a rise it is 0.0 (0.60 short).
std::vector<Eigen::Vector3d> flatRoute() {
  return {Eigen::Vector3d(0.05, 5.0, 1.0), Eigen::Vector3d(9.95, 5.0, 1.0)};
}

void testPitchesAgree() {
  std::printf("\n[1] the repairer and the judge use one stride\n");
  PathSearcher a = makeSearcher();

  const double shared = a.terrainCheckPitch();
  std::printf("         terrainCheckPitch = %.3f u (%.0f m)\n",
              shared, shared * 100.0);
  check(shared <= 0.05 + 1e-12,
        "the shared stride is at least as fine as the voxel pitch (0.05 u)");
  check(shared < 0.5,
        "and finer than the DEM-cell floor the sweep used to walk at (0.50 u)");

  // Drive both sides and compare what each ACTUALLY used, not what a function
  // returns — a function trivially equals itself.
  auto pts = flatRoute();
  Eigen::Vector3d hit;
  a.polylineClear(pts, &hit, 0.0);
  const double gate = a.lastGatePitch();
  std::printf("         gate used %.3f u\n", gate);
  check(std::abs(gate - shared) < 1e-12,
        "the final gate walked at the shared stride");
}

void testCoarseStrideStepsOverTheRise() {
  std::printf("\n[2] the fixture is the real failure in miniature\n");
  PathSearcher a = makeSearcher();
  auto pts = flatRoute();

  // Walk the route by hand at each stride and count how often the rise is seen.
  auto worstAt = [&](double pitch) {
    const Eigen::Vector3d p = pts[0], q = pts[1];
    const double len = (q - p).norm();
    const int n = std::max(1, static_cast<int>(std::ceil(len / pitch)));
    double worst = 0.0;
    for (int k = 0; k <= n; ++k) {
      const Eigen::Vector3d x = p + (q - p) * (double(k) / n);
      const double need = narrowRise(x.x(), x.y()) + kMargin;
      worst = std::max(worst, need - x.z());
    }
    return worst;
  };
  const double coarse = worstAt(0.5);
  const double fine = worstAt(a.terrainCheckPitch());
  std::printf("         worst shortfall: 50 m stride %.3f u, shared stride "
              "%.3f u\n", coarse, fine);
  check(coarse <= 0.0,
        "a 50 m stride steps clean over the rise and reports no shortfall");
  check(fine > 0.0,
        "the shared stride sees it — this is the gap that refused r19");
}

void testGateRefusesTheUnrepairedRoute() {
  std::printf("\n[3] before repair the route is short of clearance\n");
  PathSearcher a = makeSearcher();
  auto pts = flatRoute();
  Eigen::Vector3d hit(0, 0, 0);
  const bool clear = a.polylineClear(pts, &hit, 0.0);
  check(!clear, "the gate refuses a route that grazes the rise");
  if (!clear) {
    const double h = narrowRise(hit.x(), hit.y());
    std::printf("         refused at (%.3f, %.3f, %.3f), ground %.2f, "
                "clearance %.3f u\n", hit.x(), hit.y(), hit.z(), h,
                hit.z() - h);
    check(hit.z() - h < kMargin,
          "and the refusal point really is inside the margin");
  }
}

void testLiftedRoutePassesTheGate() {
  std::printf("\n[4] what the production sweep repairs is what the gate "
              "judges\n");
  PathSearcher a = makeSearcher();
  Eigen::Vector3d hit;

  // THE PRODUCTION SWEEP, not a copy of it. A copy cannot notice the sweep's
  // stride drifting away from the gate's, which is the entire defect: before
  // this fix the sweep walked at 0.50 u and the gate at 0.05 u, so the sweep
  // reported a route clean that the gate then refused 30 cm short.
  auto pts = flatRoute();
  const int lifts = a.sweepTerrainClearance(pts);
  const bool ok = a.polylineClear(pts, &hit, 0.0);
  std::printf("         sweep %.3f u / gate %.3f u: %d lift(s), %zu points, "
              "gate says %s\n",
              a.lastSweepPitch(), a.lastGatePitch(), lifts, pts.size(),
              ok ? "clear" : "REFUSED");

  check(a.lastSweepPitch() > 0.0, "the sweep recorded the stride it used");
  check(std::abs(a.lastSweepPitch() - a.lastGatePitch()) < 1e-12,
        "the repairer and the judge walked the SAME stride");
  check(lifts > 0, "the sweep found the rises and repaired them");
  check(ok, "and the gate accepts what the sweep produced");
  check(lifts < 512, "the repair converges well inside the 512 lift cap");

  // The failure this replaces: an unrepaired route is refused, so the sweep
  // above is doing real work rather than being handed something already clear.
  auto raw = flatRoute();
  check(!a.polylineClear(raw, &hit, 0.0),
        "the same route without the sweep is refused");
}

void testObstaclesAreStillRefusedImmediately() {
  std::printf("\n[5] the obstacle answer is unchanged\n");
  // No SDF at all: obstacleBlocked returns false, so the only thing that can
  // refuse is terrain. A flat route well above the rise must pass — proving
  // the terrain change did not make the gate refuse everything.
  PathSearcher a = makeSearcher();
  std::vector<Eigen::Vector3d> high = {Eigen::Vector3d(0.05, 5.0, 3.0),
                                       Eigen::Vector3d(9.95, 5.0, 3.0)};
  Eigen::Vector3d hit;
  check(a.polylineClear(high, &hit, 0.0),
        "a route with room to spare is not refused");

  // And the start-relief exemption still exempts only the start.
  auto pts = flatRoute();
  check(!a.polylineClear(pts, &hit, 0.5),
        "a short relief arc does not excuse a shortfall further along");
  check(a.polylineClear(pts, &hit, 100.0),
        "a relief arc covering the whole route does excuse it");
}

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  testPitchesAgree();
  testCoarseStrideStepsOverTheRise();
  testGateRefusesTheUnrepairedRoute();
  testLiftedRoutePassesTheGate();
  testObstaclesAreStillRefusedImmediately();
  std::printf("\n==== terrain_pitch_test: passed=%d failed=%d ====\n",
              g_pass, g_fail);
  rclcpp::shutdown();
  return g_fail == 0 ? 0 : 1;
}
