// [CHAIN] Stage-1 experiment harness: baseline vs N-segment chained plan,
// headless, on a synthetic mission-scale DEM. Drives SegmentChainPlanner
// directly (no FSM, no RViz, no terrain server) with the REAL production
// parameter set loaded from optimizer_params.yaml, then audits the stitched
// trajectory's EVERY piece junction — internal MINCO joints and chain seams
// alike — for P/V/A continuity. A seam that broke shows up here as a junction
// whose left-piece end state disagrees with the right-piece start state;
// MINCO's own joints sit at solver noise (~1e-12), so one threshold covers
// both.
//
// Run inside the dev container from the workspace root:
//   ./install/path_manager/lib/path_manager/chain_experiment_test \
//       [optimizer_params.yaml] [segments] [zone] [segdiff]
// Optional literals (any order after the first two args):
//   zone     — plants one risk zone straddling the first junction's nominal
//              position, exercising the moat+taper nudge: the junction must
//              move off its equal-time station and seams stay exact.
//   segdiff  — stage-2 per-segment requirement overrides: segment 1 prices
//              time 4x (optimization/weight_time=1000), the last segment
//              lowers its speed ceiling (optimization/max_vel=1.8). The
//              [CHAIN-REPORT] mean-speed column shows the differentiation;
//              the seam audit must stay at solver noise regardless.
//   auto     — [AUTO-N] chain/segments=0: N sized from the mission's piece
//              count (target forced to 6 pieces/segment so the 13-piece
//              fixture resolves to N=2, distinct from the argv default 3).
//   autosmall— chain/segments=0 with the production target (70): the fixture
//              sits below ~1.5 targets, so the mission must NOT split and
//              the single-shot plan flies (both slots identical).
// The [CHAIN]/[CHAIN-REPORT] narrative lands in ./logs/runtime/ (LogManager
// is file-only); this binary prints the machine-checkable verdicts to stdout.
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "path_manager/segment_chain_planner.h"

namespace {

int failures = 0;

void expect(bool condition, const std::string &label)
{
  std::cout << (condition ? "[OK]   " : "[FAIL] ") << label << '\n';
  if (!condition) ++failures;
}

// Rolling-hills DEM, mission scale (1 u = 100 m): 360 x 300 u at 2 u/cell,
// crests ~1.8 u (180 m). Heights are authored through the SAME forward map
// terrainToWorld uses ([TERRAIN-FRAME] per-axis mirrors), so the planner
// samples exactly the surface written here regardless of the mirror.
grid_map_msgs::msg::GridMap::SharedPtr makeHillsMap()
{
  auto msg = std::make_shared<grid_map_msgs::msg::GridMap>();
  constexpr double kRes = 2.0;
  constexpr double kLx = 360.0, kLy = 300.0;
  constexpr int kRows = static_cast<int>(kLx / kRes);  // row spans X
  constexpr int kCols = static_cast<int>(kLy / kRes);  // col spans Y
  msg->info.resolution = kRes;
  msg->info.length_x = kLx;
  msg->info.length_y = kLy;
  msg->info.pose.position.x = kLx / 2.0;  // world spans [0, kLx] x [0, kLy]
  msg->info.pose.position.y = kLy / 2.0;
  msg->layers.push_back("elevation");
  msg->data.resize(1);
  auto &layer = msg->data.front();
  layer.layout.dim.resize(2);
  layer.layout.dim[0].label = "column_index";
  layer.layout.dim[0].size = kCols;
  layer.layout.dim[0].stride = kCols * kRows;
  layer.layout.dim[1].label = "row_index";
  layer.layout.dim[1].size = kRows;
  layer.layout.dim[1].stride = kRows;
  layer.data.assign(static_cast<size_t>(kCols) * kRows, 0.0f);
  const double ox = 0.0, oy = 0.0;
  for (int col = 0; col < kCols; ++col) {
    for (int row = 0; row < kRows; ++row) {
      const double wx = ox + kLx - (row + 0.5) * kRes;
      const double wy = oy + kLy - (col + 0.5) * kRes;
      const double h = 1.0 + 0.8 * std::sin(wx / 22.0) * std::cos(wy / 31.0);
      layer.data[static_cast<size_t>(col) * kRows + row] =
          static_cast<float>(std::max(0.0, h));
    }
  }
  return msg;
}

}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  const std::string params =
      argc > 1 ? argv[1]
               : "src/mmp_path_planning/src/path_manager/config/"
                 "optimizer_params.yaml";
  const int segments = argc > 2 ? std::atoi(argv[2]) : 3;

  bool with_zone = false, with_segdiff = false, with_twice = false,
       with_failrestore = false, with_altcap = false, with_terminal = false,
       with_tinyturn = false, with_route = false, with_par = false,
       with_auto = false, with_autosmall = false, with_tailfix = false,
       with_tailzero = false, with_tailacc = false, with_exclusive = false,
       with_phase = false, with_phasefall = false, with_phase2 = false,
       with_phaseweight = false, with_mergetail = false, with_failtail = false,
       with_depedge = false, with_arredge = false, with_departop = false,
       with_initfail = false, with_initok = false, with_synthclamp = false,
       with_unsafedirect = false, with_initaccfail = false,
       with_initnan = false, with_arredge2 = false, with_twophase = false,
       with_pvaprobe = false, with_initceiling = false;
  for (int a = 3; a < argc; ++a) {
    const std::string v(argv[a]);
    if (v == "zone") with_zone = true;
    if (v == "segdiff") with_segdiff = true;
    // terminal: appends the stage-3 prescribed helix descent after the
    // chain. The whole-trajectory junction audit then covers its internal
    // Hermite joints AND the handoff seam under the same 1e-6 threshold.
    if (v == "terminal") with_terminal = true;
    // tinyturn: terminal with turns=0.05 — small enough that no curvature
    // hold fits and the TRIANGLE profile branch runs. The exit-heading
    // budget check below is what pins its turn arithmetic (a factor-of-2
    // error once delivered half the requested heading change, invisibly to
    // every continuity check).
    if (v == "tinyturn") with_tinyturn = true;
    // route / par: the baseline-free contract-authoring mode, sequential
    // and threaded — the audit found the headline feature had zero test
    // coverage. Assertions reuse the whole-trajectory junction sweep plus
    // route-specific storage semantics (both slots carry the chained
    // flight).
    if (v == "route") with_route = true;
    // [PHASE] junction-rule variants (route mode implied): phase = handoffs
    // pinned on the smooth fixture (SUCCESS, no degrade); phasefall =
    // impossible departure arc -> balanced junctions + DEGRADED(
    // PHASE_BOUNDARY_FALLBACK); phase2 = N=2 single shared handoff.
    if (v == "phase") { with_route = true; with_phase = true; }
    if (v == "phasefall") { with_route = true; with_phase = true; with_phasefall = true; }
    if (v == "phase2") { with_route = true; with_phase = true; with_phase2 = true; }
    // phaseweight: arrival profile pins max_vel=1.8 (hard-max, whitelisted)
    // in ROUTE mode — the tail region must actually slow while the middle
    // cruises, proving per-worker configs reach the cost function.
    if (v == "phaseweight") { with_route = true; with_phase = true; with_phaseweight = true; }
    // mergetail: last segment failure injected -> the MERGED span must
    // still deliver the mission's prescribed final state.
    // failtail: ALL workers fail (-1) -> the single-shot FALLBACK must
    // still deliver it (and degrade, not silently succeed).
    if (v == "mergetail") { with_route = true; with_tailfix = true; with_mergetail = true; }
    if (v == "failtail") { with_route = true; with_tailfix = true; with_failtail = true; }
    // [PHASE-DEP] edge-boundary rescues: depedge injects a departure solve
    // failure (one-shot) -> connector-seed retry at the SAME handoff must
    // rescue with phase semantics intact; arredge injects the arrival solve
    // failure -> the handoff moves EARLIER among screened candidates.
    // departop plans with an initial velocity OPPOSITE the route and shape-
    // checks the head region (no worm: bounded total turn, no self-cross).
    if (v == "depedge") { with_route = true; with_phase = true; with_depedge = true; }
    if (v == "arredge") { with_route = true; with_phase = true; with_arredge = true; }
    // [PHASE-N2] arredge2: the arrival segment fails at N=2, where ONE
    // junction serves both handoffs. Stated policy is "no arrival retry" —
    // the plan must go direct with that reason, never silently report a
    // screening failure and never mislabel a merged span.
    if (v == "arredge2") {
      with_route = true; with_phase = true; with_phase2 = true;
      with_arredge2 = true;
    }
    // twophase: two plans in ONE process with DIFFERENT segment counts, to
    // prove no per-plan state (segments_, screened candidates) survives into
    // the next mission — the merge ladder decrements segments_ in flight.
    if (v == "twophase") { with_route = true; with_phase = true; with_twophase = true; }
    if (v == "departop") { with_route = true; with_phase = true; with_departop = true; }
    if (v == "par") { with_route = true; with_par = true; }
    // twice: plan the SAME mission twice in one process. If the scope-guard
    // restore leaks a segment override, the second BASELINE (always planned
    // with mission-wide params) solves a different problem and its duration
    // shifts — bitwise-equal durations are the restore proof.
    if (v == "twice") with_twice = true;
    // failrestore: seg2 gets an unflyable ceiling (0.5 u/s = 50 m/s, far
    // below stall) so a segment run FAILS mid-chain with overrides applied;
    // the chain must fall back to the baseline AND the restore must still
    // happen (verified by the second plan's baseline).
    if (v == "failrestore") with_failrestore = true;
    // altcap: overrides one of the four PathManager-consumed optimization/*
    // values (alt_cap_headroom) on seg2 — end-to-end proof of the re-read
    // fix: the [ALT] z-band log line for seg2's solve shifts by +0.8 u.
    if (v == "altcap") with_altcap = true;
    // auto / autosmall: [AUTO-N] mission-sized segment count — see the
    // usage block above for what each one pins.
    if (v == "auto") with_auto = true;
    if (v == "autosmall") with_autosmall = true;
    // [PHASE] tail-boundary variants: tailfix (prescribed final vel+acc,
    // 1e-8 arrival tolerance — observed 1e-13, the slack is test headroom),
    // tailzero (sub-stall final speed: FAILED by default, DEGRADED with the
    // relaxation opt-in), tailacc (acceleration-only prescription rides the
    // level entry).
    if (v == "tailfix") with_tailfix = true;
    // exclusive: terminal geometry toggled ON while the mission prescribes
    // the final state — the guard must skip the helix, so the prescribed
    // tail (tailfix checks) is still the trajectory's true end.
    if (v == "exclusive") { with_tailfix = true; with_exclusive = true; }
    if (v == "tailzero") with_tailzero = true;
    if (v == "tailacc") with_tailacc = true;
    // [ENVELOPE] contract-1 entry-validation variants. initfail: commanded
    // initial velocity OUTSIDE the validity cone (γ = 32° > 30°) — must
    // FAIL(INITIAL_MODE_UNSUPPORTED) at plan entry with NO front-end or
    // optimizer work (proven by the armed fault injection staying armed).
    // initok: same shape at γ = 7° — proceeds normally through the phase
    // machinery. synthclamp: SYNTHESIZED sub-floor start keeps the
    // [STALL-FLOOR] clamp (no envelope reject — the floor repairs OUR proxy
    // states, never operator inputs). unsafedirect: the phase-mode direct
    // fallback vs the flight fitness gate — passes under the default budget
    // (DEGRADED), FAILS(DIRECT_FALLBACK_UNSAFE) under an impossible one.
    if (v == "initfail") { with_route = true; with_phase = true; with_initfail = true; }
    // initaccfail: lawful velocity (180 m/s level) but a commanded
    // acceleration only ~51 g of lateral load could deliver — the full-PVA
    // judgment (inverse dynamics) must reject what the velocity screen
    // alone would certify.
    if (v == "initaccfail") { with_route = true; with_phase = true; with_initaccfail = true; }
    // initnan: NaN in the commanded acceleration. Finiteness is judged
    // BEFORE the dynamics-model-off early return, so a NaN state is
    // rejected whether or not the model is loaded (fail-closed ordering).
    if (v == "initnan") { with_route = true; with_phase = true; with_initnan = true; }
    // initceiling: 210 m/s level — INSIDE the dynamics model's speed range
    // (max 230) but ABOVE the frame cap (max_vel 2.0 u/s = 200 m/s). The
    // validator must reject on the EFFECTIVE ceiling, or the state passes
    // the entrance and fails in the solve.
    if (v == "initceiling") { with_route = true; with_phase = true; with_initceiling = true; }
    // [CONTRACT-2 EVAL] pvaprobe: INTERFACE-ONLY check that a
    // transition-generator handoff state (the JSBSim experiment's
    // dwell-complete PVA) can cross the boundary into the planner: fixed
    // ENU -> planner axes, SI -> planner units, mission-anchor addition so
    // ABSOLUTE altitude survives (air density), pvaEnvelopeProblem() call,
    // and PlanReason/detail propagation. Deliberately NOT a physics
    // verdict: the example aircraft and the cruise model are different
    // vehicles, so pass or reject are both acceptable outcomes here —
    // the assertions only pin that plan() agrees with the validator.
    if (v == "pvaprobe") { with_route = true; with_phase = true; with_pvaprobe = true; }
    if (v == "initok") { with_route = true; with_phase = true; with_initok = true; }
    if (v == "synthclamp") { with_route = true; with_autosmall = true; with_synthclamp = true; }
    if (v == "unsafedirect") { with_route = true; with_autosmall = true; with_phase = true; with_unsafedirect = true; }
  }
  if (with_tinyturn) with_terminal = true;

  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", params});
  auto node = std::make_shared<rclcpp::Node>("chain_experiment", options);
  node->declare_parameter("drone_id", 0);

  // The params file is the LIVE tuning yaml: whatever chain/* toggles the
  // user is currently experimenting with arrive as parameter overrides and
  // would silently reconfigure every variant (observed once: the yaml's
  // author_from_route + segments=0 flipped baseline-mode variants into
  // route/auto mode and 5 of 9 failed). The harness owns chain config —
  // argv picks the variant — so every toggle the planner reads is forced
  // here: declare if needed, then set, which outranks any file override.
  const auto force = [&node](const std::string &name, auto value) {
    if (!node->has_parameter(name))
      node->declare_parameter(name, rclcpp::ParameterValue(value));
    node->set_parameter(rclcpp::Parameter(name, value));
  };
  force("chain/segments", (with_auto || with_autosmall) ? 0 : segments);
  force("chain/author_from_route", with_route);
  force("chain/parallel", with_par);
  force("chain/terminal/enable", with_terminal);
  force("chain/difficulty_balance", false);
  for (int i = 1; i <= std::max(segments, 8); ++i)
    force("chain/seg" + std::to_string(i) + "/params",
          std::vector<std::string>{});
  if (with_auto) force("chain/auto_pieces_per_segment", 6);
  // Forced BOTH ways: the live yaml ships chain/phase/enable true, and a
  // non-phase variant picking it up would route its direct fallback through
  // the phase-mode fitness gate (observed: synthclamp's clamped-floor flight
  // gated FAILED at 765% thrust peak instead of pinning the clamp doctrine).
  force("chain/phase/enable", with_phase);
  if (with_depedge) force("chain/jitter/fail_segment", 1);
  if (with_arredge) force("chain/jitter/fail_segment", 3);
  // initfail arms all-worker failure injection as a tripwire: entry
  // validation must exit BEFORE the worker stage that consumes (and resets)
  // it, so the parameter still reads -1 after the plan.
  if (with_initfail || with_initaccfail || with_initnan || with_initceiling)
    force("chain/jitter/fail_segment", -1);
  if (with_mergetail) force("chain/jitter/fail_segment", 3);
  if (with_arredge2) force("chain/jitter/fail_segment", 2);  // last of N=2
  if (with_failtail) force("chain/jitter/fail_segment", -1);
  if (with_phaseweight)
    force("chain/phase/arrival/params",
          std::vector<std::string>{"optimization/max_vel=1.8"});
  if (with_phasefall) force("chain/phase/depart_min_arc_u", 280.0);
  if (with_phase2) force("chain/segments", 2);
  if (with_segdiff) {
    force("chain/seg1/params",
          std::vector<std::string>{"optimization/weight_time=1000.0"});
    force("chain/seg" + std::to_string(segments) + "/params",
          std::vector<std::string>{"optimization/max_vel=1.8"});
    std::cout << "segdiff: seg1 weight_time=1000, seg" << segments
              << " max_vel=1.8\n";
  } else if (with_failrestore) {
    force("chain/seg2/params",
          std::vector<std::string>{"optimization/max_vel=0.5"});
    std::cout << "failrestore: seg2 max_vel=0.5 (sub-stall, must fail)\n";
  } else if (with_altcap) {
    force("chain/seg2/params",
          std::vector<std::string>{"optimization/alt_cap_headroom=1.2"});
    std::cout << "altcap: seg2 alt_cap_headroom=1.2 (default 0.4)\n";
  }

  const double helix_turns = with_tinyturn ? 0.05 : 1.0;
  if (with_tinyturn) {
    force("chain/terminal/turns", 0.05);
    // The tiny arc cuts across hills the full turn avoids; a higher exit
    // AGL keeps the whole descent above terrain — the underground gate
    // (audit find) DISCARDS a cutting helix, and this fixture's heading
    // assertions need the helix appended.
    force("chain/terminal/final_agl", 1.2);
  }
  if (with_route)
    std::cout << (with_par ? "route-parallel" : "route-sequential")
              << " mode\n";

  auto pm = std::make_shared<path_manager::PathManager>(node);
  pm->initOptimizer();
  pm->deliverTrajToOptimizer();
  pm->setTerrainData(makeHillsMap());

  // Optional zone on the first junction's nominal station (x=130 for 3
  // segments): reach 5 + taper 30 forces the nudge but leaves the ±40%-span
  // window (±40 u) enough room to clear.
  if (with_zone) {
    path_manager::RiskZone zone;
    zone.center = Eigen::Vector3d(130.0, 150.0, 2.0);
    zone.reach = 5.0;
    zone.peak = 0.8;
    pm->setRiskZonesRuntime({zone});
    std::cout << "risk zone planted at (130, 150), reach 5\n";
  }

  auto log = std::make_unique<swarm_formation::LogManager>(
      "chain_experiment", "./logs/runtime", swarm_formation::LogManager::INFO);
  path_manager::SegmentChainPlanner chain(node, pm, log.get(), segments);

  // 30 km eastbound mission over the hills: start at cruise, goal at 150 m
  // AGL. Start z is absolute (max terrain 1.8 u); goal z rides [GOAL AGL].
  const Eigen::Vector3d start_pos(30.0, 150.0, 3.0);
  Eigen::Vector3d start_vel(2.0, 0.0, 0.0);
  const Eigen::Vector3d start_acc(0.0, 0.0, 0.0);
  const std::vector<Eigen::Vector3d> goal = {{330.0, 150.0, 1.5}};

  if (with_exclusive) {
    force("chain/terminal/enable", true);
    std::cout << "exclusive: terminal enabled + final state prescribed\n";
  }
  ego_planner::TailBoundary mtail;
  if (with_tailfix) {
    mtail.prescribe_vel = true;
    mtail.prescribe_acc = true;
    mtail.vel = Eigen::Vector3d(1.55, 0.41, 0.0);   // |v|=1.60, +15 deg yaw
    mtail.acc = Eigen::Vector3d(0.0, 0.05, 0.0);
    std::cout << "tailfix: final vel (1.55,0.41,0) acc (0,0.05,0)\n";
  } else if (with_tailacc) {
    mtail.prescribe_acc = true;
    mtail.acc = Eigen::Vector3d(0.0, 0.05, 0.0);
    std::cout << "tailacc: final acc (0,0.05,0), velocity free\n";
  }

  if (with_tailzero) {
    // Sub-stall prescribed final speed: FAILED without the opt-in, DEGRADED
    // with it. Dedicated flow — the standard checks assume a normal tail.
    ego_planner::TailBoundary bad;
    bad.prescribe_vel = true;
    bad.vel = Eigen::Vector3d(0.5, 0.0, 0.0);
    const path_manager::PlanResult r1 =
        chain.plan(start_pos, start_vel, start_acc, goal, true, bad);
    expect(!r1.hasTrajectory(),
           "sub-stall final boundary rejected (FAILED, no opt-in)");
    force("planning/allow_final_boundary_relaxation", true);
    const path_manager::PlanResult r2 =
        chain.plan(start_pos, start_vel, start_acc, goal, true, bad);
    expect(r2.hasTrajectory(), "relaxed plan produces a trajectory");
    expect(r2.outcome == path_manager::PlanOutcome::DEGRADED &&
               r2.reason == path_manager::PlanReason::FINAL_BOUNDARY_RELAXED,
           "relaxed plan reports DEGRADED(FINAL_BOUNDARY_RELAXED)");
    rclcpp::shutdown();
    if (failures == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  if (with_initfail) {
    // [0, -160, 100] m/s in units: speed 188.7 m/s IN range, γ = 32° OUT of
    // the ±30° cone — the r5 defect input. Must fail at plan entry: reason
    // set, no trajectory, and the armed fault injection untouched (nothing
    // downstream of validation ever ran).
    const path_manager::PlanResult r =
        chain.plan(start_pos, Eigen::Vector3d(0.0, -1.6, 1.0), start_acc,
                   goal, /*start_vel_synthesized=*/false, {},
                   /*start_vel_commanded=*/true);
    expect(!r.hasTrajectory(),
           "out-of-cone commanded initial state FAILED (no trajectory)");
    expect(r.reason == path_manager::PlanReason::INITIAL_MODE_UNSUPPORTED,
           "reason is INITIAL_MODE_UNSUPPORTED");
    expect(node->get_parameter("chain/jitter/fail_segment").as_int() == -1,
           "fault injection still armed — no front-end/optimizer work ran");
    rclcpp::shutdown();
    if (failures == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  if (with_pvaprobe) {
    // Dwell-complete PVA from the JSBSim experiment, LOCAL ENU
    // displacement / SI (experiments/jsbsim_probe, v1.3.1 @ 3b25f25e,
    // neutral civilian example model, grid search stage [4]). Override
    // with MMP_PVA_CSV = "t,px,py,pz,vx,vy,vz,ax,ay,az" for other states.
    const char *csv = std::getenv("MMP_PVA_CSV");
    double f[10] = {2.280000, -0.060109, -362.933609, 194.702555,
                    -0.053851, -160.150060, 50.897341,
                    -0.022946, 7.116869, -44.595838};
    if (csv && std::sscanf(csv, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                           &f[0], &f[1], &f[2], &f[3], &f[4], &f[5], &f[6],
                           &f[7], &f[8], &f[9]) != 10) {
      std::cout << "[FAIL] MMP_PVA_CSV malformed\n";
      rclcpp::shutdown();
      return 1;
    }
    // Anchor: the ENU p is a DISPLACEMENT from the JSBSim initial point,
    // but pvaEnvelopeProblem uses ABSOLUTE altitude for air density. The
    // anchor's z (15 u = 1500 m) matches the JSBSim initial ASL altitude,
    // so planner_position = anchor + displacement/unit preserves the
    // altitude the state was actually propagated at.
    const double um = 100.0;  // optimization/dynamics_unit_xy_m == _z_m
    const Eigen::Vector3d anchor(30.0, 150.0, 15.0);
    const Eigen::Vector3d p_u = anchor + Eigen::Vector3d(f[1], f[2], f[3]) / um;
    const Eigen::Vector3d v_u = Eigen::Vector3d(f[4], f[5], f[6]) / um;
    const Eigen::Vector3d a_u = Eigen::Vector3d(f[7], f[8], f[9]) / um;
    const std::string prob = pm->pvaEnvelopeProblem(p_u, v_u, a_u);
    std::cout << "pvaprobe: dwell-complete state at t=" << f[0] << " s\n"
              << "  planner p=(" << p_u.x() << ", " << p_u.y() << ", "
              << p_u.z() << ") u  |v|=" << v_u.norm() << " u/s  |a|="
              << a_u.norm() << " u/s^2\n"
              << "  pvaEnvelopeProblem: "
              << (prob.empty() ? "(within region)" : prob) << "\n"
              << "  NOTE: NOT a physics verdict — example aircraft vs "
                 "cruise model.\n";
    // Interface contract: plan() must AGREE with the validator, and on
    // rejection the reason and the validator's own words must reach the
    // caller. (start_vel_synthesized=false, commanded=true: this is an
    // explicit handoff state.)
    const path_manager::PlanResult r =
        chain.plan(p_u, v_u, a_u, goal, false, {}, true);
    if (prob.empty()) {
      expect(r.reason != path_manager::PlanReason::INITIAL_MODE_UNSUPPORTED,
             "validator passed -> plan() does not reject the head");
    } else {
      expect(!r.hasTrajectory() &&
                 r.reason ==
                     path_manager::PlanReason::INITIAL_MODE_UNSUPPORTED,
             "validator rejected -> FAILED(INITIAL_MODE_UNSUPPORTED)");
      expect(r.detail.find(prob) != std::string::npos,
             "validator's reason text reaches PlanResult.detail verbatim");
    }
    rclcpp::shutdown();
    if (failures == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  if (with_twophase) {
    // Plan 1 at N=3, plan 2 at N=5 in the same object. If segments_ or the
    // screened candidate lists leaked, the second plan inherits the first
    // one's N (possibly already decremented by a merge) and its handoffs.
    const path_manager::PlanResult r1 =
        chain.plan(start_pos, start_vel, start_acc, goal, true, {});
    expect(r1.hasTrajectory(), "first plan (N=3) produces a trajectory");
    const int p1 = pm->traj_.local_traj.traj.getPieceNum();
    force("chain/segments", 5);
    const path_manager::PlanResult r2 =
        chain.plan(start_pos, start_vel, start_acc, goal, true, {});
    expect(r2.hasTrajectory(), "second plan (N=5) produces a trajectory");
    const int p2 = pm->traj_.local_traj.traj.getPieceNum();
    std::cout << "twophase: N=3 -> " << p1 << " pieces, N=5 -> " << p2
              << " pieces\n";
    // Same mission, more segments: the piece count must actually change.
    // Equality is the leak signature (the second plan re-used stale N).
    expect(p1 != p2, "segment count change reaches the second plan (no "
                     "per-plan state leak)");
    rclcpp::shutdown();
    if (failures == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  if (with_initceiling) {
    // 2.1 u/s = 210 m/s level: within the model's 230 m/s but above the
    // frame's max_vel 200 m/s. Must reject on the EFFECTIVE ceiling with
    // the frame cap named in the reason.
    const path_manager::PlanResult r = chain.plan(
        start_pos, Eigen::Vector3d(2.1, 0.0, 0.0), start_acc, goal,
        /*start_vel_synthesized=*/false, {}, /*start_vel_commanded=*/true);
    expect(!r.hasTrajectory(),
           "above-frame-cap commanded speed FAILED (no trajectory)");
    expect(r.reason == path_manager::PlanReason::INITIAL_MODE_UNSUPPORTED,
           "reason is INITIAL_MODE_UNSUPPORTED");
    expect(r.detail.find("frame max_vel cap") != std::string::npos,
           "reason names the frame cap, not the model maximum");
    expect(node->get_parameter("chain/jitter/fail_segment").as_int() == -1,
           "fault injection still armed — no front-end/optimizer work ran");
    rclcpp::shutdown();
    if (failures == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  if (with_initnan) {
    const path_manager::PlanResult r = chain.plan(
        start_pos, Eigen::Vector3d(1.8, 0.0, 0.0),
        Eigen::Vector3d(0.0, std::nan(""), 0.0), goal,
        /*start_vel_synthesized=*/false, {}, /*start_vel_commanded=*/true);
    expect(!r.hasTrajectory(), "NaN commanded acceleration FAILED");
    expect(r.reason == path_manager::PlanReason::INITIAL_MODE_UNSUPPORTED,
           "reason is INITIAL_MODE_UNSUPPORTED");
    expect(node->get_parameter("chain/jitter/fail_segment").as_int() == -1,
           "fault injection still armed — no front-end/optimizer work ran");
    rclcpp::shutdown();
    if (failures == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  if (with_initaccfail) {
    // Velocity alone is lawful — 180 m/s level. The commanded acceleration
    // (0, 5, 0) u/s² = 500 m/s² lateral (~51 g) is not: the inverse-dynamics
    // stage of the PVA judgment must reject it before any planning.
    const path_manager::PlanResult r =
        chain.plan(start_pos, Eigen::Vector3d(1.8, 0.0, 0.0),
                   Eigen::Vector3d(0.0, 5.0, 0.0), goal,
                   /*start_vel_synthesized=*/false, {},
                   /*start_vel_commanded=*/true);
    expect(!r.hasTrajectory(),
           "undeliverable commanded acceleration FAILED (no trajectory)");
    expect(r.reason == path_manager::PlanReason::INITIAL_MODE_UNSUPPORTED,
           "reason is INITIAL_MODE_UNSUPPORTED");
    expect(node->get_parameter("chain/jitter/fail_segment").as_int() == -1,
           "fault injection still armed — no front-end/optimizer work ran");
    rclcpp::shutdown();
    if (failures == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  if (with_synthclamp) {
    // Synthesized 50 m/s start (floor ≈ 130 m/s): the [STALL-FLOOR] clamp
    // repairs it and the too-small mission flies single-shot as SUCCESS —
    // proving the envelope gate never fires on non-commanded starts.
    const path_manager::PlanResult r =
        chain.plan(start_pos, Eigen::Vector3d(0.5, 0.0, 0.0), start_acc,
                   goal, /*start_vel_synthesized=*/true, {});
    expect(r.hasTrajectory(),
           "synthesized sub-floor start still plans (clamped, not rejected)");
    expect(r.outcome == path_manager::PlanOutcome::SUCCESS,
           "single-shot on a below-threshold mission stays SUCCESS");
    expect(r.reason != path_manager::PlanReason::INITIAL_MODE_UNSUPPORTED,
           "no INITIAL_MODE_UNSUPPORTED misclassification");
    rclcpp::shutdown();
    if (failures == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  if (with_unsafedirect) {
    // Phase requested on a below-threshold mission -> direct fallback. Under
    // the default budget the SAME flight passes the fitness gate and reports
    // DEGRADED; under an impossible peak budget (1%) it must be REFUSED —
    // FAILED(DIRECT_FALLBACK_UNSAFE), never handed to the FSM.
    const path_manager::PlanResult r1 =
        chain.plan(start_pos, start_vel, start_acc, goal, true, {});
    expect(r1.hasTrajectory(),
           "phase-mode direct fallback flies under the default gate");
    expect(r1.outcome == path_manager::PlanOutcome::DEGRADED,
           "direct fallback reports DEGRADED");
    force("optimization/audit_envelope_peak_max", 0.01);
    const path_manager::PlanResult r2 =
        chain.plan(start_pos, start_vel, start_acc, goal, true, {});
    expect(!r2.hasTrajectory(), "unsafe direct fallback FAILED, not flown");
    expect(r2.reason == path_manager::PlanReason::DIRECT_FALLBACK_UNSAFE,
           "reason is DIRECT_FALLBACK_UNSAFE");
    rclcpp::shutdown();
    if (failures == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  if (with_departop) start_vel = Eigen::Vector3d(-1.55, 0.41, 0.0);
  // initok: the r5 defect DIRECTION at a lawful γ = 7° (speed 161 m/s) —
  // commanded, so it passes the SAME gate that kills initfail, then the
  // standard flow (junction audit included) must hold. departop doubles as
  // the reverse-horizontal case: 160 m/s level opposite the route is IN the
  // envelope and must never be misclassified as INITIAL_MODE_UNSUPPORTED.
  if (with_initok) start_vel = Eigen::Vector3d(0.0, -1.6, 0.2);
  const bool vel_commanded = with_departop || with_initok;
  const path_manager::PlanResult pres =
      chain.plan(start_pos, start_vel, start_acc, goal,
                 /*start_vel_synthesized=*/!vel_commanded, mtail,
                 vel_commanded);
  if (vel_commanded)
    expect(pres.reason != path_manager::PlanReason::INITIAL_MODE_UNSUPPORTED,
           "within-envelope commanded start not misclassified as "
           "INITIAL_MODE_UNSUPPORTED");
  const bool ok = pres.hasTrajectory();
  expect(ok, "chained plan returns success");
  if (!ok) {
    rclcpp::shutdown();
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  const poly_traj::Trajectory &chained = pm->traj_.local_traj.traj;
  const poly_traj::Trajectory &baseline = pm->traj_.global_traj.traj;
  expect(chained.getPieceNum() > 0 && baseline.getPieceNum() > 0,
         "both chained and baseline trajectories are stored");

  // Continuity audit across EVERY junction of the stitched trajectory.
  double max_dp = 0.0, max_dv = 0.0, max_da = 0.0;
  int worst_junc = -1;
  for (int j = 1; j < chained.getPieceNum(); ++j) {
    const auto left = chained.getPiece(j - 1);
    const double dur = left.getDuration();
    const double dp = (chained.getJuncPos(j) - left.getPos(dur)).norm();
    const double dv = (chained.getJuncVel(j) - left.getVel(dur)).norm();
    const double da = (chained.getJuncAcc(j) - left.getAcc(dur)).norm();
    if (std::max({dp, dv, da}) > std::max({max_dp, max_dv, max_da}))
      worst_junc = j;
    max_dp = std::max(max_dp, dp);
    max_dv = std::max(max_dv, dv);
    max_da = std::max(max_da, da);
  }
  std::cout << "junctions audited: " << chained.getPieceNum() - 1
            << " (chain seams + internal MINCO joints)\n"
            << "  max |dP| = " << max_dp << " u\n"
            << "  max |dV| = " << max_dv << " u/s\n"
            << "  max |dA| = " << max_da << " u/s^2\n"
            << "  worst junction index: " << worst_junc << '\n';
  expect(max_dp < 1e-6, "position continuous at every junction (< 1e-6 u)");
  expect(max_dv < 1e-6, "velocity continuous at every junction (< 1e-6 u/s)");
  expect(max_da < 1e-6,
         "acceleration continuous at every junction (< 1e-6 u/s^2)");

  // Sanity: the split must not change the flight materially. (A terminal
  // helix deliberately extends the flight past the goal — gate those.)
  const double bt = baseline.getTotalDuration();
  const double ct = chained.getTotalDuration();
  std::cout << "baseline: " << baseline.getPieceNum() << " pieces, " << bt
            << " s | chained: " << chained.getPieceNum() << " pieces, " << ct
            << " s (" << 100.0 * (ct / bt - 1.0) << "% time)\n";
  expect((chained.getJuncPos(0) - start_pos).norm() < 1e-9,
         "chained trajectory starts at the mission start");
  if (!with_terminal) {
    expect(std::abs(ct / bt - 1.0) < 0.15,
           "chained flight time within 15% of the baseline");
    expect((chained.getJuncPos(chained.getPieceNum()) -
            baseline.getJuncPos(baseline.getPieceNum())).norm() < 1e-6,
           "chained and baseline end at the same resolved goal");
  } else {
    // The prescribed helix must exit level, unaccelerated, at the
    // configured AGL over the terrain under its exit point.
    const int M = chained.getPieceNum();
    const Eigen::Vector3d exit_p = chained.getJuncPos(M);
    const Eigen::Vector3d exit_v = chained.getJuncVel(M);
    const Eigen::Vector3d exit_a = chained.getJuncAcc(M);
    double ground = 0.0;
    pm->terrainElevation(exit_p.x(), exit_p.y(), &ground);
    const double want_agl = with_tinyturn ? 1.2 : 0.15;
    expect(std::abs(exit_p.z() - ground - want_agl) < 0.02,
           "terminal helix exits at the configured AGL");
    expect(std::abs(exit_v.z()) < 1e-9 && exit_a.norm() < 1e-9,
           "terminal helix exits level and unaccelerated");
    expect(ct > bt + (with_tinyturn ? 5.0 : 30.0),
           "terminal helix genuinely extends the flight");

    // Heading budget: exit heading vs the helix ENTRY heading (= the
    // baseline's arrival direction — the handoff contract) must differ by
    // exactly -2*pi*turns for a right helix, wrapped. Full turn closes to
    // 0; turns=0.05 must read -18 deg — the triangle branch once delivered
    // half of it, invisible to every continuity check.
    const Eigen::Vector3d in_v = baseline.getJuncVel(baseline.getPieceNum());
    double dth = std::atan2(exit_v.y(), exit_v.x()) -
                 std::atan2(in_v.y(), in_v.x());
    while (dth > M_PI) dth -= 2.0 * M_PI;
    while (dth < -M_PI) dth += 2.0 * M_PI;
    double want = -2.0 * M_PI * helix_turns;  // right turn
    while (want < -M_PI) want += 2.0 * M_PI;
    std::cout << "helix heading change " << dth * 180.0 / M_PI
              << " deg (want " << want * 180.0 / M_PI << ")\n";
    expect(std::abs(dth - want) < 0.01,
           "helix delivers the requested turn angle");

    if (!with_tinyturn) {
      // Constant-speed regression net: piece durations mismatched to their
      // sampled arcs once put a ±1.6% speed ripple and a spurious
      // tangential acceleration (~37% of centripetal) into the
      // "constant-speed" helix. Sample deep inside the hold phase.
      double v_min = std::numeric_limits<double>::infinity(), v_max = 0.0;
      double tang_max = 0.0, lat_max = 0.0;
      for (double tt = ct - 150.0; tt < ct - 5.0; tt += 0.1) {
        const Eigen::Vector3d vv = chained.getVel(tt);
        const Eigen::Vector3d aa = chained.getAcc(tt);
        const double s = vv.head<2>().norm();
        v_min = std::min(v_min, s);
        v_max = std::max(v_max, s);
        const Eigen::Vector2d vh = vv.head<2>().normalized();
        tang_max = std::max(tang_max,
                            std::abs(aa.head<2>().dot(vh)));
        lat_max = std::max(lat_max, aa.head<2>().norm());
      }
      const double cent = v_min * v_min / 58.0;  // v^2/R at default radius
      std::cout << "helix horizontal speed in [" << v_min << ", " << v_max
                << "] u/s; tangential acc max " << tang_max
                << " (centripetal " << cent << "), lateral acc max "
                << lat_max << "\n";
      expect((v_max - v_min) / std::max(1e-9, v_min) < 0.005,
             "helix speed ripple below 0.5% (constant-speed phase)");
      expect(tang_max < 0.02 * cent,
             "no spurious tangential acceleration (< 2% of centripetal)");
      expect(lat_max < 1.05 * cent + 0.02,
             "lateral acceleration bounded by v^2/R (turn radius honored)");
    }
  }

  if (with_route && !with_failtail && !with_arredge2) {
    // (failtail and arredge2 EXPECT the single-shot fallback, where the
    // slots follow single-plan semantics — this assert is for chain
    // successes.)
    // Route mode has no baseline solve: both container slots must carry the
    // SAME chained flight (the comparison channel mirrors it), and the
    // whole-trajectory junction sweep above already covered its seams.
    expect(std::abs(bt - ct) < 1e-9 &&
               baseline.getPieceNum() == chained.getPieceNum(),
           "route mode stores the chained flight in both slots");
  }
  if (with_failtail) {
    expect(pres.outcome == path_manager::PlanOutcome::DEGRADED,
           "all-fail fallback reports DEGRADED");
  }
  if (with_phaseweight) {
    const double T = chained.getTotalDuration();
    double v_end = 0.0, n_end = 0.0, v_mid = 0.0, n_mid = 0.0;
    for (double tt = std::max(0.0, T - 6.0); tt < T - 1.0; tt += 0.2) {
      v_end += chained.getVel(tt).norm();
      n_end += 1.0;
    }
    for (double tt = T * 0.4; tt < T * 0.6; tt += 0.2) {
      v_mid += chained.getVel(tt).norm();
      n_mid += 1.0;
    }
    v_end /= std::max(1.0, n_end);
    v_mid /= std::max(1.0, n_mid);
    std::cout << "phaseweight: mid mean " << v_mid << " u/s, tail mean "
              << v_end << " u/s\n";
    expect(v_end < 1.9, "arrival profile slows the tail region (< 1.9)");
    expect(v_mid > 1.95, "cruise region keeps mission speed (> 1.95)");
  }
  if (with_phase && !with_phasefall && !with_departop && !with_initok &&
      !with_arredge2) {
    expect(pres.outcome == path_manager::PlanOutcome::SUCCESS,
           "phase handoffs pinned — SUCCESS, no degrade");
  }
  if (with_initok) {
    // The commanded start disagrees with the route heading, so the plan may
    // legitimately land on the connector rescue or the (gated) direct
    // fallback — the regression pins "proceeds", not the exact outcome.
    expect(pres.outcome != path_manager::PlanOutcome::FAILED,
           "lawful commanded start (γ = 7°): plan exists");
  }
  if (with_departop) {
    expect(pres.outcome != path_manager::PlanOutcome::FAILED,
           "opposite initial velocity: plan exists (SUCCESS or explicit "
           "DEGRADED direct)");
    // Worm detector on the head region: bounded total turn, no xy self-
    // intersection, bounded arc/chord (the 548 u worm fails all three).
    const double t_end = std::min(60.0, chained.getTotalDuration());
    std::vector<Eigen::Vector2d> xy;
    double turn_sum = 0.0, arc = 0.0;
    Eigen::Vector2d prev_dir(0, 0);
    for (double tt = 0.0; tt <= t_end; tt += 0.5) {
      const Eigen::Vector3d pp = chained.getPos(tt);
      if (!xy.empty()) {
        Eigen::Vector2d d = pp.head<2>() - Eigen::Vector2d(xy.back());
        arc += d.norm();
        if (d.norm() > 1e-6) {
          d.normalize();
          if (prev_dir.norm() > 0.5)
            turn_sum += std::abs(std::atan2(
                prev_dir.x() * d.y() - prev_dir.y() * d.x(),
                prev_dir.dot(d)));
          prev_dir = d;
        }
      }
      xy.emplace_back(pp.x(), pp.y());
    }
    bool self_cross = false;
    for (size_t i = 1; i + 2 < xy.size() && !self_cross; ++i)
      for (size_t j = i + 2; j + 1 < xy.size(); ++j) {
        const auto &a1 = xy[i - 1]; const auto &a2 = xy[i];
        const auto &b1 = xy[j]; const auto &b2 = xy[j + 1];
        const auto cross2 = [](const Eigen::Vector2d &u,
                               const Eigen::Vector2d &v) {
          return u.x() * v.y() - u.y() * v.x();
        };
        const double d1 = cross2(a2 - a1, b1 - a1);
        const double d2 = cross2(a2 - a1, b2 - a1);
        const double d3 = cross2(b2 - b1, a1 - b1);
        const double d4 = cross2(b2 - b1, a2 - b1);
        if (((d1 > 0) != (d2 > 0)) && ((d3 > 0) != (d4 > 0))) {
          self_cross = true;
          break;
        }
      }
    const double chord =
        (xy.back() - xy.front()).norm() > 1e-6
            ? (xy.back() - xy.front()).norm() : 1e-6;
    std::cout << "departop head: turn " << turn_sum * 180.0 / M_PI
              << " deg, arc/chord " << arc / chord
              << (self_cross ? ", SELF-CROSS" : ", no cross") << "\n";
    expect(turn_sum < 4.2, "head total turn bounded (< ~240 deg)");
    expect(!self_cross, "head region has no self-intersection");
    expect(arc / chord < 3.5, "head arc/chord bounded (no worm loop)");
  }
  if (with_depedge) {
    expect(pres.outcome == path_manager::PlanOutcome::SUCCESS,
           "departure solve failure rescued by connector at the SAME "
           "handoff (phase intact)");
  }
  if (with_arredge) {
    expect(pres.outcome == path_manager::PlanOutcome::SUCCESS,
           "arrival solve failure rescued by moving the handoff earlier");
  }
  if (with_phasefall) {
    expect(pres.outcome == path_manager::PlanOutcome::DEGRADED &&
               pres.reason ==
                   path_manager::PlanReason::PHASE_BOUNDARY_FALLBACK,
           "impossible handoff arc -> DEGRADED(PHASE_BOUNDARY_FALLBACK)");
  }
  if (with_arredge2) {
    // [PHASE-N2] The shared handoff admits no arrival retry: the plan goes
    // DIRECT and says so. It must NOT come back SUCCESS (that would mean a
    // merged span wearing phase labels). The single-shot fallback owns the
    // stronger reason by the priority ladder — PHASE_BOUNDARY_FALLBACK
    // rides along in `detail`, which is where the policy has to be legible.
    expect(pres.outcome == path_manager::PlanOutcome::DEGRADED &&
               pres.reason == path_manager::PlanReason::SINGLE_PLAN_FALLBACK,
           "N=2 arrival failure -> DEGRADED(SINGLE_PLAN_FALLBACK), direct");
    expect(pres.detail.find("N=2") != std::string::npos,
           "reason names the N=2 shared-handoff policy, not a screening miss");
  }
  if (with_tailfix) {
    const Eigen::Vector3d tv = chained.getJuncVel(chained.getPieceNum());
    const Eigen::Vector3d ta = chained.getJuncAcc(chained.getPieceNum());
    std::cout << "tailfix arrival: |dv|=" << (tv - mtail.vel).norm()
              << " |da|=" << (ta - mtail.acc).norm() << "\n";
    expect((tv - mtail.vel).norm() < 1e-8,
           "prescribed final velocity reached (1e-8)");
    expect((ta - mtail.acc).norm() < 1e-8,
           "prescribed final acceleration reached (1e-8)");
  }
  if (with_tailacc) {
    const Eigen::Vector3d tv = chained.getJuncVel(chained.getPieceNum());
    const Eigen::Vector3d ta = chained.getJuncAcc(chained.getPieceNum());
    expect((ta - mtail.acc).norm() < 1e-8,
           "acc-only prescription reached (1e-8)");
    expect(tv.norm() > 1.5, "velocity stays a free level entry near cruise");
  }
  if (with_auto) {
    // [AUTO-N] 13 baseline pieces / target 6 -> round to 2, which differs
    // from the argv/ctor default 3 — segments() proves resolveAutoSegments
    // actually sized the chain (a silent fall-through would leave 3).
    expect(chain.segments() == 2,
           "auto-N resolved 2 segments (13 pieces / target 6)");
  }
  if (with_autosmall) {
    // [AUTO-N] production target 70: the fixture is far below ~1.5 targets,
    // so the mission must not split — the single-shot plan fills both slots.
    expect(std::abs(bt - ct) < 1e-9 &&
               baseline.getPieceNum() == chained.getPieceNum(),
           "auto-N below split threshold: single-shot plan flown");
  }
  if (with_failrestore) {
    // The sub-stall segment must have failed and the chain degraded to the
    // baseline (identical trajectory in the local slot).
    expect(std::abs(ct - bt) < 1e-9 &&
               chained.getPieceNum() == baseline.getPieceNum(),
           "unflyable segment fell back to the baseline");
  }
  if (with_twice || with_failrestore) {
    // Restore proof: a second plan in the SAME process. Its baseline always
    // runs with mission-wide parameters — if the scope guard leaked any
    // segment override, the second baseline solves a different problem and
    // its duration/piece count shifts.
    const double base1 = bt;
    const int base1_pieces = baseline.getPieceNum();
    const bool ok2 = chain.plan(start_pos, start_vel, start_acc, goal,
                                /*start_vel_synthesized=*/true)
                         .hasTrajectory();
    expect(ok2, "second plan in the same process succeeds");
    const poly_traj::Trajectory &base2 = pm->traj_.global_traj.traj;
    expect(std::abs(base2.getTotalDuration() - base1) < 1e-9 &&
               base2.getPieceNum() == base1_pieces,
           "second baseline identical — mission-wide parameters were "
           "restored after the first chain");
  }

  rclcpp::shutdown();
  std::cout << (failures == 0 ? "PASS" : "FAIL") << ": " << failures
            << " failed check(s)\n";
  return failures == 0 ? 0 : 1;
}
