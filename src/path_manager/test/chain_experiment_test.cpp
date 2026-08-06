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
       with_tailzero = false, with_tailacc = false, with_exclusive = false;
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
  const Eigen::Vector3d start_vel(2.0, 0.0, 0.0);
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

  const bool ok = chain.plan(start_pos, start_vel, start_acc, goal,
                             /*start_vel_synthesized=*/true, mtail)
                      .hasTrajectory();
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

  if (with_route) {
    // Route mode has no baseline solve: both container slots must carry the
    // SAME chained flight (the comparison channel mirrors it), and the
    // whole-trajectory junction sweep above already covered its seams.
    expect(std::abs(bt - ct) < 1e-9 &&
               baseline.getPieceNum() == chained.getPieceNum(),
           "route mode stores the chained flight in both slots");
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
