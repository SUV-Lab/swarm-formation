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
#include "path_manager/transition_phase.h"
#include "path_manager/waypoint_eval.h"

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

#include <csignal>
#include <execinfo.h>
static void segvHandler(int sig)
{
  void *frames[48];
  const int n = backtrace(frames, 48);
  fprintf(stderr, "=== signal %d backtrace (%d frames) ===\n", sig, n);
  backtrace_symbols_fd(frames, n, 2);
  _exit(139);
}

int main(int argc, char **argv)
{
  signal(SIGSEGV, segvHandler);
  signal(SIGABRT, segvHandler);
  rclcpp::init(argc, argv);

  const std::string params =
      argc > 1 ? argv[1]
               : "src/mmp_path_planning/src/path_manager/config/"
                 "optimizer_params.yaml";
  int segments = argc > 2 ? std::atoi(argv[2]) : 3;

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
       with_pvaprobe = false, with_initceiling = false,
       with_handoffcap = false, with_overroutebad = false,
       with_transfallback = false, with_overrouteretry = false,
       with_badspans = false, with_zonesnapshot = false,
       with_zonewall = false, with_zonepass0 = false,
       with_zonemultileg = false, with_transition = false,
       with_transitionauto = false, with_s8bounds = false,
       with_waypoints = false;
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
    // handoffcap: the EXPLICIT handoff ceiling decouples the entry contract
    // from the planning cap — 220 admits 210 m/s (which the default cap
    // refuses), 190 refuses 195 m/s, and the reason names the explicit cap.
    if (v == "handoffcap") { with_route = true; with_handoffcap = true; }
    // [S13] overroutebad: planOverRoute's fail-closed input contract — a
    // truncated cap, a displaced head, and a NaN vertex must each FAIL,
    // never silently degrade (sliceCommittedRoute would drop the cap).
    if (v == "overroutebad") { with_route = true; with_overroutebad = true; }
    // [S13] overrouteretry: the coordinator's retry pattern — planOverRoute
    // twice on the SAME committed route. The second call must see clean
    // plan-scoped state (phase blackboard reset, segments_ restored after
    // the first call's possible merge), so both produce the same chain.
    if (v == "overrouteretry") { with_route = true; with_phase = true; with_overrouteretry = true; }
    // [S13] badspans: the evaluator's span input contract and the
    // stitched-verdict fail-closed path — malformed spans must yield
    // UNEVALUATED, and stitchedVerdictResult must turn UNEVALUATED into
    // FAILED with the stored trajectory invalidated.
    if (v == "badspans") { with_route = true; with_badspans = true; }
    // [S13] zonesnapshot: the per-zone policy snapshot the transition
    // generator consumes — dispositions from the searcher's real state,
    // the shared contact predicate, and generation-staleness on a zone
    // list change.
    if (v == "zonesnapshot") { with_route = true; with_zonesnapshot = true; }
    // zonewall: pass-2/3 dispositions on an unavoidable full-corridor wall.
    if (v == "zonewall") { with_route = true; with_zonewall = true; }
    // zonepass0: zones + 3-pass policy OFF -> INVALID snapshot fail-closed.
    if (v == "zonepass0") { with_route = true; with_zonepass0 = true; }
    if (v == "zonemultileg") { with_route = true; with_zonemultileg = true; }
    if (v == "transition") {
      with_route = true; with_phase = true; with_transition = true;
    }
    if (v == "s8bounds") { with_route = true; with_s8bounds = true; }
    if (v == "waypoints") {
      with_route = true; with_phase = true; with_waypoints = true;
    }
    if (v == "transitionauto") {
      with_route = true; with_phase = true; with_transition = true;
      with_transitionauto = true;
    }
    // [S13] transfallback: with a transition active, the single-shot
    // fallback is forbidden — the below-threshold mission that normally
    // degrades to a direct plan must FAIL instead.
    if (v == "transfallback") { with_route = true; with_phase = true; with_transfallback = true; }
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
  // overrouteretry needs a mergeable interior junction: at N=3 with phase
  // on, the middle segment's BOTH junctions are protected (departure +
  // arrival), so the ladder exhausts into the single-shot fallback — a
  // false positive for "merge happened" (review find). N=4 leaves an
  // interior cruise junction the generic merge may drop.
  if (with_overrouteretry) segments = std::max(segments, 4);

  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", params});
  // manager/* values are read ONCE in the PathManager constructor, and
  // force() before construction collides with its declare_parameter — so
  // variant-specific manager overrides ride NodeOptions instead (the
  // later declare then returns the override).
  {
    std::vector<rclcpp::Parameter> ovr;
    // zonepass0: 3-pass policy off -> pass stays 0 with zones present.
    if (with_zonepass0)
      ovr.emplace_back("manager/zone_avoid_lexicographic", false);
    // zonewall: block the OVER-THE-TOP escape (default vertical ratio
    // 0.35 leaves a ~12 u ceiling the front end can climb past) so the
    // wall is genuinely unavoidable and the soft passes must run.
    if (with_transition || with_waypoints)
      ovr.emplace_back("transition/enable", true);
    if (with_zonewall) {
      ovr.emplace_back("manager/risk_vertical_ratio", 3.0);
      // ... and the terrain-shadow escape: with LOS masking on, ridge
      // shadows carve INVISIBLE corridors through the wall and pass 1
      // legitimately threads them. Whole-ellipsoid volumes make the wall
      // airtight.
      ovr.emplace_back("manager/risk_terrain_mask_enable", false);
    }
    if (!ovr.empty()) options.parameter_overrides(ovr);
  }
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
  if (with_transitionauto) {
    force("chain/segments", 0);  // auto-N
    // resolveAutoSegments floors the target at 5, and the straight
    // corridor's sub-route thins to ~10 pieces — the HONEST auto result
    // here is N=2. That is exactly what this variant pins: N=2 applied
    // means the option reached the transition path (the pre-fix code
    // kept the constructor's fixed 3). N>=4 sizing is pinned by the r5
    // live smoke's 276-piece route.
    force("chain/auto_pieces_per_segment", 2);
  }
  // Forced BOTH ways: the live yaml ships chain/phase/enable true, and a
  // non-phase variant picking it up would route its direct fallback through
  // the phase-mode fitness gate (observed: synthclamp's clamped-floor flight
  // gated FAILED at 765% thrust peak instead of pinning the clamp doctrine).
  force("chain/phase/enable", with_phase);
  // zonepass0 needs the lexicographic policy OFF before PathManager reads
  // it at construction.

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
    const path_manager::PlanResult r = chain.plan(
        p_u, v_u, a_u, goal, false, {}, true, /*start_acc_commanded=*/true);
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

  if (with_waypoints) {
    // [WPE] The measurement that matters: waypoints extracted from a REAL
    // planned flight — transition + departure + cruise + arrival — flown
    // by the benchmark follower against the REAL terrain and zone hooks.
    // The synthetic harness pins the machinery; this one asks whether the
    // launch-transition flight survives being encoded as waypoints.
    namespace we = path_manager::waypoint_eval;
    // Zones must actually EXIST or "zero hard-zone contacts" is a check on
    // an empty list (review find: the first table reported zoneH=0 with no
    // zones installed). Two avoidable zones flanking the corridor: the
    // committed route goes around them, and a flown path that cuts corners
    // would touch them.
    {
      std::vector<path_manager::RiskZone> zs;
      path_manager::RiskZone z;
      // Placed DOWNSTREAM of the transition's entry region (entry
      // candidates sit near arc 50-115 u): zones that bend the route
      // there rotate the entry tangents past what the v1 primitive
      // family can capture, and the mission fails for a reason that has
      // nothing to do with waypoints.
      z.center = Eigen::Vector3d(225.0, 139.0, 2.0);
      z.reach = 11.0;
      z.peak = 0.7;
      zs.push_back(z);
      z.center = Eigen::Vector3d(285.0, 161.0, 2.0);
      z.reach = 11.0;
      z.peak = 0.7;
      zs.push_back(z);
      pm->setRiskZonesRuntime(zs);
    }
    const Eigen::Vector3d v32(1.6, 0.0, 1.0);   // 32 deg, out of cruise cone
    const path_manager::PlanResult r =
        chain.plan(start_pos, v32, start_acc, goal,
                   /*start_vel_synthesized=*/false, {},
                   /*start_vel_commanded=*/true);
    expect(r.hasTrajectory(), "transition mission planned");
    if (!r.hasTrajectory()) {
      std::cout << "FAIL: " << failures << " failed check(s)\n";
      return 1;
    }
    const poly_traj::Trajectory &flight = pm->traj_.local_traj.traj;
    const auto &spans = chain.lastPhaseSpans();
    double um = 100.0, uz = 100.0;
    node->get_parameter("optimization/dynamics_unit_xy_m", um);
    node->get_parameter("optimization/dynamics_unit_z_m", uz);
    we::FrameScale fscale;
    fscale.unit_xy_m = um;
    fscale.unit_z_m = uz;
    const we::SourcePath src = we::buildSourcePath(flight, fscale);
    expect(!src.empty(), "source path built from the planned flight");

    const auto *dynp = pm->dynamicsParams();
    expect(dynp != nullptr, "assumption parameter set available");
    if (!dynp || src.empty()) {
      std::cout << "FAIL: " << failures << " failed check(s)\n";
      return 1;
    }
    we::FollowerParams fprm;
    fprm.dyn = *dynp;

    // REAL hooks: terrain from the DEM, zones from the committed policy
    // snapshot. A lookup that fails voids the evaluation rather than
    // assuming sea level.
    const auto snap = pm->zonePolicySnapshot();
    expect(snap.valid && snap.zones.size() >= 2,
           "zone policy snapshot is valid and NON-EMPTY (a zero-contact "
           "result on an empty list would prove nothing)");
    we::SafetyHooks hooks;
    hooks.min_agl_m = pm->minGoalAgl() * uz;
    hooks.terrain_z = [&](double x_m, double y_m, double *e) {
      double elev_u = 0.0;
      if (!pm->terrainElevation(x_m / um, y_m / um, &elev_u)) return false;
      if (e) *e = elev_u * uz;
      return true;
    };
    hooks.zone_probe = [&](const Eigen::Vector3d &p_m) {
      const Eigen::Vector3d p_u(p_m.x() / um, p_m.y() / um, p_m.z() / uz);
      for (size_t i = 0; i < snap.zones.size(); ++i) {
        switch (pm->zoneContact(snap, i, p_u)) {
          case path_manager::PathManager::ZoneContactResult::CLEAR: break;
          case path_manager::PathManager::ZoneContactResult::CONTACT:
            if (snap.zones[i].disposition ==
                path_manager::PathManager::ZoneDisposition::HARD_AVOID)
              return path_manager::transition_phase::ZoneProbe::CONTACT_HARD;
            break;
          default:
            return path_manager::transition_phase::ZoneProbe::
                STALE_OR_INVALID;
        }
      }
      return path_manager::transition_phase::ZoneProbe::CLEAR;
    };

    we::FollowerStart fstart;
    fstart.pos_m = src.pos_m.front();
    fstart.vel_mps = src.vel_mps.front();
    we::EvalParams ep;
    ep.terminal_pos_gate_m = 1.5 * we::derivedAcceptRadius(fprm);
    // A deviation tolerance so PASS means the path was REPRODUCED, not
    // merely that the flight ended near the last waypoint with a
    // plausible length (review find). No external requirement exists yet,
    // so this is an EXPERIMENT PARAMETER at the follower's own capture
    // scale — it gets replaced by a real corridor width when one is
    // specified, the same way the section-8 jerk cap waits for the
    // execution layer.
    ep.max_xtrack_gate_m = we::derivedAcceptRadius(fprm);

    // Continuous-reference baseline. Reported TWICE and labeled, because
    // the lead time dominates both followers: matched-lead is the only
    // apples-to-apples comparison against the waypoint rows (which use the
    // default lead), while best-over-lead says what the law can do when
    // its own aiming is tuned. Quoting the tuned number against fixed-lead
    // rows was the review's finding.
    double base_matched = -1.0, base_best = 1e18, best_lead = 0.0;
    {
      const auto rt = we::flyReferenceTrack(src, fstart, fprm);
      const auto mt = we::evaluateReproduction(src, rt, *dynp, hooks, ep);
      if (mt.measured) base_matched = mt.max_xtrack_m;
    }
    for (double lead : {0.5, 1.0, 2.0, 4.0}) {
      auto p2 = fprm;
      p2.lead_time_s = lead;
      const auto rt = we::flyReferenceTrack(src, fstart, p2);
      const auto mt = we::evaluateReproduction(src, rt, *dynp, hooks, ep);
      if (mt.measured && mt.max_xtrack_m < base_best) {
        base_best = mt.max_xtrack_m;
        best_lead = lead;
      }
    }
    const bool floor_ok = base_best < 1e17;
    std::printf("[WPE] %s\n", we::scopeLabel());
    std::printf("[WPE] source: production planner output on SYNTHETIC "
                "harness terrain (makeHillsMap) — not a full-map mission\n");
    std::printf("[WPE] planned flight: %.1f s, %d pieces, %.0f m, %zu "
                "phase span(s)\n",
                flight.getTotalDuration(), flight.getPieceNum(),
                src.total_len_m, spans.size());
    for (const auto &sp : spans)
      std::printf("[WPE]   span %-10s t_end %.1f s\n", sp.name.c_str(),
                  sp.t_end);
    if (floor_ok)
      std::printf("[WPE] continuous-reference baseline: %.1f m at the SAME "
                  "lead as the rows (%.1f s), %.1f m at its own best lead "
                  "(%.1f s). A BASELINE, not a bound — a waypoint list can "
                  "beat it.\n",
                  base_matched, fprm.lead_time_s, base_best, best_lead);
    else
      std::printf("[WPE] continuous-reference baseline: UNMEASURED\n");

    std::printf("\nstrategy    N   maxXT[m]  rmsXT[m]  lenR   endErr[m] "
                "minAGL[m] zoneH  verdict\n");
    const auto row = [&](const char *name, const std::vector<we::Waypoint> &w) {
      const auto ro = we::flyWaypoints3Dof(w, fstart, fprm);
      const auto m = we::evaluateReproduction(src, ro, *dynp, hooks, ep);
      if (!m.measured) {
        std::printf("%-9s %3zu  n/a(%s)\n", name, w.size(),
                    we::failName(m.fail));
        return m;
      }
      std::printf("%-9s %3zu  %8.1f %8.1f  %5.3f %9.1f %9.1f %5d  %s\n",
                  name, w.size(), m.max_xtrack_m, m.rms_xtrack_m,
                  m.len_ratio, m.terminal_pos_err_m,
                  m.agl_measured ? m.min_agl_m : -1.0,
                  m.zone_measured ? m.zone_hard_contacts : -1,
                  m.verdictName());
      return m;
    };
    // Mandatory anchors: the exact state at every phase junction, taken
    // from the polynomial rather than the nearest dense sample. The
    // extractor merges them; the experiment below shows what they buy.
    std::vector<we::Waypoint> anchors;
    for (size_t i = 0; i + 1 < spans.size(); ++i)
      anchors.push_back(
          we::waypointAtTime(flight, fscale, src, spans[i].t_end));
    const double anchor_sep = 2.0 * we::derivedAcceptRadius(fprm);

    int completed = 0, full_pass = 0, zone_fail_rows = 0, best_n = 0;
    bool best_uniform = true;
    double best_dev = 1e18;
    std::vector<we::Waypoint> best_set;
    for (int n : {8, 16, 32, 64}) {
      const auto wu = we::extractUniformArc(src, n).waypoints;
      const auto wa = we::extractCurvatureAdaptive(src, n).waypoints;
      const auto mu = row("uniform", wu);
      const auto ma = row("adaptive", wa);
      const std::pair<const we::ReproductionMetrics *,
                      const std::vector<we::Waypoint> *>
          rows[] = {{&mu, &wu}, {&ma, &wa}};
      for (const auto &e : rows) {
        if (!e.first->measured) continue;
        ++completed;
        // ONE row must satisfy terrain AND zone AND deviation together —
        // counting them separately let different rows cover different
        // gates (review find).
        if (e.first->verdict() ==
            we::ReproductionMetrics::Verdict::kPass)
          ++full_pass;
        if (e.first->zone_measured && e.first->zone_hard_contacts > 0)
          ++zone_fail_rows;
        // Best = lowest deviation among rows that actually PASSED. A
        // row that violates a zone is not a candidate for the follow-up
        // experiments no matter how small its deviation (review find).
        if (e.first->verdict() == we::ReproductionMetrics::Verdict::kPass &&
            e.first->max_xtrack_m < best_dev) {
          best_dev = e.first->max_xtrack_m;
          best_set = *e.second;
          best_uniform = (e.first == &mu);
          best_n = n;
        }
      }
    }
    std::printf("[WPE] best deviation %.1f m on a %.0f m flight (%.2f%%), "
                "matched-lead baseline %.1f m, deviation gate %.1f m "
                "(experiment parameter, not a validated requirement)\n",
                best_dev, src.total_len_m,
                100.0 * best_dev / std::max(1.0, src.total_len_m),
                base_matched, ep.max_xtrack_gate_m);

    expect(completed >= 6, "most waypoint sets fly the real flight");
    expect(floor_ok, "the continuous-reference baseline is measurable");
    expect(full_pass >= 1,
           "at least ONE waypoint set passes terrain, zone AND deviation "
           "in the SAME row (verdict PASS, safety measured not skipped)");
    // The zone gate has to be able to FAIL something, or a table of
    // passes proves only that nothing was ever at risk. Rows that cut
    // corners near the installed zones do fail it — reported, not
    // asserted, since which row cuts depends on the route.
    std::printf("[WPE] rows failing on hard-zone contact: %d of %d\n",
                zone_fail_rows, completed);
    // The zone gate has to be provably capable of failing, or a table of
    // passes only proves nothing was ever at risk. Three pins:
    //  1. the SOURCE trajectory is clean (the planner did its job);
    //  2. a rollout deliberately driven through a zone centre is caught
    //     (deterministic — does not depend on which row cuts a corner);
    //  3. the incidental row failures are reported for context.
    {
      int src_contacts = 0;
      for (const auto &p_m : src.pos_m)
        if (hooks.zone_probe(p_m) ==
            path_manager::transition_phase::ZoneProbe::CONTACT_HARD)
          ++src_contacts;
      expect(src_contacts == 0,
             "the PLANNED trajectory itself never contacts a hard zone");

      // A synthetic rollout straight through the first zone's centre.
      we::RolloutResult probe;
      probe.completed = true;
      const Eigen::Vector3d zc(snap.zones[0].zone.center.x() * um,
                               snap.zones[0].zone.center.y() * um,
                               snap.zones[0].zone.center.z() * uz);
      const int np = 200;
      for (int i = 0; i <= np; ++i) {
        const double u = static_cast<double>(i) / np;
        probe.samples.push_back(we::RolloutSample{
            u * src.total_time_s,
            src.pos_m.front() + u * (zc - src.pos_m.front()),
            Eigen::Vector3d(170.0, 0.0, 0.0), Eigen::Vector3d::Zero()});
      }
      probe.total_steps = np + 1;
      const auto mprobe =
          we::evaluateReproduction(src, probe, *dynp, hooks, ep);
      expect(mprobe.zone_measured && mprobe.zone_hard_contacts > 0,
             "a rollout driven through a zone centre IS caught by the "
             "zone gate (the gate can fail, deterministically)");
      expect(mprobe.verdict() != we::ReproductionMetrics::Verdict::kPass,
             "a zone-contacting rollout never reads PASS");
    }

    // ---- phase junction stats: does the seam reproduce as well as the
    // flight as a whole? The whole-flight maximum can hide a local spike
    // exactly where the transition hands over.
    if (!best_set.empty()) {
      const auto rb = we::flyWaypoints3Dof(best_set, fstart, fprm);
      bool phase_named = false;
      for (const auto &sp : spans)
        if (sp.name == "departure") phase_named = true;
      std::printf("\n[WPE] junction windows (best set, +/-5 s). %s\n"
                  "junction              t[s]   maxXT[m]  rmsXT[m]  "
                  "posErr[m] spdErr[m/s]\n",
                  phase_named
                      ? "Phase labels applied."
                      : "Phase handoff screening did not apply here (zones "
                        "narrowed the calm windows), so the chain segments "
                        "carry generic names — the transition handoff is "
                        "still the first junction.");
      double worst_junction = 0.0;
      for (size_t i = 0; i + 1 < spans.size(); ++i) {
        const double tj = spans[i].t_end;
        const auto ws = we::windowStats(src, rb, tj - 5.0, tj + 5.0, tj);
        if (!ws.measured) {
          std::printf("%-20s %6.1f  n/a\n", spans[i].name.c_str(), tj);
          continue;
        }
        std::printf("%-10s->%-8s %6.1f  %8.1f %8.1f %9.1f %10.2f\n",
                    spans[i].name.c_str(), spans[i + 1].name.c_str(), tj,
                    ws.max_xtrack_m, ws.rms_xtrack_m, ws.pos_err_at_t_m,
                    ws.speed_err_at_t_mps);
        worst_junction = std::max(worst_junction, ws.max_xtrack_m);
      }
      // Compared against the whole-flight ARC-MATCHED figure, because
      // the window statistic is arc-matched too. The headline 66.9 m is
      // NEAREST-POINT deviation and the two are different measurements —
      // comparing them was apples to oranges (review find).
      // Same implementation as the windows (dense sampling), so the two
      // numbers are the same measurement — max_arcmatch_m samples only
      // 101 fractions and would not be comparable (review find).
      const auto w_all = we::windowStats(src, rb, 0.0, src.total_time_s,
                                         0.5 * src.total_time_s);
      const double whole_arc = w_all.measured ? w_all.max_xtrack_m : 0.0;
      std::printf("[WPE] worst junction window %.1f m vs whole-flight "
                  "arc-matched %.1f m (both arc-matched; the 66.9 m "
                  "headline is nearest-point and NOT comparable here)\n",
                  worst_junction, whole_arc);
      expect(whole_arc > 0.0, "whole-flight arc-matched figure measurable");
      expect(worst_junction <= 1.5 * std::max(whole_arc, 1.0),
             "no junction window reproduces markedly worse than the flight "
             "as a whole, on the SAME metric");

      // The transition handoff showed a larger speed error than the other
      // junctions. That is NOT evidence that the planner's handoff is
      // loose — the source junction is C2 to 1e-13 by construction, and
      // this number is a speed difference at matched PROGRESS, produced
      // by the waypoint reproduction. The experiment that separates the
      // two: force a waypoint exactly AT the junction arc and see whether
      // the error follows the waypoint (reproduction artifact) or stays
      // (something in the handoff itself).
      if (!anchors.empty()) {
        // Same base set, anchors merged by the EXTRACTOR (not by hand):
        // the anchored variant is what a caller would actually get.
        // Through the REAL extractor API — a broken mergeAnchors must
        // fail this regression, not be masked by a test-local
        // reimplementation (review find: the loop used to live here).
        // TWO budgets, because they answer different questions:
        //   equal  : n unchanged, so anchors DISPLACE automatic points —
        //            isolates placement, and exposes what the
        //            displacement costs;
        //   raised : n + anchors, so the automatic set is untouched —
        //            what a caller would actually ship.
        const auto extract = [&](int n, const std::vector<we::Waypoint> &a) {
          const auto e =
              best_uniform
                  ? we::extractUniformArc(src, n, a, anchor_sep)
                  : we::extractCurvatureAdaptive(src, n, 4.0, 1e-4, a,
                                                 anchor_sep);
          expect(e.ok(), std::string("anchored extraction ok (") +
                             we::extractStatusName(e.status) + " " +
                             e.reason + ")");
          return e.waypoints;
        };
        const std::vector<we::Waypoint> forced = extract(best_n, anchors);
        const std::vector<we::Waypoint> raised =
            extract(best_n + static_cast<int>(anchors.size()), anchors);
        const auto rf = we::flyWaypoints3Dof(forced, fstart, fprm);
        const auto mf = we::evaluateReproduction(src, rf, *dynp, hooks, ep);
        const double tj = spans[0].t_end;
        const auto wf = we::windowStats(src, rf, tj - 5.0, tj + 5.0, tj);
        const auto w0 = we::windowStats(src, rb, tj - 5.0, tj + 5.0, tj);
        if (wf.measured && w0.measured)
          std::printf("[WPE] phase anchors via the extractor at the SAME "
                      "budget (%zu -> %zu waypoints, n=%d): transition "
                      "junction maxXT %.1f -> %.1f m, spdErr %.2f -> %.2f "
                      "m/s | whole flight %s\n",
                      best_set.size(), forced.size(), best_n,
                      w0.max_xtrack_m, wf.max_xtrack_m,
                      w0.speed_err_at_t_mps, wf.speed_err_at_t_mps,
                      mf.verdictName());
        expect(wf.measured && w0.measured,
               "the anchored set flies and both junction windows measure");
        const auto rr2 = we::flyWaypoints3Dof(raised, fstart, fprm);
        const auto mr = we::evaluateReproduction(src, rr2, *dynp, hooks, ep);
        const auto wr = we::windowStats(src, rr2, tj - 5.0, tj + 5.0, tj);
        std::printf("[WPE]   equal budget  (%zu wp): %s, maxXT %.1f m, "
                    "minAGL %.1f m, zoneH %d\n"
                    "[WPE]   raised budget (%zu wp): %s, maxXT %.1f m, "
                    "minAGL %.1f m, zoneH %d\n",
                    forced.size(), mf.verdictName(),
                    mf.measured ? mf.max_xtrack_m : -1.0,
                    mf.agl_measured ? mf.min_agl_m : -1.0,
                    mf.zone_measured ? mf.zone_hard_contacts : -1,
                    raised.size(), mr.verdictName(),
                    mr.measured ? mr.max_xtrack_m : -1.0,
                    mr.agl_measured ? mr.min_agl_m : -1.0,
                    mr.zone_measured ? mr.zone_hard_contacts : -1);
        // NOT asserted: that the equal-budget anchored set passes. It may
        // not, and the measurement says why — anchors displace automatic
        // waypoints, and the sparser remainder can lose safety margin
        // elsewhere. Anchors are not free at fixed count; a caller raises
        // the budget instead of displacing. That IS asserted:
        expect(mr.verdict() == we::ReproductionMetrics::Verdict::kPass,
               "with the budget raised by the anchor count, the anchored "
               "set passes terrain, zone AND deviation");
        expect(mr.zone_hard_contacts == 0 && mr.gate_agl_pass,
               "the raised-budget anchored set keeps zone and terrain "
               "clearance");
        if (wr.measured && w0.measured)
          expect(wr.speed_err_at_t_mps < 0.5 * w0.speed_err_at_t_mps,
                 "the junction anchor works at the raised budget too");
        expect(forced.size() <= best_set.size(),
               "anchors consumed the budget (equal-count comparison, not "
               "extra waypoints)");
        // Every junction must literally be in the delivered list.
        int found = 0;
        for (const auto &a : anchors)
          for (const auto &w : forced)
            if (std::abs(w.src_arc_m - a.src_arc_m) <= 1e-9 &&
                (w.pos_m - a.pos_m).norm() < 1e-9) {
              ++found;
              break;
            }
        expect(found == static_cast<int>(anchors.size()),
               "every phase junction survives into the delivered waypoint "
               "list, verbatim");
        if (wf.measured && w0.measured)
          expect(wf.speed_err_at_t_mps < 0.5 * w0.speed_err_at_t_mps,
                 "a junction anchor materially reduces the junction speed "
                 "error (the encoding was the gap, not the handoff)");
        int found_r = 0;
        for (const auto &a : anchors)
          for (const auto &w : raised)
            if (std::abs(w.src_arc_m - a.src_arc_m) <= 1e-9) {
              ++found_r;
              break;
            }
        expect(found_r == static_cast<int>(anchors.size()),
               "the raised-budget set carries every junction too");
      }

      // ---- speed-channel ablation: does the output schema need speed?
      std::vector<we::Waypoint> pos_only = best_set;
      for (auto &w : pos_only) w.speed_mps = 0.0;   // follower cruises
      const auto rp = we::flyWaypoints3Dof(pos_only, fstart, fprm);
      const auto mp = we::evaluateReproduction(src, rp, *dynp, hooks, ep);
      const auto mb = we::evaluateReproduction(src, rb, *dynp, hooks, ep);
      std::printf("\n[WPE] speed-channel ablation (same waypoint "
                  "positions)\n");
      const auto abl = [&](const char *tag, const we::ReproductionMetrics &m) {
        if (!m.measured) {
          std::printf("%-16s n/a(%s)\n", tag, we::failName(m.fail));
          return;
        }
        std::printf("%-16s maxXT %7.1f m  durRatio %5.3f  timeSkew %6.1f s "
                    " endErr %6.1f m  %s\n", tag, m.max_xtrack_m,
                    m.duration_ratio, m.max_time_skew_s,
                    m.terminal_pos_err_m, m.verdictName());
      };
      abl("position+speed", mb);
      abl("position only", mp);
      expect(mb.measured, "position+speed set flies");
      expect(mp.measured, "position-only set flies (both arms measured, or "
                          "the comparison is one-sided)");
      // Not asserted which wins on deviation — the ablation is about the
      // TIMING channel, and the numbers decide the schema rather than a
      // prior belief deciding them. Note what the result does and does
      // NOT say: it shows timing information is needed, not that SPEED
      // specifically is. A per-waypoint arrival time would carry the same
      // information; which encoding the consumer accepts is the open
      // question.
      if (mb.measured && mp.measured)
        std::printf("[WPE] dropping the speed channel: deviation %+.1f m, "
                    "duration ratio %+.3f, time skew %+.1f s. Position is "
                    "required; TIMING must be carried somehow (leg speed "
                    "or arrival time) if schedule matters.\n",
                    mp.max_xtrack_m - mb.max_xtrack_m,
                    mp.duration_ratio - mb.duration_ratio,
                    mp.max_time_skew_s - mb.max_time_skew_s);
    }

    rclcpp::shutdown();
    if (failures == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  if (with_s8bounds) {
    // [S8] Classifier band boundaries, below/at/above. The band is a
    // MODEL hard limit computed from the injected Parameters
    // (model_activation_speed_mps .. speed_max_mps); boundary semantics
    // are inclusive (V == edge is inside). gamma = 32 deg keeps the
    // envelope problem non-empty so the classifier actually reaches the
    // band check.
    using SR = path_manager::SegmentChainPlanner::StartRegime;
    const double g32 = 32.0 * M_PI / 180.0;
    // The band edges come from the INJECTED Parameters, never literals —
    // the contract's 'computed, not written down' rule applies to the
    // test as much as to the code.
    const auto *dynp = pm->dynamicsParams();
    expect(dynp != nullptr, "assumption parameter set available");
    const double v_act = dynp->model_activation_speed_mps;
    const double v_ceiling = dynp->speed_max_mps;
    const auto classify = [&](double v_mps) {
      const Eigen::Vector3d vel_u =
          (v_mps / 100.0) *
          Eigen::Vector3d(std::cos(g32), 0.0, std::sin(g32));
      std::string why;
      return chain.classifyStartState(start_pos, vel_u, Eigen::Vector3d::Zero(),
                                      false, &why);
    };
    expect(classify(v_act - 0.01) == SR::UNSUPPORTED,
           "below the activation floor -> UNSUPPORTED");
    expect(classify(v_act) == SR::TRANSITION_REQUIRED,
           "AT the activation floor -> inside the band (inclusive)");
    expect(classify(v_act + 0.01) == SR::TRANSITION_REQUIRED,
           "above the activation floor -> inside the band");
    expect(classify(v_ceiling - 0.01) == SR::TRANSITION_REQUIRED,
           "below the model ceiling -> inside the band");
    expect(classify(v_ceiling) == SR::TRANSITION_REQUIRED,
           "AT the model ceiling -> inside the band (inclusive)");
    expect(classify(v_ceiling + 0.01) == SR::UNSUPPORTED,
           "above the model ceiling -> UNSUPPORTED");
    // Transition-policy gamma cone at the classifier, below/at/above —
    // same single definition the generator and validator read.
    {
      const double g_pol = path_manager::transition_phase::TransitionLimits{}
                               .max_abs_gamma_rad;
      const auto classifyG = [&](double gam) {
        const Eigen::Vector3d vel_u =
            1.65 * Eigen::Vector3d(std::cos(gam), 0.0, std::sin(gam));
        std::string why;
        return chain.classifyStartState(start_pos, vel_u,
                                        Eigen::Vector3d::Zero(), false,
                                        &why);
      };
      expect(classifyG(g_pol - 0.01) == SR::TRANSITION_REQUIRED,
             "gamma below the policy cone -> inside");
      expect(classifyG(g_pol) == SR::TRANSITION_REQUIRED,
             "gamma AT the policy cone -> inside (inclusive)");
      expect(classifyG(g_pol + 0.01) == SR::UNSUPPORTED,
             "gamma above the policy cone -> UNSUPPORTED");
    }

    if (failures == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  if (with_transitionauto) {
    // [S13] Orchestration regression: the segment-count OPTION must reach
    // the transition path. Auto-N (chain/segments=0) with a small
    // per-segment target has to size the chain from the SUB-route and
    // deliver the full phase semantics behind the transition — a leading
    // TRANSITION span, a departure, at least one cruise-*, an arrival
    // that does not swallow the cruise. (Review find: the option was
    // interpreted inside planImpl, which the transition branch never
    // reaches — the live smoke chained 2 segments with no cruise span.)
    // The pin here is the OPTION PLUMBING: the harness constructs the
    // chain with N=3, forces chain/segments=0, and the auto sizing of
    // the STRAIGHT corridor's sub-route honestly yields N=2 — a value
    // that can only appear if the option was interpreted on the
    // transition path (the pre-fix code kept the reset default 3).
    // Slalom-zone fixtures that would push auto-N to 4+ bend the entry
    // tangents beyond the v1 primitive family's lateral capture
    // capability (three attempts documented in the fix commit) — the
    // N=4 + cruise-* evidence lives in the r5 live smoke, whose 280
    // vertices size honestly.
    const Eigen::Vector3d v32(1.6, 0.0, 1.0);
    const path_manager::PlanResult r =
        chain.plan(start_pos, v32, start_acc, goal,
                   /*start_vel_synthesized=*/false, {},
                   /*start_vel_commanded=*/true);
    expect(r.hasTrajectory(), "auto-N transition mission returns a flight");
    const int n = chain.segments();
    const auto &spans = chain.lastPhaseSpans();
    using PK = path_manager::SegmentChainPlanner::PhaseKind;
    bool lead_trans = !spans.empty() && spans.front().kind == PK::TRANSITION;
    int cruise_ct = 0;
    bool has_dep = false, has_arr = false;
    double arr_dur = 0.0, prev_end = 0.0;
    const double total = spans.empty() ? 0.0 : spans.back().t_end;
    for (const auto &sp : spans) {
      const double d = sp.t_end - prev_end;
      prev_end = sp.t_end;
      if (sp.name.rfind("cruise-", 0) == 0) ++cruise_ct;
      if (sp.name == "departure") has_dep = true;
      if (sp.name == "arrival") { has_arr = true; arr_dur = d; }
    }
    std::cout << "transitionauto: N=" << n << " spans=" << spans.size()
              << " cruise_ct=" << cruise_ct << " arrival=" << arr_dur
              << "/" << total << " s\n";
    expect(n == 2 && n != 3,
           "auto-N REACHED the transition path (fixed default was 3; the "
           "sub-route honestly sizes to 2)");
    expect(lead_trans, "leading TRANSITION span present");
    expect(has_dep && has_arr,
           "phase handoffs present behind the transition (cruise-* needs "
           "N >= 3: pinned by the r5 live smoke)");
    expect(arr_dur < 0.75 * total,
           "arrival does not swallow the flight");

    if (failures == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  if (with_transition) {
    // [S13] Coordinator end-to-end: the r5-class commanded vector
    // (188.7 m/s, gamma = 32 deg — outside the cruise cone, inside the
    // transition model) toward the route direction. With
    // transition/enable=true the mission must come back as a FULL flight:
    // a leading TRANSITION span whose tail sits on the chain head at the
    // C2 tolerance (the seam gate inside planOverRoute), storage and viz
    // of the whole product. The disabled-default outcome is pinned by
    // initfail (reason INITIAL_MODE_UNSUPPORTED, unchanged).
    const Eigen::Vector3d v32(1.6, 0.0, 1.0);  // 188.7 m/s at 32 deg
    const path_manager::PlanResult r =
        chain.plan(start_pos, v32, start_acc, goal,
                   /*start_vel_synthesized=*/false, {},
                   /*start_vel_commanded=*/true);
    std::cout << "transition: outcome " << static_cast<int>(r.outcome)
              << " reason " << static_cast<int>(r.reason) << "\n";
    expect(r.hasTrajectory(), "transition mission returns a flight");
    const poly_traj::Trajectory &full = pm->traj_.local_traj.traj;
    expect(full.getPieceNum() > 0 && full.getTotalDuration() > 10.0,
           "full flight stored (transition + chain)");
    if (!r.hasTrajectory() || full.getPieceNum() == 0) {
      // Probing an EMPTY trajectory is UB (locatePieceIdx indexes past
      // the empty piece vector) — count the failures above and stop.
      std::cout << "FAIL: " << failures << " failed check(s)\n";
      return 1;
    }
    poly_traj::Trajectory probe = full;
    expect((probe.getPos(0.0) - start_pos).norm() < 1e-6,
           "stored flight starts at the COMMANDED mission start");
    const Eigen::Vector3d vel0 = probe.getVel(0.0);
    expect((vel0 - v32).norm() < 1e-6,
           "stored flight leaves with the commanded velocity (no re-aim, "
           "no floor — the head guards held)");
    // start_acc here is 0 = the UNSPECIFIED sentinel (model-implied
    // start acc, mismatch reported in the audit log). The exact-preserve
    // contract for a commanded nonzero acc is pinned at component level
    // (acc_preserve), the refusal of an unflyable one right below.
    // The flight must end at the resolved goal (the chain finished the
    // mission the transition opened).
    const double T = probe.getTotalDuration();
    std::cout << "transition: " << full.getPieceNum() << " pieces, " << T
              << " s, end (" << probe.getPos(T).transpose() << ")\n";
    expect((probe.getPos(T).head<2>() - goal[0].head<2>()).norm() < 12.0,
           "flight reaches the goal area (terminal phase may extend)");
    // Fixed N=3 + phase mode: the FULL phase order behind the transition
    // is deliverable on the straight corridor (no lateral-capture
    // dependence) — assert it explicitly.
    {
      using PK = path_manager::SegmentChainPlanner::PhaseKind;
      const auto &spans = chain.lastPhaseSpans();
      std::vector<std::string> names;
      for (const auto &sp : spans) names.push_back(sp.name);
      std::string joined;
      for (const auto &nm : names) joined += nm + " ";
      std::cout << "transition: spans = " << joined << "\n";
      expect(spans.size() == 4 && spans[0].kind == PK::TRANSITION &&
                 spans[1].kind == PK::CRUISE &&
                 spans[2].kind == PK::CRUISE &&
                 spans[3].kind == PK::CRUISE &&
                 names[0] == "transition" && names[1] == "departure" &&
                 names[2] == "cruise-1" && names[3] == "arrival",
             "EXACT span structure: [TRANSITION, CRUISE x3] named "
             "transition/departure/cruise-1/arrival");
    }

    // A commanded acceleration the model cannot fly (50 g class) with
    // the transition ENABLED must still fail — the initaccfail
    // protection is not allowed to be laundered through the new path.
    const path_manager::PlanResult racc =
        chain.plan(start_pos, v32, Eigen::Vector3d(0.0, 0.0, 5.0), goal,
                   false, {}, true, /*start_acc_commanded=*/true);
    expect(!racc.hasTrajectory(),
           "unflyable commanded start acceleration FAILED with the "
           "transition enabled (no laundering)");
    // Prescribed EXACTLY ZERO through the plumbed message bool: at the
    // 32-deg start a=0 is unflyable — refuse, never model-substitute.
    const path_manager::PlanResult rz =
        chain.plan(start_pos, v32, Eigen::Vector3d::Zero(), goal, false,
                   {}, true, /*start_acc_commanded=*/true);
    expect(!rz.hasTrajectory(),
           "prescribed zero acc refused (the numeric value never decides "
           "prescription)");

    // Classification must not read an UNPRESCRIBED internal acc: an
    // in-cone climb (25 deg < 30) with the FSM's internal a=0 is
    // CRUISE_VALID; the SAME pos/vel with a PRESCRIBED zero is an
    // unflyable hold and must classify differently.
    {
      using SR = path_manager::SegmentChainPlanner::StartRegime;
      const Eigen::Vector3d v25(1.6 * std::cos(25.0 * M_PI / 180.0), 0.0,
                                1.6 * std::sin(25.0 * M_PI / 180.0));
      std::string why;
      const SR un = chain.classifyStartState(
          start_pos, v25, Eigen::Vector3d::Zero(), false, &why);
      const SR pre = chain.classifyStartState(
          start_pos, v25, Eigen::Vector3d::Zero(), true, &why);
      expect(un == SR::CRUISE_VALID,
             "in-cone climb with UNPRESCRIBED internal a=0 -> CRUISE_VALID "
             "(the value is not evidence)");
      expect(pre != SR::CRUISE_VALID,
             "same PVA with PRESCRIBED zero classifies differently (an "
             "unflyable hold)");
    }

    // UNSUPPORTED classification: above the model ceiling — immediate
    // FAILED, nothing downstream runs (over-ceiling is a physics claim
    // v1 refuses to make).
    const path_manager::PlanResult r2 =
        chain.plan(start_pos, Eigen::Vector3d(2.6, 0.0, 0.0), start_acc,
                   goal, false, {}, true);
    expect(!r2.hasTrajectory() &&
               r2.reason ==
                   path_manager::PlanReason::INITIAL_MODE_UNSUPPORTED,
           "above the model ceiling -> UNSUPPORTED, not a transition");

    if (failures == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  if (with_zonesnapshot) {
    using ZD = path_manager::PathManager::ZoneDisposition;
    using ZC = path_manager::PathManager::ZoneContactResult;
    // (a) an AVOIDABLE zone off to the side of the route -> HARD_AVOID,
    // and the structured contact result agrees with the searcher's own
    // hard-volume judgment (not the smooth risk tail).
    path_manager::RiskZone z;
    z.center = Eigen::Vector3d(130.0, 180.0, 2.0);
    z.reach = 8.0;
    z.peak = 0.8;
    pm->setRiskZonesRuntime({z});
    const path_manager::PlanResult r1 =
        chain.plan(start_pos, start_vel, start_acc, goal, true, {});
    expect(r1.hasTrajectory(), "plan with an avoidable zone succeeds");
    const auto snap = pm->zonePolicySnapshot();
    expect(snap.valid, "snapshot valid after a 3-pass search");
    expect(snap.zones.size() == 1 &&
               snap.zones[0].disposition == ZD::HARD_AVOID,
           "avoidable zone -> HARD_AVOID");
    expect(pm->zoneContact(snap, 0, z.center) == ZC::CONTACT,
           "zone centre -> CONTACT (searcher's hard-exclusion judgment)");
    expect(pm->zoneContact(snap, 0, Eigen::Vector3d(30.0, 150.0, 3.0)) ==
               ZC::CLEAR,
           "mission start -> CLEAR");
    double exposure = -1.0;
    expect(pm->zoneExposureRaw(snap, 0, z.center, &exposure) &&
               exposure > 0.0,
           "raw smooth exposure available separately from contact");
    // Boundary pins for the contact gate: the standard is the search's
    // OWN hard-exclusion volume (1.05x + vis>0.35), not the nominal
    // 1.0x rim — a point in the standoff shell must read CONTACT.
    // (A visibility-band pin (0.35 < vis <= 0.5) needs a deterministic
    // terrain-shadow point this DEM harness cannot promise — the shared
    // primitive is the guarantee there.)
    // Probe from the SNAPSHOT's shape copy (the derived zone: AGL mode
    // re-bases center.z on the DEM), exactly as the generator will.
    const path_manager::RiskZone &zderived = snap.zones[0].zone;
    const Eigen::Vector3d shell =
        zderived.center + Eigen::Vector3d(zderived.reach * 1.02, 0.0, 0.0);
    expect(pm->zoneContact(snap, 0, shell) == ZC::CONTACT,
           "standoff shell (q=1.02) -> CONTACT, same standard as a route");
    const Eigen::Vector3d beyond =
        zderived.center + Eigen::Vector3d(zderived.reach * 1.10, 0.0, 0.0);
    expect(pm->zoneContact(snap, 0, beyond) == ZC::CLEAR,
           "beyond the standoff (q=1.10) -> CLEAR");
    // (b) SAME zone list, new plan with the goal INSIDE the zone: the
    // dispositions are per-plan state, so the OLD snapshot must go STALE
    // even though no zone data changed (policy epoch, review find).
    const Eigen::Vector3d goal_in_zone(130.0, 180.0, 2.2);
    const path_manager::PlanResult r2 = chain.plan(
        start_pos, start_vel, start_acc, {goal_in_zone}, true, {});
    expect(r2.hasTrajectory(), "re-plan into the zone succeeds");
    expect(pm->zoneContact(snap, 0, z.center) == ZC::STALE,
           "same-list re-plan makes the old snapshot STALE (epoch)");
    const auto snap2 = pm->zonePolicySnapshot();
    expect(snap2.valid && snap2.zones.size() == 1 &&
               snap2.zones[0].disposition == ZD::SOFT_ENDPOINT,
           "goal-contained zone -> SOFT_ENDPOINT");
    // (c) zone-list change: snapshot taken BEFORE the next search reads
    // INVALID (the searcher still holds the previous binding), and the
    // pre-change snapshot is STALE.
    path_manager::RiskZone z2 = z;
    z2.center = Eigen::Vector3d(200.0, 150.0, 2.0);
    pm->setRiskZonesRuntime({z2});
    expect(pm->zoneContact(snap2, 0, z.center) == ZC::STALE,
           "zone-list change makes the previous snapshot STALE");
    const auto snap3 = pm->zonePolicySnapshot();
    expect(!snap3.valid,
           "snapshot between zone change and next search is INVALID");
    expect(pm->zoneContact(snap3, 0, z2.center) == ZC::INVALID,
           "contact on an INVALID snapshot says INVALID, never CLEAR");
    rclcpp::shutdown();
    if (failures == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  if (with_zonewall) {
    using ZD = path_manager::PathManager::ZoneDisposition;
    // A WALL of overlapping zones spanning the corridor's full height,
    // NOT containing start or goal (a single giant circle swallowed both
    // endpoints and read as an exemption — fixture lesson). Pass 1 finds
    // no zone-free route, pass 2 crosses where exposure is least, pass 3
    // re-hardens the zones the crossing did not need: the crossed zones
    // must read SOFT (UNAVOIDABLE at pass 3, FALLBACK at pass 2), never
    // HARD_AVOID.
    // A straight wall kept losing to the planning grid's true extent (the
    // pass-1 probe legally went around at y=344, then y=463 — the domain
    // is far larger than the DEM's nominal 360x300). Topology beats
    // extent: a RING around the goal cannot be circumnavigated, so every
    // route must cross it.
    std::vector<path_manager::RiskZone> wall;
    const Eigen::Vector3d ring_c(330.0, 150.0, 1.0);
    const double ring_r = 60.0;
    for (int k = 0; k < 12; ++k) {
      const double a = 2.0 * M_PI * k / 12.0;
      path_manager::RiskZone z;
      z.center = ring_c + Eigen::Vector3d(ring_r * std::cos(a),
                                          ring_r * std::sin(a), 0.0);
      z.reach = 30.0;
      z.peak = 0.4;
      wall.push_back(z);
    }
    pm->setRiskZonesRuntime(wall);
    const path_manager::PlanResult r =
        chain.plan(start_pos, start_vel, start_acc, goal, true, {});
    expect(r.hasTrajectory(), "plan through the unavoidable wall succeeds");
    const int pass = pm->zoneAvoidPassNow();
    std::cout << "zonewall: zone-avoid pass " << pass << "\n";
    // diagnostics: did the overrides land, and does the committed route
    // touch any wall volume under the snapshot's own contact judgment?
    double vr = -1.0; bool mask = true;
    node->get_parameter("manager/risk_vertical_ratio", vr);
    node->get_parameter("manager/risk_terrain_mask_enable", mask);
    std::cout << "zonewall: vertical_ratio=" << vr << " mask="
              << (mask ? "on" : "off") << "\n";
    const auto snap = pm->zonePolicySnapshot();
    expect(snap.valid, "snapshot valid after the wall search");
    expect(snap.zones.size() == wall.size(), "all wall zones in snapshot");
    expect(pass >= 2, "search actually needed the soft passes");
    int soft_unavoid = 0, soft_fallback = 0, hard = 0;
    for (const auto &e : snap.zones) {
      if (e.disposition == ZD::SOFT_UNAVOIDABLE) ++soft_unavoid;
      else if (e.disposition == ZD::SOFT_FALLBACK) ++soft_fallback;
      else if (e.disposition == ZD::HARD_AVOID) ++hard;
    }
    std::cout << "zonewall: dispositions unavoid=" << soft_unavoid
              << " fallback=" << soft_fallback << " hard=" << hard << "\n";
    {
      const auto &route = pm->lastCommittedRoute();
      int contact_pts = 0;
      for (const auto &v : route)
        for (size_t zi = 0; zi < snap.zones.size(); ++zi)
          if (pm->zoneContact(snap, zi, v) ==
              path_manager::PathManager::ZoneContactResult::CONTACT) {
            ++contact_pts;
            break;
          }
      std::cout << "zonewall: route vertices in contact " << contact_pts
                << "/" << route.size() << "\n";


    }
    expect(soft_unavoid + soft_fallback >= 1,
           "the needed crossing is SOFT (UNAVOIDABLE/FALLBACK), never "
           "HARD_AVOID");
    expect(pass == 3 && soft_unavoid >= 2 && hard >= 1,
           "overlap crossing releases BOTH members and pass 3 re-hardens "
           "the rest (first-hit marking collapsed this ring to the "
           "all-soft pass 2)");
    if (pass == 3)
      expect(soft_fallback == 0,
             "pass 3: only the needed crossings stay soft (no fallback "
             "labels)");

    // Stage 2 — pass 3 with a disposition MIX: three circles enclosing the
    // goal in their curvilinear-triangle hole. A crossing threads ONE
    // circle's body, so pass 3 re-hardens the other two: expect
    // SOFT_UNAVOIDABLE for the crossed and HARD_AVOID for the rest.
    std::vector<path_manager::RiskZone> tri;
    for (int k = 0; k < 3; ++k) {
      const double a = M_PI / 2.0 + 2.0 * M_PI * k / 3.0;
      path_manager::RiskZone z;
      z.center = ring_c + Eigen::Vector3d(50.0 * std::cos(a),
                                          50.0 * std::sin(a), 0.0);
      z.reach = 45.0;
      z.peak = 0.4;
      tri.push_back(z);
    }
    pm->setRiskZonesRuntime(tri);
    const path_manager::PlanResult r3 =
        chain.plan(start_pos, start_vel, start_acc, goal, true, {});
    expect(r3.hasTrajectory(), "triangle-enclosure plan succeeds");
    const int pass3 = pm->zoneAvoidPassNow();
    const auto snap3 = pm->zonePolicySnapshot();
    int t_unavoid = 0, t_hard = 0, t_fb = 0;
    for (const auto &e : snap3.zones) {
      if (e.disposition == ZD::SOFT_UNAVOIDABLE) ++t_unavoid;
      else if (e.disposition == ZD::HARD_AVOID) ++t_hard;
      else if (e.disposition == ZD::SOFT_FALLBACK) ++t_fb;
    }
    std::cout << "zonewall: triangle pass " << pass3 << " unavoid="
              << t_unavoid << " hard=" << t_hard << " fallback=" << t_fb
              << "\n";
    expect(pass3 >= 2, "triangle enclosure needed the soft passes too");
    if (pass3 == 3) {
      expect(t_unavoid >= 1 && t_hard >= 1,
             "pass 3 mixes SOFT_UNAVOIDABLE (crossed) with re-hardened "
             "HARD_AVOID");
    } else {
      expect(t_fb == 3, "pass 2: all three read SOFT_FALLBACK");
    }

    // Stage 3 — deterministic pass-2 SOFT_FALLBACK: a POROUS ring. Six
    // zones leave 1 u rim gaps the zero-risk soft geodesic threads, so
    // the crossed set stays empty; but the hard field's inflated standoff
    // (1.05 x reach) seals those gaps, so pass 3 (== pass 1 field) cannot
    // connect and the search must settle on the all-soft pass 2.
    std::vector<path_manager::RiskZone> porous;
    for (int k = 0; k < 6; ++k) {
      const double a = 2.0 * M_PI * k / 6.0;
      path_manager::RiskZone z;
      z.center = ring_c + Eigen::Vector3d(60.0 * std::cos(a),
                                          60.0 * std::sin(a), 0.0);
      z.reach = 29.5;
      z.peak = 0.4;
      porous.push_back(z);
    }
    pm->setRiskZonesRuntime(porous);
    const path_manager::PlanResult r4 =
        chain.plan(start_pos, start_vel, start_acc, goal, true, {});
    expect(r4.hasTrajectory(), "porous-ring plan succeeds");
    const int pass4 = pm->zoneAvoidPassNow();
    const auto snap4 = pm->zonePolicySnapshot();
    int p_fb = 0, p_other = 0;
    for (const auto &e : snap4.zones) {
      if (e.disposition == ZD::SOFT_FALLBACK) ++p_fb;
      else if (e.disposition != ZD::SOFT_ENDPOINT) ++p_other;
    }
    std::cout << "zonewall: porous pass " << pass4 << " fallback=" << p_fb
              << " other=" << p_other << "\n";
    expect(pass4 == 2, "sealed gaps force the all-soft pass 2");
    expect(p_fb == 6 && p_other == 0,
           "pass 2: every non-endpoint zone reads SOFT_FALLBACK");
    rclcpp::shutdown();
    if (failures == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  if (with_zonemultileg) {
    using ZC = path_manager::PathManager::ZoneContactResult;
    // A multi-waypoint mission runs one segment search per leg; each leg
    // rewrites the searcher's whole policy state, so the snapshot after
    // such a plan describes only the LAST leg. Publishing that as
    // plan-wide policy is fail-open (a zone an earlier leg hard-avoided
    // could read soft) — the contract is fail-closed: INVALID.
    path_manager::RiskZone z;
    z.center = Eigen::Vector3d(130.0, 180.0, 2.0);
    z.reach = 8.0;
    z.peak = 0.8;
    pm->setRiskZonesRuntime({z});
    const bool ok = pm->planGlobalTraj(
        start_pos, start_vel, start_acc,
        {Eigen::Vector3d(180.0, 80.0, 3.0), goal[0]});
    expect(ok, "two-leg mission plans");
    const auto snap = pm->zonePolicySnapshot();
    expect(!snap.valid, "multi-leg epoch -> snapshot INVALID (fail-closed)");
    expect(pm->zoneContact(snap, 0, z.center) == ZC::INVALID,
           "contact through a multi-leg snapshot reads INVALID");
    // A fresh SINGLE-goal search restores validity on the same zone list.
    const path_manager::PlanResult r1 =
        chain.plan(start_pos, start_vel, start_acc, goal, true, {});
    expect(r1.hasTrajectory(), "single-goal replan succeeds");
    const auto snap2 = pm->zonePolicySnapshot();
    expect(snap2.valid, "single-goal epoch -> snapshot valid again");

    if (failures == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  if (with_zonepass0) {
    using ZC = path_manager::PathManager::ZoneContactResult;
    // Zones present but the lexicographic 3-pass DISABLED: pass stays 0
    // and the soft-override buffer is untrustworthy — the snapshot must be
    // INVALID, fail-closed (review find: the default mapping would have
    // read HARD_AVOID plus possibly a previous search's soft set).
    path_manager::RiskZone z;
    z.center = Eigen::Vector3d(130.0, 180.0, 2.0);
    z.reach = 8.0;
    z.peak = 0.8;
    pm->setRiskZonesRuntime({z});
    const path_manager::PlanResult r =
        chain.plan(start_pos, start_vel, start_acc, goal, true, {});
    expect(r.hasTrajectory(), "plan with policy off succeeds");
    std::cout << "zonepass0: zone-avoid pass " << pm->zoneAvoidPassNow()
              << "\n";
    expect(pm->zoneAvoidPassNow() == 0, "3-pass did not run (policy off)");
    const auto snap = pm->zonePolicySnapshot();
    expect(!snap.valid, "zones + pass 0 -> INVALID snapshot");
    expect(pm->zoneContact(snap, 0, z.center) == ZC::INVALID,
           "contact on the pass-0 snapshot is INVALID, never CLEAR/CONTACT");
    rclcpp::shutdown();
    if (failures == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  if (with_badspans) {
    // A real flight to judge: plan once, keep the stored trajectory.
    const path_manager::PlanResult r0 =
        chain.plan(start_pos, start_vel, start_acc, goal, true, {});
    expect(r0.hasTrajectory(), "fixture plan succeeds");
    const poly_traj::Trajectory traj = pm->traj_.local_traj.traj;
    const double T = traj.getTotalDuration();
    using PK = path_manager::SegmentChainPlanner::PhaseKind;
    using Span = path_manager::SegmentChainPlanner::PhaseSpan;
    // Each span-contract violation must yield UNEVALUATED.
    expect(!chain.evaluateFlight(traj, {{T * 0.5, PK::CRUISE, "short"}})
                .evaluated,
           "last span not covering the flight -> UNEVALUATED");
    expect(!chain.evaluateFlight(
                  traj, {{T * 0.6, PK::CRUISE, "a"}, {T * 0.4, PK::CRUISE,
                                                      "b"}})
                .evaluated,
           "non-increasing t_end -> UNEVALUATED");
    expect(!chain.evaluateFlight(
                  traj, {{T * 0.5, PK::CRUISE, "cruise"},
                         {T, PK::TRANSITION, "late-transition"}})
                .evaluated,
           "TRANSITION after cruise -> UNEVALUATED");
    expect(!chain.evaluateFlight(
                  traj, {{T * 0.5, PK::TERMINAL, "terminal"},
                         {T, PK::CRUISE, "tail"}})
                .evaluated,
           "TERMINAL not last -> UNEVALUATED");
    // The UNEVALUATED -> FAILED(+storage invalidated) verdict mapping is
    // pinned through the PUBLIC path only now (stitchedVerdictResult went
    // private with the coordinator): the transition coordinator variants
    // exercise it end-to-end via planOverRoute. This variant owns the
    // evaluateFlight span contract alone — an intentional coverage
    // relocation, recorded in the privatizing commit.
    rclcpp::shutdown();
    if (failures == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  if (with_overrouteretry) {
    // Coordinator retry pattern, with the leak actually FORCED (review
    // find: piece-count equality on two clean calls never exercised the
    // merge decrement the RAII exists to undo).
    //   A: clean reference call
    //   B: one-shot mid-segment failure -> merge ladder decrements
    //      segments_ mid-plan; after return segments() must be the
    //      confirmed N again
    //   C: clean call — must reproduce A to solver determinism (duration
    //      bitwise, junction PVA), proving B leaked nothing.
    std::vector<Eigen::Vector3d> route;
    std::vector<double> cap;
    double fe_ms = 0.0;
    expect(chain.commitRoute(start_pos, start_vel, start_acc, goal, false,
                             &route, &cap, &fe_ms),
           "commitRoute produces the route");
    const int n_confirmed = chain.segments();
    const path_manager::PlanResult ra = chain.planOverRoute(
        route, cap, fe_ms, start_pos, start_vel, start_acc, goal, true,
        false, {});
    expect(ra.hasTrajectory(), "reference call succeeds");
    const poly_traj::Trajectory ref = pm->traj_.local_traj.traj;

    force("chain/jitter/fail_segment", 2);  // one-shot: forces a merge
    const path_manager::PlanResult rb = chain.planOverRoute(
        route, cap, fe_ms, start_pos, start_vel, start_acc, goal, true,
        false, {});
    expect(rb.hasTrajectory(), "merged call still delivers a trajectory");
    expect(node->get_parameter("chain/jitter/fail_segment").as_int() == 0,
           "fault injection consumed (the failure was actually injected)");
    // The MERGE must be what repaired it — not the single-shot fallback
    // (review find: at N=3 the protected phase boundaries exhausted the
    // ladder and the fallback produced a false positive here).
    expect(rb.outcome == path_manager::PlanOutcome::SUCCESS,
           "merge rescue stays SUCCESS (no fallback, no degrade)");
    expect(rb.reason != path_manager::PlanReason::SINGLE_PLAN_FALLBACK,
           "repair was the merge ladder, not the single-shot fallback");
    expect(chain.segments() == n_confirmed,
           "segments() restored to the confirmed N after the merge call");

    const path_manager::PlanResult rc = chain.planOverRoute(
        route, cap, fe_ms, start_pos, start_vel, start_acc, goal, true,
        false, {});
    expect(rc.hasTrajectory(), "post-merge clean call succeeds");
    const poly_traj::Trajectory &out = pm->traj_.local_traj.traj;
    std::cout << "overrouteretry: ref " << ref.getPieceNum() << " pieces "
              << ref.getTotalDuration() << " s vs post-merge "
              << out.getPieceNum() << " pieces " << out.getTotalDuration()
              << " s\n";
    expect(out.getPieceNum() == ref.getPieceNum(),
           "post-merge clean call reproduces the reference piece count");
    expect(std::abs(out.getTotalDuration() - ref.getTotalDuration()) < 1e-9,
           "duration reproduced within deterministic tolerance");
    bool junc_ok = true;
    for (int j = 1; j < ref.getPieceNum() && junc_ok; ++j)
      junc_ok = (out.getJuncPos(j) - ref.getJuncPos(j)).norm() < 1e-9 &&
                (out.getJuncVel(j) - ref.getJuncVel(j)).norm() < 1e-9 &&
                (out.getJuncAcc(j) - ref.getJuncAcc(j)).norm() < 1e-9;
    expect(junc_ok, "every junction P/V/A reproduced within deterministic "
                    "tolerance (no plan-state leak)");
    rclcpp::shutdown();
    if (failures == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  if (with_overroutebad) {
    std::vector<Eigen::Vector3d> route;
    std::vector<double> cap;
    double fe_ms = 0.0;
    expect(chain.commitRoute(start_pos, start_vel, start_acc, goal, false,
                             &route, &cap, &fe_ms),
           "commitRoute produces a route to tamper with");
    // (a) truncated cap — the silent-abandonment case
    std::vector<double> cap_bad(cap.begin(), cap.end() - 1);
    const path_manager::PlanResult ra = chain.planOverRoute(
        route, cap_bad, fe_ms, start_pos, start_vel, start_acc, goal, true,
        false, {});
    expect(!ra.hasTrajectory() &&
               ra.detail.find("cap size") != std::string::npos,
           "mismatched cap FAILS with the invariant named");
    // (b) head displaced off the route start
    const path_manager::PlanResult rb = chain.planOverRoute(
        route, cap, fe_ms, start_pos + Eigen::Vector3d(1.0, 0.0, 0.0),
        start_vel, start_acc, goal, true, false, {});
    expect(!rb.hasTrajectory() &&
               rb.detail.find("head position") != std::string::npos,
           "head off the route start FAILS");
    // (c) NaN vertex
    std::vector<Eigen::Vector3d> route_nan = route;
    route_nan[route_nan.size() / 2].z() = std::nan("");
    const path_manager::PlanResult rc = chain.planOverRoute(
        route_nan, cap, fe_ms, start_pos, start_vel, start_acc, goal, true,
        false, {});
    expect(!rc.hasTrajectory() &&
               rc.detail.find("non-finite route") != std::string::npos,
           "NaN route vertex FAILS");
    rclcpp::shutdown();
    if (failures == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  if (with_transfallback) {
    // Coordinator-style direct drive (plan() would clear the flag at its
    // resetPlanState): commit the route, assert the transition, then make
    // every worker fail so the repair path wants the single-shot fallback —
    // which must now be REFUSED, not a re-plan from the mission start.
    std::vector<Eigen::Vector3d> route;
    std::vector<double> cap;
    double fe_ms = 0.0;
    expect(chain.commitRoute(start_pos, start_vel, start_acc, goal, false,
                             &route, &cap, &fe_ms),
           "commitRoute produces the route");
    chain.setTransitionActive(true);
    force("chain/jitter/fail_segment", -1);
    const path_manager::PlanResult r = chain.planOverRoute(
        route, cap, fe_ms, start_pos, start_vel, start_acc, goal, true,
        false, {});
    expect(!r.hasTrajectory(),
           "transition-active: single-shot fallback refused (FAILED)");
    expect(r.detail.find("forbidden while a transition is active") !=
               std::string::npos,
           "reason states the fallback prohibition");
    rclcpp::shutdown();
    if (failures == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  if (with_handoffcap) {
    // The PRODUCTION declaration is part of the contract: force() would
    // declare the name itself, so a deleted declare_parameter() in
    // PathManager would go unnoticed (review find). Assert it exists
    // FIRST, then use plain set_parameter — which fails on undeclared
    // names — for the rest.
    expect(node->has_parameter("planning/handoff_max_vel_mps"),
           "PathManager declares the handoff cap parameter");
    auto sp = [&](double v) {
      return node->set_parameter(
                     rclcpp::Parameter("planning/handoff_max_vel_mps", v))
          .successful;
    };
    expect(sp(220.0), "set_parameter(220) accepted");
    const std::string ok210 =
        pm->stateEnvelopeProblem(Eigen::Vector3d(2.1, 0.0, 0.0));
    expect(ok210.empty(),
           "handoff cap 220: 210 m/s passes the speed check");
    expect(sp(190.0), "set_parameter(190) accepted");
    const std::string no195 =
        pm->stateEnvelopeProblem(Eigen::Vector3d(1.95, 0.0, 0.0));
    expect(!no195.empty() &&
               no195.find("explicit handoff cap") != std::string::npos,
           "handoff cap 190: 195 m/s refused, reason names the explicit cap");
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
    expect(r.detail.find("default planning cap") != std::string::npos,
           "reason names the DEFAULT planning cap (handoff param unset)");
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
                   /*start_vel_commanded=*/true,
                   /*start_acc_commanded=*/true);
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
