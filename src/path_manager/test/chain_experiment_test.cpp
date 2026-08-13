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
#include <filesystem>
#include <fstream>
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

// The FINAL-EVAL verdict is what a reader treats as the answer, and it goes
// to the log file rather than to stdout — so a claim about it has to be
// checked there. Returns false when no verdict line exists at all, which is
// itself a failure for any assertion phrased as "the verdict says X".
bool lastFinalEvalHas(const std::string &needle)
{
  namespace fs = std::filesystem;
  const fs::path dir{"./logs/runtime"};
  if (!fs::exists(dir)) return false;
  fs::path newest;
  fs::file_time_type best{};
  for (const auto &e : fs::directory_iterator(dir)) {
    const auto n = e.path().filename().string();
    if (n.rfind("chain_experiment", 0) != 0) continue;
    const auto t = fs::last_write_time(e);
    if (newest.empty() || t > best) { newest = e.path(); best = t; }
  }
  if (newest.empty()) return false;
  std::ifstream in(newest);
  std::string line, last;
  while (std::getline(in, line))
    if (line.find("[FINAL-EVAL] verdict:") != std::string::npos) last = line;
  return !last.empty() && last.find(needle) != std::string::npos;
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


// [HEAD-POLICY] plan() takes the head as one value now. These call sites
// used to assemble three booleans by hand, which is exactly the assembly
// that produced an illegal combination in production (a prescribed
// acceleration dropped because the velocity came in the scalar form).
static path_manager::StartHead mkHead(
    path_manager::StartStateSource src, const Eigen::Vector3d &p,
    const Eigen::Vector3d &v, const Eigen::Vector3d &a,
    bool acc_prescribed = false)
{
  path_manager::StartHead h;
  h.src = src;
  h.pos_u = p;
  h.vel_u = v;
  h.acc_u = a;
  h.acc_prescribed = acc_prescribed;
  return h;
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
       with_zonewall = false, with_zonepass0 = false, with_hardpen = false,
       with_standoffpen = false, with_wpzone = false,
       with_nophasedirect = false, with_baserefuse = false,
       with_reststart = false, with_capstart = false,
       with_headsrc = false, with_legpolicy = false, with_legtags = false,
       with_legleadin = false, with_legchain = false, with_legaudit = false,
       with_wpzonepass0 = false, with_legmid = false,
       with_cutarc = false, with_transwp = false, with_legseam = false,
       with_dynprobe = false, with_fm2fail = false,
       with_legfail = false,
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
    // machinery. synthclamp: where the [STALL-FLOOR] clamp may and may not
    // reach — a SYNTHESIZED sub-floor start is REFUSED (its speed is the
    // mission's own initial_speed; only the direction was ours), a flyable
    // synthesized start plans, and a TRAJECTORY-DERIVED sub-floor head is
    // still clamped and flies (that state this stack authored itself).
    // unsafedirect: the phase-mode direct
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
    // hardpen: a flight is judged against a zone the policy marked
    // HARD_AVOID and that the trajectory demonstrably passes through. No
    // planner run can produce this — the 3-pass exists to avoid it — so the
    // flight is SYNTHESISED and handed to the evaluator directly. Without
    // it, "a hard contact refuses the flight" is a branch nothing executes.
    if (v == "hardpen") { with_route = true; with_hardpen = true; }
    if (v == "standoffpen") { with_route = true; with_standoffpen = true; }
    if (v == "wpzone") { with_route = true; with_wpzone = true; }
    if (v == "nophasedirect") { with_route = true; with_autosmall = true; with_nophasedirect = true; }
    if (v == "baserefuse") { with_autosmall = true; with_baserefuse = true; }
    if (v == "reststart") { with_route = true; with_reststart = true; }
    if (v == "capstart") { with_route = true; with_capstart = true; }
    if (v == "headsrc") { with_route = true; with_headsrc = true; }
    if (v == "legpolicy") { with_route = true; with_legpolicy = true; }
    if (v == "legtags") { with_route = true; with_legtags = true; }
    if (v == "legleadin") { with_route = true; with_legleadin = true; }
    if (v == "legchain") { with_legchain = true; }
    if (v == "legaudit") { with_legaudit = true; }
    if (v == "legmid") { with_legmid = true; }
    if (v == "cutarc") { with_cutarc = true; }
    if (v == "legseam") { with_legseam = true; }
    if (v == "legfail") { with_legfail = true; }
    if (v == "dynprobe") { with_dynprobe = true; }
    if (v == "fm2fail") { with_fm2fail = true; }
    if (v == "transwp") { with_transwp = true; with_route = true; }
    if (v == "wpzonepass0") { with_wpzonepass0 = true; }
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
    if (with_zonepass0 || with_baserefuse || with_wpzonepass0)
      ovr.emplace_back("manager/zone_avoid_lexicographic", false);
    // zonewall: block the OVER-THE-TOP escape (default vertical ratio
    // 0.35 leaves a ~12 u ceiling the front end can climb past) so the
    // wall is genuinely unavoidable and the soft passes must run.
    if (with_transition || with_waypoints || with_transwp)
      ovr.emplace_back("transition/enable", true);
    // These four pin the flag OFF because that is the contract they test:
    // with no transition planner behind it, a commanded start outside the
    // cruise envelope is refused at plan entry, with reason
    // INITIAL_MODE_UNSUPPORTED and with nothing downstream of validation
    // having run. optimizer_params.yaml now ships the flag ON, and with it
    // on the planner legitimately behaves differently — it ATTEMPTS the
    // transition, so the refusal (when the generator finds no flyable
    // candidate) arrives later and as TRANSITION_GENERATION_FAILED, which
    // is the more accurate statement. Both behaviours are wanted; each is
    // pinned by the variants that mean to test it, so neither depends on
    // what the shipped default happens to be.
    if (with_initfail || with_initaccfail || with_initceiling ||
        with_pvaprobe)
      ovr.emplace_back("transition/enable", false);
    if (with_zonewall) {
      ovr.emplace_back("manager/risk_vertical_ratio", 3.0);
      // ... and the terrain-shadow escape: with LOS masking on, ridge
      // shadows carve INVISIBLE corridors through the wall and pass 1
      // legitimately threads them. Whole-ellipsoid volumes make the wall
      // airtight.
      ovr.emplace_back("manager/risk_terrain_mask_enable", false);
    }
    // [LEG-POLICY] The leg-junction split only exists where a corner fillet
    // is placed, and manager/corner_fillet_radius ships as 0.0 — fillets are
    // OFF by default. The first draft of `legtags` asserted the handover
    // vertex without turning them on, and a mutation deleting the split
    // survived: the branch was never entered at all.
    if (with_legtags || with_legleadin)
      ovr.emplace_back("manager/corner_fillet_radius", 12.0);
    // [FM2-FAIL] One cell is not a grid. The FM2 map cannot be built, so the
    // geodesic cannot be extracted — the one exit that the obstacle fixtures
    // do NOT reach (they fail later, in the search branch). manager/* is read
    // in the PathManager constructor, so it rides NodeOptions.
    if (with_fm2fail) ovr.emplace_back("manager/fm2_max_cells", 1);
    // [LEG-POLICY] optimization/lead_in_time ships as 0.0, so the optimizer's
    // lead-in insertion — one of the two places clean_path grows after the
    // tags were built — never fires in the default configuration and its
    // mirror onto the tag vector was dead code no variant entered.
    if (with_legleadin) ovr.emplace_back("optimization/lead_in_time", 1.0);
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
        chain.plan(mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos, start_vel, start_acc, false), goal, bad);
    expect(!r1.hasTrajectory(),
           "sub-stall final boundary rejected (FAILED, no opt-in)");
    force("planning/allow_final_boundary_relaxation", true);
    const path_manager::PlanResult r2 =
        chain.plan(mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos, start_vel, start_acc, false), goal, bad);
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
        chain.plan(mkHead(path_manager::StartStateSource::STATED_VECTOR, start_pos, Eigen::Vector3d(0.0, -1.6, 1.0), start_acc, false), goal, {});
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
    const path_manager::PlanResult r = chain.plan(mkHead(path_manager::StartStateSource::STATED_VECTOR, p_u, v_u, a_u, true), goal, {});
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
        chain.plan(mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos, start_vel, start_acc, false), goal, {});
    expect(r1.hasTrajectory(), "first plan (N=3) produces a trajectory");
    const int p1 = pm->traj_.local_traj.traj.getPieceNum();
    force("chain/segments", 5);
    const path_manager::PlanResult r2 =
        chain.plan(mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos, start_vel, start_acc, false), goal, {});
    expect(r2.hasTrajectory(), "second plan (N=5) produces a trajectory");
    const int p2 = pm->traj_.local_traj.traj.getPieceNum();
    std::cout << "twophase: N=3 -> " << p1 << " pieces, N=5 -> " << p2
              << " pieces, segments() -> " << chain.segments() << "\n";
    // The leak this guards against is a STALE N: plan 2 re-using plan 1's
    // segment count (possibly already decremented by a merge) instead of
    // re-reading chain/segments. segments_ is exactly that state, so read
    // it directly.
    //
    // This used to assert p1 != p2 — the piece count as a proxy for "N
    // changed". The proxy collides: on this straight-line fixture the
    // per-segment piece counts are set by length_per_piece, so 3 segments
    // (2/9/2) and 5 segments (2/3/3/3/2) both total 13 once the junctions
    // are spread evenly over the route. The piece counts stay as printed
    // diagnostics; they are not evidence either way.
    expect(chain.segments() == 5,
           "segment count change reaches the second plan (no per-plan "
           "state leak)");
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
        chain.plan(mkHead(path_manager::StartStateSource::STATED_VECTOR, start_pos, v32, start_acc, false), goal, {});
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
              return path_manager::transition_phase::ZoneProbe::CONTACT_AUTHORED;
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
            path_manager::transition_phase::ZoneProbe::CONTACT_AUTHORED)
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
      // the window statistic is arc-matched too. The headline deviation is
      // NEAREST-POINT and the two are different measurements — comparing
      // them was apples to oranges (review find).
      // Same implementation as the windows (dense sampling), so the two
      // numbers are the same measurement — max_arcmatch_m samples only
      // 101 fractions and would not be comparable (review find).
      const auto w_all = we::windowStats(src, rb, 0.0, src.total_time_s,
                                         0.5 * src.total_time_s);
      const double whole_arc = w_all.measured ? w_all.max_xtrack_m : 0.0;
      std::printf("[WPE] worst junction window %.1f m vs whole-flight "
                  "arc-matched %.1f m (both arc-matched; the %.1f m "
                  "headline is nearest-point and NOT comparable here)\n",
                  worst_junction, whole_arc, best_dev);
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
        // What an anchor promises is an ABSOLUTE one: it pins position,
        // time and speed magnitude at the junction, so the reproduced
        // speed there lands near the source's. kAnchoredSpdErrMax is 1% of
        // the ~200 m/s cruise; measured anchored values sit near 0.9 m/s.
        //
        // This used to be written as "anchored < 0.5 x unanchored", which
        // silently assumed the unanchored layout leaves a LARGE error at
        // the junction to begin with. That assumption was an artifact of
        // where the automatic waypoints happened to fall: when a junction
        // moves, the curvature-weighted layout shifts with it, and an
        // unanchored point landing near the junction already drives the
        // error down (measured: unanchored 9.24 m/s before the junction
        // selection fix, 0.43 m/s after — same fixture, same tj, same
        // strategy and budget). A ratio cannot be met when there is
        // nothing left to halve, so the ratio form is kept below only
        // where its precondition actually holds.
        constexpr double kAnchoredSpdErrMax = 2.0;  // m/s
        if (wr.measured && w0.measured) {
          expect(wr.speed_err_at_t_mps < kAnchoredSpdErrMax,
                 "the junction anchor works at the raised budget too (speed "
                 "pinned at the junction)");
          if (w0.speed_err_at_t_mps >= kAnchoredSpdErrMax)
            expect(wr.speed_err_at_t_mps < 0.5 * w0.speed_err_at_t_mps,
                   "and where the unanchored layout DOES leave a material "
                   "junction speed error, the raised-budget anchor halves it");
        }
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
        // Same reasoning as the raised-budget check above. The claim being
        // pinned is that the junction speed error lives in the ENCODING,
        // not in the handoff: the source junction is C2 to 1e-13, and the
        // reproduced error tracks where the waypoints sit. An anchor that
        // holds the error near zero demonstrates that in either regime;
        // the halving is only demonstrable when the unanchored layout left
        // a material error there.
        if (wf.measured && w0.measured) {
          expect(wf.speed_err_at_t_mps < kAnchoredSpdErrMax,
                 "a junction anchor holds the junction speed error down "
                 "(the encoding was the gap, not the handoff)");
          if (w0.speed_err_at_t_mps >= kAnchoredSpdErrMax)
            expect(wf.speed_err_at_t_mps < 0.5 * w0.speed_err_at_t_mps,
                   "and it materially reduces a large unanchored junction "
                   "speed error at equal budget");
        }
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
        chain.plan(mkHead(path_manager::StartStateSource::STATED_VECTOR, start_pos, v32, start_acc, false), goal, {});
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
        chain.plan(mkHead(path_manager::StartStateSource::STATED_VECTOR, start_pos, v32, start_acc, false), goal, {});
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
        chain.plan(mkHead(path_manager::StartStateSource::STATED_VECTOR, start_pos, v32, Eigen::Vector3d(0.0, 0.0, 5.0), true), goal, {});
    expect(!racc.hasTrajectory(),
           "unflyable commanded start acceleration FAILED with the "
           "transition enabled (no laundering)");
    // Prescribed EXACTLY ZERO through the plumbed message bool: at the
    // 32-deg start a=0 is unflyable — refuse, never model-substitute.
    const path_manager::PlanResult rz =
        chain.plan(mkHead(path_manager::StartStateSource::STATED_VECTOR, start_pos, v32, Eigen::Vector3d::Zero(), true), goal, {});
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
        chain.plan(mkHead(path_manager::StartStateSource::STATED_VECTOR, start_pos, Eigen::Vector3d(2.6, 0.0, 0.0), start_acc, false), goal, {});
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
        chain.plan(mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos, start_vel, start_acc, false), goal, {});
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
    const path_manager::PlanResult r2 = chain.plan(mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos, start_vel, start_acc, false), {goal_in_zone}, {});
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
        chain.plan(mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos, start_vel, start_acc, false), goal, {});
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
        chain.plan(mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos, start_vel, start_acc, false), goal, {});
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
        chain.plan(mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos, start_vel, start_acc, false), goal, {});
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
    // The whole-flight evaluation must agree with the policy it was handed.
    // With every zone SOFT_FALLBACK the crossing is what the 3-pass DECIDED
    // to do, so the flight may touch zone volume and still be clean — the
    // HARD counter has to stay at zero. Counting containment instead of
    // disposition reported 3 "hard contacts" on exactly this shape and
    // called a permitted crossing a safety breach.
    const auto &fvp = chain.lastFlightVerdict();
    expect(fvp.evaluated, "the porous-ring flight was actually evaluated");
    expect(fvp.policy_measurable,
           "a valid pass-2 snapshot is measurable");
    expect(fvp.zone_hard_n == 0,
           "a pass-2 all-soft crossing counts ZERO hard contacts");
    expect(fvp.zone_soft_n > 0,
           "...and the crossing IS counted as soft — a zero here would mean "
           "the check never ran, which the old log-string form could not "
           "tell apart from success");
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
        mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos,
               start_vel, start_acc),
        {Eigen::Vector3d(180.0, 80.0, 3.0), goal[0]});
    expect(ok, "two-leg mission plans");
    const auto snap = pm->zonePolicySnapshot();
    expect(!snap.valid, "multi-leg epoch -> snapshot INVALID (fail-closed)");
    expect(pm->zoneContact(snap, 0, z.center) == ZC::INVALID,
           "contact through a multi-leg snapshot reads INVALID");
    // A fresh SINGLE-goal search restores validity on the same zone list.
    const path_manager::PlanResult r1 =
        chain.plan(mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos, start_vel, start_acc, false), goal, {});
    expect(r1.hasTrajectory(), "single-goal replan succeeds");
    const auto snap2 = pm->zonePolicySnapshot();
    expect(snap2.valid, "single-goal epoch -> snapshot valid again");

    if (failures == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  if (with_hardpen) {
    using ZD = path_manager::PathManager::ZoneDisposition;
    using ZC = path_manager::PathManager::ZoneContactResult;
    // A zone far from the route, so a NORMAL plan avoids it and the 3-pass
    // leaves it HARD_AVOID on a VALID snapshot. Then a flight is synthesised
    // straight THROUGH its centre and handed to the evaluator: the planner
    // will not build such a trajectory, which is exactly why the refusal
    // branch needs one made by hand. Without this the "hard contact refuses
    // the flight" path ships untested.
    path_manager::RiskZone z;
    z.center = Eigen::Vector3d(180.0, 120.0, 3.0);
    z.reach = 20.0;
    z.peak = 0.9;
    pm->setRiskZonesRuntime({z});
    const path_manager::PlanResult r0 =
        chain.plan(mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos, start_vel, start_acc, false), goal, {});
    expect(r0.hasTrajectory(), "the avoiding plan itself succeeds");
    const auto snap = pm->zonePolicySnapshot();
    expect(snap.valid, "snapshot is VALID (the policy really ran)");
    const bool is_hard = !snap.zones.empty() &&
                         snap.zones[0].disposition == ZD::HARD_AVOID;
    expect(is_hard, "an unneeded zone stays HARD_AVOID");
    expect(pm->zoneContact(snap, 0, z.center) == ZC::CONTACT,
           "the zone centre reads CONTACT on this snapshot");

    // Straight line through the zone centre. The SPEED matters: 120 u in
    // 120 s is 1.0 u/s = 100 m/s, below the 122 m/s cruise minimum, so such
    // a flight is unflyable on no_cruise ALONE and the test would keep
    // passing if the hard-zone condition were later dropped from
    // unflyable(). 60 u per 40 s piece is 1.5 u/s = 150 m/s — inside the
    // band — which leaves the hard contact as the only cause available.
    const Eigen::Vector3d a(z.center.x() - 60.0, z.center.y(), z.center.z());
    const Eigen::Vector3d b(z.center.x() + 60.0, z.center.y(), z.center.z());
    const auto seg = [&](double f0, double f1) {
      Eigen::Matrix<double, 3, 6> c = Eigen::Matrix<double, 3, 6>::Zero();
      const Eigen::Vector3d p0 = a + f0 * (b - a);
      const Eigen::Vector3d p1 = a + f1 * (b - a);
      c.col(5) = p0;                 // constant term
      c.col(4) = (p1 - p0) / 40.0;   // linear over the 40 s piece
      return c;
    };
    const poly_traj::Trajectory through(
        std::vector<double>{40.0, 40.0},
        std::vector<poly_traj::CoefficientMat>{seg(0.0, 0.5), seg(0.5, 1.0)});
    const auto fv = chain.evaluateFlightForTest(
        through, {{through.getTotalDuration(),
                   path_manager::SegmentChainPlanner::PhaseKind::CRUISE,
                   "synthetic"}});
    expect(fv.evaluated, "the synthetic flight was evaluated");
    // Every OTHER unflyable cause must be absent, or "unflyable" below says
    // nothing about the hard zone.
    expect(!fv.underground, "the synthetic flight does not clip terrain");
    expect(!fv.no_cruise, "...and it does reach cruise speed");
    expect(fv.policy_measurable, "...on a snapshot that IS measurable");
    expect(fv.zone_hard_n > 0,
           "a flight through a HARD_AVOID volume counts hard contacts");
    expect(fv.unflyable(),
           "and THAT alone makes it unflyable — the hard contact is the only "
           "cause left standing");

    // The verdict must also reach the caller as a refusal. Detecting a hard
    // contact and then returning it as a gradeable degradation would be the
    // original defect wearing a new number.
    // Store it first, the way an earlier stage would have. ReplanFSM
    // executes local_traj on (duration > 0 && start_time > 0) alone
    // (replan_fsm.cpp:461), so those two fields ARE the executability
    // contract — and both must be non-zero going in, or the assertion
    // below would pass against a default-constructed slot.
    pm->traj_.setLocalTraj(through, 100.0, 0);
    expect(pm->traj_.local_traj.duration > 0.0 &&
               pm->traj_.local_traj.start_time > 0.0,
           "the trajectory really is executable before the gate runs");
    const auto pr = chain.verdictResultForTest(fv);
    expect(!pr.hasTrajectory(), "a hard contact FAILS the plan");
    expect(pr.reason == path_manager::PlanReason::STITCHED_FLIGHT_UNSAFE,
           "...as STITCHED_FLIGHT_UNSAFE, not an envelope budget");
    expect(pm->traj_.local_traj.duration == 0.0 &&
               pm->traj_.local_traj.start_time == 0.0,
           "...and the stored trajectory stops being executable");
    rclcpp::shutdown();
    if (failures == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  if (with_legpolicy) {
    using ZD = path_manager::PathManager::ZoneDisposition;
    // [LEG-POLICY] step 1: each leg's policy is captured while THAT leg's
    // search owns the searcher, so a multi-leg mission has one policy per
    // leg instead of the last leg's policy published as plan-wide.
    //
    // The zone sits at the MISSION GOAL. Leg 1 ends inside it, so its
    // policy must be SOFT_ENDPOINT — you may not avoid a zone you are told
    // to arrive in. Leg 0 neither starts nor ends there, so the same zone is
    // HARD_AVOID for it. That difference is the whole reason per-leg policy
    // exists: a plan-wide snapshot has to pick one and is wrong for the
    // other.
    //
    // The waypoint itself must be OUTSIDE the zone — the first draft put the
    // zone on the intermediate waypoint, which is leg 1's START, so both
    // legs saw an endpoint and the asymmetry never arose.
    const Eigen::Vector3d mid(180.0, 80.0, 3.0);
    path_manager::RiskZone z;
    z.center = goal[0];
    z.reach = 20.0;
    z.peak = 0.9;
    pm->setRiskZonesRuntime({z});

    const bool ok = pm->planGlobalTraj(
        mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos,
               start_vel, start_acc),
        {mid, goal[0]});
    expect(ok, "the two-leg mission plans");

    const auto &legs = pm->legPolicySnapshots();
    expect(legs.size() == 2,
           "one policy per leg was captured, not one for the mission");
    if (legs.size() == 2) {
      expect(legs[0].leg == 0 && legs[1].leg == 1,
             "...in leg order");
      expect(legs[0].search_serial != legs[1].search_serial,
             "...each with its own search serial (no double-write)");
      expect(legs[0].policy.valid && legs[1].policy.valid,
             "...and each is valid for ITS leg");
      expect(legs[0].policy.zones.size() == 1 &&
                 legs[1].policy.zones.size() == 1,
             "...describing the one zone installed");
      if (!legs[0].policy.zones.empty() && !legs[1].policy.zones.empty()) {
        const auto d0 = legs[0].policy.zones[0].disposition;
        const auto d1 = legs[1].policy.zones[0].disposition;
        std::cout << "[LEG-POLICY] leg0=" << static_cast<int>(d0)
                  << " leg1=" << static_cast<int>(d1) << "\n";
        expect(d1 == ZD::SOFT_ENDPOINT,
               "leg 1 ends INSIDE the zone -> SOFT_ENDPOINT");
        expect(d0 != ZD::SOFT_ENDPOINT,
               "leg 0 neither starts nor ends there -> not SOFT_ENDPOINT");
        expect(d0 != d1,
               "...the SAME zone has different dispositions per leg, which "
               "is what a plan-wide snapshot cannot express");
      }
    }
    // The plan-wide snapshot must STILL refuse this mission — its rule is
    // not relaxed by the existence of per-leg capture.
    expect(!pm->zonePolicySnapshot().valid,
           "the plan-wide snapshot is still INVALID for a multi-leg epoch");
    rclcpp::shutdown();
    if (failures == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  if (with_fm2fail) {
    // [FM2-FAIL] When FM2 cannot produce a geodesic the search FAILS. It used
    // to answer with { start_pt, end_pt } — a two-point straight segment
    // through whatever is in the way — and because the caller treats any
    // result with >= 2 points as a successful search, the mission planned and
    // flew on it. The failure lived only in a log line nothing downstream
    // reads.
    //
    // fm2_max_cells = 1 makes that deterministic: no grid, no field, no
    // geodesic. The obstacle fixtures in `dynprobe` exercise the OTHER exit
    // (the search branch), so without this the extraction-failure exit had no
    // regression at all.
    const auto head = [&]() {
      return mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos,
                    start_vel, start_acc);
    };
    const bool ok = pm->planGlobalTraj(head(), {goal[0]});
    std::cout << "[FM2-FAIL] plan=" << (ok ? 1 : 0)
              << " route=" << pm->lastCommittedRoute().size()
              << " piece_leg=" << pm->lastPieceLeg().size()
              << " pieces=" << pm->traj_.local_traj.traj.getPieceNum()
              << " dur=" << pm->traj_.local_traj.duration << "\n";
    expect(!ok, "an FM2 grid that cannot be built fails the plan");
    expect(pm->lastCommittedRoute().empty(),
           "...leaving NO committed route — a straight line nobody checked "
           "for obstacles is not an answer to a failed search");
    expect(pm->lastPieceLeg().empty() && pm->legPolicySnapshots().empty(),
           "...and no provenance");
    expect(pm->traj_.local_traj.duration <= 0.0,
           "...and nothing executable in the trajectory slot");
    rclcpp::shutdown();
    if (failures == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  if (with_dynprobe) {
    // [DYN-OBSTACLE] The front end must not route through a registered
    // dynamic obstacle, and when it cannot get around one it must say so
    // rather than hand back a straight line.
    //
    // MEASURING THIS IS THE HARD PART, and two earlier attempts got it
    // wrong. addDynamicBox applies a random yaw, so an axis-aligned
    // inside-test is not testing the box. addDynamicSphere calls
    // groundedCenter(centre, radius), which moves the sphere to
    // z = terrain_base + radius — so the collision volume is NOT where it was
    // asked for, and a distance measured to the requested centre says
    // nothing. Both errors reported penetration that was not happening.
    // A sphere has no yaw and its grounded centre is computable here, so
    // this measures against the volume that actually exists.
    const auto head = [&]() {
      return mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos,
                    start_vel, start_acc);
    };
    expect(pm->planGlobalTraj(head(), {goal[0]}), "warm-up builds the SDF");

    const auto grounded = [&](const Eigen::Vector3d &c, double r) {
      double g = 0.0;
      pm->terrainElevation(c.x(), c.y(), &g);
      return Eigen::Vector3d(c.x(), c.y(), (g > 0.0 ? g : 0.0) + r);
    };
    // SEGMENT clearance, not vertex clearance. Two vertices can both sit
    // outside a sphere while the chord between them passes through it, and a
    // vertex-only test calls that clear. Exact point-to-segment distance, so
    // no sampling pitch can hide a chord.
    const auto segDist = [](const Eigen::Vector3d &a, const Eigen::Vector3d &b,
                            const Eigen::Vector3d &c) {
      const Eigen::Vector3d ab = b - a;
      const double L2 = ab.squaredNorm();
      const double t = (L2 > 1e-12)
                           ? std::clamp((c - a).dot(ab) / L2, 0.0, 1.0)
                           : 0.0;
      return (a + t * ab - c).norm();
    };
    const auto clearance = [&](const std::vector<Eigen::Vector3d> &pts,
                               const std::vector<std::pair<Eigen::Vector3d,
                                                           double>> &obs) {
      double worst = 1e9;
      for (const auto &o : obs) {
        for (const auto &q : pts)
          worst = std::min(worst, (q - o.first).norm() - o.second);
        for (size_t i = 0; i + 1 < pts.size(); ++i)
          worst = std::min(worst, segDist(pts[i], pts[i + 1], o.first) -
                                      o.second);
      }
      return worst;
    };
    // Trajectory sampled to a SPATIAL pitch rather than a fixed count: a
    // fixed 4000 samples is a different resolution on a 30 u flight than on a
    // 300 u one, and the number that matters is how far apart the samples are
    // in metres.
    const auto trajClearance =
        [&](const std::vector<std::pair<Eigen::Vector3d, double>> &obs) {
      const poly_traj::Trajectory &fl = pm->traj_.local_traj.traj;
      if (fl.getPieceNum() == 0) return -1e9;
      const double T = fl.getTotalDuration();
      double len = 0.0;
      Eigen::Vector3d prev = fl.getPos(0.0);
      for (int k = 1; k <= 2000; ++k) {
        const Eigen::Vector3d p = fl.getPos(T * k / 2000.0);
        len += (p - prev).norm();
        prev = p;
      }
      const double pitch = 0.05;   // 5 m at 1 unit = 100 m
      const int n = std::clamp(static_cast<int>(std::ceil(len / pitch)),
                               2000, 400000);
      double worst = 1e9;
      Eigen::Vector3d a = fl.getPos(0.0);
      for (int k = 1; k <= n; ++k) {
        const Eigen::Vector3d b = fl.getPos(T * k / n);
        for (const auto &o : obs) {
          worst = std::min(worst, (b - o.first).norm() - o.second);
          worst = std::min(worst, segDist(a, b, o.first) - o.second);
        }
        a = b;
      }
      return worst;
    };

    // ONE sphere on the corridor: there is room around it, so the route must
    // go around it — not through, and not by refusing the mission.
    {
      const double R = 10.0;
      const Eigen::Vector3d ask(180.0, 150.0, 3.0);
      const std::vector<std::pair<Eigen::Vector3d, double>> obs{
          {grounded(ask, R), R}};
      expect(pm->addDynamicSphere(ask, R) >= 0, "the sphere is registered");
      const bool ok = pm->planGlobalTraj(head(), {goal[0]});
      const double rc = clearance(pm->lastCommittedRoute(), obs);
      const double tc = trajClearance(obs);
      std::cout << "[DYN] one sphere: plan=" << (ok ? 1 : 0)
                << " route=" << pm->lastCommittedRoute().size()
                << " route_clearance=" << rc << " traj_clearance=" << tc
                << "\n";
      expect(ok, "a mission with one avoidable obstacle still plans");
      expect(rc > 0.0, "...and NO committed route vertex is inside it");
      expect(tc > 0.0, "...nor any point of the flown trajectory");
      pm->clearDynamicObstacles();
    }

    // A GENUINELY SEALED corridor. Getting this right needs both traps
    // accounted for at once:
    //   - grounding lifts a sphere to z = base + radius, so at flight
    //     altitude its cross-section is only 2*sqrt(r-1) wide. Ten radius-40
    //     spheres spaced 40 apart look like a wall and leave ~22 u gaps at
    //     z = 3. That fixture was never sealed, and its "+1.02 clearance"
    //     result was the route walking through a gap.
    //   - addDynamicBox applies a random yaw, so no axis-aligned test is
    //     valid for a general box.
    // A box with EQUAL x and y extents defeats both: rotation about z leaves
    // the inscribed cylinder (radius = half the extent) invariant, so a point
    // within that radius in xy is inside the box whatever the yaw. Grounding
    // lifts it by half the z size, and a 200 u tall box then spans the entire
    // 31.8 u map column from the terrain up. Spaced 30 apart with a 20 u
    // inscribed radius, every y is within 15 of a centre — so ANY route point
    // at x = 180 is inside an obstacle, for any yaw and any grounding.
    {
      std::vector<Eigen::Vector3d> centres;
      for (double y = -20.0; y <= 320.0; y += 30.0) {
        pm->addDynamicBox(Eigen::Vector3d(180.0, y, 3.0),
                          Eigen::Vector3d(40.0, 40.0, 200.0));
        centres.emplace_back(180.0, y, 3.0);
      }
      const bool ok = pm->planGlobalTraj(head(), {goal[0]});
      const auto &rt = pm->lastCommittedRoute();
      int pen = 0; double worst = 1e9;
      for (const auto &q : rt) {
        double best = 1e9;
        for (const auto &c : centres)
          best = std::min(best, std::hypot(q.x() - c.x(), q.y() - c.y()));
        worst = std::min(worst, best);
        if (best < 20.0) ++pen;
      }
      int jpen = 0;
      const poly_traj::Trajectory &fl = pm->traj_.local_traj.traj;
      if (ok && fl.getPieceNum() > 0) {
        const double T = fl.getTotalDuration();
        for (int k = 0; k <= 4000; ++k) {
          const Eigen::Vector3d p = fl.getPos(T * k / 4000.0);
          double best = 1e9;
          for (const auto &c : centres)
            best = std::min(best, std::hypot(p.x() - c.x(), p.y() - c.y()));
          if (best < 20.0) ++jpen;
        }
      }
      std::cout << "[DYN] WALL(" << centres.size() << " cubes): plan="
                << (ok ? 1 : 0) << " route=" << rt.size()
                << " route_pts_inside=" << pen
                << " min_xy_to_axis=" << worst
                << " traj_pts_inside=" << jpen << "\n";
      expect(pen == 0,
             "no committed route vertex is inside the wall — a mission that "
             "cannot get through must FAIL, not be routed through");
      expect(jpen == 0, "...and no point of the flown trajectory either");
      pm->clearDynamicObstacles();
    }

    // A DENSE BARRIER ARRAY — deliberately NOT called sealed, because it is
    // not. Grounding lifts each radius-40 sphere to z = base + 40, so at
    // flight altitude the cross-sections are only 2*sqrt(r-1) wide and ~22 u
    // gaps remain between centres 40 apart. Before the route was validated
    // the planner threaded one of those gaps at +1.02, and reading that as
    // "sealed, and it got through cleanly" is what made me retract a real
    // defect. It is kept as the case that LOOKS impassable and is not.
    // Whatever comes back, it may not be a route through them.
    {
      std::vector<std::pair<Eigen::Vector3d, double>> obs;
      for (double y = -40.0; y <= 340.0; y += 40.0) {
        pm->addDynamicSphere(Eigen::Vector3d(180.0, y, 3.0), 40.0);
        obs.emplace_back(grounded(Eigen::Vector3d(180.0, y, 3.0), 40.0), 40.0);
      }
      const bool ok = pm->planGlobalTraj(head(), {goal[0]});
      const double rc = clearance(pm->lastCommittedRoute(), obs);
      const double tc = ok ? trajClearance(obs) : 1e9;
      std::cout << "[DYN] sealed(" << obs.size() << "): plan=" << (ok ? 1 : 0)
                << " route=" << pm->lastCommittedRoute().size()
                << " route_clearance=" << rc << " traj_clearance=" << tc
                << "\n";
      // Pinned, not either/or. Threading these gaps costs less than the
      // dynamic-obstacle berth the system asks for, so the validated front
      // end refuses — and an "either outcome is fine" assertion would have
      // let a silent return to threading pass as success.
      expect(!ok,
             "a barrier whose gaps cost more berth than dyn_obstacle_margin "
             "allows is REFUSED, not threaded");
      expect(pm->lastCommittedRoute().empty(),
             "...leaving no route behind");
      pm->clearDynamicObstacles();
    }
    rclcpp::shutdown();
    if (failures == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  if (with_legfail) {
    // [LEG-POLICY] A leg whose SEARCH fails leaves NOTHING behind.
    //
    // This is the case the capture reordering was written for, and until the
    // front end validated its own extracted route it could not be reached
    // from here at all: out-of-map waypoints clamp and reach the goal, an
    // obstacle clearance the empty SDF cannot satisfy was ignored, and a
    // dynamic wall was routed through rather than refused. With
    // [FM2-OCCUPANCY] in place the clearance demand below is honoured, the
    // geodesic is discarded, and the search genuinely fails — so the
    // fail-closed behaviour is finally observable instead of merely written
    // down.
    path_manager::RiskZone z;
    z.center = Eigen::Vector3d(600.0, 600.0, 3.0);   // off the route
    z.reach = 15.0;
    z.peak = 0.9;
    pm->setRiskZonesRuntime({z});
    const Eigen::Vector3d mid(180.0, 80.0, 3.0);
    const auto head = [&]() {
      return mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos,
                    start_vel, start_acc);
    };

    // A good two-leg plan first, so there IS something to be left standing.
    pm->setObstacleClearance(0.7);
    expect(pm->planGlobalTraj(head(), {mid, goal[0]}),
           "a two-leg mission plans");
    expect(pm->legPolicySnapshots().size() == 2 &&
               !pm->lastPieceLeg().empty() &&
               !pm->lastCommittedRouteEdgeLeg().empty(),
           "...leaving captures, a piece map and route tags behind it");

    pm->setObstacleClearance(500.0);   // nothing can satisfy this
    const bool ok = pm->planGlobalTraj(head(), {mid, goal[0]});
    std::cout << "[LEG-FAIL] plan=" << (ok ? 1 : 0)
              << " legs=" << pm->legPolicySnapshots().size()
              << " piece_leg=" << pm->lastPieceLeg().size()
              << " tags=" << pm->lastCommittedRouteEdgeLeg().size()
              << " route=" << pm->lastCommittedRoute().size() << "\n";
    expect(!ok, "a front end that cannot honour the clearance fails the plan");
    expect(pm->legPolicySnapshots().empty(),
           "and the EPOCH is void — not a prefix of the legs that ran before "
           "the failing one, and not the previous plan's two either");
    expect(pm->lastPieceLeg().empty() &&
               pm->lastCommittedRouteEdgeLeg().empty() &&
               pm->lastCommittedRoute().empty() &&
               pm->lastCommittedRouteEpoch() == 0,
           "...and no provenance or geometry survives from the plan before it");
    rclcpp::shutdown();
    if (failures == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  if (with_legseam) {
    using ZD = path_manager::PathManager::ZoneDisposition;
    // [LEG-POLICY] The REFUSING half of the handover rule, on a map that
    // production could actually have produced.
    //
    // Why it needs a synthesised flight at all: a seam is a mission waypoint,
    // i.e. the outgoing leg's goal AND the incoming leg's start, and the front
    // end grants a containment exemption on either (dyn_a_star.h
    // prepareBarrier, `if (s_in || g_in)`). So any zone covering a REAL seam
    // exempts both adjoining legs and the two sides can never disagree there.
    // Planning cannot produce the case; the evaluator still has to get it
    // right.
    //
    // The first version of this handed the audit {1, 0} — leg 1 before leg 0 —
    // to force the merge to reach backwards. That map is not one any route can
    // produce, and building the only test of the refusal on an input the
    // contract should reject was the wrong way round. The zone goes on the
    // MISSION GOAL instead: leg 0 has neither endpoint inside so it stays
    // HARD_AVOID, leg 1 ends there so it is SOFT_ENDPOINT, and the natural
    // ascending map {0, 1} then puts the SOFT side on the piece the seam sits
    // on — the merge still has to reach back to the previous leg for the
    // HARD_AVOID, and deleting it still reads soft.
    path_manager::RiskZone z;
    z.center = goal[0];
    z.reach = 20.0;
    z.peak = 0.9;
    pm->setRiskZonesRuntime({z});
    const Eigen::Vector3d mid(180.0, 80.0, 3.0);
    expect(pm->planGlobalTraj(
               mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos,
                      start_vel, start_acc),
               {mid, goal[0]}),
           "the two-leg mission plans, capturing both legs");
    const auto &legs = pm->legPolicySnapshots();
    expect(legs.size() == 2 && !legs[0].policy.zones.empty() &&
               !legs[1].policy.zones.empty(),
           "both legs captured a policy for the zone");
    if (legs.size() != 2 || legs[0].policy.zones.empty() ||
        legs[1].policy.zones.empty()) {
      std::cout << "FAIL: " << failures << " failed check(s)\n";
      return 1;
    }
    std::cout << "[LEG-SEAM] dispositions leg0="
              << (int)legs[0].policy.zones[0].disposition << " leg1="
              << (int)legs[1].policy.zones[0].disposition << "\n";
    expect(legs[0].policy.zones[0].disposition == ZD::HARD_AVOID &&
               legs[1].policy.zones[0].disposition == ZD::SOFT_ENDPOINT,
           "leg 0 must avoid the zone; leg 1 is told to arrive in it");

    // Straight and level THROUGH the zone centre with the piece seam on it.
    // 25 u per 16 s piece is 1.5625 u/s = 156 m/s — inside the cruise band, so
    // no_cruise cannot stand in for the cause under test.
    const Eigen::Vector3d a(z.center.x() - 25.0, z.center.y(), z.center.z());
    const Eigen::Vector3d b(z.center.x() + 25.0, z.center.y(), z.center.z());
    const auto seg = [&](const Eigen::Vector3d &p0, const Eigen::Vector3d &p1) {
      Eigen::Matrix<double, 3, 6> c = Eigen::Matrix<double, 3, 6>::Zero();
      c.col(5) = p0;
      c.col(4) = (p1 - p0) / 16.0;
      return c;
    };
    const poly_traj::Trajectory seam(
        std::vector<double>{16.0, 16.0},
        std::vector<poly_traj::CoefficientMat>{seg(a, z.center),
                                               seg(z.center, b)});
    const std::vector<size_t> map = {0, 1};   // flight order, contiguous
    const uint64_t ep = pm->lastCommittedRouteEpoch();
    const auto fv = chain.evaluateFlightForTest(
        seam, {{seam.getTotalDuration(),
                path_manager::SegmentChainPlanner::PhaseKind::CRUISE,
                "synthetic"}},
        &map, ep);
    std::cout << "[LEG-SEAM] hard=" << fv.zone_hard_n
              << " junctions=" << fv.zone_junction_n
              << " junction_hard=" << fv.zone_junction_hard_n
              << " measurable=" << (fv.policy_measurable ? 1 : 0)
              << " no_cruise=" << (fv.no_cruise ? 1 : 0) << "\n";
    expect(fv.evaluated, "the synthetic flight was evaluated");
    expect(fv.policy_measurable, "...with per-piece attribution engaged");
    expect(!fv.no_cruise && !fv.underground,
           "...and no other unflyable cause standing");
    expect(fv.zone_junction_n == 1, "the one handover was examined");
    expect(fv.zone_junction_hard_n == 1,
           "and it is inside the authored volume under the STRICTER of the "
           "two policies — the piece it sits on says SOFT_ENDPOINT, so the "
           "merge has to reach BACK to the previous leg to find HARD_AVOID");
    expect(fv.unflyable(), "which refuses the flight");
    const path_manager::PlanResult pr = chain.verdictResultForTest(fv);
    expect(!pr.hasTrajectory(),
           "the refusal reaches the caller, not just the counter");
    expect(pr.detail.find("leg handover") != std::string::npos,
           "...and it SAYS which hazard, instead of an empty parenthesis");

    // The provenance CONTRACT, on the same fixture. Every one of these is a
    // map that a size-and-range check waves through and that no route can
    // produce; each must switch attribution off rather than be interpreted.
    const auto rejects = [&](const std::vector<size_t> &m, uint64_t e,
                             const char *what) {
      const auto r = chain.evaluateFlightForTest(
          seam, {{seam.getTotalDuration(),
                  path_manager::SegmentChainPlanner::PhaseKind::CRUISE,
                  "synthetic"}},
          &m, e);
      expect(!r.policy_measurable, what);
    };
    rejects({1, 0}, ep, "a map that runs BACKWARDS is refused, not read");
    rejects({0, 2}, ep, "a map that SKIPS a leg is refused");
    rejects({0, 1}, ep + 1,
            "a map stamped with another epoch is refused — its indices name "
            "legs from a different search");
    rejects({0, 1}, 0, "an unstamped map is refused");
    rclcpp::shutdown();
    if (failures == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  if (with_transwp) {
    // [LEG-POLICY] The scope contract for transition + multi-waypoint.
    //
    // plan() dispatches TRANSITION_REQUIRED at :426, BEFORE planImpl's
    // single-goal guard, so the coordinator really does see the full
    // waypoint list — the earlier claim that multi-leg missions cannot
    // reach a stitched flight was simply wrong, and this variant is the
    // measurement that settles it.
    const Eigen::Vector3d v32(1.6, 0.0, 1.0);  // 188.7 m/s at 32 deg
    std::vector<Eigen::Vector3d> legs;
    legs.push_back(0.5 * (start_pos + goal[0]));
    legs.push_back(goal[0]);
    const auto fly = [&]() {
      return chain.plan(mkHead(path_manager::StartStateSource::STATED_VECTOR,
                               start_pos, v32, start_acc, false), legs, {});
    };

    // ZONE-FREE: supported, and it flies. Nothing to attribute means the
    // plan-wide snapshot is valid and is the right answer, so refusing this
    // would be removing working behaviour to make a comment true.
    const path_manager::PlanResult r0 = fly();
    std::cout << "[TRANS-WP] no zones: outcome=" << (int)r0.outcome
              << " reason=" << (int)r0.reason
              << " legs=" << pm->legPolicySnapshots().size()
              << " pieces=" << pm->traj_.local_traj.traj.getPieceNum() << "\n";
    expect(r0.hasTrajectory(),
           "a multi-waypoint TRANSITION mission with no zones flies — it "
           "reaches the stitched path that planImpl's guard hides");
    expect(pm->legPolicySnapshots().size() == 2,
           "...over a genuinely multi-leg committed route");

    // WITH A ZONE: refused, and refused for the RIGHT reason. The
    // coordinator screens its entry against one plan-wide policy and a
    // multi-leg front end has none; the stitched product has no
    // piece-to-leg map either, so nothing downstream could attribute a
    // contact. That is a scope statement, and it used to be reported as
    // TRANSITION_GENERATION_FAILED — a stage that had not run.
    path_manager::RiskZone z;
    z.center = Eigen::Vector3d(600.0, 600.0, 3.0);   // far off the route
    z.reach = 20.0;
    z.peak = 0.9;
    pm->setRiskZonesRuntime({z});
    const path_manager::PlanResult r1 = fly();
    std::cout << "[TRANS-WP] with zone: outcome=" << (int)r1.outcome
              << " reason=" << (int)r1.reason << " detail=" << r1.detail
              << "\n";
    expect(!r1.hasTrajectory(), "add a zone and the same mission is refused");
    expect(r1.reason ==
               path_manager::PlanReason::TRANSITION_MULTI_LEG_UNSUPPORTED,
           "...as an explicit SCOPE refusal, not as a generation failure");
    expect(r1.detail.find("not supported") != std::string::npos,
           "...and the detail says so in words a caller can act on");

    // ...and it is decided BEFORE anything is attempted. Not "the reason
    // looked right on one input" — the front-end epoch is incremented on
    // entry to planFrontEnd, once per run, so an unchanged epoch across the
    // call is direct evidence that no search was started at all. While the
    // check sat after commitRoute a route was committed first, and any
    // front-end failure then displaced the scope answer with a
    // geometry-shaped one.
    const uint64_t ep_before = pm->zonePolicyEpoch();
    const path_manager::PlanResult r2 = fly();
    const uint64_t ep_after = pm->zonePolicyEpoch();
    std::cout << "[TRANS-WP] epoch " << ep_before << " -> " << ep_after
              << " reason=" << (int)r2.reason << "\n";
    expect(!r2.hasTrajectory() &&
               r2.reason ==
                   path_manager::PlanReason::TRANSITION_MULTI_LEG_UNSUPPORTED,
           "the refusal repeats");
    expect(ep_after == ep_before,
           "...and NO front end ran for it — the policy epoch is untouched, "
           "which is what 'decided before anything is attempted' means");
    rclcpp::shutdown();
    if (failures == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  if (with_cutarc) {
    // [LEG-POLICY] cutAtArc's tag rule, pinned as a PURE FUNCTION. Its only
    // production consumer is the transition path, which does not read the
    // tags yet, so an off-by-one in the suffix would have sat here unnoticed
    // — a mutation was tried against the transition variants and neither
    // died. Nothing about that rule needs a mission to check it.
    //
    // A straight 4-vertex route, 3 edges, arc lengths 0/10/20/30, with the
    // last edge on a different leg so a shifted suffix is visible in the
    // VALUES and not only in the length.
    const std::vector<Eigen::Vector3d> route = {
        {0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {20.0, 0.0, 0.0}, {30.0, 0.0, 0.0}};
    const std::vector<double> cap = {1.0, 2.0, 3.0, 4.0};
    const std::vector<size_t> tags = {0, 0, 1};   // edge i -> leg
    std::vector<Eigen::Vector3d> out;
    std::vector<double> ocap;
    std::vector<size_t> oleg;
    const auto cut = [&](double s_cut, const std::vector<size_t> &in) {
      out.clear(); ocap.clear(); oleg.clear();
      return chain.cutAtArc(route, cap, s_cut, &out, &ocap, in, &oleg);
    };
    const auto shows = [&](const std::vector<size_t> &want) {
      if (oleg.size() != want.size()) return false;
      for (size_t i = 0; i < want.size(); ++i)
        if (oleg[i] != want[i]) return false;
      return true;
    };

    // (1) inside an edge: the cut splits edge 0 and BOTH halves keep it, so
    // the surviving tail still starts on edge 0.
    expect(cut(5.0, tags), "a cut inside the first edge succeeds");
    expect(out.size() == 4 && oleg.size() + 1 == out.size(),
           "...one tag per surviving edge");
    expect(shows({0, 0, 1}), "...and the tail is the whole tag list");

    // (2) exactly on a vertex: nothing of edge 0 survives, so the tail starts
    // on edge 1. This is the boundary an off-by-one lands on either side of.
    expect(cut(10.0, tags), "a cut exactly on a vertex succeeds");
    expect(out.size() == 3 && shows({0, 1}),
           "...and the tail starts on the edge LEAVING that vertex");

    // (3) the coincident branch: the cut lands within the dedup tolerance of
    // the next vertex, so the code advances past the edge it consumed.
    expect(cut(19.9995, tags), "a cut a hair short of a vertex succeeds");
    expect(out.size() == 2 && shows({1}),
           "...and lands on the same tail as landing on the vertex would");

    // (4) near the end: one edge left, and it is the last one.
    expect(cut(29.0, tags), "a cut inside the last edge succeeds");
    expect(out.size() == 2 && shows({1}), "...leaving only that edge's tag");

    // (5) tags that do not describe this route are refused whole. Not
    // truncated, not padded — a wrong-length tag list is not evidence.
    expect(cut(5.0, {0, 0}), "a cut with a short tag list still cuts");
    expect(oleg.empty(), "...but publishes NO provenance for it");

    // (6) past the end there is nothing to chain over.
    expect(!cut(30.0, tags), "a cut at or past the total arc length fails");
    rclcpp::shutdown();
    if (failures == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  if (with_legmid) {
    // [LEG-POLICY] The optimizer's OTHER clean_path insertion. A route short
    // enough not to be subdivided arrives as two vertices, which is one MINCO
    // piece — too few — so a midpoint is planted, and the tag vector has to be
    // grown with it exactly as the lead-in case does. Nothing in the shipped
    // configuration reaches this: it needs a mission short enough that the
    // whole route is one undivided edge.
    const Eigen::Vector3d near_goal = start_pos + Eigen::Vector3d(20.0, 0.0, 0.0);
    const bool ok = pm->planGlobalTraj(
        mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos,
               start_vel, start_acc),
        {near_goal});
    expect(ok, "the short mission plans");
    const poly_traj::Trajectory &flight = pm->traj_.local_traj.traj;
    const auto &route = pm->lastCommittedRoute();
    const auto &pl = pm->lastPieceLeg();
    std::cout << "[LEG-MID] route=" << route.size()
              << " pieces=" << flight.getPieceNum()
              << " piece_leg=" << pl.size() << "\n";
    expect(route.size() == 2,
           "the route really is a single undivided edge — the premise");
    expect(flight.getPieceNum() == 2,
           "...so a midpoint was planted and the trajectory has two pieces");
    // Not just "the sizes agree": 0 == 0 agrees too, and the first draft of
    // this variant reported that as a pass while the mission was not flying
    // at all.
    expect(!pl.empty() && pl.size() == static_cast<size_t>(flight.getPieceNum()),
           "the tag vector grew with it");
    if (pl.size() == 2)
      expect(pl[0] == 0 && pl[1] == 0,
             "...both halves of the split edge on the one leg there is");

    // A REFUSED plan must not leave this map standing. The entry gate returns
    // before anything is written, so without a reset at the top the accessor
    // would still describe the plan above — and both halves would be
    // individually well-formed, so no size check could catch the mismatch.
    // This assertion belongs HERE, where a successful plan has just left a
    // NON-EMPTY map: asserting it after a plan that legitimately produced
    // none passes without testing anything.
    expect(!pm->lastPieceLeg().empty(), "the map above is non-empty — the "
                                        "premise for the check below");
    expect(!pm->planGlobalTraj(path_manager::StartHead{}, {near_goal}),
           "an UNSPECIFIED head is refused at the entry gate");
    expect(pm->lastPieceLeg().empty(),
           "...and the refused call leaves NO piece map from the plan before "
           "it");
    rclcpp::shutdown();
    if (failures == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  if (with_legaudit) {
    // [LEG-POLICY] The payoff, and the case that made the whole thing
    // necessary. The zone sits on the MISSION START, so leg 0 owns it as a
    // SOFT_ENDPOINT containment exemption while leg 1 — which neither starts
    // nor ends there — calls the same zone HARD_AVOID. The flight
    // NECESSARILY begins inside the authored volume, so the early samples are
    // a contact no matter what the optimizer does; the only question is which
    // leg's policy judges them.
    //
    // Two failures this pins at once. Reading the LAST leg's policy for the
    // whole flight refuses a start the mission itself authored. Refusing to
    // read any policy — what a plan-wide-or-nothing snapshot forces on every
    // multi-leg mission — refuses it too, just with a different sentence. It
    // must fly, and it must fly MEASURED: manager/allow_unmeasured_zone_policy
    // is left at its shipped false, so nothing here is excused rather than
    // checked.
    const Eigen::Vector3d mid(180.0, 80.0, 3.0);
    path_manager::RiskZone z;
    z.center = start_pos;
    z.reach = 20.0;
    z.peak = 0.9;
    pm->setRiskZonesRuntime({z});

    const path_manager::PlanResult r = chain.plan(
        mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos,
               start_vel, start_acc, false),
        {mid, goal[0]}, {});
    std::cout << "[LEG-AUDIT] outcome=" << (int)r.outcome
              << " reason=" << (int)r.reason
              << " legs=" << pm->legPolicySnapshots().size()
              << " pieces=" << pm->lastPieceLeg().size()
              << " detail=" << r.detail << "\n";
    expect(r.hasTrajectory(),
           "a multi-leg mission starting inside a zone IT authored flies");
    expect(r.reason != path_manager::PlanReason::ZONE_POLICY_UNEVALUATED,
           "...and it was measured, not excused: the policy was evaluated");
    expect(r.detail.find("allow_unmeasured_zone_policy") == std::string::npos,
           "...without the unmeasured-policy escape being involved at all");

    // The leg handover is audited EXHAUSTIVELY, off the 0.1 s grid. A seam is
    // one instant: the sampler lands on it only by coincidence, so the times
    // have to be enumerated from the piece durations instead of sampled for.
    // The first version of this did neither — it tried to detect seams inside
    // the sampler using a locatePieceIdx reading that never occurs (that
    // function advances only while t > duration, so AT a seam it returns the
    // previous piece with the remainder equal to that piece's whole duration,
    // not the next piece at remainder zero). The branch was dead.
    const auto &fv = chain.lastFlightVerdict();
    const auto &pl = pm->lastPieceLeg();
    size_t changes = 0;
    for (size_t i = 1; i < pl.size(); ++i)
      if (pl[i] != pl[i - 1]) ++changes;
    std::cout << "[LEG-AUDIT] leg changes=" << changes
              << " junctions audited=" << fv.zone_junction_n
              << " junction_hard=" << fv.zone_junction_hard_n << "\n";
    expect(changes >= 1, "the flight really does change legs");
    expect(fv.zone_junction_n == static_cast<int>(changes),
           "...and EVERY handover was examined, not the ones a 0.1 s grid "
           "happened to hit");
    expect(fv.zone_junction_hard_n == 0,
           "this flight hands over clear of any authored volume");

    const auto &legs = pm->legPolicySnapshots();
    expect(legs.size() == 2, "both legs were captured");
    if (legs.size() == 2 && !legs[0].policy.zones.empty() &&
        !legs[1].policy.zones.empty()) {
      using ZD = path_manager::PathManager::ZoneDisposition;
      expect(legs[0].policy.zones[0].disposition == ZD::SOFT_ENDPOINT,
             "leg 0 starts in the zone -> SOFT_ENDPOINT");
      expect(legs[1].policy.zones[0].disposition == ZD::HARD_AVOID,
             "leg 1 does not touch it -> HARD_AVOID, and judging leg 0 by "
             "THAT is what refuses an authored start");
    }
    rclcpp::shutdown();
    if (failures == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  if (with_legchain) {
    // [LEG-POLICY] Chaining and multi-leg are orthogonal: planImpl chains
    // SINGLE-GOAL missions only and hands anything with more waypoints to the
    // single-shot planner (its own comment says so). So a chained mission has
    // exactly one leg — and the point here is not attribution but that the
    // provenance survives the segment re-solves at all. Those go through the
    // INHERITED-ROUTE branch, which runs no search and so mints no tags of
    // its own; without the slice carrying them, a chained plan would end with
    // no attribution and the manager's capture cleared under the fail-closed
    // rule.
    path_manager::RiskZone z;
    z.center = Eigen::Vector3d(30.0, 30.0, 3.0);   // off the corridor
    z.reach = 15.0;
    z.peak = 0.9;
    pm->setRiskZonesRuntime({z});

    const path_manager::PlanResult r = chain.plan(
        mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos,
               start_vel, start_acc, false),
        goal, {});
    expect(r.hasTrajectory(), "the chained single-goal mission flies");

    const auto &legs = pm->legPolicySnapshots();
    const auto &pl = pm->lastPieceLeg();
    const auto &slice = pm->lastCommittedRoute();
    std::cout << "[LEG-CHAIN] legs=" << legs.size() << " piece_leg=" << pl.size()
              << " slice=" << slice.size()
              << " epoch=" << pm->lastCommittedRouteEpoch() << "\n";
    // ONE front-end epoch for the whole chained mission. A segment that fell
    // back to its own span search would have run the front end again and
    // pushed this up, which would mean the assertions below were describing
    // a re-search rather than an inherited slice.
    expect(pm->lastCommittedRouteEpoch() == 1,
           "every segment re-solved the SAME committed route — no segment ran "
           "its own front end");
    expect(legs.size() == 1,
           "a single-goal mission has one leg, captured once");
    expect(!pm->lastCommittedRouteEdgeLeg().empty() &&
               pm->lastCommittedRouteEdgeLeg().size() + 1 == slice.size(),
           "the retained slice arrived with its own tags");
    expect(!pl.empty() && pl.size() + 1 == slice.size(),
           "...so the segment's pieces are attributed");
    if (!pl.empty()) {
      bool all_leg0 = true;
      for (size_t leg : pl) if (leg != 0) all_leg0 = false;
      expect(all_leg0, "...to the only leg there is");
    }
    rclcpp::shutdown();
    if (failures == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  if (with_legtags || with_legleadin) {
    // [LEG-POLICY] step 2: edge provenance survives the two transforms that
    // rewrite the route's point list after the search — corner fillets and
    // the piece-boundary subdivision. Tags are carried through both; nothing
    // is re-derived by projecting the final geometry back onto the legs.
    //
    // A corner at `mid` guarantees the fillet actually fires at the leg
    // junction, which is the one place a tag can be attributed to the wrong
    // leg: the fillet DELETES the junction vertex and replaces it with an arc
    // belonging to neither leg alone.
    // Several corner geometries, not one. The arc's sample count is
    // ceil(|P1-P2| / 3), so the corner angle decides whether it comes out odd
    // or even — and the even-N forcing that puts the split exactly on the
    // apex is invisible at a corner that was already even. One `mid` tested
    // the split's existence but not its placement.
    const std::vector<Eigen::Vector3d> mids =
        with_legleadin
            ? std::vector<Eigen::Vector3d>{{180.0, 80.0, 3.0}}
            : std::vector<Eigen::Vector3d>{
                  {180.0, 80.0, 3.0},   {180.0, 40.0, 3.0},
                  {150.0, 100.0, 3.0},  {200.0, 60.0, 4.0},
              };
    for (const auto &mid : mids) {
      const std::string at_mid =
          " [mid " + std::to_string((int)mid.x()) + "," +
          std::to_string((int)mid.y()) + "]";
      const bool ok = pm->planGlobalTraj(
          mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos,
                 start_vel, start_acc),
          {mid, goal[0]});
      expect(ok, "the two-leg mission plans" + at_mid);
      if (!ok) continue;

      const auto &route = pm->lastCommittedRoute();
      const auto &tags = pm->lastCommittedRouteEdgeLeg();
      expect(!tags.empty() && tags.size() + 1 == route.size(),
             "every route EDGE carries a leg, so tags are one shorter than "
             "vertices" + at_mid);
      expect(pm->lastCommittedRouteEpoch() != 0,
             "...stamped with the epoch they index into" + at_mid);
      if (tags.empty() || tags.size() + 1 != route.size()) continue;

      size_t transitions = 0, at = 0;
      bool monotone = true;
      for (size_t i = 0; i + 1 < tags.size(); ++i) {
        if (tags[i + 1] < tags[i]) monotone = false;
        if (tags[i + 1] != tags[i]) { ++transitions; at = i + 1; }
      }
      expect(monotone,
             "legs appear in flight order along the route (never backwards)" +
                 at_mid);
      expect(tags.front() == 0, "the route leaves on leg 0" + at_mid);
      expect(tags.back() == 1, "and arrives on leg 1" + at_mid);
      expect(transitions == 1,
             "a two-leg route changes hands exactly once — no leg is dropped "
             "and none reappears" + at_mid);

      // Where it changes hands. The junction vertex is `mid` itself when no
      // fillet is placed, and the arc apex when one is — either way it is the
      // route vertex CLOSEST to mid, because the arc is symmetric about the
      // corner and the apex is its nearest point to it. An off-by-one in the
      // subdivision, an arc attributed wholly to one side, or a split that
      // misses the apex all move the handover off that vertex, and none of
      // them needs the test to know the fillet radius or the edge lengths.
      size_t closest = 0;
      double best = std::numeric_limits<double>::max();
      for (size_t i = 0; i < route.size(); ++i) {
        const double d = (route[i] - mid).norm();
        if (d < best) { best = d; closest = i; }
      }
      std::cout << "[LEG-TAGS]" << at_mid << " route=" << route.size()
                << " epoch=" << pm->lastCommittedRouteEpoch()
                << " hands over at " << at << " (|.-mid|="
                << (route[at] - mid).norm() << "), nearest is " << closest
                << " (" << best << ")\n";
      expect(at == closest,
             "the handover sits on the vertex nearest the junction "
             "waypoint" + at_mid);

      // ...and the same ownership after the solve. Route edge i becomes MINCO
      // piece i, so the map is only usable if it is exactly as long as the
      // trajectory that actually came out — the optimizer withholds it
      // otherwise. This is what turns a flight time into a policy: find the
      // piece, read its leg.
      const poly_traj::Trajectory &flight = pm->traj_.local_traj.traj;
      const auto &pl = pm->lastPieceLeg();
      if (with_legleadin) {
        // The lead-in plants an extra vertex just past the start, so the
        // trajectory carries one piece MORE than the route had edges. If the
        // tag vector is not grown alongside it, every tag from the start
        // onwards describes the wrong piece — and the size check makes the
        // optimizer withhold the map rather than publish the shift.
        expect(flight.getPieceNum() == static_cast<int>(route.size()),
               "the lead-in added a piece" + at_mid);
        expect(pl.size() >= 2 && pl[0] == pl[1] && pl[0] == 0,
               "...and both halves of the split first edge stay on leg 0" +
                   at_mid);
      } else {
        expect(flight.getPieceNum() ==
                   static_cast<int>(route.size()) - 1,
               "no insertion: one piece per route edge" + at_mid);
      }
      expect(!pl.empty() &&
                 pl.size() == static_cast<size_t>(flight.getPieceNum()),
             "one leg per MINCO piece of the trajectory that was stored" +
                 at_mid);
      if (!pl.empty() &&
          pl.size() == static_cast<size_t>(flight.getPieceNum())) {
        size_t p_trans = 0, p_count = 0;
        bool p_monotone = true;
        for (size_t i = 0; i + 1 < pl.size(); ++i) {
          if (pl[i + 1] < pl[i]) p_monotone = false;
          if (pl[i + 1] != pl[i]) { ++p_count; p_trans = i + 1; }
        }
        expect(p_monotone && p_count == 1 && pl.front() == 0 &&
                   pl.back() == 1,
               "the pieces change hands once, in flight order" + at_mid);
        // The piece boundary is where the flight passes the junction. Walk
        // the trajectory and find the piece holding the closest approach to
        // mid; it must be one of the two the handover separates. (Not an
        // exact index: the optimized curve rounds the corner, so the nearest
        // approach can fall either side of the boundary — but never pieces
        // away from it, which is what a shifted map would produce.)
        const double T = flight.getTotalDuration();
        double best_t = 0.0, best_d = std::numeric_limits<double>::max();
        for (int k = 0; k <= 2000; ++k) {
          const double t = T * k / 2000.0;
          const double d = (flight.getPos(t) - mid).norm();
          if (d < best_d) { best_d = d; best_t = t; }
        }
        double rem = best_t;
        const int pi = flight.locatePieceIdx(rem);
        std::cout << "[LEG-TAGS]" << at_mid << " pieces=" << pl.size()
                  << " hand over between " << (p_trans - 1) << " and "
                  << p_trans << "; closest approach to mid in piece " << pi
                  << " (" << best_d << ")\n";
        expect(std::abs(pi - static_cast<int>(p_trans)) <= 1,
               "the piece flying nearest the junction is the one the pieces "
               "change hands at" + at_mid);
      }
    }
    const Eigen::Vector3d mid = mids.front();
    const auto &route = pm->lastCommittedRoute();

    // The inherited-route branch runs no search at all, so it never reaches
    // the epoch reset that clears the per-leg captures. Before this was
    // handled, a chained re-solve over a supplied route read the PREVIOUS
    // plan's legs as though they described this one.
    const auto inherited = route;   // by value: the call overwrites route
    const auto inherited_cap = pm->lastCommittedCapRef();
    const auto inherited_tags = pm->lastCommittedRouteEdgeLeg();
    const uint64_t good_epoch = pm->lastCommittedRouteEpoch();
    const auto resolve = [&](const path_manager::PathManager::RouteProvenance
                                 *prov) {
      return pm->planGlobalTraj(
          mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos,
                 start_vel, start_acc),
          {mid, goal[0]}, ego_planner::TailBoundary{}, false, &inherited,
          inherited_cap.size() == inherited.size() ? &inherited_cap : nullptr,
          false, prov);
    };

    // Order matters: each refusal clears the captures, so the accepting case
    // has to run while there is still something to keep.
    path_manager::PathManager::RouteProvenance good{&inherited_tags,
                                                    good_epoch};
    expect(resolve(&good), "the same route re-solves as an inherited route");
    expect(pm->lastCommittedRouteEdgeLeg().size() == inherited_tags.size(),
           "provenance minted in the CURRENT epoch is accepted with it");
    expect(pm->legPolicySnapshots().size() == 2,
           "...and the per-leg captures it indexes are kept");

    // The epoch is the whole guard: same tags, one epoch stale. Nothing about
    // the tag vector itself says it describes a different search.
    path_manager::PathManager::RouteProvenance stale{&inherited_tags,
                                                     good_epoch + 1};
    expect(resolve(&stale), "a stale-epoch re-solve still plans");
    expect(pm->lastCommittedRouteEdgeLeg().empty(),
           "...but its tags are refused: they name legs from another search");
    expect(pm->legPolicySnapshots().empty(),
           "...and the captures they would have indexed are dropped with them");

    expect(resolve(nullptr), "a re-solve with no provenance still plans");
    expect(pm->lastCommittedRouteEdgeLeg().empty(),
           "a route that arrived from outside has NO provenance — not leg 0 "
           "by default");

    // A plan that is REFUSED at the entry gate must not leave the previous
    // plan's products standing behind the accessors. Those returns fire
    // before anything is written, so without a reset at the top the audit
    // would size a stale map against a trajectory it never came from — and
    // both halves would be individually well-formed, so no size check could
    // catch it. A default-constructed head is UNSPECIFIED, which is the
    // cheapest of those refusals to reach.
    expect(!pm->planGlobalTraj(path_manager::StartHead{}, {goal[0]}),
           "an UNSPECIFIED head is refused at the entry gate");
    expect(pm->lastPieceLeg().empty() &&
               pm->lastCommittedRouteEdgeLeg().empty() &&
               pm->lastCommittedRouteEpoch() == 0,
           "...and the refused call leaves NO provenance from the plan before "
           "it");
    expect(pm->lastCommittedRoute().empty() &&
               pm->lastCommittedCapRef().empty(),
           "...nor the committed route it never produced");
    rclcpp::shutdown();
    if (failures == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  if (with_headsrc) {
    // The PLUMBING, not the pure function. The policy matrix in
    // start_claim_test asserts applyHeadPolicy on hand-built objects and
    // passed while production re-derived the source from a boolean at the
    // far end of the call chain — TRAJECTORY_DERIVED arrived as
    // CHAIN_JUNCTION and silently lost its floor correction. These drive
    // the real entry points.
    const double floor_u = pm->cruiseFloorUnits();
    expect(floor_u > 0.0, "the cruise floor is available");
    const Eigen::Vector3d slow =
        Eigen::Vector3d(1.0, 0.0, 0.0) * (0.5 * floor_u);

    // 1. In ROUTE mode a TRAJECTORY_DERIVED head below the floor IS raised.
    //    This is the case the re-derivation broke: it is our own product,
    //    so it may be repaired, and the repaired plan must fly.
    const path_manager::PlanResult rd =
        chain.plan(mkHead(path_manager::StartStateSource::TRAJECTORY_DERIVED,
                          start_pos, slow, start_acc),
                   goal, {});
    expect(rd.hasTrajectory(),
           "a TRAJECTORY_DERIVED head below the floor flies");
    // The DECISION, and that it was reached at all. "!floored" alone is
    // ambiguous between "decided not to" and "never ran".
    expect(chain.lastHeadPolicy().evaluated,
           "...the head policy actually RAN");
    expect(chain.lastHeadPolicy().source ==
               path_manager::StartStateSource::TRAJECTORY_DERIVED,
           "...on the source it was given, not a re-derived one");
    expect(chain.lastHeadPolicy().floored,
           "...and the floor correction was APPLIED");

    // 2. The SAME state stated by an operator is refused, never raised.
    const path_manager::PlanResult sv =
        chain.plan(mkHead(path_manager::StartStateSource::STATED_VECTOR,
                          start_pos, slow, start_acc),
                   goal, {});
    expect(!sv.hasTrajectory(),
           "the same speed STATED is refused, not raised");
    // Which refusal depends on where the speed sits relative to the
    // transition model — 0.5x the floor is above the activation speed, so it
    // classifies TRANSITION_REQUIRED and the coordinator answers. Pinning a
    // particular reason here would pin that unrelated fact. The contract is
    // the ASYMMETRY: the identical state is repaired when we authored it and
    // refused when an operator stated it.
    expect(rd.hasTrajectory() && !sv.hasTrajectory(),
           "...and that asymmetry is the contract: same state, repaired as "
           "ours, refused as theirs");
    // NOT "!floored" — that state is refused by the classifier before the
    // head policy is ever reached, so !floored would pass because nothing
    // ran. The honest assertion is that nothing ran.
    expect(!chain.lastHeadPolicy().evaluated,
           "...and it was refused UPSTREAM: the head policy never ran");

    // A stated vector INSIDE the cruise band does reach the policy, and is
    // left alone there. This is the row that pins "stated is not touched".
    const Eigen::Vector3d cruising =
        Eigen::Vector3d(0.0, 1.0, 0.0) * (1.05 * floor_u);
    const path_manager::PlanResult svc =
        chain.plan(mkHead(path_manager::StartStateSource::STATED_VECTOR,
                          start_pos, cruising, start_acc),
                   goal, {});
    (void)svc;
    expect(chain.lastHeadPolicy().evaluated,
           "an in-band STATED_VECTOR reaches the head policy");
    expect(chain.lastHeadPolicy().source ==
               path_manager::StartStateSource::STATED_VECTOR,
           "...with its own source");
    expect(!chain.lastHeadPolicy().floored &&
               !chain.lastHeadPolicy().reaimed,
           "...and is neither floored nor re-aimed");

    // 3. A default-constructed head is refused rather than planned from.
    //    StartHead{} is UNSPECIFIED, so "required parameter" guarantees the
    //    CALL, not the VALUE.
    // Ordering matters here: the plan immediately above SUCCEEDED and left
    // evaluated=true behind. If resetPlanState() does not precede the
    // fail-closed entry checks, this refusal returns while the audit still
    // shows the PREVIOUS plan's answer — which is how a reader concludes a
    // refused mission had its head policy applied.
    expect(chain.lastHeadPolicy().evaluated,
           "the previous plan left an evaluated policy behind");
    const path_manager::PlanResult un =
        chain.plan(path_manager::StartHead{}, goal, {});
    expect(!un.hasTrajectory(), "an UNSPECIFIED head is refused");
    expect(un.reason == path_manager::PlanReason::INITIAL_STATE_UNSPECIFIED,
           "...as INITIAL_STATE_UNSPECIFIED");
    expect(!chain.lastHeadPolicy().evaluated,
           "...and the audit shows NOTHING RAN — not the previous plan's "
           "answer");
    expect(!chain.lastFlightVerdict().evaluated,
           "...the flight verdict is cleared too");

    // 4. A non-finite head is refused too.
    path_manager::StartHead nan_head =
        mkHead(path_manager::StartStateSource::STATED_VECTOR, start_pos,
               Eigen::Vector3d(std::nan(""), 0.0, 0.0), start_acc);
    const path_manager::PlanResult nn = chain.plan(nan_head, goal, {});
    expect(!nn.hasTrajectory(), "a non-finite head is refused");
    expect(nn.reason == path_manager::PlanReason::INITIAL_STATE_MALFORMED,
           "...as INITIAL_STATE_MALFORMED");
    // --- planOverRoute is PUBLIC and re-entrant ---------------------------
    // The audit state used to be cleared only in plan(), so a direct
    // re-call that returned at the input contract left the previous call's
    // answers readable.
    {
      std::vector<Eigen::Vector3d> route;
      std::vector<double> cap;
      double fe = 0.0;
      const auto ok_head =
          mkHead(path_manager::StartStateSource::TRAJECTORY_DERIVED,
                 start_pos, Eigen::Vector3d(1.8, 0.0, 0.0), start_acc);
      const bool committed =
          chain.commitRoute(ok_head, goal, false, &route, &cap, &fe);
      expect(committed, "a route commits for the re-entrancy check");
      if (committed) {
        chain.planOverRoute(route, cap, fe, ok_head, goal, false, {});
        expect(chain.lastHeadPolicy().evaluated,
               "a direct planOverRoute leaves an evaluated audit");

        // Now a call that dies at the input contract: a cap of the wrong
        // size. Everything must read NOT evaluated.
        std::vector<double> short_cap(cap.begin(), cap.end() - 1);
        const auto bad =
            chain.planOverRoute(route, short_cap, fe, ok_head, goal, false, {});
        expect(!bad.hasTrajectory(), "a mismatched cap is refused");
        expect(!chain.lastHeadPolicy().evaluated,
               "...and the head-policy audit is cleared, not stale");
        expect(!chain.lastFlightVerdict().evaluated,
               "...and so is the flight verdict");
        expect(chain.lastPhaseSpans().empty(),
               "...and so are the phase spans");

        // The prefix/source invariant must fire even on a route so short
        // that the auto-N fallback would otherwise return first. That
        // fallback only exists in AUTO mode, so the check has to run there
        // — with chain/segments fixed it never triggers and the ordering
        // this asserts is not exercised at all.
        // AUTO must be latched by a plan() call: planOverRoute never reads
        // the option (readSegmentsOption runs in plan()), so forcing the
        // parameter alone leaves auto_segments_ false and the fallback this
        // is meant to out-race never triggers.
        force("chain/segments", 0);
        chain.plan(ok_head, goal, {});       // latches auto_segments_
        std::vector<Eigen::Vector3d> tiny(route.begin(), route.begin() + 2);
        std::vector<double> tiny_cap(cap.begin(), cap.begin() + 2);
        const auto ho_head =
            mkHead(path_manager::StartStateSource::TRANSITION_HANDOFF,
                   start_pos, Eigen::Vector3d(1.8, 0.0, 0.0), start_acc,
                   true);
        const auto bad2 = chain.planOverRoute(tiny, tiny_cap, fe, ho_head,
                                              goal, false, {}, nullptr);
        expect(!bad2.hasTrajectory(),
               "a TRANSITION_HANDOFF head with NO prefix is refused");
        expect(bad2.reason ==
                   path_manager::PlanReason::TRANSITION_ADAPTER_UNSOUND,
               "...as TRANSITION_ADAPTER_UNSOUND, even on a route short "
               "enough that auto-N falls back first");
        force("chain/segments", 3);
      }
    }

    // --- the OTHER real policy call site ---------------------------------
    // PathManager::planGlobalTraj applies the same policy and has its own
    // audit; a regression reading only the chain's accessor pins only one
    // of the two.
    {
      const bool ok = pm->planGlobalTraj(
          mkHead(path_manager::StartStateSource::TRAJECTORY_DERIVED,
                 start_pos, Eigen::Vector3d(1.8, 0.0, 0.0), start_acc),
          goal);
      expect(ok, "a direct planGlobalTraj succeeds");
      expect(pm->lastHeadPolicy().evaluated,
             "...and PathManager's own audit says the policy ran");
      expect(pm->lastHeadPolicy().source ==
                 path_manager::StartStateSource::TRAJECTORY_DERIVED,
             "...on the source it was given");
      const bool bad = pm->planGlobalTraj(path_manager::StartHead{}, goal);
      expect(!bad, "an UNSPECIFIED head is refused there too");
      expect(!pm->lastHeadPolicy().evaluated,
             "...and that audit is cleared, not the previous call's answer");
    }

    rclcpp::shutdown();
    if (failures == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  if (with_capstart) {
    // A start speed stated EXACTLY at the planning cap must be CRUISE_VALID.
    // Eight of the ten shipped missions state exactly optimization/max_vel *
    // unit = 200.0 m/s, and the handoff ceiling comparison was a strict
    // vm > vmax with no tolerance — so those missions sat on the last bits
    // of the first-leg direction normalization (|dir| = 1 +- 1e-16 puts vm
    // at 200.0 +- 2e-14) and passed or classified TRANSITION_REQUIRED by
    // rounding. Nothing flipped that coin while the scalar form bypassed the
    // validator; unifying the two forms started flipping it, and
    // r5_extended_corridor came up tails in a live sweep.
    double mv = 2.0;
    if (node->has_parameter("optimization/max_vel"))
      node->get_parameter("optimization/max_vel", mv);
    const Eigen::Vector3d dir =
        Eigen::Vector3d(-0.0916, -0.9958, 0.0).normalized();
    const Eigen::Vector3d at_cap = dir * mv;

    // The VALIDATOR directly, because going through plan() only exercises
    // whichever way this particular direction happens to round — the first
    // attempt at this variant picked a direction that rounds DOWN, so it
    // passed with the strict comparison too and pinned nothing.
    //
    // The contract is: exactly at the cap passes, one ULP above the cap
    // still passes (that is arithmetic noise, not a faster aircraft), and a
    // real overspeed is still refused.
    expect(pm->stateEnvelopeProblem(at_cap).empty(),
           "a state exactly AT the handoff ceiling is accepted");
    const double one_ulp_over = std::nextafter(mv, 1e9);
    expect(pm->stateEnvelopeProblem(dir * one_ulp_over).empty(),
           "...and one ULP above it too — that is rounding, not overspeed");

    // ALL FOUR speed boundaries, three points each: exactly at it passes,
    // half an epsilon past it passes, two epsilons past it is refused. One
    // rule, one constant, four limits — the ceiling was fixed alone first
    // and the floor turned out to have the identical defect: the unit round
    // trip (FSM divides by initial_speed_unit_m, the gate multiplies back by
    // dynamics_unit_xy_m) puts the direction (1,1,0) at 2.8e-14 m/s BELOW a
    // floor commanded exactly, and (-18.3, -199.2, 0) — r5's real first-leg
    // aim — with it.
    {
      // Derived, never assumed: the test's unit and epsilon must be the ones
      // the RUNTIME uses, or the assertion drifts from the code the moment
      // either is retuned. That is the same class as the harness reading its
      // scenarios from a different tree than the binary.
      double um_xy = 100.0;
      if (node->has_parameter("optimization/dynamics_unit_xy_m"))
        node->get_parameter("optimization/dynamics_unit_xy_m", um_xy);
      const double eps_u =
          path_manager::PathManager::kSpeedBoundaryEpsMps / um_xy;
      const Eigen::Vector3d diag =
          Eigen::Vector3d(1.0, 1.0, 0.0).normalized();
      const double floor_u = pm->cruiseFloorUnits();
      expect(floor_u > 0.0, "the cruise floor is available to test against");

      // FLOOR. The diagonal is the direction that actually rounds under.
      expect(pm->stateEnvelopeProblem(diag * floor_u).empty(),
             "exactly AT the margin-backed cruise floor passes (diagonal — "
             "the direction that rounds under)");
      expect(pm->stateEnvelopeProblem(diag * (floor_u - 0.5 * eps_u)).empty(),
             "floor - 0.5 um/s passes");
      expect(!pm->stateEnvelopeProblem(diag * (floor_u - 2.0 * eps_u)).empty(),
             "floor - 2 um/s is refused — the rule is 1 um/s, not 'small'");
    }

    // The TOLERANCE ITSELF, both sides of it. Checking "1 ULP passes, 1%
    // fails" leaves everything from 0.1 to 1.9 m/s passing as well, so it
    // pins the sign of the fix and not its value — the earlier claim that
    // the mutation bracketed 1 um/s was wrong. kCapEpsMps is 1e-6 m/s and
    // the frame is 100 m per unit, so 1 um/s is 1e-8 u/s.
    const double half_eps_over = mv + 0.5e-8;    // cap + 0.5 um/s
    const double two_eps_over  = mv + 2.0e-8;    // cap + 2   um/s
    expect(pm->stateEnvelopeProblem(dir * half_eps_over).empty(),
           "cap + 0.5 um/s is INSIDE the tolerance and passes");
    const std::string just_over =
        pm->stateEnvelopeProblem(dir * two_eps_over);
    expect(!just_over.empty(),
           "cap + 2 um/s is OUTSIDE it and is refused — the tolerance is "
           "1 um/s, not 'some small number'");

    const std::string over = pm->stateEnvelopeProblem(dir * (mv * 1.01));
    expect(!over.empty(),
           "...and a genuine 1% overspeed is refused as well");
    expect(over.find("maximum") != std::string::npos ||
               over.find("cap") != std::string::npos,
           "...for the ceiling, by name");

    for (int form = 0; form < 2; ++form) {
      const bool as_vector = (form == 1);
      const path_manager::PlanResult r =
          as_vector
              ? chain.plan(mkHead(path_manager::StartStateSource::STATED_VECTOR, start_pos, at_cap, start_acc, false), goal, {})
              : chain.plan(mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos, at_cap, start_acc, false), goal, {});
      const std::string tag = as_vector ? " (vector form)" : " (scalar form)";
      expect(r.hasTrajectory(),
             "a start stated exactly AT the planning cap plans" + tag);
      expect(r.reason != path_manager::PlanReason::INITIAL_MODE_UNSUPPORTED &&
                 r.reason !=
                     path_manager::PlanReason::TRANSITION_GENERATION_FAILED,
             "...and is not pushed into the transition regime by rounding" +
                 tag);
    }
    // The remaining two boundaries live in classifyStartState, which is
    // private — so they are exercised through plan() and read off the
    // REASON. Below the activation speed there is no transition model and
    // the answer is INITIAL_MODE_UNSUPPORTED; at or above it the state is
    // TRANSITION_REQUIRED and the mission goes somewhere else, whatever the
    // coordinator then decides.
    {
      const auto *dyn = pm->dynamicsParams();
      expect(dyn != nullptr, "the dynamics model is available");
      if (dyn) {
        double UM = 100.0;
        if (node->has_parameter("optimization/dynamics_unit_xy_m"))
          node->get_parameter("optimization/dynamics_unit_xy_m", UM);
        const double eps_u =
            path_manager::PathManager::kSpeedBoundaryEpsMps / UM;
        const double act_u = dyn->model_activation_speed_mps / UM;
        const Eigen::Vector3d diag =
            Eigen::Vector3d(1.0, 1.0, 0.0).normalized();
        const auto reason_at = [&](double mag_u) {
          return chain.plan(mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos, diag * mag_u, start_acc, false), goal, {})
              .reason;
        };
        const auto UNSUP = path_manager::PlanReason::INITIAL_MODE_UNSUPPORTED;
        expect(reason_at(act_u) != UNSUP,
               "exactly AT the transition activation speed is not "
               "UNSUPPORTED");
        expect(reason_at(act_u - 0.5 * eps_u) != UNSUP,
               "activation - 0.5 um/s is not UNSUPPORTED either");
        expect(reason_at(act_u - 2.0 * eps_u) == UNSUP,
               "activation - 2 um/s IS UNSUPPORTED — the same 1 um/s rule");

        const double max_u = dyn->speed_max_mps / UM;
        expect(reason_at(max_u) != UNSUP,
               "exactly AT the model maximum speed is not UNSUPPORTED");
        expect(reason_at(max_u + 0.5 * eps_u) != UNSUP,
               "model max + 0.5 um/s is not UNSUPPORTED either");
        expect(reason_at(max_u + 2.0 * eps_u) == UNSUP,
               "model max + 2 um/s IS UNSUPPORTED");
      }
    }

    rclcpp::shutdown();
    if (failures == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  if (with_reststart) {
    // A STATED rest start. Legal to say (parseStartClaim accepts it, so the
    // operator's words survive intact); refused here, BY NAME, and never
    // raised to the margin-backed cruise floor — a clamped rest start publishes a flight
    // that begins at 131.8 m/s when the mission asked for 0.
    //
    // The refusal must be model-INDEPENDENT. Justifying it with the stall
    // floor would leave it unrefused under optimization/dynamics_enable:
    // false, where the floor does not exist; a zero head has no direction in
    // any configuration.
    for (int pass = 0; pass < 2; ++pass) {
      const bool dyn_on = (pass == 0);
      force("optimization/dynamics_enable", dyn_on);
      // force() alone does NOT reach the optimizer: dynamics_enable_ is
      // latched by setParam at init, so the second pass silently ran with
      // dynamics still ON and the model-independence claim went untested.
      // A forced re-init is what re-reads the parameter surface.
      pm->initOptimizer(/*force_reinit=*/true);
      expect(pm->dynamicsParams() != nullptr ? dyn_on : !dyn_on,
             std::string("the dynamics model really is ") +
                 (dyn_on ? "ON" : "OFF") + " for this pass");
      // Both stated forms, since the contract is that they agree.
      const path_manager::PlanResult rs =
          chain.plan(mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos, Eigen::Vector3d::Zero(), start_acc, false), goal, {});
      const path_manager::PlanResult rv =
          chain.plan(mkHead(path_manager::StartStateSource::STATED_VECTOR, start_pos, Eigen::Vector3d::Zero(), start_acc, false), goal, {});
      const std::string tag = dyn_on ? " (dynamics ON)" : " (dynamics OFF)";
      expect(!rs.hasTrajectory(),
             "a stated rest start is REFUSED, not clamped to the floor" + tag);
      expect(rs.reason == path_manager::PlanReason::INITIAL_MODE_UNSUPPORTED,
             "...as INITIAL_MODE_UNSUPPORTED, not UNSPECIFIED — the mission "
             "did say something" + tag);
      expect(!rv.hasTrajectory() && rv.reason == rs.reason,
             "...and the vector form of the same rest start agrees" + tag);
    }
    rclcpp::shutdown();
    if (failures == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  if (with_baserefuse) {
    // The BASELINE exits (mission below the split threshold, and segment
    // failure restoring the baseline) got a gate this session, and nothing
    // exercised its refusal direction: every non-route variant asserts a
    // baseline that SUCCEEDS, and the two variants asserting
    // STITCHED_FLIGHT_UNSAFE both reach it through stitchedVerdictResult
    // instead. Deleting the baseline_gate call would have gone unnoticed —
    // FlightVerdict::clean defaults to true, so the degrade beside it does
    // not misfire either.
    //
    // Mechanism borrowed from zonepass0: zones present with the
    // lexicographic 3-pass OFF leaves pass = 0, which makes the policy
    // snapshot INVALID, which makes the whole-flight verdict unmeasurable —
    // and an unjudgeable policy must refuse exactly like a condemned one.
    path_manager::RiskZone z;
    z.center = Eigen::Vector3d(180.0, 120.0, 3.0);
    z.reach = 20.0;
    z.peak = 0.9;
    pm->setRiskZonesRuntime({z});
    const auto snap = pm->zonePolicySnapshot();
    expect(!snap.valid,
           "the premise holds: pass 0 with zones leaves the snapshot INVALID");

    // Store something executable first, so the invalidation assertion below
    // cannot pass against an empty slot.
    const path_manager::PlanResult r =
        chain.plan(mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos, start_vel, start_acc, false), goal, {});
    expect(!r.hasTrajectory(),
           "a baseline whose zone policy cannot be judged is REFUSED");
    expect(r.reason == path_manager::PlanReason::STITCHED_FLIGHT_UNSAFE,
           "...as STITCHED_FLIGHT_UNSAFE");
    expect(pm->traj_.local_traj.duration == 0.0 &&
               pm->traj_.local_traj.start_time == 0.0,
           "...and nothing executable is left behind");
    rclcpp::shutdown();
    if (failures == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  if (with_nophasedirect) {
    // unsafedirect with ONE difference: chain/phase/enable is OFF. That
    // variant is the only one asserting the direct gate's refusal
    // direction, and it runs with the flag ON — so re-wrapping the gate in
    // "if (phase enabled)" would not have failed a single check anywhere in
    // the suite. The flag is the whole point of the comparison, so nothing
    // else may differ.
    force("chain/phase/enable", false);
    force("optimization/audit_envelope_peak_max", 0.01);
    const path_manager::PlanResult r =
        chain.plan(mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos, start_vel, start_acc, false), goal, {});
    expect(!r.hasTrajectory(),
           "with phase OFF the direct product is still gated and refused");
    expect(r.reason == path_manager::PlanReason::DIRECT_FALLBACK_UNSAFE,
           "...as DIRECT_FALLBACK_UNSAFE");
    expect(pm->traj_.local_traj.duration == 0.0 &&
               pm->traj_.local_traj.start_time == 0.0,
           "...and the stored trajectory stops being executable");
    rclcpp::shutdown();
    if (failures == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  if (with_wpzone || with_wpzonepass0) {
    // A multi-waypoint mission with a risk zone ANYWHERE must still plan.
    // The whole-flight gate first added to this branch refused all of them:
    // a multi-leg front end runs one zone search PER LEG and
    // zonePolicySnapshot is plan-wide-or-nothing, so policy_measurable was
    // false for every such mission and unflyable() fired on it — whatever
    // the flight did, and however far the zone was from the route.
    //
    // Per-leg capture plus per-piece attribution answers that properly, so
    // `wpzone` now pins the mission FLYING, MEASURED. The refusal it used to
    // pin has not gone away — it moved to the case where attribution is
    // genuinely unavailable, which `wpzonepass0` drives by turning the 3-pass
    // zone policy off so no leg can produce a capture at all.
    path_manager::RiskZone z;
    z.center = Eigen::Vector3d(600.0, 600.0, 3.0);   // far off the route
    z.reach = 20.0;
    z.peak = 0.9;
    pm->setRiskZonesRuntime({z});

    // Two legs: an intermediate waypoint plus the goal.
    std::vector<Eigen::Vector3d> legs;
    legs.push_back(0.5 * (start_pos + goal[0]));
    legs.push_back(goal[0]);
    const auto snap = pm->zonePolicySnapshot();
    expect(!snap.valid,
           "the multi-leg snapshot really is invalid — the premise holds");

    const path_manager::PlanResult r =
        chain.plan(mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos, start_vel, start_acc, false), legs, {});

    if (with_wpzone) {
      expect(r.hasTrajectory(),
             "a multi-leg mission with a zone far off the route flies");
      expect(r.reason != path_manager::PlanReason::ZONE_POLICY_UNEVALUATED,
             "...measured per leg, not excused");
      expect(r.detail.find("allow_unmeasured_zone_policy") == std::string::npos,
             "...with the opt-in never reached");
      expect(pm->legPolicySnapshots().size() == 2,
             "...because both legs were captured and both were usable");
    } else {
      // No pass ran, so there is no policy to capture and none to fall back
      // on. DEGRADED is a result the FSM EXECUTES — "we could not check, but
      // we said so" is fail-open however loud the log is. The cost of
      // refusing is real; flying an unchecked flight is a bigger one, and the
      // choice belongs to an operator rather than to a default.
      expect(pm->legPolicySnapshots().empty(),
             "with the 3-pass policy off no leg yields a capture");
      expect(!r.hasTrajectory(),
             "by DEFAULT an unmeasurable zone policy REFUSES the mission");
      expect(r.reason == path_manager::PlanReason::STITCHED_FLIGHT_UNSAFE,
             "...as STITCHED_FLIGHT_UNSAFE");
      expect(r.detail.find("allow_unmeasured_zone_policy") != std::string::npos,
             "...and the refusal names the opt-in that would change it");

      // OPT-IN: flown, DEGRADED, and the gap NAMED in the machine-readable
      // reason — not only in the detail string. Asserting the detail alone is
      // what let the reason be silently overwritten with
      // STITCHED_ENVELOPE_BUDGET, which is about the airframe and was simply
      // not true here.
      force("manager/allow_unmeasured_zone_policy", true);
      const path_manager::PlanResult r2 =
          chain.plan(mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos, start_vel, start_acc, false), legs, {});
      expect(r2.hasTrajectory(),
             "with the opt-in set the mission plans");
      expect(r2.outcome == path_manager::PlanOutcome::DEGRADED,
             "...as DEGRADED, because something really was not checked");
      expect(r2.reason == path_manager::PlanReason::ZONE_POLICY_UNEVALUATED,
             "...and the REASON says so — a program reads this, not the detail");
      expect(r2.detail.find("zone policy was NOT evaluated") != std::string::npos,
             "...with the gap spelled out in the detail as well");
    }
    rclcpp::shutdown();
    if (failures == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  if (with_standoffpen) {
    using ZC = path_manager::PathManager::ZoneContactResult;
    using ZD = path_manager::PathManager::ZoneDisposition;
    // The COMPLEMENT of hardpen, and the reason the refusal surface moved.
    // hardpen flies through the authored volume and must be REFUSED.
    // This flies through the 1.05x routing standoff shell OUTSIDE that
    // volume and must be DEGRADED, not refused — a flight half a kilometre
    // clear of anything the mission declared is not an unsafe flight, and
    // refusing it on that surface flipped identical missions between CLEAN
    // and FAILED because the optimizer's zone force is identically zero
    // exactly there (docs/design/zone_gate_surface.md).
    path_manager::RiskZone z;
    z.center = Eigen::Vector3d(180.0, 120.0, 3.0);
    z.reach = 20.0;
    z.peak = 0.9;
    pm->setRiskZonesRuntime({z});
    const path_manager::PlanResult r0 =
        chain.plan(mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos, start_vel, start_acc, false), goal, {});
    expect(r0.hasTrajectory(), "the avoiding plan itself succeeds");
    const auto snap = pm->zonePolicySnapshot();
    expect(snap.valid, "snapshot is VALID (the policy really ran)");
    expect(!snap.zones.empty() &&
               snap.zones[0].disposition == ZD::HARD_AVOID,
           "an unneeded zone stays HARD_AVOID");

    // Tangent at 20.5 u from the centre: reach is 20.0, so q = 1.025 there —
    // inside the 1.05 shell, OUTSIDE the authored volume. The geometry is
    // asserted below rather than assumed, because a test that silently
    // measures zero contacts would pass every assertion about refusal.
    const double kOffset = 20.5;
    const Eigen::Vector3d tangent(z.center.x(), z.center.y() + kOffset,
                                  z.center.z());
    expect(pm->zoneContact(snap, 0, tangent) == ZC::CONTACT,
           "the tangent point reads CONTACT (it IS in the standoff shell)");
    expect(!pm->zoneContactAuthored(snap, 0, tangent),
           "...but it is NOT inside the volume the mission authored");
    expect(pm->zoneContactAuthored(snap, 0, z.center),
           "...while the centre IS — the two tests are not the same test");

    // 120 u in 80 s = 1.5 u/s = 150 m/s, inside the cruise band, same as
    // hardpen: no_cruise must not be what decides this.
    const Eigen::Vector3d a(z.center.x() - 60.0, z.center.y() + kOffset,
                            z.center.z());
    const Eigen::Vector3d b(z.center.x() + 60.0, z.center.y() + kOffset,
                            z.center.z());
    const auto seg = [&](double f0, double f1) {
      Eigen::Matrix<double, 3, 6> c = Eigen::Matrix<double, 3, 6>::Zero();
      const Eigen::Vector3d p0 = a + f0 * (b - a);
      const Eigen::Vector3d p1 = a + f1 * (b - a);
      c.col(5) = p0;
      c.col(4) = (p1 - p0) / 40.0;
      return c;
    };
    const poly_traj::Trajectory grazing(
        std::vector<double>{40.0, 40.0},
        std::vector<poly_traj::CoefficientMat>{seg(0.0, 0.5), seg(0.5, 1.0)});
    const auto fv = chain.evaluateFlightForTest(
        grazing, {{grazing.getTotalDuration(),
                   path_manager::SegmentChainPlanner::PhaseKind::CRUISE,
                   "synthetic"}});
    expect(fv.evaluated, "the grazing flight was evaluated");
    expect(!fv.underground, "it does not clip terrain");
    expect(!fv.no_cruise, "it reaches cruise speed");
    expect(fv.policy_measurable, "on a snapshot that IS measurable");
    expect(fv.zone_standoff_n > 0,
           "the standoff shell is entered and COUNTED");
    expect(fv.zone_hard_n == 0,
           "...and the authored volume is not entered");
    expect(!fv.unflyable(),
           "a standoff graze does NOT make the flight unflyable");

    // And the caller must receive it as a flyable, labelled degradation
    // with its trajectory intact.
    pm->traj_.setLocalTraj(grazing, 100.0, 0);
    const auto pr = chain.verdictResultForTest(fv);
    expect(pr.hasTrajectory(), "the standoff graze keeps its trajectory");
    expect(pr.outcome == path_manager::PlanOutcome::DEGRADED,
           "...as DEGRADED, not SUCCESS — the operator is told");
    expect(pr.reason == path_manager::PlanReason::STITCHED_ZONE_STANDOFF,
           "...with the standoff reason, not an envelope budget");
    expect(pm->traj_.local_traj.duration > 0.0 &&
               pm->traj_.local_traj.start_time > 0.0,
           "...and the stored trajectory stays executable");
    rclcpp::shutdown();
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
        chain.plan(mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos, start_vel, start_acc, false), goal, {});
    // Contract change: an unjudgeable zone policy is now fail-closed. Before,
    // the flight was returned and only the log said CHECK — a verdict nobody
    // downstream consumed, so the trajectory was stored and publishable.
    expect(!r.hasTrajectory(),
           "zones + unjudgeable policy -> no trajectory (fail-closed)");
    expect(r.reason == path_manager::PlanReason::STITCHED_FLIGHT_UNSAFE,
           "reason is STITCHED_FLIGHT_UNSAFE, not an envelope budget");
    std::cout << "zonepass0: zone-avoid pass " << pm->zoneAvoidPassNow()
              << "\n";
    expect(pm->zoneAvoidPassNow() == 0, "3-pass did not run (policy off)");
    const auto snap = pm->zonePolicySnapshot();
    expect(!snap.valid, "zones + pass 0 -> INVALID snapshot");
    expect(pm->zoneContact(snap, 0, z.center) == ZC::INVALID,
           "contact on the pass-0 snapshot is INVALID, never CLEAR/CONTACT");
    // Structured, not log text: the earlier form was
    //   has("UNEVALUATED") || !has("verdict: CLEAN")
    // which passes when NO log is found at all — the opposite of what it
    // claims to check.
    const auto &fv0 = chain.lastFlightVerdict();
    expect(fv0.evaluated, "the flight was actually evaluated");
    expect(!fv0.policy_measurable,
           "zone policy reads UNMEASURABLE on a pass-0 snapshot");
    expect(fv0.unflyable(), "an unmeasurable zone policy is unflyable");
    rclcpp::shutdown();
    if (failures == 0) { std::cout << "PASS: 0 failed check(s)\n"; return 0; }
    std::cout << "FAIL: " << failures << " failed check(s)\n";
    return 1;
  }

  if (with_badspans) {
    // A real flight to judge: plan once, keep the stored trajectory.
    const path_manager::PlanResult r0 =
        chain.plan(mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos, start_vel, start_acc, false), goal, {});
    expect(r0.hasTrajectory(), "fixture plan succeeds");
    const poly_traj::Trajectory traj = pm->traj_.local_traj.traj;
    const double T = traj.getTotalDuration();
    using PK = path_manager::SegmentChainPlanner::PhaseKind;
    using Span = path_manager::SegmentChainPlanner::PhaseSpan;
    // Each span-contract violation must yield UNEVALUATED.
    expect(!chain.evaluateFlight(traj, {{T * 0.5, PK::CRUISE, "short"}},
                                 nullptr, 0)
                .evaluated,
           "last span not covering the flight -> UNEVALUATED");
    expect(!chain.evaluateFlight(
                  traj, {{T * 0.6, PK::CRUISE, "a"}, {T * 0.4, PK::CRUISE,
                                                      "b"}}, nullptr, 0)
                .evaluated,
           "non-increasing t_end -> UNEVALUATED");
    expect(!chain.evaluateFlight(
                  traj, {{T * 0.5, PK::CRUISE, "cruise"},
                         {T, PK::TRANSITION, "late-transition"}}, nullptr, 0)
                .evaluated,
           "TRANSITION after cruise -> UNEVALUATED");
    expect(!chain.evaluateFlight(
                  traj, {{T * 0.5, PK::TERMINAL, "terminal"},
                         {T, PK::CRUISE, "tail"}}, nullptr, 0)
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
    expect(chain.commitRoute(mkHead(path_manager::StartStateSource::TRAJECTORY_DERIVED, start_pos, start_vel, start_acc), goal, false, &route, &cap, &fe_ms),
           "commitRoute produces the route");
    const int n_confirmed = chain.segments();
    const path_manager::PlanResult ra = chain.planOverRoute(route, cap, fe_ms, mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos, start_vel, start_acc), goal, false, {});
    expect(ra.hasTrajectory(), "reference call succeeds");
    const poly_traj::Trajectory ref = pm->traj_.local_traj.traj;

    force("chain/jitter/fail_segment", 2);  // one-shot: forces a merge
    const path_manager::PlanResult rb = chain.planOverRoute(route, cap, fe_ms, mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos, start_vel, start_acc), goal, false, {});
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

    const path_manager::PlanResult rc = chain.planOverRoute(route, cap, fe_ms, mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos, start_vel, start_acc), goal, false, {});
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
    expect(chain.commitRoute(mkHead(path_manager::StartStateSource::TRAJECTORY_DERIVED, start_pos, start_vel, start_acc), goal, false, &route, &cap, &fe_ms),
           "commitRoute produces a route to tamper with");
    // (a) truncated cap — the silent-abandonment case
    std::vector<double> cap_bad(cap.begin(), cap.end() - 1);
    const path_manager::PlanResult ra = chain.planOverRoute(route, cap_bad, fe_ms, mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos, start_vel, start_acc), goal, false, {});
    expect(!ra.hasTrajectory() &&
               ra.detail.find("cap size") != std::string::npos,
           "mismatched cap FAILS with the invariant named");
    // (b) head displaced off the route start
    const path_manager::PlanResult rb = chain.planOverRoute(route, cap, fe_ms, mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos + Eigen::Vector3d(1.0, 0.0, 0.0), start_vel, start_acc), goal, false, {});
    expect(!rb.hasTrajectory() &&
               rb.detail.find("head position") != std::string::npos,
           "head off the route start FAILS");
    // (c) NaN vertex
    std::vector<Eigen::Vector3d> route_nan = route;
    route_nan[route_nan.size() / 2].z() = std::nan("");
    const path_manager::PlanResult rc = chain.planOverRoute(route_nan, cap, fe_ms, mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos, start_vel, start_acc), goal, false, {});
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
    expect(chain.commitRoute(mkHead(path_manager::StartStateSource::TRAJECTORY_DERIVED, start_pos, start_vel, start_acc), goal, false, &route, &cap, &fe_ms),
           "commitRoute produces the route");
    chain.setTransitionActive(true);
    force("chain/jitter/fail_segment", -1);
    const path_manager::PlanResult r = chain.planOverRoute(route, cap, fe_ms, mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos, start_vel, start_acc), goal, false, {});
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
    const path_manager::PlanResult r = chain.plan(mkHead(path_manager::StartStateSource::STATED_VECTOR, start_pos, Eigen::Vector3d(2.1, 0.0, 0.0), start_acc, false), goal, {});
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
    const path_manager::PlanResult r = chain.plan(mkHead(path_manager::StartStateSource::STATED_VECTOR, start_pos, Eigen::Vector3d(1.8, 0.0, 0.0), Eigen::Vector3d(0.0, std::nan(""), 0.0), false), goal, {});
    expect(!r.hasTrajectory(), "NaN commanded acceleration FAILED");
    // The reason moved from INITIAL_MODE_UNSUPPORTED to
    // INITIAL_STATE_MALFORMED, and that is more accurate rather than a
    // regression: the head's fail-closed entry check now catches a
    // non-finite PVA before classifyStartState runs. "Unsupported mode" is a
    // statement about the flight regime; a NaN is a statement about the
    // command being unreadable, which is exactly what MALFORMED names.
    // Still refused before any planning work — the third check below.
    expect(r.reason == path_manager::PlanReason::INITIAL_STATE_MALFORMED,
           "reason is INITIAL_STATE_MALFORMED — a NaN is unreadable input, "
           "not an unsupported regime");
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
        chain.plan(mkHead(path_manager::StartStateSource::STATED_VECTOR, start_pos, Eigen::Vector3d(1.8, 0.0, 0.0), Eigen::Vector3d(0.0, 5.0, 0.0), true), goal, {});
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
    // "Synthesized" means the DIRECTION is ours — level, along the first
    // leg — while the SPEED is the mission's own initial_speed_mps. So the
    // two halves get opposite treatment, and this variant pins both.
    //
    // Until the planner owned the initial phase, a sub-floor synthesized
    // start was repaired by the [STALL-FLOOR] clamp and flown, on the
    // reasoning that a synthesized state is our invention to fix. That
    // reasoning only ever covered the direction: raising the speed
    // publishes a flight that begins faster than the mission asked for,
    // which is the same fabrication as inventing a start for a mission
    // that stated none.
    //
    // Half 1: 50 m/s stated, floor ~132 m/s -> refused, not repaired.
    const path_manager::PlanResult r =
        chain.plan(mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos, Eigen::Vector3d(0.5, 0.0, 0.0), start_acc, false), goal, {});
    expect(!r.hasTrajectory(),
           "synthesized sub-floor start is REFUSED, not clamped and flown");
    // The REASON is no longer INITIAL_MODE_UNSUPPORTED, and that is the fix
    // rather than a regression. 50 m/s sits above the model activation speed
    // (40) and below the cruise floor (131.8): it is the TRANSITION regime.
    // The scalar form used to be judged by statedStartSpeedProblem, which
    // tested the margin-backed cruise floor and refused outright, so classifyStartState was
    // unreachable from it and a launch-regime start stated as a scalar could
    // never dispatch to the coordinator — while the identical speed stated
    // as a vector did. Now both reach it.
    //
    // So the assertion that matters is not which reason came back, it is
    // that the two stated forms of the SAME physical state agree. That is
    // the contract in one line, and it holds whatever the coordinator then
    // decides.
    const path_manager::PlanResult rv =
        chain.plan(mkHead(path_manager::StartStateSource::STATED_VECTOR, start_pos, Eigen::Vector3d(0.5, 0.0, 0.0), start_acc, false), goal, {});
    expect(r.hasTrajectory() == rv.hasTrajectory() && r.reason == rv.reason,
           "the scalar and vector forms of the same start state get the SAME "
           "outcome — one quantity, one gate");
    expect(r.reason == path_manager::PlanReason::TRANSITION_GENERATION_FAILED,
           "...and a sub-cruise, above-activation start now REACHES the "
           "transition coordinator instead of being refused before it");
    // Half 2: a flyable stated speed still plans, and is still not
    // misread as a launch-regime state — the original coverage, kept.
    const path_manager::PlanResult r2 =
        chain.plan(mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos, Eigen::Vector3d(2.0, 0.0, 0.0), start_acc, false), goal, {});
    expect(r2.hasTrajectory(),
           "synthesized start above the floor plans normally");
    expect(r2.outcome == path_manager::PlanOutcome::SUCCESS,
           "single-shot on a below-threshold mission stays SUCCESS");
    expect(r2.reason != path_manager::PlanReason::INITIAL_MODE_UNSUPPORTED,
           "no INITIAL_MODE_UNSUPPORTED misclassification on a flyable "
           "synthesized start");
    // Half 3: the [STALL-FLOOR] clamp is still LIVE and still correct for a
    // head this stack authored itself — an in-flight replan reads its start
    // off the current trajectory (neither commanded nor synthesized), and a
    // converged flight may legitimately dip below the margin-backed floor.
    // Refusing that would strand a flying vehicle over a rounding error.
    // This half is here because the reversal above removed the only variant
    // that reached the clamp at all; without it the clamp ships untested.
    const path_manager::PlanResult r3 =
        chain.plan(mkHead(path_manager::StartStateSource::TRAJECTORY_DERIVED, start_pos, Eigen::Vector3d(0.5, 0.0, 0.0), start_acc, false), goal, {});
    expect(r3.hasTrajectory(),
           "a trajectory-derived sub-floor head is still clamped and flies "
           "(the planner may repair a state it authored itself)");
    expect(r3.reason != path_manager::PlanReason::INITIAL_MODE_UNSUPPORTED,
           "and is not refused as a stated initial state");
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
        chain.plan(mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos, start_vel, start_acc, false), goal, {});
    expect(r1.hasTrajectory(),
           "phase-mode direct fallback flies under the default gate");
    expect(r1.outcome == path_manager::PlanOutcome::DEGRADED,
           "direct fallback reports DEGRADED");
    force("optimization/audit_envelope_peak_max", 0.01);
    const path_manager::PlanResult r2 =
        chain.plan(mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos, start_vel, start_acc, false), goal, {});
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
      chain.plan(mkHead(path_manager::StartStateSource::TRAJECTORY_DERIVED, start_pos, start_vel, start_acc, false), goal, mtail);
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
    // [AUTO-N] production target (35 since 2410bdb): the fixture is far below
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
    const bool ok2 = chain.plan(mkHead(path_manager::StartStateSource::STATED_SPEED, start_pos, start_vel, start_acc, false), goal, {})
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
