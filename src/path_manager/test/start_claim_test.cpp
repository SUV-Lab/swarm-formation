// Stage 1 of the initial-state contract, exhaustively.
//
// These are MESSAGE-level states — two sources stated at once, a negative
// speed, a NaN, a value with no claim bit — and none of them is expressible
// through SegmentChainPlanner::plan(), which takes a resolved start state.
// chain_experiment_test contains no TrajectoryCommand at all, so before this
// binary existed the malformed rows had nowhere to live and were checked by
// reading the code.
//
// No ROS spin, no node, no message package: parseStartClaim is templated so
// the row under test is a plain struct with the same field names, which is
// also the point — the function must be a pure function of the message.

#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include "path_manager/start_state.h"

namespace {

int failures = 0;

void expect(bool ok, const std::string &what)
{
  std::printf("%s   %s\n", ok ? "[OK]  " : "[FAIL]", what.c_str());
  if (!ok) ++failures;
}

struct V3 { double x{0.0}, y{0.0}, z{0.0}; };

// The subset of TrajectoryCommand the claim parser reads, by the same names.
struct Cmd {
  bool use_initial_speed{false};
  double initial_speed{0.0};
  bool use_initial_velocity{false};
  V3 initial_velocity{};
  bool use_initial_acceleration{false};
  V3 initial_acceleration{};
};

using path_manager::MissionStartClaim;
using path_manager::StartStateSource;

constexpr double kUnit = 100.0;  // 1 planner unit = 100 m
const double kNaN = std::numeric_limits<double>::quiet_NaN();
const double kInf = std::numeric_limits<double>::infinity();

MissionStartClaim parse(const Cmd &c)
{
  return path_manager::parseStartClaim(c, kUnit);
}

bool says(const MissionStartClaim &c, const char *fragment)
{
  return c.problem.find(fragment) != std::string::npos;
}

}  // namespace

int main()
{
  // --- row 1: both sources stated -----------------------------------------
  {
    Cmd c;
    c.use_initial_speed = true;   c.initial_speed = 150.0;
    c.use_initial_velocity = true; c.initial_velocity = {150.0, 0.0, 0.0};
    const auto r = parse(c);
    expect(r.malformed(), "both sources stated is MALFORMED");
    expect(says(r, "one quantity, one claim"),
           "...and the message says which two sources collided");
  }

  // --- row 2: non-finite speed --------------------------------------------
  for (double bad : {kNaN, kInf, -kInf}) {
    Cmd c;
    c.use_initial_speed = true; c.initial_speed = bad;
    const auto r = parse(c);
    expect(r.malformed() && says(r, "non-finite initial_speed"),
           "a non-finite stated speed is MALFORMED, not downgraded");
  }

  // --- row 3: negative speed ----------------------------------------------
  {
    Cmd c;
    c.use_initial_speed = true; c.initial_speed = -5.0;
    const auto r = parse(c);
    expect(r.malformed() && says(r, "publishers must not clamp"),
           "a negative stated speed is MALFORMED");
    // The reason the old std::max(0.0, ...) had to go: a clamped -5 and a
    // stated rest start become the same message, and they mean opposite
    // things.
    Cmd rest;
    rest.use_initial_speed = true; rest.initial_speed = 0.0;
    expect(!parse(rest).malformed(),
           "...while a stated ZERO is well-formed — the two must not collapse");
  }

  // --- row 4: stated speed, including zero --------------------------------
  {
    Cmd c;
    c.use_initial_speed = true; c.initial_speed = 150.0;
    const auto r = parse(c);
    expect(!r.malformed() && r.src == StartStateSource::STATED_SPEED,
           "a stated speed parses as STATED_SPEED");
    expect(std::abs(r.speed_u - 1.5) < 1e-12,
           "...converted to planner units (150 m/s / 100 = 1.5 u/s)");
    expect(r.vel_u.isZero(),
           "...with no velocity vector: the direction is the first route leg "
           "and does not exist yet");

    Cmd z;
    z.use_initial_speed = true; z.initial_speed = 0.0;
    const auto rz = parse(z);
    expect(!rz.malformed() && rz.src == StartStateSource::STATED_SPEED &&
               rz.speed_u == 0.0,
           "a stated REST start is legal to SAY (it is refused later, not "
           "here, and never raised to the floor)");
  }

  // --- row 5: non-finite velocity vector ----------------------------------
  {
    Cmd c;
    c.use_initial_velocity = true; c.initial_velocity = {1.0, kNaN, 0.0};
    const auto r = parse(c);
    expect(r.malformed() && says(r, "non-finite initial_velocity"),
           "a non-finite stated vector is MALFORMED, not a WARN-and-downgrade");
  }

  // --- row 6: stated vector, including zero -------------------------------
  {
    Cmd c;
    c.use_initial_velocity = true; c.initial_velocity = {150.0, -50.0, 10.0};
    const auto r = parse(c);
    expect(!r.malformed() && r.src == StartStateSource::STATED_VECTOR,
           "a stated vector parses as STATED_VECTOR");
    expect((r.vel_u - Eigen::Vector3d(1.5, -0.5, 0.1)).norm() < 1e-12,
           "...converted componentwise into planner units");

    Cmd z;
    z.use_initial_velocity = true;
    const auto rz = parse(z);
    expect(!rz.malformed() && rz.src == StartStateSource::STATED_VECTOR &&
               rz.vel_u.isZero(),
           "a stated ZERO vector is legal to say, like a stated zero speed");
  }

  // --- row 7: a value with no claim bit -----------------------------------
  {
    Cmd c;
    c.initial_speed = 150.0;   // no use_initial_speed
    const auto r = parse(c);
    expect(r.malformed() && says(r, "predates the initial-state contract"),
           "a scalar with no claim bit is MALFORMED — an un-updated publisher "
           "must break loudly, not have its stale field flown");
  }

  // --- row 8: nothing said ------------------------------------------------
  {
    const auto r = parse(Cmd{});
    expect(!r.malformed() && r.src == StartStateSource::UNSPECIFIED,
           "an empty command is UNSPECIFIED — read, and it said nothing");
    expect(!r.acc_prescribed, "...with no acceleration prescribed");
  }

  // --- acceleration rows, independent of the velocity form ----------------
  {
    Cmd c;
    c.use_initial_acceleration = true;
    c.initial_acceleration = {0.0, 0.0, kNaN};
    const auto r = parse(c);
    expect(r.malformed() && says(r, "non-finite initial_acceleration"),
           "a non-finite prescribed acceleration is MALFORMED");
  }
  {
    Cmd c;
    c.initial_acceleration = {1.0, 0.0, 0.0};   // no claim bit
    const auto r = parse(c);
    expect(r.malformed() && says(r, "initial_acceleration is non-zero"),
           "a non-zero acceleration with no claim bit is MALFORMED");
  }
  {
    Cmd c;
    c.use_initial_speed = true; c.initial_speed = 150.0;
    c.use_initial_acceleration = true;   // value left at exactly zero
    const auto r = parse(c);
    expect(!r.malformed() && r.acc_prescribed && r.acc_u.isZero(),
           "a prescribed acceleration of exactly zero is PRESCRIBED — the "
           "bool decides, the value never does");
  }

  // --- the deliberate non-error -------------------------------------------
  {
    Cmd c;
    c.use_initial_velocity = true; c.initial_velocity = {150.0, 0.0, 0.0};
    c.initial_speed = 99.0;            // stale scalar beside a claimed vector
    const auto r = parse(c);
    expect(!r.malformed() && r.src == StartStateSource::STATED_VECTOR,
           "a stale scalar beside a claimed vector is NOT malformed — old and "
           "new contracts agree the vector wins, so nothing is misread");
    expect(path_manager::statedVectorIgnoresScalar(c),
           "...but it is reportable, so the ignored value can be named");
  }

  // --- the unit itself ----------------------------------------------------
  {
    Cmd c;
    c.use_initial_speed = true; c.initial_speed = 150.0;
    for (double bad_unit : {0.0, -1.0, kNaN}) {
      const auto r = path_manager::parseStartClaim(c, bad_unit);
      expect(r.malformed() && says(r, "initial_speed_unit_m"),
             "a non-positive or non-finite unit is MALFORMED — converting by "
             "it would silently change the magnitude that was stated");
    }
  }

  // --- the predicates the planners key on ---------------------------------
  {
    using path_manager::mayClamp;
    using path_manager::mayReaim;
    expect(mayReaim(StartStateSource::STATED_SPEED, false),
           "only a stated SPEED with no prescribed acceleration may be "
           "re-aimed onto the route");
    expect(!mayReaim(StartStateSource::STATED_SPEED, true),
           "...a prescribed acceleration pins the direction");
    expect(!mayReaim(StartStateSource::STATED_VECTOR, false),
           "...a stated vector already has a direction");
    expect(mayClamp(StartStateSource::TRAJECTORY_DERIVED),
           "only our own in-flight trajectory may be raised to the stall "
           "floor");
    for (auto s : {StartStateSource::STATED_SPEED,
                   StartStateSource::STATED_VECTOR,
                   StartStateSource::TEST_INJECTED,
                   StartStateSource::CHAIN_JUNCTION,
                   StartStateSource::UNSPECIFIED}) {
      expect(!mayClamp(s), std::string("...never ") +
                               path_manager::sourceName(s) +
                               ": raising a stated speed answers a different "
                               "question than the one asked");
    }
  }

  // --- the per-source policy matrix, directly -----------------------------
  // applyHeadPolicy is the single owner of [VEL-ALIGN] and [STALL-FLOOR].
  // Before it existed the two rules were implemented twice and had drifted;
  // this table is the contract, one row per source, asserted rather than
  // described in a comment.
  {
    using path_manager::applyHeadPolicy;
    using path_manager::StartHead;
    const std::vector<Eigen::Vector3d> route = {
        Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d(10.0, 0.0, 1.0)};
    const double floor_u = 1.3176;                  // 131.76 m/s at 100 m/u
    const double eps_u = kUnit > 0.0 ? 1e-6 / kUnit : 0.0;

    struct Row {
      StartStateSource src;
      const char *name;
      bool expect_reaim;
      bool expect_floor;
    };
    const Row rows[] = {
        {StartStateSource::STATED_SPEED,       "STATED_SPEED",       true,  false},
        {StartStateSource::STATED_VECTOR,      "STATED_VECTOR",      false, false},
        {StartStateSource::TEST_INJECTED,      "TEST_INJECTED",      false, false},
        {StartStateSource::CHAIN_JUNCTION,     "CHAIN_JUNCTION",     false, false},
        {StartStateSource::TRAJECTORY_DERIVED, "TRAJECTORY_DERIVED", false, true},
    };
    for (const auto &r : rows) {
      // Aimed ACROSS the route and BELOW the floor, so both rules would fire
      // if the source allowed them.
      StartHead h;
      h.src = r.src;
      h.vel_u = Eigen::Vector3d(0.0, 0.5, 0.0);
      const auto out = applyHeadPolicy(h, route, floor_u, true, eps_u);
      expect(out.reaimed == r.expect_reaim,
             std::string(r.name) + (r.expect_reaim ? " IS re-aimed onto the route"
                                                   : " is NOT re-aimed"));
      expect(out.floored == r.expect_floor,
             std::string(r.name) +
                 (r.expect_floor ? " IS raised to the cruise floor"
                                 : " is NOT raised — it may not be rewritten"));
      if (!r.expect_floor)
        expect(out.floor_declined,
               std::string(r.name) +
                   ": the declined correction is REPORTED, not silent");
    }

    // A prescribed acceleration pins the frame: even the one re-aimable
    // source stops being re-aimable.
    StartHead pinned;
    pinned.src = StartStateSource::STATED_SPEED;
    pinned.acc_prescribed = true;
    pinned.vel_u = Eigen::Vector3d(0.0, 2.0, 0.0);
    expect(!applyHeadPolicy(pinned, route, floor_u, true, eps_u).reaimed,
           "a prescribed acceleration stops the re-aim");

    // The numeric-equality rule reaches this path too. Asserting only
    // "exactly at the floor is not raised" pins nothing: an axis-aligned
    // magnitude divides exactly, so it passes with or without the epsilon —
    // the same way capstart's first version picked a direction that rounded
    // the harmless way. The load-bearing point is HALF AN EPSILON UNDER,
    // which is direction-independent and is precisely what the tolerance
    // exists to accept.
    StartHead at_floor;
    at_floor.src = StartStateSource::TRAJECTORY_DERIVED;
    at_floor.vel_u = Eigen::Vector3d(1.0, 0.0, 0.0) * floor_u;
    expect(!applyHeadPolicy(at_floor, route, floor_u, true, eps_u).floored,
           "a head exactly AT the floor is not raised");
    at_floor.vel_u = Eigen::Vector3d(1.0, 0.0, 0.0) * (floor_u - 0.5 * eps_u);
    expect(!applyHeadPolicy(at_floor, route, floor_u, true, eps_u).floored,
           "half an epsilon under it is not raised either — that is rounding");
    at_floor.vel_u = Eigen::Vector3d(1.0, 0.0, 0.0) * (floor_u - 2.0 * eps_u);
    expect(applyHeadPolicy(at_floor, route, floor_u, true, eps_u).floored,
           "...and two epsilons under it IS raised");

    // align disabled = no re-aim for anyone.
    StartHead sp;
    sp.src = StartStateSource::STATED_SPEED;
    sp.vel_u = Eigen::Vector3d(0.0, 2.0, 0.0);
    expect(!applyHeadPolicy(sp, route, floor_u, false, eps_u).reaimed,
           "align disabled suppresses the re-aim");
  }

  // --- the three cases the boolean plumbing could not express -------------
  // These are the defects the head value exists to make impossible, asserted
  // at the level where they were lost.
  {
    using path_manager::applyHeadPolicy;
    using path_manager::isOperatorInput;
    using path_manager::judgeFullPva;
    using path_manager::StartHead;
    const std::vector<Eigen::Vector3d> route = {
        Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d(10.0, 0.0, 1.0)};

    // 1. A stated SPEED with a prescribed acceleration. The classifier used
    //    to ask for "start_vel_commanded && start_acc_commanded", and the
    //    scalar form sets the first to false — so the acceleration was
    //    discarded and the head reached the optimizer unjudged. It must be
    //    judged as a full PVA, and it must NOT be re-aimed.
    StartHead sp_acc;
    sp_acc.src = StartStateSource::STATED_SPEED;
    sp_acc.vel_u = Eigen::Vector3d(0.0, 2.0, 0.0);
    sp_acc.acc_prescribed = true;
    expect(judgeFullPva(sp_acc.src, sp_acc.acc_prescribed),
           "a stated SPEED with a prescribed acceleration is judged as a "
           "full PVA");
    expect(!applyHeadPolicy(sp_acc, route, 1.3176, true, 1e-8).reaimed,
           "...and is NOT re-aimed — the acceleration pins the frame");
    StartHead sp_noacc = sp_acc;
    sp_noacc.acc_prescribed = false;
    expect(!judgeFullPva(sp_noacc.src, sp_noacc.acc_prescribed),
           "...while the same speed WITHOUT one is judged on velocity alone");
    expect(applyHeadPolicy(sp_noacc, route, 1.3176, true, 1e-8).reaimed,
           "...and is re-aimed");

    // 2. A test injection with a prescribed acceleration is always full-PVA.
    StartHead inj;
    inj.src = StartStateSource::TEST_INJECTED;
    inj.vel_u = Eigen::Vector3d(0.0, 2.0, 0.0);
    inj.acc_prescribed = true;
    expect(judgeFullPva(inj.src, inj.acc_prescribed),
           "an injected state with a prescribed acceleration is judged as a "
           "full PVA");
    expect(isOperatorInput(inj.src),
           "...because an injection is operator input, not our own product");

    // 3. TRANSITION_HANDOFF: neither re-aimed nor floored, and it is a
    //    SOURCE rather than "the caller happened to hold a pointer".
    StartHead ho;
    ho.src = StartStateSource::TRANSITION_HANDOFF;
    ho.vel_u = Eigen::Vector3d(0.0, 0.5, 0.0);   // across the route AND under
    ho.acc_prescribed = true;
    const auto hp = applyHeadPolicy(ho, route, 1.3176, true, 1e-8);
    expect(!hp.reaimed, "a transition handoff is NOT re-aimed");
    expect(!hp.floored, "...and NOT floored — the generator validated it");
    expect(hp.floor_declined, "...and the declined correction is reported");
    expect(path_manager::isAlreadyValidated(ho.src),
           "...because it is a state the planner already validated");
  }

  if (failures == 0) {
    std::printf("PASS: 0 failed check(s)\n");
    return 0;
  }
  std::printf("FAIL: %d failed check(s)\n", failures);
  return 1;
}
