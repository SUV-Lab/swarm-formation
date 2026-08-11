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

  if (failures == 0) {
    std::printf("PASS: 0 failed check(s)\n");
    return 0;
  }
  std::printf("FAIL: %d failed check(s)\n", failures);
  return 1;
}
