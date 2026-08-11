#ifndef PATH_MANAGER_START_STATE_H
#define PATH_MANAGER_START_STATE_H

#include <cmath>
#include <string>

#include <Eigen/Eigen>

// Where a mission's initial state came from, and what may be done to it.
//
// Three booleans used to carry this — start_vel_commanded, start_vel_
// synthesized, start_state_stated — which is eight combinations for four
// legal states, and the illegal ones were reachable: the have_local_traj_
// branch never reset the acceleration flag, so a stale bool travelled into
// the planner and was harmless only by accident. One value with exactly the
// legal set replaces them.
//
// Lives beside planning_result.h rather than in the FSM header, so planners
// do not depend on the state machine.
namespace path_manager {

enum class StartStateSource {
  // Nothing was said and nothing could be derived. Refused before planning:
  // inventing a start state is what this whole contract exists to stop.
  UNSPECIFIED,
  // The command stated a scalar speed. The DIRECTION is the first route leg,
  // which does not exist at callback time — hence speed_u on the claim and a
  // head vector built later in startMissionPlan.
  STATED_SPEED,
  // The command stated a full velocity vector.
  STATED_VECTOR,
  // Taken from the trajectory currently being flown. The only source that
  // may be repaired, because it is our own product rather than an operator's
  // statement.
  TRAJECTORY_DERIVED,
  // test/inject_init_state. Judged in full, never re-aimed, never clamped.
  TEST_INJECTED,
  // The tail of the previous chain segment. Judged as that segment's tail
  // already was; kept verbatim.
  CHAIN_JUNCTION,
};

inline const char *sourceName(StartStateSource s)
{
  switch (s) {
    case StartStateSource::UNSPECIFIED:        return "UNSPECIFIED";
    case StartStateSource::STATED_SPEED:       return "STATED_SPEED";
    case StartStateSource::STATED_VECTOR:      return "STATED_VECTOR";
    case StartStateSource::TRAJECTORY_DERIVED: return "TRAJECTORY_DERIVED";
    case StartStateSource::TEST_INJECTED:      return "TEST_INJECTED";
    case StartStateSource::CHAIN_JUNCTION:     return "CHAIN_JUNCTION";
  }
  return "UNKNOWN";
}

// An operator SAID this, in one of the two stated forms.
inline bool isStated(StartStateSource s)
{
  return s == StartStateSource::STATED_SPEED ||
         s == StartStateSource::STATED_VECTOR;
}

// Came from outside the planner — a stated claim or a test injection. These
// are judged; they are never rewritten to something flyable.
inline bool isOperatorInput(StartStateSource s)
{
  return isStated(s) || s == StartStateSource::TEST_INJECTED;
}

// The head may be re-aimed onto the front-end route. Only the scalar form
// qualifies, and only when no acceleration was prescribed: a stated vector
// means a stated direction, and a prescribed acceleration pins the frame the
// direction lives in.
inline bool mayReaim(StartStateSource s, bool acc_prescribed)
{
  return s == StartStateSource::STATED_SPEED && !acc_prescribed;
}

// The head magnitude may be raised to the stall floor. Exactly one source
// qualifies: our own in-flight trajectory. Raising an operator's stated
// speed would answer a different question than the one they asked, which is
// the fabrication this contract exists to stop.
inline bool mayClamp(StartStateSource s)
{
  return s == StartStateSource::TRAJECTORY_DERIVED;
}

// The full position-velocity-acceleration triple is judged (rather than the
// velocity alone) exactly when an acceleration was prescribed. A numeric
// zero acceleration is NOT evidence: the FSM's internal 0 read as an
// in-cone hold and flipped the regime (review find), which is why the bool
// decides and the value never does.
inline bool judgeFullPva(StartStateSource s, bool acc_prescribed)
{
  return acc_prescribed && isOperatorInput(s);
}

// What a mission command claimed about its initial state, after pure
// message validation and before any FSM context is consulted.
struct MissionStartClaim {
  StartStateSource src{StartStateSource::UNSPECIFIED};
  // STATED_SPEED carries a magnitude only; the direction is the first route
  // leg and is not known at callback time.
  double speed_u{0.0};
  Eigen::Vector3d vel_u{Eigen::Vector3d::Zero()};
  bool acc_prescribed{false};
  Eigen::Vector3d acc_u{Eigen::Vector3d::Zero()};
  // Non-empty means the COMMAND could not be read. Distinct from
  // UNSPECIFIED, which means it was read and said nothing.
  std::string problem;

  bool malformed() const { return !problem.empty(); }
};

// The value threaded through the planners in place of the three booleans.
struct StartHead {
  StartStateSource src{StartStateSource::UNSPECIFIED};
  bool acc_prescribed{false};
};

namespace start_state_detail {

inline bool finite3(const Eigen::Vector3d &v)
{
  return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

}  // namespace start_state_detail

// Stage 1 of the initial-state contract: a PURE function of the message.
//
// It reads no node parameter, no FSM member and no context, so it can and
// must run unconditionally at the top of the command callback — before any
// state is consumed or overwritten. The previous ordering validated after
// mutating, so a refusal still dropped the flying trajectory and overwrote
// the start point with the rejected mission's.
//
// `unit_m` converts SI metres to planner units. The caller is responsible
// for it being the same unit the envelope gates convert back with; the FSM
// checks that at construction, because a drift there silently makes the
// magnitude JUDGED differ from the magnitude STATED.
//
// Templated on the message type so this header does not pull in the message
// package — the test binary constructs a plain struct with the same fields.
template <typename CommandT>
MissionStartClaim parseStartClaim(const CommandT &m, double unit_m)
{
  MissionStartClaim c;
  const double inv = (std::isfinite(unit_m) && unit_m > 0.0) ? 1.0 / unit_m
                                                             : 0.0;
  if (inv == 0.0) {
    c.problem = "initial_speed_unit_m is not a positive finite length";
    return c;
  }

  const bool S = m.use_initial_speed;
  const bool V = m.use_initial_velocity;
  const double v = m.initial_speed;
  const Eigen::Vector3d vec(m.initial_velocity.x, m.initial_velocity.y,
                            m.initial_velocity.z);

  // Row order is the contract. First match wins and the table is total.
  if (S && V) {
    c.problem =
        "two stated initial-state sources (use_initial_speed and "
        "use_initial_velocity) — one quantity, one claim";
    return c;
  }
  if (S && !std::isfinite(v)) {
    c.problem = "non-finite initial_speed";
    return c;
  }
  if (S && v < 0.0) {
    char b[96];
    snprintf(b, sizeof b,
             "negative initial_speed %.3f — publishers must not clamp", v);
    c.problem = b;
    return c;
  }
  if (S) {
    // Includes v == 0: a stated REST start is legal to SAY. It is refused
    // later by the envelope gate as INITIAL_MODE_UNSUPPORTED, never
    // silently raised to the stall floor — a clamped rest start is
    // indistinguishable from a speed the operator chose.
    c.src = StartStateSource::STATED_SPEED;
    c.speed_u = v * inv;
  } else if (V && !start_state_detail::finite3(vec)) {
    c.problem = "non-finite initial_velocity";
    return c;
  } else if (V) {
    // Includes the zero vector, for the same reason.
    c.src = StartStateSource::STATED_VECTOR;
    c.vel_u = vec * inv;
  } else if (v != 0.0) {
    // A scalar with no claim bit. The publisher predates the contract, or
    // the bit was forgotten — either way the value cannot be honoured
    // silently, because doing so is how an un-updated publisher's stale
    // field became a flown initial state.
    char b[144];
    snprintf(b, sizeof b,
             "initial_speed=%.3f with use_initial_speed=false — publisher "
             "predates the initial-state contract, or the claim bit was "
             "forgotten", v);
    c.problem = b;
    return c;
  } else {
    c.src = StartStateSource::UNSPECIFIED;
  }

  // Acceleration is independent of the velocity form and is checked after
  // it, so a malformed acceleration cannot mask a malformed velocity.
  const Eigen::Vector3d a(m.initial_acceleration.x, m.initial_acceleration.y,
                          m.initial_acceleration.z);
  if (m.use_initial_acceleration) {
    if (!start_state_detail::finite3(a)) {
      c.problem = "non-finite initial_acceleration";
      return c;
    }
    c.acc_prescribed = true;   // including exactly zero
    c.acc_u = a * inv;
  } else if (a != Eigen::Vector3d::Zero()) {
    c.problem =
        "initial_acceleration is non-zero with use_initial_acceleration="
        "false — publisher predates the initial-state contract, or the "
        "claim bit was forgotten";
    return c;
  }
  return c;
}

// NOT malformed, and deliberately so: a stale publisher that left a scalar
// beside a claimed vector. The old contract and the new one agree the vector
// wins, so nothing is misread — but the ignored value is worth naming.
template <typename CommandT>
bool statedVectorIgnoresScalar(const CommandT &m)
{
  return m.use_initial_velocity && !m.use_initial_speed &&
         m.initial_speed != 0.0;
}

}  // namespace path_manager

#endif  // PATH_MANAGER_START_STATE_H
