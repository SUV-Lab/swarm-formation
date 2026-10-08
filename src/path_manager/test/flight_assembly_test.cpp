// Analytic trajectories exercise assembly without ROS or an optimizer.
#include "path_manager/flight_assembly.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <string>

namespace {
using path_manager::AssembledFlight;
using path_manager::PhaseKind;
using path_manager::prependFlightPrefix;
using path_manager::stitchRouteFlight;
int failures = 0;

void check(bool condition, const std::string &message) {
  std::cout << (condition ? "[OK] " : "[FAIL] ") << message << '\n';
  if (!condition) ++failures;
}

bool near(double a, double b) { return std::abs(a - b) < 1e-12; }

// x(t) = x + v*t + a*t^2/2 + j*t^3/6 + q*t^5; y=3, z=4.
poly_traj::Trajectory piece(double duration, double x, double v = 1.0,
                            double a = 0.0, double j = 0.0, double q = 0.0) {
  poly_traj::CoefficientMat c = poly_traj::CoefficientMat::Zero();
  c.col(5) = Eigen::Vector3d(x, 3.0, 4.0);
  c(0, 4) = v;
  c(0, 3) = a / 2.0;
  c(0, 2) = j / 6.0;
  c(0, 0) = q;
  return poly_traj::Trajectory({duration}, {c});
}

bool phase(const AssembledFlight &f, size_t i, double end, PhaseKind kind,
           const std::string &name) {
  return i < f.phases.size() && f.phases[i].t_end == end &&
         f.phases[i].kind == kind && f.phases[i].name == name;
}
}  // namespace

int main() {
  path_manager::PrefixJoinResult retained;
  {
    const auto prefix = piece(2.0, 0.0);
    const std::vector<poly_traj::Trajectory> runs = {
        piece(2.0, 2.0), piece(3.0, 4.0), piece(1.0, 7.0)};
    auto route = stitchRouteFlight(runs, true);
    check(route.trajectory.getPieceNum() == 3 &&
              phase(route, 0, 2.0, PhaseKind::CRUISE, "departure") &&
              phase(route, 1, 5.0, PhaseKind::CRUISE, "cruise-1") &&
              phase(route, 2, 6.0, PhaseKind::CRUISE, "arrival"),
          "route labels retain cruise judgment and cumulative run times");
    const auto plain = stitchRouteFlight(runs, false);
    check(phase(plain, 0, 2.0, PhaseKind::CRUISE, "seg1") &&
              phase(plain, 1, 5.0, PhaseKind::CRUISE, "seg2") &&
              phase(plain, 2, 6.0, PhaseKind::CRUISE, "seg3") &&
              phase(stitchRouteFlight({runs[0]}, true), 0, 2.0,
                    PhaseKind::CRUISE, "departure"),
          "plain labels and the single departure span retain their semantics");
    route.trajectory.append(piece(2.0, 8.0));
    route.phases.push_back({8.0, PhaseKind::TERMINAL, "terminal"});
    retained = prependFlightPrefix(route, prefix, 0.0);
    check(retained.joined && retained.position_error == 0.0 &&
              retained.velocity_error == 0.0 && retained.acceleration_error == 0.0,
          "exact analytic PVA seam joins at zero tolerance");
    const auto &full = retained.flight;
    check(full.phases.size() == 5 &&
              phase(full, 0, 2.0, PhaseKind::TRANSITION, "transition") &&
              phase(full, 1, 4.0, PhaseKind::CRUISE, "departure") &&
              phase(full, 2, 7.0, PhaseKind::CRUISE, "cruise-1") &&
              phase(full, 3, 8.0, PhaseKind::CRUISE, "arrival") &&
              phase(full, 4, 10.0, PhaseKind::TERMINAL, "terminal"),
          "prefix shifts every original phase once, including terminal");
    check(prefix.getPieceNum() == 1 && prefix.getTotalDuration() == 2.0 &&
              prefix.getPos(1.0) == Eigen::Vector3d(1.0, 3.0, 4.0) &&
              route.trajectory.getPieceNum() == 4 &&
              route.trajectory.getTotalDuration() == 8.0 &&
              phase(route, 3, 8.0, PhaseKind::TERMINAL, "terminal"),
          "joining leaves both input prefix and retained unprefixed flight unchanged");

    const std::vector<poly_traj::Trajectory> mismatches = {
        piece(1.0, 2.1), piece(1.0, 2.0, 1.1), piece(1.0, 2.0, 1.0, 0.1)};
    for (size_t i = 0; i < mismatches.size(); ++i) {
      const auto candidate = stitchRouteFlight({mismatches[i]}, false);
      const auto refused = prependFlightPrefix(candidate, prefix, 0.01);
      const double errors[] = {refused.position_error, refused.velocity_error,
                               refused.acceleration_error};
      check(!refused.joined && near(errors[i], 0.1) &&
                refused.flight.trajectory.getPieceNum() == 1 &&
                refused.flight.trajectory[0].getCoeffMat() ==
                    mismatches[i][0].getCoeffMat() &&
                phase(refused.flight, 0, 1.0, PhaseKind::CRUISE, "seg1"),
            "PVA component " + std::to_string(i) + " refuses without altering the flight");
    }
    const auto refused = prependFlightPrefix(
        route, prefix, std::numeric_limits<double>::quiet_NaN());
    check(!refused.joined && refused.position_error == 0.0 &&
              refused.flight.trajectory.getTotalDuration() == 8.0 &&
              phase(refused.flight, 3, 8.0, PhaseKind::TERMINAL, "terminal"),
          "NaN tolerance refuses even a continuous seam");

    // The quintic has matching initial PVA but initial jerk 6 rather than 0.
    const auto quintic = prependFlightPrefix(
        stitchRouteFlight({piece(1.0, 2.0, 1.0, 0.0, 6.0, 1.0)}, true), prefix, 0.0);
    check(quintic.joined && near(quintic.jerk_jump, 6.0) &&
              near(quintic.flight.trajectory.getPos(3.0).x(), 5.0) &&
              near(quintic.flight.trajectory.getVel(3.0).x(), 9.0) &&
              near(quintic.flight.trajectory.getAcc(3.0).x(), 26.0),
          "quintic jerk discontinuity is measured without becoming a join gate");
    const auto constant = prependFlightPrefix(
        stitchRouteFlight({piece(1.0, 5.0, 0.0)}, false), piece(2.0, 5.0, 0.0), 0.0);
    check(constant.joined && constant.flight.trajectory.getVel(2.5).isZero() &&
              constant.flight.trajectory.getPos(2.5) == Eigen::Vector3d(5.0, 3.0, 4.0),
          "constant trajectory assembles with zero derivatives");
  }
  bool analytic = retained.flight.trajectory.getPieceNum() == 5;
  for (double t : {0.0, 1.0, 2.0, 4.0, 6.0, 9.0, 10.0}) {
    analytic = analytic &&
        retained.flight.trajectory.getPos(t) == Eigen::Vector3d(t, 3.0, 4.0) &&
        retained.flight.trajectory.getVel(t) == Eigen::Vector3d::UnitX() &&
        retained.flight.trajectory.getAcc(t).isZero();
  }
  check(analytic && phase(retained.flight, 4, 10.0, PhaseKind::TERMINAL, "terminal"),
        "owned flight retains analytic global PVA and phases after all inputs die");
  const auto empty = stitchRouteFlight({}, true);
  const auto empty_join = prependFlightPrefix(empty, piece(1.0, 0.0), 0.0);
  const auto no_prefix = prependFlightPrefix(retained.flight, {}, 0.0);
  check(empty.trajectory.getPieceNum() == 0 && empty.phases.empty() &&
            !empty_join.joined && !no_prefix.joined &&
            std::isinf(empty_join.position_error) &&
            no_prefix.flight.trajectory.getPieceNum() == 5,
        "empty assembly and either empty join side are handled without sampling");
  std::cout << (failures ? "FAIL: " : "PASS: ") << failures << " failed checks\n";
  return failures ? 1 : 0;
}
