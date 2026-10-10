// An external trajectory can be evaluated without constructing a planner,
// PathManager, ROS node, or execution slot. Coordinates retain planner units.
#include "path_manager/flight_evaluator.h"

#include <iostream>
#include <memory>

int main()
{
  // Level flight at 200 m/s, 1000 m global altitude (1 u = 100 m).
  poly_traj::CoefficientMat coefficients = poly_traj::CoefficientMat::Zero();
  coefficients(0, 4) = 2.0;
  coefficients(2, 5) = 10.0;
  poly_traj::Trajectory trajectory({2.0}, {coefficients});
  const std::vector<path_manager::PhaseSpan> phases{
      {2.0, path_manager::PhaseKind::CRUISE, "external"}};

  path_manager::FlightEvaluationInput input{trajectory, phases};
  input.model = std::make_shared<mmp_vehicle_dynamics::FixedWingModel>();
  // An explicit flat terrain provider. A real caller supplies its existing
  // terrain lookup; its owner must live through this synchronous call.
  input.environment.terrain_elevation = [](double, double, double *height) {
    *height = 0.0;
    return true;
  };
  // zone_count = 0 describes this example's empty world. Worlds with zones
  // must also provide measured policy provenance and canonical zone queries.
  const auto report = path_manager::evaluateFlight(input);
  std::cout << "evaluated=" << report.verdict.evaluated
            << " clean=" << report.verdict.clean
            << " min_agl_u=" << report.total.min_agl
            << " utilization_peak=" << report.verdict.util_peak << '\n';
  // Optional presentation, separate from the measurement itself.
  for (const auto &line : path_manager::formatFlightEvaluation(report))
    std::cout << line.text << '\n';
  return report.verdict.evaluated && !report.verdict.unflyable() ? 0 : 1;
}
