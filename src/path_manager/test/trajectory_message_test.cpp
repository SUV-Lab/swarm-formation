#include "path_manager/trajectory_message.h"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace {
int failures = 0;
void check(bool ok, const std::string &description)
{
  std::cout << (ok ? "[OK] " : "[FAIL] ") << description << '\n';
  if (!ok) ++failures;
}

double wirePosition(const std::vector<float> &coefficients, int piece, double t)
{
  double value = 0.0;
  for (int power = 0; power < 6; ++power)
    value = value * t + coefficients[piece * 6 + power];
  return value;
}
}  // namespace

int main()
{
  poly_traj::CoefficientMat first = poly_traj::CoefficientMat::Zero();
  first(0, 4) = 2.0; first(0, 5) = 3.0;  // x = 2t + 3
  first(1, 3) = 0.5; first(1, 5) = -4.0; // y = t^2/2 - 4
  first(2, 0) = 0.25; first(2, 5) = 1.0; // z = t^5/4 + 1
  poly_traj::CoefficientMat second = poly_traj::CoefficientMat::Zero();
  second(0, 5) = 7.0; second(1, 5) = 8.0; second(2, 5) = 9.0;
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 123; stamp.nanosec = 987654321;
  mmp_traj_msgs::msg::PolyTraj retained;
  {
    poly_traj::Trajectory trajectory({1.25, 2.5}, {first, second});
    retained = path_manager::makeTrajectoryMessage(trajectory, 7, 42, stamp);
    check(retained.drone_id == 7 && retained.trajectory_id == 42 &&
              retained.start_time == stamp && retained.order == 5,
          "caller-selected identity and exact ROS timestamp survive serialization");
    check(retained.duration == std::vector<float>({1.25f, 2.5f}) &&
              retained.coef_x.size() == 12 && retained.coef_y.size() == 12 &&
              retained.coef_z.size() == 12,
          "wire arrays keep one duration and six coefficients per piece");
    const double t = 0.5;
    check(wirePosition(retained.coef_x, 0, t) == 4.0 &&
              wirePosition(retained.coef_y, 0, t) == -3.875 &&
              wirePosition(retained.coef_z, 0, t) == 1.0078125 &&
              wirePosition(retained.coef_x, 1, t) == 7.0 &&
              wirePosition(retained.coef_y, 1, t) == 8.0 &&
              wirePosition(retained.coef_z, 1, t) == 9.0,
          "published polynomial order and piece boundaries reproduce analytic positions");
    check(trajectory.getPiece(0).getCoeffMat() == first &&
              trajectory.getPiece(1).getCoeffMat() == second,
          "serialization leaves the supplied owned trajectory unchanged");
    builtin_interfaces::msg::Time reference_stamp;
    reference_stamp.sec = 456;
    const auto reference = path_manager::makeTrajectoryMessage(
        trajectory, 7, 0, reference_stamp);
    check(reference.trajectory_id == 0 && reference.start_time == reference_stamp &&
              reference.coef_x == retained.coef_x &&
              reference.duration == retained.duration,
          "reference publication can select separate identity/time for the same coefficients");
  }
  check(wirePosition(retained.coef_z, 0, 0.5) == 1.0078125 &&
            retained.duration == std::vector<float>({1.25f, 2.5f}),
        "wire result remains valid after the source trajectory is destroyed");
  retained = path_manager::makeTrajectoryMessage({}, 1, 99, stamp);
  check(retained.duration.empty() && retained.coef_x.empty() &&
            retained.coef_y.empty() && retained.coef_z.empty() &&
            retained.trajectory_id == 99 && retained.start_time == stamp,
        "an empty result cannot retain coefficients from a prior serialization");
  std::cout << (failures ? "FAIL" : "PASS") << ": " << failures << " failed check(s)\n";
  return failures ? 1 : 0;
}
