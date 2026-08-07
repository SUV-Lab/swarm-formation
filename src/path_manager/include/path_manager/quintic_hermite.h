#ifndef PATH_MANAGER_QUINTIC_HERMITE_H
#define PATH_MANAGER_QUINTIC_HERMITE_H

#include <Eigen/Core>

#include "path_optimizer/poly_traj_utils.hpp"

namespace path_manager {

// One quintic Hermite piece matching exact P/V/A at both ends. Coefficient
// convention (poly_traj::Piece): p(t) = sum_i col(i) * t^(5-i), col(5) the
// constant. Verified against getPos/getVel/getAcc closed forms.
// Single definition for every PVA-matching adapter (terminal descent,
// transition polynomialization) — the coefficient convention must not fork.
inline poly_traj::CoefficientMat quinticHermite(const Eigen::Vector3d &p0,
                                                const Eigen::Vector3d &v0,
                                                const Eigen::Vector3d &a0,
                                                const Eigen::Vector3d &p1,
                                                const Eigen::Vector3d &v1,
                                                const Eigen::Vector3d &a1,
                                                double T)
{
  const double T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;
  const Eigen::Vector3d A = p1 - p0 - v0 * T - 0.5 * a0 * T2;
  const Eigen::Vector3d B = v1 - v0 - a0 * T;
  const Eigen::Vector3d C = a1 - a0;
  poly_traj::CoefficientMat M;
  M.col(5) = p0;
  M.col(4) = v0;
  M.col(3) = 0.5 * a0;
  M.col(2) = 10.0 * A / T3 - 4.0 * B / T2 + 0.5 * C / T;
  M.col(1) = -15.0 * A / T4 + 7.0 * B / T3 - C / T2;
  M.col(0) = 6.0 * A / T5 - 3.0 * B / T4 + 0.5 * C / T3;
  return M;
}

}  // namespace path_manager

#endif  // PATH_MANAGER_QUINTIC_HERMITE_H
