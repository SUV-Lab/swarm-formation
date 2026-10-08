#ifndef PATH_OPTIMIZER_TAIL_BOUNDARY_H
#define PATH_OPTIMIZER_TAIL_BOUNDARY_H

#include <Eigen/Eigen>

namespace ego_planner
{
  // [PHASE] Explicit tail boundary for one solve. Replaces the old
  // "end_vel != 0 means prescribed" sentinel, which could not express a
  // zero-velocity tail or an acceleration-only prescription (design review
  // find). prescribe_vel pins the velocity verbatim; prescribe_acc pins the
  // acceleration (defaults to zero when only the velocity is pinned —
  // contract semantics). Neither flag = the legacy arrival-contract tail
  // (level entry at cruise speed).
  struct TailBoundary
  {
    bool prescribe_vel{false};
    bool prescribe_acc{false};
    Eigen::Vector3d vel{Eigen::Vector3d::Zero()};
    Eigen::Vector3d acc{Eigen::Vector3d::Zero()};

    bool anyPrescribed() const { return prescribe_vel || prescribe_acc; }
    bool allFinite() const
    {
      return vel.allFinite() && acc.allFinite();
    }
    static TailBoundary pinned(const Eigen::Vector3d &v,
                               const Eigen::Vector3d &a)
    {
      return TailBoundary{true, true, v, a};
    }
  };
}  // namespace ego_planner

#endif  // PATH_OPTIMIZER_TAIL_BOUNDARY_H
