#ifndef PATH_MANAGER_OPTIMIZER_WORKER_PROFILE_H
#define PATH_MANAGER_OPTIMIZER_WORKER_PROFILE_H

#include <string>

namespace path_manager {

// The worker-local configuration boundary. Soft weights can be dropped with
// an explicit degradation note; hard maxima survive merges/fallbacks as the
// conservative minimum. Thread budget is assigned by the coordinator only,
// never accepted from a phase/segment cost profile.
//
// These keys are optimizer-owned and cannot change the manager's shared
// terrain/clearance/cap machinery. Types, defaults and numerical clamps remain
// with the existing ROS parameter reader and PolyTrajOptimizer::setParam.
enum class OptimizerWorkerParameterKind { kSoft, kHardMax, kThreadBudget };
struct OptimizerWorkerParameter {
  const char *name;
  OptimizerWorkerParameterKind tag;
};

inline const OptimizerWorkerParameter *findOptimizerWorkerParameter(
    const std::string &name)
{
  static constexpr OptimizerWorkerParameter keys[] = {
      {"optimization/weight_time", OptimizerWorkerParameterKind::kSoft},
      {"optimization/weight_Risk", OptimizerWorkerParameterKind::kSoft},
      {"optimization/max_vel", OptimizerWorkerParameterKind::kHardMax},
      {"optimization/risk_parallel_threads",
       OptimizerWorkerParameterKind::kThreadBudget},
  };
  for (const auto &key : keys)
    if (name == key.name) return &key;
  return nullptr;
}

}  // namespace path_manager

#endif  // PATH_MANAGER_OPTIMIZER_WORKER_PROFILE_H
