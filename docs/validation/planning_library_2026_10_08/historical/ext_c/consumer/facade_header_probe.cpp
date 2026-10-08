#include <path_manager/mission_planner.h>

bool hasRetainedResult(const path_manager::MissionPlanResult &result)
{
  return result.hasTrajectory();
}
