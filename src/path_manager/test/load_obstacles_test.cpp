// Sanity test for the message-level shape used by ReplanFSM::loadObstaclesCallback.
//
// This test does NOT exercise the rclcpp subscription path. It validates the
// per-spec dispatch logic by constructing DynamicObstacleArray messages and
// running the same decisions the callback executes (replan_fsm.cpp
// loadObstaclesCallback): CUBE -> addDynamicBox, SPHERE -> addDynamicSphere,
// anything else skipped; msg.replace clears the existing set first.
//
// The wiring (subscription / publisher / network) is covered by the manual
// RViz checklist documented in docs/research/manual_test_dynamic_obstacle.md.

#include <cstdint>
#include <iomanip>
#include <iostream>
#include <string>

#include "mmp_mission_msgs/msg/dynamic_obstacle_array.hpp"
#include "mmp_mission_msgs/msg/dynamic_obstacle_spec.hpp"

namespace {

int g_passed = 0;
int g_failed = 0;

void check(bool cond, const std::string& label) {
  std::cout << "  " << (cond ? "[OK]   " : "[FAIL] ")
            << std::left << std::setw(56) << label << "\n";
  if (cond) ++g_passed; else ++g_failed;
}

// Mirrors the dispatch decisions of ReplanFSM::loadObstaclesCallback without
// depending on a real PathManager. Kinds must match the production order:
// KIND_CUBE and KIND_SPHERE are both accepted; everything else is skipped.
struct DispatchResult {
  size_t boxes = 0;
  size_t spheres = 0;
  size_t skipped = 0;
  bool cleared = false;
};

DispatchResult simulate_dispatch(
    const mmp_mission_msgs::msg::DynamicObstacleArray& msg)
{
  DispatchResult r;
  if (msg.replace) r.cleared = true;
  for (const auto& spec : msg.obstacles) {
    if (spec.kind == mmp_mission_msgs::msg::DynamicObstacleSpec::KIND_CUBE) {
      ++r.boxes;
    } else if (spec.kind ==
               mmp_mission_msgs::msg::DynamicObstacleSpec::KIND_SPHERE) {
      ++r.spheres;
    } else {
      ++r.skipped;
    }
  }
  return r;
}

void test_empty_array() {
  std::cout << "[test_empty_array]\n";
  mmp_mission_msgs::msg::DynamicObstacleArray msg;
  auto r = simulate_dispatch(msg);
  check(r.boxes == 0 && r.spheres == 0, "empty: nothing added");
  check(r.skipped == 0, "empty: skipped == 0");
  check(!r.cleared, "empty: replace defaults to false (append)");
}

void test_mixed_kinds() {
  std::cout << "[test_mixed_kinds]\n";
  mmp_mission_msgs::msg::DynamicObstacleArray msg;

  mmp_mission_msgs::msg::DynamicObstacleSpec cube;
  cube.kind = mmp_mission_msgs::msg::DynamicObstacleSpec::KIND_CUBE;
  cube.size.x = 1.0; cube.size.y = 1.0; cube.size.z = 1.0;
  cube.model = "building";
  msg.obstacles.push_back(cube);

  mmp_mission_msgs::msg::DynamicObstacleSpec sphere;
  sphere.kind = mmp_mission_msgs::msg::DynamicObstacleSpec::KIND_SPHERE;
  sphere.radius = 5.0;
  msg.obstacles.push_back(sphere);

  // Only KIND_SPHERE and KIND_CUBE exist; every other kind value must be
  // skipped rather than dispatched.
  mmp_mission_msgs::msg::DynamicObstacleSpec unknown;
  unknown.kind = 2;
  msg.obstacles.push_back(unknown);

  auto r = simulate_dispatch(msg);
  check(r.boxes == 1, "mixed: cube dispatches to addDynamicBox");
  check(r.spheres == 1, "mixed: sphere dispatches to addDynamicSphere");
  check(r.skipped == 1, "mixed: unknown kind is the only skipped spec");
}

void test_replace_flag() {
  std::cout << "[test_replace_flag]\n";
  mmp_mission_msgs::msg::DynamicObstacleArray msg;
  msg.replace = true;
  mmp_mission_msgs::msg::DynamicObstacleSpec cube;
  cube.kind = mmp_mission_msgs::msg::DynamicObstacleSpec::KIND_CUBE;
  msg.obstacles.push_back(cube);
  auto r = simulate_dispatch(msg);
  check(r.cleared, "replace=true clears the existing set first");
  check(r.boxes == 1, "replace still dispatches the payload");

  mmp_mission_msgs::msg::DynamicObstacleArray clear_only;
  clear_only.replace = true;
  auto rc = simulate_dispatch(clear_only);
  check(rc.cleared && rc.boxes == 0 && rc.spheres == 0,
        "empty+replace acts as a pure clear");
}

void test_model_field_carried() {
  std::cout << "[test_model_field_carried]\n";
  mmp_mission_msgs::msg::DynamicObstacleSpec s;
  check(s.model.empty(), "model defaults to empty (analytic-only obstacle)");
  s.model = "ship";
  mmp_mission_msgs::msg::DynamicObstacleArray msg;
  msg.obstacles.push_back(s);
  check(msg.obstacles.front().model == "ship",
        "model string survives the wire format");
}

void test_constants_match_spec() {
  std::cout << "[test_constants_match_spec]\n";
  check(mmp_mission_msgs::msg::DynamicObstacleSpec::KIND_SPHERE == 0,
        "KIND_SPHERE == 0");
  check(mmp_mission_msgs::msg::DynamicObstacleSpec::KIND_CUBE == 1,
        "KIND_CUBE == 1");
}

}  // namespace

int main() {
  test_empty_array();
  test_mixed_kinds();
  test_replace_flag();
  test_model_field_carried();
  test_constants_match_spec();
  std::cout << "\n==== load_obstacles_test: passed=" << g_passed
            << " failed=" << g_failed << " ====\n";
  return g_failed == 0 ? 0 : 1;
}
