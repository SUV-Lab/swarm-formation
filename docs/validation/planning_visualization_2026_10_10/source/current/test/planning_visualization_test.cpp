#include "path_manager/planning_visualization_data.h"

#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>

namespace viz = path_manager::visualization;
namespace {
int failures = 0;
void check(bool condition, const char *label) {
  std::cout << (condition ? "[OK] " : "[FAIL] ") << label << '\n';
  if (!condition) ++failures;
}
template<class Function> void rejects(Function function, const char *label) {
  bool rejected = false;
  try { function(); } catch (const std::invalid_argument &) { rejected = true; }
  check(rejected, label);
}
poly_traj::Trajectory line(double duration = 2.0) {
  auto coefficients = poly_traj::CoefficientMat::Zero().eval();
  coefficients.col(5) = Eigen::Vector3d(-1.0, 0.0, 2.0);
  coefficients(0, 4) = 1.0;
  return poly_traj::Trajectory({duration}, {coefficients});
}
viz::RiskSettings settings() {
  viz::RiskSettings result;
  result.vertical_ratio = 0.35;
  result.softness = 0.1;
  result.threshold = 0.5;
  result.obstacle_clearance = 0.7;
  result.grounded = true;
  result.max_velocity = 2.0;
  result.step = 1.0;
  result.agl = 0.15;
  result.volume_alpha = 0.24;
  result.contours = 5;
  result.mode = "heatmap";
  result.heatmap_agl_max = 2.0;
  result.heatmap_offset = 0.3;
  result.heatmap_preview_lift = 20.0;
  result.heatmap_max_dim = 16;
  return result;
}
}  // namespace

int main() {
  const viz::Stamp stamp = [] { builtin_interfaces::msg::Time t; t.sec = 7; return t; };
  std::vector<Eigen::Vector3d> points{{1, 2, 3}, {4, 5, 6}};
  auto route = viz::makeFrontEndPath(points, stamp());
  auto tube = viz::makeTube(points, 3, 1.5, "route_", 0.1f, 0.2f, 0.3f, 0.4f, stamp);
  points.front().setZero();
  points.clear();
  check(route.poses.front().pose.position.x == 1.0 &&
        tube && tube->points.front().x == 1.0, "prepared messages own source point values");
  check(route.poses.front().header.stamp.sec == 7 && tube->header.stamp.sec == 7,
        "caller supplies message clock without a ROS node");
  check(!viz::makeTube({}, 0, 1.0, "", 0, 0, 0, 1, stamp) &&
        !viz::makeTrajectoryTube({}, 0, 1.0, "", 0, 0, 0, 1, stamp),
        "empty geometry retains no-tube behavior");
  auto owned_tube = viz::makeTrajectoryTube(line(), 0, 1.0, "", 0, 0, 0, 1, stamp);
  check(owned_tube && !owned_tube->points.empty() && std::abs(owned_tube->points.back().x - 1.0) < 1e-8,
        "sampled geometry survives temporary trajectory destruction");
  const auto hidden = viz::makeTrajectoryTube(line(), 0, 0.0, "", 0, 0, 0, 1, stamp);
  const auto hidden_chain = viz::makeChain({line()}, {}, nullptr, 0.0, stamp);
  check(hidden && hidden->scale.x == 0.0 && hidden_chain.markers.at(1).scale.x == 0.0,
        "legacy zero path scale still prepares hidden geometry");
  viz::Obstacle sphere;
  sphere.size = Eigen::Vector3d::Ones();
  sphere.mesh_resource = "fixture://sphere";
  const auto hidden_obstacle = viz::makeDynamicObstacles({sphere}, 0.0, stamp);
  check(hidden_obstacle.markers.at(1).scale.z == 0.0,
        "legacy zero sphere mesh height does not reject planning");
  const auto clear = viz::makeTrajectoryClear();
  check(clear.action == visualization_msgs::msg::Marker::DELETEALL &&
        clear.header.frame_id == "world", "trajectory clear keeps legacy action/frame");
  check(viz::makeChain({}, {}, nullptr, 1.0, stamp).markers.front().action ==
            visualization_msgs::msg::Marker::DELETEALL,
        "empty chain remains an owned clear-only array");

  std::vector<path_manager::RiskZone> zones(1);
  zones[0].center = Eigen::Vector3d(0, 0, 2);
  zones[0].reach = 10;
  zones[0].peak = 0.8;
  viz::RiskView field{zones, {},
    [](size_t, const Eigen::Vector3d &) { return 0.25; },
    [](size_t, const Eigen::Vector3d &) { return 1.0; },
    [](size_t, const Eigen::Vector3d &) { return -std::numeric_limits<double>::infinity(); }};
  const auto config = settings();
  const auto profile = viz::makeRiskProfile(line(), field, config);
  check(profile.message.data.size() > 2 && profile.message.data[0] == 4.0 &&
        profile.message.data[1] == 1.0, "external profile preserves self-describing v4 contract");
  const auto risk_line = viz::makeTrajectoryRisk(line(), field, stamp);
  check(!risk_line.message.points.empty() && risk_line.message.colors.size() ==
        risk_line.message.points.size(), "external canonical provider prepares colored geometry");
  const auto heatmap = viz::makeRiskHeatmap(field, config, stamp);
  check(heatmap.message.layers == std::vector<std::string>({"elevation", "color"}) &&
        heatmap.message.data[0].data.size() > 1,
        "external risk view prepares an owned elevation/color grid");
  const auto risk_field = viz::makeRiskField(field, config, stamp);
  check(risk_field.markers.size() > 1 && risk_field.markers.front().action ==
        visualization_msgs::msg::Marker::DELETEALL,
        "risk field owns one complete replaceable marker array");

  std::weak_ptr<int> provider_lifetime;
  viz::Prepared<std_msgs::msg::Float64MultiArray> detached;
  {
    auto token = std::make_shared<int>(0);
    provider_lifetime = token;
    auto borrowed = field;
    borrowed.value = [token](size_t, const Eigen::Vector3d &) { ++*token; return 0.25; };
    detached = viz::makeRiskProfile(line(), borrowed, config);
    check(*token > 0, "builder queries provider synchronously");
  }
  check(provider_lifetime.expired() && !detached.message.data.empty(),
        "returned payload retains neither callback nor provider owner");

  std::vector<path_manager::RiskZone> none;
  viz::RiskView empty{none, {}, {}, {}, {}};
  check(viz::makeRiskProfile({}, empty, {}).message.data.empty(),
        "empty trajectory clears profile without unused settings/providers");
  check(viz::makeTrajectoryRisk({}, empty, stamp).message.action ==
            visualization_msgs::msg::Marker::DELETE,
        "empty trajectory clears colored line");
  const auto empty_grid = viz::makeRiskHeatmap(empty, {}, stamp).message;
  check(empty_grid.data.size() == 2 && empty_grid.data.front().data.size() == 1 &&
        std::isnan(empty_grid.data.front().data.front()),
        "empty field replaces retained heatmap with NaN stub");

  rejects([&] { viz::makeTrajectoryRisk(line(std::numeric_limits<double>::infinity()), field, stamp); },
          "infinite duration is rejected before sample conversion");
  rejects([&] { viz::makeTrajectoryTube(line(std::numeric_limits<double>::quiet_NaN()), 0, 1, "", 0, 0, 0, 1, stamp); },
          "NaN duration is rejected before sampling loop");
  auto invalid = line();
  auto bad_coefficients = invalid[0].getCoeffMat();
  bad_coefficients(0, 0) = std::numeric_limits<double>::quiet_NaN();
  poly_traj::Trajectory bad({2.0}, {bad_coefficients});
  rejects([&] { viz::makeRiskProfile(bad, field, config); },
          "nonfinite coefficients cannot escape as prepared geometry");
  rejects([&] { viz::makeTrajectoryRisk(line(1e20), field, stamp); },
          "unrepresentable sample count is rejected");
  rejects([&] { viz::makeTrajectoryTube(line(1e300), 0, 1, "", 0, 0, 0, 1, stamp); },
          "finite huge tube duration is rejected before size conversion");
  auto invalid_settings = config; invalid_settings.heatmap_max_dim = 0;
  rejects([&] { viz::makeRiskHeatmap(field, invalid_settings, stamp); },
          "zero heatmap dimension is rejected");
  invalid_settings = config; invalid_settings.vertical_ratio = std::numeric_limits<double>::quiet_NaN();
  rejects([&] { viz::makeRiskField(field, invalid_settings, stamp); },
          "nonfinite geometric setting is rejected");
  invalid_settings = config; invalid_settings.max_velocity = 1e308;
  rejects([&] { viz::makeRiskProfile(line(), field, invalid_settings); },
          "overflowed profile count is rejected");
  auto missing = field; missing.value = {};
  rejects([&] { viz::makeTrajectoryRisk(line(), missing, stamp); },
          "required provider absence is diagnosed");
  rejects([&] { viz::makeRiskField(field, config, {}); },
          "required clock absence is diagnosed");
  auto throwing = field;
  throwing.value = [](size_t, const Eigen::Vector3d &) -> double { throw std::runtime_error("provider failure"); };
  bool propagated = false;
  try { viz::makeTrajectoryRisk(line(), throwing, stamp); }
  catch (const std::runtime_error &e) { propagated = std::string(e.what()) == "provider failure"; }
  check(propagated, "provider exceptions propagate without a partial result");
  std::cout << "failures=" << failures << '\n';
  return failures ? 1 : 0;
}
