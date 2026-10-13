// Per-call ROS adapter settings must reach the actual route/optimizer session
// without becoming shared parameters or retained canonical query providers.
#include "path_manager/direct_plan_settings.h"
#include "path_manager/path_manager.h"
#include "path_manager/builtin_route_search.h"
#include "path_manager/ros_route_search.h"

#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace pm = path_manager;
namespace {
int failures = 0;
void check(bool condition, const std::string &message)
{
  std::cout << (condition ? "[OK] " : "[FAIL] ") << message << '\n';
  if (!condition) ++failures;
}

std::vector<double> coefficients(const poly_traj::Trajectory &trajectory)
{
  std::vector<double> values;
  for (const auto &piece : trajectory) {
    values.push_back(piece.getDuration());
    const auto &matrix = piece.getCoeffMat();
    values.insert(values.end(), matrix.data(), matrix.data() + matrix.size());
  }
  return values;
}

grid_map_msgs::msg::GridMap::SharedPtr flatTerrain()
{
  auto map = std::make_shared<grid_map_msgs::msg::GridMap>();
  constexpr int rows = 180, cols = 150;
  map->info.resolution = 2.0;
  map->info.length_x = 360.0;
  map->info.length_y = 300.0;
  map->info.pose.position.x = 180.0;
  map->info.pose.position.y = 150.0;
  map->info.pose.orientation.w = 1.0;
  map->layers = {"elevation"};
  map->data.resize(1);
  auto &layer = map->data.front();
  layer.layout.dim.resize(2);
  layer.layout.dim[0].label = "column_index";
  layer.layout.dim[0].size = cols;
  layer.layout.dim[0].stride = cols * rows;
  layer.layout.dim[1].label = "row_index";
  layer.layout.dim[1].size = rows;
  layer.layout.dim[1].stride = rows;
  layer.data.assign(rows * cols, 2.0f);
  return map;
}

class SearchFailure : public std::runtime_error {
 public:
  SearchFailure() : std::runtime_error("search failure sentinel") {}
};

class ObservedBackend final : public pm::RouteSearchBackend {
 public:
  enum class Mode { NORMAL, THROW_TYPED, THROW_UNTYPED, NO_ROUTE };
  explicit ObservedBackend(pm::BuiltinRouteSearchConfig config)
      : delegate_(pm::makeBuiltinRouteSearchBackend(std::move(config))) {}

  pm::RouteSearchResult search(const pm::RouteSearchProblem &problem) override
  {
    ++calls;
    fpa = problem.constraints.flight_path_angle_max_rad;
    model = problem.model;
    model_fpa = problem.model ? problem.model->limits().flight_path_angle_max_rad : -1.0;
    visibility = problem.environment.risk_visibility(0, probe);
    canonical_front_during_search = front_query();
    if (mode == Mode::THROW_TYPED) throw SearchFailure();
    if (mode == Mode::THROW_UNTYPED) throw 73;
    if (mode == Mode::NO_ROUTE) return {};
    return delegate_->search(problem);
  }

  Mode mode{Mode::NORMAL};
  size_t calls{0};
  double fpa{0.0}, model_fpa{0.0}, visibility{0.0};
  double canonical_front_during_search{0.0};
  Eigen::Vector3d probe{66.0, 40.0, 2.05};
  std::weak_ptr<const mmp_vehicle_dynamics::VehicleModel> model;
  std::function<double()> front_query;
 private:
  std::shared_ptr<pm::RouteSearchBackend> delegate_;
};

void exercise(const char *config)
{
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", config});
  options.parameter_overrides({
      rclcpp::Parameter("manager/front_end", std::string("fm2")),
      rclcpp::Parameter("manager/risk_grounded", true),
      rclcpp::Parameter("manager/risk_zone_agl", false),
      rclcpp::Parameter("manager/risk_terrain_mask_enable", true),
      rclcpp::Parameter("optimization/risk_parallel_threads", 1)});
  auto node = std::make_shared<rclcpp::Node>("direct_plan_settings_test", options);
  node->declare_parameter("drone_id", 7);
  pm::PathManager manager(node);
  manager.initOptimizer();
  manager.deliverTrajToOptimizer();
  manager.setTerrainData(flatTerrain());
  check(manager.setRiskZonesRuntime({{Eigen::Vector3d(60.0, 40.0, 2.0), 24.0, 0.8}}),
        "distant grounded risk fixture installed");
  auto backend = std::make_shared<ObservedBackend>(pm::readRosBuiltinRouteSearchConfig(*node));
  backend->front_query = [&] { return manager.getFrontEndRisk(backend->probe); };
  manager.setRouteSearchBackend(backend);

  pm::StartHead head;
  head.src = pm::StartStateSource::STATED_VECTOR;
  head.pos_u = Eigen::Vector3d(30.0, 150.0, 6.0);
  head.vel_u = Eigen::Vector3d(2.0, 0.0, 0.0);
  const std::vector<Eigen::Vector3d> goal{{330.0, 150.0, 4.0}};
  const auto mission_model = manager.vehicleModel();
  const double mission_visibility = manager.getRiskVisibility(0, backend->probe);
  const double mission_front = manager.getFrontEndRisk(backend->probe);
  const double mission_risk = manager.getEffectiveRisk(0, backend->probe);
  Eigen::Vector3d gradient_position, gradient_velocity;
  double mission_cost = 0.0;
  const bool mission_cost_active = manager.getBackEndRiskCost(
      backend->probe, head.vel_u, gradient_position, gradient_velocity, mission_cost);
  check(mission_front > 0.0 && mission_cost_active && mission_cost > 0.0,
        "risk probes reach a nonzero canonical field and real optimizer cost");

  std::vector<std::string> keys;
  for (const auto &name : node->list_parameters({}, 0).names)
    if (name.rfind("optimization/", 0) == 0) keys.push_back(name);
  const auto pristine = node->get_parameters(keys);
  size_t parameter_writes = 0;
  const auto watch = node->add_on_set_parameters_callback(
      [&](const std::vector<rclcpp::Parameter> &changes) {
        for (const auto &change : changes)
          // First-use declarations invoke this callback before insertion;
          // later updates to every declared optimization key are forbidden.
          if (change.get_name().rfind("optimization/", 0) == 0 &&
              node->has_parameter(change.get_name())) ++parameter_writes;
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
      });
  const auto canonical_fields_match = [&]() {
    double cost = 0.0;
    const bool active = manager.getBackEndRiskCost(
        backend->probe, head.vel_u, gradient_position, gradient_velocity, cost);
    return manager.vehicleModel() == mission_model &&
        manager.getRiskVisibility(0, backend->probe) == mission_visibility &&
        manager.getEffectiveRisk(0, backend->probe) == mission_risk &&
        manager.getFrontEndRisk(backend->probe) == mission_front &&
        active == mission_cost_active && cost == mission_cost;
  };
  const auto canonical_unchanged = [&](const std::string &label) {
    check(parameter_writes == 0 && node->get_parameters(keys) == pristine,
          label + ": no transient parameter writes and exact typed mission values");
    check(canonical_fields_match(),
          label + ": canonical model and front/back/manager risk remain exact");
  };

  const auto baseline = manager.computeGlobalTrajectory(head, goal);
  check(baseline.has_value(), "ordinary session solves the real flat-terrain fixture");
  if (!baseline) return;
  const auto baseline_coefficients = coefficients(baseline->trajectory);
  const auto baseline_reference = coefficients(baseline->reference_trajectory);
  const auto next_normal_exact = [&](const std::string &label) {
    backend->mode = ObservedBackend::Mode::NORMAL;
    const auto next = manager.computeGlobalTrajectory(head, goal);
    check(next && coefficients(next->trajectory) == baseline_coefficients &&
              coefficients(next->reference_trajectory) == baseline_reference,
          label + ": next ordinary solve has exact baseline coefficients and durations");
    canonical_unchanged(label);
  };

  const auto empty = manager.computeGlobalTrajectory(pm::DirectPlanSettings{}, head, goal);
  check(empty && coefficients(empty->trajectory) == baseline_coefficients &&
            coefficients(empty->reference_trajectory) == baseline_reference,
        "empty explicit settings preserve exact seed/final numerics");
  check(backend->model.expired(), "even empty explicit settings own an independent temporary model");
  canonical_unchanged("empty profile");

  pm::DirectPlanSettings settings;
  settings.overrides = {
      rclcpp::Parameter("optimization/dynamics_flight_path_max_deg", 22.0),
      rclcpp::Parameter("optimization/obstacle_clearance", 0.1),
      rclcpp::Parameter("optimization/weight_altitude", 1.0),
      rclcpp::Parameter("optimization/alt_cap_headroom", 1.2),
      rclcpp::Parameter("optimization/alt_floor_headroom", 0.4),
      rclcpp::Parameter("optimization/risk_parallel_threads", 2)};
  const auto profiled = manager.computeGlobalTrajectory(settings, head, goal);
  check(profiled.has_value(), "full direct profile solves with manager settings and private thread budget");
  check(std::abs(backend->fpa - 22.0 * M_PI / 180.0) < 1e-12 &&
            std::abs(backend->model_fpa - backend->fpa) < 1e-12,
        "real route backend receives both the per-call physical cone and per-call vehicle model");
  check(std::abs(backend->visibility - mission_visibility) > 1e-3 &&
            std::abs(backend->canonical_front_during_search - mission_front) > 1e-3,
        "profile clearance changes the actual search visibility and borrowed canonical provider");
  check(backend->model.expired(), "successful profile releases its temporary model");
  canonical_unchanged("successful full profile");
  next_normal_exact("after successful profile");

  pm::DirectPlanSettings malformed;
  malformed.overrides.emplace_back("optimization/alt_cap_headroom", std::string("wrong type"));
  const size_t calls_before_bad = backend->calls;
  bool configuration_error = false;
  try { (void)manager.computeGlobalTrajectory(malformed, head, goal); }
  catch (const pm::DirectPlanConfigurationError &) { configuration_error = true; }
  catch (...) {}
  check(configuration_error && backend->calls == calls_before_bad,
        "malformed typed settings throw DirectPlanConfigurationError before route search");
  canonical_unchanged("configuration exception");
  next_normal_exact("after configuration exception");

  // The public external-backend adapter has always mapped search exceptions
  // to a failed route. Per-call settings must retain that existing boundary,
  // rather than relabel these as optimizer configuration errors.
  backend->mode = ObservedBackend::Mode::THROW_TYPED;
  bool typed_empty = false, typed_escaped = false;
  try { typed_empty = !manager.computeGlobalTrajectory(settings, head, goal); }
  catch (...) { typed_escaped = true; }
  check(typed_empty && !typed_escaped,
        "typed external search exception retains its existing empty-route mapping");
  check(backend->model.expired(), "failed search releases the per-call model");
  canonical_unchanged("search exception");
  next_normal_exact("after search exception");

  backend->mode = ObservedBackend::Mode::THROW_UNTYPED;
  bool untyped_empty = false, untyped_escaped = false;
  try { untyped_empty = !manager.computeGlobalTrajectory(settings, head, goal); }
  catch (...) { untyped_escaped = true; }
  check(untyped_empty && !untyped_escaped,
        "non-standard external search exception retains its existing empty-route mapping");
  canonical_unchanged("non-standard search exception");
  next_normal_exact("after non-standard search exception");

  backend->mode = ObservedBackend::Mode::NO_ROUTE;
  check(!manager.computeGlobalTrajectory(settings, head, goal),
        "no-route result remains an ordinary empty computation");
  canonical_unchanged("no-route return");
  next_normal_exact("after no-route return");

  // A serialized user edit is allowed between missions. New independent
  // workers reread this base, but constructing them must not reconfigure the
  // already initialized primary optimizer or its canonical mission field.
  const std::vector<rclcpp::Parameter> edited_base = {
      rclcpp::Parameter("optimization/obstacle_clearance", 0.1),
      rclcpp::Parameter("optimization/alt_cap_headroom", 1.3),
      rclcpp::Parameter("optimization/alt_floor_headroom", 0.7),
      rclcpp::Parameter("optimization/weight_altitude", 2.0)};
  bool edit_accepted = true;
  for (const auto &result : node->set_parameters(edited_base))
    edit_accepted &= result.successful;
  check(edit_accepted, "serialized user edits to base optimization settings are accepted");
  const size_t intentional_writes = parameter_writes;
  const auto edited_values = node->get_parameters(keys);
  auto worker = manager.makeConfiguredOptimizer();
  const auto slice_config = manager.sliceSolveConfig();
  check(slice_config.alt_cap_headroom == 1.3 &&
            slice_config.alt_floor_headroom == 0.7 &&
            slice_config.weight_altitude == 2.0,
        "new worker slice snapshot uses the same live altitude base as worker construction");
  double worker_cost = 0.0;
  const bool worker_active = worker->inspectRiskCostAt(
      backend->probe, head.vel_u, gradient_position, gradient_velocity, worker_cost);
  check(worker_active && std::abs(worker_cost - mission_cost) > 1e-3,
        "new worker actually consumes edited clearance in its private risk cost");
  check(canonical_fields_match(),
        "worker construction leaves initialized mission model and canonical risk latched");
  const auto unrefreshed = manager.computeGlobalTrajectory(head, goal);
  check(unrefreshed && coefficients(unrefreshed->trajectory) == baseline_coefficients &&
            coefficients(unrefreshed->reference_trajectory) == baseline_reference &&
            canonical_fields_match(),
        "ordinary direct planning keeps its existing initialized mission settings until reinit");

  bool restricted = false;
  try {
    (void)manager.makeConfiguredOptimizer({
        rclcpp::Parameter("optimization/alt_cap_headroom", 9.0)});
  } catch (const std::invalid_argument &) { restricted = true; }
  check(restricted, "full-profile altitude overrides remain outside the four-key worker API");
  check(parameter_writes == intentional_writes &&
            node->get_parameters(keys) == edited_values,
        "worker/slice construction never writes even an already edited base parameter");

  // Reinitialization is explicit; the searcher's retained canonical provider
  // must follow the new mission clearance immediately, before another search.
  worker.reset();
  manager.initOptimizer(true);
  manager.deliverTrajToOptimizer();
  double refreshed_cost = 0.0;
  const bool refreshed_active = manager.getBackEndRiskCost(
      backend->probe, head.vel_u, gradient_position, gradient_velocity, refreshed_cost);
  check(std::abs(manager.getEffectiveRisk(0, backend->probe) - mission_risk) > 1e-3 &&
            std::abs(manager.getFrontEndRisk(backend->probe) -
                     manager.getEffectiveRisk(0, backend->probe)) < 1e-12 &&
            refreshed_active == worker_active && refreshed_cost == worker_cost,
        "explicit reinit refreshes primary risk and retained front-end visibility together");
  check(parameter_writes == intentional_writes &&
            node->get_parameters(keys) == edited_values,
        "explicit reinit reads updated base settings without rewriting the node");
}
}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  if (argc < 2) {
    std::cerr << "usage: direct_plan_settings_test optimizer_params.yaml\n";
    rclcpp::shutdown();
    return 2;
  }
  try { exercise(argv[1]); }
  catch (const std::exception &error) { check(false, std::string("unexpected exception: ") + error.what()); }
  catch (...) { check(false, "unexpected non-standard exception"); }
  rclcpp::shutdown();
  std::cout << (failures ? "FAIL" : "PASS") << ": " << failures << " failed check(s)\n";
  return failures ? 1 : 0;
}
