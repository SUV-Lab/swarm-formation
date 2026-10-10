# Installed API consumer

This is a separate CMake project that uses installed MMP targets. It does not
compile MMP implementation sources or include headers from the source checkout.
The planner backend retains ROS dependencies. The evaluator and simulator need
no ROS node at runtime; the current package configuration still requires the
installed ROS workspace.

After building/installing the workspace, run inside the existing container:

```sh
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
cmake -S /ws/install/path_manager/share/path_manager/examples/library_consumer \
  -B /tmp/mmp_api_consumer_build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/mmp_api_consumer_build -j2
/tmp/mmp_api_consumer_build/flight_evaluator_client
/tmp/mmp_api_consumer_build/trajectory_simulation_client
/tmp/mmp_api_consumer_build/planning_visualization_client
/tmp/mmp_api_consumer_build/mission_planner_client \
  /ws/install/path_manager/share/path_manager/config/optimizer_params.yaml
```

Use another build directory when testing a different install. The planner
example uses synthetic terrain and its own mission. It does not load the R2
mission or issue a vehicle execution command. Its numerical result is not an
R2 regression reference.

The displayed target split is:

| Imported target | Entry points |
|---|---|
| `path_manager::path_manager_lib` | `MissionPlanner::plan()` with `RosPlanningBackend`, retained ROS visualization adapters |
| `path_manager::path_manager_calculations` | `evaluateFlight()`, geometry/route/assembly helpers, generic planner facade |
| `mmp_vehicle_dynamics::mmp_vehicle_dynamics` | `simulate()` / `SimulationSession::step()`, model math |
| `path_optimizer::path_optimizer_lib` | Complete `PolyTrajOptimizer::solve()` boundary; no solver decomposition |

`path_manager_lib` transitively supplies `path_manager_calculations`, preserving
existing callers. Calculations still link the existing optimizer's dependencies;
the target split is not a ROS-free packaging guarantee. The simulator source is
installed under `share/mmp_vehicle_dynamics/examples` and uses that package's
exported target, without depending on `path_manager`.

The visualization consumer prepares owned display messages without a ROS node; actual RViz display requires the existing
ROS topics/subscribers. Its expected summary is `route_points=2 tube_points=21 frame=map`. It cannot
synchronously certify that an image was drawn.
`PlanSettings.attach_visualization=false` disables the manager display connection,
including `/viz/*`, `/debug/pipeline`, and `/planning/front_end_path`; it does not
disable the backend, terrain/SDF, or execution behavior.

For evaluator results, require `verdict.evaluated` and `issue == NONE` before
interpreting `clean` or `unflyable()`. The legacy default is `clean=true` even
when `evaluated=false`; an unevaluated result is not evidence of a clean flight.

Map loading is a separate Python API:

```python
from mmp_terrain.map_loader import MapLoadInput, load_map
```

Its installed example and complete input/error contract are
`share/mmp_terrain/examples/load_map_example.py` and
`share/mmp_terrain/docs/map_loader_api_ko.md`. Follow that example's explicit
`--data-dir` and `--mission` arguments; resource discovery must not depend on a
hidden source checkout. Python terrain loading remains Python and does not link
a C++ planner library.
