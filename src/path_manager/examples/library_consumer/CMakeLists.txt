cmake_minimum_required(VERSION 3.16)
project(mmp_installed_api_consumer LANGUAGES CXX)

# Configure after sourcing ROS and the installed MMP workspace. This project
# has no ament build, source-tree include directory, or manual library lookup.
find_package(path_manager REQUIRED)
find_package(mmp_vehicle_dynamics REQUIRED)

add_executable(mission_planner_client mission_planner_example.cpp)
target_link_libraries(mission_planner_client PRIVATE path_manager::path_manager_lib)

add_executable(flight_evaluator_client flight_evaluator_example.cpp)
target_link_libraries(flight_evaluator_client PRIVATE path_manager::path_manager_calculations)

# This source ships with the vehicle package that owns the simulation API.
get_filename_component(_simulation_example
  "${mmp_vehicle_dynamics_DIR}/../examples/trajectory_simulation_example.cpp" ABSOLUTE)
add_executable(trajectory_simulation_client "${_simulation_example}")
target_link_libraries(trajectory_simulation_client PRIVATE
  mmp_vehicle_dynamics::mmp_vehicle_dynamics)

# Message preparation uses ROS message types but does not create a ROS node.
add_executable(planning_visualization_client planning_visualization_example.cpp)
target_link_libraries(planning_visualization_client PRIVATE path_manager::path_manager_lib)
