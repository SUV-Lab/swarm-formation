cmake_minimum_required(VERSION 3.5)
project(path_manager)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Set build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

# Set optimization flags  
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(path_optimizer REQUIRED)
find_package(path_planner REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(mmp_mission_msgs REQUIRED)
find_package(mmp_traj_msgs REQUIRED)
find_package(mmp_vehicle_dynamics REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

# This package generates no interfaces of its own. Mission-authoring types
# (TrajectoryCommand, risk zones, dynamic obstacles) live in mmp_mission_msgs;
# the trajectory wire format (PolyTraj) lives in mmp_traj_msgs.

add_library(${PROJECT_NAME}_lib
  src/path_manager.cpp
  src/replan_fsm.cpp
  src/segment_chain_planner.cpp
  src/terminal_phase.cpp
  src/transition_phase.cpp
  src/waypoint_eval.cpp
  ../common/log_manager.cpp
)

set(PATH_MANAGER_DEPENDENCIES
  rclcpp
  std_msgs
  nav_msgs
  Eigen3
  path_optimizer
  path_planner
  geometry_msgs
  mmp_mission_msgs
  mmp_traj_msgs
  mmp_vehicle_dynamics
  visualization_msgs
  grid_map_msgs
)

ament_target_dependencies(${PROJECT_NAME}_lib
  ${PATH_MANAGER_DEPENDENCIES}
)

# Link path_planner_sdf (CPU ESDF library installed by the path_planner package).
# OpenMP is required because the SDF library uses parallel EDT sweeps.
get_filename_component(PATH_PLANNER_PREFIX
    "${path_planner_DIR}/../../.." ABSOLUTE)
find_library(PATH_PLANNER_SDF_LIB
    NAMES path_planner_sdf
    HINTS "${PATH_PLANNER_PREFIX}/lib"
    REQUIRED)
message(STATUS "path_manager: linking ${PATH_PLANNER_SDF_LIB}")

find_package(OpenMP)
# Plain signature (ament_target_dependencies uses plain style, so mixing
# keyword would break). OpenMP compile/link flags are attached as raw flags
# instead of the imported target to avoid leaking the target into our
# ament export set (downstream packages should not need find_package(OpenMP)).
target_link_libraries(${PROJECT_NAME}_lib ${PATH_PLANNER_SDF_LIB})
if(OpenMP_CXX_FOUND)
  target_compile_options(${PROJECT_NAME}_lib PRIVATE ${OpenMP_CXX_FLAGS})
  target_link_libraries(${PROJECT_NAME}_lib ${OpenMP_CXX_LIBRARIES})
endif()

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_lib
)
ament_target_dependencies(${PROJECT_NAME}_node
  ${PATH_MANAGER_DEPENDENCIES}
)


# Standalone sanity test for the DynamicObstacleArray dispatch loop. The array
# type now lives in mmp_mission_msgs, so this target depends on that package
# instead of this one's generated interfaces.
add_executable(load_obstacles_test test/load_obstacles_test.cpp)
ament_target_dependencies(load_obstacles_test mmp_mission_msgs)

# Stage 1 of the initial-state contract: a pure function of the message, so
# this target needs neither the message package nor rclcpp. That is the point
# — the malformed rows are MESSAGE states and are not expressible through the
# planner's own entry point, which is why they had no test before.
add_executable(start_claim_test test/start_claim_test.cpp)
target_include_directories(start_claim_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(start_claim_test Eigen3::Eigen)

# V3 scenario harness for risk-aware A*.
add_executable(risk_scenarios_test test/risk_scenarios_test.cpp)
target_link_libraries(risk_scenarios_test
  ${PROJECT_NAME}_lib
  yaml-cpp)
ament_target_dependencies(risk_scenarios_test
  ${PATH_MANAGER_DEPENDENCIES})

add_executable(terrain_risk_mask_test test/terrain_risk_mask_test.cpp)
target_link_libraries(terrain_risk_mask_test ${PROJECT_NAME}_lib)
ament_target_dependencies(terrain_risk_mask_test
  ${PATH_MANAGER_DEPENDENCIES})

# [CHAIN] Stage-1 experiment: baseline vs chained plan on a synthetic DEM,
# auditing every junction of the stitched trajectory for P/V/A continuity.
# Manual harness (like risk_scenarios_test), NOT registered with ctest: it
# runs segments+1 full L-BFGS solves with the production parameter set.
add_executable(chain_experiment_test test/chain_experiment_test.cpp)
target_link_libraries(chain_experiment_test ${PROJECT_NAME}_lib)
ament_target_dependencies(chain_experiment_test
  ${PATH_MANAGER_DEPENDENCIES})

# Transition generator harness: the model-math variants (analytic trims,
# RK4 convergence, roundtrip consistency, Hermite representation) are pure
# functions and run in milliseconds; the gate/feasibility variants drive
# transition_phase::generate with synthetic closures. No node, no params.
# Waypoint extraction + reproduction harness: pure math + a benchmark
# follower, no node and no solver — fast and hermetic.
# [ABORT] Wire-contract harness: publishes PolyTraj + TrajectoryExecutionControl
# with the production QoS and drives a consumer implementation against them.
# Covers what an in-process test cannot — that a second message goes out, that
# a late joiner gets both, and that an abort binds to one trajectory id.
# [ENV-CHANGE/ABORT] Production plumbing: stands up a real ReplanFSM and
# drives its real callbacks. Manual harness (real solves), like
# chain_experiment_test.
add_executable(fsm_scenarios_test test/fsm_scenarios_test.cpp)
target_link_libraries(fsm_scenarios_test ${PROJECT_NAME}_lib)
ament_target_dependencies(fsm_scenarios_test
  ${PATH_MANAGER_DEPENDENCIES})

add_executable(exec_abort_test test/exec_abort_test.cpp)
target_link_libraries(exec_abort_test ${PROJECT_NAME}_lib)
ament_target_dependencies(exec_abort_test
  ${PATH_MANAGER_DEPENDENCIES})

add_executable(waypoint_experiment_test test/waypoint_experiment_test.cpp)
target_link_libraries(waypoint_experiment_test ${PROJECT_NAME}_lib)
ament_target_dependencies(waypoint_experiment_test
  ${PATH_MANAGER_DEPENDENCIES})

add_executable(transition_experiment_test test/transition_experiment_test.cpp)
target_link_libraries(transition_experiment_test ${PROJECT_NAME}_lib)
ament_target_dependencies(transition_experiment_test
  ${PATH_MANAGER_DEPENDENCIES})


# Register the self-contained tests with ctest. They existed for months as
# plain executables that `colcon test` never ran — a regression net that
# catches nothing. risk_scenarios_test is NOT registered here: it is the
# 154-case harness with fixture/argv conventions, driven by
# tools/run_risk_harness.sh so its invocation stays pinned.
if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  # GENERATE_RESULT_FOR_RETURN_CODE_ZERO: these are plain executables (own
  # check() harness, no gtest), so run_test.py must synthesize the result xml
  # from the exit code instead of expecting the binary to write one.
  ament_add_test(load_obstacles_test
    COMMAND $<TARGET_FILE:load_obstacles_test>
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 60)
  ament_add_test(start_claim_test
    COMMAND $<TARGET_FILE:start_claim_test>
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 60)
  # Spins its own node + intra-process pubs; needs a working RMW but no
  # external nodes.
  ament_add_test(terrain_risk_mask_test
    COMMAND $<TARGET_FILE:terrain_risk_mask_test>
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 300)
  # Pure math + synthetic-closure gates: fast and hermetic, so it IS
  # ctest-registered (unlike the solver-driving chain harness).
  ament_add_test(waypoint_experiment_test
    COMMAND $<TARGET_FILE:waypoint_experiment_test>
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 300)
  ament_add_test(transition_experiment_test
    COMMAND $<TARGET_FILE:transition_experiment_test>
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 120)
endif()

install(TARGETS ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS load_obstacles_test
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS start_claim_test
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS exec_abort_test
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS fsm_scenarios_test
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS risk_scenarios_test
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS terrain_risk_mask_test
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS chain_experiment_test
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS transition_experiment_test
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS waypoint_experiment_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

set(EXPORT_DEPS rclcpp std_msgs nav_msgs Eigen3 path_optimizer geometry_msgs path_planner mmp_mission_msgs mmp_traj_msgs mmp_vehicle_dynamics visualization_msgs grid_map_msgs)
ament_export_dependencies(${EXPORT_DEPS})

ament_package()
