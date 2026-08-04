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
  # Spins its own node + intra-process pubs; needs a working RMW but no
  # external nodes.
  ament_add_test(terrain_risk_mask_test
    COMMAND $<TARGET_FILE:terrain_risk_mask_test>
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 300)
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

install(TARGETS risk_scenarios_test
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS terrain_risk_mask_test
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS chain_experiment_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

set(EXPORT_DEPS rclcpp std_msgs nav_msgs Eigen3 path_optimizer geometry_msgs path_planner mmp_mission_msgs mmp_traj_msgs visualization_msgs grid_map_msgs)
ament_export_dependencies(${EXPORT_DEPS})

ament_package()
