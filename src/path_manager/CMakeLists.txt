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

# [PA-2] Opt-in UndefinedBehaviorSanitizer build.
#   colcon build --cmake-args -DMMP_SANITIZE=ON \
#     --build-base build_ubsan --install-base install_ubsan
# Default OFF, so the Release regression measures exactly what it did before:
# mixing sanitizer objects into the normal tree would silently change the
# thing every other test is calibrated against.
option(MMP_SANITIZE "Build with UndefinedBehaviorSanitizer" OFF)
if(MMP_SANITIZE)
  add_compile_options(-fsanitize=undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=undefined)
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

add_executable(risk_contract_test test/risk_contract_test.cpp)
target_link_libraries(risk_contract_test ${PROJECT_NAME}_lib)
ament_target_dependencies(risk_contract_test
  ${PATH_MANAGER_DEPENDENCIES})

add_executable(terrain_pitch_test test/terrain_pitch_test.cpp)
target_link_libraries(terrain_pitch_test ${PROJECT_NAME}_lib)
ament_target_dependencies(terrain_pitch_test
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

# [V-7] Refusal visibility across the four (enable_debug_logs x file_logging)
# configurations. Redirects its own stderr and reads back what rclcpp really
# wrote — grepping the source for a log string passes even when the macro
# routes it away from the console, which is how this went unnoticed.
add_executable(log_visibility_test test/log_visibility_test.cpp)
target_link_libraries(log_visibility_test ${PROJECT_NAME}_lib)
ament_target_dependencies(log_visibility_test
  ${PATH_MANAGER_DEPENDENCIES})

# [V-8] Unit-consistency guard: it must read the values it compares. The guard
# probed with has_parameter before the optimizer had declared them, so it
# compared 100.0 against 100.0 forever.
add_executable(unit_guard_test test/unit_guard_test.cpp)
target_link_libraries(unit_guard_test ${PROJECT_NAME}_lib)
ament_target_dependencies(unit_guard_test
  ${PATH_MANAGER_DEPENDENCIES})

# [V-6] Parameter-read contract: drone_id reaches the optimizer, a missing
# drone_id is refused rather than defaulted, and an undeclared parameter is
# reported instead of silently becoming a setting.
add_executable(param_contract_test test/param_contract_test.cpp)
target_link_libraries(param_contract_test ${PROJECT_NAME}_lib)
ament_target_dependencies(param_contract_test
  ${PATH_MANAGER_DEPENDENCIES})

# [V-2] Command-adoption harness: stands up a real ReplanFSM and drives its
# real callbacks, then reads the PUBLISHED trajectory. It is the first thing
# in this tree that links replan_fsm.o at all — every other test binary has
# zero ReplanFSM symbols, which is why trajectoryCommandCallback had never
# been executed by anything. Manual harness (real solves), like
# chain_experiment_test; ctest registration is decided separately.
add_executable(mission_sequence_test test/mission_sequence_test.cpp)
target_link_libraries(mission_sequence_test ${PROJECT_NAME}_lib)
ament_target_dependencies(mission_sequence_test
  ${PATH_MANAGER_DEPENDENCIES})


# Register the self-contained tests with ctest. They existed for months as
# plain executables that `colcon test` never ran — a regression net that
# catches nothing.
#
# The two BIGGEST nets used to be excluded here on the grounds that they run
# real solves. That premise was measured and does not hold: the 69-variant
# chain sweep is 28.8 s and the 154-case risk harness is 0.54 s, against
# 1.7 s for the five that were registered. Thirty seconds is not a reason to
# leave the parts of the system that actually plan outside the regression net
# — and while they were outside it, a wrong start position reached a supplier
# contract tag.
#
# Both are registered through their DRIVER SCRIPTS, not their binaries. The
# scripts already are the canonical invocation (argv order, fixture path,
# variant list), and registering the binaries directly would create a second
# definition of how to run them that drifts from the first.
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
  # [PA-1/PA-7] The risk CONTRACT — properties, cross-layer identity, and peak
  # admissibility. Deliberately does not pin the exponent; see the file header.
  ament_add_test(risk_contract_test
    COMMAND $<TARGET_FILE:risk_contract_test>
      ${CMAKE_CURRENT_SOURCE_DIR}/config/optimizer_params.yaml
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 300)
  # The terrain repairer and the terrain judge must share one stride. Pure
  # math on a synthetic heightmap: fast, hermetic, no map, no node peers.
  ament_add_test(terrain_pitch_test
    COMMAND $<TARGET_FILE:terrain_pitch_test>
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 120)
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

  ament_add_test(unit_guard_test
    COMMAND $<TARGET_FILE:unit_guard_test>
      ${CMAKE_CURRENT_SOURCE_DIR}/config/optimizer_params.yaml
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 120)

  ament_add_test(param_contract_test
    COMMAND $<TARGET_FILE:param_contract_test>
      ${CMAKE_CURRENT_SOURCE_DIR}/config/optimizer_params.yaml
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 120)

  # [V-2] The command-adoption harness. It was left unregistered when it was
  # written, which cost us: merging the abort layer changed replan_fsm.cpp by
  # 179 lines and `colcon test` could not tell us whether adoption still
  # worked, because the one binary that exercises it was not in the suite.
  # 31 s, three real solves.
  ament_add_test(mission_sequence_test
    COMMAND $<TARGET_FILE:mission_sequence_test>
      ${CMAKE_CURRENT_SOURCE_DIR}/config/optimizer_params.yaml
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 300)

  # [V-7] Four configuration combinations, ~22 s. Registered because the
  # defect it pins is invisible by construction: every string it looks for
  # exists in the source either way, and only the DESTINATION changes.
  ament_add_test(log_visibility_test
    COMMAND $<TARGET_FILE:log_visibility_test>
      ${CMAKE_CURRENT_SOURCE_DIR}/config/optimizer_params.yaml
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 300)

  # The 69-variant chain sweep. The binary takes ONE variant per invocation
  # and has no "all" switch, so registering $<TARGET_FILE:chain_experiment_test>
  # would silently run the baseline variant alone — 1 of 69, reported green.
  # The script derives the variant list from the source, which is what keeps a
  # newly added variant from quietly stopping being run.
  #
  # Measured 28.8 s. TIMEOUT is well above that because a variant that hangs
  # should be reported as this test failing, not as ctest killing the sweep
  # mid-list with no tally.
  # The workspace is passed explicitly: an EMPTY argument is dropped by the
  # time this reaches the shell, which shifts every later positional and made
  # the outdir arrive as the workspace. The script validates what it is given
  # and falls back to its own location, so a wrong path here self-corrects.
  get_filename_component(MMP_WS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../.."
    ABSOLUTE)
  ament_add_test(chain_variants
    COMMAND bash
      ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/run_chain_variants.sh
      ${MMP_WS_DIR} ${CMAKE_CURRENT_BINARY_DIR}/chain_variants
      $<TARGET_FILE:chain_experiment_test>
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 900)

  # The 154-case risk harness. Its fixture path is resolved RELATIVE TO CWD
  # inside the binary, and ament runs tests from the build directory, so a
  # bare registration of the binary exits 2 on a missing fixture. The driver
  # passes the fixture absolutely and pins the positional argv.
  #
  # Measured 0.54 s.
  ament_add_test(risk_scenarios
    COMMAND bash
      ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/run_risk_harness.sh
      NONE $<TARGET_FILE:risk_scenarios_test>
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

install(TARGETS risk_contract_test
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS terrain_pitch_test
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

# [PA-2] Under MMP_SANITIZE, a sanitizer diagnostic must FAIL the test that
# produced it. By default UBSan prints and continues, so an instrumented suite
# would still report 86/0/0 with undefined behaviour on stderr -- green, and
# worthless as evidence. halt_on_error makes the run abort at the first one.
#
# ASAN_OPTIONS is required, not decoration: without it the process aborts at
# exit 134 on an unrelated new-delete mismatch inside uninstrumented
# librcutils, BEFORE the planner is even constructed, and the run produces no
# UBSan output at all -- which reads as "does not reproduce".
#
# Appended to each test's existing ENVIRONMENT rather than assigned, so
# anything ament already set there survives.
if(MMP_SANITIZE)
  set(MMP_SAN_ENV
    "UBSAN_OPTIONS=halt_on_error=1:abort_on_error=1:print_stacktrace=0"
    "ASAN_OPTIONS=new_delete_type_mismatch=0:detect_leaks=0")
  get_property(_mmp_tests DIRECTORY PROPERTY TESTS)
  foreach(_t IN LISTS _mmp_tests)
    get_test_property(${_t} ENVIRONMENT _prev)
    if(_prev STREQUAL "NOTFOUND")
      set(_prev "")
    endif()
    set_tests_properties(${_t} PROPERTIES ENVIRONMENT "${_prev};${MMP_SAN_ENV}")
  endforeach()
  message(STATUS "MMP_SANITIZE: ${_mmp_tests}")
endif()

ament_package()
