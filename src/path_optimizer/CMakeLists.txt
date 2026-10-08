cmake_minimum_required(VERSION 3.5)
project(path_optimizer)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Set build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

# Set optimization flags.
# NOTE: -march=native was tried but triggered a SIGSEGV in Eigen assignment
# (likely an AVX alignment mismatch with path_manager, which doesn't set
# -march=native). Leaving at -O3 only until all packages that share Eigen
# matrices are compiled with consistent vector ISA flags.
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -funroll-loops -Wall -g -DNDEBUG")
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
find_package(Eigen3 REQUIRED)
find_package(swarm_graph REQUIRED)
find_package(path_planner REQUIRED)
find_package(mmp_vehicle_dynamics REQUIRED)
# [RISK-PAR] OpenMP for the per-piece risk-term precompute in
# addPVAGradCost2CT. Raw flags (not OpenMP::OpenMP_CXX) for the same
# plain-signature target_link_libraries reason as path_manager.
find_package(OpenMP)

add_library(${PROJECT_NAME}_lib
  src/poly_traj_optimizer.cpp
  src/route_cost_profile.cpp
  src/optimizer_session.cpp
  ../common/log_manager.cpp
)
# The optimizer's public solve API and logging implementation are usable
# through this target alone; no path_manager link is required.
target_compile_features(${PROJECT_NAME}_lib PUBLIC cxx_std_17)
if(OpenMP_CXX_FOUND)
  target_compile_options(${PROJECT_NAME}_lib PRIVATE ${OpenMP_CXX_FLAGS})
  target_link_libraries(${PROJECT_NAME}_lib ${OpenMP_CXX_LIBRARIES})
endif()

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  Eigen3
  swarm_graph
  path_planner
  mmp_vehicle_dynamics
)

install(TARGETS ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# Required by the installed public optimizer header's relative include.
install(FILES ../common/log_manager.hpp DESTINATION common)

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  # Compile the actual calculation source using only Eigen. This keeps the
  # reusable profile calculations independent of the ROS optimizer adapter.
  add_executable(route_cost_profile_test
    test/route_cost_profile_test.cpp src/route_cost_profile.cpp)
  target_include_directories(route_cost_profile_test PRIVATE include)
  target_link_libraries(route_cost_profile_test Eigen3::Eigen)
  ament_add_test(route_cost_profile_test
    COMMAND $<TARGET_FILE:route_cost_profile_test>
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 30)
  add_executable(optimizer_session_test test/optimizer_session_test.cpp)
  target_link_libraries(optimizer_session_test ${PROJECT_NAME}_lib)
  ament_add_test(optimizer_session_test
    COMMAND $<TARGET_FILE:optimizer_session_test>
      ${CMAKE_CURRENT_SOURCE_DIR}/../path_manager/config/optimizer_params.yaml
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 120)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp Eigen3 swarm_graph path_planner mmp_vehicle_dynamics)

ament_package()
