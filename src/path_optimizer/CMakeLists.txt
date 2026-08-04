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
)
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

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp Eigen3 swarm_graph path_planner mmp_vehicle_dynamics)

ament_package()
