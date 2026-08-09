cmake_minimum_required(VERSION 3.22)
project(path_planner LANGUAGES CXX)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
  add_compile_options(-O3 -funroll-loops)
  # NOTE: -march=native was tried but caused Eigen AVX alignment SIGSEGV
  # when matrices flowed between packages compiled with different vector ISA.
  # All packages sharing Eigen types need identical flags or this breaks.
  # -ffast-math intentionally omitted: SDFManager uses std::isfinite to
  # identify out-of-map / unobserved voxels, which -ffast-math may constant-fold.
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
  message(STATUS "OpenMP found and enabled")
endif()

# Optional CUDA: enables the GPU eikonal (Fast Iterative Method). When absent,
# the build is CPU-only and FM2 uses the CPU FMM (no behaviour change).
find_package(CUDAToolkit QUIET)
set(PP_HAVE_CUDA OFF)
if(CUDAToolkit_FOUND)
  enable_language(CUDA)
  set(PP_HAVE_CUDA ON)
  message(STATUS "CUDA ${CUDAToolkit_VERSION} found - GPU eikonal enabled (sm_120)")
else()
  message(STATUS "CUDA not found - GPU eikonal disabled (CPU FMM only)")
endif()

# =====================================================================
# path_planner_sdf : CPU ESDF (FH + OpenMP) + flat binary save/load
# =====================================================================
add_library(${PROJECT_NAME}_sdf
    src/sdf/sdf_manager.cpp
)
target_include_directories(${PROJECT_NAME}_sdf PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_sdf
    PUBLIC
        Eigen3::Eigen
)
if(OpenMP_CXX_FOUND)
  # PRIVATE so OpenMP doesn't leak into downstream targets that link this lib.
  target_link_libraries(${PROJECT_NAME}_sdf PRIVATE OpenMP::OpenMP_CXX)
endif()
set_target_properties(${PROJECT_NAME}_sdf PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

# =====================================================================
# path_planner_lib : common utilities (log_manager) + A* front-end
# =====================================================================
set(PP_LIB_SOURCES
  ../common/log_manager.cpp
  src/dyn_a_star.cpp
)
if(PP_HAVE_CUDA)
  list(APPEND PP_LIB_SOURCES src/fm2_gpu.cu)
endif()
add_library(${PROJECT_NAME}_lib ${PP_LIB_SOURCES})

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../common>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  Eigen3
)

# A* calls SDFManager::getDistance/hasData, so it needs the SDF lib.
# Plain-style linkage (ament_target_dependencies above forces plain style).
target_link_libraries(${PROJECT_NAME}_lib ${PROJECT_NAME}_sdf)

if(OpenMP_CXX_FOUND)
  # FM2 speed-map build is per-cell independent -> parallelise it.
  target_link_libraries(${PROJECT_NAME}_lib OpenMP::OpenMP_CXX)
endif()

if(PP_HAVE_CUDA)
  target_compile_definitions(${PROJECT_NAME}_lib PRIVATE PP_HAVE_CUDA)
  # Link cudart by path, NOT the CUDA::cudart imported target: ament exports
  # this target's link interface, and an imported CUDA target would force every
  # downstream package to find_package(CUDAToolkit). A plain .so path just rides
  # the link line of the final executable and resolves at runtime.
  target_link_libraries(${PROJECT_NAME}_lib ${CUDAToolkit_LIBRARY_DIR}/libcudart.so)
  set_target_properties(${PROJECT_NAME}_lib PROPERTIES CUDA_ARCHITECTURES 120)
endif()

# =====================================================================
# sdf_tool : sanity check binary
# =====================================================================
add_executable(sdf_tool src/sdf/sdf_tool.cpp)
target_link_libraries(sdf_tool PRIVATE ${PROJECT_NAME}_sdf)

# sdf_dynamic_test : exercises the dynamic obstacle layer (addObstacle/...)
add_executable(sdf_dynamic_test src/sdf/sdf_dynamic_test.cpp)
target_link_libraries(sdf_dynamic_test PRIVATE ${PROJECT_NAME}_sdf)

# fm2_gpu_test : standalone GPU eikonal correctness/perf check (CUDA only)
if(PP_HAVE_CUDA)
  add_executable(fm2_gpu_test src/fm2_gpu_test.cpp)
  target_link_libraries(fm2_gpu_test ${PROJECT_NAME}_lib)
  install(TARGETS fm2_gpu_test DESTINATION lib/${PROJECT_NAME})
endif()

# =====================================================================
# Install
# =====================================================================
install(TARGETS ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}_sdf
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS sdf_tool
    DESTINATION lib/${PROJECT_NAME})

install(TARGETS sdf_dynamic_test
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
# OpenMP exported so downstream packages linking path_planner_sdf (via the
# exported target set) can resolve the OpenMP::OpenMP_CXX link dependency.
ament_export_dependencies(rclcpp Eigen3 OpenMP)

ament_package()
