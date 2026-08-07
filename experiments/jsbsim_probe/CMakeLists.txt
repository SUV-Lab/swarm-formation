# [CONTRACT-2 EVAL] Standalone experiment — NOT part of the ROS2 build.
# Deliberately outside the colcon workspace graph: it depends on JSBSim,
# which the planner does not, and nothing in src/ may start linking against
# it before the transition model spec exists.
#
#   ./run.sh            fetch + build the pinned JSBSim, build, run both
#   ./run.sh --version v1.2.1   re-run against another tag
cmake_minimum_required(VERSION 3.16)
project(jsbsim_probe CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Where run.sh installed JSBSim. Override with -DJSBSIM_ROOT=...
set(JSBSIM_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.jsbsim/inst"
    CACHE PATH "JSBSim install prefix")

find_path(JSBSIM_INCLUDE_DIR FGFDMExec.h
          HINTS "${JSBSIM_ROOT}/include/JSBSim" NO_DEFAULT_PATH)
find_library(JSBSIM_LIBRARY NAMES JSBSim
             HINTS "${JSBSIM_ROOT}/lib" NO_DEFAULT_PATH)
if(NOT JSBSIM_INCLUDE_DIR OR NOT JSBSIM_LIBRARY)
  message(FATAL_ERROR
    "JSBSim not found under ${JSBSIM_ROOT}. Run ./run.sh, which fetches and "
    "builds the pinned version, or pass -DJSBSIM_ROOT=<prefix>.")
endif()

find_package(Eigen3 QUIET)
if(NOT Eigen3_FOUND)
  set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3")
endif()

foreach(prog jsbsim_probe hermite_check)
  add_executable(${prog} ${prog}.cpp)
  target_include_directories(${prog} PRIVATE
    "${JSBSIM_INCLUDE_DIR}" "${EIGEN3_INCLUDE_DIR}")
  target_link_libraries(${prog} PRIVATE "${JSBSIM_LIBRARY}")
  # RPATH, not LD_LIBRARY_PATH — and RELATIVE ($ORIGIN), not absolute
  # (review find, round 2): an absolute RPATH bakes in the CONTAINER path
  # (/ws/...), so the same bind-mounted binary run from the HOST path
  # (/home/.../ws) could not find libJSBSim. The workspace layout is
  # identical on both sides of the mount, so a binary-relative path works
  # everywhere.
  file(RELATIVE_PATH _rel_lib "${CMAKE_BINARY_DIR}" "${JSBSIM_ROOT}/lib")
  set_target_properties(${prog} PROPERTIES
    BUILD_RPATH "\$ORIGIN/${_rel_lib}"
    INSTALL_RPATH "\$ORIGIN/${_rel_lib}")
endforeach()
