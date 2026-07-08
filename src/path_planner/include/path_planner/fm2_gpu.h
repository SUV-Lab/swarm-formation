#ifndef PATH_PLANNER_FM2_GPU_H_
#define PATH_PLANNER_FM2_GPU_H_

// GPU eikonal solver entry points. Compiled only when CUDA is available
// (CMake defines PP_HAVE_CUDA and adds fm2_gpu.cu). The FM2 front-end calls
// these when a device is present and falls back to the CPU FMM otherwise.

namespace path_planner {

// True if a usable CUDA device is present at runtime.
bool fm2CudaAvailable();

// Solve the eikonal |grad T| * F = 1 on a regular grid via the Fast Iterative
// Method on the GPU. Returns true on success (T filled), false on any error
// (caller should fall back to the CPU solver).
//
//   F    : speed map, length nx*ny*nz, flat index = i + nx*(j + ny*k).
//          F <= 0 marks a blocked cell (infinite arrival time there).
//   hx/hy/hz : per-axis grid spacing (anisotropic voxels; matches CPU FMM).
//   gi/gj/gk : goal cell (wave source, T = 0).
//   T    : output arrival times (same layout); the function initialises it.
bool fm2EikonalGPU(const float* F, int nx, int ny, int nz,
                   float hx, float hy, float hz,
                   int gi, int gj, int gk, float* T);

}  // namespace path_planner

#endif  // PATH_PLANNER_FM2_GPU_H_
