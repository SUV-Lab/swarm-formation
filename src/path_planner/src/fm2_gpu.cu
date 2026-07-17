// GPU eikonal (Fast Iterative Method, block variant) for the FM2 front-end.
// Built only when CUDA is found (see CMakeLists). Numerically mirrors the CPU
// FMM (shared eikSolve in eikonal_godunov.h) so the extracted geodesic is the
// same path; the FM2 solver calls this when a device is present and falls back
// to the CPU FMM otherwise.
//
// Method: BLOCK-FIM with an active-block mask.
//  * Each thread block loads a TILE (+1 halo) of arrival times into shared
//    memory and runs ITERS Jacobi Godunov sweeps internally, so the wavefront
//    advances ~TILE cells per global round (round count ~ diameter/TILE).
//  * Only blocks flagged active are processed; a block that improves re-flags
//    itself and its 6 face-neighbour blocks for the next round. The flag is a
//    double-buffered char mask (cheap, ~N/TILE^3 entries) — block-granular, so
//    no per-cell atomics. The loop ends when a round improves nothing.
//
// DETERMINISM: within a round, halos are read from a FROZEN snapshot (Tread) and
// results are written to the live buffer (T). Without the snapshot, a block's
// halo load raced its neighbour block's write to the same global cells (no
// inter-block ordering in CUDA), so the arrival field — and hence the extracted
// geodesic — varied run-to-run on identical input (clean 818-pt path one run,
// noisy 3681-pt zigzag the next, tipping the back-end between convergence and
// a -1005 line-search stall). Reading from Tread makes each round a pure
// block-Jacobi step: same input -> bit-identical T -> reproducible geodesic.

#include "path_planner/fm2_gpu.h"
#include "path_planner/eikonal_godunov.h"

#include <cuda_runtime.h>
#include <cstdio>
#include <algorithm>

namespace path_planner {

namespace {

constexpr int TS = 8;            // tile edge (threads per block axis)
constexpr int ITERS = 24;       // internal sweeps per round (>= tile diameter so
                                 // each block converges internally each round)
constexpr int SS = TS + 2;      // shared edge (tile + 1 halo each side)

__global__ void fillKernel(float* a, long N, float v) {
  long c = (long)blockIdx.x * blockDim.x + threadIdx.x;
  if (c < N) a[c] = v;
}

__global__ void fimBlock(float* T, const float* Tread, const float* F,
                         int nx, int ny, int nz,
                         float hx, float hy, float hz,
                         const char* blockActive, char* blockNext,
                         int* changed) {
  const int bx = blockIdx.x, by = blockIdx.y, bz = blockIdx.z;
  const int bid = (bz * gridDim.y + by) * gridDim.x + bx;
  if (!blockActive[bid]) return;          // uniform across the block -> safe

  __shared__ float s[SS][SS][SS];         // [z][y][x]
  __shared__ int improved;

  const int tx = threadIdx.x, ty = threadIdx.y, tz = threadIdx.z;
  const int tid = (tz * TS + ty) * TS + tx;
  if (tid == 0) improved = 0;

  for (int idx = tid; idx < SS * SS * SS; idx += TS * TS * TS) {
    const int sx = idx % SS;
    const int sy = (idx / SS) % SS;
    const int sz = idx / (SS * SS);
    const int wx = bx * TS + sx - 1;
    const int wy = by * TS + sy - 1;
    const int wz = bz * TS + sz - 1;
    float v = kEikInf;
    if (wx >= 0 && wx < nx && wy >= 0 && wy < ny && wz >= 0 && wz < nz)
      v = Tread[(long)wx + nx * ((long)wy + (long)ny * wz)];  // frozen snapshot: no halo race
    s[sz][sy][sx] = v;
  }
  __syncthreads();

  const int gx = bx * TS + tx, gy = by * TS + ty, gz = bz * TS + tz;
  const bool valid = (gx < nx && gy < ny && gz < nz);
  float Fc = 1.0f;
  if (valid) Fc = F[(long)gx + nx * ((long)gy + (long)ny * gz)];
  const bool live = valid && (Fc > 0.0f);
  const float slow = 1.0f / fmaxf(Fc, 1e-6f);
  const int sx = tx + 1, sy = ty + 1, sz = tz + 1;

  for (int it = 0; it < ITERS; ++it) {
    float newT = kEikInf;
    if (live) {
      const float mx = fminf(s[sz][sy][sx - 1], s[sz][sy][sx + 1]);
      const float my = fminf(s[sz][sy - 1][sx], s[sz][sy + 1][sx]);
      const float mz = fminf(s[sz - 1][sy][sx], s[sz + 1][sy][sx]);
      newT = eikSolve(mx, my, mz, hx, hy, hz, slow);
    }
    __syncthreads();
    if (live && newT < s[sz][sy][sx]) s[sz][sy][sx] = newT;
    __syncthreads();
  }

  if (live) {
    const long c = (long)gx + nx * ((long)gy + (long)ny * gz);
    // Re-flag threshold (eikonal convergence eps). A block stops re-flagging once
    // its cells settle to within this, so a LARGER eps freezes more block-grained
    // residual noise into T — and the geodesic, descending T, follows that noise
    // into tortuous high-risk detours (observed: eps 1e-4 produced a 3681-pt
    // zigzag with 5x the risk exposure that the back-end could not smooth, tipping
    // it to a -1005 stall). Tightened 1e-4 -> 1e-6 so the field relaxes closer to
    // the true solution and the geodesic follows the real (weak, in-band)
    // z-gradients instead of noise. Costs a few more relaxation rounds (still well
    // under max_iter); acceptable on capable hardware.
    // Compare against the frozen snapshot (== T[c] at round start; this block is
    // the only writer of cell c), then commit the improvement to the live buffer.
    if (s[sz][sy][sx] < Tread[c] - 1e-6f) { T[c] = s[sz][sy][sx]; improved = 1; }
  }
  __syncthreads();

  if (tid == 0 && improved) {
    *changed = 1;
    const long gxy = (long)gridDim.x * gridDim.y;
    blockNext[bid] = 1;
    if (bx > 0)              blockNext[bid - 1] = 1;
    if (bx < gridDim.x - 1)  blockNext[bid + 1] = 1;
    if (by > 0)              blockNext[bid - gridDim.x] = 1;
    if (by < gridDim.y - 1)  blockNext[bid + gridDim.x] = 1;
    if (bz > 0)              blockNext[bid - gxy] = 1;
    if (bz < gridDim.z - 1)  blockNext[bid + gxy] = 1;
  }
}

#define PP_CK(x) do { if ((x) != cudaSuccess) { cudaGetLastError(); freeAll(); return false; } } while (0)

}  // namespace

bool fm2CudaAvailable() {
  int n = 0;
  if (cudaGetDeviceCount(&n) != cudaSuccess) { cudaGetLastError(); return false; }
  return n > 0;
}

bool fm2EikonalGPU(const float* F, int nx, int ny, int nz,
                   float hx, float hy, float hz,
                   int gi, int gj, int gk, float* T) {
  if (nx <= 0 || ny <= 0 || nz <= 0) return false;
  if (gi < 0 || gi >= nx || gj < 0 || gj >= ny || gk < 0 || gk >= nz) return false;

  const long N = (long)nx * ny * nz;
  const long gflat = gi + (long)nx * (gj + (long)ny * gk);

  const int GX = (nx + TS - 1) / TS, GY = (ny + TS - 1) / TS, GZ = (nz + TS - 1) / TS;
  const long NB = (long)GX * GY * GZ;

  float* d_F = nullptr;
  float* d_T = nullptr;
  float* d_Tread = nullptr;  // frozen per-round snapshot of d_T (halo reads)
  char*  d_ba = nullptr;
  char*  d_bn = nullptr;
  int*   d_changed = nullptr;
  auto freeAll = [&]() {
    cudaFree(d_F); cudaFree(d_T); cudaFree(d_Tread);
    cudaFree(d_ba); cudaFree(d_bn); cudaFree(d_changed);
  };

  PP_CK(cudaMalloc(&d_F, N * sizeof(float)));
  PP_CK(cudaMalloc(&d_T, N * sizeof(float)));
  PP_CK(cudaMalloc(&d_Tread, N * sizeof(float)));
  PP_CK(cudaMalloc(&d_ba, NB));
  PP_CK(cudaMalloc(&d_bn, NB));
  PP_CK(cudaMalloc(&d_changed, sizeof(int)));

  PP_CK(cudaMemcpy(d_F, F, N * sizeof(float), cudaMemcpyHostToDevice));

  fillKernel<<<(N + 255) / 256, 256>>>(d_T, N, kEikInf);
  const float zero = 0.0f;
  PP_CK(cudaMemcpy(d_T + gflat, &zero, sizeof(float), cudaMemcpyHostToDevice));
  if (F[gflat] <= 0.0f) {
    const float half = 0.5f;
    PP_CK(cudaMemcpy(d_F + gflat, &half, sizeof(float), cudaMemcpyHostToDevice));
  }

  PP_CK(cudaMemset(d_ba, 0, NB));
  PP_CK(cudaMemset(d_bn, 0, NB));
  // Seed: activate the block containing the goal.
  const long gblock = ((long)(gk / TS) * GY + (gj / TS)) * GX + (gi / TS);
  const char one = 1;
  PP_CK(cudaMemcpy(d_ba + gblock, &one, 1, cudaMemcpyHostToDevice));

  const dim3 block(TS, TS, TS);
  const dim3 grid(GX, GY, GZ);
  // Round budget must scale with worst-case GEODESIC length, not grid
  // perimeter: the frozen-halo design advances the wavefront exactly one
  // TS-block layer per round, so a winding corridor needs ~path_cells/TS
  // rounds — a 40-leg serpentine needed ~2000 rounds against the old
  // (nx+ny+nz)/TS + 1000 cap and silently fell back to the ~17x slower CPU
  // FMM while still converging. Total block count bounds any simple path;
  // the loop exits early via changed==0, so the larger cap costs nothing
  // on normal maps.
  const long total_blocks = (long)GX * GY * GZ;
  const int max_iter = (int)std::min<long>(total_blocks + 1000, 1000000);
  int iter = 0;
  for (; iter < max_iter; ++iter) {
    PP_CK(cudaMemset(d_changed, 0, sizeof(int)));
    PP_CK(cudaMemset(d_bn, 0, NB));
    // Freeze this round's field: halos are read from d_Tread, writes go to d_T,
    // so no block's halo load can race a neighbour's write -> deterministic.
    PP_CK(cudaMemcpy(d_Tread, d_T, N * sizeof(float), cudaMemcpyDeviceToDevice));
    fimBlock<<<grid, block>>>(d_T, d_Tread, d_F, nx, ny, nz, hx, hy, hz, d_ba, d_bn, d_changed);
    int changed = 0;
    PP_CK(cudaMemcpy(&changed, d_changed, sizeof(int), cudaMemcpyDeviceToHost));
    char* tmp = d_ba; d_ba = d_bn; d_bn = tmp;
    if (!changed) break;
  }

  PP_CK(cudaMemcpy(T, d_T, N * sizeof(float), cudaMemcpyDeviceToHost));
  freeAll();
  if (iter >= max_iter) {
    // The caller falls back to the CPU FMM (correct but ~17x slower) —
    // that latency cliff must never be silent.
    fprintf(stderr,
            "[FM2-GPU] FIM hit round cap %d while still converging "
            "(%dx%dx%d grid) — falling back to CPU FMM\n",
            max_iter, nx, ny, nz);
    return false;
  }
  return true;
}

}  // namespace path_planner
