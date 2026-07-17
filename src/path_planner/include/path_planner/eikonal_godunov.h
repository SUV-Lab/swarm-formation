#ifndef PATH_PLANNER_EIKONAL_GODUNOV_H_
#define PATH_PLANNER_EIKONAL_GODUNOV_H_

// Shared Godunov upwind update for the eikonal |grad T| * F = 1, used by both
// the CPU FMM and the GPU FIM so the two solvers produce identical numbers.
// Mirrors the 1-D/2-D/3-D quadratic in PathSearcher::fm2SolveEikonal::solveQuad.

#include <cmath>

#if defined(__CUDACC__)
  #define PP_HD __host__ __device__
#else
  #define PP_HD
#endif

namespace path_planner {

// Sentinel for "unreached / no finite neighbour". Large but finite so plain
// float arithmetic never produces NaN/Inf.
constexpr float kEikInf = 1e18f;

// Per-axis minimum frozen-neighbour arrival times mx/my/mz (kEikInf if none)
// with per-axis grid spacings hx/hy/hz and slowness slow = 1/F. Solves the
// WEIGHTED Godunov quadratic  sum_i ((T - m_i)/h_i)^2 = slow^2  over the
// causal subset — the anisotropic-voxel generalisation (thin-z grids). With
// hx = hy = hz = h it reduces exactly to the classic isotropic update with
// rhs = h * slow. Returns kEikInf if no finite neighbour exists.
PP_HD inline float eikSolve(float mx, float my, float mz,
                            float hx, float hy, float hz, float slow) {
  // Sort (m, h) pairs so m0 <= m1 <= m2 (3-element sorting network).
  float m0 = mx, m1 = my, m2 = mz;
  float h0 = hx, h1 = hy, h2 = hz;
  float t;
  if (m0 > m1) { t = m0; m0 = m1; m1 = t; t = h0; h0 = h1; h1 = t; }
  if (m1 > m2) { t = m1; m1 = m2; m2 = t; t = h1; h1 = h2; h2 = t; }
  if (m0 > m1) { t = m0; m0 = m1; m1 = t; t = h0; h0 = h1; h1 = t; }

  if (m0 >= kEikInf) return kEikInf;        // no finite neighbour

  // SHIFTED FORM: solve for u = T - m0 with d_i = m_i - m0 — every term is
  // O(h*slow). The naive form kept B ~ T/h^2, so disc = B*B - A*C subtracted
  // two T^2-magnitude floats whose true difference is O(h^2*slow^2): once
  // T >~ 3e3 * h * slow the subtraction was rounding noise, the min-only
  // relaxation kept every NEGATIVE error, and the arrival field drifted
  // hundreds of units below the fixed point at mission scale (measured:
  // 4001-cell corridor -976 u; GPU-vs-CPU divergence 22% on a kFMin
  // serpentine; local dT/dx REVERSED past T ~ 3000 — the noisy-zigzag
  // geodesic regime). Shifted: corridor error 0.046 u and GPU == CPU to
  // <= 1e-3 on the same audit cases.
  float u = slow * h0;                      // 1-axis update
  if (m1 < kEikInf && m0 + u > m1) {
    // 2-axis: w0 u^2 + w1 (u - d1)^2 = slow^2, w_i = 1/h_i^2
    const float w0 = 1.0f / (h0 * h0);
    const float w1 = 1.0f / (h1 * h1);
    const float d1 = m1 - m0;
    float A = w0 + w1;
    float B = w1 * d1;
    float C = w1 * d1 * d1 - slow * slow;
    float disc = B * B - A * C;
    if (disc >= 0.0f) u = (B + sqrtf(disc)) / A;
    if (m2 < kEikInf && m0 + u > m2) {
      // 3-axis
      const float w2 = 1.0f / (h2 * h2);
      const float d2 = m2 - m0;
      A += w2;
      B += w2 * d2;
      C += w2 * d2 * d2;
      disc = B * B - A * C;
      if (disc >= 0.0f) u = (B + sqrtf(disc)) / A;
    }
  }
  return m0 + u;
}

}  // namespace path_planner

#endif  // PATH_PLANNER_EIKONAL_GODUNOV_H_
