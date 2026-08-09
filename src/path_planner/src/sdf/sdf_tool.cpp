// Sanity check for SDFManager, including save/load round-trip.

#include <cstdint>
#include <iomanip>
#include <iostream>
#include <vector>

#include "path_planner/sdf/sdf_manager.h"

namespace {

struct Probe {
  Eigen::Vector3d pos;
};

void printRow(const Probe& pr, float d, const Eigen::Vector3d& g, bool ok) {
  std::cout << std::setw(7) << pr.pos.x()
            << "," << std::setw(6) << pr.pos.y()
            << "," << std::setw(6) << pr.pos.z()
            << std::setw(12) << d
            << std::setw(10) << g.x()
            << std::setw(10) << g.y()
            << std::setw(10) << g.z()
            << "  " << (ok ? "OK" : "FAIL") << "\n";
}

}  // namespace

int main() {
  using namespace path_planner::sdf;

  const int N = 20;
  const double voxel = 0.5;

  std::vector<uint8_t> occ(N * N * N, 0);
  auto set = [&](int x, int y, int z, uint8_t v) {
    occ[((size_t)x * N + y) * N + z] = v;
  };
  for (int x = 8; x < 13; ++x)
    for (int y = 8; y < 13; ++y)
      for (int z = 8; z < 13; ++z)
        set(x, y, z, 1);

  SDFManager sdf;
  if (!sdf.initialize(voxel)) { std::cerr << "init failed\n"; return 1; }
  Eigen::Vector3d origin(0, 0, 0);
  if (!sdf.buildFromVoxels(occ.data(), N, N, N, origin)) {
    std::cerr << "build failed\n"; return 1;
  }

  std::cout << "\n=== SDFManager sanity check ===\n";
  std::cout << "voxel_size=" << sdf.voxelSize() << "\n";
  std::cout << "Cube world range: [4.0, 6.5] m each axis.\n\n";

  std::vector<Probe> probes = {
    {{5.0, 5.0, 5.0}},
    {{5.0, 5.0, 7.0}},
    {{5.0, 5.0, 8.0}},
    {{2.0, 5.0, 5.0}},
    {{0.0, 0.0, 0.0}},
    {{9.0, 9.0, 9.0}},
  };

  std::cout << std::fixed << std::setprecision(3);
  std::cout << std::setw(24) << "pos [m]"
            << std::setw(12) << "dist [m]"
            << std::setw(30) << "grad" << "\n";
  std::cout << std::string(66, '-') << "\n";

  std::vector<float> d_before;
  std::vector<Eigen::Vector3d> g_before;
  for (const auto& pr : probes) {
    float d; Eigen::Vector3d g;
    bool ok = sdf.getDistanceAndGradient(pr.pos, &d, &g);
    printRow(pr, d, g, ok);
    d_before.push_back(d);
    g_before.push_back(g);
  }

  // --- Save / load round-trip ---
  const std::string path = "/tmp/mmp_sdf_test.esdf";
  std::cout << "\n=== save -> load round-trip ===\n";
  if (!sdf.saveToFile(path)) { std::cerr << "save failed\n"; return 1; }

  SDFManager sdf2;
  if (!sdf2.loadFromFile(path, Eigen::Vector3d::Zero(),
                         Eigen::Vector3d::Zero())) {
    std::cerr << "load failed\n"; return 1;
  }

  std::cout << std::setw(24) << "pos [m]"
            << std::setw(12) << "dist [m]"
            << std::setw(30) << "grad" << "\n";
  std::cout << std::string(66, '-') << "\n";
  int mismatch = 0;
  for (size_t i = 0; i < probes.size(); ++i) {
    float d; Eigen::Vector3d g;
    bool ok = sdf2.getDistanceAndGradient(probes[i].pos, &d, &g);
    printRow(probes[i], d, g, ok);
    const float tol = 1e-4f;
    if (std::abs(d - d_before[i]) > tol ||
        (g - g_before[i]).norm() > tol) {
      ++mismatch;
    }
  }
  if (mismatch > 0) {
    std::cerr << "\nROUND-TRIP MISMATCH: " << mismatch << " / "
              << probes.size() << " probes differ.\n";
    return 2;
  }
  std::cout << "\nround-trip OK (" << probes.size() << " probes match).\n";
  return 0;
}
