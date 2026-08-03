// V3 scenario harness for risk-aware A*.
// Builds a synthetic flat ESDF in memory (no terrain dependency), loads
// scenarios from yaml, runs A*, prints metrics per scenario.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <yaml-cpp/yaml.h>

#include "path_planner/dyn_a_star.h"
#include "path_planner/sdf/sdf_manager.h"

namespace {

using path_planner::search::PathSearcher;
using path_planner::search::RiskZoneLite;
using path_planner::sdf::SDFManager;

// CLI-selected front end: "astar" or "fm2" (default fm2).
std::string g_front_end = "fm2";
int g_fm2_k = 4;
bool g_fm2_star = true;
double g_barrier = 0.0;   // front-end finite "hard wall" K (7th CLI arg)

int g_passed = 0;
int g_failed = 0;

void check(bool cond, const std::string &label) {
  std::cout << "  " << (cond ? "[OK]   " : "[FAIL] ")
            << std::left << std::setw(48) << label << "\n";
  if (cond) ++g_passed; else ++g_failed;
}

struct ObstacleBox {  // axis-aligned, world meters (inclusive lo, exclusive hi)
  Eigen::Vector3d lo, hi;
};

struct Scenario {
  std::string name;
  Eigen::Vector3d start, goal;
  std::vector<RiskZoneLite> zones;
  std::string expected;
};

struct MapSpec {
  Eigen::Vector3d size = Eigen::Vector3d(200, 200, 20);
  double voxel = 1.0;
  // If non-empty, load this ESDF file instead of building flat free-space.
  // Bounding box [bbox_lo, bbox_hi] is read from the file when set.
  std::string esdf_path;
  Eigen::Vector3d bbox_lo = Eigen::Vector3d(0, 0, 0);
  Eigen::Vector3d bbox_hi = Eigen::Vector3d(1000, 1000, 100);
  std::vector<ObstacleBox> obstacles;  // filled into occ when esdf_path empty
};

struct LoadResult {
  MapSpec map;
  std::vector<Scenario> scenarios;
};

LoadResult load(const std::string &path) {
  YAML::Node root = YAML::LoadFile(path);
  LoadResult out;
  if (root["map"]) {
    auto m = root["map"];
    if (m["size"]) {
      auto s = m["size"];
      out.map.size = Eigen::Vector3d(s[0].as<double>(),
                                     s[1].as<double>(),
                                     s[2].as<double>());
    }
    if (m["voxel"]) out.map.voxel = m["voxel"].as<double>();
    if (m["esdf"]) out.map.esdf_path = m["esdf"].as<std::string>();
    if (m["bbox_lo"]) {
      auto v = m["bbox_lo"];
      out.map.bbox_lo = Eigen::Vector3d(v[0].as<double>(), v[1].as<double>(), v[2].as<double>());
    }
    if (m["bbox_hi"]) {
      auto v = m["bbox_hi"];
      out.map.bbox_hi = Eigen::Vector3d(v[0].as<double>(), v[1].as<double>(), v[2].as<double>());
    }
    if (m["obstacles"]) {
      for (const auto &o : m["obstacles"]) {
        ObstacleBox b;
        auto lo = o["lo"]; auto hi = o["hi"];
        b.lo = Eigen::Vector3d(lo[0].as<double>(), lo[1].as<double>(), lo[2].as<double>());
        b.hi = Eigen::Vector3d(hi[0].as<double>(), hi[1].as<double>(), hi[2].as<double>());
        out.map.obstacles.push_back(b);
      }
    }
  }
  for (const auto &n : root["scenarios"]) {
    Scenario s;
    s.name = n["name"].as<std::string>();
    auto a = n["start"];
    auto b = n["goal"];
    s.start = Eigen::Vector3d(a[0].as<double>(), a[1].as<double>(), a[2].as<double>());
    s.goal  = Eigen::Vector3d(b[0].as<double>(), b[1].as<double>(), b[2].as<double>());
    for (const auto &z : n["zones"]) {
      RiskZoneLite tz;
      auto c = z["center"];
      tz.center = Eigen::Vector3d(c[0].as<double>(), c[1].as<double>(), c[2].as<double>());
      tz.reach  = z["reach"].as<double>();
      tz.peak   = z["peak"].as<double>();
      s.zones.push_back(tz);
    }
    s.expected = n["expected"].as<std::string>();
    out.scenarios.push_back(s);
  }
  return out;
}

double pathLength(const std::vector<Eigen::Vector3d> &p) {
  double L = 0;
  for (size_t i = 1; i < p.size(); ++i) L += (p[i] - p[i-1]).norm();
  return L;
}

// Path-integrated risk metric: sum segment_len * risk(midpoint).
// Uses the same OR-moat formula as PathSearcher::getRiskCost (with alpha=1).
double pathRisk(const std::vector<Eigen::Vector3d> &p,
                const std::vector<RiskZoneLite> &zones) {
  double total = 0.0;
  for (size_t i = 1; i < p.size(); ++i) {
    const auto &a = p[i-1];
    const auto &b = p[i];
    const Eigen::Vector3d m = 0.5 * (a + b);
    double survival = 1.0;
    for (const auto &tz : zones) {
      const double d = (m - tz.center).norm();
      if (d >= tz.reach) continue;
      const double u = 1.0 - d / tz.reach;
      const double moat = std::min(tz.peak * u * u, 1.0 - 1e-3);
      survival *= (1.0 - moat);
    }
    total += (b - a).norm() * (1.0 - survival);
  }
  return total;
}

bool buildFlatSDF(SDFManager &sdf, const MapSpec &map) {
  if (!sdf.initialize(map.voxel)) {
    std::cerr << "SDF init failed\n";
    return false;
  }
  if (!map.esdf_path.empty()) {
    if (!sdf.loadFromFile(map.esdf_path, map.bbox_lo, map.bbox_hi)) {
      std::cerr << "SDF loadFromFile failed: " << map.esdf_path << "\n";
      return false;
    }
    return true;
  }
  const int nx = static_cast<int>(map.size.x() / map.voxel);
  const int ny = static_cast<int>(map.size.y() / map.voxel);
  const int nz = static_cast<int>(map.size.z() / map.voxel);
  std::vector<uint8_t> occ(static_cast<size_t>(nx) * ny * nz, 0);
  // Mark box obstacles occupied (layout ((x*ny)+y)*nz+z, see sdf_manager.h).
  for (const auto &b : map.obstacles) {
    const int x0 = std::max(0, (int)std::floor(b.lo.x() / map.voxel));
    const int y0 = std::max(0, (int)std::floor(b.lo.y() / map.voxel));
    const int z0 = std::max(0, (int)std::floor(b.lo.z() / map.voxel));
    const int x1 = std::min(nx, (int)std::ceil(b.hi.x() / map.voxel));
    const int y1 = std::min(ny, (int)std::ceil(b.hi.y() / map.voxel));
    const int z1 = std::min(nz, (int)std::ceil(b.hi.z() / map.voxel));
    for (int x = x0; x < x1; ++x)
      for (int y = y0; y < y1; ++y)
        for (int z = z0; z < z1; ++z)
          occ[((static_cast<size_t>(x) * ny) + y) * nz + z] = 1;
  }
  Eigen::Vector3d origin(0, 0, 0);
  if (!sdf.buildFromVoxels(occ.data(), nx, ny, nz, origin)) {
    std::cerr << "SDF buildFromVoxels failed\n";
    return false;
  }
  return true;
}

void run(const Scenario &s, double alpha, double h_weight, const MapSpec &map) {
  std::cout << "\n[" << s.name << " alpha=" << alpha
            << " hw=" << h_weight << "]\n";

  SDFManager sdf;
  if (!buildFlatSDF(sdf, map)) {
    check(false, "SDF build");
    return;
  }

  PathSearcher astar;
  Eigen::Vector3d origin(0, 0, 0);
  astar.setSDF(&sdf, origin, map.size, map.voxel);
  astar.setRiskZones(&s.zones);
  astar.setObstacleMargin(0.5);
  astar.setRiskAlpha(alpha);
  astar.setRiskBarrier(g_barrier);
  astar.setSmhaW(h_weight);
  astar.setFrontEnd(g_front_end == "fm2"
      ? path_planner::search::PathSearcher::FrontEnd::FM2
      : path_planner::search::PathSearcher::FrontEnd::ASTAR);
  astar.setFm2CoarseK(g_fm2_k);
  astar.setFm2Star(g_fm2_star);
  // Allow runtime-scale grids in offline debugging (default cap is 8M cells).
  if (const char *mc = std::getenv("FM2_MAX_CELLS"))
    astar.setFm2MaxCells(static_cast<size_t>(std::strtoull(mc, nullptr, 10)));
  // Match the runtime obstacle margin when reproducing live-flight issues.
  if (const char *om = std::getenv("OBSTACLE_MARGIN"))
    astar.setObstacleMargin(std::atof(om));
  // Keep the raw front-end geodesic (no shortcut collapse) so the
  // path-integrated risk metric reflects the actual route taken,
  // not a 4-point straight-line simplification.
  bool bypass_sc = true;
  if (const char *bs = std::getenv("BYPASS_SHORTCUT")) bypass_sc = (std::string(bs) != "0");
  astar.setBypassShortcut(bypass_sc);
  Eigen::Vector3i pool(
      static_cast<int>(map.size.x() / map.voxel),
      static_cast<int>(map.size.y() / map.voxel),
      static_cast<int>(map.size.z() / map.voxel));
  astar.initGridMap(pool);

  auto t0 = std::chrono::steady_clock::now();
  auto path = astar.astarSearchAndGetSimplePath(map.voxel, s.start, s.goal, 0);
  auto t1 = std::chrono::steady_clock::now();
  double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

  check(path.size() >= 2, "A* returned a path");
  if (path.size() < 2) return;

  // Optional grid dump for visualization (set DUMP_GRID=1).
  // Writes raw binary <prefix>_meta.txt + <prefix>_T.bin + <prefix>_G.bin +
  // <prefix>_F.bin + <prefix>_path.txt for python to load.
  if (const char *dump = std::getenv("DUMP_GRID"); dump && std::string(dump) == "1") {
    std::string prefix = std::getenv("DUMP_PREFIX") ? std::getenv("DUMP_PREFIX") : "/tmp/grid_dump";
    std::string tag = g_front_end + "_" + s.name;
    std::string base = prefix + "_" + tag;
    // meta
    {
      std::ofstream ofs(base + "_meta.txt");
      auto fm2d = astar.getFm2Dims();
      auto cgd = astar.getCoarseDims();
      ofs << "front_end " << g_front_end << "\n"
          << "scenario " << s.name << "\n"
          << "alpha " << alpha << "\n"
          << "h_weight " << h_weight << "\n"
          << "fm2_dims " << fm2d.x() << " " << fm2d.y() << " " << fm2d.z() << "\n"
          << "fm2_coarse_k " << astar.getFm2CoarseK() << "\n"
          << "coarse_dims " << cgd.x() << " " << cgd.y() << " " << cgd.z() << "\n"
          << "coarse_k " << astar.getCoarseK() << "\n"
          << "start " << s.start.x() << " " << s.start.y() << " " << s.start.z() << "\n"
          << "goal " << s.goal.x() << " " << s.goal.y() << " " << s.goal.z() << "\n"
          << "voxel " << map.voxel << "\n"
          << "n_zones " << s.zones.size() << "\n";
      for (const auto &z : s.zones) {
        ofs << "zone " << z.center.x() << " " << z.center.y() << " " << z.center.z()
            << " " << z.reach << " " << z.peak << "\n";
      }
    }
    // FM2 T(x)
    {
      const auto &T = astar.getFm2T();
      if (!T.empty()) {
        std::ofstream ofs(base + "_T.bin", std::ios::binary);
        ofs.write(reinterpret_cast<const char*>(T.data()), T.size() * sizeof(float));
      }
    }
    // FM2 speed map F(x)
    {
      const auto &F = astar.getFm2F();
      if (!F.empty()) {
        std::ofstream ofs(base + "_F.bin", std::ios::binary);
        ofs.write(reinterpret_cast<const char*>(F.data()), F.size() * sizeof(float));
      }
    }
    // A* coarse cost-to-go G(x)
    {
      const auto &G = astar.getCoarseG();
      if (!G.empty()) {
        std::ofstream ofs(base + "_G.bin", std::ios::binary);
        ofs.write(reinterpret_cast<const char*>(G.data()), G.size() * sizeof(double));
      }
    }
    // A* fine grid gScore z-slice at the goal z (visualization).
    {
      Eigen::Vector3i pool = astar.getPoolSize();
      int gz = std::clamp(static_cast<int>(s.goal.z() / map.voxel),
                          0, pool.z() - 1);
      auto gslice = astar.getFineGScoreSlice(gz);
      if (!gslice.empty()) {
        std::ofstream ofs(base + "_finegslice.bin", std::ios::binary);
        ofs.write(reinterpret_cast<const char*>(gslice.data()),
                  gslice.size() * sizeof(double));
        // dim 메모: 위 meta 에 fine pool 정보 추가
        std::ofstream m(base + "_finegslice_meta.txt");
        m << "fine_pool " << pool.x() << " " << pool.y() << " " << pool.z() << "\n"
          << "slice_z " << gz << "\n"
          << "voxel " << map.voxel << "\n";
      }
    }
    // Output path
    {
      std::ofstream ofs(base + "_path.txt");
      for (const auto &p : path) {
        ofs << p.x() << " " << p.y() << " " << p.z() << "\n";
      }
    }
    std::cout << "  [dump] wrote " << base << "_*\n";
  }

  double L = pathLength(path);
  double R = pathRisk(path, s.zones);
  double Ldirect = (s.goal - s.start).norm();

  // Maximum in-zone depth: how far inside any zone any waypoint sits.
  double max_depth = 0.0;
  for (const auto &p : path) {
    for (const auto &tz : s.zones) {
      double d = (p - tz.center).norm();
      if (d < tz.reach) {
        double depth = tz.reach - d;
        if (depth > max_depth) max_depth = depth;
      }
    }
  }

  // Geodesic jitter: mean turn angle between consecutive segments.
  // High = the raw path zig-zags (coarse-grid gradient stair-step);
  // useful for comparing fm2_coarse_k settings.
  double turn_sum_deg = 0.0;
  int turn_n = 0;
  for (size_t i = 2; i < path.size(); ++i) {
    Eigen::Vector3d a = path[i-1] - path[i-2];
    Eigen::Vector3d b = path[i]   - path[i-1];
    double na = a.norm(), nb = b.norm();
    if (na < 1e-6 || nb < 1e-6) continue;
    double c = std::clamp(a.dot(b) / (na * nb), -1.0, 1.0);
    turn_sum_deg += std::acos(c) * 180.0 / M_PI;
    ++turn_n;
  }
  double mean_turn = turn_n > 0 ? turn_sum_deg / turn_n : 0.0;

  std::printf("  length=%.1fm direct=%.1fm risk_int=%.3f max_depth=%.1fm time=%.1fms wp=%zu mean_turn=%.2fdeg\n",
              L, Ldirect, R, max_depth, ms, path.size(), mean_turn);

  // Per-zone traversal diagnostic. For every zone, report its peak and
  // whether the path went through it (and how deep). With an asymmetric
  // gate layout this directly shows which gate was used: the chosen
  // gate's zones are traversed, the rejected gate's are not.
  for (size_t zi = 0; zi < s.zones.size(); ++zi) {
    const auto &tz = s.zones[zi];
    double min_d = std::numeric_limits<double>::max();
    int hits = 0;
    for (const auto &p : path) {
      double d = (p - tz.center).norm();
      if (d < min_d) min_d = d;
      if (d < tz.reach) ++hits;
    }
    const bool through = hits > 0;
    const double pen = through ? (tz.reach - min_d) : 0.0;
    std::printf("  zone[%zu] peak=%.2f reach=%.0f : %s (min_d=%.1fm pen=%.1fm)\n",
                zi, tz.peak, tz.reach,
                through ? "THROUGH" : "clear  ", min_d, pen);
  }

  // DETOUR success = path stays mostly outside zones (low risk_int).
  // Threshold scales with direct length so it's resolution-independent.
  //
  // Asserted only at production-representative risk weight: the alpha sweep
  // deliberately includes sub-production weights (production
  // manager/risk_weight = 30), and at alpha <= 3 crossing a wide zone IS the
  // cost-optimal eikonal answer — asserting detour there tested the wrong
  // invariant, and the suite carried 40 permanent "failures" that were the
  // sweep behaving as designed (alpha 10/30 pass every scenario). Weaker
  // alphas stay in the sweep as informational output.
  const double kDetourAssertAlpha = 10.0;
  const double low_risk = 0.05 * Ldirect;
  if (s.expected == "DETOUR") {
    if (alpha >= kDetourAssertAlpha) {
      check(R < low_risk,        "DETOUR: risk_int < 5% of direct");
      check(max_depth < 5.0,     "DETOUR: max_depth < 5m");
    } else {
      std::printf("  [info] sub-production alpha=%.1f: DETOUR not asserted "
                  "(risk_int=%.3f, max_depth=%.1fm)\n", alpha, R, max_depth);
    }
  }
  if (s.expected == "TRANSIT") {
    // Forced transit: path must reach the goal and not explode.
    check(L < Ldirect * 2.0,   "TRANSIT: length < 2× direct");
    check(ms < 18000.0,        "TRANSIT: time < 18s (no timeout)");
  }
  if (s.expected == "APPROACH") {
    check(L < Ldirect * 1.5,   "APPROACH: length < 1.5× direct");
    check(ms < 18000.0,        "APPROACH: time < 18s (no timeout)");
  }
}

}  // namespace

int main(int argc, char **argv) {
  std::string yaml_path = (argc > 1)
      ? argv[1]
      : "src/terrain/mmp_terrain/data/risk_scenarios/synthetic_flat.yaml";

  LoadResult loaded;
  try {
    loaded = load(yaml_path);
  } catch (const std::exception &e) {
    std::cerr << "Failed to load yaml: " << e.what() << "\n";
    return 2;
  }
  std::cout << "Loaded " << loaded.scenarios.size()
            << " scenarios from " << yaml_path
            << "  (map " << loaded.map.size.transpose()
            << " voxel " << loaded.map.voxel << ")\n";

  std::vector<double> alphas = {0.1, 0.3, 1.0, 3.0, 10.0};
  if (argc > 2) alphas = {std::stod(argv[2])};

  double h_weight = (argc > 3) ? std::stod(argv[3]) : 1.0;
  if (argc > 4) g_front_end = argv[4];          // astar | fm2
  if (argc > 5) g_fm2_k = std::stoi(argv[5]);
  if (argc > 6) g_fm2_star = (std::string(argv[6]) != "0");
  if (argc > 7) g_barrier = std::stod(argv[7]);   // finite hard-wall K (0=off)
  std::cout << "front_end=" << g_front_end
            << " fm2_k=" << g_fm2_k
            << " fm2_star=" << (g_fm2_star ? 1 : 0) << "\n";

  for (double a : alphas) {
    for (const auto &s : loaded.scenarios) run(s, a, h_weight, loaded.map);
  }

  std::cout << "\n==== risk_scenarios_test: passed=" << g_passed
            << " failed=" << g_failed << " ====\n";
  return g_failed == 0 ? 0 : 1;
}
