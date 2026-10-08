// TerrainData is the public terrain value stored independently by each PM.
// Identical per-manager generations must not share thread-local samples.
#include "path_manager/path_manager.h"

#include <iostream>
#include <new>
#include <string>
#include <type_traits>
#include <utility>

namespace {
int failures = 0;

void check(bool condition, const std::string &message)
{
  std::cout << (condition ? "[OK] " : "[FAIL] ") << message << '\n';
  if (!condition) ++failures;
}

path_manager::TerrainData terrain(float height)
{
  path_manager::TerrainData result;
  result.rows = result.cols = 2;
  result.resolution = 1.0;
  result.length_x = result.length_y = 2.0;
  result.elevation.assign(4, height);
  result.valid = true;
  result.generation = 1;
  return result;
}

bool sample(const path_manager::TerrainData &data, float expected)
{
  float h = -100.0f, gx = -100.0f, gy = -100.0f;
  const bool ok = data.getElevationAndGrad(1.0, 1.0, &h, &gx, &gy);
  std::cout << "  generation=" << data.generation << " expected=" << expected
            << " actual=" << h << " valid=" << ok << '\n';
  return ok && h == expected && gx == 0.0f && gy == 0.0f;
}

bool agreesWithUncached(const path_manager::TerrainData &data)
{
  float eh = 0.0f, ex = 0.0f, ey = 0.0f, h = 0.0f, x = 0.0f, y = 0.0f;
  const bool expected = data.getElevationAndGradUncached(1.0, 1.0, &eh, &ex, &ey);
  const bool actual = data.getElevationAndGrad(1.0, 1.0, &h, &x, &y);
  return expected == actual && eh == h && ex == x && ey == y;
}
}  // namespace

int main()
{
  auto first = terrain(2.0f);
  auto second = terrain(8.0f);
  check(sample(first, 2.0f) && sample(second, 8.0f) && sample(first, 2.0f),
        "distinct manager terrain values at the same generation never share cached samples");
  sample(first, 2.0f);
  first = second;
  check(first.generation == 1 && sample(first, 8.0f),
        "copy assignment changes cache identity without changing the public generation");

  auto copied = second;
  copied.elevation.assign(4, 9.0f);
  ++copied.generation;
  ++second.generation;
  check(sample(second, 8.0f) && sample(copied, 9.0f) && sample(second, 8.0f),
        "a copied terrain remains independent after both values advance to the same generation");

  auto target = terrain(3.0f);
  auto source = terrain(7.0f);
  sample(target, 3.0f);
  target = std::move(source);
  check(target.generation == 1 && sample(target, 7.0f),
        "move assignment cannot reuse the destination's previous cached value");
  auto moved = std::move(target);
  target = terrain(11.0f);
  check(sample(moved, 7.0f) && sample(target, 11.0f) && sample(moved, 7.0f),
        "move construction and reuse of the source keep independent identities");

  auto move_source = terrain(13.0f);
  sample(move_source, 13.0f);
  auto move_destination = std::move(move_source);
  check(agreesWithUncached(move_source) &&
            sample(move_destination, 13.0f),
        "a moved-from terrain cannot return the cache of its transferred elevation buffer");
  auto assigned_source = terrain(14.0f);
  sample(assigned_source, 14.0f);
  move_destination = std::move(assigned_source);
  check(agreesWithUncached(assigned_source) && sample(move_destination, 14.0f),
        "move assignment also invalidates the source object's memo");
  sample(move_destination, 14.0f);
  move_destination = std::move(move_destination);
  check(agreesWithUncached(move_destination),
        "self-move assignment cannot leave a sample for storage the vector released");

  using Storage = std::aligned_storage_t<sizeof(path_manager::TerrainData),
                                         alignof(path_manager::TerrainData)>;
  Storage storage;
  auto *reused = new (&storage) path_manager::TerrainData(terrain(4.0f));
  sample(*reused, 4.0f);
  reused->~TerrainData();
  reused = new (&storage) path_manager::TerrainData(terrain(12.0f));
  check(sample(*reused, 12.0f),
        "a new terrain at the exact same address cannot inherit the destroyed object's memo");
  reused->~TerrainData();

  auto toggled = terrain(5.0f);
  sample(toggled, 5.0f);
  toggled.valid = false;
  float h = 99.0f, gx = 99.0f, gy = 99.0f;
  check(!toggled.getElevationAndGrad(1.0, 1.0, &h, &gx, &gy) &&
            h == 0.0f && gx == 0.0f && gy == 0.0f && toggled.generation == 1,
        "invalid terrain cannot receive a previously cached valid sample");
  toggled.valid = true;
  check(sample(toggled, 5.0f), "restoring validity recomputes the existing terrain");
  toggled.elevation.assign(4, 6.0f);
  ++toggled.generation;
  check(sample(toggled, 6.0f) && sample(toggled, 6.0f),
        "generation changes still invalidate same-buffer updates and repeated reads remain valid");
  std::cout << (failures ? "FAIL" : "PASS") << ": " << failures << " failed check(s)\n";
  return failures ? 1 : 0;
}
