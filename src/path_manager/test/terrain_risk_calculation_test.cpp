// This public calculation header must remain usable without ROS headers.
#include "path_manager/terrain_risk.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <utility>

namespace tr = path_manager::terrain_risk;
namespace {
int checks = 0, failures = 0;
void check(bool ok, const std::string &label) {
  ++checks;
  if (!ok) { ++failures; std::cerr << "[FAIL] " << label << '\n'; }
}
path_manager::TerrainData terrain(bool ridge = false) {
  path_manager::TerrainData value;
  value.cols = 20; value.rows = 20; value.resolution = 1.0;
  value.origin_x = value.origin_y = -10.;
  value.length_x = value.length_y = 20.; value.valid = true;
  value.elevation.assign(400, 0.0f);
  if (ridge) for (int col = 0; col < 20; ++col)
    value.elevation[col * 20 + 6] = 4.0f;
  return value;
}
}

int main() {
  const tr::Settings settings{true, true, 0., .1, .15};
  const path_manager::RiskZone zone{Eigen::Vector3d(.5, .5, 2.), 10., .8};
  auto flat = terrain();
  auto ridge = terrain(true);
  const auto built = tr::buildMask(zone, ridge, settings);
  const auto &mask = built.mask;
  check(mask.valid && mask.shadow_ceiling.size() ==
            static_cast<std::size_t>(mask.angular_count) * mask.radial_count,
        "built result owns a complete polar table");
  check(mask.source == zone.center && mask.max_range == 1.05 * zone.reach,
        "mask covers optimizer rim without changing authored source");
  check(mask.radial_count >= 16 && mask.angular_count >= 72,
        "sampling retains minimum angular and radial coverage");
  check(!built.source_at_or_below_ground && built.source_ground == 0.,
        "source diagnostic uses original terrain height");
  auto below_zone = zone; below_zone.center = Eigen::Vector3d(3.5,.5,.5);
  const auto below = tr::buildMask(below_zone,ridge,settings);
  check(below.source_at_or_below_ground && below.source_ground == 4.f,
        "below-ground diagnostic is returned for the ROS logger");
  const Eigen::Vector3d behind(6.5,.5,2.);
  check(tr::shadowCeiling(&mask,true,behind) > 4.5,
        "near ridge casts a far shadow");
  check(tr::visibilityValue(&mask,ridge,settings,behind) < .01,
        "low far-side flight is occluded");
  check(tr::visibilityValue(&mask,ridge,settings,Eigen::Vector3d(6.5,.5,8.)) > .99,
        "climbing above the horizon restores visibility");
  check(tr::visibilityValue(&mask,ridge,settings,Eigen::Vector3d(-6.5,.5,2.)) > .99,
        "unblocked azimuth remains visible");
  check(tr::shadowCeiling(&mask,true,zone.center) ==
            -std::numeric_limits<double>::infinity(), "source has no shadow");
  check(tr::shadowCeiling(&mask,true,zone.center+Eigen::Vector3d(10.500001,0.,0.)) ==
            -std::numeric_limits<double>::infinity(), "outside mask retains ideal fallback");
  const double wrap_lo = tr::shadowCeiling(&mask,true,Eigen::Vector3d(6.5,.5-1e-10,3.));
  const double wrap_hi = tr::shadowCeiling(&mask,true,Eigen::Vector3d(6.5,.5+1e-10,3.));
  check(std::abs(wrap_lo-wrap_hi) < 1e-8, "azimuth interpolation is continuous across wrap");

  tr::Settings disabled = settings; disabled.enabled = false;
  auto unavailable = flat; unavailable.valid = false;
  auto empty_zone = zone; empty_zone.reach = 0.;
  check(!tr::buildMask(zone,flat,disabled).mask.valid, "disabled build is an ideal field");
  check(!tr::buildMask(zone,unavailable,settings).mask.valid, "missing DEM is an ideal field");
  check(!tr::buildMask(empty_zone,flat,settings).mask.valid, "zero reach skips horizon allocation");
  empty_zone.reach = -1.;
  check(!tr::buildMask(empty_zone,flat,settings).mask.valid, "negative reach retains skip behavior");
  empty_zone.reach = .01;
  check(tr::buildMask(empty_zone,flat,settings).mask.radial_count == 16,
        "subcell zone still has sixteen horizon rings");
  Eigen::Vector3d gradient = Eigen::Vector3d::Ones();
  check(tr::visibility(nullptr,ridge,settings,behind,&gradient) == 1. && gradient.isZero(),
        "null mask has full visibility and zero gradient");
  check(tr::visibility(&mask,ridge,disabled,behind,&gradient) == 1. && gradient.isZero(),
        "disabled query bypasses a previously built mask");

  tr::Settings ungrounded = settings; ungrounded.grounded = false;
  double derivative = -1.;
  check(tr::groundedZ(flat,ungrounded,Eigen::Vector3d(0.,0.,-2.),&derivative) == -2. && derivative == 1.,
        "ungrounded mode preserves absolute altitude");
  check(tr::groundedZ(unavailable,settings,Eigen::Vector3d(0.,0.,-2.),&derivative) == -2. && derivative == 1.,
        "missing DEM does not invent a grounded floor");
  check(tr::groundedZ(flat,settings,Eigen::Vector3d(20.,0.,-2.),&derivative) == -2. && derivative == 1.,
        "off-map query retains original altitude");
  check(tr::groundedZ(flat,settings,Eigen::Vector3d(0.,0.,-10.),&derivative) == .15 && derivative == 0.,
        "deep below clearance band saturates at physical floor");
  check(tr::groundedZ(flat,settings,Eigen::Vector3d(0.,0.,10.),&derivative) == 10. && derivative == 1.,
        "high altitude is an identity query");
  const Eigen::Vector3d floor_point(0.,0.,.15);
  const double grounded = tr::groundedZ(flat,settings,floor_point,&derivative);
  check(std::abs(grounded-(.15+.1*std::log(2.)))<1e-15 && derivative == .5,
        "clearance join retains smooth value and derivative");
  const double ceiling = tr::shadowCeiling(&mask,true,behind);
  const Eigen::Vector3d seam(behind.x(),behind.y(),ceiling);
  const double v = tr::visibility(&mask,ridge,ungrounded,seam,&gradient);
  check(v == .5 && gradient.z() == 2.5,
        "ungrounded horizon midpoint has original sigmoid slope");
  constexpr double eps = 1e-6;
  auto up=seam, down=seam; up.z()+=eps; down.z()-=eps;
  const double finite_difference = (tr::visibilityValue(&mask,ridge,ungrounded,up)-
      tr::visibilityValue(&mask,ridge,ungrounded,down))/(2.*eps);
  check(std::abs(gradient.z()-finite_difference)<1e-8,
        "analytic vertical gradient agrees with scalar field");

  auto water = terrain();
  std::fill(water.elevation.begin(),water.elevation.end(),std::numeric_limits<float>::quiet_NaN());
  ++water.generation;
  check(water.getElevation(0.,0.) == -std::numeric_limits<float>::infinity(),
        "pure water keeps no-terrain scalar sampler contract");
  float h=1.,gx=1.,gy=1.;
  check(water.getElevationAndGrad(0.,0.,&h,&gx,&gy) && h==0. && gx==0. && gy==0.,
        "pure water keeps sea-level safety floor for gradient sampler");
  check(tr::groundedZ(water,settings,Eigen::Vector3d(0.,0.,-10.)) == .15,
        "grounded visibility uses the same water safety floor");
  const auto owned = [&] {
    auto temporary = terrain(true); auto temporary_zone=zone;
    return tr::buildMask(temporary_zone,temporary,settings).mask;
  }();
  check(owned.shadow_ceiling == mask.shadow_ceiling &&
            tr::shadowCeiling(&owned,true,behind) == ceiling,
        "returned horizon survives destroyed build inputs");
  auto reusable = tr::buildMask(zone,ridge,settings).mask;
  const float *buffer = reusable.shadow_ceiling.data();
  const auto rebuilt = tr::buildMask(zone,ridge,settings,std::move(reusable));
  check(rebuilt.mask.shadow_ceiling.data() == buffer &&
            rebuilt.mask.shadow_ceiling == mask.shadow_ceiling,
        "rebuild consumes owned storage without allocating another sample buffer");
  auto replacement = tr::buildMask(zone,ridge,settings).mask;
  const auto cleared = tr::buildMask(zone,unavailable,settings,std::move(replacement));
  check(!cleared.mask.valid && cleared.mask.shadow_ceiling.empty() &&
            cleared.mask.radial_count == 0 && cleared.mask.source.isZero(),
        "reused output clears previous metadata when no DEM is available");
  const auto moved = std::move(tr::buildMask(zone,ridge,settings).mask);
  check(moved.shadow_ceiling == mask.shadow_ceiling,
        "moving result preserves owned horizon values");
  std::cout << (checks-failures) << '/' << checks << " checks passed\n";
  return failures ? 1 : 0;
}
