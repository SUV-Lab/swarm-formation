#pragma once

#include <risk_model/model.hpp>
#include "path_manager/terrain_data.h"
#include "path_manager/terrain_risk.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>

namespace path_manager {

struct ProbabilityVisibilityPartitionSettings {
  double time_tolerance_s=1e-10;
  unsigned maximum_depth=40;
  std::size_t maximum_intervals=32768;
  std::size_t maximum_table_queries=2000000;
};

// Enclose the actual bilinear polar horizon over polynomial position bounds.
// Sampling agreement alone never certifies that a narrow visibility window
// is absent. Unresolved transition brackets are explicitly returned as such.
inline risk_model::BoundaryPartition partitionProbabilityVisibility(
    const TerrainData& terrain, const terrain_risk::Mask* mask, bool enabled,
    bool source_covered, risk_model::FrameScale scale,
    const risk_model::PolynomialPiece& piece,
    const ProbabilityVisibilityPartitionSettings& settings = {}) {
  risk_model::BoundaryPartition result;
  result.resolved = true;
  const auto invalid=[&](const char* reason) {
    result.resolved=false;result.reason=reason;return result;
  };
  if (!std::isfinite(scale.xy_m) || scale.xy_m<=0 ||
      !std::isfinite(scale.z_m) || scale.z_m<=0 ||
      !std::isfinite(piece.duration_s) || piece.duration_s<=0 ||
      !std::isfinite(settings.time_tolerance_s) || settings.time_tolerance_s<=0)
    return invalid("invalid visibility partition scale, duration or tolerance");
  for (const auto& axis:piece.coefficients_m) {
    if (axis.empty() || axis.size()>25 ||
        !std::all_of(axis.begin(),axis.end(),[](double x){return std::isfinite(x);}))
      return invalid("invalid visibility polynomial coefficients");
  }
  // These cases have spatially constant assumed visibility. Evaluation still
  // reports the assumption for sources with nonzero physical damage.
  if (!enabled || !terrain.valid || !source_covered || !mask || !mask->valid)
    return result;
  if (!std::isfinite(mask->radial_step) || mask->radial_step<=0 ||
      !std::isfinite(mask->max_range) || mask->max_range<=0 ||
      !mask->source.allFinite() || !std::isfinite(terrain.resolution) ||
      terrain.resolution<=0 || terrain.rows<=0 || terrain.cols<=0 ||
      !std::isfinite(terrain.origin_x) || !std::isfinite(terrain.origin_y) ||
      !std::isfinite(terrain.length_x) || !std::isfinite(terrain.length_y) ||
      mask->radial_count < 2 || mask->angular_count < 2 ||
      mask->shadow_ceiling.size() != static_cast<size_t>(mask->radial_count)*mask->angular_count) {
    result.resolved = false;
    result.reason = "malformed polar visibility table";
    return result;
  }
  constexpr double pi = 3.14159265358979323846;
  constexpr double two_pi = 2*pi;
  const double tolerance = settings.time_tolerance_s*std::max(1.0,piece.duration_s);
  const double xmax = terrain.origin_x+terrain.length_x;
  const double xmin = xmax-terrain.rows*terrain.resolution;
  const double ymax = terrain.origin_y+terrain.length_y;
  const double ymin = ymax-terrain.cols*terrain.resolution;
  if (!std::isfinite(tolerance) || !std::isfinite(xmin) || !std::isfinite(xmax) ||
      !std::isfinite(ymin) || !std::isfinite(ymax))
    return invalid("visibility geometry or tolerance overflowed");
  // The DEM query forms the same rectangle through several additions and
  // subtractions. Keep points near its floating point edge uncertified.
  const double dem_pad=32*std::numeric_limits<double>::epsilon()*
      std::max({1.0,std::abs(terrain.origin_x),std::abs(terrain.origin_y),
                std::abs(terrain.length_x),std::abs(terrain.length_y),
                terrain.rows*terrain.resolution,terrain.cols*terrain.resolution});
  std::vector<double> cuts{0,piece.duration_s};
  auto rootsAt=[&](size_t axis,double level,double a,double b) {
    auto polynomial=piece.coefficients_m[axis];
    polynomial[0]-=level*(axis==2?scale.z_m:scale.xy_m);
    if (!std::isfinite(polynomial[0])) {
      result.resolved=false;result.reason="visibility polynomial boundary overflowed";
      return std::vector<double>{};
    }
    const auto roots=risk_model::polynomialRoots(polynomial,a,b,tolerance);
    if (!roots.resolved) {
      result.resolved=false;
      result.reason="visibility polynomial boundary root could not be certified";
    }
    return roots.roots_s;
  };
  for (const auto& boundary:std::array<std::pair<size_t,double>,4>{{{0,xmin},{0,xmax},{1,ymin},{1,ymax}}}) {
    const auto roots=rootsAt(boundary.first,boundary.second,0,piece.duration_s);
    cuts.insert(cuts.end(),roots.begin(),roots.end());
  }
  std::sort(cuts.begin(),cuts.end());
  cuts.erase(std::unique(cuts.begin(),cuts.end()),cuts.end());
  result.breakpoints_s=cuts;
  size_t visited=0, table_queries=0;
  auto unresolved=[&](double a,double b,const char* reason) {
    result.resolved=false;
    result.reason=reason;
    result.breakpoints_s.push_back(a);
    result.breakpoints_s.push_back(a+(b-a)*.5);
    result.breakpoints_s.push_back(b);
  };
  std::function<void(double,double,unsigned)> partition;
  partition=[&](double a,double b,unsigned depth) {
    if (++visited>settings.maximum_intervals ||
        table_queries>settings.maximum_table_queries) {
      unresolved(a,b,"visibility interval enclosure work limit reached");
      return;
    }
    std::array<risk_model::PolynomialBounds,3> box;
    for (size_t axis=0;axis<3;++axis) {
      box[axis]=risk_model::polynomialBounds(piece.coefficients_m[axis],a,b,tolerance);
      // Bound the original SI polynomial first. Dividing its coefficients
      // before bounding could round a different polynomial into existence.
      // XY queries use frame units; Z stays SI, as in Exact visibility's
      // comparison with the frame horizon multiplied by scale.z_m.
      if (axis<2) {
        box[axis].minimum=std::nextafter(box[axis].minimum/scale.xy_m,
                                        -std::numeric_limits<double>::infinity());
        box[axis].maximum=std::nextafter(box[axis].maximum/scale.xy_m,
                                         std::numeric_limits<double>::infinity());
      }
      if (!box[axis].resolved || !std::isfinite(box[axis].minimum) ||
          !std::isfinite(box[axis].maximum)) {
        unresolved(a,b,"visibility position bounds could not be certified");
        return;
      }
    }
    // Entirely outside the DEM: sample() consistently uses assumed V=1.
    if (box[0].maximum<xmin-dem_pad || box[0].minimum>xmax+dem_pad ||
        box[1].maximum<ymin-dem_pad || box[1].minimum>ymax+dem_pad) return;
    const bool inside=box[0].minimum>=xmin+dem_pad && box[0].maximum<=xmax-dem_pad &&
                      box[1].minimum>=ymin+dem_pad && box[1].maximum<=ymax-dem_pad;
    const double infinity=std::numeric_limits<double>::infinity();
    const double dx0=std::nextafter(box[0].minimum-mask->source.x(),-infinity);
    const double dx1=std::nextafter(box[0].maximum-mask->source.x(),infinity);
    const double dy0=std::nextafter(box[1].minimum-mask->source.y(),-infinity);
    const double dy1=std::nextafter(box[1].maximum-mask->source.y(),infinity);
    const auto minabs=[](double lo,double hi) {return lo<=0&&hi>=0?0:std::min(std::abs(lo),std::abs(hi));};
    const double radial_pad=8*std::numeric_limits<double>::epsilon();
    const double rmin=std::nextafter(std::hypot(minabs(dx0,dx1),minabs(dy0,dy1))*(1-radial_pad),0.0);
    const double rmax=std::nextafter(std::hypot(std::max(std::abs(dx0),std::abs(dx1)),
                                 std::max(std::abs(dy0),std::abs(dy1)))*(1+radial_pad),infinity);
    if (!std::isfinite(rmin) || !std::isfinite(rmax)) {
      unresolved(a,b,"visibility radial bounds overflowed");return;
    }
    if (rmin>mask->max_range || rmax<=1e-9) return;
    const bool covered=inside && rmax<=mask->max_range && rmin>1e-9;
    double amin=0,amax=two_pi;
    if (!(dx0<=0 && dx1>=0 && dy0<=0 && dy1>=0)) {
      const double centre=std::atan2(dy0*.5+dy1*.5,dx0*.5+dx1*.5);
      amin=std::numeric_limits<double>::infinity(); amax=-amin;
      for (double x:{dx0,dx1}) for(double y:{dy0,dy1}) {
        const double angle=centre+std::remainder(std::atan2(y,x)-centre,two_pi);
        amin=std::min(amin,angle);amax=std::max(amax,angle);
      }
    }
    // Clamp in floating point before integer conversion, matching the
    // actual query. A large enclosing radius must never overflow an int.
    const double radial_limit=static_cast<double>(mask->radial_count-1);
    const double rfmin=std::clamp(rmin/mask->radial_step,0.0,radial_limit);
    const double rfmax=std::clamp(rmax/mask->radial_step,0.0,radial_limit);
    const int rlo=static_cast<int>(std::floor(rfmin));
    const int rhi=static_cast<int>(std::floor(rfmax));
    const double angular_pad=32*std::numeric_limits<double>::epsilon()*
        std::max({1.0,std::abs(amin),std::abs(amax)});
    amin=std::nextafter(amin-angular_pad,-infinity);
    amax=std::nextafter(amax+angular_pad,infinity);
    const double afmin=amin*mask->angular_count/two_pi-.5;
    const double afmax=amax*mask->angular_count/two_pi-.5;
    std::int64_t alo=static_cast<std::int64_t>(std::floor(afmin));
    std::int64_t ahi=static_cast<std::int64_t>(std::floor(afmax));
    const bool all_angles=ahi-alo+1>mask->angular_count;
    if (all_angles) {alo=0;ahi=mask->angular_count-1;}
    double low=infinity,high=-infinity,vertex_magnitude=0,constant_value=0;
    bool constant_table=true,first_vertex=true;
    const size_t cells=static_cast<size_t>(ahi-alo+1)*static_cast<size_t>(rhi-rlo+1);
    if (cells>(settings.maximum_table_queries-table_queries)/4) {
      unresolved(a,b,"visibility interval enclosure work limit reached");return;
    }
    table_queries+=4*cells;
    for(std::int64_t ai=alo;ai<=ahi;++ai) {
      const int angular=(ai%mask->angular_count+mask->angular_count)%mask->angular_count;
      const int next_angular=(angular+1)%mask->angular_count;
      const double at0=all_angles?0:std::clamp(afmin-ai,0.0,1.0);
      const double at1=all_angles?1:std::clamp(afmax-ai,0.0,1.0);
      for(int ri=rlo;ri<=rhi;++ri) {
        const int next_radius=std::min(ri+1,mask->radial_count-1);
        const auto vertex=[&](int angle,int radius) {
          const double value=mask->shadow_ceiling[static_cast<size_t>(angle)*mask->radial_count+radius];
          if (first_vertex) {constant_value=value;first_vertex=false;}
          constant_table=constant_table && value==constant_value;
          vertex_magnitude=std::max(vertex_magnitude,std::abs(value));
          return value;
        };
        const double v00=vertex(angular,ri),v01=vertex(angular,next_radius);
        const double v10=vertex(next_angular,ri),v11=vertex(next_angular,next_radius);
        if (!std::isfinite(v00) || !std::isfinite(v01) ||
            !std::isfinite(v10) || !std::isfinite(v11)) {
          unresolved(a,b,"non-finite visibility table bound");return;
        }
        const double rt0=std::clamp(rfmin-ri,0.0,1.0);
        const double rt1=std::clamp(rfmax-ri,0.0,1.0);
        // A bilinear patch reaches its extrema at the four corners of a
        // coordinate rectangle. Restrict that rectangle to the interval's
        // fractional cell coordinates: whole-cell vertex bounds alone do
        // not tighten as time shrinks inside one polar cell.
        for (double at:{at0,at1}) for (double rt:{rt0,rt1}) {
          const double z0=(1-rt)*v00+rt*v01;
          const double z1=(1-rt)*v10+rt*v11;
          const double value=(1-at)*z0+at*z1;
          low=std::min(low,value);high=std::max(high,value);
        }
      }
    }
    const double arithmetic_pad=32*std::numeric_limits<double>::epsilon()*vertex_magnitude;
    const double low_si=std::nextafter((low-arithmetic_pad)*scale.z_m,-infinity);
    const double high_si=std::nextafter((high+arithmetic_pad)*scale.z_m,infinity);
    if (!std::isfinite(low_si) || !std::isfinite(high_si)) {
      unresolved(a,b,"visibility SI horizon bounds overflowed");return;
    }
    if (box[2].minimum>=high_si) return; // also visible in any uncovered portion
    if (covered && box[2].maximum<low_si) return;
    if (covered && constant_table) {
      // The production sampler returns identical table vertices exactly.
      const auto roots=rootsAt(2,constant_value,a,b);
      result.breakpoints_s.insert(result.breakpoints_s.end(),roots.begin(),roots.end());
      return;
    }
    const double mid=a+(b-a)*.5;
    if (depth>=settings.maximum_depth || b-a<=tolerance || mid==a || mid==b) {
      unresolved(a,b,"visibility transition enclosed but not certified within time tolerance");
      return;
    }
    partition(a,mid,depth+1);
    partition(mid,b,depth+1);
  };
  for(size_t i=1;i<cuts.size();++i) partition(cuts[i-1],cuts[i],0);
  std::sort(result.breakpoints_s.begin(),result.breakpoints_s.end());
  result.breakpoints_s.erase(std::unique(result.breakpoints_s.begin(),result.breakpoints_s.end()),
                             result.breakpoints_s.end());
  return result;
}

} // namespace path_manager
