#include <path_manager/probability_visibility_partition.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>

namespace {
int checks=0,failures=0;
void check(bool ok,const std::string& label) {
  ++checks;
  if (!ok) {++failures;std::cerr<<"[FAIL] "<<label<<'\n';}
}
bool hasCut(const risk_model::BoundaryPartition& result,double time,double tolerance=1e-8) {
  return std::any_of(result.breakpoints_s.begin(),result.breakpoints_s.end(),
      [&](double cut){return std::abs(cut-time)<=tolerance;});
}
path_manager::TerrainData terrain() {
  path_manager::TerrainData result;
  result.valid=true;result.rows=100;result.cols=100;result.resolution=1;
  result.origin_x=-50;result.origin_y=-50;result.length_x=100;result.length_y=100;
  result.elevation.assign(10000,0);
  return result;
}
path_manager::terrain_risk::Mask mask(int angles=64,int radii=21) {
  path_manager::terrain_risk::Mask result;
  result.valid=true;result.max_range=radii-1;result.radial_step=1;
  result.radial_count=radii;result.angular_count=angles;
  result.shadow_ceiling.assign(static_cast<std::size_t>(radii)*angles,2);
  return result;
}
risk_model::PolynomialPiece line(double x,double dx,double y,double dy,double z,double dz,
                                risk_model::FrameScale scale={1,1}) {
  return {1,{{{x*scale.xy_m,dx*scale.xy_m},
              {y*scale.xy_m,dy*scale.xy_m},
              {z*scale.z_m,dz*scale.z_m}}}};
}
bool actualVisible(const path_manager::TerrainData& dem,
                   const path_manager::terrain_risk::Mask& horizon,
                   const risk_model::PolynomialPiece& piece,double time,
                   risk_model::FrameScale scale={1,1}) {
  Eigen::Vector3d position;
  for (std::size_t axis=0;axis<3;++axis) {
    double value=0;
    for (auto it=piece.coefficients_m[axis].rbegin();it!=piece.coefficients_m[axis].rend();++it)
      value=value*time+*it;
    position[axis]=value/(axis==2?scale.z_m:scale.xy_m);
  }
  float height,gx,gy;
  if (!dem.getElevationAndGrad(position.x(),position.y(),&height,&gx,&gy)) return true;
  return position.z()>=path_manager::terrain_risk::shadowCeiling(&horizon,true,position);
}
void narrowWindow(bool around_negative_x) {
  auto dem=terrain();auto horizon=mask(8192);
  const int first=around_negative_x?horizon.angular_count/2-1:horizon.angular_count-1;
  for (int ai:{first,(first+1)%horizon.angular_count})
    for (int ri=0;ri<horizon.radial_count;++ri)
      horizon.shadow_ceiling[static_cast<std::size_t>(ai)*horizon.radial_count+ri]=0;
  const auto piece=line(around_negative_x?-10:10,0,-2,4,1,0);
  const auto result=path_manager::partitionProbabilityVisibility(dem,&horizon,true,true,{1,1},piece);
  const double half_width=2.5*std::tan(2*3.14159265358979323846/horizon.angular_count);
  const std::string label=around_negative_x?"atan2 branch cut":"angular table wrap";
  check(!result.resolved,label+" reports residual nonpolynomial root brackets as unresolved");
  check(!result.reason.empty(),label+" exposes degradation reason");
  check(hasCut(result,.5-half_width),label+" brackets first narrow-window crossing");
  check(hasCut(result,.5+half_width),label+" brackets second narrow-window crossing");
  check(!actualVisible(dem,horizon,piece,.5-2*half_width) &&
        actualVisible(dem,horizon,piece,.5) &&
        !actualVisible(dem,horizon,piece,.5+2*half_width),
        label+" fixture has actual hidden-visible-hidden transition");
  std::cout<<label<<": width_s="<<2*half_width
           <<" cuts="<<result.breakpoints_s.size()<<" resolved="<<result.resolved<<'\n';
}
} // namespace

int main() {
  auto dem=terrain();auto horizon=mask();
  const risk_model::FrameScale scale{2,4};
  auto piece=line(10,1,2,0,0,4,scale);
  auto result=path_manager::partitionProbabilityVisibility(dem,&horizon,true,true,scale,piece);
  check(result.resolved,"constant polar horizon has polynomial crossing certificate");
  check(hasCut(result,.5),"constant plane crossing root survives anisotropic SI conversion");
  check(!actualVisible(dem,horizon,piece,.4,scale) && actualVisible(dem,horizon,piece,.6,scale),
        "constant plane fixture crosses actual horizon");
  piece=line(10,1,2,0,0,0);
  piece.coefficients_m[2]={1.92,.66,-1.5,1};
  result=path_manager::partitionProbabilityVisibility(dem,&horizon,true,true,{1,1},piece);
  check(result.resolved && hasCut(result,.2) && hasCut(result,.5) && hasCut(result,.8),
        "constant horizon includes every cubic height crossing");
  for (double z:{1.,3.}) {
    piece=line(10,1,2,1,z,0);
    result=path_manager::partitionProbabilityVisibility(dem,&horizon,true,true,{1,1},piece);
    check(result.resolved,z<2?"entirely below table is certified":"entirely above table is certified");
    check(result.breakpoints_s.size()==2,"constant visibility needs only interval endpoints");
  }
  // A rounded float table value must also stay exactly constant in the
  // actual sampler, so its polynomial crossing uses that same threshold.
  horizon.shadow_ceiling.assign(horizon.shadow_ceiling.size(),1.1f);
  for (double time:{.031,.317,.743,.999})
    check(path_manager::terrain_risk::shadowCeiling(&horizon,true,{10+time,2+time,0})==
          static_cast<double>(1.1f),"constant float table preserved exactly");
  piece=line(10,1,2,1,0,2);
  result=path_manager::partitionProbabilityVisibility(dem,&horizon,true,true,{1,1},piece);
  check(result.resolved && hasCut(result,static_cast<double>(1.1f)/2),
        "non-dyadic constant float horizon crossing is certified");
  horizon=mask();
  narrowWindow(false);narrowWindow(true);

  auto radial=horizon;
  for (int ai=0;ai<radial.angular_count;++ai)
    for (int ri=0;ri<radial.radial_count;++ri)
      radial.shadow_ceiling[static_cast<std::size_t>(ai)*radial.radial_count+ri]=.2f*ri;
  piece=line(10,0,0,0,2.1,0);
  piece.coefficients_m[0]={10,0,3};
  const double radial_root=std::sqrt((2.1-radial.shadow_ceiling[10])/
      (radial.shadow_ceiling[11]-radial.shadow_ceiling[10])/3);
  result=path_manager::partitionProbabilityVisibility(dem,&radial,true,true,{1,1},piece);
  check(!result.resolved && hasCut(result,radial_root),
        "fractional radial enclosure brackets curved-path crossing");
  check(actualVisible(dem,radial,piece,radial_root-.01) &&
        !actualVisible(dem,radial,piece,radial_root+.01),
        "curved radial fixture crosses the actual interpolated table");

  path_manager::ProbabilityVisibilityPartitionSettings budget;
  budget.maximum_table_queries=1;
  result=path_manager::partitionProbabilityVisibility(dem,&horizon,true,true,{1,1},piece,budget);
  check(!result.resolved && result.reason.find("work limit")!=std::string::npos,
        "table-query budget exhaustion degrades explicitly");
  check(hasCut(result,0) && hasCut(result,.5) && hasCut(result,1),
        "budget exhaustion returns enclosing interval and midpoint");
  budget={};budget.maximum_intervals=0;
  result=path_manager::partitionProbabilityVisibility(dem,&horizon,true,true,{1,1},piece,budget);
  check(!result.resolved,"interval budget exhaustion degrades explicitly");

  auto wide=mask(64,101);
  piece=line(-60,20,1,0,1,0);
  result=path_manager::partitionProbabilityVisibility(dem,&wide,true,true,{1,1},piece);
  check(hasCut(result,.5),"DEM entry polynomial is included in partition");
  check(!result.resolved,"floating DEM boundary bracket is not falsely certified");
  check(actualVisible(dem,wide,piece,.25) && !actualVisible(dem,wide,piece,.75),
        "DEM entry fixture changes assumed-visible to known-hidden");
  piece=line(19,2,0,0,1,0);
  result=path_manager::partitionProbabilityVisibility(dem,&horizon,true,true,{1,1},piece);
  check(!result.resolved && hasCut(result,.5),"finite horizon range boundary is enclosed and degraded");
  check(!actualVisible(dem,horizon,piece,.25) && actualVisible(dem,horizon,piece,.75),
        "range boundary fixture changes known-hidden to assumed-visible");

  piece=line(10,1,2,1,1,0);
  auto malformed=horizon;malformed.radial_step=0;
  check(!path_manager::partitionProbabilityVisibility(dem,&malformed,true,true,{1,1},piece).resolved,
        "invalid radial step rejected before floating-to-integer conversion");
  check(!path_manager::partitionProbabilityVisibility(dem,&horizon,true,true,{0,1},piece).resolved,
        "invalid coordinate scale rejected");
  auto empty=piece;empty.coefficients_m[0].clear();
  check(!path_manager::partitionProbabilityVisibility(dem,&horizon,true,true,{1,1},empty).resolved,
        "empty polynomial rejected before coefficient access");
  malformed=horizon;malformed.shadow_ceiling.assign(malformed.shadow_ceiling.size(),
                                                   std::numeric_limits<float>::quiet_NaN());
  check(!path_manager::partitionProbabilityVisibility(dem,&malformed,true,true,{1,1},piece).resolved,
        "non-finite horizon table degrades explicitly");
  check(path_manager::partitionProbabilityVisibility(dem,&horizon,false,true,{1,1},piece).resolved,
        "disabled visibility has constant assumed-visible partition");
  check(path_manager::partitionProbabilityVisibility(dem,&horizon,true,false,{1,1},piece).resolved,
        "uncovered source has constant assumed-visible partition");
  auto missing=dem;missing.valid=false;
  check(path_manager::partitionProbabilityVisibility(missing,&horizon,true,true,{1,1},piece).resolved,
        "missing DEM has constant assumed-visible partition");

  // Exercise a finite radius whose ratio to radial_step overflows. Production
  // sampling clamps this ratio before converting it to an integer as well.
  auto huge=dem;huge.origin_x=-5e200;huge.origin_y=-5e200;
  huge.length_x=1e201;huge.length_y=1e201;huge.resolution=1e199;
  malformed=horizon;malformed.radial_step=1e-300;malformed.max_range=1e201;
  piece=line(1e200,0,1e200,0,3,0);
  check(path_manager::partitionProbabilityVisibility(huge,&malformed,true,true,{1,1},piece).resolved,
        "huge finite radius ratio is clamped without integer overflow");
  std::cout<<checks-failures<<'/'<<checks<<" visibility partition checks passed\n";
  return failures?1:0;
}
