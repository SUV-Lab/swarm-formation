// v0 종방향 공력 계수 공급자 구현. **비생산.**
//
// 아직 수치가 회수되지 않은 부분은 **던진다.** 값을 지어내는 것보다
// 실행이 멈추는 편이 낫다 — 이 실험의 규칙이 그것이다.
#include "v0_aero.hpp"

#include <cstdio>

namespace v0aero {

const char *gradeName(Grade g)
{
  switch (g) {
    case Grade::Declared: return "DECLARED";
    case Grade::DeclaredTest: return "DECLARED_TEST";
    case Grade::ScaledPublic: return "SCALED_PUBLIC";
    case Grade::MeasuredDigitized: return "MEASURED_DIGITIZED";
    case Grade::Estimated: return "ESTIMATED";
    case Grade::Unknown: default: return "UNKNOWN";
  }
}

void requireInBand(double mach, const char *what)
{
  if (!std::isfinite(mach)) {
    throw OutOfRange(std::string(what) + ": 마하가 비유한");
  }
  if (mach < kMachMin || mach > kMachMax) {
    char buf[192];
    std::snprintf(buf, sizeof(buf),
                  "%s: 마하 %.4f 가 적용 범위 [%.4f, %.4f] 밖이다. "
                  "저속 시험값을 대역 밖까지 늘려 쓰지 않는다.",
                  what, mach, kMachMin, kMachMax);
    throw OutOfRange(buf);
  }
}

// eq (3) 의 꼬리 기여, per-radian 형태.
//   원문: (dCm_q)_H = -114.6 (CL_alpha)_H[/deg] (1 - d eps_r/d(ql/V))
//                     (S_H/S_W) (l/c_W)^2
//   -114.6 = -2 x 57.3 이므로 CL_alpha 를 per-radian 으로 받으면 상수는 -2.
//
// (S_H/S)(l/c)^2 = V_H (l/c) 이지만, 원문 형태 그대로 두어 대조가 쉽게
// 한다. 부피계수는 매니페스트가 따로 선언하고 하니스가 검산한다.
double tailPitchDampingContribution(double cl_alpha_htail_per_rad,
                                    double downwash_factor,
                                    const Geometry &g)
{
  if (g.wing_area_m2 <= 0.0 || g.wing_mac_m <= 0.0) {
    throw Unavailable("꼬리 감쇠: 형상이 비어 있다");
  }
  const double area_ratio = g.htail_area_m2 / g.wing_area_m2;
  const double arm_ratio = g.tail_arm_m / g.wing_mac_m;
  return -2.0 * cl_alpha_htail_per_rad * downwash_factor
       * area_ratio * arm_ratio * arm_ratio;
}

Compressibility liftSlopeRatio(double mach, double mach_anchor,
                               double aspect_ratio, double sweep_c4_rad)
{
  requireInBand(mach, "압축성 보정");
  (void)mach_anchor;
  (void)aspect_ratio;
  (void)sweep_c4_rad;
  // TR 1188 의 유한 날개·후퇴각 관계가 아직 회수되지 않았다.
  //
  // 여기에 단순 Prandtl-Glauert 1/sqrt(1-M^2) 를 넣고 싶은 유혹이 있는데
  // **계약이 그것을 금지한다.** 종횡비와 후퇴각이 들어가지 않은 보정은
  // 이 형상(AR 4.0, 후퇴각 45°)에서 틀린 방향으로 크다. 근거가 올
  // 때까지 던진다.
  throw Unavailable(
      "압축성 보정: TR 1188 의 유한 날개·후퇴각 관계 미회수. "
      "단순 1/sqrt(1-M^2) 로 대체하지 않는다 (계약).");
}

PitchDampingAlone pitchDamping(double mach, const Geometry &g,
                               const Tr1096Anchor &anchor,
                               const DownwashLag &downwash)
{
  requireInBand(mach, "Cm_q");
  if (!anchor.available) {
    throw Unavailable("Cm_q: TR 1096 M=0.13 기준점 미회수");
  }
  if (!downwash.available) {
    throw Unavailable("Cm_q: 다운워시 항 미회수 — CL_alpha 와 분리해 "
                      "따로 세워야 한다 (계약)");
  }
  // 마하 보정은 CL_alpha,H 에 걸린다. 던지면 여기서 같이 멈춘다.
  const Compressibility c = liftSlopeRatio(
      mach, anchor.mach, g.htail_aspect_ratio, g.htail_sweep_c4_rad);
  if (!c.available) {
    throw Unavailable("Cm_q: 압축성 비 미회수");
  }

  // 기준점에서 꼬리 기여와 나머지(날개+동체)를 가른다. 마하가 오르면
  // 꼬리 기여만 CL_alpha,H 를 통해 자라고 나머지는 그대로 둔다 —
  // 나머지의 마하 의존을 뒷받침할 근거가 아직 없기 때문이다.
  const double tail_anchor = tailPitchDampingContribution(
      anchor.cl_alpha_htail_per_rad, downwash.factor, g);
  const double remainder = anchor.cm_q_total_per_rad - tail_anchor;
  const double tail_at_mach = tail_anchor * c.ratio_to_anchor;

  PitchDampingAlone out;
  out.cm_q_per_rad = remainder + tail_at_mach;
  out.mach = mach;
  // 실측 기준점에서 출발했어도 결과는 **추정**이다 — 마하 보정과
  // 분해가 모두 모형이다.
  out.grade = Grade::Estimated;
  return out;
}

PitchDampingAlone subtractAlphaDot(const PitchDampingSum &sum,
                                   double cm_alphadot_per_rad,
                                   Grade cm_alphadot_grade,
                                   const std::string &basis)
{
  if (cm_alphadot_grade == Grade::Unknown || basis.empty()) {
    throw Unavailable(
        "합에서 단독을 빼려면 Cm_alphadot 의 근거가 있어야 한다. "
        "근거 없이 빼는 것은 합을 단독으로 저장하는 것과 같다.");
  }
  PitchDampingAlone out;
  out.cm_q_per_rad = sum.cm_q_plus_cm_alphadot_per_rad - cm_alphadot_per_rad;
  out.mach = sum.mach;
  out.grade = Grade::Estimated;
  return out;
}

}  // namespace v0aero
