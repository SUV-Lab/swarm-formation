// v0 종방향 공력 계수 공급자 구현. **비생산.**
// 회수되지 않은 자리는 전부 던진다 — 지어내는 것보다 멈추는 편이 낫다.
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

const char *pitchRateNormName(PitchRateNorm n)
{
  switch (n) {
    case PitchRateNorm::HalfChordOverV: return "q c/(2V)";
    case PitchRateNorm::ChordOverV: return "q c/V";
    case PitchRateNorm::Unspecified: default: return "규약 미지정";
  }
}

void requireInBand(double mach, const char *what)
{
  if (!std::isfinite(mach)) throw OutOfRange(std::string(what) + ": 마하 비유한");
  if (mach < kMachMin || mach > kMachMax) {
    char b[192];
    std::snprintf(b, sizeof(b),
                  "%s: 마하 %.4f 가 적용 범위 [%.4f, %.4f] 밖이다. "
                  "저속 시험값을 대역 밖까지 늘려 쓰지 않는다.",
                  what, mach, kMachMin, kMachMax);
    throw OutOfRange(b);
  }
}

void requireUsable(double value, Grade grade, const std::string &basis,
                   const char *what)
{
  if (!std::isfinite(value)) {
    throw Unavailable(std::string(what) + ": 값이 비유한");
  }
  if (grade == Grade::Unknown) {
    throw Unavailable(std::string(what) + ": 등급이 UNKNOWN");
  }
  if (basis.empty()) {
    throw Unavailable(std::string(what) + ": 근거 문자열이 비어 있다 — "
                      "어느 문서의 무엇인지 적지 않은 수는 쓰지 않는다");
  }
}

namespace {
void requireNorm(PitchRateNorm n, const char *what)
{
  if (n != PitchRateNorm::HalfChordOverV) {
    throw Unavailable(std::string(what) + ": 무차원화 규약이 " +
                      pitchRateNormName(n) +
                      " 다. 이 모형은 q c/(2V) 만 받는다 — 규약이 다르면 "
                      "2배(그리고 기준길이비 제곱)만큼 어긋난다.");
  }
}
}  // namespace

PitchDampingAlone PitchDampingAlone::make(double cm_q, PitchRateNorm norm,
                                          double mach, Grade grade,
                                          const std::string &basis)
{
  requireUsable(cm_q, grade, basis, "Cm_q 단독");
  requireNorm(norm, "Cm_q 단독");
  PitchDampingAlone o;
  o.cm_q_ = cm_q; o.norm_ = norm; o.mach_ = mach;
  o.grade_ = grade; o.basis_ = basis;
  return o;
}

void PitchDampingAlone::markDiagnosticOnly(const std::string &why)
{
  diagnostic_only_ = true;
  diagnostic_reason_ = why;
}

PitchDampingSum PitchDampingSum::make(double sum, PitchRateNorm norm,
                                      double mach, Grade grade,
                                      const std::string &basis)
{
  requireUsable(sum, grade, basis, "Cm_q + Cm_alphadot 합");
  requireNorm(norm, "Cm_q + Cm_alphadot 합");
  PitchDampingSum o;
  o.sum_ = sum; o.norm_ = norm; o.mach_ = mach;
  o.grade_ = grade; o.basis_ = basis;
  return o;
}

PitchDampingAlone subtractAlphaDot(const PitchDampingSum &sum,
                                   double cm_alphadot, PitchRateNorm norm,
                                   Grade grade, const std::string &basis)
{
  requireUsable(cm_alphadot, grade, basis, "Cm_alphadot");
  requireNorm(norm, "Cm_alphadot");
  if (norm != sum.norm()) {
    throw Unavailable("합과 Cm_alphadot 의 무차원화 규약이 다르다");
  }
  return PitchDampingAlone::make(
      sum.value() - cm_alphadot, norm, sum.mach(), Grade::Estimated,
      "합(" + sum.basis() + ") - Cm_alphadot(" + basis + ")");
}

double tailSensitivityEq3(double cl_alpha_htail_per_rad,
                          double downwash_factor, const Geometry &g)
{
  if (g.wing_area_m2 <= 0.0 || g.wing_mac_m <= 0.0) {
    throw Unavailable("식 (3) 민감도: 형상이 비어 있다");
  }
  const double area_ratio = g.htail_area_m2 / g.wing_area_m2;
  const double arm_ratio = g.tail_arm_m / g.wing_mac_m;
  return -2.0 * cl_alpha_htail_per_rad * downwash_factor
       * area_ratio * arm_ratio * arm_ratio;
}

ClosureDiagnostic closureDiagnostic(const Tr1096Anchor &anchor,
                                    const Geometry &g,
                                    const DownwashLag &downwash)
{
  if (!anchor.available) throw Unavailable("폐쇄 진단: 기준점 미회수");
  requireUsable(anchor.cm_q_whole_config, anchor.grade, anchor.basis,
                "기준점 Cm_q (전체 형상 W+F2+V+H2)");
  requireUsable(anchor.cm_q_wing_fuselage, anchor.grade, anchor.basis,
                "기준점 Cm_q (날개+동체 W+F2, 수직꼬리 없음)");
  requireUsable(anchor.cl_alpha_htail_per_rad, anchor.grade, anchor.basis,
                "기준점 CL_alpha,H");
  requireNorm(anchor.norm, "기준점");
  if (!downwash.available) throw Unavailable("폐쇄 진단: 다운워시 항 미회수");
  requireUsable(downwash.factor, downwash.grade, downwash.basis, "다운워시 항");

  ClosureDiagnostic d;
  d.measured_increment = anchor.configurationIncrementVPlusH();
  d.eq3_prediction = tailSensitivityEq3(anchor.cl_alpha_htail_per_rad,
                                        downwash.factor, g);
  d.residual = d.eq3_prediction - d.measured_increment;
  // 두 실측의 차이라 판독 오차가 독립적으로 들어간다. **판독 오차만**
  // 이다 — 형상 오차, 레이놀즈수 차이, 수직꼬리 오염은 여기 없다.
  d.read_error_tolerance = std::sqrt(
      anchor.read_error_whole * anchor.read_error_whole +
      anchor.read_error_wing_fuselage * anchor.read_error_wing_fuselage);
  d.within_read_error = std::isfinite(d.residual)
                     && std::fabs(d.residual) <= d.read_error_tolerance;
  return d;
}

double halfChordSweepRad(double sweep_c4_rad, double aspect_ratio,
                         double taper_ratio)
{
  if (aspect_ratio <= 0.0 || taper_ratio < 0.0) {
    throw Unavailable("반시위 후퇴각: 형상이 유효하지 않다");
  }
  const double t = std::tan(sweep_c4_rad)
                 - (4.0 / aspect_ratio)
                   * ((0.5 - 0.25) * (1.0 - taper_ratio)
                      / (1.0 + taper_ratio));
  return std::atan(t);
}

double liftSlopeTn3911PerRad(double mach, double aspect_ratio,
                             double half_chord_sweep_rad)
{
  const double A = aspect_ratio;
  const double c = std::cos(half_chord_sweep_rad);
  if (A <= 0.0 || c <= 0.0) {
    throw Unavailable("TN 3911 (A1): 형상이 유효하지 않다");
  }
  const double radicand = 4.0 + (A / c) * (A / c) - (A * mach) * (A * mach);
  if (radicand <= 0.0) {
    // (A M)^2 가 나머지를 넘어서면 식이 정의되지 않는다 — 아공속
    // 가정이 깨지는 지점이다. 조용히 넘기지 않는다.
    throw OutOfRange("TN 3911 (A1): 근호 안이 비양수 — 적용 밖");
  }
  return 2.0 * M_PI * A / (2.0 + std::sqrt(radicand));
}

Compressibility liftSlopeRatio(double mach, double mach_anchor,
                               double aspect_ratio, double sweep_c4_rad)
{
  requireInBand(mach, "압축성 보정");
  (void)mach_anchor; (void)aspect_ratio; (void)sweep_c4_rad;
  // 관계는 회수됐다(TN 3911 eq. A1, 위 liftSlopeTn3911PerRad). 다만
  // **출처 감사가 끝나지 않았다** — 권리·중립성 확인 전에는 연결하지
  // 않는다. TR 1188 에는 이 관계가 없다는 것이 확인됐으므로 여기서
  // TR 1188 을 근거로 댈 수도 없다.
  throw Unavailable(
      "압축성 보정: 관계는 TN 3911 eq.(A1) 로 회수됐으나 그 문서의 "
      "권리·중립성 감사가 끝나지 않았다. 단순 1/sqrt(1-M^2) 로 "
      "대체하지 않는다 (대역 상단에서 26.5% 과대).");
}

void pitchDampingInBand(double mach, const Geometry &g,
                        const Tr1096Anchor &anchor)
{
  (void)g;
  char b[640];
  std::snprintf(b, sizeof(b),
      "목표 대역의 Cm_q 는 UNKNOWN 이다 (요청 마하 %.4f).\n"
      "  · M = %.2f 실측은 **전체 형상**(W+F2+V+H2) 것이고 대역 "
      "[%.4f, %.4f] 밖이다\n"
      "  · 실측 차이는 수평꼬리 단독이 아니라 **V+H2 구성 증분**이라 "
      "CL_alpha,H 의 마하 보정을 통째로 걸 근거가 없다 — 섞인 수직꼬리 "
      "기여가 같은 비율로 변한다는 보장이 없다\n"
      "  · 식 (3) 은 우리 형상에서 실측과 17.5%% 어긋나 절대값 보정에 "
      "쓸 수 없다\n"
      "  다음 중 하나가 생기기 전까지 활성화하지 않는다: "
      "(a) 수직꼬리·수평꼬리 기여의 분리 근거, "
      "(b) 전체 형상의 대역 내 자료.",
      mach, anchor.mach, kMachMin, kMachMax);
  throw Unavailable(b);
}

}  // namespace v0aero
