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

TailClosure checkTailClosure(const Tr1096Anchor &anchor, const Geometry &g,
                             const DownwashLag &downwash)
{
  if (!anchor.available) throw Unavailable("폐쇄 검사: 기준점 미회수");
  requireUsable(anchor.cm_q_total, anchor.grade, anchor.basis,
                "기준점 Cm_q (꼬리 on)");
  requireUsable(anchor.cm_q_tail_off, anchor.grade, anchor.basis,
                "기준점 Cm_q (꼬리 off)");
  requireUsable(anchor.cl_alpha_htail_per_rad, anchor.grade, anchor.basis,
                "기준점 CL_alpha,H");
  requireNorm(anchor.norm, "기준점");
  if (!downwash.available) throw Unavailable("폐쇄 검사: 다운워시 항 미회수");
  requireUsable(downwash.factor, downwash.grade, downwash.basis, "다운워시 항");

  TailClosure c;
  c.measured_delta = anchor.cm_q_total - anchor.cm_q_tail_off;
  c.predicted_delta = tailPitchDampingContribution(
      anchor.cl_alpha_htail_per_rad, downwash.factor, g);
  c.residual = c.predicted_delta - c.measured_delta;
  // 두 실측값의 차이라 판독 오차가 독립적으로 두 번 들어간다.
  c.tolerance = anchor.read_error * std::sqrt(2.0);
  c.closed = std::isfinite(c.residual)
          && std::fabs(c.residual) <= c.tolerance;
  return c;
}

Compressibility liftSlopeRatio(double mach, double mach_anchor,
                               double aspect_ratio, double sweep_c4_rad)
{
  requireInBand(mach, "압축성 보정");
  (void)mach_anchor; (void)aspect_ratio; (void)sweep_c4_rad;
  // TR 1188 의 유한 날개·후퇴각 관계가 아직 회수되지 않았다. 여기에
  // 단순 Prandtl-Glauert 를 넣는 것은 계약 위반이다 — 종횡비와 후퇴각이
  // 빠진 보정은 AR 4.0·후퇴각 45° 에서 틀린 방향으로 크다.
  throw Unavailable(
      "압축성 보정: TR 1188 의 유한 날개·후퇴각 관계 미회수. "
      "단순 1/sqrt(1-M^2) 로 대체하지 않는다 (계약).");
}

PitchDampingAlone pitchDamping(double mach, const Geometry &g,
                               const Tr1096Anchor &anchor,
                               const DownwashLag &downwash,
                               const NonTailMachModel &non_tail,
                               bool allow_diagnostic)
{
  requireInBand(mach, "Cm_q");

  // 1) 식 (3) 이 실측 on/off 차이와 닫히는지 **먼저** 본다.
  //    닫히지 않으면 분해 자체가 성립하지 않는다.
  const TailClosure c = checkTailClosure(anchor, g, downwash);
  if (!c.closed) {
    char b[288];
    std::snprintf(b, sizeof(b),
                  "식 (3) 이 실측 꼬리 on/off 차이와 닫히지 않는다: "
                  "실측 %.5f, 예측 %.5f, 잔차 %.5f, 허용 %.5f. "
                  "나머지를 (실측 total - 계산 tail) 로 정의하면 이 불일치가 "
                  "나머지에 숨는다 — 그래서 여기서 멈춘다.",
                  c.measured_delta, c.predicted_delta, c.residual,
                  c.tolerance);
    throw NotClosed(b);
  }

  // 2) 비꼬리 기여의 마하 의존. 마하 무관은 **추정**이지 근거가 아니다.
  if (!non_tail.available) {
    throw Unavailable(
        "Cm_q: 비꼬리(날개+동체) 기여의 마하 모델 미회수. 마하 무관으로 "
        "두는 것은 공개 근거가 아니라 추정이므로 기본값으로 삼지 않는다.");
  }
  requireUsable(non_tail.ratio_to_anchor, non_tail.grade, non_tail.basis,
                "비꼬리 마하 모델");
  if (non_tail.diagnostic_only && !allow_diagnostic) {
    throw Unavailable(
        "Cm_q: 비꼬리 마하 모델이 진단 전용이다. 진단으로 쓰려면 "
        "호출부가 allow_diagnostic 을 명시해야 한다.");
  }

  // 3) 압축성은 CL_alpha,H 에 걸린다.
  const Compressibility comp = liftSlopeRatio(
      mach, anchor.mach, g.htail_aspect_ratio, g.htail_sweep_c4_rad);
  requireUsable(comp.ratio_to_anchor, comp.grade, comp.basis, "압축성 비");

  // 4) 조립. 꼬리는 CL_alpha,H 를 통해, 비꼬리는 자기 모델을 통해 자란다.
  //    나머지는 **실측 tail_off** 다 — 계산값을 뺀 잔여가 아니다.
  const double tail_anchor = c.predicted_delta;
  const double non_tail_anchor = anchor.cm_q_tail_off;
  const double value = non_tail_anchor * non_tail.ratio_to_anchor
                     + tail_anchor * comp.ratio_to_anchor;

  PitchDampingAlone out = PitchDampingAlone::make(
      value, PitchRateNorm::HalfChordOverV, mach, Grade::Estimated,
      "TR 1096 M=0.13 기준점(" + anchor.basis + ") + 압축성(" + comp.basis
      + ") + 비꼬리 마하(" + non_tail.basis + ")");
  if (non_tail.diagnostic_only) {
    out.markDiagnosticOnly("비꼬리 마하 모델이 진단 전용");
  }
  return out;
}

}  // namespace v0aero
