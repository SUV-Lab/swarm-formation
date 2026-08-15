// v0_aero 계약 시험. **비생산.**
// 값이 아직 없으므로 시험하는 것은 **계약**이다: 대역 밖 거부, 미회수
// 입력에 대한 fail-closed, 그리고 합과 단독이 섞이지 않는다는 것.
#include "v0_aero.hpp"

#include <cstdio>
#include <string>

namespace {
int failures = 0;
void expect(bool ok, const std::string &what)
{
  std::printf("%s  %s\n", ok ? "[OK]  " : "[FAIL]", what.c_str());
  if (!ok) ++failures;
}
template <typename F>
bool throws(F f)
{
  try { f(); } catch (const std::exception &) { return true; }
  return false;
}

v0aero::Geometry geom()
{
  v0aero::Geometry g;
  g.wing_area_m2 = 1.0;
  g.wing_span_m = 2.000000;
  g.wing_mac_m = 0.510556;
  g.wing_aspect_ratio = 4.0;
  g.wing_sweep_c4_rad = 45.0 * 3.14159265358979323846 / 180.0;
  g.htail_area_m2 = 0.200000;
  g.htail_aspect_ratio = 4.0;
  g.htail_sweep_c4_rad = g.wing_sweep_c4_rad;
  g.tail_arm_m = 0.927837;
  g.tail_arm_over_mac = 1.817200;
  g.htail_volume_coeff = 0.363440;
  return g;
}
}  // namespace

int main()
{
  using namespace v0aero;
  const Geometry g = geom();

  // ── 적용 범위 ───────────────────────────────────────────
  expect(throws([&] { requireInBand(0.30, "t"); }),
         "대역 아래 마하는 거부된다 (저속값을 늘려 쓰지 않는다)");
  expect(throws([&] { requireInBand(0.75, "t"); }),
         "대역 위 마하는 거부된다");
  expect(throws([&] { requireInBand(0.13, "t"); }),
         "TR 1096 의 시험 마하 0.13 조차 **대역 밖이라 거부된다** — "
         "기준점이지 사용점이 아니다");
  expect(!throws([&] { requireInBand(kMachMin, "t"); }),
         "대역 하단 경계는 통과");
  expect(!throws([&] { requireInBand(kMachMax, "t"); }),
         "대역 상단 경계는 통과");
  expect(throws([&] { requireInBand(std::nan(""), "t"); }),
         "비유한 마하는 거부된다");

  // ── 미회수 입력은 fail-closed ───────────────────────────
  Tr1096Anchor anchor;              // available = false
  DownwashLag downwash;             // available = false
  expect(throws([&] { pitchDamping(0.50, g, anchor, downwash); }),
         "기준점 미회수면 Cm_q 를 내지 않는다");
  anchor.available = true;
  anchor.cm_q_total_per_rad = -10.0;
  anchor.cl_alpha_htail_per_rad = 3.0;
  expect(throws([&] { pitchDamping(0.50, g, anchor, downwash); }),
         "다운워시 항 미회수면 Cm_q 를 내지 않는다 (CL_alpha 와 분리 계약)");
  downwash.available = true;
  downwash.factor = 1.0;
  expect(throws([&] { pitchDamping(0.50, g, anchor, downwash); }),
         "압축성 관계 미회수면 Cm_q 를 내지 않는다 — 단순 "
         "1/sqrt(1-M^2) 로 대체하지 않는다");
  expect(throws([&] { liftSlopeRatio(0.50, 0.13, 4.0, 0.785); }),
         "압축성 보정 자체가 미회수 상태에서 던진다");

  // ── eq (3) 꼬리 기여: 부호와 형상 의존 ──────────────────
  const double tail = tailPitchDampingContribution(3.0, 1.0, g);
  expect(tail < 0.0, "꼬리 기여는 음수다 (감쇠)");
  // 식의 **형태**를 검사한다. 1차량에서 같은 방식으로 유도해 맞춘다 —
  // 매니페스트의 선언 비 1.8172 는 소수 4자리 반올림이라 1차량에서
  // 유도한 0.927837/0.510556 = 1.817308 과 1.1e-4 다르고, 그 차이를
  // 기대값에 쓰면 식이 아니라 반올림을 시험하게 된다.
  const double arm = 0.927837 / 0.510556;
  const double expected = -2.0 * 3.0 * (0.200000 / 1.0) * arm * arm;
  expect(std::fabs(tail - expected) < 1e-12,
         "eq (3) per-radian 형태가 -2 CLα (S_H/S)(l/c)^2 와 일치");
  // 매니페스트의 중복 선언(l/c 를 따로 적어 둔 것)이 1차량과 어긋나지
  // 않는지도 같이 본다. 어긋나면 형상 전사가 틀린 것이다.
  const double from_declared = -2.0 * 3.0 * 0.20 * 1.8172 * 1.8172;
  expect(std::fabs(tail - from_declared) / std::fabs(tail) < 1e-3,
         "선언된 l/c 로 계산해도 1e-3 안에서 같다 (매니페스트 자기정합)");
  // 꼬리거리 2배 -> 기여 4배 (제곱 의존). TR 1096 의 핵심 주장이다.
  Geometry g2 = g;
  g2.tail_arm_m = g.tail_arm_m * 2.0;
  const double tail2 = tailPitchDampingContribution(3.0, 1.0, g2);
  expect(std::fabs(tail2 / tail - 4.0) < 1e-9,
         "꼬리거리를 2배로 하면 감쇠 기여가 4배 — 거리의 제곱 의존");
  Geometry g3 = g;
  g3.htail_area_m2 = g.htail_area_m2 * 2.0;
  const double tail3 = tailPitchDampingContribution(3.0, 1.0, g3);
  expect(std::fabs(tail3 / tail - 2.0) < 1e-9,
         "꼬리 면적을 2배로 하면 감쇠 기여가 2배 — 면적의 1차 의존");

  // ── 합과 단독은 섞이지 않는다 ───────────────────────────
  PitchDampingSum sum;
  sum.cm_q_plus_cm_alphadot_per_rad = -12.0;
  sum.mach = 0.5;
  sum.grade = Grade::MeasuredDigitized;
  expect(throws([&] { subtractAlphaDot(sum, -4.0, Grade::Unknown, "x"); }),
         "Cm_alphadot 등급이 UNKNOWN 이면 합에서 단독을 빼지 못한다");
  expect(throws([&] { subtractAlphaDot(sum, -4.0, Grade::Estimated, ""); }),
         "근거 문자열이 비면 빼지 못한다 — 근거 없는 뺄셈은 합을 "
         "단독으로 저장하는 것과 같다");
  const PitchDampingAlone alone =
      subtractAlphaDot(sum, -4.0, Grade::Estimated, "TR 1188 fig 23");
  expect(std::fabs(alone.cm_q_per_rad - (-8.0)) < 1e-12,
         "근거가 있으면 뺄셈이 성립한다");
  expect(alone.grade == Grade::Estimated,
         "뺀 결과의 등급은 ESTIMATED 다 (MEASURED 가 아니다)");

  std::printf("\n%s: %d failed\n", failures ? "FAIL" : "PASS", failures);
  return failures ? 1 : 0;
}
