// v0 종방향 공력 계수 공급자. **비생산.**
//
// 계약 (사용자 지시, 3단계):
//   · TR 1096 의 M = 0.13 값을 Cm_q **단독** 기준점으로 쓴다
//   · CL_alpha,H 에는 TR 1188 의 유한 날개·후퇴각 압축성 관계를 적용한다.
//     단순 1/sqrt(1-M^2) 로 끝내지 않는다
//   · 동적 다운워시 항은 CL_alpha,H 와 **분리해** 계산한다
//   · 근거 없이 M = 0.13 값을 전 대역에 복사하지 않는다
//   · 결과 등급은 MEASURED 가 아니라 ESTIMATED 다
//
// 그리고 가장 중요한 제한:
//
//   (Cm_q + Cm_alphadot) 측정에서 나온 추세를 **단독 Cm_q 의 합격 범위로
//   쓰면 안 된다.** 그 추세는 나중에 Cm_alphadot 까지 구성했을 때
//   합계의 진단값으로만 비교한다.
//
// 이 헤더는 그 제한을 규율이 아니라 **타입**으로 강제한다. 단독과 합은
// 서로 다른 타입이고 서로 변환되지 않는다. 합을 단독이 필요한 자리에
// 넣으면 컴파일이 실패한다 — 주석으로 적어 둔 규칙은 언젠가 깨지지만
// 타입은 깨지지 않는다.
#ifndef V0_AERO_HPP_
#define V0_AERO_HPP_

#include <cmath>
#include <stdexcept>
#include <string>

namespace v0aero {

// ─── 출처 등급 ──────────────────────────────────────────────
// 매니페스트의 등급과 같은 어휘를 쓴다. 값 하나하나가 어디서 왔는지
// 코드에서도 따라갈 수 있어야 한다.
enum class Grade {
  Declared,           // 생산 계약값
  DeclaredTest,       // 이 실험이 선언한 시험값
  ScaledPublic,       // 공개 문헌의 무차원량을 스케일
  MeasuredDigitized,  // 공개 문헌의 그림에서 판독 (판독 오차 동반)
  Estimated,          // 공개 방법으로 추정
  Unknown,            // 미상 — 필수면 fail-closed
};

const char *gradeName(Grade g);

// ─── 적용 범위 ──────────────────────────────────────────────
// 122 m/s(해면) ~ 230 m/s(3000 m). 밖에서는 값을 내지 않는다.
// 저속 시험값을 대역 밖까지 늘려 쓰는 것이 정확히 금지된 일이다.
constexpr double kMachMin = 0.3585;
constexpr double kMachMax = 0.7000;

class OutOfRange : public std::runtime_error {
 public:
  explicit OutOfRange(const std::string &what) : std::runtime_error(what) {}
};

class Unavailable : public std::runtime_error {
 public:
  explicit Unavailable(const std::string &what) : std::runtime_error(what) {}
};

void requireInBand(double mach, const char *what);

// ─── 감쇠: 단독과 합은 다른 타입이다 ────────────────────────
// 강제오차 진동 시험은 보통 합만 낸다. 그것을 단독으로 저장하는 순간
// 모형은 조용히 틀린다 — 두 항의 크기가 형상에 따라 비슷하기 때문에
// 결과가 그럴듯해 보이고, 그래서 잡히지 않는다.
//
// 두 타입 사이에 변환 연산자도, 변환 생성자도 두지 않는다. 합에서
// 단독을 얻으려면 Cm_alphadot 을 **따로** 세워 빼는 수밖에 없고, 그
// 뺄셈은 근거를 요구하는 명시적 함수여야 한다.
struct PitchDampingAlone {
  double cm_q_per_rad = 0.0;   // Cm_q, 규약 q c / (2V)
  Grade grade = Grade::Unknown;
  double mach = 0.0;
};

struct PitchDampingSum {
  double cm_q_plus_cm_alphadot_per_rad = 0.0;
  Grade grade = Grade::Unknown;
  double mach = 0.0;
};

// ─── 형상 (매니페스트에서 주입) ─────────────────────────────
// 공급자는 형상을 스스로 정하지 않는다. 매니페스트가 권위다.
struct Geometry {
  double wing_area_m2 = 0.0;
  double wing_span_m = 0.0;
  double wing_mac_m = 0.0;
  double wing_aspect_ratio = 0.0;
  double wing_sweep_c4_rad = 0.0;
  double htail_area_m2 = 0.0;
  double htail_aspect_ratio = 0.0;
  double htail_sweep_c4_rad = 0.0;
  double tail_arm_m = 0.0;
  double tail_arm_over_mac = 0.0;
  double htail_volume_coeff = 0.0;
};

// ─── TR 1096 기준점 (M = 0.13) ──────────────────────────────
// 원문이 Cm_q 를 **단독으로** 다룬다는 것이 이 문서를 쓰는 이유다:
//   "the total damping is determined by a combination of the rotary
//    derivative C_mq, which is considered herein, and the acceleration
//    derivative C_m_alphadot"  (p.20)
// 규약도 원문 그대로 q c / (2V) 다.
struct Tr1096Anchor {
  double mach = 0.13;              // 단일 시험 마하
  double reynolds = 0.71e6;        // 날개 MAC 기준
  double cm_q_total_per_rad = 0.0; // 전체 형상 (꼬리 포함)
  double cm_q_tail_off_per_rad = 0.0;
  double cl_alpha_htail_per_rad = 0.0;
  double read_error_per_rad = 0.0; // 그림 판독 오차
  Grade grade = Grade::Unknown;
  bool available = false;          // 수치 미회수면 false — fail-closed
};

// ─── 다운워시 항 (CL_alpha 와 분리) ─────────────────────────
// eq (3) 의 (1 - d(eps_r)/d(ql/V)) 인자. 계약상 CL_alpha,H 와 **따로**
// 계산해야 하므로 별도 구조체로 낸다. 값과 근거가 같이 다닌다.
struct DownwashLag {
  double factor = 1.0;             // (1 - d eps_r / d(ql/V))
  Grade grade = Grade::Unknown;
  std::string basis;               // 어느 문서의 어느 진술인지
  bool available = false;
};

// ─── 압축성 (TR 1188) ───────────────────────────────────────
// 유한 날개 + 후퇴각. 단순 Prandtl-Glauert 로 끝내지 않는다는 것이
// 계약이므로, 종횡비와 후퇴각이 실제로 들어가는지 검사할 수 있게
// 인자를 그대로 받는다.
struct Compressibility {
  double ratio_to_anchor = 1.0;    // CL_alpha(M) / CL_alpha(M_anchor)
  Grade grade = Grade::Unknown;
  bool available = false;
};

Compressibility liftSlopeRatio(double mach, double mach_anchor,
                               double aspect_ratio, double sweep_c4_rad);

// ─── 조립: Cm_q(M) ──────────────────────────────────────────
//   TR 1096 M=0.13 단독 Cm_q
//     → TR 1188 기반 CL_alpha,H 마하 보정
//     → 동적 다운워시 항 별도
//     → Cm_q(M)  등급 ESTIMATED
//
// 어느 입력이든 available 이 false 면 던진다. 값을 지어내지 않는다.
PitchDampingAlone pitchDamping(double mach, const Geometry &g,
                               const Tr1096Anchor &anchor,
                               const DownwashLag &downwash);

// eq (3) 의 꼬리 기여. 원문은 per-degree 계수에 -114.6 을 쓴다.
// -114.6 = -2 x 57.3 이므로 per-radian 에서는 상수가 -2 다.
//   (dCm_q)_H = -2 CL_alpha_H[/rad] (1 - d eps_r/d(ql/V)) (S_H/S)(l/c)^2
double tailPitchDampingContribution(double cl_alpha_htail_per_rad,
                                    double downwash_factor,
                                    const Geometry &g);

// ─── 합에서 단독을 빼는 것은 명시적이어야 한다 ──────────────
// 이 함수가 존재하는 이유는 편의가 아니라 **가시성**이다. 합에서
// 단독으로 가는 경로가 코드에 딱 한 군데만 있고, 그 자리에서
// Cm_alphadot 의 근거를 요구하게 만든다.
PitchDampingAlone subtractAlphaDot(const PitchDampingSum &sum,
                                   double cm_alphadot_per_rad,
                                   Grade cm_alphadot_grade,
                                   const std::string &basis);

}  // namespace v0aero

#endif  // V0_AERO_HPP_
