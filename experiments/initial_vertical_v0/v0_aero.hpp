// v0 종방향 공력 계수 공급자. **비생산.**
//
// 계약 (3단계):
//   · TR 1096 의 M = 0.13 값을 Cm_q **단독** 기준점으로 쓴다
//   · CL_alpha,H 에는 TR 1188 의 유한 날개·후퇴각 압축성 관계를 적용한다.
//     단순 1/sqrt(1-M^2) 로 끝내지 않는다
//   · 동적 다운워시 항은 CL_alpha,H 와 **분리해** 계산한다
//   · 근거 없이 M = 0.13 값을 전 대역에 복사하지 않는다
//   · 결과 등급은 MEASURED 가 아니라 ESTIMATED 다
//   · (Cm_q + Cm_alphadot) 측정에서 나온 추세를 단독 Cm_q 의 합격 범위로
//     쓰지 않는다
#ifndef V0_AERO_HPP_
#define V0_AERO_HPP_

#include <cmath>
#include <stdexcept>
#include <string>

namespace v0aero {

// ─── 출처 등급 ──────────────────────────────────────────────
enum class Grade {
  Declared, DeclaredTest, ScaledPublic, MeasuredDigitized, Estimated, Unknown,
};
const char *gradeName(Grade g);

// ─── 무차원화 규약 ──────────────────────────────────────────
// **Cm_q 는 /rad 값이 아니다.** q c / (2V) 로 정규화된 무차원 미계수다.
// /rad 인 것은 CL_alpha 쪽이다. 이름에 per_rad 를 달면 언젠가 57.3 을
// 곱하거나 나누는 사고가 난다 — 그래서 규약을 값과 함께 실어 나른다.
//
// 문헌이 실제로 갈린다:
//   q c / (2V)  TR 1096, TR 1188
//   q l / V     일부 문헌 — 반감되지 않고 기준길이도 루트시위일 수 있다
// 규약이 다르면 2배(그리고 기준길이비의 제곱)만큼 어긋난다.
enum class PitchRateNorm {
  Unspecified,
  HalfChordOverV,   // q c_bar / (2 V)
  ChordOverV,       // q c / V         — 반감 없음
};
const char *pitchRateNormName(PitchRateNorm n);

// ─── 예외 ───────────────────────────────────────────────────
class OutOfRange : public std::runtime_error {
 public: explicit OutOfRange(const std::string &w) : std::runtime_error(w) {}
};
class Unavailable : public std::runtime_error {
 public: explicit Unavailable(const std::string &w) : std::runtime_error(w) {}
};
class NotClosed : public std::runtime_error {
 public: explicit NotClosed(const std::string &w) : std::runtime_error(w) {}
};

// ─── 적용 범위 ──────────────────────────────────────────────
constexpr double kMachMin = 0.3585;
constexpr double kMachMax = 0.7000;
void requireInBand(double mach, const char *what);

// 값 하나가 쓸 수 있는 상태인지 — 유한성·등급·근거를 **함께** 본다.
// available 플래그 하나로는 부족하다: NaN 이나 Unknown 이 섞인 채로
// 통과하는 경로가 생긴다.
void requireUsable(double value, Grade grade, const std::string &basis,
                   const char *what);

// ─── 감쇠: 단독과 합 ────────────────────────────────────────
// 필드를 공개 구조체로 두면 값을 복사해 다른 타입을 만들 수 있어서
// 암시적 변환만 막는 꼴이 된다. 생성자를 감추고 **검사를 거친 팩토리**
// 로만 만들게 한다 — 규약과 근거를 대지 못하면 아예 만들어지지 않는다.
class PitchDampingAlone {
 public:
  static PitchDampingAlone make(double cm_q, PitchRateNorm norm, double mach,
                                Grade grade, const std::string &basis);
  double value() const { return cm_q_; }          // 무차원
  PitchRateNorm norm() const { return norm_; }
  double mach() const { return mach_; }
  Grade grade() const { return grade_; }
  const std::string &basis() const { return basis_; }
  bool diagnosticOnly() const { return diagnostic_only_; }
  void markDiagnosticOnly(const std::string &why);
  const std::string &diagnosticReason() const { return diagnostic_reason_; }

 private:
  PitchDampingAlone() = default;
  double cm_q_ = 0.0;
  PitchRateNorm norm_ = PitchRateNorm::Unspecified;
  double mach_ = 0.0;
  Grade grade_ = Grade::Unknown;
  std::string basis_;
  bool diagnostic_only_ = false;
  std::string diagnostic_reason_;
};

class PitchDampingSum {
 public:
  static PitchDampingSum make(double cm_q_plus_cm_alphadot,
                              PitchRateNorm norm, double mach, Grade grade,
                              const std::string &basis);
  double value() const { return sum_; }
  PitchRateNorm norm() const { return norm_; }
  double mach() const { return mach_; }
  Grade grade() const { return grade_; }
  const std::string &basis() const { return basis_; }

 private:
  PitchDampingSum() = default;
  double sum_ = 0.0;
  PitchRateNorm norm_ = PitchRateNorm::Unspecified;
  double mach_ = 0.0;
  Grade grade_ = Grade::Unknown;
  std::string basis_;
};

// 합에서 단독으로 가는 **유일한** 경로. Cm_alphadot 의 근거를 요구한다.
PitchDampingAlone subtractAlphaDot(const PitchDampingSum &sum,
                                   double cm_alphadot, PitchRateNorm norm,
                                   Grade grade, const std::string &basis);

// ─── 형상 ───────────────────────────────────────────────────
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
// 꼬리 on 과 off 를 **둘 다** 회수해야 한다. 하나만으로는 식 (3) 을
// 검증할 수 없다 — 아래 폐쇄 검사 참조.
struct Tr1096Anchor {
  double mach = 0.13;
  double reynolds = 0.71e6;
  double cm_q_total = 0.0;         // 꼬리 on, 무차원
  double cm_q_tail_off = 0.0;      // 꼬리 off, 무차원
  double cl_alpha_htail_per_rad = 0.0;   // 이쪽은 정말 /rad 다
  double read_error = 0.0;         // 그림 판독 오차 (무차원, Cm_q 와 같은 단위)
  PitchRateNorm norm = PitchRateNorm::Unspecified;
  Grade grade = Grade::Unknown;
  std::string basis;
  bool available = false;
};

// ─── 다운워시 항 (CL_alpha 와 분리) ─────────────────────────
struct DownwashLag {
  double factor = 1.0;             // (1 - d eps_r / d(ql/V))
  Grade grade = Grade::Unknown;
  std::string basis;
  bool available = false;
};

// ─── 압축성 (TR 1188) ───────────────────────────────────────
struct Compressibility {
  double ratio_to_anchor = 1.0;    // CL_alpha(M) / CL_alpha(M_anchor)
  Grade grade = Grade::Unknown;
  std::string basis;
  bool available = false;
};
Compressibility liftSlopeRatio(double mach, double mach_anchor,
                               double aspect_ratio, double sweep_c4_rad);

// ─── 비꼬리(날개+동체) 기여의 마하 의존 ─────────────────────
// **이것을 마하 무관으로 두는 것은 공개 근거가 아니라 추정이다.**
// 그래서 별도 입력으로 뺀다. 미회수면 거부하고, 진단 전용으로 쓰려면
// 호출부가 그 사실을 명시해야 하며 결과에 표시가 남는다.
struct NonTailMachModel {
  double ratio_to_anchor = 1.0;
  Grade grade = Grade::Unknown;
  std::string basis;
  bool available = false;
  bool diagnostic_only = true;
};

// ─── 식 (3) 의 꼬리 기여 ────────────────────────────────────
//   (dCm_q)_H = -2 CL_alpha_H[/rad] (1 - d eps/d(ql/V)) (S_H/S)(l/c)^2
// 원문 상수 -114.6 은 per-degree 계수용이고 -114.6 = -2 x 57.3 이다.
double tailPitchDampingContribution(double cl_alpha_htail_per_rad,
                                    double downwash_factor,
                                    const Geometry &g);

// ─── 폐쇄 검사: 식 (3) 대 실측 on/off 차이 ──────────────────
// **이것이 없으면 식 (3) 은 검증되지 않는다.** 나머지를
// (실측 total - 계산한 tail) 로 정의해 버리면 식이 틀려도 항상
// 재조립되어 오차가 나머지에 숨는다. 실측 (total - tail_off) 와
// 식 (3) 의 예측을 **독립적으로** 비교해 판독 오차 안에서 닫혀야 한다.
struct TailClosure {
  double measured_delta = 0.0;     // 실측 total - 실측 tail_off
  double predicted_delta = 0.0;    // 식 (3)
  double residual = 0.0;
  double tolerance = 0.0;          // 판독 오차에서 전파
  bool closed = false;
};
TailClosure checkTailClosure(const Tr1096Anchor &anchor, const Geometry &g,
                             const DownwashLag &downwash);

// ─── 조립: Cm_q(M) ──────────────────────────────────────────
// 폐쇄 검사를 통과하지 못하면 던진다. 비꼬리 마하 모델이 진단 전용이면
// 결과에 그 표시가 남는다.
PitchDampingAlone pitchDamping(double mach, const Geometry &g,
                               const Tr1096Anchor &anchor,
                               const DownwashLag &downwash,
                               const NonTailMachModel &non_tail,
                               bool allow_diagnostic = false);

}  // namespace v0aero

#endif  // V0_AERO_HPP_
