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

// **합은 일반 6DOF 전파에 쓸 수 없다.** 두 계수는 일반 운동방정식에서
// 서로 다른 입력에 곱해진다:
//   Cm_q      -> q c/(2V)        (몸체 피치율)
//   Cm_alphadot -> alphadot c/(2V) (받음각 변화율)
// q 와 alphadot 은 같은 양이 아니다 — 정상 선회나 돌풍 중에는 크게
// 갈린다. 강제진동 시험이 둘을 합으로만 내는 것은 그 시험에서 두 입력이
// 묶여 움직이기 때문이지, 물리적으로 하나여서가 아니다.
//
// 따라서 이 타입은:
//   · 별도 타입으로 보관한다 (Cm_q 로 변환 금지 — subtractAlphaDot 만이
//     유일한 경로이고 Cm_alphadot 의 근거를 요구한다)
//   · **그 시험 조건에서의 총 감쇠 진단에만** 쓴다
//   · 일반 6DOF 전파에는 두 항이 분리되기 전까지 UNKNOWN 이다
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
// **이름이 곧 계약이다.** 두 실측값의 차이는 수평꼬리 단독 기여가
// 아니라 `V+H2 구성 증분`이다 — 꼬리 off 기준 W+F2 에는 수직꼬리가
// 없고 꼬리 on 형상에는 있기 때문이다. 그 차이에 CL_alpha,H 의 마하
// 보정을 통째로 거는 것은 **근거가 없다**: 섞여 있는 수직꼬리 기여가
// 같은 비율로 변한다는 보장이 없다.
struct Tr1096Anchor {
  double mach = 0.13;              // 대역 [0.3585, 0.7000] **밖**이다
  double reynolds = 0.71e6;
  // 전체 형상 W+F2+V+H2 (그림 9(b) / 10(a))
  double cm_q_whole_config = 0.0;
  // 날개+동체 W+F2, **수직꼬리 없음** (같은 그림)
  double cm_q_wing_fuselage = 0.0;
  double cl_alpha_htail_per_rad = 0.0;   // 이쪽은 정말 /rad 다
  double read_error_whole = 0.0;   // 그림 판독 오차 (Cm_q 와 같은 무차원)
  double read_error_wing_fuselage = 0.0;
  PitchRateNorm norm = PitchRateNorm::Unspecified;
  Grade grade = Grade::Unknown;
  std::string basis;
  bool available = false;

  // 두 실측의 차이. **수평꼬리 단독이 아니다.** 이름으로 못박는다.
  double configurationIncrementVPlusH() const {
    return cm_q_whole_config - cm_q_wing_fuselage;
  }
};

// ─── 다운워시 항 (CL_alpha 와 분리) ─────────────────────────
struct DownwashLag {
  double factor = 1.0;             // (1 - d eps_r / d(ql/V))
  Grade grade = Grade::Unknown;
  std::string basis;
  bool available = false;
};

// ─── 압축성 ─────────────────────────────────────────────────
// **TR 1188 에는 유한 날개 압축성 관계가 없다.** 전제가 틀렸다 — 그
// 문서의 압축성 내용은 2차원 단순 Prandtl-Glauert 가 전부이고, 유한
// 스팬도 후퇴각도 다루지 않는다. 필요한 관계는 NACA TN 3911
// (Lowry & Polhamus, 1957) eq. (A1) 이다:
//
//   CL_alpha = 2 pi A / [ 2 + sqrt( 4 + (A/cos L)^2 - (A M)^2 ) ]   /rad
//
// L 은 **반시위 후퇴각**이다(TN 3911 이 명시). 테이퍼비는 들어가지
// 않는다 — 반시위 기준이 테이퍼 효과를 흡수하기 때문이다.
//
// 왜 단순 Prandtl-Glauert 로 끝내면 안 되는지가 여기서 수치로 보인다.
// A=4.0, 반시위 후퇴각 43.15° 에서 M 0.13 -> 0.70 의 CL_alpha 비는
// 1.0972 인데 단순 PG 는 1.3884 를 준다 — 대역 상단에서 **26.5% 과대**.
// 평면 무한날개(A -> inf, L = 0)에서만 단순 PG 가 맞는다.
//
// 1/4시위 후퇴각에서 반시위 후퇴각으로:
//   tan L_c2 = tan L_c4 - (4/A) ( (0.5-0.25)(1-taper)/(1+taper) )
double halfChordSweepRad(double sweep_c4_rad, double aspect_ratio,
                         double taper_ratio);

// TN 3911 eq. (A1). 관계 자체는 검산했으나 **출처 감사가 끝나기
// 전에는 쓰지 않는다** — liftSlopeRatio 가 계속 거부한다.
double liftSlopeTn3911PerRad(double mach, double aspect_ratio,
                             double half_chord_sweep_rad);

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

// ─── 식 (3) — **민감도·스케일링 진단 전용** ─────────────────
//   (dCm_q)_H = -2 CL_alpha_H[/rad] (1 - d eps/d(ql/V)) (S_H/S)(l/c)^2
// 원문 상수 -114.6 은 per-degree 계수용이고 -114.6 = -2 x 57.3 이다.
//
// **이 함수의 값은 실측 절대값 보정에도, 전체 형상의 마하 외삽에도
// 쓰지 않는다.** 우리 형상에서 실측과 17.5% 어긋났다(아래 폐쇄 진단).
// 남은 역할은 수평꼬리 형상을 바꿨을 때 감쇠가 어느 방향으로 얼마나
// 움직이는지 보는 것 하나뿐이다.
double tailSensitivityEq3(double cl_alpha_htail_per_rad,
                          double downwash_factor, const Geometry &g);

// ─── 폐쇄 진단: 식 (3) 대 실측 구성 증분 ────────────────────
// 실측 (전체 - 날개동체) 와 식 (3) 예측을 **독립 비교**한다. 나머지를
// (실측 전체 - 계산한 꼬리) 로 정의하면 식이 틀려도 항상 재조립되어
// 오차가 나머지에 숨는다.
//
// **결과: 우리 형상에서 닫히지 않는다.** 실측 -3.48, 식 (3) -4.088,
// 잔차 -0.608, 허용 0.1118 → 식 (3) 이 17.5% 과대예측.
// 원문의 "d eps_r/d(ql/V) 는 사실상 0" 은 **앙상블 진술**이고
// 개별 형상에서는 흩어진다(H1 비 0.98 / H2 0.85 / H3 1.07).
//
// tolerance 는 **판독 오차만** 합친 폭이다. 전체 모델 불확실성이
// 아니다 — 형상 오차, 레이놀즈수 차이, 수직꼬리 오염은 들어 있지 않다.
struct ClosureDiagnostic {
  double measured_increment = 0.0;   // 실측 (전체 - 날개동체) = V+H2 증분
  double eq3_prediction = 0.0;
  double residual = 0.0;
  double read_error_tolerance = 0.0; // **판독 오차만**
  bool within_read_error = false;
};
ClosureDiagnostic closureDiagnostic(const Tr1096Anchor &anchor,
                                    const Geometry &g,
                                    const DownwashLag &downwash);

// ─── 대역 내 Cm_q — 현재 UNKNOWN ────────────────────────────
// **성공 경로가 없다.** 이것은 미구현이 아니라 판정이다:
//
//   · M = 0.13 실측은 전체 형상(W+F2+V+H2) 것이고 대역 밖이다
//   · 실측 차이 -3.48 은 V+H2 구성 증분이라 CL_alpha,H 의 마하 보정을
//     통째로 걸 근거가 없다 — 수직꼬리 기여가 같은 비율로 변한다는
//     보장이 없다
//   · 식 (3) 은 우리 형상에서 실측과 17.5% 어긋나 절대값 보정에
//     쓸 수 없다
//
// 다음 둘 중 하나가 생기기 전까지 목표 대역의 Cm_q 는 UNKNOWN 이다:
//   (a) 수직꼬리와 수평꼬리 기여의 분리 근거
//   (b) 전체 형상의 **대역 내** 자료
//
// 그때까지 이 함수는 항상 던진다. 값을 내는 분기를 두지 않는 것이
// 이 판정을 코드에 남기는 방법이다.
[[noreturn]] void pitchDampingInBand(double mach, const Geometry &g,
                                     const Tr1096Anchor &anchor);

}  // namespace v0aero

#endif  // V0_AERO_HPP_
