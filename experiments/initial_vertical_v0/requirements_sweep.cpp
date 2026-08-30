// 요구조건 역산 — **물리 예측이 아니다.** 비생산. **조건부 초안.**
//
// 능력(T/W, CL_max, W/S, V0)이 주어졌을 때 **성공하는 정책이 존재하는가**를
// 묻는다. 능력은 상한이지 명령이 아니다.
//
// 이전 판의 네 가지 의미 충돌을 고친 것이다. 무엇이 왜 틀렸는지 남긴다.
//
//  1. T/W 를 가용 능력이 아니라 **고정 명령**으로 썼다. 전 구간 전출력을
//     밀었으므로 높은 T/W 의 과속은 능력 한계가 아니라 스로틀 정책의
//     결과였다. → 추력을 [0, T_max] 안에서 **선택**한다.
//  2. CL_max 도 능력과 명령 포화가 섞였다. 한계가 커지면 정책이 더 세게
//     꺾어 속력을 더 잃었다 — 가용 최대치가 커지는 것이 가능한 명령
//     집합을 줄일 수는 없으므로 이는 정책의 산물이다.
//     → 실현 가능성을 **능력 집합 안에 성공하는 정책이 존재하는가**로
//       정의한다. 그러면 단조성이 구조적으로 성립하고 회귀가 검사한다.
//  3. 경로 모델과 포락선 판정이 **서로 다른 기체**를 봤다. 경로는 사례별
//     W/S·T/W·CL_max 를 쓰는데 handoffVerdict 에는 생산 mp 를 그대로
//     넘겨, W/S 1500 사례도 판정기는 12748.6 으로 봤다.
//     → 사례별 case_mp 를 만들어 **양쪽에 같이** 적용한다.
//  4. 실패 사유와 출력 속력이 **다른 시각**이었다. 사유는 마지막 정렬
//     표본, 속력은 최저 경로각 오차 시점이라 168.6 m/s 옆에 "speed above
//     maximum" 이 찍혔다. → 같은 시각의 증인을 한 묶음으로 보관한다.
//
// 중기 포락선 판정은 복제하지 않는다 — 생산과 같은 handoffVerdict 다.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <string>
#include <vector>

#include <mmp_vehicle_dynamics/flight_dynamics.hpp>

namespace vd = mmp_vehicle_dynamics;

namespace {

constexpr double kDeg = M_PI / 180.0;

bool loadProductionYaml(const std::string &path,
                        std::map<std::string, double> *out)
{
  std::FILE *f = std::fopen(path.c_str(), "rb");
  if (!f) return false;
  std::string text;
  char buf[4096];
  size_t n;
  while ((n = std::fread(buf, 1, sizeof(buf), f)) > 0) text.append(buf, n);
  std::fclose(f);
  size_t pos = 0;
  while (pos < text.size()) {
    size_t eol = text.find('\n', pos);
    if (eol == std::string::npos) eol = text.size();
    std::string line = text.substr(pos, eol - pos);
    pos = eol + 1;
    const size_t hash = line.find('#');
    if (hash != std::string::npos) line = line.substr(0, hash);
    const size_t colon = line.find(':');
    if (colon == std::string::npos) continue;
    std::string key = line.substr(0, colon), val = line.substr(colon + 1);
    const size_t ks = key.find_first_not_of(" \t");
    if (ks == std::string::npos) continue;
    key = key.substr(ks, key.find_last_not_of(" \t") - ks + 1);
    const size_t vs = val.find_first_not_of(" \t\r");
    if (vs == std::string::npos) continue;
    val = val.substr(vs);
    char *end = nullptr;
    const double d = std::strtod(val.c_str(), &end);
    if (end != val.c_str()) (*out)[key] = d;
  }
  return true;
}

// ─── 능력 (상한이지 명령이 아니다) ──────────────────────────
struct Capability {
  double tw_max, ws, v0, cl_max;
};

// ─── 정책 ───────────────────────────────────────────────────
// **한계를 절대값으로 든다.** 가용치의 비율로 들면 능력이 커질 때 같은
// 정책이 다른 궤적을 만들어, 작은 능력에서 되던 것을 큰 능력이 재현하지
// 못한다 — 첫 회귀가 잡은 위반 7건이 정확히 그것이었다.
//
// 절대값으로 두고 능력으로 자르면, 능력이 커질 때 정책 집합이 **포함
// 관계로 커진다**. 단조성이 구조적으로 성립하고 회귀는 그것을 확인한다.
struct Policy {
  double turn_gain;      // gamma 오차 → 요구 선회율 [1/s]
  double cl_limit_abs;   // 명령 CL 상한 (절대값). 능력으로 잘린다
  double tw_cmd_abs;     // 명령 추력하중 (절대값). 능력으로 잘린다
};

// 같은 시각의 증인. 사유와 수치가 갈리지 않게 한 묶음으로 든다.
struct Witness {
  double t = 0.0, v = 0.0, gamma = 0.0, h = 0.0, cl = 0.0, util = 0.0;
  vd::HandoffReject reject = vd::HandoffReject::None;
  bool aligned = false;
  bool valid = false;
  double violation = 1e9;   // 정규화 위반량 — 정렬된 표본에서만 의미
  double gamma_err = 1e9;   // 목표 경로각 오차 [rad]
};

// 증인 순위. **정렬 이력이 있는 것이 항상 우선**이고, 그 안에서 위반량이
// 작은 쪽이 낫다. 정렬이 한 번도 없었으면 경로각 오차로 비교한다.
// 두 부류를 한 수로 섞으면 정렬 없는 증인이 violation = 1e9 때문에
// 상위 선택에서 통째로 버려진다.
struct WitnessRank { int tier; double value; };
WitnessRank rankOf(const Witness &w)
{
  return w.aligned ? WitnessRank{0, w.violation}
                   : WitnessRank{1, w.gamma_err};
}
bool better(const Witness &a, const Witness &b)
{
  if (!b.valid) return a.valid;
  if (!a.valid) return false;
  const auto ra = rankOf(a), rb = rankOf(b);
  if (ra.tier != rb.tier) return ra.tier < rb.tier;
  return ra.value < rb.value;
}

// 실패 증인을 "경로각 오차 최소" 로 고르면 **더 공격적이라 속력을 많이
// 잃은 정책**이 뽑힌다. 그러면 능력이 커질수록 나빠 보이는데, 그것은
// 능력 악화가 아니라 증인 선택 기준의 변화다. 그래서 **포락선 위반량**
// 으로 고른다.
double normalisedViolation(const vd::HandoffVerdict &v, double floor_mps,
                           double vmax_mps, const vd::Parameters &p)
{
  switch (v.reject) {
    case vd::HandoffReject::None: return 0.0;
    case vd::HandoffReject::SpeedBelowFloor:
      return (floor_mps - v.speed_mps) / std::max(floor_mps, 1.0);
    case vd::HandoffReject::SpeedAboveMaximum:
      return (v.speed_mps - vmax_mps) / std::max(vmax_mps, 1.0);
    case vd::HandoffReject::FlightPathOutsideCone:
      // 콘은 **크기** 판정이다. speedConeVerdict 가 이미 atan2(|vz|, vh)
      // 로 크기를 내지만, 부호 있는 각이 흘러들어도 안전하도록 명시한다.
      return (std::fabs(v.flight_path_angle_rad) - p.flight_path_angle_max_rad)
           / std::max(p.flight_path_angle_max_rad, 1e-6);
    case vd::HandoffReject::EnvelopeExceeded:
      return std::max(0.0, v.utilization - 1.0);
    default: return 1e6;   // 비유한·역동역학 미정의
  }
}

// 초기 전환 성공의 정의:
//   수직 상태에서 목표 경로각 허용구간에 **위쪽에서 최초 진입**하고,
//   진입 전 목표구간 아래로 지나치거나 고도가 감소하지 않은 상태에서
//   필요한 체류시간과 중기 포락선을 만족하는 것.
//
// "하강 금지" 만으로는 부족하다. 속력이 양수면 gamma<0 금지와 고도 단조
// 증가는 거의 같은 조건이고, 둘 다 **gamma 가 목표구간 하한 아래(예:
// +5°)까지 지나쳤다가 +20° 로 복귀하는 경우**를 잡지 못한다.
enum class Outcome {
  DirectTransition,    // 목표구간에 위쪽에서 최초 진입
  OvershootRecovery,   // 목표구간 하한 아래로 갔다가 복귀
  DescentRecovery,     // 실제 고도 감소 후 복귀
  NoHandoff,           // 끝까지 인계 조건 미충족
};
const char *outcomeName(Outcome o)
{
  switch (o) {
    case Outcome::DirectTransition: return "DIRECT_TRANSITION";
    case Outcome::OvershootRecovery: return "OVERSHOOT_RECOVERY";
    case Outcome::DescentRecovery: return "DESCENT_RECOVERY";
    default: return "NO_HANDOFF";
  }
}

// 수치 진동 허용오차. 없으면 적분 잡음이 오버슈트로 오판된다.
constexpr double kAltTol = 1.0;            // m
constexpr double kGammaTol = 0.05 * kDeg;  // rad

// 이력 분류를 **적분기에서 떼어** 단위로 시험할 수 있게 한다.
// 자연 실행에서 OVERSHOOT 분기가 한 번도 안 밟히므로, ODE 를 통해서만
// 시험하면 그 분기는 검증되지 않은 채 남는다.
class HistoryTracker {
 public:
  HistoryTracker(double band_lo, double band_hi, double alt0)
      : lo_(band_lo), hi_(band_hi), h_max_(alt0) {}
  void update(double gamma, double h)
  {
    if (h > h_max_) h_max_ = h;
    if (h < h_max_ - kAltTol) descended_ = true;
    if (gamma < lo_ - kGammaTol) went_below_ = true;
    if (!entered_ && gamma <= hi_ && gamma >= lo_ && !went_below_ && !descended_)
      entered_ = true;
  }
  bool entered() const { return entered_; }
  bool descended() const { return descended_; }
  bool wentBelow() const { return went_below_; }
  Outcome classify(bool handoff_met) const
  {
    if (!handoff_met) return Outcome::NoHandoff;
    if (descended_) return Outcome::DescentRecovery;
    if (went_below_) return Outcome::OvershootRecovery;
    return Outcome::DirectTransition;
  }

 private:
  double lo_, hi_, h_max_;
  bool entered_ = false, descended_ = false, went_below_ = false;
};

struct RunResult {
  Outcome outcome = Outcome::NoHandoff;
  bool reached = false;
  Witness at_handoff;
  Witness best_witness;
  double cl_peak = 0.0;        // 실제로 쓴 최대 |CL| — 요구량이다
  double thrust_tw_used = 0.0; // 실제로 쓴 추력하중
  // "도달" 이 진짜 초기 전환인지 가리는 이력. |gamma-목표|<2° 만 보면
  // 오버슛으로 내려갔다가 하강 중 가속해 다시 올라오며 목표를 지나는
  // 순간도 도달로 센다 — 그건 초기 전환이 아니다.
  bool descended_before = false;   // 도달 전에 gamma < 0 을 지났는가
  double gamma_min_before = 1e9;   // 도달 전 최저 gamma [rad]
  double alt_at_handoff = 0.0;
  double alt_gain = 0.0;           // 도달 시 고도 - 초기 고도
};

// 사례별 기체. 경로와 판정이 **같은 기체**를 보게 하는 핵심.
//
// **이 도구는 가상 기체 지도다.** 생산 기체의 인계 계약을 고정하는 것이
// 아니다. 두 해석이 섞이면 안 된다 — W/S 와 CL_max 를 바꾸면서 순항하한만
// 생산값으로 고정하면, 사례 기체의 실속속도와 무관한 하한을 강요하게 된다.
//
// 그래서 speed_min 을 사례 기체의 **실속속도**로 다시 세운다:
//   V_stall = sqrt( 2 (W/S) / (rho0 CL_max) )
// 생산 사례(W/S 12748.6, CL_max 1.40)에서 121.93 이 나와 생산 YAML 의
// speed_min 122.0("v_stall = 121.9 @ 1300kg")을 재현한다 — 아래에서
// 그 일치를 실행 시 검사한다.
vd::Parameters caseParameters(const vd::Parameters &base, const Capability &c)
{
  vd::Parameters p = base;
  const double weight = p.mass_kg * p.gravity_mps2;
  p.wing_area_m2 = weight / c.ws;
  p.thrust_max_n = c.tw_max * weight;
  p.lift_coefficient_max = c.cl_max;
  p.speed_min_mps =
      std::sqrt(2.0 * c.ws / (p.sea_level_density_kgpm3 * c.cl_max));
  return p;
}

RunResult runOne(const Capability &c, const Policy &pol,
                 const vd::Parameters &cmp, double gamma_target,
                 double alt0, double floor_mps, double vmax_mps,
                 double dwell_required, double dt, double t_max)
{
  RunResult r;
  const double g = cmp.gravity_mps2;
  const double m_over_s = c.ws / g;
  // 정책의 절대 한계를 능력으로 자른다. 이것이 포함 관계를 만든다.
  const double cl_use = std::min(pol.cl_limit_abs, c.cl_max);
  const double thrust_accel = std::min(pol.tw_cmd_abs, c.tw_max) * g;
  double v = c.v0, gamma = 0.5 * M_PI, h = alt0, dwell = 0.0;
  r.thrust_tw_used = std::min(pol.tw_cmd_abs, c.tw_max);
  // 목표구간 [하한, 상한]. 위쪽에서 들어와야 직접 전환이다.
  const double band_lo = gamma_target - 2.0 * kDeg;
  const double band_hi = gamma_target + 2.0 * kDeg;
  HistoryTracker hist(band_lo, band_hi, alt0);
  const long steps = static_cast<long>(t_max / dt);

  for (long k = 0; k <= steps && v > 1.0; ++k) {
    const double t = k * dt;
    const double q = 0.5 * vd::airDensity(cmp, h) * v * v;

    // 명령은 정책이 만든다. 능력은 여기서 **자르기만** 한다 —
    // 상한이 커진다고 더 세게 꺾지 않는다.
    const double gerr = gamma - gamma_target;
    const double lift_accel_want = v * (-pol.turn_gain * gerr)
                                 + g * std::cos(gamma);
    double cl = (q > 1.0) ? m_over_s * lift_accel_want / q : 0.0;
    cl = std::max(-cl_use, std::min(cl_use, cl));
    r.cl_peak = std::max(r.cl_peak, std::fabs(cl));

    const double lift_accel = (q > 1.0) ? cl * q / m_over_s : 0.0;
    const double cd = vd::dragCoefficient(cmp, std::fabs(cl));
    const double drag_accel = (q > 1.0) ? cd * q / m_over_s : 0.0;

    const double at = thrust_accel - drag_accel - g * std::sin(gamma);
    const double an = lift_accel - g * std::cos(gamma);
    v += at * dt;
    gamma += (an / std::max(v, 1.0)) * dt;
    h += v * std::sin(gamma) * dt;
    if (h < 0.0 || !std::isfinite(v) || !std::isfinite(gamma)) break;

    // ── 이력 판정 ──────────────────────────────────────────
    r.gamma_min_before = std::min(r.gamma_min_before, gamma);
    hist.update(gamma, h);
    // 체류는 최초 진입 이후에만 쌓인다 — 아래 dwell 누적이
    // `w.aligned && hist.entered()` 가드 안에 있기 때문이다. 여기서
    // 따로 0 으로 되돌리는 줄을 두었다가 **도달 불가능한 죽은 코드**임을
    // 변이시험이 드러내 지웠다. 남겨두면 그 줄이 조건을 지킨다고
    // 오해하게 된다.

    // ── 같은 시각에 증인을 만든다 ──────────────────────────
    const double err_now = std::fabs(gamma - gamma_target);
    Witness w;
    w.t = t; w.v = v; w.gamma = gamma; w.h = h; w.cl = cl; w.valid = true;
    w.aligned = err_now < 2.0 * kDeg;
    if (w.aligned && hist.entered()) {
      const Eigen::Vector3d pos(0.0, 0.0, h);
      const Eigen::Vector3d vel(v * std::cos(gamma), 0.0, v * std::sin(gamma));
      const Eigen::Vector3d t_hat(std::cos(gamma), 0.0, std::sin(gamma));
      const Eigen::Vector3d n_hat(-std::sin(gamma), 0.0, std::cos(gamma));
      const auto verdict = vd::handoffVerdict(cmp, pos, vel,
                                              at * t_hat + an * n_hat,
                                              floor_mps, vmax_mps);
      w.reject = verdict.reject;
      w.util = verdict.utilization;
      w.violation = normalisedViolation(verdict, floor_mps, vmax_mps, cmp);
      if (verdict.ok()) {
        dwell += dt;
        // hist.entered() 를 여기서도 다시 본다. 위 w.aligned 가드와
        // **중복**이라 하나만 지우는 변이는 죽지 않는다(확인함). 둘을
        // 동시에 지우면 OVERSHOOT/DESCENT 궤적이 인계로 들어와 회귀가
        // 깨진다 — 계약이 하중을 받는 지점은 이 둘의 조합이다.
        if (dwell >= dwell_required && hist.entered()) {
          r.reached = true;
          r.at_handoff = w;
          r.alt_at_handoff = h;
          r.alt_gain = h - alt0;
          r.outcome = hist.classify(true);
          r.descended_before = hist.descended();
          return r;
        }
      } else {
        dwell = 0.0;
      }
    } else {
      dwell = 0.0;
    }
    w.gamma_err = err_now;
    if (better(w, r.best_witness)) r.best_witness = w;
  }
  return r;
}

// 실현 가능성 = **능력 집합 안에 성공하는 정책이 존재하는가.**
// 이렇게 정의해야 능력이 커질 때 가능성이 줄지 않는다.
struct Feasibility {
  bool feasible = false;
  Policy winner{};
  Witness witness{};
  int policies_tried = 0;
};

Feasibility feasible(const Capability &c, const vd::Parameters &base,
                     double gamma_target, double alt0,
                     double vmax_mps, double dwell, double dt, double tmax)
{
  Feasibility f;
  const vd::Parameters cmp = caseParameters(base, c);
  const double floor_mps = vd::marginBackedSpeedFloorMps(cmp);
  for (double kg : {0.05, 0.10, 0.20, 0.35, 0.50})
    for (double cll : {0.3, 0.6, 1.0, 1.4, 2.0, 3.0})
      for (double twc : {0.0, 0.15, 0.25, 0.40, 0.60, 0.80, 1.00}) {
      const Policy p{kg, cll, twc};
      ++f.policies_tried;
      const auto r = runOne(c, p, cmp, gamma_target, alt0, floor_mps,
                            vmax_mps, dwell, dt, tmax);
      if (r.reached) {
        f.feasible = true; f.winner = p; f.witness = r.at_handoff;
        return f;
      }
      if (better(r.best_witness, f.witness)) {
        f.witness = r.best_witness; f.winner = p;
      }
      }
  return f;
}

const char *witnessReason(const Witness &w)
{
  if (!w.valid) return "실행 없음";
  if (!w.aligned) return "경로각 미도달";
  return vd::handoffRejectName(w.reject);
}

}  // namespace

int main(int argc, char **argv)
{
  double alt0 = 2000.0, gamma_tgt = 20.0, dwell = 1.0, dt = 0.01, tmax = 200.0;
  bool monotone = false, design_space = false, self_test = false;
  bool attitude = false;
  double iyy = 4200.0, mac = 0.510556, m_ctrl = 9000.0;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto v = [&]() { return (i + 1 < argc) ? std::atof(argv[++i]) : 0.0; };
    if (a == "--alt") alt0 = v();
    else if (a == "--gamma-target-deg") gamma_tgt = v();
    else if (a == "--dwell") dwell = v();
    else if (a == "--dt") dt = v();
    else if (a == "--monotone-check") monotone = true;
    else if (a == "--design-space") design_space = true;
    else if (a == "--self-test") self_test = true;
    else if (a == "--attitude") attitude = true;
    else if (a == "--iyy") iyy = v();
    else if (a == "--mac") mac = v();
    else if (a == "--m-control") m_ctrl = v();
    else { std::fprintf(stderr, "알 수 없는 인자: %s\n", a.c_str()); return 2; }
  }

  vd::Parameters base;
  std::map<std::string, double> Y;
  const std::string yaml = "../../src/path_manager/config/optimizer_params.yaml";
  if (!loadProductionYaml(yaml, &Y)) {
    std::fprintf(stderr, "중단: 생산 YAML 을 못 읽었다: %s\n", yaml.c_str());
    return 2;
  }
  const struct { const char *k; double *d; } pulls[] = {
    {"optimization/dynamics_mass_kg", &base.mass_kg},
    {"optimization/dynamics_wing_area_m2", &base.wing_area_m2},
    {"optimization/dynamics_g", &base.gravity_mps2},
    {"optimization/dynamics_rho0_kgpm3", &base.sea_level_density_kgpm3},
    {"optimization/dynamics_density_scale_height_m", &base.density_scale_height_m},
    {"optimization/dynamics_cd0", &base.zero_lift_drag_coefficient},
    {"optimization/dynamics_induced_drag_factor", &base.induced_drag_factor},
    {"optimization/dynamics_cl_max", &base.lift_coefficient_max},
    {"optimization/dynamics_load_factor_max", &base.load_factor_max},
    {"optimization/dynamics_thrust_max_n", &base.thrust_max_n},
    {"optimization/dynamics_speed_min_mps", &base.speed_min_mps},
    {"optimization/dynamics_speed_max_mps", &base.speed_max_mps},
    {"optimization/dynamics_activation_speed_mps", &base.model_activation_speed_mps},
    {"optimization/dynamics_dynamic_pressure_max_pa", &base.dynamic_pressure_max_pa},
    {"optimization/dynamics_margin", &base.constraint_margin},
  };
  for (const auto &p : pulls) {
    auto it = Y.find(p.k);
    if (it == Y.end()) { std::fprintf(stderr, "중단: %s 없음\n", p.k); return 2; }
    *p.d = it->second;
  }
  base.bank_angle_max_rad = Y.at("optimization/dynamics_bank_max_deg") * kDeg;
  base.flight_path_angle_max_rad =
      Y.at("optimization/dynamics_flight_path_max_deg") * kDeg;
  const double vmax_mps = base.speed_max_mps;
  // 생산 사례에서 재계산한 실속속도가 생산 YAML 의 speed_min 을 재현하는지
  // 확인한다. 어긋나면 사례별 하한 자체를 믿을 수 없다.
  {
    const auto prod = caseParameters(base, {0.25, 12748.6, 230.0, 1.40});
    if (std::fabs(prod.speed_min_mps - 122.0) > 0.2) {
      std::fprintf(stderr, "중단: 사례별 실속 재계산이 생산 speed_min 을 "
                   "재현하지 못한다 (%.3f vs 122.0)\n", prod.speed_min_mps);
      return 2;
    }
  }

  const std::vector<double> tws{0.25, 0.40, 0.60, 0.80, 1.00};
  const std::vector<double> wss{1500.0, 3000.0, 6000.0, 12748.6};
  const std::vector<double> v0s{180.0, 230.0};
  const std::vector<double> clmaxes{0.6, 1.0, 1.4, 2.0, 3.0};

  // ── 단조성 회귀 ───────────────────────────────────────────
  // 능력이 커질 때 실현 가능성이 **줄면 안 된다.** 줄면 능력과 정책이
  // 다시 섞인 것이다. 이번 수정의 핵심 불변조건이다.
  if (monotone) {
    int viol = 0;
    auto check = [&](const char *axis, const std::vector<double> &axis_vals,
                     auto make) {
      for (double ws : wss)
        for (double v0 : v0s)
          for (double other : (std::string(axis) == "T/W" ? clmaxes : tws)) {
            bool seen = false;
            for (double a : axis_vals) {
              const Capability c = make(a, ws, v0, other);
              const bool f = feasible(c, base, gamma_tgt * kDeg, alt0,
                                      vmax_mps, dwell, dt, tmax).feasible;
              if (seen && !f) {
                std::printf("!! 단조성 위반 (%s): W/S %.0f V0 %.0f 기타 %.2f "
                            "— 더 큰 능력 %.2f 에서 불가\n",
                            axis, ws, v0, other, a);
                ++viol;
              }
              seen = seen || f;
            }
          }
    };
    check("T/W", tws, [](double a, double ws, double v0, double o) {
      return Capability{a, ws, v0, o}; });
    check("CL_max", clmaxes, [](double a, double ws, double v0, double o) {
      return Capability{o, ws, v0, a}; });
    std::printf("\n단조성 회귀: 위반 %d 건\n", viol);
    return viol == 0 ? 0 : 1;
  }

  std::printf("요구조건 역산 — **물리 예측이 아니다. 조건부 초안.**\n");
  std::printf("  주 분석은 **생산 계약 고정**이다. --design-space 는 부록.\n");
  std::printf("  능력이 주어졌을 때 **성공하는 정책이 존재하는가**를 묻는다.\n"
              "  능력은 상한이지 명령이 아니다. 사례마다\n"
              "  wing_area = mg/(W/S) · thrust_max = (T/W)mg · CL_max = 사례값\n"
              "  으로 case_mp 를 만들어 **경로와 포락선 판정에 같이** 적용한다.\n\n");
  std::printf("  목표 %.1f° · 고도 %.0f m · dwell %.2f s · 상한 %.1f m/s\n",
              gamma_tgt, alt0, dwell, vmax_mps);
  std::printf("  순항하한은 **사례마다** 그 기체의 실속속도에서 다시 세운다:\n"
              "    V_stall = sqrt(2 (W/S)/(rho0 CL_max)), floor = V_stall (1+margin)\n"
              "  생산 사례에서 121.93 → 생산 YAML 의 speed_min 122.0 재현 확인.\n\n");
  // ── 음성대조 ────────────────────────────────────────────
  // 스윕이 정말 조건을 보고 있는지. 통과하면 아무것도 검사하지 않는 것이다.
  {
    const auto z = feasible({1.00, 12748.6, 230.0, 1e-6}, base,
                            gamma_tgt * kDeg, alt0, vmax_mps, dwell, dt, tmax);
    std::printf("  [음성대조] CL_max ~ 0: %s\n",
                z.feasible ? "발견 — **고장**" : "미발견 (정상)");
    if (z.feasible) return 3;
    const auto y = feasible({1.00, 12748.6, 230.0, 1.40}, base,
                            89.9 * kDeg, alt0, vmax_mps, dwell, dt, tmax);
    std::printf("  [음성대조] 목표 89.9° (콘 30° 밖): %s\n\n",
                y.feasible ? "발견 — **고장**" : "미발견 (정상)");
    if (y.feasible) return 3;
  }

  if (attitude) {
    // ── B. 회전 권한 — **무차원량으로만 보고한다** ──────────
    // 정확한 관성을 역산했다고 말하지 않는다. 실제 질량배치 자료가
    // 없으므로 참값도 그 범위도 모른다.
    std::printf("[B] 회전 권한 — **무차원량 보고. 관성 역산이 아니다.**\n\n");
    const double m = base.mass_kg;
    const double S = base.wing_area_m2;
    const double c_bar = mac;
    std::printf("  기준량: m %.1f kg · S %.3f m² · c̄ %.6f m\n", m, S, c_bar);
    std::printf("  선언 Iyy %.1f kg·m² — **근거 없는 시험값**이다.\n\n", iyy);

    // (1) 피치 각가속도 권한
    std::printf("  (1) 제어 회전 권한  M_control / Iyy\n");
    std::printf("      선언 제어 모멘트 %.1f N·m / Iyy %.1f = **%.5f rad/s²**"
                " (%.3f °/s²)\n", m_ctrl, iyy, m_ctrl / iyy,
                m_ctrl / iyy / kDeg);
    std::printf("      Iyy 가 바뀌면 이 값이 선형으로 바뀐다. 그래서 아래\n"
                "      무차원 관성과 함께 읽어야 한다.\n\n");

    // (2) 무차원 관성 — **기준길이를 명시**한다
    std::printf("  (2) 무차원 관성  Iyy / (m c̄²)\n");
    std::printf("      %.1f / (%.1f × %.6f²) = **%.4f**  (기준길이 c̄ = %.6f m)\n",
                iyy, m, c_bar, iyy / (m * c_bar * c_bar), c_bar);
    std::printf("      **기준길이를 밝히지 않은 무차원 관성은 비교 불가**다.\n"
                "      그리고 이 값이 비슷하다는 이유로 다른 기체를 같다고\n"
                "      말할 수 없다 — 질량배치·공력이 함께 맞아야 한다.\n\n");

    // (3) 피치 모멘트 기여 분해 — 계수가 없으면 fail-closed
    std::printf("  (3) 전체 피치 모멘트의 기여 분해\n");
    std::printf("      M = q S c̄ [ Cm_alpha·α + Cm_q·(q c̄/2V)\n"
                "                  + Cm_alphadot·(α̇ c̄/2V) ] + M_control\n\n");
    std::printf("      %8s %10s | %14s %14s %14s %12s\n", "V[m/s]", "고도[m]",
                "정적 Cm_α", "회전감쇠 Cm_q", "α̇감쇠 Cm_αdot", "제어");
    // 배율은 q S c̄ = (1/2 rho V²) S c̄ 이므로 **rho 와 S, c̄ 를 함께
    // 밝혀야** 다른 조건과 비교할 수 있다. 배율만 적으면 어느 밀도·
    // 기준면적에서 나온 수인지 사라진다.
    for (double v : {150.0, 200.0, 230.0}) {
      const double rho = vd::airDensity(base, 2000.0);
      const double q = 0.5 * rho * v * v;
      const double scale = q * S * c_bar;      // 계수 1 당 모멘트 [N·m]
      std::printf("      %8.0f %10.0f | %14s %14s %14s %12.1f\n",
                  v, 2000.0, "UNKNOWN", "UNKNOWN", "UNKNOWN", m_ctrl);
      std::printf("      %8s %10s   계수 1 당 %.1f N·m  = q %.1f Pa × S %.3f m² "
                  "× c̄ %.6f m  (rho %.5f kg/m³)\n",
                  "", "", scale, q, S, c_bar, rho);
    }
    std::printf("\n  (4) 출처와 사용 가능 범위\n");
    std::printf("      Cm_q        대역 내 UNKNOWN. TR 1096 이 M=0.13 에서\n"
                "                  **단독으로** 다루지만(전체 형상 -5.48,\n"
                "                  규약 q c̄/2V) 대역 밖이고, 식 (3) 이 실측\n"
                "                  구성 증분과 닫히지 않았다(SOURCES.md).\n");
    std::printf("      Cm_alphadot 회수 자료 없음. UNKNOWN.\n");
    std::printf("      둘의 합     TR 1188 대역 내 그림과 NTRS 20150018562\n"
                "                  계측이 합만 준다. **일반 6DOF 전파에 쓸 수\n"
                "                  없다** — 두 계수는 q 와 α̇ 라는 서로 다른\n"
                "                  입력에 곱해진다. 그 시험 조건에서의 총\n"
                "                  감쇠 진단에만 쓰고, 분리 전까지 UNKNOWN.\n");
    std::printf("      Cm_alpha    대역 내 UNKNOWN (공력중심 이동 근거 없음).\n");

    std::printf("\n  (5) 판정 — fail-closed\n");
    std::printf("      네 계수가 모두 UNKNOWN 이므로 **정착시간을 내지 않는다.**\n"
                "      정착시간은 관성만이 아니라 정적 안정성·감쇠 두 항·\n"
                "      동압·속력 변화·제어 입력을 포함한 시간응답으로만\n"
                "      측정된다. Cm_q 하나로 계산한 이전 판의 정착시간\n"
                "      (572 s / 79 s)은 Cm_alphadot 이 빠져 **무효이며 철회**한다.\n");
    std::printf("\n  결론: 현재 모델이 요구하는 회전 권한을 무차원량으로\n"
                "  표현했으며, 실제 관성·감쇠 자료가 공급되면 같은 계약으로\n"
                "  6DOF 응답을 판정할 수 있다. **현 공개 자료만으로 실제\n"
                "  기체의 정착시간은 확정할 수 없다.**\n");
    std::printf("\n[ATTITUDE] m_ctrl_over_iyy=%.5f iyy_nondim=%.4f "
                "cbar=%.6f cm_alpha=UNKNOWN cm_q=UNKNOWN "
                "cm_alphadot=UNKNOWN settle=UNDETERMINED\n",
                m_ctrl / iyy, iyy / (m * c_bar * c_bar), c_bar);
    return 0;
  }

  if (self_test) {
    // 합성 이력으로 **네 분기 전부** 시험한다. 자연 실행에서는
    // OVERSHOOT 가 0 이라 그 분기가 검증되지 않은 채 남는다.
    int fails = 0;
    auto expect = [&](bool ok, const char *what) {
      std::printf("  %s  %s\n", ok ? "[OK]  " : "[FAIL]", what);
      if (!ok) ++fails;
    };
    const double lo = 18.0 * kDeg, hi = 22.0 * kDeg;

    {   // 위쪽에서 곧장 진입, 고도 계속 상승
      HistoryTracker t(lo, hi, 2000.0);
      for (double g : {90.0, 60.0, 40.0, 25.0, 20.0, 19.5})
        t.update(g * kDeg, 2000.0 + (90.0 - g) * 10.0);
      expect(t.entered(), "DIRECT: 위쪽에서 최초 진입이 인식된다");
      expect(t.classify(true) == Outcome::DirectTransition,
             "DIRECT: 인계 충족 시 DIRECT_TRANSITION");
      expect(t.classify(false) == Outcome::NoHandoff,
             "NONE: 인계 미충족이면 NO_HANDOFF (이력과 무관)");
    }
    {   // 목표구간 하한 아래로 지나쳤다가 복귀. **고도는 계속 상승.**
      HistoryTracker t(lo, hi, 2000.0);
      const double gs[] = {90.0, 40.0, 20.0, 5.0, 12.0, 20.0};
      double h = 2000.0;
      for (double g : gs) { h += 50.0; t.update(g * kDeg, h); }
      expect(t.wentBelow(), "OVERSHOOT: 하한 아래 통과가 기록된다");
      expect(!t.descended(), "OVERSHOOT: 고도는 감소하지 않았다");
      expect(t.classify(true) == Outcome::OvershootRecovery,
             "OVERSHOOT: 고도 감소 없이 하한을 지나치면 "
             "OVERSHOOT_RECOVERY — 하강 금지만으로는 못 잡는 경우다");
    }
    {   // 실제 고도 감소 후 복귀
      HistoryTracker t(lo, hi, 2000.0);
      const double gs[] = {90.0, 40.0, 20.0, -10.0, 5.0, 20.0};
      const double hs[] = {2000.0, 2500.0, 2600.0, 2400.0, 2300.0, 2350.0};
      for (int i = 0; i < 6; ++i) t.update(gs[i] * kDeg, hs[i]);
      expect(t.descended(), "DESCENT: 고도 감소가 기록된다");
      expect(t.classify(true) == Outcome::DescentRecovery,
             "DESCENT: 고도가 감소했으면 하한 통과보다 DESCENT 가 우선");
    }
    {   // **하강한 뒤에야** 목표구간에 처음 들어오는 경우.
        // entered_ 가 켜지면 안 된다 — 이것이 최초 진입 가드의 존재
        // 이유다. 현재 스윕 격자는 이 궤적을 만들지 않으므로(수직에서
        // 내려오며 첫 통과 때 이미 진입) 합성으로만 덮인다.
      HistoryTracker t(lo, hi, 2000.0);
      const double gs[] = {90.0, 60.0, 40.0, 30.0, -20.0, 0.0, 20.0};
      const double hs[] = {2000, 2400, 2700, 2900, 2500, 2300, 2350};
      for (int i = 0; i < 7; ++i) t.update(gs[i] * kDeg, hs[i]);
      expect(!t.entered(),
             "가드: 하강 후에 목표구간에 처음 들어오면 최초 진입이 "
             "아니다 (분류만으로는 못 막는 경우)");
      expect(t.descended(), "가드: 그 경우 하강 이력이 남는다");
    }
    {   // 목표구간에 아예 못 들어옴
      HistoryTracker t(lo, hi, 2000.0);
      for (double g : {90.0, 80.0, 70.0, 60.0})
        t.update(g * kDeg, 2000.0 + (90.0 - g) * 10.0);
      expect(!t.entered(), "NONE: 목표구간 미진입");
      expect(t.classify(false) == Outcome::NoHandoff, "NONE: NO_HANDOFF");
    }
    {   // 허용오차: 잡음이 오버슈트로 오판되면 안 된다
      HistoryTracker t(lo, hi, 2000.0);
      double h = 2000.0;
      for (double g : {90.0, 30.0, 20.0, 19.99, 20.0}) {
        h += 10.0; t.update(g * kDeg, h);
      }
      expect(!t.wentBelow(),
             "허용오차: 하한 아래 0.01° 진동은 오버슈트가 아니다");
      HistoryTracker t2(lo, hi, 2000.0);
      double h2 = 2000.0;
      for (double g : {90.0, 30.0, 20.0}) { h2 += 10.0; t2.update(g*kDeg, h2); }
      t2.update(20.0 * kDeg, h2 - 0.5);     // 0.5 m 하강 — 허용오차 안
      expect(!t2.descended(), "허용오차: 0.5 m 하강은 감소로 세지 않는다");
      t2.update(20.0 * kDeg, h2 - 5.0);     // 5 m — 허용오차 밖
      expect(t2.descended(), "허용오차: 5 m 하강은 감소로 센다");
    }
    std::printf("\n분류 자기시험: %s (%d 실패)\n",
                fails ? "FAIL" : "PASS", fails);
    return fails ? 1 : 0;
  }

  if (design_space) {
    // ── 부록: 설계공간 민감도 (가상 기체 지도) ────────────
    std::printf("[부록] 설계공간 민감도 — **주 분석이 아니다.**\n");
    std::printf("  W/S·CL_max 를 바꾸면 그 기체의 순항하한도 함께 움직인다\n"
                "  (V_stall = sqrt(2(W/S)/(rho0 CL_max))). 즉 목표가 기체를\n"
                "  따라가므로 **자기정합적**이고, 현재 시스템의 요구사항을\n"
                "  말하지 않는다. 다른 시스템끼리 비교하는 별도 설계 연구다.\n\n");
    int n_ok = 0, n_tot = 0;
    for (double tw : {0.25, 0.40, 0.60, 0.80, 1.00})
      for (double ws : {1500.0, 3000.0, 6000.0, 12748.6})
        for (double v0 : {180.0, 230.0})
          for (double clm : {0.6, 1.0, 1.4, 2.0, 3.0}) {
            ++n_tot;
            if (feasible({tw, ws, v0, clm}, base, gamma_tgt * kDeg, alt0,
                         vmax_mps, dwell, dt, tmax).feasible) ++n_ok;
          }
    std::printf("  시험한 210개 정책 집합에서 성공 발견 %d / %d 조합\n",
                n_ok, n_tot);
    return 0;
  }

  // ── 주 분석: 생산 계약 고정 ─────────────────────────────
  // W/S · CL_max · 포락선 · 순항하한을 생산값으로 **고정**한다. 초기
  // 구간이 현재 중기 플래너에 연결되는지가 질문이므로, 목표가 기체를
  // 따라 움직이면 안 된다.
  const double ws_prod = base.mass_kg * base.gravity_mps2 / base.wing_area_m2;
  const double tw_prod = base.thrust_max_n
                       / (base.mass_kg * base.gravity_mps2);
  const double floor_prod = vd::marginBackedSpeedFloorMps(base);
  std::printf("[주 분석] 생산 계약 고정\n");
  std::printf("  W/S %.1f N/m² · CL_max %.2f · T/W %.4f · 순항하한 %.2f m/s "
              "· 상한 %.1f m/s\n", ws_prod, base.lift_coefficient_max,
              tw_prod, floor_prod, vmax_mps);
  std::printf("  변화축: 초기속력 · 선회이득 · CL 명령 한계 · 추력 명령\n");
  std::printf("  보고: 성공 개수가 아니라 **최소 요구량**\n\n");

  // 한 행의 값들이 **같은 정책**에서 나와야 한다. 서로 다른 정책의
  // 극값을 한 줄에 모으면 함께 일어난 적 없는 조합을 읽게 된다.
  //
  // 그리고 **요구량 집계에는 DIRECT_TRANSITION 만 쓴다.** 나머지 셋은
  // 진단으로만 남긴다 — 오버슈트 후 복귀나 하강 후 복귀는 초기 전환이
  // 아니므로 초기 구간 요구량이 될 수 없다.
  struct Sel {
    const char *label;
    bool found = false;
    Policy pol{};
    double cl_used = 0.0, t_handoff = 0.0, v_handoff = 0.0, v_loss = 0.0;
    double tw_used = 0.0, gamma_min_deg = 0.0, alt_gain = 0.0;
  };

  std::printf("  판정 넷으로 가른다. **요구량 집계는 DIRECT_TRANSITION 만.**\n");
  std::printf("    DIRECT_TRANSITION  목표구간에 위쪽에서 최초 진입\n");
  std::printf("    OVERSHOOT_RECOVERY 목표구간 하한 아래로 갔다가 복귀\n");
  std::printf("    DESCENT_RECOVERY   실제 고도 감소 후 복귀\n");
  std::printf("    NO_HANDOFF         끝까지 인계 조건 미충족\n\n");

  for (double v0 : {150.0, 180.0, 200.0, 230.0}) {
    const Capability cap{tw_prod, ws_prod, v0, base.lift_coefficient_max};
    const vd::Parameters cmp = caseParameters(base, cap);
    int n_direct = 0, n_over = 0, n_desc = 0, n_none = 0;
    Sel lowest_cl{"최저 CL 한계"}, zero_thrust{"추력 명령 0"},
        fastest{"최단 전환"}, least_loss{"최소 속력손실"};
    for (double kg : {0.05, 0.10, 0.20, 0.35, 0.50})
      for (double cll : {0.05, 0.10, 0.20, 0.30, 0.60, 1.00, 1.40})
        for (double twc : {0.0, 0.15, 0.25, 0.40, 0.60, 0.80, 1.00}) {
          const Policy pol{kg, cll, twc};
          const auto r = runOne(cap, pol, cmp, gamma_tgt * kDeg, alt0,
                                floor_prod, vmax_mps, dwell, dt, tmax);
          switch (r.outcome) {
            case Outcome::DirectTransition: ++n_direct; break;
            case Outcome::OvershootRecovery: ++n_over; break;
            case Outcome::DescentRecovery: ++n_desc; break;
            default: ++n_none; break;
          }
          // **여기서 걸러진다.** 직접 전환만 요구량 집계에 든다.
          if (r.outcome != Outcome::DirectTransition) continue;
          const double loss = v0 - r.at_handoff.v;
          auto fill = [&](Sel &sl) {
            sl.found = true; sl.pol = pol; sl.cl_used = r.cl_peak;
            sl.t_handoff = r.at_handoff.t; sl.v_handoff = r.at_handoff.v;
            sl.v_loss = loss; sl.tw_used = r.thrust_tw_used;
            sl.gamma_min_deg = r.gamma_min_before / kDeg;
            sl.alt_gain = r.alt_gain;
          };
          if (!lowest_cl.found || cll < lowest_cl.pol.cl_limit_abs)
            fill(lowest_cl);
          if (twc == 0.0 && (!zero_thrust.found
                             || r.at_handoff.t < zero_thrust.t_handoff))
            fill(zero_thrust);
          if (!fastest.found || r.at_handoff.t < fastest.t_handoff)
            fill(fastest);
          if (!least_loss.found || loss < least_loss.v_loss)
            fill(least_loss);
        }
    std::printf("  V0 %.0f — 정책 판정 분포: DIRECT %d · OVERSHOOT %d · "
                "DESCENT %d · NONE %d\n", v0, n_direct, n_over, n_desc, n_none);
    if (n_direct == 0) {
      std::printf("    시험한 정책 격자에서 **직접 전환 없음**\n\n");
      std::printf("[SWEEP] v0=%.0f direct=%d over=%d desc=%d none=%d "
                  "witness=none\n\n", v0, n_direct, n_over, n_desc, n_none);
      continue;
    }
    std::printf("    %-18s %6s %6s %6s %6s %7s %7s %7s %6s\n", "선정 기준",
                "이득", "CL한계", "실제CL", "T/W", "전환[s]", "속력손실",
                "고도증가", "최저γ");
    for (const Sel *sl : {&lowest_cl, &zero_thrust, &fastest, &least_loss}) {
      if (!sl->found) {
        std::printf("    %-20s %s\n", sl->label, "직접 전환 중에는 없음");
        continue;
      }
      std::printf("    %-18s %6.2f %6.2f %6.3f %6.2f %7.2f %7.1f %7.0f %6.1f\n",
                  sl->label, sl->pol.turn_gain, sl->pol.cl_limit_abs,
                  sl->cl_used, sl->tw_used, sl->t_handoff, sl->v_loss,
                  sl->alt_gain, sl->gamma_min_deg);
    }
    std::printf("[SWEEP] v0=%.0f direct=%d over=%d desc=%d none=%d "
                "fastest_t=%.2f fastest_cl=%.2f fastest_kg=%.2f "
                "fastest_tw=%.2f fastest_loss=%.1f "
                "lowcl_cl=%.2f lowcl_kg=%.2f lowcl_t=%.2f\n\n",
                v0, n_direct, n_over, n_desc, n_none, fastest.t_handoff,
                fastest.pol.cl_limit_abs, fastest.pol.turn_gain,
                fastest.tw_used, fastest.v_loss,
                lowest_cl.pol.cl_limit_abs, lowest_cl.pol.turn_gain,
                lowest_cl.t_handoff);
  }

  std::printf("  ※ 생산 T/W 는 %.4f 로 고정이므로 표의 T/W 는 **명령된**\n"
              "    값이다. 추력 명령 0 은 \"중력만으로 선회\" 가 아니다 —\n"
              "    CL 을 쓰므로 정확히는 **공력과 중력으로 전환에 성공한\n"
              "    정책이 시험 집합 안에 있었다** 는 뜻이다. 자세·각속도\n"
              "    동역학이 없으므로 실제 6DOF 가능성을 증명하지 않는다.\n",
              tw_prod);
  std::printf("  ※ 여기 나온 수는 **최소 제원이 아니다.** 시험한 격자에서\n"
              "    발견된 직접 전환 증인일 뿐이다. 격자 사이와 시간에 따라\n"
              "    변하는 정책은 탐색하지 않았다.\n");
  std::printf("  ※ 최소 CL 한계는 **시험한 격자값** 중 최소다. 그 사이는\n"
              "    찾지 않았으므로 진짜 하한이 아니다.\n");
  std::printf("  ※ 점질량이며 받음각·자세 동역학이 없다. CL 을 받음각으로\n"
              "    환산하려면 CL_alpha 가 필요한데 목표 대역 근거가 없다\n"
              "    (SOURCE_GAP.md). 따라서 \"양력만 있으면 된다\" 같은\n"
              "    일반화는 이 도구로 할 수 없다.\n");
  return 0;
}
