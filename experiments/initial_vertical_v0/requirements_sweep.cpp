// 요구조건 역산 — **물리 예측이 아니다.** 비생산.
//
// 계수를 맞히려는 것이 아니라, 초기 수직에서 중기 인계에 도달하려면
// **어떤 계수 범위가 필요한가**를 되묻는다. 어떤 조합부터 1초 dwell 과
// 중기 포락선을 함께 만족하는지 지도로 만든다.
//
// 이것이 말하지 않는 것:
//   · 그 계수를 실제 형상이 낼 수 있는가 (그건 공개 근거가 없다)
//   · 궤적이 정말 저렇게 될 것인가 (점질량 근사다)
//   · 어느 조합이 좋은가 (설계 권고가 아니다)
//
// 두 단계로 나눈다. 섞으면 무엇이 무엇을 요구했는지 사라진다.
//
//   A. 경로 단계 — 점질량. 중력선회에 필요한 CL 과 추력을 되묻는다.
//      출력: 필요 CL_max, 인계 도달 여부, 도달 시 속력, 포락선 판정.
//   B. 자세 단계 — 단주기 근사. A 가 요구한 받음각을 **만들고 유지**
//      하려면 필요한 Cm_alpha 와 Cm_q 를 되묻는다.
//
// 중기 포락선 판정은 복제하지 않는다 — 생산과 같은
// mmp_vehicle_dynamics::handoffVerdict 를 부른다.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <cstdio>
#include <map>

#include <mmp_vehicle_dynamics/flight_dynamics.hpp>

namespace vd = mmp_vehicle_dynamics;

namespace {

// 생산 YAML 을 읽는다. 하니스와 **같은 영역**을 써야 한다 — 구조체
// 기본값은 순항하한 122.40 이고 배포값은 131.76 이라 판정이 갈린다.
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

constexpr double kDeg = M_PI / 180.0;
constexpr double kRad2Deg = 180.0 / M_PI;

struct Case {
  double tw;          // 추력하중 T/W
  double ws;          // 면적하중 W/S [N/m^2]
  double v0;          // 초기 속력 [m/s]
  double cl_max;      // 가용 최대 양력계수
};

struct PathResult {
  bool reached = false;      // gamma 목표 도달
  double t_reach = 0.0;
  double v_reach = 0.0;
  double cl_peak = 0.0;      // 요구된 최대 CL
  double alpha_proxy_max = 0.0;  // CL_peak / CL_alpha 로 본 받음각 대용
  double dwell = 0.0;        // 인계 조건 연속 유지 시간
  bool envelope_ok = false;
  vd::EnvelopeLimit limit = vd::EnvelopeLimit::None;
  double util = 0.0;
  // 왜 못 갔는지. 속도 거절은 역동역학 이전에 끊겨 EnvelopeLimit 가
  // None 이므로 그것만 찍으면 정보가 없다.
  vd::HandoffReject last_reject = vd::HandoffReject::None;
  bool ever_aligned = false;
  double v_at_best_gamma = 0.0, best_gamma_err = 1e9;
};

// A. 경로 단계 — 점질량 중력선회.
//
//   V' = T/m cos(a) - D/m - g sin(gamma)
//   gamma' = ( L/m + T/m sin(a) - g cos(gamma) ) / V
//
// 받음각은 직접 풀지 않는다. **필요한 CL 을 되묻는 것**이 목적이므로
// gamma 를 목표로 끌어내리는 데 필요한 CL 을 매 스텝 계산하고 CL_max 로
// 자른다. 잘리면 그만큼 선회가 느려지고, 그 결과가 곧 답이다.
PathResult sweepPath(const Case &c, const vd::Parameters &mp,
                     double gamma_target_rad, double alt0,
                     double floor_mps, double vmax_mps,
                     double dwell_required, double dt, double t_max)
{
  PathResult r;
  const double g = mp.gravity_mps2;
  const double m_over_s = c.ws / g;          // 면적하중 [kg/m^2]
  const double thrust_accel = c.tw * g;      // T/m
  double v = c.v0, gamma = 0.5 * M_PI, h = alt0;
  const long steps = static_cast<long>(t_max / dt);

  for (long k = 0; k <= steps && v > 1.0; ++k) {
    const double t = k * dt;
    const double rho = vd::airDensity(mp, h);
    const double q = 0.5 * rho * v * v;
    // 이 gamma 에서 목표로 향하는 데 필요한 법선가속도.
    // gamma' 를 한 스텝에 목표까지 끌 수는 없으므로 비례로 요구한다.
    const double gamma_err = gamma - gamma_target_rad;
    const double gdot_want = -std::min(std::fabs(gamma_err) / (5.0 * dt),
                                       std::fabs(gamma_err) * 0.5)
                           * (gamma_err > 0 ? 1.0 : -1.0);
    // gamma' = (L/m - g cos gamma)/V  =>  L/m = V gamma' + g cos gamma
    const double lift_accel_want = v * gdot_want + g * std::cos(gamma);
    // CL = (m/S) a_L / q
    double cl = (q > 1.0) ? m_over_s * lift_accel_want / q : 0.0;
    cl = std::max(-c.cl_max, std::min(c.cl_max, cl));
    r.cl_peak = std::max(r.cl_peak, std::fabs(cl));

    const double lift_accel = (q > 1.0) ? cl * q / m_over_s : 0.0;
    const double cd = vd::dragCoefficient(mp, std::fabs(cl));
    const double drag_accel = (q > 1.0) ? cd * q / m_over_s : 0.0;

    v += (thrust_accel - drag_accel - g * std::sin(gamma)) * dt;
    gamma += ((lift_accel - g * std::cos(gamma)) / std::max(v, 1.0)) * dt;
    h += v * std::sin(gamma) * dt;
    if (h < 0.0) break;

    // 인계 조건: 경로각 정렬 + 속력 하한 + 중기 포락선.
    const double gerr = std::fabs(gamma - gamma_target_rad);
    if (gerr < r.best_gamma_err) { r.best_gamma_err = gerr; r.v_at_best_gamma = v; }
    const bool aligned = gerr < 2.0 * kDeg;
    if (aligned) r.ever_aligned = true;
    bool env_ok = false;
    if (aligned) {
      // 이 순간의 PVA 를 생산 판정에 건다.
      const Eigen::Vector3d pos(0.0, 0.0, h);
      const Eigen::Vector3d vel(v * std::cos(gamma), 0.0, v * std::sin(gamma));
      // 가속도: 접선 + 법선. 점질량이라 닫힌 형태로 만든다.
      const double at = thrust_accel - drag_accel - g * std::sin(gamma);
      const double an = lift_accel - g * std::cos(gamma);
      const Eigen::Vector3d t_hat(std::cos(gamma), 0.0, std::sin(gamma));
      const Eigen::Vector3d n_hat(-std::sin(gamma), 0.0, std::cos(gamma));
      const Eigen::Vector3d acc = at * t_hat + an * n_hat;
      const auto verdict = vd::handoffVerdict(mp, pos, vel, acc,
                                              floor_mps, vmax_mps);
      env_ok = verdict.ok();
      r.limit = verdict.limit;
      r.util = verdict.utilization;
      r.last_reject = verdict.reject;
    }
    if (aligned && env_ok) {
      r.dwell += dt;
      if (!r.reached && r.dwell >= dwell_required) {
        r.reached = true;
        r.t_reach = t;
        r.v_reach = v;
        r.envelope_ok = true;
      }
    } else {
      r.dwell = 0.0;
    }
  }
  return r;
}

// B. 자세 단계 — 단주기 근사.
//
// A 가 요구한 받음각을 **만들고 유지**하려면 무엇이 필요한가.
// 단주기 근사에서
//   omega_n^2 ~ -(q S c / Iyy) Cm_alpha
//   2 zeta omega_n ~ -(q S c^2 / (2 Iyy V)) (Cm_q + Cm_alphadot)
// 이므로, 요구 응답시간과 감쇠비를 주면 필요한 Cm_alpha 와 Cm_q 가 나온다.
//
// **Cm_q 와 Cm_alphadot 은 여기서 분리되지 않는다.** 단주기 근사가 둘을
// 합으로만 담기 때문이다. 그래서 출력 이름을 합으로 적는다 — 이 프로젝트가
// 합을 단독으로 저장하지 않기로 한 이유와 같다.
struct AttitudeNeed {
  double cm_alpha_per_rad = 0.0;
  double cm_q_plus_alphadot = 0.0;
  bool valid = false;
};

AttitudeNeed sweepAttitude(const Case &c, const vd::Parameters &mp,
                           double v, double alt, double iyy, double mac,
                           double settle_s, double zeta)
{
  AttitudeNeed n;
  const double rho = vd::airDensity(mp, alt);
  const double q = 0.5 * rho * v * v;
  const double S = mp.wing_area_m2;
  if (q <= 1.0 || iyy <= 0.0 || settle_s <= 0.0) return n;
  // 정착시간 ~ 4/(zeta omega_n)
  const double wn = 4.0 / (zeta * settle_s);
  n.cm_alpha_per_rad = -(wn * wn) * iyy / (q * S * mac);
  n.cm_q_plus_alphadot =
      -(2.0 * zeta * wn) * (2.0 * iyy * v) / (q * S * mac * mac);
  n.valid = true;
  return n;
}

}  // namespace

int main(int argc, char **argv)
{
  double alt0 = 2000.0, gamma_tgt = 20.0, dwell = 1.0, dt = 0.01, tmax = 200.0;
  double iyy = 4200.0, mac = 0.510556, settle = 2.0, zeta = 0.7;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto v = [&]() { return (i + 1 < argc) ? std::atof(argv[++i]) : 0.0; };
    if (a == "--alt") alt0 = v();
    else if (a == "--gamma-target-deg") gamma_tgt = v();
    else if (a == "--dwell") dwell = v();
    else if (a == "--iyy") iyy = v();
    else if (a == "--mac") mac = v();
    else if (a == "--settle") settle = v();
    else if (a == "--zeta") zeta = v();
    else { std::fprintf(stderr, "알 수 없는 인자: %s\n", a.c_str()); return 2; }
  }

  // 중기 포락선은 생산 설정이 정의한다. 여기서는 구조체 기본값을 쓰되
  // 그 사실을 출력한다 — 요구조건 분석이라 절대 판정이 아니다.
  vd::Parameters mp;
  std::map<std::string, double> Y;
  const std::string yaml =
      "../../src/path_manager/config/optimizer_params.yaml";
  if (!loadProductionYaml(yaml, &Y)) {
    std::fprintf(stderr, "중단: 생산 YAML 을 못 읽었다: %s\n"
                 "  포락선은 생산 설정이 정의한다. 기본값으로 대체하지 "
                 "않는다.\n", yaml.c_str());
    return 2;
  }
  const struct { const char *k; double *d; } pulls[] = {
    {"optimization/dynamics_mass_kg", &mp.mass_kg},
    {"optimization/dynamics_wing_area_m2", &mp.wing_area_m2},
    {"optimization/dynamics_g", &mp.gravity_mps2},
    {"optimization/dynamics_rho0_kgpm3", &mp.sea_level_density_kgpm3},
    {"optimization/dynamics_density_scale_height_m", &mp.density_scale_height_m},
    {"optimization/dynamics_cd0", &mp.zero_lift_drag_coefficient},
    {"optimization/dynamics_induced_drag_factor", &mp.induced_drag_factor},
    {"optimization/dynamics_cl_max", &mp.lift_coefficient_max},
    {"optimization/dynamics_load_factor_max", &mp.load_factor_max},
    {"optimization/dynamics_thrust_max_n", &mp.thrust_max_n},
    {"optimization/dynamics_speed_min_mps", &mp.speed_min_mps},
    {"optimization/dynamics_speed_max_mps", &mp.speed_max_mps},
    {"optimization/dynamics_activation_speed_mps", &mp.model_activation_speed_mps},
    {"optimization/dynamics_dynamic_pressure_max_pa", &mp.dynamic_pressure_max_pa},
    {"optimization/dynamics_margin", &mp.constraint_margin},
  };
  for (const auto &p : pulls) {
    auto it = Y.find(p.k);
    if (it == Y.end()) {
      std::fprintf(stderr, "중단: 생산 YAML 에 %s 가 없다\n", p.k);
      return 2;
    }
    *p.d = it->second;
  }
  {
    auto b = Y.find("optimization/dynamics_bank_max_deg");
    auto f = Y.find("optimization/dynamics_flight_path_max_deg");
    if (b == Y.end() || f == Y.end()) {
      std::fprintf(stderr, "중단: bank/flight_path 상한이 YAML 에 없다\n");
      return 2;
    }
    mp.bank_angle_max_rad = b->second * kDeg;
    mp.flight_path_angle_max_rad = f->second * kDeg;
  }
  const double floor_mps = vd::marginBackedSpeedFloorMps(mp);
  const double vmax_mps = mp.speed_max_mps;

  std::printf("요구조건 역산 — **물리 예측이 아니다.**\n");
  std::printf("  계수를 맞히려는 것이 아니라 인계에 도달하려면 어떤 계수\n"
              "  범위가 필요한지 되묻는다. 점질량 근사이며, 그 계수를 실제\n"
              "  형상이 낼 수 있는지는 **말하지 않는다.**\n\n");
  std::printf("  목표 경로각 %.1f° · 초기고도 %.0f m · dwell %.2f s\n",
              gamma_tgt, alt0, dwell);
  std::printf("  중기 포락선(생산 YAML): 순항하한 %.2f m/s · 상한 %.1f m/s · "
              "CL_max(모델) %.2f · n_max %.2f\n\n",
              floor_mps, vmax_mps, mp.lift_coefficient_max, mp.load_factor_max);

  // ── A. 경로 단계 ──────────────────────────────────────────
  const double tws[] = {0.25, 0.40, 0.60, 0.80, 1.00};
  const double wss[] = {1500.0, 3000.0, 6000.0, 12748.6};  // N/m^2
  const double v0s[] = {180.0, 230.0};
  const double clmaxes[] = {0.6, 1.0, 1.4, 2.0, 3.0};

  std::printf("[A] 경로 단계 — 어느 조합이 인계에 도달하는가\n");
  std::printf("  W/S 12748.6 N/m² 가 현재 선언값(1300 kg / 1.0 m²)이다.\n\n");
  std::printf("  %8s %10s %6s %8s | %6s %7s %8s  %s\n",
              "T/W", "W/S", "V0", "CL_max", "도달", "t[s]", "V", "사유");
  int n_reach = 0, n_total = 0;
  for (double tw : tws)
    for (double ws : wss)
      for (double v0 : v0s)
        for (double clm : clmaxes) {
          Case c{tw, ws, v0, clm};
          const auto r = sweepPath(c, mp, gamma_tgt * kDeg, alt0, floor_mps,
                                   vmax_mps, dwell, dt, tmax);
          ++n_total;
          if (r.reached) ++n_reach;
          const char *why =
              r.reached ? "-"
              : (!r.ever_aligned ? "경로각 미도달"
                 : vd::handoffRejectName(r.last_reject));
          if (r.reached || clm >= 1.4)
            std::printf("  %8.2f %10.1f %6.0f %8.2f | %6s %7.2f %8.1f  %s\n",
                        tw, ws, v0, clm, r.reached ? "예" : "아니오",
                        r.t_reach,
                        r.reached ? r.v_reach : r.v_at_best_gamma, why);
        }
  std::printf("\n  도달 %d / %d 조합\n\n", n_reach, n_total);

  // ── B. 자세 단계 ──────────────────────────────────────────
  // ── B. 자세 단계 ──────────────────────────────────────────
  std::printf("[B] 자세 단계 — **파라미터 집합의 내부 불일치가 먼저 걸린다**\n\n");

  // 관성 정합성부터. 선언 Iyy 는 "형상 미정 — 임의 시험값" 이고
  // 회전반경으로 되돌려 보면 형상과 맞지 않는다.
  const double body_len = 2.222222;   // TR 1096 F2 스케일
  const double k_declared = std::sqrt(iyy / mp.mass_kg);
  std::printf("  관성 정합성\n");
  std::printf("    선언 Iyy %.0f kg·m² → 회전반경 %.3f m · 동체길이 %.3f m "
              "→ k/L = %.3f\n", iyy, k_declared, body_len,
              k_declared / body_len);
  std::printf("    통상 항공기 k/L 은 0.25~0.35 다. 형상 정합 Iyy 는\n");
  for (double r : {0.25, 0.30, 0.35}) {
    const double k = r * body_len, I = mp.mass_kg * k * k;
    std::printf("      k/L %.2f → Iyy %7.1f kg·m² (선언값의 %.1f%%)\n",
                r, I, I / iyy * 100.0);
  }
  std::printf("    → 선언값은 형상 정합 범위의 약 %.0f 배다. B 단계의 요구량이\n"
              "      터무니없이 큰 것은 형상 요구가 아니라 **이 불일치의 산물**이다.\n\n",
              iyy / (mp.mass_kg * std::pow(0.30 * body_len, 2)));

  // 정방향: 실측 계수를 가진 형상이 얼마나 빨리 응답하나.
  // TR 1096 M=0.13 전체 형상 실측. 대역 밖이므로 **참고값**이다.
  const double cmq_measured = -5.48;
  std::printf("  정방향 — TR 1096 실측 Cm_q %.2f (M=0.13, 대역 밖 참고값)로\n"
              "  선언·정합 관성에서 나오는 자연 응답\n", cmq_measured);
  std::printf("    %6s %9s %12s %14s\n", "V", "Iyy", "2*zeta*wn", "정착 4/(zeta wn)");
  for (double v : {130.0, 170.0, 230.0})
    for (double I : {iyy, mp.mass_kg * std::pow(0.30 * body_len, 2)}) {
      const double q = 0.5 * vd::airDensity(mp, 2000.0) * v * v;
      const double tzw = -q * mp.wing_area_m2 * mac * mac * cmq_measured
                       / (2.0 * v * I);
      std::printf("    %6.0f %9.1f %12.5f %14.1f s\n", v, I, tzw,
                  tzw > 0 ? 8.0 / tzw : INFINITY);
    }
  std::printf("\n  → 경로 단계가 요구한 선회 시간은 8 초대다. 자연 응답이\n"
              "    그보다 훨씬 느리면 자세는 공력이 아니라 **제어 모멘트가**\n"
              "    끌어야 한다. v0 하니스가 선언 제어 모멘트를 직접 명령하는\n"
              "    이유가 이것이고, 그 크기가 곧 요구조건이 된다.\n\n");

  std::printf("  ※ 이 단계는 단주기 근사이며 Cm_q 와 Cm_alphadot 을 분리하지\n"
              "    못한다. 실측 -5.48 도 **전체 형상 M=0.13** 값이라 목표\n"
              "    대역 값이 아니다. 절대 판정으로 쓰지 않는다.\n");
  return 0;
}
