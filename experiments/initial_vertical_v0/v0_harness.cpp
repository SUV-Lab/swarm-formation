// v0 — 초기 수직 구간 end-to-end 하니스. **비생산.**
//
// 인계 판정은 **두 단계의 AND** 이고, 매 시각 순서대로 평가한다.
//
//   sixdof_ready        6DOF 준비 상태 7항목 (필요조건)
//     → 통과한 순간에만 후보 PVA 투영
//   downstream_envelope 중기 모델 포락선 (mmp_vehicle_dynamics::handoffVerdict)
//     → 둘 다 통과하면 dwell 누적, 하나라도 실패하면 dwell 초기화
//   handoff_ready       연속 dwell 충족
//
// 7항목 체류를 먼저 확정하고 나중에 한 번만 투영하면, 포락선 밖 상태를
// 인계 후보로 확정했다가 뒤집게 된다. 7항목은 필요조건이지 충분조건이
// 아니다 — gamma 25° 사례가 7/7 을 통과하고도 추력 사용률 1.271 이었다.
//
// 포락선 판정은 **복제하지 않는다.** 생산 경로(PathManager::pvaEnvelopeProblem)
// 와 같은 mmp_vehicle_dynamics::handoffVerdict 를 부르고, 거절 사유도 그
// 이름 있는 열거형을 그대로 보고한다.
//
// **결과는 "실행 엔진 검증"이다.** 파라미터가 DECLARED_TEST 이므로
// 실제 비행 가능성의 근거로 쓰면 안 된다.
//
// 파라미터는 매니페스트가 권위다. 기체 XML 은 템플릿이고 로드 **전에**
// 매니페스트 값으로 채운다 — JSBSim 의 inertia/* 속성은 전부 읽기 전용
// 이라(FGMassBalance.cpp:398-409) 실행 중 주입이 불가능하기 때문이다.
// 자리표시자가 하나라도 남으면 멈춘다: 이 파일에 없는 수는 코드에도 없다.
//
// 단, **중기 포락선을 정의하는 값은 매니페스트가 아니라 생산 YAML** 이
// 권위다. 시험값으로 덮어쓰면 측정하려는 영역 자체가 바뀐다.
//
// 종료 코드
//   0 인계 도달 (두 단계 AND 가 dwell 충족)
//   1 측정은 됐으나 인계 미도달  (정상적인 음성 결과)
//   2 실행 오류 (모델 로드, 적분 중단, 파일)
//   3 계약 위반 (필수 파라미터 부재/UNKNOWN, 초기 상태 불일치, 비유한 상태)
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include <FGFDMExec.h>
#include <models/FGPropagate.h>
#include <models/FGAuxiliary.h>
#include <models/FGMassBalance.h>
#include <models/FGAccelerations.h>
#include <initialization/FGInitialCondition.h>

#include <mmp_vehicle_dynamics/flight_dynamics.hpp>

namespace vd = mmp_vehicle_dynamics;

namespace {

constexpr double kRad2Deg = 57.29577951308232;
constexpr double kDeg2Rad = 1.0 / kRad2Deg;
constexpr double kFt2M = 0.3048;
constexpr double kSlugFt2ToKgM2 = 1.35581795;
constexpr double kSlugToKg = 14.5939029;
constexpr double kNToLbf = 0.224808943;
constexpr double kNmToLbFt = 0.737562149;
constexpr double kM2ToFt2 = 1.0 / 0.09290304;

struct Param {
  double value = 0.0;
  std::string grade;
  bool known = false;
};

std::map<std::string, Param> load_params(const std::string &path, bool &ok)
{
  std::map<std::string, Param> out;
  std::FILE *f = std::fopen(path.c_str(), "r");
  if (!f) { ok = false; return out; }
  char line[512];
  while (std::fgets(line, sizeof(line), f)) {
    if (line[0] == '#' || line[0] == '\n') continue;
    char name[128], val[64], grade[64];
    if (std::sscanf(line, "%127s %63s %63s", name, val, grade) < 3) continue;
    Param p;
    p.grade = grade;
    p.known = (std::strcmp(val, "UNKNOWN") != 0);
    p.value = p.known ? std::atof(val) : 0.0;
    out[name] = p;
  }
  std::fclose(f);
  ok = true;
  return out;
}

// 필수값이 없거나 UNKNOWN 이면 **이 실험만** 멈춘다.
double need(const std::map<std::string, Param> &p, const char *k, bool &fail)
{
  auto it = p.find(k);
  if (it == p.end()) {
    std::fprintf(stderr, "계약 위반: 필수 파라미터 '%s' 가 매니페스트에 없다\n", k);
    fail = true;
    return 0.0;
  }
  if (!it->second.known) {
    std::fprintf(stderr, "계약 위반: 필수 파라미터 '%s' 가 UNKNOWN — "
                 "임의 값을 넣지 않고 멈춘다\n", k);
    fail = true;
    return 0.0;
  }
  return it->second.value;
}

bool read_file(const std::string &p, std::string &out)
{
  std::FILE *f = std::fopen(p.c_str(), "rb");
  if (!f) return false;
  char buf[4096];
  size_t n;
  while ((n = std::fread(buf, 1, sizeof(buf), f)) > 0) out.append(buf, n);
  std::fclose(f);
  return true;
}

void subst(std::string &s, const std::string &tok, double v)
{
  char num[64];
  std::snprintf(num, sizeof(num), "%.9g", v);
  for (size_t i; (i = s.find(tok)) != std::string::npos; )
    s.replace(i, tok.size(), num);
}

// 생산 YAML 에서 "key: value" 를 그대로 긁는다. 없는 키는 **기본값으로
// 대체하지 않고** 호출부가 멈춘다 — 구조체 기본값(speed_min 120 · margin
// 0.02)과 배포값(122.0 · 0.08)은 순항 하한을 122.4 vs 131.76 으로 가른다.
bool load_production_yaml(const std::string &path,
                          std::map<std::string, double> *out)
{
  std::string text;
  if (!read_file(path, text)) return false;
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
    std::string key = line.substr(0, colon);
    std::string val = line.substr(colon + 1);
    const size_t ks = key.find_first_not_of(" \t");
    if (ks == std::string::npos) continue;
    const size_t ke = key.find_last_not_of(" \t");
    key = key.substr(ks, ke - ks + 1);
    const size_t vs = val.find_first_not_of(" \t\r");
    if (vs == std::string::npos) continue;
    val = val.substr(vs);
    char *end = nullptr;
    const double d = std::strtod(val.c_str(), &end);
    if (end == val.c_str()) continue;
    (*out)[key] = d;
  }
  return true;
}

struct Gate {
  const char *name;
  bool pass;
  double value;
  const char *unit;
  double limit;
};

// 한 시각에 두 단계를 평가한 결과.
struct Combined {
  std::vector<Gate> sixdof;
  bool sixdof_ready = false;
  // 투영과 포락선은 매 시각 계산한다. projected 는 "이 투영이
  // **인계 후보로 관문에 쓰였는가**" 이지 "계산했는가" 가 아니다.
  bool projected = false;
  vd::HandoffVerdict envelope;
  bool envelope_ok = false;
  bool handoff_ready = false;    // 둘의 AND
  double t = 0.0;
  Eigen::Vector3d p{0, 0, 0}, v{0, 0, 0}, a{0, 0, 0};
};

void report(const char *title, const Combined &c)
{
  std::printf("%s  t = %.4f s\n", title, c.t);
  std::printf("  sixdof_ready        %s\n", c.sixdof_ready ? "통과" : "실패");
  for (const auto &x : c.sixdof)
    std::printf("      %-14s %s  %13.5f %-5s (한계 %.4f)\n", x.name,
                x.pass ? "통과" : "실패", x.value, x.unit, x.limit);
  std::printf("  downstream_envelope %s\n",
              c.projected ? (c.envelope_ok ? "통과" : "실패")
                          : "미평가 (후보 아님)");
  if (c.projected) {
    const auto &e = c.envelope;
    std::printf("      유효성   %s\n", e.ok() ? "통과" : "실패");
    std::printf("      사유     %s\n", vd::handoffRejectName(e.reject));
    if (e.evaluation.valid) {
      std::printf("      사용률   %.5f\n", e.utilization);
      std::printf("      제한요소 %s\n", vd::envelopeLimitName(e.limit));
      std::printf("      속력 %.4f m/s · CL %.5f · 하중배수 %.5f · "
                  "요구추력 %.1f N\n", e.evaluation.speed_mps,
                  e.evaluation.lift_coefficient, e.evaluation.load_factor,
                  e.evaluation.thrust_required_n);
    } else {
      // 속력·콘 거절은 역동역학보다 앞에서 끊긴다. 그 경우 사용률과
      // 제한요소는 계산된 적이 없으므로 0/none 을 값처럼 보이면 안 된다.
      std::printf("      사용률·제한요소  해당 없음 — 역동역학 이전에 "
                  "거절됐다\n");
      std::printf("      속력 %.4f m/s · 비행경로각 %.4f°\n",
                  e.speed_mps, e.flight_path_angle_rad * kRad2Deg);
    }
    std::printf("      PVA p (%.3f, %.3f, %.3f) m · v (%.5f, %.5f, %.5f) m/s "
                "· a (%.5f, %.5f, %.5f) m/s²\n",
                c.p.x(), c.p.y(), c.p.z(), c.v.x(), c.v.y(), c.v.z(),
                c.a.x(), c.a.y(), c.a.z());
  } else {
    const auto &e = c.envelope;
    std::printf("      1단계가 실패해 이 투영은 후보가 아니다 — "
                "**관문에 쓰이지 않았다.**\n");
    std::printf("      [진단·관문 아님] 같은 시각에 계산해 둔 중기 판정:\n");
    std::printf("        사유 %s\n", vd::handoffRejectName(e.reject));
    if (e.evaluation.valid)
      std::printf("        사용률 %.5f · 제한요소 %s · 속력 %.4f m/s · "
                  "CL %.5f · 요구추력 %.1f N\n", e.utilization,
                  vd::envelopeLimitName(e.limit), e.evaluation.speed_mps,
                  e.evaluation.lift_coefficient,
                  e.evaluation.thrust_required_n);
    else
      std::printf("        속력 %.4f m/s · 비행경로각 %.4f° "
                  "(역동역학 이전에 거절)\n", e.speed_mps,
                  e.flight_path_angle_rad * kRad2Deg);
  }
  std::printf("  handoff_ready       %s   (둘의 AND — ADR §7-1 항목 7 "
              "연속 체류는 이 AND 위에서 잰다)\n\n",
              c.handoff_ready ? "통과" : "실패");
}

}  // namespace

int main(int argc, char **argv)
{
  std::string root = ".", pfile = "params_declared_test.txt",
              csv_path = "v0_state.csv", neg, case_name = "vertical",
              run_dir = ".build/run",
              yaml_rel = "../../src/path_manager/config/optimizer_params.yaml";
  double dt = 0.002, t_max = 120.0, ic_gamma_deg = 90.0, ic_alt_m = -1.0,
         ic_speed_mps = -1.0, thrust_override_n = -1.0,
         gamma_target_override_deg = 1e9, nonfinite_at_s = 0.3;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto val = [&]() -> const char * { return (i + 1 < argc) ? argv[++i] : ""; };
    if (a == "--root") root = val();
    else if (a == "--params") pfile = val();
    else if (a == "--yaml") yaml_rel = val();
    else if (a == "--dt") dt = std::atof(val());
    else if (a == "--tmax") t_max = std::atof(val());
    else if (a == "--csv") csv_path = val();
    else if (a == "--run-dir") run_dir = val();
    else if (a == "--negative") neg = val();
    else if (a == "--case") case_name = val();
    else if (a == "--ic-gamma-deg") ic_gamma_deg = std::atof(val());
    else if (a == "--ic-alt-m") ic_alt_m = std::atof(val());
    else if (a == "--ic-speed-mps") ic_speed_mps = std::atof(val());
    else if (a == "--thrust-n") thrust_override_n = std::atof(val());
    else if (a == "--nonfinite-at-s") nonfinite_at_s = std::atof(val());
    else if (a == "--gamma-target-deg")
      gamma_target_override_deg = std::atof(val());
    else { std::fprintf(stderr, "알 수 없는 인자: %s\n", a.c_str()); return 3; }
  }

  bool ok = false;
  auto P = load_params(root + "/" + pfile, ok);
  if (!ok) {
    std::fprintf(stderr, "계약 위반: 매니페스트를 열 수 없다: %s\n", pfile.c_str());
    return 3;
  }

  int n_decl = 0, n_est = 0, n_unk = 0;
  for (const auto &kv : P) {
    if (kv.second.grade == "UNKNOWN") ++n_unk;
    else if (kv.second.grade == "ESTIMATED") ++n_est;
    else ++n_decl;
  }
  std::printf("════ 사례: %s ════\n", case_name.c_str());
  std::printf("[매니페스트] %s — DECLARED %d · ESTIMATED %d · UNKNOWN %d\n",
              pfile.c_str(), n_decl, n_est, n_unk);
  std::printf("  결과는 **실행 엔진 검증**이다. DECLARED_TEST 파라미터이므로\n"
              "  실제 비행 가능성의 근거로 쓰지 않는다.\n");
  if (case_name != "vertical")
    std::printf("  ** 초기 수직 구간 측정이 아니다 **\n");
  if (!neg.empty()) {
    std::printf("  ** 음성대조: %s **\n", neg.c_str());
    if (neg == "nonfinite")
      std::printf("     비유한 값 주입 시각 %.4f s — **인계 가능 시각보다 앞**이어야\n     한다. 뒤면 관문이 먼저 끝나 주입이 일어나지 않는다.\n", nonfinite_at_s);
  }
  std::printf("\n");

  bool fail = false;
  const double mass = need(P, "mass_kg", fail);
  const double ixx = need(P, "ixx_kgm2", fail);
  const double iyy = need(P, "iyy_kgm2", fail);
  const double izz = need(P, "izz_kgm2", fail);
  const double d_ref = need(P, "d_ref_m", fail);
  const double s_ref = need(P, "s_ref_m2", fail);
  const double length = need(P, "length_m", fail);
  const double cg_frac = need(P, "cg_frac", fail);
  const double m_max = need(P, "pitch_moment_max_nm", fail);
  const double thrust = need(P, "thrust_max_n", fail);
  const double a_cmd_max = need(P, "ctrl_alpha_cmd_max_deg", fail) * kDeg2Rad;
  const double k_gam = need(P, "ctrl_k_gamma", fail);
  const double kp_att = need(P, "ctrl_kp_att_nm_per_rad", fail);
  const double kd_att = need(P, "ctrl_kd_att_nms_per_rad", fail);
  const double alt0_p = need(P, "alt_init_m", fail);
  const double v0_p = need(P, "speed_init_mps", fail);
  const double gam_tgt = need(P, "gamma_target_deg", fail) * kDeg2Rad;
  const double g_gam = need(P, "gate_gamma_tol_deg", fail);
  const double g_alp = need(P, "gate_alpha_max_deg", fail);
  const double g_bet = need(P, "gate_beta_max_deg", fail);
  const double g_att = need(P, "gate_att_err_deg", fail);
  const double g_rate = need(P, "gate_rate_max_dps", fail);
  const double g_marg = need(P, "gate_ctrl_margin_frac", fail);
  const double g_dwell = need(P, "gate_dwell_s", fail);
  // 음성대조 1: UNKNOWN 인 값을 필수로 요구하면 멈춰야 한다.
  if (neg == "missing_param") need(P, "cm_alpha", fail);
  if (fail) return 3;
  const double alt0 = ic_alt_m >= 0.0 ? ic_alt_m : alt0_p;
  const double v0 = ic_speed_mps >= 0.0 ? ic_speed_mps : v0_p;
  // 사례 파라미터는 매니페스트 값을 덮어쓴다. 덮어쓴 것은 **반드시 출력**
  // 한다 — 보고에 보이지 않는 수를 실행에 넣지 않는다.
  const double thrust_used =
      thrust_override_n >= 0.0 ? thrust_override_n : thrust;
  const double gam_tgt_used =
      gamma_target_override_deg < 1e8
          ? gamma_target_override_deg * kDeg2Rad : gam_tgt;
  if (thrust_override_n >= 0.0 || gamma_target_override_deg < 1e8) {
    std::printf("[사례 파라미터] 매니페스트를 덮어쓴 값\n");
    if (thrust_override_n >= 0.0)
      std::printf("  추력 %.1f N  (매니페스트 %.1f N)\n", thrust_used, thrust);
    if (gamma_target_override_deg < 1e8)
      std::printf("  목표 비행경로각 %.4f°  (매니페스트 %.4f°)\n",
                  gam_tgt_used * kRad2Deg, gam_tgt * kRad2Deg);
    std::printf("\n");
  }

  // ── 중기 포락선: 생산 YAML 이 권위 ──────────────────────────
  std::map<std::string, double> Y;
  if (!load_production_yaml(root + "/" + yaml_rel, &Y)) {
    std::fprintf(stderr, "계약 위반: 생산 YAML 을 열 수 없다: %s\n"
                 "  포락선은 생산 설정이 정의한다. 기본값으로 대체하지 않는다.\n",
                 yaml_rel.c_str());
    return 3;
  }
  vd::Parameters mp;
  double um_xy = 0.0, max_vel_u = 0.0, bank_deg = 0.0, fpa_deg = 0.0;
  const struct { const char *key; double *dst; } pulls[] = {
    {"optimization/dynamics_mass_kg", &mp.mass_kg},
    {"optimization/dynamics_wing_area_m2", &mp.wing_area_m2},
    {"optimization/dynamics_g", &mp.gravity_mps2},
    {"optimization/dynamics_rho0_kgpm3", &mp.sea_level_density_kgpm3},
    {"optimization/dynamics_density_scale_height_m", &mp.density_scale_height_m},
    {"optimization/dynamics_altitude_reference_m", &mp.altitude_reference_m},
    {"optimization/dynamics_cd0", &mp.zero_lift_drag_coefficient},
    {"optimization/dynamics_induced_drag_factor", &mp.induced_drag_factor},
    {"optimization/dynamics_cl_min", &mp.lift_coefficient_min},
    {"optimization/dynamics_cl_max", &mp.lift_coefficient_max},
    {"optimization/dynamics_load_factor_max", &mp.load_factor_max},
    {"optimization/dynamics_thrust_min_n", &mp.thrust_min_n},
    {"optimization/dynamics_thrust_max_n", &mp.thrust_max_n},
    {"optimization/dynamics_speed_min_mps", &mp.speed_min_mps},
    {"optimization/dynamics_speed_max_mps", &mp.speed_max_mps},
    {"optimization/dynamics_activation_speed_mps",
     &mp.model_activation_speed_mps},
    {"optimization/dynamics_dynamic_pressure_max_pa",
     &mp.dynamic_pressure_max_pa},
    {"optimization/dynamics_margin", &mp.constraint_margin},
    {"optimization/dynamics_unit_xy_m", &um_xy},
    {"optimization/max_vel", &max_vel_u},
    {"optimization/dynamics_bank_max_deg", &bank_deg},
    {"optimization/dynamics_flight_path_max_deg", &fpa_deg},
  };
  std::vector<std::string> missing;
  for (const auto &p : pulls) {
    auto it = Y.find(p.key);
    if (it == Y.end()) missing.push_back(p.key);
    else *p.dst = it->second;
  }
  if (!missing.empty()) {
    std::fprintf(stderr, "계약 위반: 생산 YAML 에 없는 키 %zu 개 — 기본값으로 "
                 "대체하지 않는다:\n", missing.size());
    for (const auto &k : missing) std::fprintf(stderr, "  %s\n", k.c_str());
    return 3;
  }
  mp.bank_angle_max_rad = bank_deg * kDeg2Rad;
  mp.flight_path_angle_max_rad = fpa_deg * kDeg2Rad;
  if (!vd::parametersAreValid(mp)) {
    std::fprintf(stderr, "계약 위반: 생산 YAML 로 만든 Parameters 가 "
                 "유효하지 않다\n");
    return 3;
  }
  // 생산 PathManager::effectiveHandoffMaxMps() 와 같은 규칙:
  //   min(모델 상한, 명시 인계 상한 없으면 계획 프레임 상한)
  const double floor_mps = vd::marginBackedSpeedFloorMps(mp);
  const double plan_cap = max_vel_u * um_xy;
  const double vmax_mps =
      std::min(mp.speed_max_mps, plan_cap > 1e-9 ? plan_cap : mp.speed_max_mps);
  std::printf("[중기 포락선] 생산 YAML 이 정의한다 — %s\n", yaml_rel.c_str());
  std::printf("  m %.1f kg · S %.3f m² · T [%.0f, %.0f] N · CL_max %.2f · "
              "n_max %.2f · q_max %.0f Pa\n", mp.mass_kg, mp.wing_area_m2,
              mp.thrust_min_n, mp.thrust_max_n, mp.lift_coefficient_max,
              mp.load_factor_max, mp.dynamic_pressure_max_pa);
  std::printf("  V [%.1f, %.1f] m/s · margin %.3f → 순항 하한 %.5f m/s · "
              "인계 상한 %.1f m/s\n", mp.speed_min_mps, mp.speed_max_mps,
              mp.constraint_margin, floor_mps, vmax_mps);
  std::printf("  bank_max %.1f° · gamma_max %.1f° · 모델 작동 속력 %.1f m/s\n",
              bank_deg, fpa_deg, mp.model_activation_speed_mps);
  // 읽는 것은 **배포 기본 설정**이다. 실행 시 planning/handoff_max_vel_mps
  // 같은 재정의가 걸리면 생산의 인계 상한은 달라진다 — 여기 결과의 범위는
  // 그 재정의가 없는 기본 설정 기준이다.
  std::printf("  ※ 배포 기본 설정 기준. 실행 시 planning/handoff_max_vel_mps "
              "재정의는 재현하지 않는다.\n\n");
  if (std::fabs(mp.mass_kg - mass) > 1e-6) {
    std::fprintf(stderr, "계약 위반: 질량 불일치 — 6DOF %.4f kg vs 생산 3DOF "
                 "%.4f kg. 다른 질량의 포락선과 비교하는 것은 무의미하다.\n",
                 mass, mp.mass_kg);
    return 3;
  }

  // ── 템플릿 채우기 ───────────────────────────────────────────
  const std::string tpl_rel = "aircraft/v0_declared_test/v0_declared_test.xml.in";
  std::string xml;
  if (!read_file(root + "/" + tpl_rel, xml)) {
    std::fprintf(stderr, "실행 오류: 템플릿 없음 %s\n", tpl_rel.c_str());
    return 2;
  }
  subst(xml, "@@S_REF_FT2@@", s_ref * kM2ToFt2);
  subst(xml, "@@D_REF_FT@@", d_ref / kFt2M);
  subst(xml, "@@CG_X_IN@@", cg_frac * length / kFt2M * 12.0);
  subst(xml, "@@IXX_SLUGFT2@@", ixx / kSlugFt2ToKgM2);
  subst(xml, "@@IYY_SLUGFT2@@", iyy / kSlugFt2ToKgM2);
  subst(xml, "@@IZZ_SLUGFT2@@", izz / kSlugFt2ToKgM2);
  subst(xml, "@@WEIGHT_LBS@@", mass / kSlugToKg * 32.17404856);
  if (xml.find("@@") != std::string::npos) {
    std::fprintf(stderr, "계약 위반: 템플릿에 채워지지 않은 자리가 남았다\n");
    return 3;
  }
  const std::string ac_dir = root + "/" + run_dir + "/aircraft/v0_declared_test";
  if (std::system(("mkdir -p '" + ac_dir + "'").c_str()) != 0) {
    std::fprintf(stderr, "실행 오류: 디렉터리 %s\n", ac_dir.c_str());
    return 2;
  }
  {
    std::FILE *f = std::fopen((ac_dir + "/v0_declared_test.xml").c_str(), "w");
    if (!f) { std::fprintf(stderr, "실행 오류: XML 쓰기\n"); return 2; }
    std::fwrite(xml.data(), 1, xml.size(), f);
    std::fclose(f);
  }

  // ── JSBSim ─────────────────────────────────────────────────
  JSBSim::FGFDMExec fdm;
  fdm.SetRootDir(SGPath(root + "/" + run_dir));
  fdm.SetAircraftPath(SGPath("aircraft"));
  fdm.SetDebugLevel(0);
  if (!fdm.LoadModel("v0_declared_test")) {
    std::fprintf(stderr, "실행 오류: LoadModel\n");
    return 2;
  }
  fdm.SetPropertyValue("v0/ctrl/pitch-moment-lbft", 0.0);
  fdm.SetPropertyValue("v0/ctrl/thrust-lbs", thrust_used * kNToLbf);
  fdm.Setdt(dt);
  fdm.SetPropertyValue("simulation/integrator/rate/rotational", 3);      // AB2
  fdm.SetPropertyValue("simulation/integrator/position/rotational", 7);  // Buss2

  auto ic = fdm.GetIC();
  ic->SetLatitudeDegIC(0.0);
  ic->SetLongitudeDegIC(0.0);
  ic->SetAltitudeASLFtIC(alt0 / kFt2M);
  ic->SetPhiDegIC(0.0);
  ic->SetThetaDegIC(ic_gamma_deg);   // 기수를 속도 벡터에 정렬 → alpha 0
  ic->SetPsiDegIC(0.0);
  const double g_rad = ic_gamma_deg * kDeg2Rad;
  ic->SetVNorthFpsIC(v0 * std::cos(g_rad) / kFt2M);
  ic->SetVEastFpsIC(0.0);
  ic->SetVDownFpsIC(-v0 * std::sin(g_rad) / kFt2M);
  ic->SetPRadpsIC(0.0);
  ic->SetQRadpsIC(0.0);
  ic->SetRRadpsIC(0.0);
  if (!fdm.RunIC()) { std::fprintf(stderr, "실행 오류: RunIC\n"); return 2; }

  auto prop = fdm.GetPropagate();
  auto aux = fdm.GetAuxiliary();
  auto mb = fdm.GetMassBalance();
  auto accel = fdm.GetAccelerations();

  {
    const auto v = prop->GetVel();
    const double vn = v(1) * kFt2M, ve = v(2) * kFt2M, vd_ = v(3) * kFt2M;
    const double sp = std::sqrt(vn * vn + ve * ve + vd_ * vd_);
    const double gam = std::asin(-vd_ / std::max(sp, 1e-9)) * kRad2Deg;
    const double iyy_got =
        fdm.GetPropertyValue("inertia/iyy-slugs_ft2") * kSlugFt2ToKgM2;
    std::printf("[초기 상태] getter 확인\n");
    std::printf("  속력 %.6f m/s   비행경로각 %.9f°   pitch %.6f°\n",
                sp, gam, prop->GetEuler()(2) * kRad2Deg);
    std::printf("  고도 %.3f m   질량 %.4f kg   CG_x %.4f m   Iyy %.2f kg·m²\n",
                prop->GetAltitudeASL() * kFt2M, mb->GetMass() * kSlugToKg,
                mb->GetXYZcg(1) * kFt2M / 12.0, iyy_got);
    std::printf("  추력 %.1f N   가용 피치 모멘트 %.1f N·m   공력 계수 전부 0\n\n",
                thrust_used, m_max);
    if (std::fabs(gam - ic_gamma_deg) > 1e-6) {
      std::fprintf(stderr, "계약 위반: 초기 비행경로각 %.9f° != %.9f°\n",
                   gam, ic_gamma_deg);
      return 3;
    }
    if (std::fabs(mb->GetMass() * kSlugToKg - mass) > 1e-3) {
      std::fprintf(stderr, "계약 위반: 질량 주입 실패 %.6f != %.6f kg\n",
                   mb->GetMass() * kSlugToKg, mass);
      return 3;
    }
    if (std::fabs(iyy_got - iyy) > 1e-3) {
      std::fprintf(stderr, "계약 위반: Iyy 주입 실패 %.6f != %.6f kg·m²\n",
                   iyy_got, iyy);
      return 3;
    }
  }

  const std::string csv_full =
      csv_path.empty() || csv_path[0] == '/' ? csv_path : root + "/" + csv_path;
  std::FILE *csv = std::fopen(csv_full.c_str(), "w");
  if (!csv) {
    std::fprintf(stderr, "실행 오류: CSV %s\n", csv_full.c_str());
    return 2;
  }
  std::fprintf(csv, "t,alt_m,speed_mps,gamma_deg,pitch_deg,alpha_deg,beta_deg,"
                    "p_dps,q_dps,r_dps,mass_kg,cg_x_m,ctrl_nm,ctrl_frac,"
                    "sixdof_ready,env_ok,env_util,env_reject,handoff_ready\n");

  double dwell = 0.0, t_handoff = -1.0;
  double max_pitch = 0.0, max_rate = 0.0, max_ctrl = 0.0, max_alpha = 0.0;
  double best_gam_err = 1e9, best_sp = 0.0, best_t = 0.0;
  long n_sixdof = 0, n_both = 0;
  Combined snap, best, first_sixdof;
  bool have_first = false;
  const long steps = std::llround(t_max / dt);

  for (long k = 0; k <= steps; ++k) {
    const double t = k * dt;
    const auto vned = prop->GetVel();
    const double vn = vned(1) * kFt2M, ve = vned(2) * kFt2M,
                 vdn = vned(3) * kFt2M;
    const double sp = std::sqrt(vn * vn + ve * ve + vdn * vdn);
    const double gam = std::asin(-vdn / std::max(sp, 1e-9));
    const auto eul = prop->GetEuler();
    const auto pqr = prop->GetPQR();
    const double theta = eul(2);
    const double alpha = aux->Getalpha();
    const double beta = aux->Getbeta() * kRad2Deg;
    const double m_kg = mb->GetMass() * kSlugToKg;
    const double cgx = mb->GetXYZcg(1) * kFt2M / 12.0;
    const double alt = prop->GetAltitudeASL() * kFt2M;

    // 음성대조 3 은 이 관문이 잡아야 한다.
    if (!std::isfinite(sp) || !std::isfinite(gam) || !std::isfinite(theta)
        || !std::isfinite(pqr(2)) || !std::isfinite(alt)
        || !std::isfinite(m_kg)) {
      std::fclose(csv);
      std::fprintf(stderr, "계약 위반: t=%.4f 에서 상태가 비유한 "
                   "(v=%g gamma=%g theta=%g q=%g h=%g m=%g)\n",
                   t, sp, gam, theta, pqr(2), alt, m_kg);
      return 3;
    }

    double a_cmd = -k_gam * (gam - gam_tgt_used);
    if (a_cmd > a_cmd_max) a_cmd = a_cmd_max;
    if (a_cmd < -a_cmd_max) a_cmd = -a_cmd_max;
    const double theta_cmd = gam + a_cmd;
    double m_cmd = kp_att * (theta_cmd - theta) - kd_att * pqr(2);
    // 음성대조 2: 가용 제어 모멘트를 요구량 아래로 낮춘다.
    const double m_avail = (neg == "weak_control") ? m_max * 1e-3 : m_max;
    if (m_cmd > m_avail) m_cmd = m_avail;
    if (m_cmd < -m_avail) m_cmd = -m_avail;
    const double frac = std::fabs(m_cmd) / m_avail;
    if (neg == "nonfinite" && t >= nonfinite_at_s) m_cmd = std::nan("");
    fdm.SetPropertyValue("v0/ctrl/pitch-moment-lbft", m_cmd * kNmToLbFt);

    max_pitch = std::max(max_pitch, std::fabs(theta * kRad2Deg));
    max_alpha = std::max(max_alpha, std::fabs(alpha * kRad2Deg));
    const double rate_mag = std::sqrt(pqr(1) * pqr(1) + pqr(2) * pqr(2)
                                      + pqr(3) * pqr(3)) * kRad2Deg;
    max_rate = std::max(max_rate, rate_mag);
    max_ctrl = std::max(max_ctrl, frac);
    const double gerr = std::fabs((gam - gam_tgt_used) * kRad2Deg);

    // ── 1단계: 6DOF 준비 상태 7항목 ─────────────────────────
    // 속력 하한은 **여기서 판정하지 않는다.** 그것은 중기 모델이 정의하는
    // 양이고 2단계가 생산 판정으로 본다. 두 곳에서 검사하면 드리프트한다.
    Combined c;
    c.t = t;
    // ADR-0003 §7-1 의 항목 그대로다. 1-6 이 순간 조건이고 **7 은 연속
    // 체류시간** — 속력 하한은 여기 없다. 그것은 중기 모델이 정의하는
    // 양이고 2단계가 생산 판정으로 본다.
    c.sixdof = {
      {"1 경로각·접선", gerr <= g_gam, (gam - gam_tgt_used) * kRad2Deg,
       "deg", g_gam},
      {"2 받음각", std::fabs(alpha * kRad2Deg) <= g_alp,
       alpha * kRad2Deg, "deg", g_alp},
      {"2 옆미끄럼각", std::fabs(beta) <= g_bet, beta, "deg", g_bet},
      {"3 자세 오차", std::fabs((theta - theta_cmd) * kRad2Deg) <= g_att,
       (theta - theta_cmd) * kRad2Deg, "deg", g_att},
      {"4 몸체 각속도", rate_mag <= g_rate, rate_mag, "deg/s", g_rate},
      {"5 제어 여유", (1.0 - frac) >= g_marg, 1.0 - frac, "frac", g_marg},
      {"6 질량·CG", std::isfinite(m_kg) && std::isfinite(cgx)
       && m_kg > 0.0, m_kg, "kg", 0.0},
    };
    c.sixdof_ready = true;
    for (const auto &x : c.sixdof) c.sixdof_ready &= x.pass;

    // ── 2단계: 후보 PVA 투영 → 중기 포락선 ──────────────────
    // 계산은 매 시각 하되, **후보로 인정되는 것은 1단계를 통과한 순간의
    // 투영뿐이다.** 나머지는 보고용 진단이고 dwell 에 들어가지 않는다.
    // GetUVWdot 은 몸체 성분의 시간도함수라 수송항이 이미 빠져 있다
    // (FGAccelerations.cpp:191). 되돌려 놓고 회전시켜야 d(V_ned)/dt 다.
    const auto a_ned = prop->GetTb2l()
        * (accel->GetUVWdot() + prop->GetPQR() * prop->GetUVW());
    c.p = Eigen::Vector3d(0.0, 0.0, alt);
    c.v = Eigen::Vector3d(vn, ve, -vdn);
    c.a = Eigen::Vector3d(a_ned(1) * kFt2M, a_ned(2) * kFt2M,
                          -a_ned(3) * kFt2M);
    // 생산과 **같은 판정**. 복제하지 않는다.
    c.envelope = vd::handoffVerdict(mp, c.p, c.v, c.a, floor_mps, vmax_mps);
    // **관문에 쓰이는 것은 1단계를 통과한 순간의 투영뿐이다.** 그 밖의
    // 시각에도 계산해 두는 것은 보고용 진단이며, dwell 에 들어가지 않는다.
    if (c.sixdof_ready) {
      ++n_sixdof;
      c.projected = true;
      c.envelope_ok = c.envelope.ok();
    }
    c.handoff_ready = c.sixdof_ready && c.envelope_ok;
    if (c.handoff_ready) ++n_both;
    if (c.sixdof_ready && !have_first) { first_sixdof = c; have_first = true; }
    snap = c;
    if (gerr < best_gam_err) {
      best_gam_err = gerr; best_sp = sp; best_t = t; best = c;
    }

    if (k % 50 == 0)
      std::fprintf(csv, "%.4f,%.3f,%.5f,%.6f,%.6f,%.6f,%.6f,"
                        "%.6f,%.6f,%.6f,%.4f,%.5f,%.3f,%.5f,%d,%d,%.5f,%s,%d\n",
                   t, alt, sp, gam * kRad2Deg, theta * kRad2Deg,
                   alpha * kRad2Deg, beta, pqr(1) * kRad2Deg,
                   pqr(2) * kRad2Deg, pqr(3) * kRad2Deg, m_kg, cgx,
                   m_cmd, frac, c.sixdof_ready ? 1 : 0, c.envelope_ok ? 1 : 0,
                   c.envelope.utilization,
                   c.projected ? vd::handoffRejectName(c.envelope.reject)
                               : "not-projected",
                   c.handoff_ready ? 1 : 0);

    // ── 3단계: 두 단계의 AND 만 dwell 을 누적한다 ───────────
    dwell = c.handoff_ready ? dwell + dt : 0.0;
    if (dwell >= g_dwell) { t_handoff = t; break; }

    if (k < steps && !fdm.Run()) {
      std::fclose(csv);
      std::fprintf(stderr, "실행 오류: Run() 중단 t=%.4f\n", t);
      return 2;
    }
  }
  std::fclose(csv);

  std::printf("[제어 응답] 최대 pitch %.4f°   최대 받음각 %.4f°   "
              "최대 각속도 %.5f °/s   최대 제어 사용률 %.2f%%\n",
              max_pitch, max_alpha, max_rate, max_ctrl * 100.0);
  std::printf("  gamma 목표 최근접 오차 %.4f° (t=%.3f s), 그때 속력 %.3f m/s\n",
              best_gam_err, best_t, best_sp);
  std::printf("  sixdof_ready 성립 스텝 %ld · 두 단계 동시 성립 스텝 %ld\n\n",
              n_sixdof, n_both);

  if (t_handoff >= 0.0) {
    std::printf("═══ 인계 도달 — t = %.4f s (연속 %.2f s 동안 두 단계 동시 "
                "충족) ═══\n\n", t_handoff, g_dwell);
    report("[인계 시점]", snap);
    std::fflush(stdout);
    return 0;
  }

  std::printf("═══ 인계 미도달 (%.1f s) ═══\n\n", t_max);
  report("[경로각 최근접 순간]", best);
  if (have_first && first_sixdof.t != best.t)
    report("[sixdof_ready 가 처음 성립한 순간]", first_sixdof);
  else if (!have_first)
    std::printf("sixdof_ready 는 한 번도 성립하지 않았다 — **인계 후보로 "
                "쓰인 투영이 없었다.**\n"
                "  (투영과 포락선 판정 자체는 매 시각 계산해 두었다. 진단용이며 "
                "관문에는 들어가지 않는다.)\n\n");
  std::fflush(stdout);
  return 1;
}
