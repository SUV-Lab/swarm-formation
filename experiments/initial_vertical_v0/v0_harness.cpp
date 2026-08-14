// v0 — 초기 수직 구간 end-to-end 하니스. **비생산.**
//
//   정확한 초기 수직 PVA
//     → JSBSim 자세·각속도·질량·CG 전파
//     → 자세 변화 제어 입력과 실제 응답
//     → 인계 7항목 개별 판정
//     → 전부 통과한 상태만 PVA 로 투영
//     → 기존 중기 3DOF 판정기로 유효영역 진입 확인
//
// **결과는 "실행 엔진 검증"이다.** 파라미터가 DECLARED_TEST 이므로
// 실제 비행 가능성의 근거로 쓰면 안 된다.
//
// 파라미터는 매니페스트가 권위다. 기체 XML 은 템플릿이고 로드 **전에**
// 매니페스트 값으로 채운다 — JSBSim 의 inertia/* 속성은 전부 읽기 전용
// 이라(FGMassBalance.cpp:398-409) 실행 중 주입이 불가능하기 때문이다.
// 자리표시자가 하나라도 남으면 멈춘다: 이 파일에 없는 수는 코드에도 없다.
//
// 종료 코드
//   0 인계 도달 + 중기 유효영역 진입
//   1 측정은 됐으나 인계 미도달 또는 유효영역 밖  (정상적인 음성 결과)
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

struct Gate {
  const char *name;
  bool pass;
  double value;
  const char *unit;
  double limit;
};

}  // namespace

int main(int argc, char **argv)
{
  std::string root = ".", pfile = "params_declared_test.txt",
              csv_path = "v0_state.csv", neg, run_dir = ".build/run";
  double dt = 0.002, t_max = 120.0, ic_gamma_deg = 90.0;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto val = [&]() -> const char * { return (i + 1 < argc) ? argv[++i] : ""; };
    if (a == "--root") root = val();
    else if (a == "--params") pfile = val();
    else if (a == "--dt") dt = std::atof(val());
    else if (a == "--tmax") t_max = std::atof(val());
    else if (a == "--csv") csv_path = val();
    else if (a == "--run-dir") run_dir = val();
    else if (a == "--negative") neg = val();
    else if (a == "--ic-gamma-deg") ic_gamma_deg = std::atof(val());
    else { std::fprintf(stderr, "알 수 없는 인자: %s\n", a.c_str()); return 3; }
  }
  const bool vertical = std::fabs(ic_gamma_deg - 90.0) < 1e-12;

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
  std::printf("[매니페스트] %s — DECLARED %d · ESTIMATED %d · UNKNOWN %d\n",
              pfile.c_str(), n_decl, n_est, n_unk);
  std::printf("  결과는 **실행 엔진 검증**이다. DECLARED_TEST 파라미터이므로\n"
              "  실제 비행 가능성의 근거로 쓰지 않는다.\n");
  if (!vertical)
    std::printf("  ** 하니스 양성대조 (초기 gamma %.4f°) — 수직 구간 측정이 "
                "아니다 **\n", ic_gamma_deg);
  if (!neg.empty())
    std::printf("  ** 음성대조: %s **\n", neg.c_str());
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
  const double alt0 = need(P, "alt_init_m", fail);
  const double v0 = need(P, "speed_init_mps", fail);
  const double gam_tgt = need(P, "gamma_target_deg", fail) * kDeg2Rad;
  const double floor_mps = need(P, "cruise_floor_mps", fail);
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

  // ── 템플릿 채우기 ───────────────────────────────────────────
  const std::string tpl_rel = "aircraft/v0_declared_test/v0_declared_test.xml.in";
  std::string xml;
  if (!read_file(root + "/" + tpl_rel, xml)) {
    std::fprintf(stderr, "실행 오류: 템플릿 없음 %s\n", tpl_rel.c_str());
    return 2;
  }
  const double cg_x_in = cg_frac * length / kFt2M * 12.0;
  subst(xml, "@@S_REF_FT2@@", s_ref * kM2ToFt2);
  subst(xml, "@@D_REF_FT@@", d_ref / kFt2M);
  subst(xml, "@@CG_X_IN@@", cg_x_in);
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
  fdm.SetPropertyValue("v0/ctrl/thrust-lbs", thrust * kNToLbf);
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
  // **정확한 초기 PVA**: 속도 벡터를 성분으로 직접 지정한다.
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

  // 초기 상태는 **getter 로** 확인한다 — 명령값이 아니라 실제 상태.
  {
    const auto v = prop->GetVel();
    const double vn = v(1) * kFt2M, ve = v(2) * kFt2M, vd = v(3) * kFt2M;
    const double sp = std::sqrt(vn * vn + ve * ve + vd * vd);
    const double gam = std::asin(-vd / std::max(sp, 1e-9)) * kRad2Deg;
    std::printf("[초기 상태] getter 확인\n");
    std::printf("  속력 %.6f m/s   비행경로각 %.9f°   pitch %.6f°\n",
                sp, gam, prop->GetEuler()(2) * kRad2Deg);
    std::printf("  고도 %.3f m   질량 %.4f kg   CG_x %.4f m   Iyy %.2f kg·m²\n",
                prop->GetAltitudeASL() * kFt2M, mb->GetMass() * kSlugToKg,
                mb->GetXYZcg(1) * kFt2M / 12.0,
                fdm.GetPropertyValue("inertia/iyy-slugs_ft2") * kSlugFt2ToKgM2);
    std::printf("  추력 %.1f N   가용 피치 모멘트 %.1f N·m   공력 계수 전부 0\n\n",
                thrust, m_max);
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
    if (std::fabs(fdm.GetPropertyValue("inertia/iyy-slugs_ft2") * kSlugFt2ToKgM2 - iyy) > 1e-3) {
      std::fprintf(stderr, "계약 위반: Iyy 주입 실패 %.6f != %.6f kg·m²\n",
                   fdm.GetPropertyValue("inertia/iyy-slugs_ft2") * kSlugFt2ToKgM2, iyy);
      return 3;
    }
  }

  const std::string csv_full =
      csv_path.empty() || csv_path[0] == '/' ? csv_path : root + "/" + csv_path;
  std::FILE *csv = std::fopen(csv_full.c_str(), "w");
  if (!csv) { std::fprintf(stderr, "실행 오류: CSV\n"); return 2; }
  std::fprintf(csv, "t,alt_m,speed_mps,gamma_deg,pitch_deg,alpha_deg,beta_deg,"
                    "p_dps,q_dps,r_dps,mass_kg,cg_x_m,ctrl_nm,ctrl_frac\n");

  // ── 자세 제어: gamma 오차 → alpha 명령 → 자세 PD ───────────
  // 공력 계수가 UNKNOWN 이므로 제어면을 모형화하지 않고 가용 피치 모멘트를
  // 직접 명령한다. 사용률을 매 스텝 기록해 권한 부족이 드러나게 한다.
  double dwell = 0.0, t_handoff = -1.0;
  double max_pitch = 0.0, max_rate = 0.0, max_ctrl = 0.0, max_alpha = 0.0;
  double best_gam_err = 1e9, best_sp = 0.0, best_t = 0.0;
  std::vector<Gate> snap, best_snap;
  const long steps = std::llround(t_max / dt);

  for (long k = 0; k <= steps; ++k) {
    const double t = k * dt;
    const auto vned = prop->GetVel();
    const double vn = vned(1) * kFt2M, ve = vned(2) * kFt2M,
                 vd = vned(3) * kFt2M;
    const double sp = std::sqrt(vn * vn + ve * ve + vd * vd);
    const double gam = std::asin(-vd / std::max(sp, 1e-9));
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
        || !std::isfinite(pqr(2)) || !std::isfinite(alt) || !std::isfinite(m_kg)) {
      std::fclose(csv);
      std::fprintf(stderr, "계약 위반: t=%.4f 에서 상태가 비유한 "
                   "(v=%g gamma=%g theta=%g q=%g h=%g m=%g)\n",
                   t, sp, gam, theta, pqr(2), alt, m_kg);
      return 3;
    }

    // 명령: 속도 벡터 아래로 기수를 내려 추력 방향을 꺾는다.
    double a_cmd = -k_gam * (gam - gam_tgt);
    if (a_cmd > a_cmd_max) a_cmd = a_cmd_max;
    if (a_cmd < -a_cmd_max) a_cmd = -a_cmd_max;
    const double theta_cmd = gam + a_cmd;
    double m_cmd = kp_att * (theta_cmd - theta) - kd_att * pqr(2);
    // 음성대조 2: 가용 제어 모멘트를 요구량 아래로 낮춘다.
    const double m_avail = (neg == "weak_control") ? m_max * 1e-3 : m_max;
    if (m_cmd > m_avail) m_cmd = m_avail;
    if (m_cmd < -m_avail) m_cmd = -m_avail;
    const double frac = std::fabs(m_cmd) / m_avail;
    // 음성대조 3: 비유한 값을 상태로 밀어 넣는다. 위의 관문이 잡아야 한다.
    if (neg == "nonfinite" && t >= 1.0) m_cmd = std::nan("");
    fdm.SetPropertyValue("v0/ctrl/pitch-moment-lbft", m_cmd * kNmToLbFt);

    max_pitch = std::max(max_pitch, std::fabs(theta * kRad2Deg));
    max_alpha = std::max(max_alpha, std::fabs(alpha * kRad2Deg));
    const double rate_mag = std::sqrt(pqr(1) * pqr(1) + pqr(2) * pqr(2)
                                      + pqr(3) * pqr(3)) * kRad2Deg;
    max_rate = std::max(max_rate, rate_mag);
    max_ctrl = std::max(max_ctrl, frac);
    const double gerr = std::fabs((gam - gam_tgt) * kRad2Deg);
    const bool is_best = gerr < best_gam_err;
    if (is_best) { best_gam_err = gerr; best_sp = sp; best_t = t; }

    if (k % 50 == 0)
      std::fprintf(csv, "%.4f,%.3f,%.5f,%.6f,%.6f,%.6f,%.6f,"
                        "%.6f,%.6f,%.6f,%.4f,%.5f,%.3f,%.5f\n",
                   t, alt, sp, gam * kRad2Deg, theta * kRad2Deg,
                   alpha * kRad2Deg, beta, pqr(1) * kRad2Deg,
                   pqr(2) * kRad2Deg, pqr(3) * kRad2Deg, m_kg, cgx,
                   m_cmd, frac);

    // ── 인계 7항목 ──────────────────────────────────────────
    snap = {
      {"1 경로각 정렬", gerr <= g_gam, (gam - gam_tgt) * kRad2Deg, "deg", g_gam},
      {"2 받음각", std::fabs(alpha * kRad2Deg) <= g_alp,
       alpha * kRad2Deg, "deg", g_alp},
      {"3 옆미끄럼각", std::fabs(beta) <= g_bet, beta, "deg", g_bet},
      {"4 자세 오차", std::fabs((theta - theta_cmd) * kRad2Deg) <= g_att,
       (theta - theta_cmd) * kRad2Deg, "deg", g_att},
      {"5 몸체 각속도", rate_mag <= g_rate, rate_mag, "deg/s", g_rate},
      {"6 제어 여유", (1.0 - frac) >= g_marg, 1.0 - frac, "frac", g_marg},
      {"7 속력 하한", sp >= floor_mps, sp, "m/s", floor_mps},
    };
    if (is_best) best_snap = snap;
    bool all = true;
    for (const auto &x : snap) all &= x.pass;
    dwell = all ? dwell + dt : 0.0;
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
  std::printf("  gamma 목표 최근접 오차 %.4f° (t=%.3f s), 그때 속력 %.3f m/s\n\n",
              best_gam_err, best_t, best_sp);

  if (t_handoff >= 0.0)
    std::printf("[인계 7항목] t = %.4f s 도달 (연속 %.2f s 동시 충족)\n",
                t_handoff, g_dwell);
  else
    std::printf("[인계 7항목] %.1f s 안에 미도달 — 아래는 **종료 시점** 스냅샷\n",
                t_max);
  for (const auto &x : snap)
    std::printf("  %-14s %s  %13.5f %-5s (한계 %.4f)\n", x.name,
                x.pass ? "통과" : "실패", x.value, x.unit, x.limit);

  if (t_handoff < 0.0) {
    // 종료 시점은 탄도 구간이라 정보가 없다. **경로각이 목표에 가장 가까웠던
    // 순간**의 판정이 어느 항목이 실제로 막았는지를 보여준다.
    std::printf("\n[인계 7항목] 경로각 최근접 순간 t = %.4f s 의 판정\n", best_t);
    for (const auto &x : best_snap)
      std::printf("  %-14s %s  %13.5f %-5s (한계 %.4f)\n", x.name,
                  x.pass ? "통과" : "실패", x.value, x.unit, x.limit);
    std::printf("\n[PVA 투영] 수행하지 않음 — 7항목 전부 통과가 아니다.\n");
    std::printf("[중기 유효영역] 판정 대상 없음 — 투영된 상태가 없다.\n");
    std::fflush(stdout);
    return 1;
  }

  // ── PVA 투영 — **전부 통과한 뒤에만** ──────────────────────
  const auto vned = prop->GetVel();
  // GetUVWdot 은 **몸체 성분의 시간도함수**다 — 수송항 (pqr x uvw) 이 이미
  // 빠져 있으므로(FGAccelerations.cpp:191) 그대로 Tb2l 로 돌리면
  // d(V_ned)/dt 가 아니다. 되돌려 놓고 회전시킨다.
  const auto uvw = prop->GetUVW();
  const auto pqr_h = prop->GetPQR();
  const auto a_ned = prop->GetTb2l() * (accel->GetUVWdot() + pqr_h * uvw);
  const double px = 0.0, py = 0.0, pz = prop->GetAltitudeASL() * kFt2M;
  const double vx = vned(1) * kFt2M, vy = vned(2) * kFt2M, vz = -vned(3) * kFt2M;
  const double ax = a_ned(1) * kFt2M, ay = a_ned(2) * kFt2M,
               az = -a_ned(3) * kFt2M;
  std::printf("\n[PVA 투영] 인계 상태만 투영한다 (ENU, 중기 플래너 규약)\n");
  std::printf("  p = (%.3f, %.3f, %.3f) m\n", px, py, pz);
  std::printf("  v = (%.5f, %.5f, %.5f) m/s   |v| = %.5f\n", vx, vy, vz,
              std::sqrt(vx * vx + vy * vy + vz * vz));
  std::printf("  a = (%.5f, %.5f, %.5f) m/s²\n", ax, ay, az);
  // 투영 검산. 공력이 **정확히 0** 이므로 총 힘은 추력과 중력뿐이고
  // NED 가속도가 닫힌 형태로 알려져 있다. 이 검산은 프레임 대수(수송항,
  // Tb2l 방향, 부호)를 시험하며, 계수가 0 인 덕분에만 가능하다.
  //
  // 다만 플래너의 국소 ENU 는 지구고정 회전 프레임이다. 코리올리·구심
  // 겉보기 가속도는 **a 에 실제로 들어가야 하는 항**이므로 빼지 않고,
  // 기대식이 그것을 모형화하지 않는 만큼만 허용오차로 잡는다.
  {
    const auto xb_ned = prop->GetTb2l() * JSBSim::FGColumnVector3(1.0, 0.0, 0.0);
    const double at = thrust / mass;
    const double rx = a_ned(1) * kFt2M - at * xb_ned(1);
    const double ry = a_ned(2) * kFt2M - at * xb_ned(2);
    const double rz = a_ned(3) * kFt2M - at * xb_ned(3);
    std::printf("  검산 잔차 (추력 제외) = (%.5f, %.5f, %.5f) m/s² "
                "— 중력만 남아야 한다\n", rx, ry, rz);
    constexpr double kOmegaE = 7.292115e-5;
    const double spd = std::sqrt(vx * vx + vy * vy + vz * vz);
    const double earth = 2.0 * kOmegaE * spd
                       + kOmegaE * kOmegaE * (6378137.0 + pz);
    std::printf("  지구자전 겉보기항 상한 %.5f m/s² — 검산 허용오차\n", earth);
    if (std::fabs(rx) > earth || std::fabs(ry) > earth
        || std::fabs(rz - 9.7735) > earth) {
      std::fprintf(stderr, "계약 위반: 가속도 투영 검산 실패 "
                   "(%.6f, %.6f, %.6f)\n", rx, ry, rz);
      return 3;
    }
  }

  // ── 중기 3DOF 판정기 유효영역 ──────────────────────────────
  // 유효영역은 **생산 설정이 정의한다.** 시험 매니페스트로 덮어쓰면
  // 측정하려는 대상 자체가 바뀐다. 그래서 기본값을 그대로 쓴다.
  //
  // 특히 wing_area_m2 를 s_ref_m2 로 대체하지 않는다: 생산 모델의 그것은
  // **양력 기준면적**이고 매니페스트의 s_ref 는 회수 감사의 원통 단면적
  // (pi d^2/4) 이다. 서로 다른 양이다.
  mmp_vehicle_dynamics::Parameters mp;
  if (std::fabs(mp.mass_kg - mass) > 1e-6) {
    std::fprintf(stderr, "계약 위반: 질량 불일치 — 6DOF %.4f kg vs 생산 3DOF "
                 "%.4f kg. 다른 질량의 유효영역과 비교하는 것은 무의미하다.\n",
                 mass, mp.mass_kg);
    return 3;
  }
  std::printf("\n[중기 유효영역] 기존 3DOF 판정기 (생산 코드, 생산 기본값)\n");
  std::printf("  영역 정의: m %.1f kg · S %.3f m² · T [%.0f, %.0f] N · "
              "V [%.1f, %.1f] m/s\n", mp.mass_kg, mp.wing_area_m2,
              mp.thrust_min_n, mp.thrust_max_n, mp.speed_min_mps,
              mp.speed_max_mps);
  std::printf("  CL [%.2f, %.2f] · n_max %.2f · q_max %.0f Pa · "
              "gamma_max %.1f°\n", mp.lift_coefficient_min,
              mp.lift_coefficient_max, mp.load_factor_max,
              mp.dynamic_pressure_max_pa,
              mp.flight_path_angle_max_rad * kRad2Deg);
  const auto ev = mmp_vehicle_dynamics::evaluateInverseDynamics(
      mp, Eigen::Vector3d(px, py, pz), Eigen::Vector3d(vx, vy, vz),
      Eigen::Vector3d(ax, ay, az));
  mmp_vehicle_dynamics::EnvelopeLimit lim;
  const double util = mmp_vehicle_dynamics::envelopeUtilization(mp, ev, &lim);
  std::printf("  valid=%d   속력 %.4f m/s   CL %.5f   하중배수 %.5f\n",
              ev.valid ? 1 : 0, ev.speed_mps, ev.lift_coefficient,
              ev.load_factor);
  std::printf("  포락선 사용률 %.5f   제한 요소 %s\n", util,
              mmp_vehicle_dynamics::envelopeLimitName(lim));
  const bool inside = ev.valid && util <= 1.0 && ev.speed_mps >= floor_mps;
  std::printf("  → 유효영역 진입: %s\n", inside ? "예" : "아니오");

  // 진단 — **판정 기준이 아니다.** v0 6DOF 는 공력이 0 이라 항력이 없고
  // 생산 3DOF 는 CD0 를 싣는다. 같은 PVA 를 항력 없이 다시 재면
  // 사용률 차이가 곧 두 모형의 불일치분이다. 원인 주장을 검사하는 것이지
  // 통과 기준을 낮추는 것이 아니다.
  {
    mmp_vehicle_dynamics::Parameters nd = mp;
    nd.zero_lift_drag_coefficient = 0.0;
    nd.induced_drag_factor = 0.0;
    const auto ev_nd = mmp_vehicle_dynamics::evaluateInverseDynamics(
        nd, Eigen::Vector3d(px, py, pz), Eigen::Vector3d(vx, vy, vz),
        Eigen::Vector3d(ax, ay, az));
    mmp_vehicle_dynamics::EnvelopeLimit lim_nd;
    const double util_nd =
        mmp_vehicle_dynamics::envelopeUtilization(nd, ev_nd, &lim_nd);
    std::printf("\n[진단] 같은 PVA 를 항력 0 으로 재측정 (판정 아님)\n");
    std::printf("  사용률 %.5f → %.5f   제한 요소 %s → %s\n", util, util_nd,
                mmp_vehicle_dynamics::envelopeLimitName(lim),
                mmp_vehicle_dynamics::envelopeLimitName(lim_nd));
    std::printf("  두 모형의 항력 불일치가 사용률 %.5f 를 차지한다.\n",
                util - util_nd);
  }
  std::fflush(stdout);
  return inside ? 0 : 1;
}
