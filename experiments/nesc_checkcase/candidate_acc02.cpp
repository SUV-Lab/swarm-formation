// NESC 대기권 사례 2 — 후보 생성기. 비생산.
//
// 실행 순서 (계약)
//   1. RunIC()
//   2. **JSBSim getter 로 실제 상태를 읽는다** — 기대값 상수가 아니라
//   3. 그 실제 상태로 초기조건 계약 검사. 통과해야만 다음으로
//   4. 전 구간 외부 모멘트·공력 힘/모멘트가 0인지 확인
//   5. 고정 스텝 30 s
//   6. GetPQRi() · 자세 · 위경도를 CSV 로
//
// t=0 행도 상수를 다시 쓰지 않고 **getter 값**으로 만든다. 그래야 배관
// 오류(입력은 맞았는데 기록이 틀림, 또는 그 반대)가 드러난다.
//
// 각속도 프레임: JSBSim 은 vPQRi = vPQR + Ti2b * omegaPlanet 로 둘을
// 구분한다. NESC 출력의 10/20/30 deg/s 는 **관성계**(PQRi) 값이므로
// 기록도 GetPQRi() 로 한다. 자세한 근거는 ic_contract.py 머리글.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include <FGFDMExec.h>
#include <models/FGPropagate.h>
#include <models/FGAuxiliary.h>
#include <models/FGAircraft.h>
#include <models/FGMassBalance.h>
#include <initialization/FGInitialCondition.h>

namespace {

constexpr double kOmegaEarthRadS = 7.292115e-5;
constexpr double kRad2Deg = 57.29577951308232;
constexpr double kDeg2Rad = 1.0 / kRad2Deg;

struct Tol {
  double rate = 1e-6, angle = 1e-9, geo = 1e-9, alt = 1e-6, vel = 1e-9;
};

bool near_(const char *name, double got, double want, double tol, int &bad)
{
  if (!std::isfinite(got)) {
    std::fprintf(stderr, "FAIL: %s 비유한값\n", name);
    ++bad;
    return false;
  }
  if (std::fabs(got - want) > tol) {
    std::fprintf(stderr, "FAIL: %s = %.12g, 기대 %.12g (허용 %g, 차 %.3e)\n",
                 name, got, want, tol, std::fabs(got - want));
    ++bad;
    return false;
  }
  return true;
}

}  // namespace

int main(int argc, char **argv)
{
  const std::string root = argc > 1 ? argv[1] : ".";
  const double dt = argc > 2 ? std::atof(argv[2]) : 0.001;
  const double duration = argc > 3 ? std::atof(argv[3]) : 30.0;
  const double out_dt = argc > 4 ? std::atof(argv[4]) : 0.1;
  const std::string csv_path = argc > 5 ? argv[5] : "candidate_acc02.csv";
  // CSV 는 파일로. stdout 에는 JSBSim 배너가 섞이므로 섞으면 안 된다.
  std::FILE *csv = std::fopen(csv_path.c_str(), "w");
  if (!csv) { std::fprintf(stderr, "FAIL: %s 열기 실패\n", csv_path.c_str()); return 2; }

  JSBSim::FGFDMExec fdm;
  fdm.SetRootDir(SGPath(root));
  fdm.SetAircraftPath(SGPath("aircraft"));
  fdm.SetDebugLevel(0);
  if (!fdm.LoadModel("nesc_brick")) {
    std::fprintf(stderr, "FAIL: LoadModel(nesc_brick)\n");
    return 2;
  }
  fdm.Setdt(dt);

  // ── 초기조건 입력 ───────────────────────────────────────────────
  // NESC: 적도·본초자오선, 30,000 ft, 자세 0/0/0, local velocity 0,
  // **관성계** 각속도 10/20/30 deg/s.
  // JSBSim 의 일반 p/q/r 은 ECEF 기준이므로 자전분을 뺀 값을 넣는다.
  auto ic = fdm.GetIC();
  ic->SetLatitudeDegIC(0.0);
  ic->SetLongitudeDegIC(0.0);
  ic->SetAltitudeASLFtIC(30000.0);
  ic->SetPhiDegIC(0.0);
  ic->SetThetaDegIC(0.0);
  ic->SetPsiDegIC(0.0);
  ic->SetVNorthFpsIC(0.0);
  ic->SetVEastFpsIC(0.0);
  ic->SetVDownFpsIC(0.0);
  const double omega_deg_s = kOmegaEarthRadS * kRad2Deg;
  ic->SetPRadpsIC((10.0 - omega_deg_s) * kDeg2Rad);
  ic->SetQRadpsIC(20.0 * kDeg2Rad);
  ic->SetRRadpsIC(30.0 * kDeg2Rad);

  if (!fdm.RunIC()) {
    std::fprintf(stderr, "FAIL: RunIC\n");
    return 2;
  }

  // ── 실제 상태를 getter 로 읽는다 ────────────────────────────────
  auto prop = fdm.GetPropagate();
  auto aux = fdm.GetAuxiliary(); (void)aux;
  auto air = fdm.GetAircraft();

  const auto pqri = prop->GetPQRi();
  const auto pqr = prop->GetPQR();
  const auto euler = prop->GetEuler();
  const auto vned = prop->GetVel();

  Tol tol;
  int bad = 0;
  std::fprintf(stderr, "[IC] getter 로 읽은 실제 상태 (상수 재사용 없음)\n");
  std::fprintf(stderr, "  pqri  %.10f %.10f %.10f deg/s\n",
              pqri(1) * kRad2Deg, pqri(2) * kRad2Deg, pqri(3) * kRad2Deg);
  std::fprintf(stderr, "  pqr   %.10f %.10f %.10f deg/s\n",
              pqr(1) * kRad2Deg, pqr(2) * kRad2Deg, pqr(3) * kRad2Deg);

  near_("pqri_p (관성계)", pqri(1) * kRad2Deg, 10.0, tol.rate, bad);
  near_("pqri_q (관성계)", pqri(2) * kRad2Deg, 20.0, tol.rate, bad);
  near_("pqri_r (관성계)", pqri(3) * kRad2Deg, 30.0, tol.rate, bad);
  near_("pqr_p (ECEF)", pqr(1) * kRad2Deg, 10.0 - omega_deg_s, tol.rate, bad);
  near_("pqr_q (ECEF)", pqr(2) * kRad2Deg, 20.0, tol.rate, bad);
  near_("pqr_r (ECEF)", pqr(3) * kRad2Deg, 30.0, tol.rate, bad);
  near_("자세 roll", euler(1) * kRad2Deg, 0.0, tol.angle, bad);
  near_("자세 pitch", euler(2) * kRad2Deg, 0.0, tol.angle, bad);
  near_("자세 yaw", euler(3) * kRad2Deg, 0.0, tol.angle, bad);
  near_("위도", prop->GetLatitudeDeg(), 0.0, tol.geo, bad);
  near_("경도", prop->GetLongitudeDeg(), 0.0, tol.geo, bad);
  near_("고도[ft]", prop->GetAltitudeASL(), 30000.0, tol.alt, bad);
  near_("v_north", vned(1), 0.0, tol.vel, bad);
  near_("v_east", vned(2), 0.0, tol.vel, bad);
  near_("v_down", vned(3), 0.0, tol.vel, bad);

  if (bad) {
    std::fprintf(stderr, "FAIL: 초기조건 계약 %d건 — 적분하지 않음\n", bad);
    return 3;
  }
  std::fprintf(stderr, "  → 초기조건 계약 통과\n\n");

  // ── CSV 는 getter 값으로만 만든다 ───────────────────────────────
  std::fprintf(csv, "time,latitude_deg,longitude_deg,altitudeMsl_ft,"
              "feVelocity_ft_s_X,feVelocity_ft_s_Y,feVelocity_ft_s_Z,"
              "eulerAngle_deg_Roll,eulerAngle_deg_Pitch,eulerAngle_deg_Yaw,"
              "bodyAngularRateWrtEi_deg_s_Roll,"
              "bodyAngularRateWrtEi_deg_s_Pitch,"
              "bodyAngularRateWrtEi_deg_s_Yaw\n");

  const int steps = static_cast<int>(std::llround(duration / dt));
  const int every = static_cast<int>(std::llround(out_dt / dt));
  double worst_force = 0.0, worst_moment = 0.0;

  for (int k = 0; k <= steps; ++k) {
    if (k % every == 0) {
      const auto e = prop->GetEuler();
      const auto w = prop->GetPQRi();
      const auto v = prop->GetVel();
      std::fprintf(csv, "%.10g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,"
                  "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g\n",
                  k * dt, prop->GetLatitudeDeg(), prop->GetLongitudeDeg(),
                  prop->GetAltitudeASL(), v(1), v(2), v(3),
                  e(1) * kRad2Deg, e(2) * kRad2Deg, e(3) * kRad2Deg,
                  w(1) * kRad2Deg, w(2) * kRad2Deg, w(3) * kRad2Deg);
    }
    // 공력 힘·모멘트가 전 구간 0이어야 한다 — 사례 2 의 전제이며,
    // 불변량 검사가 성립하는 근거다.
    for (int i = 1; i <= 3; ++i) {
      worst_force = std::max(worst_force, std::fabs(air->GetForces(i)));
      worst_moment = std::max(worst_moment, std::fabs(air->GetMoments(i)));
    }
    if (k < steps && !fdm.Run()) {
      std::fprintf(stderr, "FAIL: Run() 이 t=%.4f 에서 중단\n", k * dt);
      return 2;
    }
  }

  std::fprintf(stderr,
               "[검사] 전 구간 공력 힘 최대 %.3e lbf, 모멘트 최대 %.3e ftlbf\n",
               worst_force, worst_moment);
  if (worst_force > 1e-12 || worst_moment > 1e-12) {
    std::fprintf(stderr,
                 "FAIL: 사례 2 는 공력이 0이어야 한다 — 불변량 검사의 전제\n");
    return 3;
  }
  std::fclose(csv);
  std::fprintf(stderr, "[검사] 통과 — dt=%g, %d 스텝, 출력 %g s 간격 → %s\n",
               dt, steps, out_dt, csv_path.c_str());
  return 0;
}
