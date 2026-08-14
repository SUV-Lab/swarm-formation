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
#include <array>

#include <FGFDMExec.h>
#include <models/FGPropagate.h>
#include <models/FGAuxiliary.h>
#include <models/FGAircraft.h>
#include <models/FGAerodynamics.h>
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
  // 적분기: JSBSim v1.3.1 은 생성자에서 회전 각속도·자세를 **둘 다**
  // eRectEuler 로 둔다 (FGPropagate.cpp:93,95). 헤더 주석은 AB2/Trapezoidal
  // 이라 적혀 있어 문서와 구현이 다르다 (FGPropagate.h:143). 그러므로
  // 기본값에 의존하지 않고 명시적으로 설정하고 실행 시 단정한다.
  //   eNone=0 RectEuler=1 Trapezoidal=2 AB2=3 AB3=4 AB4=5
  //   Buss1=6 Buss2=7 LocalLinearization=8 AB5=9
  const int int_rate = argc > 6 ? std::atoi(argv[6]) : 1;   // 기본 = 현재 동작
  const int int_att  = argc > 7 ? std::atoi(argv[7]) : 1;
  // 사례를 **명시적으로** 분류한다. model != "nesc_brick" 같은 간접
  // 분류는 오타 하나로 조용히 다른 계약을 적용한다.
  const std::string caseid = argc > 8 ? argv[8] : "acc02";
  std::string model;
  if (caseid == "acc02")      model = "nesc_brick";
  else if (caseid == "acc03") model = "nesc_brick_damped";
  else if (caseid.rfind("forced_", 0) == 0) model = "nesc_brick_forced";
  else {
    std::fprintf(stderr, "FAIL: 사례 '%s' 는 acc02/acc03 이 아니다\n",
                 caseid.c_str());
    return 3;
  }
  const bool damped = (caseid == "acc03");
  const bool forced = (caseid.rfind("forced_", 0) == 0);
  const std::string profile = forced ? caseid.substr(7) : "";
  const std::string sched_path = argc > 9 ? argv[9] : "schedule.txt";
  // fail-open 제거: readback 은 지원하지 않는 값도 그대로 돌려주므로
  // 검사가 되지 않는다. JSBSim 의 switch 가 default 로 빠지면 적분이
  // 조용히 멈춘다. 종류별 허용 집합을 여기서 못 박는다.
  //   각속도용: RectEuler AB2 AB3 AB4 AB5 (벡터 적분)
  //   자세용:   RectEuler AB2 AB3 AB4 Buss1 Buss2 LocalLinearization
  const std::vector<int> kRateOk{1, 3, 4, 5, 9};
  const std::vector<int> kAttOk{1, 3, 4, 5, 6, 7, 8};
  auto allowed = [](const std::vector<int> &v, int x) {
    for (int a : v) if (a == x) return true;
    return false;
  };
  if (!allowed(kRateOk, int_rate)) {
    std::fprintf(stderr, "FAIL: 각속도 적분기 %d 는 허용 집합에 없다\n",
                 int_rate);
    return 3;
  }
  if (!allowed(kAttOk, int_att)) {
    std::fprintf(stderr, "FAIL: 자세 적분기 %d 는 허용 집합에 없다\n",
                 int_att);
    return 3;
  }
  // CSV 는 파일로. stdout 에는 JSBSim 배너가 섞이므로 섞으면 안 된다.
  std::FILE *csv = std::fopen(csv_path.c_str(), "w");
  if (!csv) { std::fprintf(stderr, "FAIL: %s 열기 실패\n", csv_path.c_str()); return 2; }
  // %.12g 로 직렬화한 CSV 는 작은 차이를 잘라낸다. 진짜 double 비트
  // 반복성을 보려면 손실 없는 표기가 따로 필요하다 — %a 16진 부동소수점.
  const std::string hex_path = csv_path + ".hex";
  std::FILE *hexf = std::fopen(hex_path.c_str(), "w");
  if (!hexf) { std::fprintf(stderr, "FAIL: %s 열기 실패\n", hex_path.c_str()); return 2; }
  const std::string mom_path = csv_path + ".mom";
  std::FILE *mf = std::fopen(mom_path.c_str(), "w");
  if (!mf) { std::fprintf(stderr, "FAIL: %s 열기 실패\n", mom_path.c_str()); return 2; }

  JSBSim::FGFDMExec fdm;
  fdm.SetRootDir(SGPath(root));
  fdm.SetAircraftPath(SGPath("aircraft"));
  fdm.SetDebugLevel(0);
  if (!fdm.LoadModel(model)) {
    std::fprintf(stderr, "FAIL: LoadModel(%s)\n", model.c_str());
    return 2;
  }
  // 외부 모멘트 프로퍼티는 XML 의 function 이 평가되기 전에 존재해야
  // 한다. LoadModel 직후에 만든다.
  std::vector<std::array<double, 4>> sched;
  if (forced) {
    // 스케줄은 forced_profile.py 가 만든다. 생성기는 소비만 한다.
    std::FILE *sf = std::fopen(sched_path.c_str(), "r");
    if (!sf) {
      std::fprintf(stderr, "FAIL: 스케줄 %s 없음 — forced_profile.py 로 "
                   "먼저 만든다\n", sched_path.c_str());
      return 3;
    }
    std::array<double, 4> row{};
    while (std::fscanf(sf, "%lf %lf %lf %lf", &row[0], &row[1], &row[2],
                       &row[3]) == 4)
      sched.push_back(row);
    std::fclose(sf);
    if (sched.empty()) {
      std::fprintf(stderr, "FAIL: 스케줄이 비었다\n");
      return 3;
    }
    fdm.SetPropertyValue("forced/moment-l", 0.0);
    fdm.SetPropertyValue("forced/moment-m", 0.0);
    fdm.SetPropertyValue("forced/moment-n", 0.0);
  }
  fdm.Setdt(dt);
  fdm.SetPropertyValue("simulation/integrator/rate/rotational", int_rate);
  fdm.SetPropertyValue("simulation/integrator/position/rotational", int_att);

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
  // FGAircraft::GetMoments 는 외부 반력까지 합산한다. 공력만 보려면
  // FGAerodynamics 쪽을 봐야 한다 — 강제응답 리그에서 이 구분이 필요하다.
  auto aero = fdm.GetAerodynamics();

  const auto pqri = prop->GetPQRi();
  const auto pqr = prop->GetPQR();
  const auto euler = prop->GetEuler();
  const auto vned = prop->GetVel();

  Tol tol;
  int bad = 0;
  // 설정이 실제로 먹었는지 읽어서 단정한다 — 기본값에 의존하지 않는다.
  const int got_rate =
      static_cast<int>(fdm.GetPropertyValue("simulation/integrator/rate/rotational"));
  const int got_att =
      static_cast<int>(fdm.GetPropertyValue("simulation/integrator/position/rotational"));
  std::fprintf(stderr, "[적분기] 회전 각속도 %d, 회전 자세 %d "
               "(요청 %d / %d)\n", got_rate, got_att, int_rate, int_att);
  if (got_rate != int_rate || got_att != int_att) {
    std::fprintf(stderr, "FAIL: 적분기 설정이 반영되지 않았다\n");
    return 3;
  }
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
    // 강제응답: 스케줄 파일의 k 번째 행을 그대로 쓴다. **여기에는
    // 프로파일 정의가 없다** — 앞선 판은 C++ 에 다시 구현해두고 "한 곳에
    // 정의"라고 적었다. 두 구현이 어긋나면 그 차이가 적분기 차이로 보인다.
    if (forced) {
      const size_t idx = static_cast<size_t>(k);
      if (idx >= sched.size()) {
        std::fprintf(stderr, "FAIL: 스케줄이 %zu 행뿐인데 스텝 %d\n",
                     sched.size(), k);
        return 3;
      }
      fdm.SetPropertyValue("forced/moment-l", sched[idx][1]);
      fdm.SetPropertyValue("forced/moment-m", sched[idx][2]);
      fdm.SetPropertyValue("forced/moment-n", sched[idx][3]);
    }
    if (k % every == 0) {
      const auto e = prop->GetEuler();
      const auto w = prop->GetPQRi();
      const auto v = prop->GetVel();
      // 손실 없는 상태 기록. 오일러각은 파생값이므로 **ECI 쿼터니언**을
      // 직접 쓴다. 관성 위치·속도까지 넣어야 전체 상태 반복성이 된다.
      // (질량 상태가 생기면 질량·CG 를 여기 추가한다.)
      const auto &qi = prop->GetQuaternionECI();
      const auto pi_ = prop->GetInertialPosition();
      const auto vi_ = prop->GetInertialVelocity();
      const auto wi_ = prop->GetPQRi();
      const auto vned_k = prop->GetVel();
      if (false) {  // (아래 매-스텝 블록으로 옮김)
        const double ax = fdm.GetPropertyValue("moments/l-external-lbsft");
        const double ay = fdm.GetPropertyValue("moments/m-external-lbsft");
        const double az = fdm.GetPropertyValue("moments/n-external-lbsft");
        std::fprintf(mf, "%a %a %a %a\n", k * dt, ax, ay, az);
        // 명령값과 축·부호·크기를 매 스텝 재폐쇄한다.
        // 지연 없음. Run() 뒤에 읽는 값은 방금 쓴 명령 그대로다 — 앞서
      // "한 스텝 지연"으로 보였던 것은 모멘트 적용 코드가 편집 중
      // 사라진 탓이었고, 재폐쇄가 그것을 잡았다.
      const double cmd[3] = {sched[k][1], sched[k][2], sched[k][3]};
        const double act[3] = {ax, ay, az};
        for (int i = 0; i < 3; ++i) {
          const double sc = std::max(1e-12, std::fabs(cmd[i]));
          if (std::fabs(act[i] - cmd[i]) > 1e-9 * sc) {
            std::fprintf(stderr, "FAIL: t=%.4f 축%d 외부 모멘트 재폐쇄 "
                         "실패 — 명령 %.12g, 실제 %.12g\n",
                         k * dt, i + 1, cmd[i], act[i]);
            return 3;
          }
        }
      }
      std::fprintf(hexf,
                   "%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a,%a\n",
                   k * dt,
                   qi(1), qi(2), qi(3), qi(4),
                   wi_(1), wi_(2), wi_(3),
                   pi_(1), pi_(2), pi_(3),
                   vi_(1), vi_(2), vi_(3),
                   vned_k(1), vned_k(2), vned_k(3));
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
      worst_force = std::max(worst_force, std::fabs(aero->GetForces(i)));
      worst_moment = std::max(worst_moment, std::fabs(aero->GetMoments(i)));
    }
    if (damped) {
      // 감쇠 모멘트를 해석식으로 재폐쇄한다. 모델이 "0이 아니다"만으로는
      // 계수나 축이 틀려도 통과한다.
      //   L = qbar S b  Clp (p b / 2V),  M = qbar S c Cmq (q c / 2V),
      //   N = qbar S b  Cnr (r b / 2V),  Clp=Cmq=Cnr=-1
      const double qbar = fdm.GetPropertyValue("aero/qbar-psf");
      const double S = fdm.GetPropertyValue("metrics/Sw-sqft");
      const double b = fdm.GetPropertyValue("metrics/bw-ft");
      const double c = fdm.GetPropertyValue("metrics/cbarw-ft");
      const double b2v = fdm.GetPropertyValue("aero/bi2vel");
      const double c2v = fdm.GetPropertyValue("aero/ci2vel");
      const double pa = fdm.GetPropertyValue("velocities/p-aero-rad_sec");
      const double qa = fdm.GetPropertyValue("velocities/q-aero-rad_sec");
      const double ra = fdm.GetPropertyValue("velocities/r-aero-rad_sec");
      const double want[3] = {-qbar * S * b * b2v * pa,
                              -qbar * S * c * c2v * qa,
                              -qbar * S * b * b2v * ra};
      for (int i = 0; i < 3; ++i) {
        const double got = aero->GetMoments(i + 1);
        const double scale = std::max(1e-12, std::fabs(want[i]));
        if (std::fabs(got - want[i]) > 1e-9 * scale) {
          std::fprintf(stderr,
                       "FAIL: t=%.4f 축%d 감쇠 모멘트 재폐쇄 실패 — "
                       "모델 %.12g, 해석식 %.12g\n", k * dt, i + 1, got,
                       want[i]);
          return 3;
        }
      }
    }
    if (k < steps && !fdm.Run()) {
      std::fprintf(stderr, "FAIL: Run() 이 t=%.4f 에서 중단\n", k * dt);
      return 2;
    }
    if (forced && k < steps) {
      // **Run() 이후에** 읽는다. 외부 반력은 Run 안에서 계산되므로,
      // 프로퍼티를 쓰자마자 읽으면 직전 값이 나온다 (재폐쇄가 이 순서
      // 오류를 t=1.001 에서 잡았다). 여기서 읽는 값이 [t_k, t_k+dt]
      // 구간에 실제로 적용된 모멘트다 — ZOH 재생의 입력이 된다.
      const double ax = fdm.GetPropertyValue("moments/l-external-lbsft");
      const double ay = fdm.GetPropertyValue("moments/m-external-lbsft");
      const double az = fdm.GetPropertyValue("moments/n-external-lbsft");
      std::fprintf(mf, "%a %a %a %a\n", k * dt, ax, ay, az);
      // 지연 없음. Run() 뒤에 읽는 값은 방금 쓴 명령 그대로다 — 앞서
      // "한 스텝 지연"으로 보였던 것은 모멘트 적용 코드가 편집 중
      // 사라진 탓이었고, 재폐쇄가 그것을 잡았다.
      const double cmd[3] = {sched[k][1], sched[k][2], sched[k][3]};
      const double act[3] = {ax, ay, az};
      for (int i = 0; i < 3; ++i) {
        const double sc = std::max(1e-12, std::fabs(cmd[i]));
        if (std::fabs(act[i] - cmd[i]) > 1e-9 * sc) {
          std::fprintf(stderr, "FAIL: t=%.4f 축%d 외부 모멘트 재폐쇄 실패 "
                       "— 명령 %.12g, 실제 %.12g\n", k * dt, i + 1,
                       cmd[i], act[i]);
          return 3;
        }
      }
    }
  }

  std::fprintf(stderr,
               "[검사] 전 구간 공력 힘 최대 %.3e lbf, 모멘트 최대 %.3e ftlbf\n",
               worst_force, worst_moment);
  if (forced) {
    // 강제응답 리그는 공력이 0이어야 한다 — 외부 모멘트만 작용해야
    // 일-에너지 폐쇄 dKE/dt = M·ω 가 성립한다.
    if (worst_force > 1e-12 || worst_moment > 1e-12) {
      std::fprintf(stderr, "FAIL: 강제응답 리그의 공력이 0이 아니다 "
                   "(힘 %.3e, 모멘트 %.3e)\n", worst_force, worst_moment);
      return 3;
    }
    std::fprintf(stderr, "[검사] 강제응답 %s — 공력 0, 외부 모멘트만\n",
                 profile.c_str());
  } else if (!damped) {
    if (worst_force > 1e-12 || worst_moment > 1e-12) {
      std::fprintf(stderr,
                   "FAIL: 사례 2 는 공력이 0이어야 한다 — 불변량의 전제\n");
      return 3;
    }
  } else {
    // 사례 3 은 "Dragless Brick with Aerodynamic Damping" 이다.
    //   공력 힘   전 구간 0        (CD 를 사례가 0 으로 덮어쓴다)
    //   공력 모멘트 유의미하게 0 아님 + 축별 해석식 재폐쇄 (위 루프)
    if (worst_force > 1e-12) {
      std::fprintf(stderr,
                   "FAIL: 사례 3 도 무항력이다 — 공력 힘 최대 %.3e lbf. "
                   "기준 표의 CD 0.01 을 사례가 0 으로 덮어쓴다\n",
                   worst_force);
      return 3;
    }
    if (worst_moment <= 1e-9) {
      std::fprintf(stderr,
                   "FAIL: 감쇠 모멘트 최대 %.3e — 모델이 붙지 않았다\n",
                   worst_moment);
      return 3;
    }
    std::fprintf(stderr, "[검사] 감쇠 사례 — 불변량(KE·H)은 여기서 "
                 "보존되지 않으며 판정에 쓰지 않는다\n");
  }
  std::fclose(csv);
  std::fclose(hexf);
  std::fclose(mf);
  std::fprintf(stderr, "[검사] 통과 — dt=%g, %d 스텝, 출력 %g s 간격 → %s\n",
               dt, steps, out_dt, csv_path.c_str());
  return 0;
}
