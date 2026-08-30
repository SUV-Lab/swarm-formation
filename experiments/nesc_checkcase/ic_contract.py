#!/usr/bin/env python3
"""NESC 대기권 사례 2 — t=0 초기조건 계약. 후보 생성 전에 반드시 통과.

생산 코드가 아니다.

왜 이 파일이 따로 있나 — 각속도 프레임이 둘이다
------------------------------------------------
JSBSim 은 몸체 각속도를 두 가지로 들고 있다:

    vPQRi = vPQR + Ti2b * omegaPlanet      (FGPropagate.cpp)

    p/q/r     ECEF(회전하는 지구)에 대한 몸체 각속도
    pi/qi/ri  관성계에 대한 몸체 각속도

**NESC 출력의 10/20/30 °/s 는 후자(관성계)다.** 사양 본문이 못 박는다 —
*"The initial rotation rate … was such that the inertial rates were 10, 20
and 30 deg/s … Table 26 gives these rates relative to the rotating Earth, so
a smaller value for 'roll rate' is specified."*

그러므로 JSBSim 의 일반 p/q/r 에 10/20/30 을 그대로 넣으면 **지구 자전분이
한 번 더 붙는다.** 이 사례의 위치·자세(적도·본초자오선, 자세 0/0/0)에서는
NED 기저의 지구 자전 벡터가 북(roll)축 성분만 갖는다:

    ω_지구 = 7.292115e-5 rad/s = 0.0041780741 °/s
    p_ecef = 10 − 0.0041780741 = 9.9958219259 °/s
    q, r 은 변화 없음 (동·하 성분이 0)

초기화 직후 아래 값을 **직접 단정**하는 것이 가장 안전하다. 어떤 입력
방법을 쓰든(attitude_rate frame="eci" 를 쓰면 입력이 몸체 PQR 이 아니라
ECI 성분이라 좌표변환이 또 필요하다) 결과를 확인하면 방법에 무관하다.
"""
import argparse
import math
import sys

OMEGA_EARTH = 7.292115e-5                      # rad/s, WGS-84 / NESC
OMEGA_DEG_S = math.degrees(OMEGA_EARTH)        # 0.0041780741 deg/s

# 관성계 기준 — NESC 사양이 요구하는 값
PQRi_DEG_S = (10.0, 20.0, 30.0)

# ECEF 기준 — 위 값에서 유도한다. 상수로 적어 넣지 않는다.
# 적도·본초자오선·자세 0/0/0 이므로 몸체축 = NED 이고 ω 는 북(roll)만.
PQR_DEG_S = (PQRi_DEG_S[0] - OMEGA_DEG_S, PQRi_DEG_S[1], PQRi_DEG_S[2])

ALT_FT = 30000.0
TOL = dict(rate=1e-6, angle=1e-9, geo=1e-9, alt=1e-6, vel=1e-9)


def check_ic(st):
    """t=0 상태를 fail-closed 로 검사. st 는 dict.

    필수 키: pqri(3), pqr(3), euler_deg(3), lat_deg, lon_deg, alt_ft,
             v_local_fps(3)
    """
    bad = []

    def near(name, got, want, tol):
        if got is None or not math.isfinite(got):
            bad.append(f"{name}: 값 없음/비유한 ({got})")
        elif abs(got - want) > tol:
            bad.append(f"{name}: {got!r} vs {want} (허용 {tol:g})")

    for i, ax in enumerate("pqr"):
        near(f"pqri_{ax} (관성계)", st.get("pqri", [None] * 3)[i],
             PQRi_DEG_S[i], TOL["rate"])
    for i, ax in enumerate("pqr"):
        near(f"pqr_{ax} (ECEF)", st.get("pqr", [None] * 3)[i],
             PQR_DEG_S[i], TOL["rate"])
    for i, nm in enumerate(("roll", "pitch", "yaw")):
        near(f"자세 {nm}", st.get("euler_deg", [None] * 3)[i], 0.0,
             TOL["angle"])
    near("위도", st.get("lat_deg"), 0.0, TOL["geo"])
    near("경도", st.get("lon_deg"), 0.0, TOL["geo"])
    near("고도[ft]", st.get("alt_ft"), ALT_FT, TOL["alt"])
    for i, nm in enumerate(("north", "east", "down")):
        near(f"local velocity {nm}", st.get("v_local_fps", [None] * 3)[i],
             0.0, TOL["vel"])

    if bad:
        for b in bad:
            print(f"FAIL: 초기조건 계약 — {b}", file=sys.stderr)
        return False
    return True


def nominal():
    return dict(pqri=list(PQRi_DEG_S), pqr=list(PQR_DEG_S),
                euler_deg=[0.0, 0.0, 0.0], lat_deg=0.0, lon_deg=0.0,
                alt_ft=ALT_FT, v_local_fps=[0.0, 0.0, 0.0])


def selftest():
    """계약이 실제로 거절하는지 — 특히 **프레임 혼동**을 잡는가."""
    # 상수 자체를 **리터럴로** 단정한다. 픽스처를 nominal() 에서만 만들면
    # 상수를 바꾸는 변이가 픽스처도 함께 바꿔 살아남는다 — 자기참조다.
    # 아래 값은 NESC 사양(관성계 10/20/30)과 ω_지구에서 독립적으로 나온다.
    const_bad = []
    if tuple(PQRi_DEG_S) != (10.0, 20.0, 30.0):
        const_bad.append(f"PQRi 가 {PQRi_DEG_S} — (10,20,30) 이어야")
    if abs(PQR_DEG_S[0] - 9.9958219259) > 1e-9:
        const_bad.append(f"PQR roll 이 {PQR_DEG_S[0]:.10f} — 9.9958219259 "
                         f"이어야 (관성계 10 에서 ω_지구를 뺀 값)")
    if abs(PQR_DEG_S[0] - PQRi_DEG_S[0]) < 1e-9:
        const_bad.append("PQR 과 PQRi 가 같다 — 프레임 구분이 사라졌다")
    for c in const_bad:
        print(f"FAIL: 상수 — {c}", file=sys.stderr)

    cases = [("정확한 초기조건", nominal(), True)]

    # 이 파일이 존재하는 이유. 10/20/30 을 ECEF 쪽에 그대로 넣으면
    # 관성계가 10.0041780741 이 된다.
    st = nominal()
    st["pqr"] = list(PQRi_DEG_S)
    st["pqri"] = [PQRi_DEG_S[0] + OMEGA_DEG_S, 20.0, 30.0]
    cases.append(("프레임 혼동 — p/q/r 에 10/20/30 을 그대로 입력", st, False))

    # 자전 보정을 반대 부호로
    st = nominal(); st["pqr"] = [10.0 + OMEGA_DEG_S, 20.0, 30.0]
    cases.append(("자전 보정 부호 반대", st, False))

    st = nominal(); st["euler_deg"] = [0.0, 1e-6, 0.0]
    cases.append(("자세 피치 1e-6 deg", st, False))
    st = nominal(); st["alt_ft"] = 29999.9
    cases.append(("고도 29999.9 ft", st, False))
    st = nominal(); st["v_local_fps"] = [0.0, 1e-6, 0.0]
    cases.append(("local velocity 비영", st, False))
    st = nominal(); st["lat_deg"] = 1e-6
    cases.append(("위도 1e-6 deg", st, False))

    import contextlib
    import io
    fails = 0
    print("초기조건 계약 자기시험\n")
    print(f"  ω_지구        {OMEGA_DEG_S:.10f} deg/s")
    print(f"  pqri (관성계)  {PQRi_DEG_S[0]:.10f} / {PQRi_DEG_S[1]} / "
          f"{PQRi_DEG_S[2]}")
    print(f"  pqr  (ECEF)   {PQR_DEG_S[0]:.10f} / {PQR_DEG_S[1]} / "
          f"{PQR_DEG_S[2]}\n")
    for name, st, want in cases:
        buf = io.StringIO()
        with contextlib.redirect_stderr(buf):
            got = check_ic(st)
        ok = (got == want)
        fails += 0 if ok else 1
        print(f"  {'OK  ' if ok else 'FAIL'}  {name:<44}"
              f"{'통과' if got else '거절'} ({'통과' if want else '거절'} 기대)")
    if fails or const_bad:
        print(f"\nFAIL: 픽스처 {fails}건, 상수 {len(const_bad)}건",
              file=sys.stderr)
        return 1
    print(f"\nOK: {len(cases)}개 전부 의도대로")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--print", action="store_true",
                    help="후보 생성기가 맞춰야 할 값을 출력")
    a = ap.parse_args()
    if a.selftest:
        return selftest()
    if a.print:
        print(f"pqri_deg_s  {PQRi_DEG_S[0]:.10f} {PQRi_DEG_S[1]:.10f} "
              f"{PQRi_DEG_S[2]:.10f}   # 관성계 — NESC 출력 기준")
        print(f"pqr_deg_s   {PQR_DEG_S[0]:.10f} {PQR_DEG_S[1]:.10f} "
              f"{PQR_DEG_S[2]:.10f}   # ECEF — JSBSim 일반 p/q/r 입력")
        print(f"euler_deg   0 0 0")
        print(f"lat_lon_deg 0 0")
        print(f"alt_ft      {ALT_FT}")
        print(f"v_local_fps 0 0 0")
        return 0
    ap.print_help()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
