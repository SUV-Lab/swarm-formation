#!/usr/bin/env python3
"""적용범위 진단 — 궤적이 자료의 표본 범위 안에 얼마나 있었나.

**비생산. 계수를 맞추거나 외삽하는 도구가 아니다.**

이 도구가 내는 수는 **조건부 민감도 결과**다. 어떤 출처도 확정한 적 없는
경계를 가정하고 그 가정 아래 무엇이 나오는지 보는 것이다. 그래서 경계를
인자로만 받고 근거 문자열을 강제한다.

세 가지를 분리한다 — 섞으면 없는 근거가 생긴다.

1. **비대칭 범위.** 자료는 대칭이 아니다. TR 1096 의 우리 형상 곡선은
   음의 받음각 쪽으로 CL 이 약 -2.6°, Cm 이 약 -3.5° 까지만 그려져 있다.
   |alpha| 로 자르면 없는 음각 자료를 있는 것처럼 쓰게 된다.

2. **CL 과 Cm 의 범위가 다르다.** CL 표는 +20° 까지 있으나 비선형이고,
   Cm 은 약 +10° 에서 형상이 달라진다(원문 p.4 의 관측). 두 계수를 한
   창으로 묶으면 더 좁은 쪽이 더 넓은 쪽을 조용히 제한하거나 그 반대가 된다.

3. **관측점과 유효범위는 다르다.** 원문의 "약 10°" 는 **양의 받음각에서
   관측된 Cm 기울기 변화점**이지, 출처가 선언한 수치적 유효 한계가
   아니다. TN 3911 은 받음각 한계를 **아예 주지 않는다**. 그러므로 10° 를
   쓰더라도 그것은 **보수적으로 선택한 진단 정책 경계**이지 출처의
   적용범위가 아니다.

연속 시간은 하니스가 dwell 을 쌓는 **방식**과 같게 잰다(성립 시 dt 누적,
실패 시 0 복귀). 그러나 **시간 해상도가 다르다.**

  하니스: dt = 0.002 s 마다 판정·누적
  이 CSV: 50 스텝마다 기록 → 0.1 s 간격

즉 이 도구는 0.1 s 표본 하나를 0.1 s 로 누적하므로 **중간 49 스텝의
실패를 놓칠 수 있다.** 여기서 나오는 연속시간은 상한이지 확정값이
아니다. 확정하려면 하니스 루프에서 매 적분 스텝을 직접 집계해야 한다.
그때까지 이 값은 "0.1 초 표본 기반 조건부 진단"으로만 인용한다.
"""
import argparse
import csv
import math
import sys

R, GAMMA, T0, LAPSE = 287.053, 1.4, 288.15, 0.0065


def mach(speed_mps, alt_m):
    t = T0 - LAPSE * min(alt_m, 11000.0)
    return speed_mps / math.sqrt(GAMMA * R * t)


def window(rows, dt, mmin, mmax, amin, amax):
    """(비율, 최장 연속시간, 최장구간 시각). dwell 은 하니스와 같은 방식."""
    n_ok = 0
    dwell = 0.0
    best = 0.0
    best_end = None
    for r in rows:
        m = mach(float(r["speed_mps"]), float(r["alt_m"]))
        al = float(r["alpha_deg"])
        ok = (mmin <= m <= mmax) and (amin <= al <= amax)
        if ok:
            n_ok += 1
            dwell += dt
            if dwell > best:
                best, best_end = dwell, float(r["t"])
        else:
            dwell = 0.0
    return n_ok / len(rows), best, best_end


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--mach-min", type=float, required=True)
    ap.add_argument("--mach-max", type=float, required=True)
    # 비대칭. 자료가 비대칭이므로 계약도 비대칭이어야 한다.
    ap.add_argument("--cl-alpha-min-deg", type=float, required=True)
    ap.add_argument("--cl-alpha-max-deg", type=float, required=True)
    ap.add_argument("--cm-alpha-min-deg", type=float, required=True)
    ap.add_argument("--cm-alpha-max-deg", type=float, required=True)
    ap.add_argument("--basis", required=True,
                    help="경계의 근거. 출처가 확정한 유효범위가 아니면 "
                         "그렇게 적을 것.")
    ap.add_argument("--dwell-required-s", type=float, default=1.0)
    a = ap.parse_args()
    if not a.basis.strip():
        sys.exit("거부: 근거를 적지 않았다.")

    rows = list(csv.DictReader(open(a.csv)))
    if len(rows) < 2:
        sys.exit("거부: 표본이 부족하다.")
    dt = float(rows[1]["t"]) - float(rows[0]["t"])

    print(f"적용범위 진단 — {a.csv}")
    print("  ** 조건부 민감도 결과다. 아래 경계는 어떤 출처도 수치적")
    print("     유효범위로 확정한 적이 없다. **")
    print(f"  근거: {a.basis}")
    print(f"  마하 대역 [{a.mach_min:.4f}, {a.mach_max:.4f}] · "
          f"표본 간격 {dt:.4f} s · dwell 요구 {a.dwell_required_s:.2f} s")
    print(f"  ** 시간 해상도 주의: 하니스는 0.002 s 마다 판정하는데 이")
    print(f"     CSV 는 {dt:.3f} s 간격이다. 중간 스텝의 실패를 놓칠 수")
    print(f"     있으므로 아래 연속시간은 **상한**이다. **")
    print()
    for name, lo, hi in (("CL", a.cl_alpha_min_deg, a.cl_alpha_max_deg),
                         ("Cm", a.cm_alpha_min_deg, a.cm_alpha_max_deg)):
        frac, best, end = window(rows, dt, a.mach_min, a.mach_max, lo, hi)
        ok = best >= a.dwell_required_s
        print(f"  [{name}]  범위 {lo:+.2f}° <= alpha <= {hi:+.2f}°")
        print(f"       ① 동시 만족 표본  {frac*100:.2f}%")
        if end is None:
            print("       ② 최장 연속       없음")
        else:
            print(f"       ② 최장 연속       {best:.4f} s "
                  f"(끝 t = {end:.3f})")
        print(f"       → dwell {a.dwell_required_s:.2f} s "
              f"{'충족' if ok else '미충족'}")
        print()
    print("  ※ 인계 가능성은 ①이 아니라 ②가 정한다.")
    print("  ※ 이 궤적은 **공력 0 모델**에서 나온 것이다. 공력이 들어가면")
    print("     궤적이 달라지므로 예측이 아니라 현재 요구의 기록이다.")


if __name__ == "__main__":
    main()
