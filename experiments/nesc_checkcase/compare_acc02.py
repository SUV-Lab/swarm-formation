#!/usr/bin/env python3
"""NESC 대기권 사례 acc02 — 참여 도구 간 자세·각속도 산포 측정.

생산 코드가 아니다. `./fetch.sh` 로 받은 `.nesc/ref/` 를 읽는다.

무엇을 하는가
-------------
참여 도구 5종의 기준 시계열을 서로 비교해 **해의 족이 얼마나 퍼져 있는지**
측정한다. 우리 구현을 넣기 전에 족 자체의 산포를 알아야, 나중에 우리
결과의 차이가 "틀린 것"인지 "족 안"인지 말할 수 있다.

합격선을 만들지 않는다
----------------------
보고서 자체가 단일 정답을 거부하고 해의 족을 제시한다. 최솟값–최댓값 안에
드는 것을 조건으로 삼지 않으며, 여기서 내는 것은 전부 **측정값**이다.

시각 정렬 — 보간하지 않는다
---------------------------
열 파일 모두 0–30 s 를 0.1 s 로 덮는 301점 공통 격자를 갖는다(고주기 파일은
3001점이지만 같은 격자를 포함). 실측한 최근접 시각 오차는 최대 7.629e-7 s
이므로 **최근접 표본을 1e-6 s 이내에서만 수용**한다.

오일러각을 보간하면 래핑과 특이점에서 조용히 틀린다. 보간하지 않는 편이
안전하고, 이 자료에서는 보간할 이유도 없다.

자세 차이 — 오일러각을 직접 빼지 않는다
---------------------------------------
±180° 경계에서 1° 차이가 359° 로 보이고, 피치 ±90° 근처에서는 롤·요가
겹쳐 같은 자세가 전혀 다른 각으로 표기된다. 그래서:

    오일러각 → 쿼터니언 (NESC 규약: NED 기준 3-2-1 = yaw, pitch, roll)
    자세 차이 = 2 · acos(|q1 · q2|)      (부호 모호성 때문에 절댓값)

이 각거리는 표기법에 무관한 실제 회전 각이다.

각속도 — 단위·프레임을 맞춘 뒤 성분별
--------------------------------------
공식 출력 사양에 따르면 `bodyAngularRateWrtEi_deg_s_*` 는 **지구 관성계에
대한 몸체 각속도를 몸체축으로 표현**한 값이고 단위는 deg/s 다. 다섯 파일이
같은 정의를 쓰므로 성분별로 그대로 비교한다.
"""
import argparse
import csv
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TIME_TOL_S = 1e-6
GRID = [round(i * 0.1, 10) for i in range(301)]

EUL = ("eulerAngle_deg_Roll", "eulerAngle_deg_Pitch", "eulerAngle_deg_Yaw")
RATE = ("bodyAngularRateWrtEi_deg_s_Roll",
        "bodyAngularRateWrtEi_deg_s_Pitch",
        "bodyAngularRateWrtEi_deg_s_Yaw")


def quat_from_euler_deg(roll, pitch, yaw):
    """NESC 규약 3-2-1 (yaw→pitch→roll), NED 기준. 반환 (w, x, y, z)."""
    cr, sr = math.cos(math.radians(roll) / 2), math.sin(math.radians(roll) / 2)
    cp, sp = math.cos(math.radians(pitch) / 2), math.sin(math.radians(pitch) / 2)
    cy, sy = math.cos(math.radians(yaw) / 2), math.sin(math.radians(yaw) / 2)
    return (cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)


def attitude_angle_deg(q1, q2):
    """두 자세 사이의 각거리 [deg]. q 와 −q 가 같은 자세이므로 절댓값."""
    d = abs(sum(a * b for a, b in zip(q1, q2)))
    return math.degrees(2.0 * math.acos(min(1.0, max(-1.0, d))))


def load(path):
    """공통 격자 시각에 대해 최근접 표본을 뽑는다. 보간하지 않는다."""
    with open(path, newline="") as fh:
        rows = list(csv.reader(fh))
    hdr = [c.strip() for c in rows[0]]
    idx = {c: i for i, c in enumerate(hdr)}
    for c in ("time",) + EUL + RATE:
        if c not in idx:
            raise SystemExit(f"FAIL: {path} 에 열 '{c}' 없음")
    ts = [float(r[idx["time"]]) for r in rows[1:]]
    out = []
    for g in GRID:
        j = min(range(len(ts)), key=lambda k: abs(ts[k] - g))
        if abs(ts[j] - g) > TIME_TOL_S:
            raise SystemExit(
                f"FAIL: {os.path.basename(path)} t={g}s 최근접 오차 "
                f"{abs(ts[j]-g):.3e} s > {TIME_TOL_S:.0e} — 재표본 필요")
        r = rows[1 + j]
        out.append((
            quat_from_euler_deg(*[float(r[idx[c]]) for c in EUL]),
            tuple(float(r[idx[c]]) for c in RATE),
        ))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="02", choices=("02", "03"))
    ap.add_argument("--data", default=os.path.join(HERE, ".nesc", "ref"))
    a = ap.parse_args()

    d = os.path.join(a.data, f"atmos_scn_{a.scenario}")
    sims = ["01", "02", "04", "05", "06"]
    paths = {s: os.path.join(d, f"Atmos_{a.scenario}_sim_{s}.csv") for s in sims}
    missing = [s for s, p in paths.items() if not os.path.exists(p)]
    if missing:
        print(f"FAIL: sim {missing} 없음 — ./fetch.sh 를 먼저 실행", file=sys.stderr)
        return 3

    series = {s: load(p) for s, p in paths.items()}
    print(f"NESC 대기권 사례 acc{a.scenario} — 참여 도구 {len(sims)}종, "
          f"공통 격자 {len(GRID)}점 (0–30 s @ 0.1 s), 보간 없음\n")

    print("자세 각거리 [deg] — 쌍별 (오일러각 직접 차이가 아님)")
    print(f"{'쌍':<12}{'최대':>10}{'평균':>10}{'@30s':>10}")
    pair_max = []
    for i, s1 in enumerate(sims):
        for s2 in sims[i + 1:]:
            ang = [attitude_angle_deg(series[s1][k][0], series[s2][k][0])
                   for k in range(len(GRID))]
            pair_max.append(max(ang))
            print(f"{s1+'-'+s2:<12}{max(ang):>10.4f}{sum(ang)/len(ang):>10.4f}"
                  f"{ang[-1]:>10.4f}")
    print(f"\n  족의 산포: 쌍별 최대치의 최댓값 {max(pair_max):.4f} deg")

    print("\n몸체 각속도 성분별 최대 차이 [deg/s] (지구 관성계 기준, 몸체축)")
    print(f"{'쌍':<12}{'Roll':>10}{'Pitch':>10}{'Yaw':>10}")
    rate_max = [0.0, 0.0, 0.0]
    for i, s1 in enumerate(sims):
        for s2 in sims[i + 1:]:
            m = [max(abs(series[s1][k][1][c] - series[s2][k][1][c])
                     for k in range(len(GRID))) for c in range(3)]
            rate_max = [max(a_, b_) for a_, b_ in zip(rate_max, m)]
            print(f"{s1+'-'+s2:<12}{m[0]:>10.4f}{m[1]:>10.4f}{m[2]:>10.4f}")
    print(f"\n  족의 산포: 성분별 최대 "
          f"Roll {rate_max[0]:.4f}  Pitch {rate_max[1]:.4f}  "
          f"Yaw {rate_max[2]:.4f} deg/s")

    print("\n위 숫자는 전부 측정값이다. 합격선이 아니다 — 참여 결과의")
    print("최솟값–최댓값 안에 드는 것을 조건으로 삼지 않는다.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
