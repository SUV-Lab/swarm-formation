#!/usr/bin/env python3
"""NESC 대기권 사례 acc02/acc03 — 자세·각속도 족 산포 + 불변량 자기검사.

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


# NESC 부록 표 4: 벽돌의 질량·관성 (CM 기준, 곱관성 0)
BRICK_I = (0.001894220, 0.006211019, 0.007194665)   # slug-ft^2


def rate_norm(w):
    return math.sqrt(sum(c * c for c in w))


def quat_to_matrix(q):
    w, x, y, z = q
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
            (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
            (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)))


def invariants(series):
    """acc02 전용. 감쇠 없는 강체이고 중력만 작용하므로 CM 둘레의 외부
    모멘트가 0이다. 따라서 **회전 운동에너지**와 **관성계 각운동량**이
    보존된다. 이것은 참여 결과의 산포에서 만든 합격선이 아니라 해석적
    불변량이므로, 비교기 자신에 대한 강한 자기검사가 된다."""
    ke, Lm = [], []
    for q, w_deg in series:
        w = [math.radians(c) for c in w_deg]              # 몸체축 각속도
        ke.append(0.5 * sum(I * c * c for I, c in zip(BRICK_I, w)))
        h_body = [I * c for I, c in zip(BRICK_I, w)]      # 몸체축 각운동량
        R = quat_to_matrix(q)                             # 몸체 → NED
        h_ned = [sum(R[r][c] * h_body[c] for c in range(3)) for r in range(3)]
        Lm.append(h_ned)
    return ke, Lm


def momentum_drift(Lm):
    """관성계 각운동량 벡터의 크기 상대변동과 **방향 드리프트**.

    크기만 보면 자세 적분 오류를 놓친다 — 몸체축 각속도가 맞고 자세만
    틀리면 |H| 는 거의 유지되면서 방향이 돈다. 방향이 더 날카로운 검사다."""
    mag = [math.sqrt(sum(v * v for v in h)) for h in Lm]
    mrel = (max(mag) - min(mag)) / max(sum(mag) / len(mag), 1e-30)
    h0 = Lm[0]
    n0 = math.sqrt(sum(v * v for v in h0)) or 1e-30
    worst = 0.0
    for h, m in zip(Lm, mag):
        c = sum(a * b for a, b in zip(h, h0)) / (max(m, 1e-30) * n0)
        worst = max(worst, math.degrees(math.acos(min(1.0, max(-1.0, c)))))
    return mrel, worst


def contract_check(name, rows, ts):
    """시간축·유한성·단조성·301점 일대일 대응을 fail-closed로."""
    bad = []
    if len(rows) != len(GRID):
        bad.append(f"표본 {len(rows)}개 (301 기대)")
    if any(b - a <= 0 for a, b in zip(ts, ts[1:])):
        bad.append("시간축이 단조 증가가 아님")
    if len(set(ts)) != len(ts):
        bad.append("중복 시각 — 격자 일대일 대응 실패")
    for k, (q, w) in enumerate(rows):
        vals = list(q) + list(w)
        if not all(math.isfinite(v) for v in vals):
            bad.append(f"t={GRID[k]}s 에 비유한값")
            break
        if abs(math.sqrt(sum(c * c for c in q)) - 1.0) > 1e-9:
            bad.append(f"t={GRID[k]}s 쿼터니언 노름 이탈")
            break
    if bad:
        for b in bad:
            print(f"FAIL: {name}: {b}", file=sys.stderr)
        return False
    return True


def stats(diffs):
    """최대 · RMS · 종단 · 최대 발생 시각."""
    mx = max(diffs)
    return (mx,
            math.sqrt(sum(d * d for d in diffs) / len(diffs)),
            diffs[-1],
            GRID[diffs.index(mx)])


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
    out, picked = [], []
    for g in GRID:
        j = min(range(len(ts)), key=lambda k: abs(ts[k] - g))
        if abs(ts[j] - g) > TIME_TOL_S:
            raise SystemExit(
                f"FAIL: {os.path.basename(path)} t={g}s 최근접 오차 "
                f"{abs(ts[j]-g):.3e} s > {TIME_TOL_S:.0e} — 재표본 필요")
        picked.append(ts[j])
        r = rows[1 + j]
        out.append((
            quat_from_euler_deg(*[float(r[idx[c]]) for c in EUL]),
            tuple(float(r[idx[c]]) for c in RATE),
        ))
    return out, picked


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="02", choices=("02", "03"))
    ap.add_argument("--data", default=os.path.join(HERE, ".nesc", "ref"))
    ap.add_argument("--candidate", metavar="CSV",
                    help="우리 결과. 족의 여섯 번째 구성원으로 섞지 않고 "
                         "참여 5종 각각과 따로 대조한다")
    a = ap.parse_args()

    d = os.path.join(a.data, f"atmos_scn_{a.scenario}")
    sims = ["01", "02", "04", "05", "06"]
    paths = {s: os.path.join(d, f"Atmos_{a.scenario}_sim_{s}.csv") for s in sims}
    missing = [s for s, p in paths.items() if not os.path.exists(p)]
    if missing:
        print(f"FAIL: sim {missing} 없음 — ./fetch.sh 를 먼저 실행", file=sys.stderr)
        return 3

    series, ok = {}, True
    for s_, p in paths.items():
        rows, ts = load(p)
        ok &= contract_check(f"sim {s_}", rows, ts)
        series[s_] = rows
    if not ok:
        return 3

    print(f"NESC 대기권 사례 acc{a.scenario} — 참여 도구 {len(sims)}종, "
          f"공통 격자 {len(GRID)}점 (0–30 s @ 0.1 s), 보간 없음")
    print("계약 검사: 시간축 단조·격자 일대일·유한성·쿼터니언 정규화 통과\n")

    # ── 참여 5종의 족 산포. 후보를 넣어도 이 값은 재계산하지 않는다 ──
    print("[족] 자세 각거리 [deg] — 참여 5종 쌍별 (오일러각 직접 차이가 아님)")
    print(f"{'쌍':<10}{'최대':>9}{'RMS':>9}{'종단':>9}{'t_max':>8}")
    pair_max = []
    for i_, s1 in enumerate(sims):
        for s2 in sims[i_ + 1:]:
            dif = [attitude_angle_deg(series[s1][k][0], series[s2][k][0])
                   for k in range(len(GRID))]
            mx, rms, end, tm = stats(dif)
            pair_max.append(mx)
            print(f"{s1+'-'+s2:<10}{mx:>9.4f}{rms:>9.4f}{end:>9.4f}{tm:>8.1f}")
    print(f"  → 족의 산포 (고정): {max(pair_max):.4f} deg\n")

    print("[족] 몸체 각속도 [deg/s] — 성분별 최대 및 벡터 노름 최대")
    print(f"{'쌍':<10}{'Roll':>9}{'Pitch':>9}{'Yaw':>9}{'‖Δω‖':>9}")
    rmax, nmax = [0.0] * 3, 0.0
    for i_, s1 in enumerate(sims):
        for s2 in sims[i_ + 1:]:
            m = [max(abs(series[s1][k][1][c] - series[s2][k][1][c])
                     for k in range(len(GRID))) for c in range(3)]
            nm = max(rate_norm([series[s1][k][1][c] - series[s2][k][1][c]
                                for c in range(3)]) for k in range(len(GRID)))
            rmax = [max(x, y) for x, y in zip(rmax, m)]
            nmax = max(nmax, nm)
            print(f"{s1+'-'+s2:<10}{m[0]:>9.4f}{m[1]:>9.4f}{m[2]:>9.4f}{nm:>9.4f}")
    print(f"  → 족의 산포 (고정): 성분 {rmax[0]:.4f}/{rmax[1]:.4f}/{rmax[2]:.4f}"
          f", 노름 {nmax:.4f} deg/s\n")

    # ── acc02 불변량: 참여 결과가 아니라 해석적 성질에 대한 자기검사 ──
    if a.scenario == "02":
        print("[불변량] 감쇠 없는 강체 — 회전 운동에너지와 관성계 각운동량 보존")
        print("  (참여 결과에서 만든 합격선이 아니라 해석적 불변량)")
        print(f"{'sim':<10}{'KE 상대변동':>14}{'‖H‖ 상대변동':>16}"
              f"{'H 방향드리프트[deg]':>22}")
        for s_ in sims:
            ke, Lm = invariants(series[s_])
            dke = (max(ke) - min(ke)) / max(abs(sum(ke) / len(ke)), 1e-30)
            dmag, ddir = momentum_drift(Lm)
            print(f"{s_:<10}{dke:>14.3e}{dmag:>16.3e}{ddir:>22.4f}")
        print()

    # ── 우리 결과: 여섯 번째 구성원이 아니라 별도 후보 ──
    if a.candidate:
        cand, cts = load(a.candidate)
        if not contract_check("candidate", cand, cts):
            return 3
        print("[후보] 우리 결과 대 참여 5종 — 족에 섞지 않고 각각과 대조")
        print(f"{'대상':<10}{'자세 최대':>11}{'RMS':>9}{'종단':>9}{'t_max':>8}"
              f"{'‖Δω‖ 최대':>12}")
        for s_ in sims:
            dif = [attitude_angle_deg(cand[k][0], series[s_][k][0])
                   for k in range(len(GRID))]
            mx, rms, end, tm = stats(dif)
            nm = max(rate_norm([cand[k][1][c] - series[s_][k][1][c]
                                for c in range(3)]) for k in range(len(GRID)))
            print(f"{s_:<10}{mx:>11.4f}{rms:>9.4f}{end:>9.4f}{tm:>8.1f}{nm:>12.4f}")
        if a.scenario == "02":
            ke, Lm = invariants(cand)
            dke = (max(ke) - min(ke)) / max(abs(sum(ke) / len(ke)), 1e-30)
            dmag, ddir = momentum_drift(Lm)
            print(f"\n  후보 불변량: KE {dke:.3e}, ‖H‖ {dmag:.3e}, "
                  f"H 방향 드리프트 {ddir:.4f} deg")
        print()

    print("위 숫자는 전부 측정값이다. 참여 결과의 최솟값–최댓값 안에 드는")
    print("것을 합격 조건으로 삼지 않는다.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
