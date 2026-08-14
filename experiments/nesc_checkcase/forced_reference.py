#!/usr/bin/env python3
"""강제응답 독립 기준 — DOP853 으로 회전 동역학을 따로 푼다. 비생산.

왜 독립 기준이 성립하나
-----------------------
강제응답 리그는 공력이 0 이고(생성기가 매 스텝 확인) 중력은 CM 에 작용해
모멘트를 만들지 않는다. 따라서 회전 자유도가 **자기완결**이다:

    I ω̇ = M(t) − ω × (I ω)          ω 는 관성계 기준 몸체 각속도
    q̇   = ½ q ⊗ (0, ω)              q 는 몸체 → ECI

병진과 결합하지 않으므로 JSBSim 의 나머지 모델을 재현할 필요가 없다.
초기값은 JSBSim 이 t=0 에 실제로 들고 있던 값을 `.hex` 에서 읽어 쓴다 —
기대값 상수를 다시 쓰지 않는다.

이것은 **다른 방법·다른 코드**이므로 JSBSim 적분기에 대한 독립 기준이다.
"""
import argparse
import math
import sys

import numpy as np
from scipy.integrate import solve_ivp

# 부록 표 4 (slug·ft²). 대각이므로 역행렬이 자명하다.
I_DIAG = np.array([0.001894220, 0.006211019, 0.007194665])
FT_LB_TO_SI = 1.0  # 전부 같은 단위계(slug·ft²·ft·lbf)로 닫으므로 변환 없음


def rhs(t, y, profile, T):
    q = y[:4]
    w = y[4:]
    M = np.array(FP.moment(profile, t, T))
    wdot = (M - np.cross(w, I_DIAG * w)) / I_DIAG
    # q̇ = ½ q ⊗ (0, ω)
    qw, qx, qy, qz = q
    wx, wy, wz = w
    qdot = 0.5 * np.array([
        -qx * wx - qy * wy - qz * wz,
        qw * wx + qy * wz - qz * wy,
        qw * wy - qx * wz + qz * wx,
        qw * wz + qx * wy - qy * wx,
    ])
    return np.concatenate([qdot, wdot])


def load_hex(path):
    """생성기의 손실 없는 `.hex` 를 읽는다.
    열: t, q(4), ω_i(3), r_i(3), v_i(3), v_ned(3)"""
    rows = []
    for line in open(path):
        v = [float.fromhex(x) for x in line.strip().split(",")]
        rows.append(v)
    return rows


def quat_angle_deg(q1, q2):
    """상대 쿼터니언 + atan2. acos 는 작은 각에서 무너진다."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 + x1 * x2 + y1 * y2 + z1 * z2
    x = w1 * x2 - x1 * w2 - y1 * z2 + z1 * y2
    y = w1 * y2 + x1 * z2 - y1 * w2 - z1 * x2
    z = w1 * z2 - x1 * y2 + y1 * x2 - z1 * w2
    return math.degrees(2.0 * math.atan2(math.sqrt(x * x + y * y + z * z),
                                         abs(w)))


def load_mom(path):
    """JSBSim 이 **실제로 적용한** 외부 모멘트 로그. 매 적분 스텝."""
    out = []
    for line in open(path):
        v = [float.fromhex(x) for x in line.split()]
        out.append((v[0], np.array(v[1:4])))
    return out


def rhs_const(t, y, M):
    """ZOH 구간 안에서는 모멘트가 상수다."""
    w = y[4:]
    wdot = (M - np.cross(w, I_DIAG * w)) / I_DIAG
    qw, qx, qy, qz = y[:4]
    wx, wy, wz = w
    qdot = 0.5 * np.array([
        -qx * wx - qy * wy - qz * wz,
        qw * wx + qy * wz - qz * wy,
        qw * wy - qx * wz + qz * wx,
        qw * wz + qx * wy - qy * wx,
    ])
    return np.concatenate([qdot, wdot])


def run(hex_path, mom_path, dt, rtol=1e-13, atol=1e-15):
    """**동일 입력** 재생. 앞선 판의 세 결함을 고친 것:

      1. Python 이 프로파일을 다시 계산하지 않는다. JSBSim 이 적용한
         모멘트 로그를 그대로 읽는다 — 두 구현이 어긋나도 검출된다.
      2. JSBSim 은 dt 마다 명령을 갱신하고 그 사이를 유지하는 ZOH 다.
         연속 함수 M(t) 를 RHS 마다 부르면 **다른 입력**을 푸는 것이다.
      3. 불연속(계단의 국면 경계, 그리고 ZOH 의 매 스텝 경계)에서
         적분기를 **재시작**한다. rtol 을 줄여도 불연속을 가로지르는
         적분은 낫지 않는다.
    """
    rows = load_hex(hex_path)
    mom = load_mom(mom_path)
    ts = [r[0] for r in rows]
    y = np.concatenate([np.array(rows[0][1:5]) / np.linalg.norm(rows[0][1:5]),
                        np.array(rows[0][5:8])])
    ref = {0.0: y.copy()}
    want = set(round(t, 9) for t in ts)
    for tk, M in mom:
        sol = solve_ivp(rhs_const, (tk, tk + dt), y, method="DOP853",
                        rtol=rtol, atol=atol, args=(M,))
        if not sol.success:
            raise SystemExit(f"FAIL: t={tk} 기준 적분 실패 — {sol.message}")
        y = sol.y[:, -1]
        tn = round(tk + dt, 9)
        if tn in want:
            ref[tn] = y.copy()
    Y = np.array([ref[round(t, 9)] for t in ts]).T
    return rows, type("S", (), {"y": Y})(), mom


def metrics(rows, sol, mom, dt_step, T):
    ang, dw = [], []
    for k in range(len(rows)):
        qj = rows[k][1:5]
        qr = sol.y[:4, k]
        qr = qr / np.linalg.norm(qr)
        ang.append(quat_angle_deg(qj, qr))
        wj = np.array(rows[k][5:8])
        dw.append(np.linalg.norm(wj - sol.y[4:, k]))
    ang = np.array(ang)
    dw = np.array(dw)
    ts = np.array([r[0] for r in rows])

    # 모멘트 변경 직후 excursion — 국면 경계 이후 T/60 창
    seg = T / 6.0
    exc = 0.0
    for b in (seg, 2 * seg, 3 * seg, 4 * seg, 5 * seg):
        m = (ts >= b) & (ts <= b + T / 60.0)
        if m.any():
            exc = max(exc, ang[m].max())

    # 일-에너지 폐쇄 — **실제 적용 모멘트**로 계산한다. 앞선 판은
    # 여기서만 FP.moment() 를 다시 계산해, 고쳤다고 한 결함이 이 지표에
    # 그대로 남아 있었다 (정규화도 반영되지 않았다).
    ke = np.array([0.5 * float(np.dot(I_DIAG * np.array(r[5:8]),
                                      np.array(r[5:8]))) for r in rows])
    # mom 은 매 적분 스텝, rows 는 출력 주기다. 시각으로 맞춘다.
    mmap = {round(t, 9): M for t, M in mom}
    p = []
    for t, r in zip(ts, rows):
        M = mmap.get(round(t, 9))
        if M is None:
            # 마지막 출력 시각 T 에는 적용 구간이 없다 (ZOH 는 [t_k, t_k+dt)
            # 에 상수이고 로그는 k < steps 까지다). 직전 구간 값을 쓴다 —
            # 사다리꼴의 마지막 조각에만 영향을 준다.
            if abs(t - ts[-1]) < 1e-9 and mom:
                M = mom[-1][1]
            else:
                raise SystemExit(f"FAIL: t={t} 의 실제 모멘트 기록 없음")
        p.append(float(np.dot(M, np.array(r[5:8]))))
    p = np.array(p)
    work = np.trapz(p, ts)
    # 분모는 순 일이 아니라 ∫|M·ω|dt 다. 프로파일이 반대칭이라 순 일이
    # 0 에 가까워, |W| 로 나누면 잔차가 작아도 비가 폭발한다.
    scale = np.trapz(np.abs(p), ts)
    closure = abs((ke[-1] - ke[0]) - work) / max(scale, 1e-30)

    # ② 실측 ZOH 임펄스 — 정규화가 실제로 맞았는지 여기서 보고한다.
    zoh_imp = float(sum(np.linalg.norm(M) for _, M in mom) * dt_step)

    return dict(ang_max=ang.max(), ang_rms=float(np.sqrt((ang ** 2).mean())),
                ang_end=ang[-1], dw_max=dw.max(),
                dw_rms=float(np.sqrt((dw ** 2).mean())),
                exc=exc, closure=closure, zoh_impulse=zoh_imp)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("hex_path")
    ap.add_argument("mom_path", help="JSBSim 이 실제 적용한 모멘트 로그")
    ap.add_argument("dt", type=float, help="적분 스텝 (ZOH 구간 길이)")
    ap.add_argument("profile", choices=("step", "c0ramp", "c1ramp", "c2ramp"))
    ap.add_argument("duration", type=float)
    a = ap.parse_args()
    rows, sol, mom = run(a.hex_path, a.mom_path, a.dt)
    m = metrics(rows, sol, mom, a.dt, a.duration)
    print(" ".join(f"{k}={v:.6e}" for k, v in m.items()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
