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

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import forced_profile as FP

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


def run(path, profile, T, rtol=1e-13, atol=1e-15):
    rows = load_hex(path)
    ts = [r[0] for r in rows]
    q0 = np.array(rows[0][1:5])
    w0 = np.array(rows[0][5:8])          # rad/s (JSBSim 내부 단위)
    q0 = q0 / np.linalg.norm(q0)
    sol = solve_ivp(rhs, (ts[0], ts[-1]), np.concatenate([q0, w0]),
                    method="DOP853", t_eval=ts, rtol=rtol, atol=atol,
                    args=(profile, T))
    if not sol.success:
        raise SystemExit(f"FAIL: 기준 적분 실패 — {sol.message}")
    return rows, sol


def metrics(rows, sol, profile, T):
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

    # 일-에너지 폐쇄: ΔKE − ∫ M·ω dt (JSBSim 궤적으로)
    ke = np.array([0.5 * float(np.dot(I_DIAG * np.array(r[5:8]),
                                      np.array(r[5:8]))) for r in rows])
    p = np.array([float(np.dot(np.array(FP.moment(profile, t, T)),
                               np.array(r[5:8]))) for t, r in zip(ts, rows)])
    work = np.trapz(p, ts)
    # 분모는 순 일이 아니라 ∫|M·ω|dt 다. 프로파일이 반대칭이라 순 일이
    # 0 에 가까워, |W| 로 나누면 잔차가 작아도 비가 폭발한다 (측정:
    # T=1 c2ramp 에서 순 일 −1.9e-06, 잔차 3.4e-06 → 비 1.73).
    scale = np.trapz(np.abs(p), ts)
    closure = abs((ke[-1] - ke[0]) - work) / max(scale, 1e-30)

    return dict(ang_max=ang.max(), ang_rms=float(np.sqrt((ang ** 2).mean())),
                ang_end=ang[-1], dw_max=dw.max(),
                dw_rms=float(np.sqrt((dw ** 2).mean())),
                exc=exc, closure=closure)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("hex_path")
    ap.add_argument("profile", choices=("step", "c2ramp"))
    ap.add_argument("duration", type=float)
    a = ap.parse_args()
    rows, sol = run(a.hex_path, a.profile, a.duration)
    m = metrics(rows, sol, a.profile, a.duration)
    print(" ".join(f"{k}={v:.6e}" for k, v in m.items()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
