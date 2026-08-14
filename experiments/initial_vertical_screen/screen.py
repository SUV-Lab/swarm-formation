#!/usr/bin/env python3
"""ADR-0003 §3 point-mass screen — regenerates every table in the document.

NOT PRODUCTION CODE. COLCON_IGNORE keeps it out of the build.

WHAT THIS IS
------------
A point-mass screen over the attitude-change segment: velocity starts
exactly vertical with non-zero speed, the flight path angle is walked down
to a target, and we ask whether the lift coefficient the turn demands stays
inside CLmax while speed stays above the cruise floor.

WHAT THIS IS NOT
----------------
It carries no moments, no inertia, no control authority and no actuator
state. A case that survives here has NOT been shown to be flyable -- it has
only failed to be excluded. A case that dies here is genuinely excluded,
because every neglected effect makes the demand harder, never easier.

Read the outcome column exactly as written.

EQUATIONS (SI, wind axes, bank = 0)
-----------------------------------
    Vdot     = T/m - g sin(gam) - D/m
    gamdot   = -r                       (commanded, constant magnitude)
    L        = m (V gamdot + g cos(gam))    from  gamdot = L/(mV) - g cos(gam)/V
    CL       = L / (q S),   q = 0.5 rho V^2
    D        = q S (CD0 + k CL^2)

ASSUMPTIONS, STATED
-------------------
  gravity   constant g = 9.81 m/s^2, flat earth
  atmosphere  constant rho (no altitude variation over the segment)
  thrust    aligned with the velocity vector, constant magnitude
  drag      parabolic polar, CD = CD0 + k CL^2
  bank      zero -- planar pull-down, no heading change
  integrator  explicit Euler, fixed step (convergence checked below)

Every one of these is optimistic-to-neutral for the screen except constant
rho, which is neutral over a short segment at low altitude.

GOVERNING GROUP
---------------
Substituting L and q:

    CL_req = (2 (m/S) / rho) * ( gamdot / V  +  g cos(gam) / V^2 )

so the demand is set by WING LOADING m/S, DENSITY rho, INSTANTANEOUS SPEED
V, the commanded RATE gamdot, and the gravity projection cos(gam). Thrust
enters only through V(t). "Initial speed x rate x area" is not the group --
area appears as m/S, and V appears twice with different powers.
"""
import argparse
import math
import sys

# ADR-0002 neutral baseline. S = 1 m^2 is a PLACEHOLDER, not a design value.
BASE = dict(m=1300.0, S=1.0, CD0=0.035, k=0.080, CLmax=1.40,
            g=9.81, Vfloor=122.0, rho=1.0)

EXCLUDED_CL = "점질량 선별에서 배제 (CL 초과)"
EXCLUDED_V = "점질량 선별에서 배제 (속도하한 미달)"
NOT_EXCLUDED = "이 선별에서 배제되지 않음"


def run(V0, TW, rate_deg_s, gam_end_deg=20.0, dt=0.01, p=None):
    """Integrate the pull-down. Returns (V_end|None, t, CL_peak, outcome)."""
    p = dict(BASE, **(p or {}))
    m, S, CD0, k = p['m'], p['S'], p['CD0'], p['k']
    g, rho, CLmax, Vfloor = p['g'], p['rho'], p['CLmax'], p['Vfloor']
    T = TW * m * g
    V, gam = V0, math.radians(90.0)
    gam_end, r = math.radians(gam_end_deg), math.radians(rate_deg_s)
    t, cl_peak = 0.0, 0.0
    while gam > gam_end:
        if t > 600.0:
            return None, t, cl_peak, "시간 초과"
        q = 0.5 * rho * V * V
        L = m * (V * (-r) + g * math.cos(gam))
        CL = L / (q * S) if q > 1e-9 else float('inf')
        cl_peak = max(cl_peak, abs(CL))
        D = q * S * (CD0 + k * CL * CL)
        V += (T / m - g * math.sin(gam) - D / m) * dt
        gam -= r * dt
        t += dt
        if V < Vfloor:
            return None, t, cl_peak, EXCLUDED_V
    if cl_peak > CLmax:
        return V, t, cl_peak, EXCLUDED_CL
    return V, t, cl_peak, NOT_EXCLUDED


def table_a(dt=0.01):
    """ADR-0003 §3 table A: initial speed x thrust-to-weight x rate."""
    rows = []
    for V0 in (190.0, 230.0):
        for TW in (0.25, 1.00):
            for rate in (8.6, 4.0):
                V, t, cl, out = run(V0, TW, rate, dt=dt)
                rows.append((V0, TW, rate, t, V, cl, out))
    return rows


def table_b(dt=0.01):
    """ADR-0003 §3 table B: wing-loading sensitivity. NOT a design sweep --
    span, chord, inertia and moment coefficients are held at values that do
    not follow S, so no row here is a self-consistent vehicle."""
    rows = []
    for S in (1.0, 2.0, 4.0):
        for V0 in (190.0, 230.0):
            V, t, cl, out = run(V0, 0.25, 8.6, dt=dt, p=dict(S=S))
            rows.append((S, BASE['m'] / S, V0, cl, V, out))
    return rows


def convergence():
    """dt refinement. The screen's verdicts must not move with step size."""
    cases = [(190.0, 0.25, 8.6), (230.0, 0.25, 4.0), (190.0, 1.00, 4.0)]
    out = []
    for V0, TW, rate in cases:
        row = []
        for dt in (0.02, 0.01, 0.005, 0.0025, 0.001):
            V, t, cl, verdict = run(V0, TW, rate, dt=dt)
            row.append((dt, V, cl, verdict))
        out.append(((V0, TW, rate), row))
    return out


def verify_doc(path):
    """The ADR quotes these numbers. If the document and the code disagree,
    the document is a memo, not a measurement -- so this fails loudly."""
    import re
    text = open(path, encoding='utf-8').read()
    bad, checked = [], 0

    def cell(row, idx):
        return row[idx].strip().replace('**', '')

    for V0, TW, rate, t, V, cl, out in table_a():
        pat = (rf"^\|\s*{V0:.0f}\s*\|\s*{TW:.2f}\s*\|\s*{rate}\s*"
               r"(?:°/s)?\s*\|(.*)$")
        m = re.search(pat, text, re.M)
        if not m:
            bad.append(f"표 A 행 없음: V0={V0:.0f} T/W={TW:.2f} rate={rate}")
            continue
        checked += 1
        cols = m.group(1).split('|')
        doc_cl = float(cell(cols, 2))
        if abs(doc_cl - cl) > 0.005:
            bad.append(f"표 A CL 불일치 V0={V0:.0f} T/W={TW:.2f} "
                       f"rate={rate}: 문서 {doc_cl} vs 코드 {cl:.2f}")
        doc_out = cell(cols, 3)
        key = ("배제되지 않음" if out == NOT_EXCLUDED else "배제")
        if key not in doc_out:
            bad.append(f"표 A 판정 불일치 V0={V0:.0f} T/W={TW:.2f} "
                       f"rate={rate}: 문서 '{doc_out}'")

    for S, wl, V0, cl, V, out in table_b():
        pat = rf"^\|\s*\**{S:.1f}\**\s*\|\s*{wl:.0f}\s*\|\s*{V0:.0f}\s*\|(.*)$"
        m = re.search(pat, text, re.M)
        if not m:
            bad.append(f"표 B 행 없음: S={S:.1f} V0={V0:.0f}")
            continue
        checked += 1
        cols = m.group(1).split('|')
        doc_cl = float(cell(cols, 0))
        if abs(doc_cl - cl) > 0.005:
            bad.append(f"표 B CL 불일치 S={S:.1f} V0={V0:.0f}: "
                       f"문서 {doc_cl} vs 코드 {cl:.2f}")

    if bad:
        print(f"FAIL: 문서와 코드가 어긋남 ({len(bad)}건)", file=sys.stderr)
        for b in bad:
            print("  " + b, file=sys.stderr)
        return 1
    print(f"OK: 문서의 표 {checked}행이 이 실행과 일치")
    return 0


def fmt_v(V):
    return "—" if V is None else f"{V:.1f}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--check', action='store_true',
                    help='verdict stability under dt refinement; exit 1 on drift')
    ap.add_argument('--verify-doc', metavar='ADR',
                    help='assert the ADR tables match this run; exit 1 on drift')
    a = ap.parse_args()

    if a.verify_doc:
        return verify_doc(a.verify_doc)

    print("ADR-0003 §3 표 A — 초기속도 · 추력하중 · 자세 변화율 (dt=0.01)")
    print(f"{'V0':>5} {'T/W':>5} {'변화율':>8} {'소요':>7} {'종료V':>7} {'CL peak':>8}  결과")
    for V0, TW, rate, t, V, cl, out in table_a():
        print(f"{V0:5.0f} {TW:5.2f} {rate:6.1f}°/s {t:6.1f}s {fmt_v(V):>7} {cl:8.2f}  {out}")

    print()
    print("ADR-0003 §3 표 B — 면적하중 민감도 (T/W=0.25, 8.6°/s, dt=0.01)")
    print("  경고: S만 바꾸고 스팬·시위·관성·모멘트 계수를 그대로 둔다.")
    print("        각 행은 일관된 하나의 비행체가 아니라 민감도 지점이다.")
    print(f"{'S(m²)':>6} {'m/S':>7} {'V0':>5} {'CL peak':>8} {'종료V':>7}  결과")
    for S, wl, V0, cl, V, out in table_b():
        print(f"{S:6.1f} {wl:7.0f} {V0:5.0f} {cl:8.2f} {fmt_v(V):>7}  {out}")

    print()
    print("dt 수렴 — 판정이 시간 간격에 따라 바뀌면 이 선별은 무효다")
    drift = 0
    for (V0, TW, rate), row in convergence():
        print(f"  case V0={V0:.0f} T/W={TW:.2f} rate={rate}°/s")
        base_verdict = row[-1][3]
        for dt, V, cl, verdict in row:
            flag = "" if verdict == base_verdict else "   <-- 판정 변동"
            if verdict != base_verdict:
                drift += 1
            print(f"    dt={dt:<7} CL={cl:6.3f} V={fmt_v(V):>7}  {verdict}{flag}")
        # quantitative spread at the two finest steps
        cl_c, cl_f = row[-2][2], row[-1][2]
        print(f"    CL 상대차 (dt 0.0025 대 0.001): {abs(cl_c-cl_f)/max(cl_f,1e-9)*100:.3f}%")

    if a.check:
        if drift:
            print(f"\nFAIL: {drift}건의 판정이 dt에 따라 변동", file=sys.stderr)
            return 1
        print("\nOK: 모든 판정이 dt 0.02–0.001에서 불변")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
