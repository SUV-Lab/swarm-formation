#!/usr/bin/env python3
"""ADR-0003 §3 후보 프로파일 점검 — 문서의 모든 표를 재생성한다.

생산 코드가 아니다. COLCON_IGNORE로 빌드에서 제외돼 있다.

이 실험이 말할 수 있는 정확한 범위
----------------------------------
    ADR-0002 자리표시 모델에서, 추력을 속도 방향으로 고정하고, 지정된
    일정 비행경로각 변화율을 적용한 후보 프로파일이 단순 CL·속력 조건을
    위반하는지 검사한다.

그 이상은 말할 수 없다. 특히 **위반이 불가능의 증거가 아니다.**

왜 "실패 = 불가능"이 성립하지 않는가
------------------------------------
6DOF에서는 몸체 방향과 속도 방향이 다를 수 있으므로 추력에 법선 성분이
생긴다. 선회 방정식의 정확한 형태는

    m V γ̇ = L + T⊥ − m g cos γ

인데 이 스크립트는 **T⊥ = 0으로 고정한다.** 이것은 문제를 느슨하게 만든
것이 아니라 **사용 가능한 제어 입력을 하나 제거한 것이다.** 게다가
비행경로각 변화율을 상수 하나로 묶어 가변 프로파일을 배제한다.

따라서 여기서 조건을 위반한 후보가 있어도, 다른 자세·추력 방향·가변
프로파일이 성립할 수 있다. **초기 상태나 6DOF 비행체 자체를 불가능하다고
판정하는 데 이 결과를 쓰면 안 된다.**

γ̇ 의 정체
----------
γ̇ 는 **비행경로각 변화율**이지 자세 변화율이 아니다. 자세 변화율은
쿼터니언 미분 또는 몸체 각속도 p, q, r 로 표현해야 하며 이 스크립트에는
없다.

방정식 (SI, 바람축, 뱅크 0, T⊥ = 0)
-----------------------------------
    V̇   = T/m − g sin γ − D/m
    γ̇   = −r                          (지정된 상수)
    L    = m (V γ̇ + g cos γ)          from  γ̇ = L/(mV) − g cos γ/V
    CL   = L / (q S),   q = ½ ρ V²
    D    = q S (CD0 + k CL²)

지배 변수군:

    CL_req = (2 (m/S) / ρ) · ( γ̇ / V + g cos γ / V² )

면적하중 m/S, 밀도 ρ, 순간 속력 V, 비행경로각 변화율 γ̇, 중력 투영 cos γ.

명시한 가정
-----------
    중력      g = 9.81 일정, 평면 지구
    대기      ρ 일정 (구간 내 고도 변화 무시)
    추력      속도 벡터에 평행, 크기 일정   ← 제어 입력을 제거하는 제약
    항력      포물선 극선 CD = CD0 + k CL²
    뱅크      0 — 평면 내 기동
    적분      명시적 오일러, 고정 스텝 (수렴은 --check)

속력 하한 2단 (생산 코드 계약)
------------------------------
    전환 중   model_activation_speed_mps = 40.0
              (설정 dynamics_activation_speed_mps, transition_phase.cpp:392)
    인계 시   margin-backed 중기 하한 = 122.0 × (1 + 0.08) = 131.76
              (path_manager.h cruiseFloorUnits 주석)

전환 궤적은 중기 하한 아래로 잠시 내려갈 수 있다. 전환 도중 중기 하한을
적용하면 계약에 없는 조건으로 후보를 떨어뜨리게 된다.
"""
import argparse
import math
import sys

# ADR-0002 중립 자리표시 모델. S = 1 m² 는 자리표시자이지 설계값이 아니다.
BASE = dict(m=1300.0, S=1.0, CD0=0.035, k=0.080, CLmax=1.40,
            g=9.81, rho=1.0,
            V_transit_floor=40.0,      # 전환 중 — 모델 활성 하한
            V_handoff_floor=131.76)    # 인계 시 — margin-backed 중기 하한

VIOL_CL = "단순 조건 위반 (CL)"
VIOL_TRANSIT = "단순 조건 위반 (전환 중 활성 하한)"
VIOL_HANDOFF = "단순 조건 위반 (인계 속력 하한)"
NO_VIOL = "단순 조건 위반 없음"


def run(V0, TW, rate_deg_s, gam_end_deg=20.0, dt=0.01, p=None):
    """후보 프로파일 하나를 적분한다.

    반환: (V_end|None, t, CL_peak, 결과)
    결과는 '이 후보 프로파일이 단순 조건을 위반했는가'만 말한다.
    """
    p = dict(BASE, **(p or {}))
    m, S, CD0, k = p['m'], p['S'], p['CD0'], p['k']
    g, rho, CLmax = p['g'], p['rho'], p['CLmax']
    v_tr, v_ho = p['V_transit_floor'], p['V_handoff_floor']
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
        # 전환 도중에는 활성 하한만 적용한다. 중기 하한은 인계 시점의 것.
        if V < v_tr:
            return None, t, cl_peak, VIOL_TRANSIT
    if cl_peak > CLmax:
        return V, t, cl_peak, VIOL_CL
    if V < v_ho:
        return V, t, cl_peak, VIOL_HANDOFF
    return V, t, cl_peak, NO_VIOL


CASES_A = [(V0, TW, rate)
           for V0 in (190.0, 230.0)
           for TW in (0.25, 1.00)
           for rate in (8.6, 4.0)]

CASES_B = [(S, V0) for S in (1.0, 2.0, 4.0) for V0 in (190.0, 230.0)]


def table_a(dt=0.01):
    return [(V0, TW, rate) + run(V0, TW, rate, dt=dt)
            for V0, TW, rate in CASES_A]


def table_b(dt=0.01):
    """면적하중 민감도. 설계안이 아니다 — 스팬·시위·관성·모멘트 계수를
    S와 함께 바꾸지 않으므로 각 행은 일관된 하나의 비행체가 아니다."""
    out = []
    for S, V0 in CASES_B:
        V, t, cl, res = run(V0, 0.25, 8.6, dt=dt, p=dict(S=S))
        out.append((S, BASE['m'] / S, V0, cl, V, res))
    return out


DTS = (0.02, 0.01, 0.005, 0.0025, 0.001)
CL_TOL = 1e-4          # 최미세 두 스텝 사이 CL 상대차 허용


def convergence():
    """모든 표 행에 대해 dt를 세분화한다. 3개 표본이 아니라 전수."""
    rows = []
    for V0, TW, rate in CASES_A:
        rows.append((f"A V0={V0:.0f} T/W={TW:.2f} rate={rate}",
                     [(dt,) + run(V0, TW, rate, dt=dt) for dt in DTS]))
    for S, V0 in CASES_B:
        rows.append((f"B S={S:.1f} V0={V0:.0f}",
                     [(dt,) + run(V0, 0.25, 8.6, dt=dt, p=dict(S=S))
                      for dt in DTS]))
    return rows


def fmt_v(V):
    return "—" if V is None else f"{V:.1f}"


def check():
    """판정이 dt에 불변이고 CL이 수렴하는지 — 출력이 아니라 실패 조건."""
    bad = []
    for label, row in convergence():
        finest = row[-1][4]
        for dt, V, t, cl, res in row:
            if res != finest:
                bad.append(f"{label}: dt={dt}에서 판정 '{res}' "
                           f"(최미세 '{finest}')")
        cl_c, cl_f = row[-2][3], row[-1][3]
        rel = abs(cl_c - cl_f) / max(cl_f, 1e-9)
        if rel > CL_TOL:
            bad.append(f"{label}: CL 상대차 {rel*100:.4f}% > "
                       f"{CL_TOL*100:.4f}%")
        print(f"  {label:<34} CL={cl_f:7.3f}  상대차={rel*100:.5f}%  {finest}")
    if bad:
        print(f"\nFAIL: 수렴 검사 {len(bad)}건", file=sys.stderr)
        for b in bad:
            print("  " + b, file=sys.stderr)
        return 1
    print(f"\nOK: 표 A·B 전 {len(CASES_A)+len(CASES_B)}행이 dt "
          f"{DTS[0]}–{DTS[-1]}에서 판정 불변, CL 상대차 < {CL_TOL*100:.4f}%")
    return 0


def verify_doc(path):
    """문서에 적힌 모든 숫자와 판정 칸이 이 실행과 같은지 대조한다.
    갈라지면 그 절은 측정이 아니라 계산 메모이므로 실패시킨다."""
    import re
    text = open(path, encoding='utf-8').read()
    bad, checked = [], 0

    def cells(line):
        return [c.strip().replace('**', '') for c in line.split('|')]

    def num(s):
        s = s.strip()
        return None if s in ('—', '-') else float(s.rstrip('s'))

    def cmp_(label, doc, code, tol):
        if doc is None and code is None:
            return
        if doc is None or code is None:
            bad.append(f"{label}: 문서 {doc} vs 코드 {code}")
        elif abs(doc - code) > tol:
            bad.append(f"{label}: 문서 {doc} vs 코드 {code:.2f}")

    for V0, TW, rate, V, t, cl, res in table_a():
        m = re.search(rf"^\|\s*{V0:.0f}\s*\|\s*{TW:.2f}\s*\|\s*{rate}\s*"
                      r"(?:°/s)?\s*\|(.*)$", text, re.M)
        if not m:
            bad.append(f"표 A 행 없음: V0={V0:.0f} T/W={TW:.2f} rate={rate}")
            continue
        checked += 1
        c = cells(m.group(1))
        tag = f"표 A[{V0:.0f},{TW:.2f},{rate}]"
        cmp_(f"{tag} 소요", num(c[0]), t, 0.05)
        cmp_(f"{tag} 종료V", num(c[1]), V, 0.05)
        cmp_(f"{tag} CL", num(c[2]), cl, 0.005)
        if c[3] != res:
            bad.append(f"{tag} 결과: 문서 '{c[3]}' vs 코드 '{res}'")

    for S, wl, V0, cl, V, res in table_b():
        m = re.search(rf"^\|\s*\**{S:.1f}\**\s*\|\s*\**{wl:.0f}\**\s*\|"
                      rf"\s*{V0:.0f}\s*\|(.*)$", text, re.M)
        if not m:
            bad.append(f"표 B 행 없음: S={S:.1f} V0={V0:.0f}")
            continue
        checked += 1
        c = cells(m.group(1))
        tag = f"표 B[S={S:.1f},{V0:.0f}]"
        cmp_(f"{tag} CL", num(c[0]), cl, 0.005)
        cmp_(f"{tag} 종료V", num(c[1]), V, 0.05)
        if c[2] != res:
            bad.append(f"{tag} 결과: 문서 '{c[2]}' vs 코드 '{res}'")

    if bad:
        print(f"FAIL: 문서와 코드가 어긋남 ({len(bad)}건)", file=sys.stderr)
        for b in bad:
            print("  " + b, file=sys.stderr)
        return 1
    print(f"OK: 문서의 표 {checked}행이 모든 칸에서 이 실행과 일치")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--check', action='store_true',
                    help='전 표 행의 dt 수렴 — 어긋나면 종료코드 1')
    ap.add_argument('--verify-doc', metavar='ADR',
                    help='문서의 모든 표 칸 대조 — 어긋나면 종료코드 1')
    a = ap.parse_args()

    if a.verify_doc:
        return verify_doc(a.verify_doc)
    if a.check:
        print("dt 수렴 — 표 A·B 전수")
        return check()

    print("ADR-0003 §3 표 A — 초기속도 · 추력하중 · 비행경로각 변화율 "
          "(dt=0.01)")
    print(f"{'V0':>5} {'T/W':>5} {'γ̇':>8} {'소요':>7} {'종료V':>7} "
          f"{'CL peak':>8}  결과")
    for V0, TW, rate, V, t, cl, res in table_a():
        print(f"{V0:5.0f} {TW:5.2f} {rate:6.1f}°/s {t:6.1f}s "
              f"{fmt_v(V):>7} {cl:8.2f}  {res}")

    print()
    print("ADR-0003 §3 표 B — 면적하중 민감도 (T/W=0.25, 8.6°/s, dt=0.01)")
    print("  경고: S만 바꾸고 스팬·시위·관성·모멘트 계수를 그대로 둔다.")
    print("        각 행은 일관된 비행체가 아니라 민감도 지점이다.")
    print(f"{'S(m²)':>6} {'m/S':>7} {'V0':>5} {'CL peak':>8} {'종료V':>7}"
          f"  결과")
    for S, wl, V0, cl, V, res in table_b():
        print(f"{S:6.1f} {wl:7.0f} {V0:5.0f} {cl:8.2f} {fmt_v(V):>7}  {res}")

    print()
    print("결과 칸은 '이 후보 프로파일이 단순 조건을 위반했는가'만 말한다.")
    print("추력 법선 성분을 0으로 고정하고 상수 변화율 하나만 검사하므로,")
    print("위반은 초기 상태나 6DOF 비행체의 불가능을 뜻하지 않는다.")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
