#!/usr/bin/env python3
"""부록 공급자 검사 — 대수 재폐쇄와 두 경로 동등성. 비생산.

**원문 부록에 수치 예제는 없다.** 그러므로 대조 대상은 예제가 아니라
**대수 항등성**이다.
"""
import math
import sys

from appendix_suppliers import (AppendixASupplier, AppendixBSupplier,
                                assemble_appendix, require_assembly_premises)
from assembler import AeroError, Reference, assemble

fails = []


def check(ok, label):
    print(f"  {'OK  ' if ok else 'FAIL'}  {label}")
    if not ok:
        fails.append(label)


def rel(a, b, tol=1e-12):
    """상대 허용오차. 값이 1e0 급인데 1e-18 절대 허용오차를 쓰면
    부동소수 오차에 걸린다 — 첫 판이 그랬다."""
    return abs(a - b) <= tol * max(abs(a), abs(b), 1e-300)


def refuses(fn, label):
    try:
        fn()
    except AeroError:
        check(True, label)
    else:
        check(False, label + " (거절하지 않았다)")


def main():
    print("부록 시험 공급자 검사\n")

    # ── 부록 A: A.1–A.12 대수 재폐쇄 ────────────────────────────
    print("[A] A.1–A.12 항등식 재폐쇄 (C_fin 은 외부 입력)")
    rho, v_r, q_m = 1.0, 100.0, 0.3
    # 원문 정의대로 S_ref = πD²/4. 임의 조합은 대수가 우연히 닫혀도
    # 출처 충실성 시험이 아니다.
    d_ref = 0.5
    s_ref = math.pi * d_ref ** 2 / 4.0
    c_fin = 1.7

    # A.1 → A.2: 원문 조건은 **3항 대 2항** 이다. Q 전체와 비교하면
    # 훨씬 느슨해져 전제가 깨진 경우도 통과한다 (첫 판이 그랬다).
    #     |D²q²| / |2 V D q| = |D q| / (2V) < 0.01
    ratio_a = abs(d_ref * q_m) / (2 * v_r)
    check(ratio_a < 0.01, f"A.1 → A.2: |Dq|/(2V) = {ratio_a:.3e} < 0.01")
    bad_q = 2 * v_r / d_ref * 0.02          # 비를 0.02 로 만드는 q_m
    check(abs(d_ref * bad_q) / (2 * v_r) >= 0.01,
          "  전제를 깨는 q_m 은 이 조건에 걸린다")

    # B.1 → B.2 도 같은 조건. 원통 전 구간의 **최대 |X|** 에서 본다.
    length, cg = 4.0, 0.4
    x_max = length * max(cg, 1.0 - cg)
    ratio_b = abs(q_m * x_max) / (2 * v_r)
    check(ratio_b < 0.01,
          f"B.1 → B.2: max|X| 에서 |qX|/(2V) = {ratio_b:.3e} < 0.01")
    # 음성 대조군 — 없으면 max|X| 계산 변이가 죽지 않는다.
    q_break = 2 * v_r / x_max * 0.02
    check(abs(q_break * x_max) / (2 * v_r) >= 0.01,
          "  전제를 깨는 q_m 은 B 조건에도 걸린다")
    # cg 를 한쪽 끝으로 몰면 max|X| 가 L 이 되어 조건이 더 빡세진다
    x_end = length * max(0.95, 1.0 - 0.95)
    check(x_end > x_max,
          "  cg 가 끝으로 갈수록 max|X| 가 커진다 (max(cg, 1−cg)·L)")

    # A.5 = A.4 with S_fin = S_ref/4
    q_a2 = 0.5 * rho * (v_r ** 2 + 2 * v_r * d_ref * q_m)   # 식 (A.2)
    s_fin = s_ref / 4.0
    m_a4 = -q_a2 * c_fin * s_fin * d_ref
    m_a5 = -0.25 * q_a2 * c_fin * s_ref * d_ref
    check(rel(m_a4, m_a5), "A.4 = A.5 (S_fin = S_ref/4 대입)")

    # A.6 → A.7: Q_fin 자리에 Q_d
    q_d = rho * v_r * d_ref * q_m
    m_a7_src = -0.25 * rho * v_r * q_m * c_fin * s_ref * d_ref ** 2
    m_a7_sub = -0.25 * q_d * c_fin * s_ref * d_ref
    check(rel(m_a7_src, m_a7_sub),
          "A.7 원문형 = A.6 을 A.5 에 대입한 동치식")

    # A.8 + A.9 재폐쇄: M_d 두 경로가 같은가
    c_mmd_a9 = -(q_m * d_ref / (2 * v_r)) * c_fin
    m_a8 = 0.5 * rho * v_r ** 2 * s_ref * d_ref * c_mmd_a9
    check(rel(m_a8, m_a7_src),
          "A.8(C_mmd 경로) = A.7(직접 경로) — 대수 재폐쇄")

    # A.11 · A.12 ⇒ C_mqm = −C_fin, 그리고 식 (8) 과 일관
    sup_a = AppendixASupplier(c_fin)
    # **교차류 전제 안에서** 호출한다. 첫 판은 α = 0 (축방향)으로 불렀다.
    dmp = sup_a.damping(math.pi / 2, 0.0, 0.3)
    check(rel(dmp.c_mq, -c_fin), "A.11·A.12 ⇒ C_mqm = −C_fin")
    c_mmd_eq8 = (q_m * d_ref / (2 * v_r)) * dmp.c_mq
    check(rel(c_mmd_eq8, c_mmd_a9),
          "  식 (8) 로 되돌린 C_mmd 가 A.9 와 일치")

    # ── 부록 B: 두 경로 동등성 ─────────────────────────────────
    print("\n[B] B.6 직접 M_d 와 파생 C_mqm → 조립기 경로")
    c_cf = 1.2
    sup_b = AppendixBSupplier(c_cf, length, cg, d_ref, s_ref)
    ref = Reference(s_ref=s_ref, d_ref=d_ref, mrp_b=(0.0, 0.0, 0.0))

    ok_all, worst = True, 0.0
    inv = []
    # 세 번째 사례는 첫 판에서 |qX|/(2V) = 0.0165 로 **전제 밖**이었다.
    # 두 경로가 같은 근사식에서 출발하므로 전제 밖에서도 대수적으로
    # 일치한다 — 일치만으로는 전제 준수의 증거가 되지 않는다.
    for rho_, v_, q_ in ((1.0, 100.0, 0.3), (0.4, 250.0, -0.7),
                         (1.225, 80.0, 0.5)):
        direct = sup_b.moment_direct(rho_, v_, q_)
        q_bar = 0.5 * rho_ * v_ ** 2          # 독립 입력
        # **교차류**: V⃗_R 을 +Z 로 준다 ⇒ α_tot = 90°, φ_A = 0.
        _, m_b = assemble_appendix((0.0, 0.0, v_), (0.0, q_, 0.0),
                                   q_bar, 340.0, ref, sup_b)
        e = abs(m_b[1] - direct) / max(abs(direct), 1e-300)
        worst = max(worst, e)
        ok_all &= e <= 1e-12
        # 각 상태의 **직접 모멘트에서 C_mqm 을 역산**한다.
        # C_mqm = 4 M_d / (ρ V_R q_m S_ref D_ref²)
        inv.append(4.0 * direct / (rho_ * v_ * q_ * s_ref * d_ref ** 2))
    check(ok_all, f"세 (ρ, V_R, q_m) 조합에서 두 경로 일치 "
                  f"(최대 상대차 {worst:.2e} ≤ 1e-12)")

    # "상태 무관" 을 동어반복이 아니라 **역산 값의 일치**로 검사한다.
    check(all(rel(x, inv[0]) for x in inv) and rel(inv[0], sup_b.c_mqm()),
          "직접 모멘트에서 역산한 C_mqm 이 세 상태에서 동일하고 파생식과 일치")

    # 잘못된 Q 를 넣으면 깨져야 한다
    _, m_badq = assemble_appendix((0.0, 0.0, 100.0), (0.0, 0.3, 0.0),
                                  0.5 * 1.0 * 100.0 ** 2 * 1.5, 340.0,
                                  ref, sup_b)
    check(abs(m_badq[1] - sup_b.moment_direct(1.0, 100.0, 0.3)) > 1e-12,
          "Q 를 1.5배로 틀리게 주면 두 경로가 갈린다")

    # 무차원화 정의를 **독립 입력**으로 검사
    print("\n[B] 정의를 하나씩 틀리게 넣으면 동등성이 깨지는가")
    d2 = d_ref * 2.0
    d_bad = AppendixBSupplier(c_cf, length, cg, d2,
                              math.pi * d2 ** 2 / 4.0)
    m_dir = d_bad.moment_direct(1.0, 100.0, 0.3)
    # D 불일치는 **래퍼가 잡는다** — 그것이 검사의 요점이다.
    refuses(lambda: assemble_appendix((0.0, 0.0, 100.0), (0.0, 0.3, 0.0),
                                      0.5 * 1.0 * 1e4, 340.0, ref, d_bad),
            "D ≠ D_ref 이면 래퍼가 거절 (D = D_ref 전제가 실재한다)")
    _, m_asm = assemble((0.0, 0.0, 100.0), (0.0, 0.3, 0.0), 0.5 * 1.0 * 1e4,
                        340.0, ref, d_bad)   # raw 로는 갈리는 것만 확인
    check(abs(m_asm[1] - m_dir) > 1e-12,
          "D ≠ D_ref 이면 두 경로가 갈린다 (D = D_ref 전제가 실재한다)")

    cg_sym = AppendixBSupplier(c_cf, length, 0.5, d_ref, s_ref)
    val = 1.0 - 3 * 0.5 + 3 * 0.25
    check(abs(cg_sym.c_mqm() / sup_b.c_mqm()
              - val / (1 - 3 * cg + 3 * cg ** 2)) < 1e-12,
          "cg 무차원화가 (1 − 3cg + 3cg²) 로만 들어간다")
    check(cg_sym.c_mqm() < 0, "  중앙 CG 에서도 감쇠는 음수")

    # ── fail-closed ────────────────────────────────────────────
    print("\n[fail-closed]")
    refuses(lambda: AppendixASupplier(float('nan')), "C_fin NaN 거절")
    S = lambda d: math.pi * d ** 2 / 4.0
    refuses(lambda: AppendixBSupplier(1.0, -1.0, 0.5, 0.5, S(0.5)),
            "L <= 0 거절")
    refuses(lambda: AppendixBSupplier(-1.0, 4.0, 0.5, 0.5, S(0.5)),
            "교차류 항력계수 C <= 0 거절")
    refuses(lambda: AppendixBSupplier(1.0, 4.0, 1.5, 0.5, S(0.5)),
            "cg 가 [0,1] 밖이면 거절")
    refuses(lambda: AppendixBSupplier(1.0, 4.0, 0.5, 0.0, 0.0), "D <= 0 거절")
    refuses(lambda: AppendixBSupplier(1.0, 4.0, 0.5, 0.5, 2.0),
            "S_ref ≠ πD²/4 거절 (원통 형상 불일치)")
    refuses(lambda: sup_a.damping(0.0, 0.0, 0.3),
            "부록 A 를 축방향 흐름(α=0)으로 부르면 거절")
    refuses(lambda: sup_b.damping(0.0, 0.0, 0.3),
            "부록 B 를 축방향 흐름으로 부르면 거절")
    refuses(lambda: assemble((100.0, 0.0, 0.0), (0, 0, 0), 10.0, 340.0,
                             ref, sup_b),
            "  조립기를 통해도 비교차류면 거절")
    refuses(lambda: sup_b.damping(math.pi / 2, math.pi / 2, 0.3),
            "α=90° 라도 φ_A ≠ 0 이면 거절 (M·P·B 일치 전제)")
    refuses(lambda: assemble((0.0, 100.0, 0.0), (0.0, 0.3, 0.0),
                             0.5 * 1e4, 340.0, ref, sup_b),
            "  조립기 경로에서도 φ_A = 90° 는 거절")
    for bad in (math.pi / 2, math.pi, float('nan'), -1.0):
        refuses(lambda b=bad: AppendixBSupplier(1.0, 4.0, 0.5, d_ref, s_ref,
                                                crossflow_tol_rad=b),
                f"crossflow_tol_rad = {bad!r} 거절")
    refuses(lambda: require_assembly_premises(
        Reference(s_ref, d_ref, (0.1, 0.0, 0.0)), sup_b),
        "MRP ≠ 무게중심이면 거절")
    refuses(lambda: assemble_appendix(
        (0.0, 0.0, 100.0), (0.0, 0.3, 0.0), 0.5 * 1e4, 340.0,
        Reference(s_ref, d_ref, (1.0, 0.0, 0.0)), sup_b),
        "  래퍼가 MRP 어긋남을 **조립 경로에서** 잡는다")
    refuses(lambda: assemble_appendix(
        (0.0, 0.0, 80.0), (0.0, 1.1, 0.0), 0.5 * 1.225 * 6400, 340.0,
        ref, sup_b),
        "  래퍼가 폐기항 조건 위반을 행마다 잡는다 (|qX|/2V = 0.0165)")
    check(sup_b.truncation_ratio(100.0, 0.3) < 0.01
          and sup_b.truncation_ratio(80.0, 1.1) >= 0.01,
          "truncation_ratio 가 통과·위반을 실제로 가른다")
    # **먼 쪽 끝**이 지배한다: max(cg, 1−cg) 를 cg 로 바꾸면 위반이
    # 통과로 뒤집히는 경계 사례. cg = 0.4 이므로 0.6·L 대 0.4·L.
    check(sup_b.truncation_ratio(80.0, 0.7) >= 0.01,
          "cg=0.4 에서 q=0.7 은 먼 쪽 끝(0.6L) 기준으로 위반")
    check(abs(0.7) * length * cg / (2 * 80.0) < 0.01,
          "  가까운 쪽 끝(0.4L) 로 재면 통과해버린다 — max() 가 필요한 이유")
    refuses(lambda: assemble_appendix(
        (0.0, 0.0, 80.0), (0.0, 0.7, 0.0), 0.5 * 1.225 * 6400, 340.0,
        ref, sup_b), "  래퍼도 그 경계 사례를 거절")
    huge = AppendixBSupplier(1e300, 1e100, 0.4, d_ref, s_ref)
    refuses(lambda: huge.c_mqm(), "c_mqm 곱 오버플로 거절")
    refuses(lambda: require_assembly_premises(
        Reference(math.pi * 1.0 ** 2 / 4, 1.0, (0, 0, 0)), sup_b),
        "공급자와 Reference 의 D/S 가 다르면 거절")
    check(require_assembly_premises(ref, sup_b) is None,
          "일치하는 Reference 는 통과")
    refuses(lambda: sup_b.moment_direct(-1.0, 100.0, 0.3), "음수 밀도 거절")
    refuses(lambda: sup_b.moment_direct(1.0, 0.0, 0.3), "V_R <= 0 거절")
    big = AppendixBSupplier(1e300, 1e100, 0.4, d_ref, s_ref)
    refuses(lambda: big.moment_direct(1e300, 1e300, 1e300),
            "유한 입력의 곱 오버플로 거절")
    refuses(lambda: sup_b.moment_direct(float('inf'), 100.0, 0.3),
            "비유한 ρ 거절")

    print()
    if fails:
        print(f"FAIL: {len(fails)}건", file=sys.stderr)
        return 1
    print("OK: 전 항목 통과")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
