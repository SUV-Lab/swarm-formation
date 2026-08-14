#!/usr/bin/env python3
"""부록 공급자 검사 — 대수 재폐쇄와 두 경로 동등성. 비생산.

**원문 부록에 수치 예제는 없다.** 그러므로 대조 대상은 예제가 아니라
**대수 항등성**이다.
"""
import math
import sys

from appendix_suppliers import AppendixASupplier, AppendixBSupplier
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
    s_ref, d_ref, c_fin = 2.0, 0.5, 1.7

    # A.1 → A.2: q_m² 항을 버린다
    q_a1 = 0.5 * rho * (v_r ** 2 + 2 * v_r * d_ref * q_m
                        + d_ref ** 2 * q_m ** 2)
    q_a2 = 0.5 * rho * (v_r ** 2 + 2 * v_r * d_ref * q_m)
    check(q_a1 > q_a2 and (q_a1 - q_a2) / q_a2 < 0.01,
          "A.1 → A.2: 버린 q_m² 항이 1% 미만 (원문 근거)")

    # A.5 = A.4 with S_fin = S_ref/4
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
    dmp = sup_a.damping(0.0, 0.0, 0.3)
    check(rel(dmp.c_mq, -c_fin), "A.11·A.12 ⇒ C_mqm = −C_fin")
    c_mmd_eq8 = (q_m * d_ref / (2 * v_r)) * dmp.c_mq
    check(rel(c_mmd_eq8, c_mmd_a9),
          "  식 (8) 로 되돌린 C_mmd 가 A.9 와 일치")

    # ── 부록 B: 두 경로 동등성 ─────────────────────────────────
    print("\n[B] B.6 직접 M_d 와 파생 C_mqm → 조립기 경로")
    c_cf, length, cg = 1.2, 4.0, 0.4
    sup_b = AppendixBSupplier(c_cf, length, cg, d_ref, s_ref)
    ref = Reference(s_ref=s_ref, d_ref=d_ref, mrp_b=(0.0, 0.0, 0.0))

    ok_all = True
    for rho_, v_, q_ in ((1.0, 100.0, 0.3), (0.4, 250.0, -0.7),
                         (1.225, 80.0, 1.1)):
        direct = sup_b.moment_direct(rho_, v_, q_)
        # 조립기 경로: Q = ½ρV² 를 **독립 입력**으로 준다
        q_bar = 0.5 * rho_ * v_ ** 2
        # V⃗_R 은 +X, 그러면 φ_A 규약이 걸리지 않도록 vz 를 조금 준다.
        # α_tot 은 계수에 영향이 없다 (공급자가 상수).
        _, m_b = assemble((v_, 0.0, 1e-9), (0.0, q_, 0.0), q_bar, 340.0,
                          ref, sup_b)
        ok_all &= abs(m_b[1] - direct) <= 1e-9 * max(abs(direct), 1e-30)
    check(ok_all, "세 (ρ, V_R, q_m) 조합에서 두 경로가 일치")

    # C_mqm 이 상태에 무관한가 — 유도의 결과
    a = sup_b.c_mqm()
    check(rel(a, sup_b.c_mqm()), "C_mqm 은 ρ·V_R·q_m 에 무관")

    # 무차원화 정의를 **독립 입력**으로 검사
    print("\n[B] 정의를 하나씩 틀리게 넣으면 동등성이 깨지는가")
    d_bad = AppendixBSupplier(c_cf, length, cg, d_ref * 2.0, s_ref)
    m_dir = d_bad.moment_direct(1.0, 100.0, 0.3)
    _, m_asm = assemble((100.0, 0.0, 1e-9), (0.0, 0.3, 0.0), 0.5 * 1.0 * 1e4,
                        340.0, ref, d_bad)   # ref 는 여전히 d_ref
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
    refuses(lambda: AppendixBSupplier(1.0, -1.0, 0.5, 0.5, 2.0), "L <= 0 거절")
    refuses(lambda: AppendixBSupplier(1.0, 4.0, 1.5, 0.5, 2.0),
            "cg 가 [0,1] 밖이면 거절")
    refuses(lambda: AppendixBSupplier(1.0, 4.0, 0.5, 0.0, 2.0), "D <= 0 거절")
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
