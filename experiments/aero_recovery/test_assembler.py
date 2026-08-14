#!/usr/bin/env python3
"""조립기 자동 검사. 비생산.

계수값을 검사하지 않는다 — 조립기는 계수를 만들지 않으므로 검사할 것이
없다. 검사하는 것은 **조립 규약**이다: 축·부호, 무차원 재폐쇄, 대칭성,
끝점 규약, fail-closed.
"""
import math
import sys

from assembler import (AeroError, Damping, Reference, Static, assemble,
                       flow_angles)

fails = []


def check(ok, label):
    print(f"  {'OK  ' if ok else 'FAIL'}  {label}")
    if not ok:
        fails.append(label)


def refuses(fn, label):
    try:
        fn()
    except AeroError:
        check(True, label)
    else:
        check(False, label + " (거절하지 않았다)")


class Const:
    """상수 공급자 — 시험용. 실제 공력이 아니다."""

    def __init__(self, st, dp):
        self._s, self._d = st, dp

    def static(self, a, m):
        return self._s

    def damping(self, a, m):
        return self._d


REF = Reference(s_ref=2.0, d_ref=0.5, mrp_b=(0.0, 0.0, 0.0))
ZERO_D = Damping(0.0, 0.0, 0.0)


def main():
    print("조립기 자동 검사\n")

    # ── 각도 규약 ────────────────────────────────────────────────
    print("[각도] 식 (1)–(3) 과 끝점 규약")
    a, p, mch, vr = flow_angles((100.0, 0.0, 0.0), 340.0)
    check(abs(a) < 1e-12, "V⃗_R 이 +X 축이면 α_tot = 0")
    check(p == 0.0, "  그 끝점에서 φ_A 는 원문 규약값 0")
    a, p, _, _ = flow_angles((-100.0, 0.0, 0.0), 340.0)
    check(abs(a - math.pi) < 1e-12, "V⃗_R 이 −X 축이면 α_tot = 180°")
    check(p == 0.0, "  그 끝점에서도 φ_A = 0")
    # 규약을 지우면 atan2(0.0, -0.0) = π 가 나온다. −0.0 을 써야 갈린다.
    _, p, _, _ = flow_angles((100.0, 0.0, -0.0), 340.0)
    check(p == 0.0,
          "  vz = −0.0 끝점에서도 규약값 0 (규약 없으면 atan2 가 π)")
    a, p, _, _ = flow_angles((0.0, 0.0, 100.0), 340.0)
    check(abs(a - math.pi / 2) < 1e-12, "V⃗_R 이 +Z 면 α_tot = 90°")
    check(abs(p) < 1e-12, "  φ_A = atan2(0, +) = 0")
    _, p, _, _ = flow_angles((0.0, 100.0, 0.0), 340.0)
    check(abs(p - math.pi / 2) < 1e-12, "V⃗_R 이 +Y 면 φ_A = +90°")
    _, p, _, _ = flow_angles((0.0, -1.0, -100.0), 340.0)
    check(p < -math.pi / 2, "3사분면에서 φ_A < −90° — atan2 여야 나온다")
    _, _, mch, _ = flow_angles((340.0, 0.0, 0.0), 340.0)
    check(abs(mch - 1.0) < 1e-12, "Mach = V_R / V_sound")

    # ── 축·부호 ─────────────────────────────────────────────────
    print("\n[축·부호] 식 (10) 의 음부호 규약")
    sup = Const(Static(c_a=1.0, c_y=0.0, c_n=0.0, c_l=0.0, c_m=0.0,
                       c_yaw=0.0), ZERO_D)
    f, m = assemble((100.0, 0.0, 0.0), (0, 0, 0), 10.0, 340.0, REF, sup)
    check(f[0] < 0, "C_Am > 0 이면 F_Xb < 0 (X 축 반대가 양)")
    check(abs(f[0] + 10.0 * 2.0) < 1e-12, "  크기 = Q·S_ref·C_Am")
    sup = Const(Static(0.0, 0.0, 1.0, 0.0, 0.0, 0.0), ZERO_D)
    f, _ = assemble((100.0, 0.0, 0.0), (0, 0, 0), 10.0, 340.0, REF, sup)
    check(f[2] < 0, "C_Nm > 0 이면 F_Zb < 0 (Z 축 반대가 양)")
    sup = Const(Static(0.0, 1.0, 0.0, 0.0, 0.0, 0.0), ZERO_D)
    f, _ = assemble((100.0, 0.0, 0.0), (0, 0, 0), 10.0, 340.0, REF, sup)
    check(f[1] > 0, "C_Ym > 0 이면 F_Yb > 0 (측력은 부호 반전 없음)")

    # ── 무차원 재폐쇄 ────────────────────────────────────────────
    print("\n[무차원] 식 (7)–(9) 재폐쇄")
    q_m, c_mq = 0.3, -1.7
    sup = Const(Static(0, 0, 0, 0, 0, 0), Damping(0.0, c_mq, 0.0))
    _, m = assemble((100.0, 0.0, 0.0), (0.0, q_m, 0.0), 10.0, 340.0, REF, sup)
    want = 10.0 * REF.s_ref * REF.d_ref * (q_m * REF.d_ref /
                                           (2 * 100.0)) * c_mq
    check(abs(m[1] - want) < 1e-15, "M_Yb = Q·S·D·(q·D/2V)·C_mqm")
    # 두 배 속도 → 감쇠 모멘트 계수 절반, 그러나 Q 는 네 배
    _, m2 = assemble((200.0, 0.0, 0.0), (0.0, q_m, 0.0), 40.0, 340.0, REF, sup)
    check(abs(m2[1] / m[1] - 2.0) < 1e-12,
          "V 2배·Q 4배 → 감쇠 모멘트 2배 (Q·1/V 상쇄)")

    # ── 대칭성 ──────────────────────────────────────────────────
    print("\n[대칭] 롤 자세 무관성")
    sup = Const(Static(1.0, 0.0, 0.5, 0.0, 0.2, 0.0), ZERO_D)
    base = assemble((100.0, 0.0, 30.0), (0, 0, 0), 10.0, 340.0, REF, sup)
    mag0 = math.sqrt(sum(c * c for c in base[0]))
    ok = True
    for deg in (17.0, 90.0, 213.0):
        r = math.radians(deg)
        v = (100.0, 30.0 * math.sin(r), 30.0 * math.cos(r))
        f, _ = assemble(v, (0, 0, 0), 10.0, 340.0, REF, sup)
        ok &= abs(math.sqrt(sum(c * c for c in f)) - mag0) < 1e-12
    check(ok, "축대칭 계수면 롤 자세를 돌려도 힘 크기 불변")
    # 크기만 보면 회전 **부호**가 잡히지 않는다. M→P 는 −φ_A 회전이므로
    # φ_A > 0 에서 M 프레임의 +Z 성분이 P 프레임 +Y 로 양수만큼 온다.
    sup_z = Const(Static(0.0, 0.0, -1.0, 0.0, 0.0, 0.0), ZERO_D)  # F_Zm > 0
    f, _ = assemble((100.0, 50.0, 50.0), (0, 0, 0), 10.0, 340.0, REF, sup_z)
    check(f[1] > 0 and f[2] > 0,
          "φ_A = +45° 에서 F_Zm > 0 은 F_Yp·F_Zp 를 모두 양으로 만든다")

    print("\n[B→M] 각속도 회전")
    # 감쇠는 **M 프레임** 각속도를 쓴다. B 프레임 순수 요율은 φ_A = 90° 에서
    # M 프레임 피치율이 되어야 한다.
    sup_q = Const(Static(0, 0, 0, 0, 0, 0), Damping(0.0, -1.0, 0.0))
    # 성분이 아니라 **크기**로 본다 — φ_A = 90° 에서 M 프레임 피치 모멘트는
    # 다시 P 프레임 Z 성분으로 돌아가므로 Y 성분만 보면 0 이다.
    def mag(v):
        return math.sqrt(sum(c * c for c in v))

    _, m_yaw = assemble((100.0, 50.0, 0.0), (0.0, 0.0, 1.0), 10.0, 340.0,
                        REF, sup_q)
    check(mag(m_yaw) > 1e-9,
          "φ_A = 90° 에서 B 프레임 요율이 M 프레임 피치 감쇠를 만든다")
    _, m_none = assemble((100.0, 0.0, 50.0), (0.0, 0.0, 1.0), 10.0, 340.0,
                         REF, sup_q)
    check(mag(m_none) < 1e-12,
          "  φ_A = 0° 에서는 같은 요율이 피치 감쇠를 만들지 않는다")

    # ── MRP 오프셋 ──────────────────────────────────────────────
    print("\n[MRP] 식 (16)–(18) 오프셋 모멘트")
    off = Reference(2.0, 0.5, (0.0, 0.0, 1.0))
    sup = Const(Static(0.0, 1.0, 0.0, 0.0, 0.0, 0.0), ZERO_D)
    f, m = assemble((100.0, 0.0, 0.0), (0, 0, 0), 10.0, 340.0, off, sup)
    check(abs(m[0] - (-1.0 * f[1])) < 1e-12,
          "MRP_Zb·F_Yp 가 M_Xb 에 −부호로 들어간다")
    zero = assemble((100.0, 0.0, 0.0), (0, 0, 0), 10.0, 340.0, REF, sup)
    check(abs(zero[1][0]) < 1e-15, "MRP 오프셋 0 이면 추가 모멘트 없음")

    # ── fail-closed ─────────────────────────────────────────────
    print("\n[fail-closed]")
    refuses(lambda: flow_angles((0.0, 0.0, 0.0), 340.0), "V_R = 0 거절")
    refuses(lambda: flow_angles((float('nan'), 0, 0), 340.0), "NaN 속도 거절")
    refuses(lambda: flow_angles((100.0, 0, 0), 0.0), "음속 0 거절")
    refuses(lambda: assemble((100.0, 0, 0), (0, 0, 0), float('inf'), 340.0,
                             REF, Const(Static(0, 0, 0, 0, 0, 0), ZERO_D)),
            "비유한 동압 거절")
    refuses(lambda: assemble((100.0, 0, 0), (0, 0, 0), 10.0, 340.0,
                             Reference(-1.0, 0.5, (0, 0, 0)),
                             Const(Static(0, 0, 0, 0, 0, 0), ZERO_D)),
            "S_ref <= 0 거절")

    class NoStatic:
        def static(self, a, m):
            return None

        def damping(self, a, m):
            return ZERO_D

    refuses(lambda: assemble((100.0, 0, 0), (0, 0, 0), 10.0, 340.0, REF,
                             NoStatic()), "계수 미제공 거절")

    class NanStatic:
        def static(self, a, m):
            return Static(float('nan'), 0, 0, 0, 0, 0)

        def damping(self, a, m):
            return ZERO_D

    refuses(lambda: assemble((100.0, 0, 0), (0, 0, 0), 10.0, 340.0, REF,
                             NanStatic()), "공급자가 NaN 을 내면 거절")

    print()
    if fails:
        print(f"FAIL: {len(fails)}건", file=sys.stderr)
        return 1
    print("OK: 전 항목 통과")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
