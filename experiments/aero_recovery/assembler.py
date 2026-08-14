#!/usr/bin/env python3
"""공력 프레임·힘/모멘트 조립기 — 비생산. 계수를 만들지 않는다.

범위 (ADR-0003 §11-B 에서 고정)
-------------------------------
- 외부 계수 공급자를 **주입**받는다
- M/P/B 프레임 변환과 힘·모멘트 조립
- `atan2` 및 α_tot = 0°/180° 규약
- MRP 오프셋 모멘트
- `V_R = 0`, 비유한 입력, 계수 미제공은 **fail-closed**

**이 파일은 공력 계수를 만들어내지 않는다.** 계수는 전부 주입된 공급자가
낸다. 부록 A/B 의 해석식은 `appendix_suppliers.py` 에 있고 **제한 조건용
시험 공급자**이며 기본 공력 모델이 아니다.

식 출처: NASA/CR—2012–217475 (NTRS 20130003336). 전사·전제·입력 등급은
`RECOVERY_AUDIT.md` 가 한 곳에서 정의한다.
"""
import math
from dataclasses import dataclass
from typing import Protocol


class AeroError(Exception):
    """조립기 계약 위반. 조용히 넘어가지 않는다."""


# α_tot = 0°/180° 에서 φ_A 가 정의되지 않는다. 원문이 값을 지정하라고
# 못 박고 0° 를 예로 든다 (3쪽). 그 규약을 여기 한 번만 둔다.
PHI_A_AT_SINGULARITY_RAD = 0.0

# V_Ry, V_Rz 가 이 값 아래면 φ_A 를 특이점 규약으로 처리한다.
# 임계값 자체는 **동결하지 않는다** — 호출자가 바꿀 수 있게 인자로 둔다.
DEFAULT_PHI_EPS = 1e-12


@dataclass(frozen=True)
class Reference:
    """기준량. 형상에서 계산되는 값 (감사 §3)."""
    s_ref: float          # 원통 단면적
    d_ref: float          # 원통 직경 = 기준 길이
    mrp_b: tuple          # 무게중심 → MRP 벡터 (B 프레임). 모델 선언 + 질량특성

    def validate(self):
        for n, v in (("s_ref", self.s_ref), ("d_ref", self.d_ref)):
            if not math.isfinite(v) or v <= 0.0:
                raise AeroError(f"{n} 이 유한 양수가 아니다: {v!r}")
        if len(self.mrp_b) != 3 or not all(math.isfinite(c) for c in self.mrp_b):
            raise AeroError(f"mrp_b 가 유한한 3벡터가 아니다: {self.mrp_b!r}")


@dataclass(frozen=True)
class Static:
    """M 프레임 정적 계수 6개 (감사 §4-2). 부호 규약은 식 (10) 에서 적용."""
    c_a: float
    c_y: float
    c_n: float
    c_l: float
    c_m: float
    c_yaw: float


@dataclass(frozen=True)
class Damping:
    """M 프레임 감쇠 미계수 3개 (감사 §4-3, 식 4–6)."""
    c_lp: float
    c_mq: float
    c_yawr: float


class CoefficientSupplier(Protocol):
    """**주입되는** 계수 공급자. 조립기는 이것을 구현하지 않는다."""

    def static(self, alpha_tot_rad: float, mach: float) -> Static: ...

    def damping(self, alpha_tot_rad: float, mach: float) -> Damping: ...


def flow_angles(v_r_b, v_sound, phi_eps=DEFAULT_PHI_EPS):
    """식 (1)–(3). `v_r_b` 는 B 프레임 V⃗_R 성분 (MRP 로 옮긴 것).

    반환 (α_tot, φ_A, Mach, V_R) — 각은 radian.

    fail-closed:
      - 비유한 입력
      - `V_R = 0` — 식 (1) 이 정의되지 않는다. 감쇠 모멘트는 상쇄로 유한하게
        0 으로 가지만 α_tot 은 상쇄되지 않는다 (감사 §6-1). 조용히 0 을
        반환하면 정적 계수 조회 인자가 거짓이 된다
      - `v_sound <= 0`
    """
    if len(v_r_b) != 3 or not all(math.isfinite(c) for c in v_r_b):
        raise AeroError(f"v_r_b 가 유한한 3벡터가 아니다: {v_r_b!r}")
    if not math.isfinite(v_sound) or v_sound <= 0.0:
        raise AeroError(f"v_sound 가 유한 양수가 아니다: {v_sound!r}")

    vx, vy, vz = v_r_b
    v_r = math.sqrt(vx * vx + vy * vy + vz * vz)
    if v_r <= 0.0:
        raise AeroError(
            "V_R = 0 — 식 (1) 의 α_tot 이 정의되지 않는다. 속도 하한은 "
            "원문이 주지 않으므로 호출자가 정해야 한다")

    # 식 (1). acos 인자를 클램프하되, 클램프가 실제로 물렸는지는
    # 반올림 수준이어야 한다.
    c = vx / v_r
    if abs(c) > 1.0 + 1e-12:
        raise AeroError(f"V_Rx/V_R = {c!r} — 정규화가 깨졌다")
    alpha_tot = math.acos(min(1.0, max(-1.0, c)))

    # 식 (2). 범위가 ±180° 이므로 atan2 여야 한다. 끝점에서는 원문 규약.
    if abs(vy) <= phi_eps and abs(vz) <= phi_eps:
        phi_a = PHI_A_AT_SINGULARITY_RAD
    else:
        phi_a = math.atan2(vy, vz)

    return alpha_tot, phi_a, v_r / v_sound, v_r


def damping_moment_coeffs(damp, pqr_m, d_ref, v_r):
    """식 (7)–(9). `pqr_m` 은 M 프레임 각속도 (rad/s)."""
    if not all(math.isfinite(c) for c in pqr_m):
        raise AeroError(f"pqr_m 이 유한하지 않다: {pqr_m!r}")
    k = d_ref / (2.0 * v_r)
    return (pqr_m[0] * k * damp.c_lp,
            pqr_m[1] * k * damp.c_mq,
            pqr_m[2] * k * damp.c_yawr)


def assemble(v_r_b, pqr_b, q_bar, v_sound, ref, supplier,
             phi_eps=DEFAULT_PHI_EPS):
    """식 (1)–(18) 전체 조립. 반환 (F⃗_b, M⃗_b).

    `pqr_b` 는 **B 프레임** 몸체 각속도. M 프레임 각속도는 중심선 둘레로
    `−φ_A` 회전해 얻는다 (감사 §4-3).
    """
    ref.validate()
    if not math.isfinite(q_bar) or q_bar < 0.0:
        raise AeroError(f"동압이 유한 비음수가 아니다: {q_bar!r}")
    if len(pqr_b) != 3 or not all(math.isfinite(c) for c in pqr_b):
        raise AeroError(f"pqr_b 가 유한한 3벡터가 아니다: {pqr_b!r}")

    alpha_tot, phi_a, mach, v_r = flow_angles(v_r_b, v_sound, phi_eps)

    st = supplier.static(alpha_tot, mach)
    dp = supplier.damping(alpha_tot, mach)
    for name, obj in (("static", st), ("damping", dp)):
        if obj is None:
            raise AeroError(f"공급자가 {name} 계수를 주지 않았다")
        for f, v in vars(obj).items():
            if not math.isfinite(v):
                raise AeroError(f"{name}.{f} 가 유한하지 않다: {v!r}")

    # B → M: 중심선 둘레 −φ_A 회전
    c, s = math.cos(-phi_a), math.sin(-phi_a)
    p_m = pqr_b[0]
    q_m = c * pqr_b[1] + s * pqr_b[2]
    r_m = -s * pqr_b[1] + c * pqr_b[2]

    c_lmd, c_mmd, c_yawmd = damping_moment_coeffs(dp, (p_m, q_m, r_m),
                                                  ref.d_ref, v_r)

    # 식 (10)–(11). 축력·법선력의 음부호가 §4-2 부호 규약이다.
    qs = q_bar * ref.s_ref
    f_m = (-qs * st.c_a, qs * st.c_y, -qs * st.c_n)
    qsd = qs * ref.d_ref
    m_m = (qsd * (st.c_l + c_lmd),
           qsd * (st.c_m + c_mmd),
           qsd * (st.c_yaw + c_yawmd))

    # 식 (12)–(13). M → P
    cp, sp = math.cos(phi_a), math.sin(phi_a)

    def rot(v):
        return (v[0], cp * v[1] + sp * v[2], -sp * v[1] + cp * v[2])

    f_p, m_p = rot(f_m), rot(m_m)

    # 식 (14)–(18). P → B, MRP 오프셋 모멘트
    rx, ry, rz = ref.mrp_b
    f_b = f_p
    m_b = (m_p[0] + ry * f_p[2] - rz * f_p[1],
           m_p[1] + rz * f_p[0] - rx * f_p[2],
           m_p[2] + rx * f_p[1] - ry * f_p[0])
    return f_b, m_b
