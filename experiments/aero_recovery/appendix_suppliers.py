#!/usr/bin/env python3
"""부록 A/B **제한 조건용** 시험 공급자. 비생산.

**기본 공력 모델이 아니다.** 각 공급자는 원문 부록이 세운 전제 아래에서만
성립하며, 전제는 클래스 문서에 전부 적는다. 실제 형상에 쓰려면 그 전제가
모두 만족되는지 호출자가 확인해야 한다.

이 파일도 **계수를 지어내지 않는다** — 부록 A 는 외부 `C_fin` 을,
부록 B 는 외부 교차류 계수 `C` 를 입력으로 받는다.
"""
import math

from assembler import AeroError, Damping, Static


class AppendixASupplier:
    """부록 A — `C_mqm = −C_fin` (A.11·A.12).

    **여섯 전제** (감사 §4-9). 하나라도 깨지면 이 공급자는 무효다:

    1. 후미 **수평 미익 하나**가 피치 감쇠 전부를 담당
       (원문이 *"not necessarily realistic"* 이라 단서를 단다)
    2. **미익–무게중심 거리 = `D_ref`**
    3. **M·P·B 세 프레임 일치**, 원점은 중심선 위 무게중심, Z 하향
    4. **교차류 조건**
    5. **`q_m²` 항 무시** (A.1 → A.2)
    6. **`S_fin = S_ref / 4`**

    `C_fin` 은 **외부 입력**이다 — 원문이 값을 주지 않는다.
    정적 계수는 이 공급자의 범위가 아니므로 전부 0 을 낸다.
    """

    def __init__(self, c_fin):
        if not isinstance(c_fin, (int, float)) or not math.isfinite(c_fin):
            raise AeroError(f"C_fin 이 유한한 수가 아니다: {c_fin!r}")
        self.c_fin = float(c_fin)

    def static(self, alpha_tot_rad, phi_a_rad, mach):
        return Static(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def damping(self, alpha_tot_rad, phi_a_rad, mach):
        # A.11 · A.12 의 우변이 부호만 다르다 ⇒ C_mqm = −C_fin.
        # 롤·요 감쇠는 부록 A 의 범위가 아니다 — 0 을 내되 그것이
        # "감쇠가 없다"는 물리 주장이 아니라 **범위 밖**이라는 뜻이다.
        return Damping(0.0, -self.c_fin, 0.0)


class AppendixBSupplier:
    """부록 B — 교차류 원통의 피치 감쇠 (B.6 에서 유도).

    **여섯 전제** (감사 §4-9):

    1. **교차류 조건**의 원통 — 미익·노즈콘 제외
    2. **무한 원통 공력 특성**, **끝단 유동 없음**
    3. **M·P·B 프레임이 무게중심에서 일치**
    4. **`C` 가 원통 크기·증분 면적 크기에 따라 변하지 않는다**
    5. **`q_m² X²` 항 무시** (B.1 → B.2)
    6. **정적 항 제외** (B.4 의 첫 항)

    B.6 은 **모멘트** `M_d` 를 준다. `C_mqm` 으로 가려면 A.8 과 식 (8) 을
    거쳐야 하고 **그 결합은 원문에 없다 — 파생이다**:

        A.8:  M_d   = ½ ρ V_R² S_ref D_ref C_mmd
        (8):  C_mmd = (q_m D_ref / 2V_R) C_mqm
          ⇒   C_mqm = 4 M_d / (ρ V_R q_m S_ref D_ref²)

        B.6:  M_d = −(1/3) ρ V_R q_m C D L³ (1 − 3cg + 3cg²)
          ⇒   C_mqm = −(4/3) C D L³ (1 − 3cg + 3cg²) / (S_ref D_ref²)

    **`C_mqm` 이 `ρ`·`V_R`·`q_m` 에 무관**하다는 것이 이 유도의 결과다 —
    셋이 전부 약분된다. 그래서 공급자가 상태에 의존하지 않는다.

    `cg` 무차원화: 원통 후단에서 잰 무게중심 위치를 `L` 로 나눈 값
    (B.5 의 적분 구간이 후단 → 전단). `C`, `L`, `cg`, `D` 는 **각각 독립
    입력**이다 — 등급이 다르다(감사 §5: `C` 외부 자료, `L`·`D` 형상,
    `cg` 질량특성·상태).
    """

    def __init__(self, c_crossflow, length, cg_frac, d_ref, s_ref):
        vals = dict(C=c_crossflow, L=length, cg=cg_frac, D=d_ref, S=s_ref)
        for n, v in vals.items():
            if not isinstance(v, (int, float)) or not math.isfinite(v):
                raise AeroError(f"{n} 이 유한한 수가 아니다: {v!r}")
        if length <= 0.0 or d_ref <= 0.0 or s_ref <= 0.0:
            raise AeroError(f"L·D·S_ref 는 양수여야 한다: {vals!r}")
        if not 0.0 <= cg_frac <= 1.0:
            raise AeroError(f"cg 는 [0, 1] 무차원 위치여야 한다: {cg_frac!r}")
        self.c = float(c_crossflow)
        self.l = float(length)
        self.cg = float(cg_frac)
        self.d = float(d_ref)
        self.s = float(s_ref)

    def moment_direct(self, rho, v_r, q_m):
        """B.6 을 **그대로** — 파생 경로를 거치지 않는다."""
        for n, v in (("rho", rho), ("V_R", v_r), ("q_m", q_m)):
            if not math.isfinite(v):
                raise AeroError(f"{n} 이 유한하지 않다: {v!r}")
        return (-(1.0 / 3.0) * rho * v_r * q_m * self.c * self.d
                * self.l ** 3 * (1.0 - 3.0 * self.cg + 3.0 * self.cg ** 2))

    def c_mqm(self):
        """파생 경로. ρ·V_R·q_m 이 약분돼 상태에 무관하다."""
        return (-(4.0 / 3.0) * self.c * self.d * self.l ** 3
                * (1.0 - 3.0 * self.cg + 3.0 * self.cg ** 2)
                / (self.s * self.d ** 2))

    def static(self, alpha_tot_rad, phi_a_rad, mach):
        return Static(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def damping(self, alpha_tot_rad, phi_a_rad, mach):
        return Damping(0.0, self.c_mqm(), 0.0)
