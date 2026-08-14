#!/usr/bin/env python3
"""부록 A/B **제한 조건용** 시험 공급자. 비생산.

**기본 공력 모델이 아니다.** 각 공급자는 원문 부록이 세운 전제 아래에서만
성립하며, 전제는 클래스 문서에 전부 적는다. 실제 형상에 쓰려면 그 전제가
모두 만족되는지 호출자가 확인해야 한다.

이 파일도 **계수를 지어내지 않는다** — 부록 A 는 외부 `C_fin` 을,
부록 B 는 외부 교차류 계수 `C` 를 입력으로 받는다.
"""
import math

from assembler import AeroError, Damping, Static, assemble, flow_angles


# 부록 A·B 모두 **교차류 조건**을 전제한다. 공급자가 각도를 무시하면
# 전제 밖 호출이 조용히 통과한다 — 첫 판의 시험이 α_tot ≈ 0° 축방향
# 흐름으로 돌면서 통과했다. 프로토콜 구현으로 남기되 **거부**한다.
CROSSFLOW_RAD = math.pi / 2
DEFAULT_CROSSFLOW_TOL_RAD = 1e-9


def _check_tol(tol):
    """허용오차에 **상한**이 없으면 전제를 다시 우회한다 — `tol = π` 를
    주면 α_tot = 0 축방향 흐름도 승인된다 (재현). π/2 이상이면 전 범위를
    덮으므로 반드시 그 미만이다."""
    if not math.isfinite(tol) or not 0.0 <= tol < math.pi / 2:
        raise AeroError(f"crossflow_tol_rad 는 [0, π/2) 이어야 한다: {tol!r}")


def _require_premises(alpha_tot_rad, phi_a_rad, tol, who):
    """부록 전제 중 **공급자가 볼 수 있는** 것: 교차류이고 φ_A = 0.

    φ_A ≠ 0 이면 B→M 회전 때문에 몸체 피치율이 M 프레임 피치율이 아니게
    되고, 조립 결과가 B.6 직접 모멘트와 갈린다 (측정: φ_A = 90° 에서
    조립기 −0.000, 직접 −107.520). 교차류만 보면 이 경우가 통과한다.
    """
    for n, v in (("α_tot", alpha_tot_rad), ("φ_A", phi_a_rad)):
        if not math.isfinite(v):
            raise AeroError(f"{who}: {n} 이 유한하지 않다: {v!r}")
    if abs(alpha_tot_rad - CROSSFLOW_RAD) > tol:
        raise AeroError(
            f"{who} 는 교차류(α_tot = 90°) 전제에서만 유효하다 — "
            f"α_tot = {math.degrees(alpha_tot_rad):.4f}° 로 호출됐다")
    if abs(phi_a_rad) > tol:
        raise AeroError(
            f"{who} 는 M·P·B 프레임 일치를 전제하므로 φ_A = 0 이어야 한다 — "
            f"φ_A = {math.degrees(phi_a_rad):.4f}° 로 호출됐다")


def require_assembly_premises(ref, supplier):
    """조립 경로에서만 볼 수 있는 전제. **공급자 인터페이스가 `Reference`
    를 받지 않으므로** 별도 함수로 둔다.

    - MRP = 무게중심 (부록은 무게중심 둘레 모멘트를 준다)
    - 공급자와 `Reference` 의 `D_ref`·`S_ref` 가 같아야 한다
    """
    if any(c != 0.0 for c in ref.mrp_b):
        raise AeroError(
            f"부록 공급자는 MRP = 무게중심을 전제한다 — mrp_b = {ref.mrp_b!r}")
    for attr, name, want in (("d", "D_ref", ref.d_ref),
                             ("s", "S_ref", ref.s_ref)):
        got = getattr(supplier, attr, None)
        if got is None:
            continue
        if abs(got - want) > 1e-12 * max(abs(want), 1.0):
            raise AeroError(
                f"공급자의 {name} = {got!r} 가 Reference 의 {want!r} 와 다르다")


MAX_TRUNCATION_RATIO = 0.01     # 원문의 "약 1%" 조건


def assemble_appendix(v_r_b, pqr_b, q_bar, v_sound, ref, supplier, **kw):
    """부록 공급자 전용 조립 진입점.

    **적용성 검사 → 일반 assemble** 순서를 **구조적으로** 강제한다.
    검사 함수를 따로 두기만 하면 호출자가 빠뜨릴 수 있고, 실제로 빠져
    있었다 — MRP 가 1 m 어긋난 `Reference` 로도 raw `assemble()` 이
    정상 반환했다.

    행마다 폐기항 조건도 본다. 두 경로가 **같은 근사식에서 출발**하므로
    전제 밖에서도 대수적으로 일치한다 — 그래서 일치만으로는 전제 준수의
    증거가 되지 않는다.
    """
    require_assembly_premises(ref, supplier)
    _, _, _, v_r = flow_angles(v_r_b, v_sound,
                               kw.get("phi_rel_eps", 0.0))
    ratio = None
    fn = getattr(supplier, "truncation_ratio", None)
    if fn is not None:
        ratio = fn(v_r, pqr_b[1])          # 피치율
    if ratio is not None and ratio >= MAX_TRUNCATION_RATIO:
        raise AeroError(
            f"폐기항 비 {ratio:.5f} >= {MAX_TRUNCATION_RATIO} — 부록의 "
            f"1% 근사 전제 밖이다 (V_R = {v_r!r}, q_m = {pqr_b[1]!r})")
    return assemble(v_r_b, pqr_b, q_bar, v_sound, ref, supplier, **kw)


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

    def __init__(self, c_fin, d_ref=None,
                 crossflow_tol_rad=DEFAULT_CROSSFLOW_TOL_RAD):
        if not isinstance(c_fin, (int, float)) or not math.isfinite(c_fin):
            raise AeroError(f"C_fin 이 유한한 수가 아니다: {c_fin!r}")
        _check_tol(crossflow_tol_rad)
        self.c_fin = float(c_fin)
        self.tol = float(crossflow_tol_rad)
        # 폐기항 조건 검사에 D_ref 가 필요하다. 없으면 그 검사를 건너뛴다.
        self.d_ref = None if d_ref is None else float(d_ref)

    def truncation_ratio(self, v_r, q_m):
        """A.1 → A.2 에서 버린 `D_ref² q_m²` 항의 비 — |D q|/(2V) < 0.01."""
        if self.d_ref is None:
            return None
        return abs(self.d_ref * q_m) / (2.0 * v_r)

    def static(self, alpha_tot_rad, phi_a_rad, mach):
        _require_premises(alpha_tot_rad, phi_a_rad, self.tol, "부록 A")
        return Static(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def damping(self, alpha_tot_rad, phi_a_rad, mach):
        _require_premises(alpha_tot_rad, phi_a_rad, self.tol, "부록 A")
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

    **약분되는 것은 `ρ`·`V_R`·`q_m` 뿐이다.** 외부 `C` 는 조건에 따라
    달라질 수 있고 `cg` 도 질량특성 변화에 따라 달라진다. 그러므로 이
    클래스는 **고정 `C`·`cg` 조건의 시험 공급자**이지 일반적으로 상태에
    무관한 공급자가 아니다.

    `cg` 무차원화 — **원문 정의**: *"distance from center-of-mass to
    **forward** end of cylinder, expressed as a fraction of cylinder
    length"*. 즉 **무게중심에서 전단까지의 거리 / L** 이다. 후단 기준
    위치를 쓰려면 `1 − cg` 를 넣어야 한다.

    주의: 다항식 `(1 − 3cg + 3cg²)` 는 `cg ↔ 1−cg` 대칭이므로 **이 정의를
    뒤집어도 수치가 같다.** 시험으로 잡히지 않으니 정의를 문서로 못 박는
    수밖에 없다 — 첫 판이 실제로 뒤집어 적었다. `C`, `L`, `cg`, `D` 는 **각각 독립
    입력**이다 — 등급이 다르다(감사 §5: `C` 외부 자료, `L`·`D` 형상,
    `cg` 질량특성·상태).
    """

    def __init__(self, c_crossflow, length, cg_frac, d_ref, s_ref,
                 crossflow_tol_rad=DEFAULT_CROSSFLOW_TOL_RAD):
        vals = dict(C=c_crossflow, L=length, cg=cg_frac, D=d_ref, S=s_ref)
        for n, v in vals.items():
            if not isinstance(v, (int, float)) or not math.isfinite(v):
                raise AeroError(f"{n} 이 유한한 수가 아니다: {v!r}")
        if length <= 0.0 or d_ref <= 0.0 or s_ref <= 0.0:
            raise AeroError(f"L·D·S_ref 는 양수여야 한다: {vals!r}")
        # C 는 교차류 **항력**계수다 — 음수면 흐름이 물체를 끄는 셈이다.
        if c_crossflow <= 0.0:
            raise AeroError(f"교차류 항력계수 C 는 양수여야 한다: "
                            f"{c_crossflow!r}")
        if not 0.0 <= cg_frac <= 1.0:
            raise AeroError(f"cg 는 [0, 1] 무차원 위치여야 한다: {cg_frac!r}")
        self.c = float(c_crossflow)
        self.l = float(length)
        self.cg = float(cg_frac)
        self.d = float(d_ref)
        self.s = float(s_ref)
        _check_tol(crossflow_tol_rad)
        self.tol = float(crossflow_tol_rad)
        # 원문 정의: 교차류에서 총 투영면적 = L·D, 기준면적은 원통 단면적.
        # 형상이 일관되지 않으면 대수는 닫혀도 출처 충실성 시험이 아니다.
        want_s = math.pi * d_ref ** 2 / 4.0
        if abs(s_ref - want_s) > 1e-9 * want_s:
            raise AeroError(
                f"S_ref 는 원통 단면적 πD²/4 = {want_s!r} 이어야 한다 "
                f"(받은 값 {s_ref!r})")

    def truncation_ratio(self, v_r, q_m):
        """B.1 → B.2 에서 버린 `q_m² X²` 항의 비. 원문 조건은 **3항 대 2항**:

            |q_m X| / (2 V_R) < 0.01,   X 는 원통 전 구간의 최대값

        `max|X| = L · max(cg, 1−cg)` — 무게중심에서 먼 쪽 끝이 지배한다.
        """
        return abs(q_m) * self.l * max(self.cg, 1.0 - self.cg) / (2.0 * v_r)

    def moment_direct(self, rho, v_r, q_m):
        """B.6 을 **그대로** — 파생 경로를 거치지 않는다."""
        for n, v in (("rho", rho), ("V_R", v_r), ("q_m", q_m)):
            if not isinstance(v, (int, float)) or not math.isfinite(v):
                raise AeroError(f"{n} 이 유한한 수가 아니다: {v!r}")
        if rho <= 0.0:
            raise AeroError(f"밀도는 양수여야 한다: {rho!r}")
        if v_r <= 0.0:
            raise AeroError(f"V_R 은 양수여야 한다: {v_r!r}")
        m_d = (-(1.0 / 3.0) * rho * v_r * q_m * self.c * self.d
               * self.l ** 3 * (1.0 - 3.0 * self.cg + 3.0 * self.cg ** 2))
        # 입력이 전부 유한해도 곱에서 넘칠 수 있다.
        if not math.isfinite(m_d):
            raise AeroError(f"M_d 가 유한하지 않다: {m_d!r}")
        return m_d

    def c_mqm(self):
        """파생 경로.

        `ρ`·`V_R`·`q_m` **만** 약분된다. 외부 `C` 와 `cg` 는 조건·질량특성에
        따라 변하므로, 이 값은 **고정 `C`·`cg` 조건에서만** 상수다.
        """
        v = (-(4.0 / 3.0) * self.c * self.d * self.l ** 3
             * (1.0 - 3.0 * self.cg + 3.0 * self.cg ** 2)
             / (self.s * self.d ** 2))
        if not math.isfinite(v):
            raise AeroError(f"C_mqm 이 유한하지 않다: {v!r}")
        return v

    def static(self, alpha_tot_rad, phi_a_rad, mach):
        _require_premises(alpha_tot_rad, phi_a_rad, self.tol, "부록 B")
        return Static(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def damping(self, alpha_tot_rad, phi_a_rad, mach):
        _require_premises(alpha_tot_rad, phi_a_rad, self.tol, "부록 B")
        return Damping(0.0, self.c_mqm(), 0.0)
