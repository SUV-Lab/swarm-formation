#!/usr/bin/env python3
"""강제응답 모멘트 프로파일 — 생성기와 독립 기준이 **같은 정의**를 쓴다.

생산 코드가 아니다.

왜 한 곳에 두나
---------------
JSBSim 쪽과 독립 기준 쪽이 각자 프로파일을 구현하면, 둘이 다를 때 그 차이가
적분기 차이로 보인다. 정의를 여기 한 번만 두고 양쪽이 읽는다.

두 프로파일을 **섞지 않는다**
-----------------------------
- `step`   계단. 멀티스텝 이력이 급변할 때의 취약성.
- `c0ramp` 선형 램프. 값만 이어짐.
- `c1ramp` 3차. 1차 도함수까지 이어짐.
- `c2ramp` 5차. 2차 도함수까지 이어짐.

계단과 C² 만 비교하면 "매끄러움 덕분"인지 "자극량이 작아서"인지 갈리지
않는다. 네 단계를 두어야 **필요한 연속성 차수**를 구분할 수 있고,
`--normalize` 로 절대 임펄스를 맞춰야 자극량 차이가 제거된다.

둘을 한 표에 섞으면 원인을 다시 가를 수 없으므로 표를 분리한다.

순서 (두 프로파일 공통): 일정 → 램프 → 유지 → 반전 → 해제
구간 길이 T 를 6등분해 위 다섯 국면에 배분한다.
"""
import math

# 벽돌의 관성(slug·ft²)에 견주어 몇 도/s² 급 각가속도가 나오는 크기.
# 축마다 다르게 주어 세 축이 함께 시험되도록 한다.
AMP_FTLBF = (2.0e-3, 4.0e-3, 3.0e-3)


def _c0ramp(u):
    """C⁰ — 선형. 값은 이어지지만 1차 도함수가 끊긴다."""
    return min(1.0, max(0.0, u))


def _c1step(u):
    """C¹ — 3차 smoothstep. 1차까지 이어지고 2차가 끊긴다."""
    u = min(1.0, max(0.0, u))
    return u * u * (3.0 - 2.0 * u)


def _c2step(u):
    """0→1 을 C² 로 잇는 최소차수 다항식 (5차 smoothstep).
    값·1차·2차 도함수가 양 끝에서 0 이므로 명령이 매끄럽다."""
    u = min(1.0, max(0.0, u))
    return u * u * u * (10.0 - 15.0 * u + 6.0 * u * u)


def shape(profile, t, T):
    """무차원 형상 s(t) ∈ [-1, 1]. 축 진폭을 곱하면 모멘트가 된다.

    국면 (T/6 씩):
      0  일정 0
      1  0 → +1
      2  +1 유지
      3  +1 → −1  (반전)
      4  −1 유지
      5  −1 → 0   (해제)
    """
    seg = T / 6.0
    k = min(5, int(t / seg)) if seg > 0 else 0
    u = (t - k * seg) / seg if seg > 0 else 0.0
    if profile == "step":
        return (0.0, 1.0, 1.0, -1.0, -1.0, 0.0)[k]
    ramp = {"c0ramp": _c0ramp, "c1ramp": _c1step, "c2ramp": _c2step}.get(profile)
    if ramp is not None:
        if k == 0:
            return 0.0
        if k == 1:
            return ramp(u)
        if k == 2:
            return 1.0
        if k == 3:
            return 1.0 - 2.0 * ramp(u)
        if k == 4:
            return -1.0
        return -1.0 + ramp(u)
    raise ValueError(f"알 수 없는 프로파일: {profile}")


def moment(profile, t, T):
    s = shape(profile, t, T)
    return tuple(a * s for a in AMP_FTLBF)


def abs_impulse(profile, T, n=200000):
    """∫|s(t)| dt — 자극량 척도. 정규화 기준."""
    h = T / n
    return sum(abs(shape(profile, (i + 0.5) * h, T)) for i in range(n)) * h


def schedule(profile, T, dt, normalize=None):
    """생성기가 그대로 소비할 (t, Mx, My, Mz) 표. **유일한 정의**다.

    normalize 가 주어지면 그 프로파일과 절대 임펄스가 같아지도록 진폭을
    비례 조정한다 — 매끄러움과 자극량을 분리하기 위한 것."""
    g = 1.0
    if normalize:
        g = abs_impulse(normalize, T) / max(abs_impulse(profile, T), 1e-30)
    n = int(round(T / dt))
    return [(k * dt, ) + tuple(g * v for v in moment(profile, k * dt, T))
            for k in range(n + 1)]


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("profile")
    ap.add_argument("duration", type=float)
    ap.add_argument("dt", type=float)
    ap.add_argument("--normalize", help="이 프로파일과 절대 임펄스를 맞춤")
    a = ap.parse_args()
    for row in schedule(a.profile, a.duration, a.dt, a.normalize):
        print(" ".join(f"{v!r}" if i == 0 else f"{v!r}"
                       for i, v in enumerate(row)))
