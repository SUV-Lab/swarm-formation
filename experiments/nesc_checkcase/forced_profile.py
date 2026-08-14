#!/usr/bin/env python3
"""강제응답 모멘트 프로파일 — 생성기와 독립 기준이 **같은 정의**를 쓴다.

생산 코드가 아니다.

왜 한 곳에 두나
---------------
JSBSim 쪽과 독립 기준 쪽이 각자 프로파일을 구현하면, 둘이 다를 때 그 차이가
적분기 차이로 보인다. 정의를 여기 한 번만 두고 양쪽이 읽는다.

두 프로파일을 **섞지 않는다**
-----------------------------
- `step`   계단 입력. 멀티스텝 이력이 급변할 때의 취약성을 본다.
- `c2ramp` 매끄러운 C² 램프. 실제로 매끄러운 명령에서 얻는 이점을 본다.

둘을 한 표에 섞으면 원인을 다시 가를 수 없으므로 표를 분리한다.

순서 (두 프로파일 공통): 일정 → 램프 → 유지 → 반전 → 해제
구간 길이 T 를 6등분해 위 다섯 국면에 배분한다.
"""
import math

# 벽돌의 관성(slug·ft²)에 견주어 몇 도/s² 급 각가속도가 나오는 크기.
# 축마다 다르게 주어 세 축이 함께 시험되도록 한다.
AMP_FTLBF = (2.0e-3, 4.0e-3, 3.0e-3)


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
    if profile == "c2ramp":
        if k == 0:
            return 0.0
        if k == 1:
            return _c2step(u)
        if k == 2:
            return 1.0
        if k == 3:
            return 1.0 - 2.0 * _c2step(u)
        if k == 4:
            return -1.0
        return -1.0 + _c2step(u)
    raise ValueError(f"알 수 없는 프로파일: {profile}")


def moment(profile, t, T):
    s = shape(profile, t, T)
    return tuple(a * s for a in AMP_FTLBF)


if __name__ == "__main__":
    import sys
    prof = sys.argv[1] if len(sys.argv) > 1 else "step"
    T = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0
    n = int(sys.argv[3]) if len(sys.argv) > 3 else 60
    for i in range(n + 1):
        t = T * i / n
        print(f"{t:.6f} " + " ".join(f"{v:.12g}" for v in moment(prof, t, T)))
