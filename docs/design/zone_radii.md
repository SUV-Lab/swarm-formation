# 위험 구역의 세 반경 — 한 곳에서 정의

구역 하나에 반경이 세 개 있다. 세 개가 각각 다른 질문에 답하고, 서로 다른 코드가
쓴다. 이 문서가 그 정의의 유일한 출처다.

세 개가 같은 면이 되면 무슨 일이 벌어지는지는 2026-08-12에 확인했다 — 거부면과
복원력이 0이 되는 면이 겹쳐서 같은 미션이 반복마다 CLEAN과 FAILED로 갈렸다
(`zone_gate_surface.md`).

| 반경 | 값 | 답하는 질문 | 한 곳에서 정의된 위치 |
|---|---|---|---|
| **authored** | `q < 1.0` | 미션이 금지한 부피인가 | `RiskZoneSpec.msg`의 `reach` |
| **standoff** | `q < 1.05` | 경로를 어디까지 떼어 놓을 것인가 | `dyn_a_star.h` `kZoneHardInflate` |
| **barrier support** | `q < 1.0 + 0.05` | 최적화기가 어디서부터 밀어내는가 | `poly_traj_optimizer.h` `kRiskBarrierRampFrac` |

`q`는 정규화 타원체 반경이다. `q = 1.0`이 yaml이 쓴 `reach`이고, 수직 반경은
`reach * risk_vertical_ratio`(0.35)다. 세 판정 모두 visibility 하한
`kZoneHardVis = 0.35`를 함께 요구한다 — 지형에 가려 안 보이는 점은 접촉이 아니다.

## 1. authored — 미션이 금지한 부피

시나리오 작성자가 `reach`로 쓴 그 타원체. **이것만이 비행을 거부한다.**

`reach`가 "HARD_AVOID의 금지 경계"라는 것은 **메시지에 대한 계약**이므로
`RiskZoneSpec.msg`에도 적었다. 이 문서만 소스 오브 트루스로 두면, 구역을 발행하는
쪽은 "가운데로 갈수록 비용이 오른다"까지만 읽고 "이 타원체는 금지"라는 뜻을 모른다.

**이 결정이 틀릴 수 있는 지점.** 외부 요구사항에서 1.05 여유(이 시나리오에서 500 m)까지
필수 이격거리라면, 거부면은 authored가 아니라 standoff여야 하고 이 변경은 반대로 틀린
것이 된다. 코드·문서·메시지 어디에도 그렇게 적힌 곳이 없어 routing standoff로 판단했다
(`dyn_a_star.h`의 도입 주석이 유일한 근거). 요구사항 출처가 있으면 그것이 이긴다.

- 판정: `zoneAuthoredVolumeContains(i, p)` = `zoneVolumeContains(i, p, 1.0, 0.35)`
- 쓰는 곳: `PathManager::zoneContactAuthored` → whole-flight audit의 `zone_hard_n`
  → `FlightVerdict::unflyable()` → `FAILED(STITCHED_FLIGHT_UNSAFE)` + 저장 궤적 무효화
- 회귀: `hardpen` 변형 (중심 관통 → FAILED)

## 2. standoff — 경로가 서는 자리

`authored`의 5% 바깥. **경로 생성용 여유이지 안전 경계가 아니다.**

도입 이유가 `dyn_a_star.h`에 적혀 있다: 경로가 rim에 붙으면 back end가 가장 가파른
visibility 기울기에 서게 되고, 지형 회피와의 duck-below 싸움에서 져 고도가 무너졌다
(측정: pull이 지형 복원력의 6-7배, terrain penetration, audit reject). 그래서 geodesic이
rim에서 **떨어져 서도록** 하는 여유다.

- 판정: `zoneHardVolumeContains(i, p)` = `zoneVolumeContains(i, p, 1.05, 0.35)`
- 쓰는 곳:
  - front end가 경로를 이 부피 밖으로 유지 (`insideHardZoneVol`, 3-pass)
  - transition **후보 선택**이 이 부피 안의 점을 실격 — 선택은 "떨어져 선 후보를
    고르는" 일이므로 여기서는 standoff가 기준이다 (`disq_zone_standoff`로 별도 계수)
  - whole-flight audit의 `zone_standoff_n` → `DEGRADED(STITCHED_ZONE_STANDOFF)`
- **비행을 거부하지 않는다.** 이 껍질 진입은 보고 대상이다. transition이 **생성한
  궤적**의 검증(`validateTransitionTrajectory`)도 여기서 거부하지 않는다 — 한 비행이
  구간마다 다른 배제 반경으로 판정되면 안 된다.
- 회귀: `standoffpen` 변형 (접선 통과 → DEGRADED, 궤적 유지)

### 남은 비대칭 (측정 필요)

후보 선택이 standoff에서 실격시키므로, standoff-clear 후보가 하나도 없으면 후보 집합이
비고 `TRANSITION_GENERATION_FAILED`로 미션이 거부된다. 즉 **간접적으로는** standoff가
아직 미션을 거부할 수 있다.

옳은 해법은 2단계 선택으로 보인다 — standoff-clear를 선호하되 없으면 authored-clear를
받고 DEGRADED로 표시. 하지만 그 전에 **standoff 단독으로 후보 집합이 비는 일이 실제로
얼마나 있는지**를 알아야 한다. 그래서 `TransitionAudit`에 `disq_zone`(authored)과
`disq_zone_standoff`를 나눠 세도록 했다. 판별 실험: transition이 켜진 시나리오 전체를
돌려 `disq_zone_standoff > 0 && disq_zone == 0`이면서 후보가 0개가 된 실행 수를 센다.
0이면 2단계 선택은 불필요하다.

## 3. barrier support — 복원력이 존재하는 구간

최적화기의 risk barrier가 힘을 내는 바깥 끝. 현재 `1.0 + kRiskBarrierRampFrac`
= 1.05로 **standoff와 같은 값이다.**

ramp가 smoothstep(`t²(3-2t)`)이라 그 면에서 **값도 기울기도 0**이다. 안쪽으로 미는
힘이 조금이라도 있으면 수렴 평형점은 구조적으로 그 면 안쪽에 놓인다.

`risk_max`가 읽는 moat는 이보다 더 안쪽에서만 산다 — `riskZoneValue`는
`if (!(q < 1.0)) return 0.0`이다. 그래서 standoff 껍질에서는 `risk_max = 0.0000`이
정상이며, `standoff > 0`과 같은 줄에 찍혀도 모순이 아니다.

## 4. 지켜야 할 관계

```
authored  <  barrier support  <=  standoff
   거부면      복원력이 사는 구간      경로가 서는 자리
```

**거부면은 복원력이 존재하는 구간의 안쪽에 있어야 한다.** 두 면이 겹치면 궤적은
힘이 0인 자리에 자연스럽게 자리잡고, 게이트는 그걸 위반이라고 부른다. 그 상태에서는
같은 입력의 미세한 차이가 판정을 뒤집는다.

지금 `barrier support == standoff`이므로 등호는 성립하지만 여유가 없다. `standoff`를
넓히거나 `barrier support`를 좁히는 변경을 할 때 이 부등식을 먼저 확인할 것.

## 5. 바꿀 때 확인할 것

- `kZoneHardInflate`를 바꾸면: front end의 경로 여유와 audit의 standoff 카운트가 같이
  움직인다. 통로가 좁은 시나리오(`r6_zone_slalom`, `r7_encircled_goal`,
  `r3_nested_zones`, `r3_goal_in_zone`)에서 `[ZONE-AVOID] pass` 분포와 미션 거부 수를
  같이 봐야 한다. pass=2로 내려가면 구역을 soft로 통과하는 경로가 나오고, 그건 지금보다
  나쁘다.
- `kRiskBarrierRampFrac`을 키우면: 구역에서 밀린 궤적이 지형 쪽으로 눌린다.
  `dyn_a_star.h`가 경고한 그 실패 모드다. `r3_nested_zones` / `r6_slalom_gates` /
  `r1_coastal`에서 AGL min과 env peak, L-BFGS exit code 분포를 봐야 한다.
- 거부면을 옮기면: 전 시나리오에서 `hard > 0`인 모든 경우가 `risk_max > 0`을 동반하는지
  확인한다. `hard > 0`인데 `risk_max = 0`인 줄이 남으면 거부면이 다시 moat 바깥으로
  나간 것이다.
- `zoneVisibleVolumeContains(1.0, **0.5**)`를 거부 판정에 쓰지 말 것. visibility가
  (0.35, 0.5]인 점은 `riskZoneValue`가 양수를 주는데 그 판정은 CLEAR로 읽는다.
  authored 판정은 반경만 1.0이고 visibility 하한은 0.35다.
