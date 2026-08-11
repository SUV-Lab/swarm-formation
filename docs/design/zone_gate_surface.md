# 하드 구역 거부면을 어디에 둘 것인가

`unflyable()`에 하드 구역 접촉 조건을 넣은 뒤(62e8a8e), 구역이 실린 A/B 시범에서
`r1_coastal_terrain_following` / `r1_dynamic_surface_obstacles`가 **반복마다 갈리는**
거부를 냈다. 같은 팔, 같은 미션, 연속된 두 실행이 `hard=21`(FAILED)과 `hard=0`(CLEAN)로
나뉜다.

원인을 파 보니 결함은 플래너에도 있고 **게이트 자체에도 있다.** 게이트 쪽부터 적는다.

## 1. 거부면과 복원력이 사라지는 면이 같은 면이다

| 무엇 | 값 | 한 곳에서 정의된 위치 |
|---|---|---|
| 접촉 판정면 (게이트) | `kZoneHardInflate = 1.05` | `dyn_a_star.h:735` |
| risk barrier의 바깥 support 끝 | `q >= 1.0 + kRiskBarrierRampFrac`, `= 0.05` | `poly_traj_optimizer.cpp:3905`, `poly_traj_optimizer.h:280` |
| risk 값의 정의역 | `if (!(q < 1.0)) return 0.0;` | `path_manager.cpp:2100` |

세 줄을 같이 읽으면 이렇게 된다.

- 게이트는 `q < 1.05`에서 접촉을 선언한다.
- 그 `q = 1.05`가 barrier의 바깥 끝이고, ramp가 smoothstep(`base_s = t²(3-2t)`)이라
  그 면에서 **값도 기울기도 0**이다.
- `q >= 1.0`에서 risk는 정의상 0이다. 그래서 `[ZONE-AUDIT]`이 `hard=21`과
  `risk_max=0.0000`을 한 줄에 인쇄한다. 모순이 아니라 **서로 다른 두 부피를 한 줄에
  찍은 것**이다.

즉 안쪽으로 미는 힘(시간·jerk·sqrvar)이 조금이라도 있으면 수렴 평형점은 구조적으로
`q < 1.05` 안쪽에 놓인다. 거부면이 그 평형점 바깥이 아니라 **평형점이 놓이는 바로 그
면**이다. 반복마다 갈리는 것이 당연하다.

## 2. 1.05는 무엇을 위해 도입된 값인가

`dyn_a_star.h:502-507`이 직접 적고 있다.

> ellipsoid inflated 5% and the LOS contour lowered to 0.35 ... so the zone-free
> geodesic stands OFF the visible rim instead of hugging it — rim-hugging routes
> parked the back-end in the steepest visibility gradients and lost the duck-below
> tug-of-war

**경로 생성용 standoff**다. 비행 안전 경계로 정의된 적이 없다. 바로 아래
`dyn_a_star.h:698-704`에는 nominal rim 판정(`zoneVolumeContains(i, pos, 1.0, 0.5)`)이
따로 있고, 그 주석은 *"it is NOT the contact gate"*라고 적는다.

62e8a8e에서 내가 한 것은 **그 standoff 면을 거부면으로 승격**시킨 것이다. mission yaml이
정의한 구역은 `reach = 100 u`(q=1.0)이고, 게이트는 그 5% 바깥 — 이 시나리오에서 500 m —
이다. r1은 authored 구역 밖의 margin 껍질을 스치고 FAILED가 됐다.

## 3. 경로가 그 면에 접해 있다

shortcut의 구역 판정은 여유 없는 boolean이다. `chordOccRisk`(`dyn_a_star.cpp:919-932`)가
chord를 `terrain_stride_floor_`까지 이분해 훑으면서 각 점에 `insideHardZoneVol(p)`를
그대로 적용한다. occupancy는 `checkOccupancy_esdf`로 정확 판정하고, risk와 cost에는
비교 여유(`kShortcutRiskMargin = 1.10`, `kShortcutMargin = 1.05`)가 있지만 이들은
**비용 비율 허용치이지 기하 standoff가 아니다.** 구역에는 어느 쪽도 없다.

greedy farthest-feasible 탐색은 제약이 binding이면 그 제약을 포화시킨다. 그래서 확정된
polyline이 접촉면에 접한다. 보관된 7개 실행에서 committed polyline의 zone[1]에 대한
최소 q는 전부 **1.0499 ~ 1.0501** 범위였다(로그의 A\* 정점 좌표로 재계산; 좌표가
0.01 u로 반올림 인쇄되므로 각 값의 포락은 ±0.00005).

반경 105 u 구면에서 4.2 u(21 sample)의 접촉 현을 만드는 데 필요한 안쪽 편차는 **2.1 m**다.
MINCO 모서리 둥글림 한 번의 크기다.

## 4. 침입 깊이 — 측정됨

`[ZONE-CONTACT]`를 넣고(6절 D) `manager/dyn_yaw_seed: 4155591226`으로 고정해 12회 돌렸다
(`ab_repro2`, pinned c59764b). 체인 8회(off 4 + on 4)가 **전부 같은 줄을 냈다**.

```
[ZONE-CONTACT] zone=1 t=[559.80, 561.80]s runs=1 q_min=1.04948 standoff-shell
               vis=0.921 at (3894.23, 2395.21, 1.25)
```

- **q_min = 1.04948.** authored rim(`reach = 100 u`, q = 1.0)에서 **바깥으로 494.8 m**,
  게이트면(q = 1.05)에서 **안으로 5.2 m**.
- 궤적은 mission yaml이 정의한 부피에 **들어간 적이 없다.** 앞서 로그 상한으로 얻었던
  "rim 안쪽 최대 134 m"는 상한일 뿐이었고, 실측은 rim 바깥에 머문다.
- 연속 구간 1개, 2.0 s. 흩어진 수치 잡음이 아니라 한 번의 접선 통과다.
- visibility = 0.921. 판정 하한 0.35에서 한참 위라 LOS 경계의 knife edge는 아니다.
  즉 이 접촉은 **기하 하나로 결정된다.**
- `risk_max = 0.0000`은 여기서 모순이 아니라 정의다. q ≥ 1.0이면 risk는 0이다.

즉 게이트는 **미션이 정의한 구역에서 반 km 떨어져 나는 비행을 FAILED로 거부하고 있었다.**

## 4-1. 반복마다 갈리던 것의 정체

seed를 고정하기 전 같은 팔이 CLEAN과 REJECTED로 갈렸다. 고정하니 체인 8회가 q_min
소수 다섯째 자리까지 동일하다. **비결정성의 출처는 `dyn_yaw_seed` 하나였고, back end는
결정적이다.** 5절에서 "route가 같아도 판정이 갈렸다"고 적었던 근거는 0.01 u로 반올림
인쇄된 정점 좌표 비교였고, 그 해상도로는 route 차이를 가릴 수 있다. 철회한다.

같은 seed에서 `difficulty_balance` off와 on이 **같은 접촉**을 낸다. 이 결함은 A/B 팔과
무관하다.

## 4-2. direct는 같은 seed에서 4/4 clean

같은 seed·같은 미션에서 direct 산출물은 4회 모두 hard = 0이다. 표본이 우연히 clean했던
것이 아니라 chained와 direct 사이에 실제 구조 차이가 있다. 원인은 아직 규명되지 않았다
(7절 2번).

## 5. 실행 간 차이의 출처

`optimizer_params.yaml:204`의 `manager/dyn_yaw_seed: -1`이 `path_manager.cpp`의
`std::random_device{}()` 분기를 타고, dynamic obstacle box의 spawn yaw를 매 프로세스
새로 뽑는다. yaw는 SDF collision primitive를 회전시키므로 speed map과 geodesic이 바뀐다.

로그가 뽑힌 seed를 인쇄하고 "이걸 pin하면 재현된다"고 적어 두는데, **그게 절반은
거짓이었다.** seed는 `%u`로 인쇄되는 uint32인데 읽을 때는 `int`였다. INT32_MAX를 넘는
값은 음수로 감겨 randomize 분기로 떨어진다 — 고정한 척하면서 새로 뽑고, 로그는 아무
말도 하지 않는다. 실제로 이 문서의 재현 시도가 그 값(4155591226)으로 12회를 돌렸고
12개의 서로 다른 seed가 나왔다. int64로 읽도록 고쳤다(c59764b).

운용 기본값으로는 의도된 것이다. 다만 A/B에서는 **off 팔과 on 팔이 서로 다른 장애물
배치를 본다** — 짝 비교가 제거하려던 요인이 짝 안에 남는다. 계통 편향은 아니지만
잡음이다. 2026-08-12 측정(176행)은 이 상태로 돌았고, 다음 재실행부터 하네스가
(rep, mission)별로 같은 seed를 양팔에 준다.

seed를 제대로 고정하면 체인 8회가 동일하다(4-1절). **seed가 유일한 출처였다.**

## 6. 선택지

`D`가 선행이다. `D` 없이는 `A`/`B`/`C` 중 무엇을 해도 효과를 확인할 수 없다.

### D. audit 계측 보강 — 동작 변경 없음

`[ZONE-AUDIT]`에 접촉의 zone index, t_first/t_last, 연속 구간 수, q_min과 그 위치,
그 지점의 visibility를 추가한다. 이미 도는 loop 안이라 계산량은 사실상 없다.

- 성공 판정: 4절의 "기록되지 않는다" 항목 3개가 한 번의 재현 실행으로 채워진다.

### C. 거부면을 authored rim으로 내리고, 1.0~1.05는 degrade로 보고

`unflyable()`의 하드 조건을 `zoneVolumeContains(i, p, 1.0, 0.35)` 기준으로 하고,
1.05 껍질 진입은 별도 카운터로 남겨 `STITCHED_ENVELOPE_BUDGET`처럼 등급만 낮춘다.

**이미 있는 `zoneVisibleVolumeContains`(1.0, **0.5**)를 그대로 쓰면 안 된다.**
`dyn_a_star.h:711-713`이 그 이유를 적고 있다 — visibility가 (0.35, 0.5]인 점은
여전히 양의 risk를 갖는데 그 판정은 CLEAR로 읽는다. 내려야 하는 것은 **기하 inflation
5%뿐이고 visibility floor는 0.35로 유지**해야 한다. 이걸 놓치면 거부면을 내리면서
동시에 진짜 위험한 밴드를 놓치게 된다.

- 비용: authored rim 바깥 500 m 대의 스침을 더 이상 거부하지 않는다. 이 standoff가
  비행 안전 요구사항이라면 이 안은 틀렸다 — 그런데 코드에도 문서에도 그렇게 적혀
  있지 않고, 1.05는 routing seed용으로만 정당화되어 있다. **의도가 한 곳에서 정의되어
  있지 않다는 것이 이 결함의 뿌리다.**
- 성공 판정: 전 시나리오 재실행에서 `hard > 0`인 모든 경우가 `risk_max > 0`을 동반하는지.
  하나라도 `risk_max = 0`인 hard가 남으면 두 부피가 여전히 어긋나 있는 것이다.

### A. shortcut의 구역 판정에 기하 standoff 부여

`chordOccRisk`/`probeRouteOk`가 1.05 대신 1.05+δ로 판정한다.

- 비용: **정량화되지 않았다.** 통로가 δ보다 좁은 시나리오에서 pass=1이 실패해 pass=2/3
  (soft fallback)로 강등되거나 미션이 거부된다. 위험군은 `r6_zone_slalom`,
  `r7_encircled_goal`, `r3_nested_zones`, `r3_goal_in_zone`.
- 위험: pass=2로 내려가면 구역을 soft로 통과하는 경로가 나온다. 지금보다 **나쁘다**.
  "임계값만 조이면 된다"가 실패하는 정확한 지점이다.
- 성공 판정: δ 후보 3개(0.005 / 0.01 / 0.02)로 전 시나리오를 돌려 (a) `pass` 분포 유지,
  (b) committed polyline q_min 히스토그램 이동, (c) 미션 거부 수 불변 — 셋을 같이 본다.

### B. barrier ramp를 게이트 밖으로 이동

`kRiskBarrierRampFrac`을 키워 ramp를 [1.0, 1.10]으로 하거나 band 자체를 옮긴다.

- 위험: `dyn_a_star.h:502-507`이 이미 경고한 실패 모드 — 구역에서 밀린 궤적이 terrain
  쪽으로 눌려 duck-below 싸움에서 지고 AGL이 무너진다. 그 주석은 측정치까지 적고 있다
  (pull 6-7x terrain restoring, terrain penetration, audit reject). ramp를 키우는 것은
  정확히 그 pull을 키우는 방향이다. **마지막에 검토한다.**

### E. 측정용 `dyn_yaw_seed` 고정 — 운용 기본값은 그대로

- 위험: 고정된 seed가 clean 쪽이면 회귀가 실패를 놓친다. 재현용 seed는 거부가 난 실행의
  것을 쓴다.

**권고 순서: D → 재현 측정 → C(+A). B는 마지막.**

C와 A는 배타적이지 않다. 게이트가 지키는 면, cost가 방어하는 면, route가 서는 면을 각각
다른 반경에 두고 겹치지 않게 하는 것이 의미상 가장 일관된다. 지금은 셋이 같은 면이다.

## 7. 아직 모르는 것

`manager/dyn_yaw_seed: 4155591226` 고정 상태에서 수행한다.

1. **왜 chained만 접촉하는가.** 같은 seed에서 direct 4/4 clean, chained 8/8 접촉.
   경로는 같은 front end에서 나오는데 최적화 결과가 다르다. 접점 arc ±10 u를 0.1 u로
   덤프해 committed polyline 대비 법선 편차의 크기와 부호를 봐야 한다.
   `[TERRAIN-PROFILE]`은 78.8 u 간격이라 4.2 u 접촉을 볼 수 없다.
2. 접점에서 committed polyline 자체의 q와 visibility. 3절의 접선 수치는 로그의 반올림된
   정점 좌표에서 재계산한 것이라 ±0.00005 포락이 있다. front end가 실제로 어느 쪽에
   서 있는지는 직접 인쇄해야 확정된다.
3. 접점에서 barrier가 실제로 얼마나 기여했는가. 1절의 수치는 소스에서 유도한 것이고
   측정값이 아니다.
4. A/B/C 각각의 미션 거부 비용. **어느 것도 측정되지 않았다.**

해결된 것: 접촉의 깊이·위치·연속성(4절), 실행 간 차이의 출처(4-1절), direct가 표본이
아니라 구조 차이라는 것(4-2절).

## 8. 확인했지만 원인이 아닌 것

- coarse cell aliasing — rasterization은 보수적이다(`insideHardZoneCell`이 half-cell
  키운 AABB + center/8-corner visibility probe, `dyn_a_star.h:533-567`). 자유 cell 중심은
  최소 105.6 u 밖이다.
- 정점 전용 보장 — chord는 `terrain_stride_floor_`까지 이분되어 훑린다.
- front end와 audit의 판정 함수 차이 — **같은 함수**다
  (`path_manager.cpp:3822` → `dyn_a_star.h:714-717`).
- 49 → 102 정점 세분화 — 선형 보간이라 새 기하가 없다(`path_manager.cpp:1643-1656`).
- seam / contract — 접점은 arc s≈1123 u, contract는 41.2 u와 1625.8 u. `[FINAL-EVAL]`이
  접촉을 전부 cruise span에 귀속시킨다.
- chain worker가 구역 목록을 덜 받음 — 받는다(`[RISK] Passed 2 risk zones to optimizer`가
  main + worker 3회).
