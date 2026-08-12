# 검토 인계 — 2026-08-13 (leg 정책)

## 0. 범위와 읽는 법

`review_handoff_20260812.md` 이후, 서브모듈 **`d97f8c4..732dcb5`(5커밋)**.
슈퍼프로젝트는 `af530ed..10ea8f8`.

`review_handoff_20260812.md`가 끝난 지점 = leg 스냅샷 설계를 승인받고 ①포착부터
짓기로 한 지점. 이 문서는 ①~③이다. ④는 **하지 않았다**(§3-1).

§1이 **이번 라운드 내 실수 전부**다. 그다음이 코드 변경, 열린 문제 순이다.

수치 출처 표기는 이전 문서와 같다: `[재현]` 명령 하나로 다시 나오는 값,
`[읽음]` 저장소/로그에서 읽은 값, `[내 독해]` 내가 코드를 읽고 내린 판단.

---

## 1. 내 실수

이번 라운드 실수의 **절반이 한 종류**다: 테스트가 대리 지표를 보고 있었고, 변이를
걸기 전까지 통과로 보였다. 지난 문서에서 같은 종류를 다섯 번 적었는데(§1-18, §1-21,
§1-31, §1-34, §1-36) 줄지 않았다. 그래서 이번에는 **모든 신규 단언에 변이를 걸었고**,
아래 1-2·1-4·1-5·1-6·1-7은 전부 그 변이가 잡은 것이다 — 리뷰가 아니라.

### 1-1. `legpolicy`의 첫 구역 배치가 비대칭을 만들지 못했다

leg마다 정책이 다르다는 것을 보이려고 중간 웨이포인트에 구역을 놓았다. 그런데 중간
웨이포인트는 **leg 1의 시작점**이다. 그래서 leg 0은 끝점 포함, leg 1은 시작점 포함으로
**둘 다 정당하게 SOFT_ENDPOINT**를 보고했다(`[LEG-POLICY] leg0=2 leg1=2`).

구역을 미션 목표로 옮겨서야 `leg0=0(HARD_AVOID) leg1=2(SOFT_ENDPOINT)`가 나왔다.
실행해서 알았다.

### 1-2. `legtags`가 필렛 접합부 split을 검증하지 않았다 — 그 분기가 아예 안 돌았다

leg 경계 필렛에 명시적 split 정점을 넣고, 그것을 고정하는 단언을 썼다. 통과했다.
**split을 지우는 변이도 통과했다.**

`manager/corner_fillet_radius`의 기본값이 `0.0`이다 [읽음: `path_manager.cpp:99`].
필렛 블록 전체가 `corner_fillet_radius_ > 1e-6` 뒤에 있으므로, 이 변형에서 필렛은 한
번도 실행되지 않았다. 내가 쓴 코드도, 그 코드를 지우는 변이도 죽은 코드였다.

변형에서 반경을 켜고 나서야 M-B(호 전체를 들어오는 leg로), M-D(나가는 leg로)가 죽었다.

### 1-3. 그 파라미터를 켜는 방법을 파일이 이미 적어 놨는데 안 읽었다

`force()`로 `manager/corner_fillet_radius`를 켰더니 abort 했다 —
`ParameterAlreadyDeclaredException`. `manager/*`는 PathManager 생성자가 declare 하므로
그 전에 declare 하면 충돌한다.

`chain_experiment_test.cpp:387`에 **이 상황이 정확히 그대로 적혀 있다**: "manager/* values
are read ONCE in the PathManager constructor, and force() before construction collides
with its declare_parameter — so variant-specific manager overrides ride NodeOptions
instead." 그 아래에 쓰는 자리까지 있었다. 읽고 시작했으면 안 겪었다.

### 1-4. 코너 하나로는 짝수 N 강제가 검증되지 않는다

접합부 필렛의 호 샘플 수를 짝수로 강제해서 `a = 0.5`(호의 apex)가 실제 정점이 되게
했다. 그런데 **그 강제를 지우는 변이가 통과했다** — 시험한 코너의 N이 이미 짝수였기
때문이다.

N은 `ceil(|P1-P2| / 3)`이라 코너 각도가 패리티를 정한다. 코너 4개로 늘리니 3개가
홀수였고 변이가 죽었다 [재현: `legtags`].

### 1-5. lead-in 삽입 미러링도 같은 죽은 코드였다 — 한 세션에 같은 실수를 두 번

최적화기가 `clean_path`에 삽입하는 두 지점(퇴화 중점, lead-in)에 태그 미러링을 넣고
`legtags`로 덮었다고 생각했다. `pieces = route - 1`이 찍히는 것을 보고서야 lead-in이
발화하지 않았음을 알았다.

`optimization/lead_in_time`도 `0.0`으로 출하된다 [읽음: `optimizer_params.yaml:545`,
주석 "dormant on main"]. §1-2와 **완전히 같은 형태**의 실수를 같은 세션에서 두 번 했다.

`legleadin` 변형을 따로 만들어 켰다. 그제서야 M-F(미러링 삭제)가 죽었다.

### 1-6. `legchain` 1차 — 체인은 다중 웨이포인트를 체인하지 않는다

체인된 세그먼트가 provenance를 받는지 보려고 다중 웨이포인트 미션을 썼다. `legs=2
piece_leg=14`가 찍혔고 통과했다. **provenance를 nullptr로 바꾸는 변이도 통과했다.**

`planImpl`이 자기 주석에 적어 놓았다 [읽음: `segment_chain_planner.cpp:442`]:
"Stage-1 scope: one goal. Multi-waypoint missions ... fall back to the single-shot plan."
체인이 아예 안 돌았고, 내가 읽은 숫자는 체인이 먼저 계산하는 **단발 베이스라인**의
것이었다. `[CHAIN]` 로그가 한 줄도 없다는 것이 증거였는데 처음엔 찍어보지 않았다.

**이것이 이 라운드에서 가장 중요한 발견이다**: 체인과 다중 leg는 직교한다. 체인 미션은
leg가 항상 1개다. 그래서 per-leg 귀속이 실제로 값을 내는 곳은 체인이 아니라
**다중 웨이포인트 단발 경로**다(§2-5).

### 1-7. `legchain` 2차 — 단일 목표로 고쳐도 여전히 대리 지표였다

단일 목표로 바꾸니 체인은 돌았다. 그런데 nullptr 변이가 **또 통과했다**. 슬라이스를
상속받은 세그먼트와, 슬라이스가 안 되어 자기 구간 front end를 다시 돌린 세그먼트를
단언이 구분하지 못했기 때문이다. 후자도 태그를 새로 만들어 낸다.

front-end epoch을 1로 고정하는 단언을 추가하고서야 구분됐다. epoch이 1이라는 것은
"전체 체인 미션 동안 front end가 한 번만 돌았다" = "모든 세그먼트가 같은 커밋된 경로를
상속했다"는 뜻이다.

### 1-8. "더 엄격한 정책"을 leg 번호의 최솟값으로 썼다

piece 경계 샘플은 양쪽 정책 중 더 엄격한 쪽을 따라야 한다. 나는
`leg = std::min(leg, prev_leg)`라고 썼다. **더 작은 leg 번호는 더 엄격한 정책이 아니다.**
앞 leg가 더 관대할 수도 있다.

zone별로 어느 쪽이든 HARD_AVOID면 HARD_AVOID를 취하는 병합으로 고쳤다. SOFT_* 사이에는
순서를 만들지 않았다 — 감사가 읽는 구분은 HARD_AVOID 하나뿐이라, 순서를 지어내면 아무도
안 읽는 순서가 된다.

### 1-9. 비계용 구조체 두 개를 커밋했고 둘 다 필요한 물건이 아니었다

①에서 `TaggedRoute`와 `PiecePolicyMap`을 "다음 단계용"으로 선언해 커밋했다. 실제로
경계에 필요했던 것은 `RouteProvenance`(태그 + epoch 포인터)였고, piece 쪽은 그냥
`last_piece_leg_` 벡터였다. 두 구조체 모두 두 커밋 뒤에 지웠다. 쓰이지 않는 선언을
커밋한 것 자체가 실수다.

### 1-10. 그 밖의 기계적 실수

- `RouteProvenance{}`를 기본 인자로 썼다 → "default member initializer required before
  the end of its enclosing class". 포인터 기본값으로 바꿨다.
- `RouteProvenance`를 `planGlobalTraj` **아래에** 선언했다가 매개변수 타입으로 못 쓴다.
  지난 라운드에 `LegPolicySnapshot`으로 똑같이 당했는데 또 했다 — 다만 이번엔 커밋 전에
  빌드해서 잡았다.
- `force("manager/allow_unmeasured_zone_policy", 1.0)` → yaml이 bool로 쓰므로 abort.
- `[LEG-POLICY]` 폴백 경고를 처음에 너무 넓게 걸어서, 구역이 있는 **모든** 체인 단발
  미션마다 찍혔다. 그 경우 plan-wide 스냅샷은 폴백이 아니라 정답이다. 둘 다 없을 때만
  찍도록 좁혔다.

### 1-11. ②c의 정당화를 처음에 과장했다

슬라이스 provenance를 "다중 leg 귀속을 가능하게 하는 것"으로 쓰려 했다. §1-6에서 밝혀졌듯
체인 미션은 leg가 1개라 그렇지 않다. 실제 값은 "상속 경로에서 capture가 살아남는다"까지다.
커밋 메시지는 고쳐서 넣었지만, 설계 자체를 그 전제로 진행했다.

---

## 2. 코드 변경

### 2-1. ① leg마다 정책 포착 (`29f73a8`)

`captureLegPolicySnapshot(leg, serial, out)` — 그 leg의 검색이 아직 searcher 상태를
쥐고 있는 동안 포착한다. 다음 leg가 한 줄 뒤에 덮어쓴다.

- 실패한 leg가 하나라도 있으면 `leg_policies_.clear()` — **구멍 뚫린 집합은 집합이 아니다**
- `zonePolicySnapshot()`의 "검색 1회" 규칙은 **그대로 뒀다**. 그것은 "plan 전체에 하나의
  정책이 있는가"라는 다른 질문이고, 다중 leg에 대한 정직한 답은 여전히 "없다"이다
- 회귀 `legpolicy`: 같은 구역이 leg0 HARD_AVOID / leg1 SOFT_ENDPOINT
- 변이: 마지막 leg의 정책을 전 leg에 복사 → 2개 단언 사망

### 2-2. ②a route edge provenance (`d744320`)

`edge_leg[i]` = route edge `i`(정점 i→i+1)를 만든 leg. **기하와 함께 만들고, 나중에
재유도하지 않는다.** 재유도(최근접 투영)는 U턴·자기교차·두 leg 근접 주행에서 틀린다.

세 변환을 통과한다:
1. leg 연결 — push 하나가 edge 하나이므로 태그 하나
2. 코너 필렛 — 호 샘플마다 태그. **leg 경계 필렛은 명시적 split**: 샘플 수를 짝수로
   강제해 `a = 0.5`(apex, 사라진 접합 정점의 대역)가 실제 정점이 되고, 두 반쪽이 각각
   별개 piece가 된다. 없으면 한 piece가 두 정책에 걸친다
3. piece 경계 조밀화 — 부모 edge의 leg를 그대로 물려받는다

크기가 안 맞으면 통째로 버린다. 빈 태그는 "출처 불명"이지 "leg 0"이 아니다.

**같이 고친 것**: `planGlobalTraj`의 상속 경로는 검색을 안 돌리므로 `planFrontEnd`
맨 위의 epoch 리셋에 도달하지 않는다 → **이전 계획의 leg capture가 그대로 남아 있었다.**

- 회귀 `legtags`: 코너 4종에 대해 태그 길이·단조·시작 leg 0·끝 leg 1·인계 1회·
  인계 정점이 접합 웨이포인트 최근접 정점
- 변이 5종 전부 사망 (§1-2, §1-4 포함)

### 2-3. ②b edge 태그 → piece 태그 (`1c4c33d`)

route edge i = MINCO piece i (`piece_num = clean_path.size() - 1`, L-BFGS는 개수도 순서도
안 바꾼다). 그래서 궤적을 무엇에도 재투영할 필요가 없다.

- 최적화기의 `clean_path` 삽입 2곳(퇴화 중점, lead-in)에 미러링. 둘 다 `begin()+1`이므로
  edge 0을 쪼개고, 두 반쪽은 같은 leg
- **나가는 궤적의 piece 수와 안 맞으면 지도를 아예 안 준다.** L-BFGS가 개수를 바꾸지
  않기로 되어 있다는 것은 감사의 근거가 못 된다
- 회귀 `legleadin`(§1-5), `legtags` 확장
- 변이 2종 사망

### 2-4. ②c 상속 경로 provenance (`3abbf8c`)

상속 경로는 태그를 만들 수 없으므로 호출자가 **태그와 그것을 만든 epoch을 함께** 준다.
epoch이 쓸모의 근거다 — 그 사이 front end가 안 돌았다는 증명이고, 따라서
`leg_policies_`가 이 태그가 가리키는 leg를 여전히 설명한다는 증명이다.

같은 태그라도 epoch이 한 세대 낡으면 거부한다. 인덱스 벡터 자체에는 어느 검색에서 왔는지
쓰여 있지 않다.

- `sliceCommittedRoute` — 계약 절단은 edge 하나를 쪼개고 양쪽이 그 leg를 유지
- `cutAtArc` — 살아남는 꼬리는 태그 리스트의 suffix
- 회귀 `legchain`(§1-6, §1-7), `legtags`의 epoch 양방향
- 변이 3종 사망

### 2-5. ③ 감사가 piece마다 정책을 읽는다 (`732dcb5`) — 이번 라운드의 핵심

**다중 웨이포인트 미션은 구역이 어디에 있든 전부 거부되고 있었다.** 경로에서 아무리
멀어도. 감사가 볼 수 있던 것 기준으로는 옳은 거부였다 — plan-wide 스냅샷이 다중 leg에
대해 INVALID이므로.

측정 불가였던 적이 없다. **piece 단위로 물으면 된다.** 샘플이 속한 piece가 그때 날고
있던 leg를 말하고, 그 leg의 포착된 정책이 유효한 정책이다.

- piece 경계 샘플은 양쪽 중 HARD_AVOID를 취한다 (§1-8)
- 전제조건은 전부 거부지 복구가 아니다: 이 궤적을 설명하지 않는 지도, 포착 없는 leg,
  유효하지 않은 포착, 구멍/중복 serial, 현재 구역을 안 덮는 정책
- **stitched 비행은 구조적으로 여기 걸린다** — 지도가 마지막 세그먼트만 설명하고 piece
  수가 안 맞는다. 그 불일치는 덮으면 안 되는 것이다
- 필수 회귀 `legaudit`: 구역이 **미션 시작점**에 있다. leg 0은 SOFT_ENDPOINT 포함 면제,
  leg 1은 같은 구역을 HARD_AVOID. 비행은 **반드시** authored 부피 안에서 시작하므로 초기
  샘플은 무조건 접촉이고, 남는 질문은 어느 leg가 판정하느냐뿐이다.
  `manager/allow_unmeasured_zone_policy`는 출하 기본값 false 그대로 둔다
- 변이 4종, 각각 다른 문장으로 사망:
  - 마지막 leg 정책으로 전체 판정 → `authored zone entered` (미션이 스스로 authored 한
    출발을 거부)
  - per-piece 비활성화 → 예전 전면 거부 문장
  - 첫 leg 포착 누락 / serial 중복 → 조용한 오답이 아니라 같은 거부로 폴백

**`wpzone`은 옛 계약을 고정하고 있어서 옮겼다**(지운 게 아니다). 이제 "날고, 측정된다"를
고정한다. 옛 거부는 `wpzonepass0`로 옮겼다 — 3-pass 구역 정책을 꺼서 어떤 leg도 포착을
못 만드는, 귀속이 **진짜로** 불가능한 경우다. opt-in·outcome·기계 판독 reason·detail
문자열에 대한 단언은 전부 그대로 옮겨 갔다.

### 2-6. 회귀 상태 [재현]

```
61/61 chain variants
risk harness 154/0
start_claim / transition_experiment / waypoint_experiment / terrain_risk_mask : PASS
```

신규 변형 5종: `legpolicy` `legtags` `legleadin` `legchain` `legaudit` `wpzonepass0`.

---

## 3. 열려 있는 것

### 3-1. ④를 하지 않았다 — 오늘 도달할 수 없는 경우이기 때문이다

Codex의 ④는 "전이 prefix piece는 첫 leg 정책을 명시적으로 상속, `route_start_s` 절단은
첫 호 구간만 clamp"다. 절단 쪽(`cutAtArc` 태그 suffix)은 §2-4에 들어갔다. **prefix 쪽은
안 했다.**

**시작했다가 되돌렸다.** `solveSlice`에 태그 in / piece 지도 out을 달고 stitched 지도를
짜기 직전에, 소비자가 있는지부터 확인했다. 없다:

- stitched 비행이 나오는 경로는 두 개뿐이다 — `planImpl`의 체인, `planOverRoute`의
  route/parallel
- 둘 다 **다중 웨이포인트 미션에 도달하지 않는다.** `planImpl`의 가드
  [읽음: `segment_chain_planner.cpp:444`, `if (waypoints.size() != 1)`]가
  route 모드 디스패치(같은 함수 508행)보다 **먼저** 걸려 단발로 빠진다
- 확인함 [재현]: `chain/author_from_route=true` + 웨이포인트 2개 + 구역 →
  `detail=multi-waypoint mission — chain not attempted`. route 경로에 자체 가드는
  없지만 거기까지 가지를 못한다
- 단일 목표 stitched 비행은 leg가 1개이고 plan-wide 스냅샷이 valid다. 거기서 per-piece
  귀속은 아무것도 더 말해주지 않는다

즉 stitched piece 지도는 **오늘 존재하지 않는 미션 형태**를 위한 기계다. 그것이
필요해지는 조건은 line 444의 가드를 걷어내는 것 — 그 주석이 말하는
"waypoint-to-span assignment that does not exist yet" — 이고, 그건 별개의 더 큰 작업이다.

그래서 훅까지 지웠다. §1-9에서 "쓰이지 않는 선언을 커밋한 것 자체가 실수"라고 적어
놓고 같은 커밋에 소비자 없는 매개변수를 남기는 것은 앞뒤가 안 맞는다.

**다음에 ④를 할 때의 범위**(가드를 걷어낸 뒤): `solveSlice`에 태그 in / 지도 out,
세그먼트별 지도를 순서대로 연결, 전이 prefix piece에 첫 leg 명시 부여, terminal piece에
마지막 leg 부여, 병합 경로(2슬라이스 merge)와 구조 경로(rescue)에서 지도를 만들거나
명시적으로 무효화. 그 중 하나라도 빠지면 조용히 어긋난 지도가 생긴다 — 지금처럼
"귀속 불가 → 거부"인 편이 낫다.

### 3-2. 기본 구성에서 잠자는 코드

- `manager/corner_fillet_radius = 0.0` — 필렛 OFF. 접합부 split 경로는 출하 구성에서
  동작하지 않는다. `legtags`가 NodeOptions로 켠다
- `optimization/lead_in_time = 0.0` — lead-in OFF. 삽입 미러링도 마찬가지.
  `legleadin`이 켠다
- 퇴화 중점 삽입은 `clean_path.size() < 3`일 때만 — 실제 경로에서는 안 걸린다.
  **미러링에 회귀가 없다** (§3-3)

### 3-3. 회귀가 없는 것

- 퇴화 중점 삽입의 태그 미러링 (위)
- `cutAtArc`의 태그 suffix — 변이(off-by-one)를 걸었는데 `transition`/`transitionauto`
  변형이 안 죽었다. 전이 경로가 태그를 **아직** 소비하지 않기 때문이다(§3-1).
  ②c에 들어간 이 코드는 소비자가 생길 때까지 검증되지 않은 채로 있다. 지우지 않은
  이유는 절단 로직 자체가 태그와 기하를 같이 다뤄야 옳고, 나중에 따로 붙이면 그때
  같은 실수를 다시 하기 때문이다 — 다만 **검증되지 않았다는 사실은 그대로다**

### 3-4. 유지되는 제약

- main 머지 **보류**. 작업은 `lgh/dev`(슈퍼) / `mmp_dev`(서브모듈)
- 빌드는 컨테이너 안에서만, `docker exec -u 1000:1000`
- 모델 기반 결과는 전부 "프로젝트 정의 중립 3DOF 벤치마크 기반 구조 검증이며 실제
  플랫폼 물리 검증이 아니다"

---

## 4. 방법론 — 이번에 바꾼 것

지난 문서에서 "테스트가 대리 지표를 본다"를 다섯 번 적고도 이번에 다섯 번 더 했다.
차이는 **전부 변이가 잡았다**는 것뿐이다. 그래서 이번 라운드부터 규칙을 하나 고정했다:

> 신규 단언은 그것이 고정한다고 주장하는 코드를 **지우거나 뒤집는 변이로 죽는 것을
> 확인하기 전까지** 통과로 보고하지 않는다.

이 규칙이 §1-2, §1-4, §1-5, §1-6, §1-7을 잡았다. 리뷰는 하나도 못 잡았을 것이다 —
다섯 건 전부 코드가 맞고 테스트가 틀린 경우였고, 코드를 읽어서는 보이지 않는다.

부족한 점: 변이를 **내가 고른다.** 내가 상상하지 못한 실패 모드는 여전히 안 잡힌다.
§3-3의 두 항목이 그 증거다 — 변이를 걸었지만 죽일 회귀가 애초에 없었다.
