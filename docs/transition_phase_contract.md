# TransitionPhase 설계 계약 v3 (계약 2 — 문서만, 코드 없음)

- 상태: **설계 초안 / 전환 모델 스펙 대기.** 인터페이스 방향은 리뷰로
  수렴했으나(2026-08-06), §8의 수치와 §11의 모델 결정이 확정되기 전에는
  어떤 골격 코드도 만들지 않는다 — 수치 없이 굳힌 인터페이스는 잘못 굳는다.
- 채택 근거: 실제 요구가 "순항 영역 밖 초기 상태에서 시작해 순항 궤적으로
  연결"로 확정됨(2026-08-06). 초기 벡터를 영역 안으로 바꾸는 것은 계약 1
  데모일 뿐 요구 해결이 아니다.
- **필요한 것은 두 종류다**: 한계 수치(§8)는 "무엇을 허용하는가"만 말하고
  상태를 시간 전개하지 못한다. 전개할 **동역학 모델**(§11)이 함께 있어야
  전이 생성기가 성립한다.
- 전제(계약 1, 구현 완료): 명시 초기 상태의 **PVA 전체** 판정
  (`PathManager::pvaEnvelopeProblem` — 속력·상승각 영역 + 역동역학
  하중·추력·뱅크, 판정 불가 시 거부) → `INITIAL_MODE_UNSUPPORTED` FAILED;
  phase 모드 direct fallback의 비행 적합성 게이트(fail-closed: 평가 불가도
  FAILED) → `DIRECT_FALLBACK_UNSAFE`; 유한성 검사는 모델 off 조기 반환보다
  **앞**. 회귀 31종 전부 통과 (initfail / initaccfail / initnan / initok /
  synthclamp / unsafedirect / departop 포함).
- 계약 1 라이브 마감 (2026-08-06, full_map r5 회랑):
  - 콘 안 정상 상태 (189.5 m/s, 상승각 7.3°, 가속도 0): 계획 성공 —
    departure @s=32.8 u, arrival @s=2203.8 u, 205조각 1111.3초, 판정 CLEAN,
    총 1057 ms. 로그 `replan_fsm_drone_0_20260806_081440.log`.
  - 콘 밖 (상승각 32°): 명령 수신과 **같은 밀리초**에
    `INITIAL_MODE_UNSUPPORTED` 거부, FM·옵티마이저 미가동. 로그
    `replan_fsm_drone_0_20260806_083349.log:73`.
  - 부수 확인: r5 원본의 초기 가속도 `[3, 0, 2] m/s²`는 161 m/s에서
    최대 추력의 116%를 요구한다 — 상승각을 콘 안으로 낮춰도 PVA 판정이
    거부한다. z만 바꾸는 것으로는 부족하다는 실측 근거.

## 1. 역할 정의 — 두 "출발"의 의미 분리

- TransitionPhase: 순항 모델 **밖**의 초기 PVA를 유효 영역 **안**으로
  데려오는 구간. 전용 전환 모델·한계 집합으로 계획하고 검사한다.
- 기존 departure(체인 1구간): 유효 영역 **안**의 상태를 전역 경로에
  정렬하는 구간. 순항 모델 그대로.
- 겹치지 않는다. TransitionPhase의 종료 상태가 곧 departure가 시작할 수
  있는 상태다.
- 근거 결함: 상승각 32°(콘 ±30°) 초기 벡터가 계약 1 이전에는 직행 강등,
  그 이전에는 548.7 u 벌레 궤적 SUCCESS
  (logs/runtime/replan_fsm_drone_0_20260806_054645.log, 피크 추력 299.2%는
  054808 · 064053 로그).

## 2. 실행 분기 계약 — 시작 상태 분류

계획 진입 시 명시 초기 PVA를 세 모드 중 하나로 분류한다. 문자열 반환이
아니라 열거형 판정이어야 한다(현행 `pvaEnvelopeProblem`의 문자열은 계약 1
의 통과/거부 이분법용 — 계약 2 구현 시 분류기로 대체).

```
CRUISE_VALID          순항 영역 안
  → TransitionPhase 생략, 기존 체인 그대로 실행
TRANSITION_REQUIRED   순항 영역 밖 + 전환 영역 안
  → 전역 경로 → TransitionPhase → 경로 슬라이스 → 기존 체인
UNSUPPORTED           전환 영역까지도 밖
  → 즉시 FAILED(INITIAL_MODE_UNSUPPORTED)
```

- 계약 1의 현재 동작은 "TRANSITION_REQUIRED 미구현 → UNSUPPORTED와 동일
  취급"이다. 계약 2가 켜지면 가운데 분기만 새로 열린다.

## 3. 파이프라인 (수렴 구조)

```
[실제 초기 PVA]
   ↓ 분류기: TRANSITION_REQUIRED     (전역 FM 경로는 이미 존재)
전이 생성기 — 전환 모델·한계로 시간 궤적 PVA(t) 생성
   목표 = 전역 경로 위 진입점 (§5)
   ↓
다항 어댑터 — Hermite/MINCO 구간 구성 (§6 내부 검사 필수)
   ↓
[2단계, 후속] 참조 추종 L-BFGS — §7 계약
   ↓
단계별 평가기 (§4) — 전환 구간은 전환 모델로
   ↓ 종료 PVA = prescribed head
SegmentChainPlanner — 경로 슬라이스 [route_start_s .. goal] 재계획
```

- 구현 순서 확정: **1단계 = 최적화 없는** 전이 생성기 → 다항화 → 검증 →
  C² 체인 연결. L-BFGS 참조 추종은 별도 2단계 비교 실험으로 추가.

## 4. 단계별 평가기 — "공용 검사"라는 말을 버린다

한 평가기가 전 구간을 검사할 수 없다. 단계마다 모델이 다르다:

| 대상 | 판정 모델 |
|---|---|
| TransitionPhase 구간 | 전환 모델·전환 한계 (§8) |
| departure/cruise/arrival | 순항 모델 (기존 per-solve 감사 그대로) |
| 접합부 (전이→체인, 체인 내부) | PVA 오차 + jerk 점프 크기·상한 (§6) |
| 전체 결합 궤적 | 지형·구역·시간·수치 건전성 |

- **로깅과 판정 분리**: 현행 `logFinalEvaluation`은 로그+판정 겸용이고,
  스티치된 체인은 그 판정을 버린다(정보성). 계약 2에서는 판정 전용
  `evaluateFlight()`를 분리해 위 표의 네 판정을 모두 반환하고, 로그는
  그 결과를 출력만 한다. direct fallback 게이트도 이것을 쓰도록 이관.
- 모든 판정은 fail-closed: 평가 불가 = 부적합.

## 5. 접속 계약 — 경로 슬라이스와 prescribed head

- 공간 정합은 "경로 우선": 전역 FM 경로를 먼저 만들고 전이 생성기가 그
  경로 위 진입점을 목표로 삼는다. 경로는 전역 1회 결정(r3 교훈). 투영·
  잔여 경로 재생성 방식은 채택하지 않는다.
- 진입점 좌표는 **호장 `route_start_s`** 로 확정한다(체인 접합이 호장
  기반이므로 일관, 진입점이 기존 vertex 위일 필요도 없다). vertex index는
  파생값으로만 쓴다.
- 체인 입력은 전역 경로 전체가 아니라 진입점부터 잘린 슬라이스다:

```
전역 경로  [start ...... s=route_start_s ...... goal]
                          ↑ TransitionPhase 종료
체인 입력  = 경로[route_start_s .. goal]
prescribed head = TransitionPhase 종료 PVA
```

- 슬라이스 없이 전체 경로를 넘기면 체인이 원래 시작점부터 departure를
  다시 만들어 전이 구간과 중복된다 — 금지.
- 중기 궤적은 "있는 것에 붙이는" 게 아니라 전이 종료 PVA를 head로 받아
  그 시점에 다시 푼다.

### 요청/결과 타입 (구현 시 이 형태로)

```
TransitionRequest  { initial_pva, global_route(+cap), entry_candidates[]
                     (기존 선별 규칙 재사용), transition_limits }
TransitionResult   { verdict(OK/FAILED+사유), transition_traj(PVA(t)),
                     end_pva, route_start_s, audit(전환 평가기 결과) }
```

## 6. 다항 어댑터 — 구간 내부 검사와 접합 감사

- Hermite 변환은 근사다. 샘플 점 일치가 샘플 사이 준수를 보장하지 않는다.
- 고정 간격 샘플링만으로 끝내면 안 된다. 둘 중 하나 이상:
  - 다항 극값 검사: 검사량(속력², 상승률 등)을 미분해 구간 내 극값을 직접
    판정.
  - 적응형 세분화: 위반 여유가 얇은 구간을 재귀 분할, 수렴까지.
- 접합 연속성: 체인은 C² (P/V/A ~1e-13). C³은 보장하지 않는다. 전이→체인
  접합의 jerk는 "연속 보장"이 아니라 **좌/우 극한 jerk 차 노름의 점프
  감사 + 상한 판정**(상한은 §8 항목)으로 정의한다.

## 7. 참조 추종 최적화 계약 (2단계, 후속)

- 시작·종료 PVA: hard (경계 조건, 이동 금지).
- 전이 원형(생성기의 PVA(t)): reference 비용 — L-BFGS가 전이 알고리즘의
  의도를 지우지 못하게.
- 전환 한계 집합: gate — 순항 한계가 아니라 전환 전용 수치로 판정.
- 시간 배분: 제한적 조정만 허용 (원형의 시간 구조 보존).
- 1단계(무최적화)와의 품질 비교 실험으로 채택 여부를 결정한다.

## 8. 미확정 — 전환 모델 스펙에서 올 수치 (자리 이름만 정의)

- 전환 허용 속력 구간 (순항 122–230 m/s 밖 어디까지)
- 전환 허용 상승각 (순항 콘 ±30° 밖 어디까지)
- 전환 하중·추력 가정
- 접합 jerk 점프 상한
- 전이 완료 판정 (§9의 겹침 영역 체류 조건 수치: 속력 수렴 폭, 접선 정렬
  각, 체류 시간)

이 수치들이 오기 전에는 생성기·어댑터·평가기 어느 것도 구현하지 않는다.

## 9. 순항 진입 조건 — 모델 겹침 영역과 margin 배치

현재 코드에 섞여 있는 경계를 역할별로 확정한다:

| 값 | 역할 |
|---|---|
| 122 m/s (모델 speed_min) | 순항 모델의 원(raw) 하한 — 감사·평가 기준 |
| 132 m/s (margin 포함 floor) | **시작 상태 경계** — 계약 1 검증과 합성 시작 클램프가 쓰는 값. 전이 **종료 상태도 이 값 이상**이어야 한다 (체인 head가 되는 순간 같은 검증을 받으므로) |
| 230 m/s (모델 speed_max) | 상한 — margin 없음, 시작·감사 동일 |
| ±30° (비행경로각) | 콘 — margin 없음, 시작·감사 동일 |

- 전이 완료는 경계에 "걸치는" 것이 아니다: **순항 모델과 전환 모델이
  동시에 유효한 겹침 영역**에 들어와 §8의 체류 조건(속력 수렴 폭·접선
  정렬·체류 시간)을 채워야 완료다. 겹침 영역의 하한은 위 표의 margin
  포함 floor를 쓴다 — 전이가 122 m/s 언저리에서 끝나면 체인 head 검증이
  즉시 거부하는 모순을 막는다.

## 10. 실패 의미론과 테스트 계약

실패 사유(구현 시 PlanReason 확장 또는 TransitionResult.verdict):

- `INITIAL_MODE_UNSUPPORTED` — 분류기 UNSUPPORTED (전환 영역까지도 밖)
- `TRANSITION_GENERATION_FAILED` — 전이 생성기가 진입점까지 못 감
- `TRANSITION_ADAPTER_UNSOUND` — 다항화 내부 검사 실패 (극값/세분화)
- `TRANSITION_JUNCTION_UNSOUND` — 접합 PVA 오차 또는 jerk 점프 상한 초과
- 전환 영역에서 direct fallback은 없다 — 순항 플래너는 그 영역을 못 푼다.
  위 사유는 전부 미션 FAILED다.
- 전이 성공 + 체인 강등 → 체인의 기존 DEGRADED 의미론 그대로.

회귀 계약 (구현 시 하니스 변형으로 추가):

1. CRUISE_VALID 입력 → TransitionPhase 생략, 기존 체인과 결과 동일
2. TRANSITION_REQUIRED 입력(예: 상승각 32°) → 전이+슬라이스 체인, 접합
   C² + jerk 상한, 전이 구간은 전환 한계로만 판정
3. UNSUPPORTED 입력 → 즉시 FAILED, FM/옵티마이저 미호출 (injection 트립와이어)
4. 슬라이스 검증: 체인 시작 = route_start_s, 전이 구간과 경로 중복 없음
5. 전이 종료 상태가 겹침 영역 밖(<132 m/s 등)이면 접속 거부

## 11. 전환 동역학 모델의 출처 (코드 조사로 확정, 2026-08-06)

### 11.1 결정적 사실 — 순전파가 저장소에 없다

- `mmp_vehicle_dynamics`는 **역동역학 전용 라이브러리**다. 공개 함수 7개
  (`parametersAreValid`, `airDensity`, `evaluateInverseDynamics`,
  `flightEnvelopePenalty`, `envelopeUtilization`, `isWithinEnvelope`,
  `envelopeLimitName`) 중 **시간을 진행시키는 것은 하나도 없다**.
  "PVA를 주면 필요한 힘"만 답한다.
- 유일한 RK4 순전파(`dynamics_sim_node.cpp:359 integrateRk4`)는 노드
  클래스의 private 멤버이며 실행 파일에만 컴파일된다. 설치 헤더에도,
  Python 바인딩에도 없다. 게다가 `derivative()` → `controller()` →
  `sampleReference()` 구조라 **기존 PolyTraj가 있어야 도는 추종기**다 —
  생성기로 쓸 수 없다.
- ⇒ **전이 생성기의 순전파는 새로 만들어야 한다.** "적분기를 재사용하면
  된다"는 전제는 사실이 아니다. (3DOF 운동방정식 자체는
  `dynamics_sim_node.cpp:331-342`가 올바르므로 들어낼 수 있으나, 그 주위의
  컨트롤러·클램프·준수직 가드는 못 쓴다.)

### 11.2 결론: 별도 `TransitionDynamics` (기존 Parameters 확장 불가)

파라미터 값만 바꿔서는 못 넘는 구조적 장벽 넷:

| 장벽 | 근거 |
|---|---|
| `speed_min_mps`가 이중 역할 | 실속 한계이자 q-floor 정규화항 (`flight_dynamics.cpp:169`). 전환용으로 낮추면 CL·유도항력의 1/\|v\|² 특이점 보호가 사라진다 (측정: V=60에서 CL 1.21 vs 참값 5.02, 속도 그래디언트 노름 9.6e2 → 4.1e4) |
| 준수직 표현 불가 | `parametersAreValid`가 `flight_path_angle_max_rad ≥ 90°`를 하드 거부 (`:362`, 실행 확인). 어떤 파라미터 조합으로도 수직 발진을 표현할 수 없다 |
| 질량 일정·연소 없음 | 저장소 전체에 mdot/Isp/추진제 없음. 부스트는 질량 분율이 크게 변해 T/W와 가용 하중이 배 이상 달라진다 |
| 법선력 채널이 날개 양력뿐 | `:185-187`이 법선 비추력 전체를 날개 CL로, `:225`가 추력을 속도 접선으로 배정. TVC·고받음각·저동압 부스트가 "불가능한 날개 양력 요구"로 환산된다 |

정량 근거: 순항 속도에서 **상승각 32°·가속도 0**만으로도 이미 7947 N =
`thrust_max`의 2.48배를 요구한다. 순항 파라미터 집합은 잘못 튜닝된 게
아니라 **다른 비행 모드를 기술하고 있다**.

단, 통째로 분기하지는 않는다:

- `Parameters`(대기·항력극선·형상)는 공용 기반으로 유지.
- `TransitionDynamics`는 그 상위집합 — 질량 연소, 마하 의존 CD0,
  추력/법선력 채널 추가.
- **겹침 영역 축약 시험 필수**: 속력 ≥ speed_min, |γ| < 30°, T < thrust_max
  영역에서 `evaluateInverseDynamics`와 비트 단위로 일치할 것. 이 시험이
  "모델 권위는 한 곳"이라는 원칙을 지킨다.

### 11.3 검증 기준(참조 모델) — 오늘 쓸 수 있는 것이 없다

- 역동역학 모델의 플로어 이하 CL·항력은 허구(측정 4.13배·3.45배 과소).
- `isWithinEnvelope`/`envelopeUtilization`은 정의상 순항 영역을 인코딩.
- `dynamics_sim_node`의 RK4는 순항 한계로 포화된 추종기.

⇒ 참조는 2단으로 새로 만든다:

1. **전이 내부**: 전환 EOM의 독립 RK4 순전파를 생성 궤적에 대한
   **잔차**로 검증.
2. **인계 지점**: `PathManager::pvaEnvelopeProblem`을 **그대로** 재사용 —
   이미 "순항 체인이 이 상태를 날 수 있는가"의 권위 있는 판정자이고,
   오늘 `INITIAL_MODE_UNSUPPORTED`를 올리는 바로 그 게이트다. 전이 종료
   상태에서 이 함수가 빈 문자열을 반환하는 것이 곧 인계 성공의 정의다.

### 11.4 구조적 선례와 그 한계

- `TerminalPhase`(prescribed 나선 하강)가 구조 템플릿: 옵티마이저 없이
  기하를 생성해 같은 `poly_traj::Trajectory`에 5차 Hermite 조각으로 넣어
  이음매가 C²가 되게 한다.
- 정정(적대적 검증): TerminalPhase가 "감사되지 않는다"는 서술은 틀렸다.
  `logFinalEvaluation`이 스티치된 비행을 0.1초 간격으로 샘플링하며
  `"terminal"`도 등록된 phase로 판정된다. 다만 그 판정은 **순항 모델**로
  이뤄지므로, 전이 구간에는 §4의 단계별 평가기가 반드시 필요하다.

## 12. 진입 후보 탐색 — 순차 판정 파이프라인

```
경로 후보 생성 (기존 calm/κ/grade/zone 선별 그대로)
   ↓
순항 진입 적합성 (기존 규칙 + pvaEnvelopeProblem)
   ↓
전환 모델 도달 가능성 (전환 한계로 판정 — 새로 추가되는 단계)
   ↓
가능한 후보 중 선정
   ↓ 전부 실패
TRANSITION_GENERATION_FAILED   (direct fallback 금지)
```

구현 지침(조사로 확정):

- `authorContractsFromRoute` 안의 선별 블록을 **값 반환 메서드로 추출**한다
  (`screenHandoffCandidates(...) -> PhaseScreen`). 현재는 const 메서드가
  mutable 멤버(`dep_candidates_`, `arr_candidates_`, `phase_tan_grade_`,
  `phase_turn_radius_u_`)에 쓰는 블랙보드 구조라, 다른 한계를 쓰는 세 번째
  소비자(전환)가 붙으면 순항 선별과 **교차 오염**된다. 추출은 미관이 아니라
  안전 요건이다.
- 하한 공식은 **주입식 정책**으로 바꾼다: `depRequired`와
  `phase_turn_radius_u_`(1.5 마진·`load_factor_max` 내장)를 `ReachLimits`
  구조체 필드로. 순항 선별은 순항 수치를, 전환 선별은 §8 수치를 넘긴다.
  전환 선별은 여기에 **속도 변화 항**이 추가로 필요하다 — 현재 하한은
  \|v0\| 일정을 가정하는데, 전이는 정확히 그 가정을 깨뜨린다.
- `validateConnector`는 전환 콘을 넘겨주면 재사용 가능(지형·존·grade 기하
  검사). 단 이것은 **기하 스크린일 뿐** 모델 판정이 아니다.
- `buildDepartureConnector` / `contractFromVertex`는 전이 생성기로 재사용
  **금지**. 시간이 없고, 종료 가속도를 0으로 못박으며(`contractFromVertex`는
  `c.t = 0`, `c.acc = Zero`), 방향이 경로 정점 현으로 고정된다.
- `solveGated`의 `lastEnvPeak() > audit_envelope_peak_max` 게이트는 **순항
  모델 감사**이므로 전이 구간을 판정해서는 안 된다. 진입점 이후 첫 체인
  구간에는 그대로 둔다.
- 슬라이스는 `route_start_s`를 체인 내부로 흘리지 말고, `cutAtArc(route,
  cap, s)`로 **부분 경로를 한 번 실체화**해 기존
  `authorContractsFromRoute`/`sliceCommittedRoute`에 넘긴다. 그러면 호장
  원점·`route.front().z()` 기준·가상 head cut이 전부 수정 없이 동작한다.

기존 코드에서 함께 드러난 결함(계약 2 착수 전 정리 대상):

- departure 재시도 경로에서 `edge_cap`이 채워지지 않는다(arrival 분기에만
  대입). 커넥터 solve가 빈 cap을 받아 arc-varying 고도 밴드 대신 스칼라
  밴드로 판정된다 — 조용한 비대칭.
- N==2에서는 `arr_candidates_`가 비어 arrival 구제가 구조적으로 불가능하다.
- 후보 상한 3은 리터럴이고 **선착순**(점수 없음)이다.
- arrival 선별에는 `depRequired`에 해당하는 하한이 없다(도달성 선별은
  현재 departure 전용).

## 13. 오케스트레이션 소유권

- **FSM**: 계획 시점, 시작 PVA 해석과 AGL 변환, SI→단위 변환, 시퀀스·롤백,
  `PlanResult` 소비와 발행. **영역 분류·경계 검증·강등 판정은 소유하지
  않는다.** 현재 `replan_fsm.cpp`의 단일 경로 블록이 `plan()`의 정책을
  복제하고 있어, 분류가 세 상태로 늘면 정책이 두 벌로 갈라진다 — 계약 2
  착수 시 단일 진입으로 합친다.
- **SegmentChainPlanner(또는 그 자리의 조정 계층)**: 단계 순서, 분류 분기,
  경로 커밋, 진입점 선정, 슬라이싱, head BC 인계, 스티칭, `pm_->traj_`
  저장, 시각화, `PlanResult` 작성. 전이 기하와 한계 수치는 소유하지 않는다.
- **TransitionPhase**: 생성·다항화·§6 내부 검사. `PathManager`도 노드
  파라미터도 로깅도 건드리지 않는 순수 컴포넌트(`TerminalPhase` 형태:
  정적 함수, 지형은 `std::function` 주입, 한계는 구조체 인자,
  `poly_traj::Trajectory` 반환).
- **PathManager**: 변경 없음. 임의 호장 절단은 `sliceCommittedRoute`가 이미
  쓰는 `Cut{seg, u}` 기계와 같아 새 seam이 필요 없다.

착수 전 선행 정리(전이 코드보다 먼저):

1. `planRouteParallel`을 `commitRoute()` + `planOverRoute(route, cap,
   head_pva, tail, parallel)`로 분리. 그래야 계약 2가 "분류 → 커밋 → 전이 →
   절단 → 체인 → 앞에 붙이기"의 읽히는 순서가 된다.
2. `logFinalEvaluation`에서 판정을 `evaluateFlight()`로 분리(로깅과 판정
   분리, §4). 분리 전에 전이를 끼우면 전이 구간이 순항 영역으로 판정되어
   계약 2가 막으려는 바로 그 실패가 재현된다.
3. `fallback()` 람다의 호출 지점 전부를 전이 활성 시 FAILED로. 이 람다는
   **미션 시작점부터** 단일 계획을 세우므로, 전이가 필요한 미션에서 이게
   돌면 순항 영역 밖 상태에서 순항 모델 계획이 조용히 날아간다 — 문서가
   기원 결함으로 인용하는 548.7 u 벌레 그 자체.
4. 계획 범위 상태 정리: `segments_`가 계획 도중 재작성되고
   (`segments_ -= 1`), phase 후보들이 mutable 멤버다. 진입 후보를 바꿔가며
   재진입하는 순간 낡은 N과 낡은 후보 목록을 물려받는다.
5. `[VEL-ALIGN]`/`[STALL-FLOOR]`는 전이 종료 상태에 **적용 금지**(처방된
   궤적 유래 상태). route 모드에 복제된 판본이 있으므로 양쪽 다 확인.
