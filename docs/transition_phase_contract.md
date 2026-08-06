# TransitionPhase 설계 계약 v2 (계약 2 — 문서만, 코드 없음)

- 상태: **설계 초안 / 전환 모델 스펙 대기.** 인터페이스 방향은 리뷰로
  수렴했으나(2026-08-06), 전환 영역 한계 수치(§8)가 기체 스펙에서 오기
  전에는 어떤 골격 코드도 만들지 않는다 — 수치 없이 굳힌 인터페이스는
  잘못 굳는다.
- 채택 근거: 실제 요구가 "순항 영역 밖 초기 상태에서 시작해 순항 궤적으로
  연결"로 확정됨(2026-08-06). 초기 벡터를 영역 안으로 바꾸는 것은 계약 1
  데모일 뿐 요구 해결이 아니다.
- 전제(계약 1, 구현 완료): 명시 초기 상태의 **PVA 전체** 판정
  (`PathManager::pvaEnvelopeProblem` — 속력·상승각 영역 + 역동역학
  하중·추력·뱅크, 판정 불가 시 거부) → `INITIAL_MODE_UNSUPPORTED` FAILED;
  phase 모드 direct fallback의 비행 적합성 게이트(fail-closed: 평가 불가도
  FAILED) → `DIRECT_FALLBACK_UNSAFE`. 회귀: initfail / initaccfail /
  initok / synthclamp / unsafedirect / departop.

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
