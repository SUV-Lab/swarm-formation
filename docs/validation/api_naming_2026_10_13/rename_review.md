# Private naming implementation — independent read-only review

기준: 계획 서브모듈 `068f386f7a3eadd21ab1bd9308727e6eca7ba641` 대비 현재 작업 트리. 범위는 private 메서드 네 개의 개명·미사용 인자 제거·직접 참조 주석·함수명 디버그 태그이다. 소스와 호출자를 읽었고 빌드·실행·수정은 하지 않았다. `reports/`와 `experiments/`는 검색에서도 제외했다. 이 보고서 외 파일은 수정하지 않았다.

## 결과

네 개 개명은 선언·정의·호출자까지 일치하며 실행 제어 흐름, 반환값, 출력 데이터와 공개 함수 시그니처는 바뀌지 않았다. 직접 주석의 한정·책임 표현을 보강하도록 요청했고, 아래 항목 모두 반영된 최종 소스를 확인했다. 남은 차단사항은 없다.

| 이전 → 현재 | 접근 범위 | 실제 호출자 | 검토 결과 |
|---|---|---:|---|
| `ReplanFSM::planFromGlobalTraj(int trial_times)` → `publishExecutionTrajectory()` | private 유지 (`replan_fsm.h:106`) | 2 (`replan_fsm.cpp:352,387`) | 본체에서 `trial_times`를 전혀 사용하지 않았고 두 호출 모두 상수 `1`. 함수 인자 제거로 평가 부작용도 사라지지 않음. 채택 궤적/선택적 reference 발행, `pending_plan_` 소비, `have_local_traj_` 갱신 및 bool 반환은 동일. |
| `SegmentChainPlanner::invalidateStoredTrajectory` → `recordExecutionInvalidation` | private 유지 (`segment_chain_planner.h:217`) | 7 (`segment_chain_planner.cpp:506,583,699,2122,3071,3151,3183`) | `computed.invalidate_execution = true`만 수행. 실제 execution store 철회는 후속 apply 경로라는 새 이름·선언 주석과 일치. |
| `SegmentChainPlanner::planRouteParallel` → `planFromRoute` | private 유지 | 1 (`segment_chain_planner.cpp:445`) | `run_parallel` 전달 및 commit → computeOverRoute 순서 그대로. serial 모드도 가능한 실제 계약을 반영. |
| `RosPlanningBackend::planDirect` → `computeDirect` | private 유지 (`ros_planning_backend.h:65`) | 1 (`ros_planning_backend.cpp:320`) | 직접 계산·기존 경계 검사·완화 정책·결과 기록 동일. execution 적용은 `RosPlanningBackend::plan`이 담당. 새 주석은 기존 ROS 파라미터 선언 가능성도 명시. |

라인 번호는 검토 시점 기준이며 주석 편집으로 이동할 수 있다.

## 누락·범위 확인

- 활성 `src/`에는 이전 이름을 whole-word로 검색한 결과가 없다. `invalidateStoredTrajectoryForEnvChange` 같은 보류된 public 이름은 별개의 심볼이며 그대로 둔다.
- 상위 저장소의 C++/Python/shell/CMake 소스 확장자까지 검색하면 이전 이름은 `docs/validation/planning_visualization_2026_10_10/source/{baseline,current}/...`의 과거 소스 스냅샷에만 남는다. 해당 증거는 수정 대상이 아니다.
- 직접 참조하는 `mission_sequence_test.cpp`는 설명 주석 한 줄만 바뀌었고 테스트 실행 코드는 같다.
- 로그 변화는 `[planFromGlobalTraj]` → `[publishExecutionTrajectory]` 두 문자열이다. `[CHAIN-PAR]`, `[STITCH-GATE]`, ROS topic/payload 및 실패 사유 등 파싱 가능 출력은 그대로다. 허용된 저장소 코드·스크립트 검색에서 바뀐 함수명 태그를 파싱하는 소비자는 발견하지 못했다. 외부 로그 소비자의 부재까지 보장하는 결론은 아니다.
- 편집 동결 후 변경된 서브모듈 `src/` 파일 25개와 상위 `poly_traj.hpp`·`point_mass_eom.hpp` 2개, 총 27개를 HEAD와 비교했다. C/C++ 주석을 건너뛰고 문자열은 보존하여 토큰화한 뒤, 허용된 네 이름의 치환과 `trial_times` 선언·상수 인자 삭제만 정규화하면 27개 모두 동일했다. 이는 다른 계약 설명 담당자의 변경까지 실행 토큰의 범위 초과가 없다는 보조 증거다. 컴파일/동작 시험을 대체하지 않는다.
- `git diff --check` 통과.

## 직접 주석 검토와 수정 확인

1. `recordExecutionInvalidation` 본체의 기존 `The compatibility wrapper/FSM applies this...` 문장은 실제 `RosPlanningBackend::plan` 채택 경로를 빠뜨렸다. 최종본은 `backend or compatibility plan wrapper`로 정정되었다.
2. `segment_chain_planner.h`의 `planFromRoute ... Public...`와 cpp의 transition 설명이 public `planOverRoute`와 private `computeOverRoute`를 섞어 설명했다. 최종본은 public `planOverRoute`가 계산 후 execution 효과까지 apply하고, private `planFromRoute`/transition은 context helpers를 통해 계산하며 apply는 호출자가 담당한다는 구분을 명시했다. 실제 구현 `planOverRoute`의 compute→seal→apply, `planFromRoute`의 commit→computeOverRoute, transition의 route slice→computeOverRoute와 일치한다.
3. 추가된 `MissionPlanResult` 설명의 missing-product 실패가 이미 실패한 계산도 reason을 버리는 것처럼 읽힐 수 있어 한정을 요청했다. 최종본은 adoption 거절(성공/완화성공인데 product가 없는 경우 포함)만 detail-only FAILED가 되며, 기존 실패 status는 효과가 정상 적용되면 유지한다고 명시한다. `PlanningExecutionStore::apply`의 product validation 및 backend 반환 순서와 일치한다.
4. 추가된 `ExecutionReceipt::applied` 설명은 no-product failure 효과도 수락할 수 있고 새 궤적 생성/평가 성공을 뜻하지 않는다고 명시한다. 실제 apply 분기와 일치한다.
5. 추가된 formation 설명은 successful compute 뒤 primary optimizer에 전달하고 독립 workers 전체에 동일 설정을 보장하지 않는 실제 코드 순서와 일치한다. `synthesizeCommands` 설명은 `1.02`, `0.98`, `1.1*stall`, `CL_max*(1-1e-3)` 계산 및 waypoint 측 역산과 일치한다.

## 제한

독립 검토 담당 범위는 개명과 직접 설명이다. 전체 계약 주석의 의미 검증은 다른 담당자 및 Claude의 최종 검토와 함께 평가해야 한다. 빌드·R2·FSM/ABORT 결과를 이 보고서에서 실행하거나 대신 판정하지 않았다.
