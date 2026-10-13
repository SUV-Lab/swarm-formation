# Planning public API naming and contract audit — 2026-10-12

Read-only audit 완료. 계획 공개 헤더 39개를 모두 확인했고, 문제 후보는 구현·호출자·테스트까지 추적했다. 금지된 `reports/`, `experiments/` 내용은 읽지 않았으며 앱/저장소 파일 수정·빌드·실행은 하지 않았다. 이 파일은 root 요청에 따라 조사 결과를 보존한 것이다.

**판단: 전체 API 이름은 유지하는 편이 맞다. 단위나 실제 책임을 잘못 짐작하게 하는 소수 이름만 고칠 가치가 있다. 그보다 먼저 현행 문서의 모순을 정리해야 한다.**

아래 경로는 planning submodule 루트 `src/mmp_path_planning/` 기준이다. 헤더의 축약 경로는 `src/path_manager/include/path_manager/`, 구현의 축약 경로는 `src/path_manager/src/` 기준이다.

## 1. 바로 고칠 가치가 있는 DOC_ONLY

| 항목 | 증거 | 판단 |
|---|---|---|
| 이미 제거한 public `PathManager::traj_`가 남아 있다고 설명 | `docs/design/planning_computation_usage_ko.md:113–115`; 반면 `modularized_function_api_ko.md:91–95`는 제거 및 const 조회로 설명 | 현재 사용 문서끼리 모순. `executionTrajectory()/referenceTrajectory()`의 대여 수명 규약으로 수정. 코드 변경 필요 없음 |
| `FlightVerdict.evaluated=false` 의미가 “degenerate input”으로만 설명됨 | `flight_evaluation.h:29`; 실제 `flight_evaluator.cpp:75–95`는 잘못된 궤적·설정·환경 callback 부족에도 미평가 반환 | “평가를 수행하지 못함; 자세한 이유는 FlightEvaluationReport.issue”로 설명. false를 안전 판정으로 읽지 못하게 예제의 기존 evaluated 확인 유지 |
| 전환 자체의 검증기가 앞으로 들어올 것처럼 남은 역사적 설명 | `flight_evaluation.h:15–18`의 “gate arrives with the transition evaluator”; 현재 `transition_phase.h:195–198`에 `validateTransitionTrajectory`, generator 자체 검증 존재 | “whole-flight cruise 통계에서는 제외하며 transition generator의 별도 gate가 판단”처럼 현재 책임 설명으로 정리. 검증을 새로 추가하는 작업 아님 |
| `PlanReason` 주석의 “last two are FAILED-only”가 현재 enum과 안 맞음 | `planning_result.h:32–35`, enum은 `:65–117`에 여러 FAILED 전용 사유 | 역사 설명·줄 번호를 걷어내고 현행 분류/우선순위 계약만 남김. enum 순서는 degrade 우선순위이므로 이름 정리 과정에서 정렬하면 안 됨 |
| `ExecutionReceipt::applied`가 “새 비행 채택 성공”과 다름 | `planning_computation.h:57–60`; `planning_execution.cpp:65–80`는 성공 궤적 없는 실패 효과도 적용 후 applied=true | 헤더에 applied != hasTrajectory를 명시. apply()를 adopt()로 일괄 개명하면 오히려 거짓 설명이 됨. 현재 이름 유지 |

현재 문서가 이미 잘 설명하는 다음 규약은 **누락으로 보고하면 안 된다**.

- `PlanSettings`는 전체 불변 설정이 아니라 희소 초기 override: `plan_settings.h:15–17`, `planning_library_usage_ko.md:81–104`.
- `PlanningEnvironment`는 초기 patch, live snapshot 아님: `planning_environment.h:44–47`, 사용법 `:140–170`.
- `compute()`는 새 후보를 채택하지 않지만 환경 준비·기존 실행 철회·일부 표시 부작용까지 없는 순수 함수는 아님: `ros_planning_backend.h:35–41`, 계산 문서 `:21–23`.
- `MissionPlanRequest.start_altitude`는 시작점에만 적용; waypoint z는 별도 AGL 규약: `mission_plan.h:18–22`, 사용법 `:123–138`.
- `reference_trajectory`는 모드별 의미가 다름: `mission_plan.h:43–45`, 사용법 `:208–210`.

## 2. 범위가 작은 실제 RENAME 후보

### A. route_start_s → route_start_arc_u — 권고

- `transition_phase.h:73`의 `EntryCandidate::route_start_s`는 **초가 아니라 committed route의 호 길이, planner 단위 u**.
- `TransitionResult::route_start_s` `:172` 바로 옆 `duration_s` `:173`는 초.
- 동일 헤더에 dt_s, t_max_s, dwell_s, knot_dt_s 등 초 suffix가 일관되게 있어 이 필드만 오독하기 쉽다.
- 실제 값: `segment_chain_planner.cpp:1754`에서 arc[idx], `:1907`에서 cutAtArc(..., tr.route_start_s, ...).
- 조사 범위 영향: 4개 파일, 11 occurrences. header, transition_phase.cpp, segment_chain_planner.cpp, test/transition_experiment_test.cpp. 금지된 experiments 디렉터리는 조사하지 않았으므로 그 밖 추가 소비자는 별도 범위 확인 필요.
- 알고리즘·단위변환은 전혀 바꾸지 않고 필드 식별자만 변경 가능.
- 공개 aggregate 필드라 rename은 source compatibility 변화다. 동기화된 저장소 내부 소비자만 지원한다면 한 번에 변경. 외부 소스 호환을 약속한다면 단순 alias field 중복은 피하고, 기존 필드 유지 + 명시적 accessor/문서 후 버전 경계에서 변경하는 편이 안전하다.

### B. 계산 요청의 max_vel → seed_speed_u_s 또는 seed_arrival_speed_u_s — 권고하되 optimizer 담당과 한 묶음

- DirectTrajectoryRequest::max_vel: `direct_trajectory_solver.h:52–54`.
- RouteSliceConfig::max_vel: `route_slice_solver.h:31–33`.
- 실제 전달: direct solver `:42`, slice solver `:40`.
- 하위 OptimizerSolveRequest::max_vel: `src/path_optimizer/include/path_optimizer/optimizer_request.h:18–21,35`.
- 이것은 seed/도착 상태를 만드는 속도이며 optimizer의 optimization/max_vel 제한과 별개다.
- 구현 증거: `src/path_optimizer/src/poly_traj_optimizer.cpp:209`의 des_vel = max_vel, `:267`의 traj_end_vel = approach_dir * max_vel; 반면 속도 penalty는 `:3409`의 별도 멤버 max_vel_.
- max_vel을 낮추면 전체 궤적 제한도 낮아진다고 생각하기 쉬워 단순 취향 문제가 아니다.
- YAML manager/max_vel, optimization/max_vel, legacy optimizer 멤버/수식까지 바꾸면 안 된다. 새 공개 요청 필드와 wrapper 전달부만 대상으로 해야 한다.
- 영향: direct request는 4개 파일에 나타나고, RouteSliceConfig는 8개 파일에 나타난다. optimizer request/예제/테스트까지 연쇄 범위가 있으므로 root의 optimizer 감사 결과와 합쳐 목록 확정.
- 필드 source compatibility는 A와 동일하게 판단. struct 레이아웃 보존만으로 외부 소스가 호환되는 것은 아니다.

### C. validateFlightPhases() → validateTrajectoryAndPhases() — 권고, 낮은 위험

- 선언 `flight_evaluator.h:148–150`.
- 구현 `flight_evaluator.cpp:15–26`은 phase뿐 아니라 조각 시간 양수/유한성, 계수 유한성, 총 시간까지 검사.
- validateFlightEvaluationInput은 반대로 너무 크다. 이 함수는 설정/환경 callback 검사는 안 한다. 그래서 validateTrajectoryAndPhases가 정확하다.
- 현재 직접 영향 3개 파일, 4 occurrences: 선언, 구현, evaluateFlight 호출 `flight_evaluator.cpp:75`, chain 사전 검사 `segment_chain_planner.cpp:3244`.
- 기존 외부 함수 유지 wrapper → 새 함수로 위임하는 점진 이전이 쉽다. FlightEvaluationIssue enum/판정 순서 유지.

### D. nextRouteSearchIdentity()를 일반 context ID helper로 이동 — P3, 정리할 때만

- 선언 `route_search_backend.h:137`.
- 구현 `route_search_backend.cpp:8–10`은 단순 process-wide atomic sequence.
- 실제 PlanningExecutionStore 생성자도 execution context용으로 호출: `planning_execution.cpp:2,10`.
- 실행 저장소가 route search 헤더를 include하는 이유가 ID 생성 하나다. 실제 책임과 이름·파일 소속이 어긋난다.
- 후보 detail::nextPlanningIdentity()/nextPlanningContextId(); 호출 request ID와 context ID 모두에 쓰므로 특정 객체 종류로 너무 좁히지 않아야 한다.
- 조사 범위 5개 파일, 9 occurrences: route_search header/implementation/test, PM, execution store.
- **기존과 같은 하나의 sequence**를 유지할 것. route/execution용으로 따로 카운터를 복제하는 변경은 이름 정리가 아니다.
- 기존 공개 symbol wrapper를 유지할 수 있다. 새 helper를 일반 외부 사용자에게 추천 API로 추가할 필요는 없다.

## 3. 이름만 바꾸면 해결되지 않는 CONTRACT_CHANGE — 이번에는 분리 판단

### A. 미션 요청 속 두 설정은 다음 호출에도 지속됨

- MissionPlanRequest.length_per_piece/obstacle_clearance: `mission_plan.h:31–35`.
- `ros_planning_backend.cpp:271–281`는 양수 값을 manager에 반영하고 복구하지 않는다.
- 같은 요청 타입에 계산 입력과 persistent settings update가 섞여 있다.
- 하지만 이 동작은 기존 ROS 호환 의도이고 주석·문서·테스트가 이미 고정한다. `mission_planner_test.cpp:187–212`의 비유한 거부와 nonpositive retain 검사.
- PlanOptions라고 묶거나 필드명에 override만 붙여도 호출 간 지속성이 사라지지 않는다.
- 지금 권고: **KEEP + 계약 강조**.
- 이후 외부 SDK 요구가 생기면 PersistentPlannerUpdate 또는 명시 setter / 순수 per-call option을 구분해 설계. 기존 동작을 “리팩토링”이라며 바꾸면 안 된다.

### B. direct/slice 결과가 하위 실패 사유를 잃음

- optimizer는 NOT_CONFIGURED, MALFORMED_INPUT, OPTIMIZER_FAILED를 구별: `optimizer_request.h:39–45`, `optimizer_session.cpp:57,76`.
- direct wrapper는 route-too-short/no-waypoint만 구분하고 나머지를 기본 OPTIMIZER_FAILED로 반환: `direct_trajectory_solver.cpp:49–57`.
- slice wrapper는 solver 실패를 모두 OPTIMIZER_FAILED로 반환: `route_slice_solver.cpp:42–44`.
- 테스트도 이 현재 계약을 명시: `direct_trajectory_solver_test.cpp:321`, `route_slice_solver_test.cpp:197`.
- 외부 호출자는 잘못된 입력/설정과 실제 수치 최적화 실패를 구분할 수 없다.
- OPTIMIZER_FAILED를 SOLVE_FAILED로만 개명하는 것보다 하위 cause 또는 추가 enum 값을 보존하는 것이 실질적 개선.
- 다만 이것은 **결과 계약 확장**이다. 이름 변경 작업과 별도 PR/범위로 선택해야 하며, 기존 실패 소비자가 영향을 받는지 확인해야 한다.

### C. MissionPlanResult.reference_trajectory의 세 의미

- direct=seed, baseline chain=optimized baseline, route chain=complete flight.
- 이름을 seed로 바꾸면 두 모드를 거짓으로 설명하게 된다.
- 현 이름과 문서 유지. 호출자가 참고 궤적의 종류를 프로그램으로 구분해야 한다면 추후 ReferenceTrajectoryKind 추가가 해법이지 mass rename이 아니다.

### D. addDynamicSphere/Box()의 int 반환

- >=0 ID, -2 정상 대기, -1 거부: `mission_planning_backend.h:24–29`.
- API만 보고 id < 0 전부 실패로 처리하기 쉬우나 문서는 명확하다.
- ObstacleAddResult{status,id} 같은 typed result는 개선 가능하지만 public virtual interface 전반 변경이라 계약 작업이다. 현 이름을 바꿔 해결되지 않는다.
- 조사 범위 addDynamicSphere 14개 파일, 34 occurrences.

## 4. 유지할 좋은 이름 / 불필요한 정리

다음 이름은 현재 행동과 일치하므로 사실상 고정해도 된다.

- MissionPlanner::plan, MissionPlanRequest, MissionPlanResult
- MissionPlanningBackend, RosPlanningBackend
- VehicleModel, RouteSearchBackend::search
- searchMissionRoute, roundRouteCorners, prepareRouteProfile
- resolveMissionWaypoints, computeMissionBounds
- solveDirectTrajectory, solveRouteSlice, solveRouteSliceBatch
- stitchRouteFlight, prependFlightPrefix
- evaluateFlight, FlightEvaluationReport, formatFlightEvaluation
- PlanningComputation, PlannedTrajectory, PlanningExecutionStore::apply/revoke
- makeTrajectoryMessage, visualization make* 계산과 ROS 연결 분리

PlanResult는 payload 없는 상태·사유 타입이므로 PlanStatus가 언어상 더 정확하지만, 현재 MissionPlanResult.status가 이미 용도를 분명하게 한다. 조사 범위만 23개 파일·187 occurrences다. **이름 통일만을 위한 전면 교체는 권하지 않는다.** 필요하면 나중에 alias부터 도입하되 지금 우선순위는 낮다.

StartHead, head/tail, route/slice/segment/piece/leg/phase는 의미가 실제로 다르다. 한 단어로 통일하면 구간-정책 귀속을 해칠 수 있다. 짧은 용어표로 차이를 설명하는 것이 낫다.

RosPlanningVisualization::frontEndPath/tube/chain/...는 실제 publish side effect (`ros_planning_visualization.cpp:23–43`)이므로 publishFrontEndPath/...는 검토할 수 있다. 하지만 명확한 클래스 문맥이 있고 현재 make* 함수와 이미 분리되어 있으므로 필수 개명으로 분류하지 않았다. 다른 담당이 같은 항목을 찾으면 낮은 우선순위 한 묶음으로만 판단하면 된다.

## 5. 조사 범위와 한계

헤더 39개 전수 확인:

`builtin_route_search.h`, `committed_route.h`, `direct_plan_settings.h`, `direct_trajectory_solver.h`, `flight_assembly.h`, `flight_evaluation.h`, `flight_evaluator.h`, `mission_geometry.h`, `mission_plan.h`, `mission_planner.h`, `mission_planning_backend.h`, `mission_route_search.h`, `optimizer_worker_profile.h`, `plan_settings.h`, `planning_computation.h`, `planning_environment.h`, `planning_environment_store.h`, `planning_execution.h`, `planning_result.h`, `planning_visualization_data.h`, `quintic_hermite.h`, `risk_zone.h`, `ros_planning_backend.h`, `ros_planning_visualization.h`, `ros_route_search.h`, `ros_terrain_grid.h`, `route_postprocessing.h`, `route_search_backend.h`, `route_slice_batch.h`, `route_slice_solver.h`, `start_state.h`, `terminal_phase.h`, `terrain_data.h`, `terrain_risk.h`, `traj_sampling.h`, `trajectory_message.h`, `transition_phase.h`, `waypoint_eval.h`, `zone_policy.h`.

대조한 핵심 구현:

- mission planner/backend 전체 entry 및 setter 처리.
- direct/slice wrapper, execution store, route backend admission.
- flight evaluator 입력 검증 및 평가 준비.
- transition arc field 실제 생산/소비.
- ROS visualization publish 구현.
- optimizer 요청 경계와 seed 속도 실제 사용.
- PM/chain/FSM은 위 후보의 호출 관계 확인에 한정; 전체 클래스 감사는 다른 담당 범위.

현행 예제·문서:

- mission_planner_example.cpp, flight_evaluator_example.cpp.
- planning_library_usage_ko.md, planning_computation_usage_ko.md, modularized_function_api_ko.md.
- 관련 declaration/caller/test 검색; 전체 테스트 실행이나 모든 수식 재검증은 하지 않았다.
- occurrence 수는 PM/optimizer의 현행 include/src/test/examples 범위의 문자열 출현 수다. 외부 저장소나 금지 경로의 호환 소비자 수는 알 수 없다.

## 6. 권고 작업 순서

1. 현재 문서·헤더의 모순과 오래된 설명 수정.
2. route_start_s, 요청 max_vel, validateFlightPhases의 **세 의미 혼동만** 제한적으로 정리.
3. 범위가 필요하면 일반 identity helper 소속 정리.
4. 요청 지속 설정, 상세 실패 원인, typed obstacle 결과 등은 **별도 계약 설계 선택**으로 남김.
5. 클래스/함수 전체 일괄 rename, C++ 스타일 통일, ROS/YAML/메시지 이름 변경은 하지 않음.
