# MMP API 명명 설계 초안 — 협업 세션 Claude의 독립 검토

작성: 협업 세션 Claude (Claude Code, 2026-10-13). **협업 세션의 분석·제안이며 사용자의 전달이 아니다.**
검토 대상: `src/mmp_path_planning/docs/design/api_naming_design_ko.md` (2026-10-13 초안).
기준: 상위 `13ced330`, 계획 서브모듈 `068f386f` 및 현재 작업 트리(서브모듈 소스 수정 없음, 미추적은 docs/만).
방법: 초안이 제시한 후보 하나하나의 실제 동작을 현재 소스에서 다시 읽었다(아래 file:line). 지침 원문은 Qt API Design Principles와 LLVM Coding Standards를 재조회해 인용을 확인했고, C++ Core Guidelines NL 절은 페이지가 너무 길어 도구로 가져오지 못해 기억으로만 대조했다(초안의 NL.10 해석 "선택권이 있을 때의 기본값, 기존 스타일 우선"은 내 기억과 일치). `reports/`·`experiments/`는 읽지 않았고 코드·설정·그림·빌드·실행은 건드리지 않았다. 대형 워크플로우는 돌리지 않았다(bounded review).

## 0. 결론 요약

초안의 **핵심 결정(공개 입구·표기법 유지, 계산/검사/상태반영/발행 동사 구별, PlanResult 별칭 미추가, bool helper 보류)에는 동의**한다. 외부 조사 표의 적용 판단도 과장이 없다. 반론은 아래 5건이 핵심이고, 나머지는 주석·범위 보완이다.

| # | 항목 | 판정 | 한 줄 근거 |
|---|---|---|---|
| A3 | `planRouteParallel` → `computeRoutePlan` | **수정** | 같은 클래스의 private 모드 함수 6개가 전부 compute-only인데 `plan*`로 불린다. 하나만 `compute*`로 바꾸면 나머지 5개가 "채택한다"고 읽힌다. 결함은 `Parallel`뿐이므로 `planFromRoute`(또는 `planRouteMode`) |
| B4 | `numAllocatedBlocks` → `cachedVoxelCount` | **수정 → 2차 합의 `storedVoxelCount()`** | 밀집 빌드·로드 뒤에는 정적 격자 전체 voxel 수(`nx*ny*nz`), `buildEmpty`·점유 없는 빌드에서는 shape가 있어도 0(§6 정정). `cached`는 부분 집합을 암시해 반대. 1차 제안 `voxelCount()`는 빈 격자에서 shape와 어긋나 철회, Codex 절충안 수용 |
| B1 | `invalidateStoredTrajectoryForEnvChange` → `revokeExecutionForZoneChange` | **수정** | 함수 본체는 zone을 모르고 `POLICY_UNEVALUATED` 사유로 revoke만 한다. 호출자의 트리거를 공개 이름에 고정하면 `Parallel`과 같은 실수. 사유 기반 이름 또는 사유 인자 |
| C2 | `max_vel` → `nominal_speed_u_s` | **부분 수정/보류** | `nominal_speed`는 동의. 접미사 `_u_s`는 식별자 선례가 0건이고 기존 관례는 `vel_u`(125회)·`_mps`(SI)라 같은 구조체 안에 세 번째 스타일이 생긴다. 같은 값이 `PathManager::maxVel()`·`max_velocity`로 남고, 체인 모드에서는 기본 ceiling 역할도 한다 |
| A2 | `invalidateStoredTrajectory` → `markExecutionForInvalidation` | **수정(약)** | 같은 struct의 기존 동사는 `recordProduct()`(같은 플래그를 끈다). `recordExecutionInvalidation`이 짝이 맞고 §4의 1회용 `mark…For…` 항목이 필요 없어진다 |

나머지: A1 유지(주석·로그태그 보완), A4 유지(자체 규칙과의 긴장 기록), B2 유지, B3 조건부 유지, B5 유지, C1 유지, C3 유지(JSON 키 스키마 주장은 2차에서 철회), C4 유지, `nextRouteSearchIdentity` 보류 유지.

## 1. 먼저 적용할 내부 이름 4개

### A1. `ReplanFSM::planFromGlobalTraj(int trial_times)` → `publishExecutionTrajectory()` — 유지

확인한 동작(`path_manager/src/replan_fsm.cpp:516-557`): 실행 슬롯 `executionTrajectory()`를 읽고(:520), 살아 있으면 `pending_plan_` 또는 슬롯의 궤적으로 메시지를 만들어 `optimized_path_pub_`에 발행(:533-536), `have_local_traj_ = true`(:537), 옵션이 켜져 있으면 reference 궤적도 `global_path_pub_`에 발행(:539-548), `pending_plan_.reset()` 후 `true`(:549-550). 슬롯이 비어 있으면 `false`(:553-555). 재계산·retry 없음. `trial_times`는 본문 어디에도 쓰이지 않고 호출자 둘 다 `1`을 넘긴다(:352, :387). `executionTrajectory()`·`publishExecutionAbort`(`replan_fsm.h:142`) 어휘와 맞는다.

보완 요구:
- 선언 주석에 "reference(global) 궤적도 함께 발행할 수 있다"와 "반환 bool은 발행 성공이 아니라 '발행할 살아 있는 궤적이 있었다'는 뜻"을 적어야 한다. publish 자체는 void다.
- 로그 태그 `[planFromGlobalTraj]`(:531, :554)와 시험 주석 `planFromGlobalTraj(:518)`(`path_manager/test/mission_sequence_test.cpp:22`)는 새 이름을 따라가야 한다. scripts/·tools/·docs/에서 이 태그를 파싱하는 도구는 없다(검색 결과는 과거 실행 로그와 감사 문서뿐). 초안 §6의 "로그 식별자 유지"는 2차에서 "단순 debug 태그는 소비자 확인 뒤 갱신, 파싱되는 태그는 보존"으로 정리됐고 동의한다. 내 검색 범위는 scripts/·tools/·docs/뿐이었고 reports/·experiments/는 금지 범위라 보지 못했으므로 소비자 확인은 실제로 필요하다.

### A2. `SegmentChainPlanner::invalidateStoredTrajectory(computed)` → `markExecutionForInvalidation(computed)` — 수정(약): `recordExecutionInvalidation`

확인한 동작(`segment_chain_planner.cpp:222-227`): `computed.invalidate_execution = true` 한 줄. const 멤버이고 `this`를 건드리지 않는다. 적용은 `PlanningExecutionStore::apply`가 `invalidateCurrent()`로 한다(`planning_execution.cpp:79`).

반론: 같은 플래그를 **끄는** 기존 함수가 `PlanningComputation::recordProduct()`다(`planning_computation.h:50-54`, `invalidate_execution = false`). 결과 객체에 효과를 "기록"하는 동사가 이미 `record`로 정해져 있으므로 `recordExecutionInvalidation(computed)`가 `recordProduct`의 반대쌍으로 읽힌다. 초안 §4의 `mark…For…` 항목은 이 함수 하나를 위해 만든 1회용 동사라 사전에서 뺄 수 있다. `mark…`도 틀린 이름은 아니므로 유지해도 반대하지는 않는다. 구조적 대안(`PlanningComputation`의 멤버로 옮겨 `computed.recordExecutionInvalidation()`)은 이름 변경 묶음 범위 밖이니 기록만.

함께 고칠 것: `replan_fsm.cpp:408` 주석이 `SegmentChainPlanner::invalidateStoredTrajectory`를 인용한다.

### A3. `SegmentChainPlanner::planRouteParallel` → `computeRoutePlan` — 수정: `planFromRoute` (동사 유지, `Parallel` 제거)

확인한 동작(`segment_chain_planner.cpp:1937-1945`): `commitRouteInContext` + `computeOverRoute`의 합성. 채택 없음. `run_parallel`은 인자이고 로그는 "route-sequential"도 찍는다(:1965-1970). 초안의 사실 진술은 맞다.

반론(이름 선택):
1. 이 클래스에서 `plan`/`compute`가 채택 여부를 가르는 곳은 **공개 계층뿐**이다. `plan()`은 apply(:254-259), `planOverRoute()`도 seal+apply(:2497-2526), `compute()`는 apply 없음(:229-251).
2. `compute()` 아래의 private 모드 함수는 **전부** compute-only인데 모두 `plan*`이다: `planImpl`(h:392), `planInto`(h:278), `planMultiWaypoint`(h:477), `planBaselineMission`(h:481), `planTransitionMission`(h:297), `planRouteFallback`(h:362). 여섯 중 하나만 `compute*`로 바꾸면 독자는 나머지 다섯이 채택한다고 추론한다. 초안이 "다른 private 이름도 일괄 통일하지 않는다"고 했으므로 이 부분 개명은 오늘보다 나쁜 신호를 만든다.
3. `computeRoutePlan`은 자기 하위 단계인 `computeOverRoute`(h:281)·`computeRouteAttempt`(h:272)와 같은 동사 층위에 놓여 모드 입구라는 사실이 가려지고, "계획(plan)을 계산(compute)한다"는 중복이다.
4. 실제 결함은 `Parallel`(초안 §3 규칙 그대로)이다.

제안: `planFromRoute(state, computed, head, waypoints, run_parallel, tail)` — 설정 키 `chain/author_from_route`(:438)와 짝이 맞고, 경로를 넘겨받는 `planOverRoute`와 대비된다. 대안 `planRouteMode`. Codex가 `compute`를 고집한다면 같은 묶음에서 여섯 형제를 모두 바꾸거나(`computeImpl`…) 하나도 바꾸지 않아야 한다.

### A4. `RosPlanningBackend::planDirect` → `computeDirect` — 유지(낮은 우선순위로 `computeDirectPlan` 고려)

확인한 동작(`ros_planning_backend.cpp:333-403`): 초기 상태 envelope 검사(:340-356), 최종 경계 relaxation(:358-392, 이때 ROS 파라미터 `planning/allow_final_boundary_relaxation`을 선언한다 :373-376), `manager_->computeGlobalTrajectory`(:394), `computation.recordProduct`(:397), relax 시 degrade(:400-401). 채택은 없다. 채택은 `RosPlanningBackend::plan()`이 `compute()` 직후 `manager_->applyPlanningComputation(computation)`을 부르는 곳(:213-217)에서 일어난다(1차 표현 "FSM/호환 경로"는 부정확했음, §6 정정). 형제 `chain_->compute(...)`(:317)와 동사가 맞는다.

지적: 초안 §3 "동사 + 문맥에서 빠진 대상" 규칙에 비추면 `Direct`는 형용사뿐이라 무엇을 계산하는지 이름에 없다. `chain_->compute`처럼 대상이 out-param(`PlanningComputation&`)이라는 이유로 허용하려면 §3에 그 예외를 적어야 하고, 아니면 `computeDirectPlan`. 노드 파라미터 선언 부작용은 §4 compute 정의가 허용하지만 선언 주석에는 적어야 한다.

## 2. 공개 함수 5개 (호환 이름 유지 전제)

### B1. `PathManager::invalidateStoredTrajectoryForEnvChange` → `revokeExecutionForZoneChange` — 수정: 사유 기반

확인한 동작(`path_manager.cpp:2160-2165`): `execution_->revoke(EnvChangeReason::POLICY_UNEVALUATED, Zero, 0.0, environmentRevision())` + `clearTrajectoryViz()`. 본체는 zone을 참조하지도 감지하지도 않는다. zone 조건은 유일한 호출자 `loadRiskZonesCallback`(`replan_fsm.cpp:1455-1458`)에 있다. `EnvChangeReason`은 `ExecutionRevocationReason {OBSTACLE_BLOCKED, TERRAIN_BLOCKED, POLICY_UNEVALUATED}`의 별칭이다(`path_manager.h:434`, `planning_execution.h:11-12`). 형제 `revalidateAfterEnvChange(what, reason)`(h:452, cpp:2167-2186)는 이미 사유를 인자로 받고 `revalidateStoredTrajectory`를 거쳐 같은 `revoke`를 부른다(:2138).

반론: 공개 PathManager 메서드에 호출자의 트리거("ZoneChange")를 박으면, 다음에 정책을 재판정할 수 없는 변경(기체 모델 교체, 설정 교체 등)이 생겼을 때 잘못된 이름을 재사용하거나 중복 함수를 만들게 된다. 초안이 `Parallel`을 뺀 논리와 같다. 동사 `revoke`는 store의 `PlanningExecutionStore::revoke`(`planning_execution.h:58`)와 맞으니 유지.

제안(둘 중 하나):
- (a) `revokeExecutionForUnevaluatedPolicy()` — enum 값을 그대로 비추고 호출자에 독립.
- (b) `revokeExecution(ExecutionRevocationReason reason)` — `revalidateAfterEnvChange(what, reason)`와 대칭. 옛 이름은 `POLICY_UNEVALUATED`를 넘기는 forwarding으로 유지. 시그니처가 생기지만 §8 묶음 3("새 이름을 정식 입구로")에 들어간다.

### B2. `validateFlightPhases` → `validateTrajectoryAndPhases` — 유지

확인한 동작(`flight_evaluator.cpp:10-63`): piece 수·duration·계수 유한성(:15-23), 총 시간(:24-27), span 계약(:36-61)을 검사하고 `FlightEvaluationIssue`를 돌려준다. settings 검사는 `evaluateFlight` 쪽에 따로 있다(:78-82, INVALID_SETTINGS). 헤더 주석도 이미 "Validate trajectory representation and the legacy phase contract"(h:144-147)라 새 이름이 주석과 일치한다. 호출자는 내부 둘뿐(:75, `segment_chain_planner.cpp:3246`). `validateEvaluationInput` 같은 대안은 settings를 안 보므로 부정확. 설치 헤더의 자유 함수라 inline forwarding이면 충분하되 §8의 ABI 주의는 그대로 적용.

### B3. `PathSearcher::astarSearchAndGetSimplePath` → `searchAndSimplify` — 조건부 유지

확인한 동작(`path_planner/src/dyn_a_star.cpp:629-1523`): barrier 준비(:638) → `front_end_ == FM2`면 FM2(:652-897), FM2가 실패하면 A*로 넘어감(:901-932 `if (fm2_done) … else { … AstarSearch }`) → 경로 없으면 `{}`(:982) → `bypass_shortcut_`면 원시 경로 반환(:1019-1025), 점 2개면 그대로 반환(:1007-1015) → 위험 인지 shortcut 단순화(:1029-) → 지형 여유 수리 `sweepTerrainClearance`(:1374-1375) → 전체 경로 점유 최종 게이트, 거부 시 `{}`(:1467-1520) → `simple_path`(:1523).

판단: `astar` 접두사가 틀렸다는 초안 진단은 맞다(기본 FM2, A*는 선택/폴백). 다만 `searchAndSimplify`는 네 단계 중 둘만 말한다. 지형 수리와 **경로를 거부할 수 있는** 최종 게이트가 이름에 없고, simplify는 디버그 토글·2점 경로에서 건너뛴다. 선언 주석에 "FM2/A* 선택 → shortcut 단순화(bypass 가능) → 지형 수리 → 전체 점유 게이트(실패 시 빈 벡터)"를 적는 조건으로 유지. `search` 단독은 원시 `AstarSearch()`(h:949)와 혼동되므로 반대. 명사형을 원하면 `searchSimplifiedPath`.

### B4. `SDFManager::numAllocatedBlocks` → `cachedVoxelCount` — 수정: 2차 합의 `storedVoxelCount()` (1차 제안 `voxelCount()`는 철회)

확인한 동작(`path_planner/src/sdf/sdf_manager.cpp:863-866`): `impl_->distance_cache.size()`. `distance_cache`는 정적 격자 전체를 담는 밀집 `std::vector<float>`로, 빌드(:345)와 로드(:481-482)에서 `N = nx*ny*nz`로 `assign`되고 리셋 때 비워진다(:173, :195-196, :249-250). 즉 밀집 빌드·로드 뒤에는 **정적 지도 전체 voxel 수**다. 다만 `buildEmpty`(:177-204)와 점유가 전혀 없는 빌드(:244-258)는 nx/ny/nz와 `has_data=true`를 두면서 `distance_cache`를 비우고 `static_empty`/`static_free_dist` 스칼라만 남기므로, shape가 있어도 0을 돌려준다(Codex 2차 지적으로 정정, §6).

반론: 초안의 근거 "전체 지도 크기·메모리 바이트 수가 아님"은 밀집 빌드에서는 성립하지 않고 빈 격자에서만 성립하므로 문장을 정확히 써야 한다. 어느 경우든 `cached`는 부분 캐시를 암시해 오독을 새로 만들고, "static distance cache"는 구현 변수명이지 공개 개념이 아니다. 1차에 형제 `voxelSize()`, `voxelSizes()`, `shape()`, `origin()`(h:142-149)에 맞춘 `voxelCount()`를 제안했으나 빈 격자에서 `shape().prod() > 0`인데 0을 돌려주어 오해를 만들므로 철회하고, Codex의 `storedVoxelCount()`(저장된 정적 distance sample 수; 밀집 시 N; 암묵 all-free면 0; 동적 패치 제외 — 주석으로 고정)를 받는다. 유일한 호출자의 로그 라벨 `blocks=%zu`(`planning_environment_store.cpp:250-251`)도 같이 고친다.

### B5. `SimulationSession::controller(t,state)` → `controlAt(t,state)` — 유지(낮은 우선순위)

확인한 동작(`mmp_vehicle_dynamics/src/trajectory_simulation.cpp:269-275`): reference 샘플 → 속도 → `controller->commandedAcceleration` → `model->realize` → 모델이 실현한 `SimulationControl` 반환. 초안대로 `realized`를 이름에서 빼더라도 주석에는 "model-realized"를 적어야 한다(동의).

메모: 형제 `derivative(time_s, state, used)`·`audit(state, control)`에는 `At`가 없고 `stateIndexAt(time_s)`만 시각 단독 질의다. `control(time_s, state)`가 `derivative`와 정확히 대칭이지만 `control`이 명령형 동사로 읽힐 위험이 있어 `controlAt`도 수용한다.

## 3. 공개 필드

### C1. `route_start_s` → `route_start_arc_u` — 유지

`EntryCandidate`(`transition_phase.h:73`, 주석 "arc on the committed route (planner units)")와 `TransitionResult`(h:172), 소비처 `cutAtArc(route, cap, tr.route_start_s, …)`(`segment_chain_planner.cpp:1907`). 관례 충돌은 실재한다: `_s`=초가 40개 이상(`time_s`, `dt_s`, `duration_s`…), `_u`=planner 단위가 100개 이상(`pos_u`, `vel_u`, `floor_u`…). 호 길이 선례 `src_arc_m`(53회)이 있으므로 `arc_u`가 자연스러운 짝이다. 비용: 설치 헤더의 공개 struct 필드, 내부 호출 4곳 + 시험(`transition_experiment_test.cpp:100, 683, 1028`).

### C2. scalar `max_vel`(3계층) → `nominal_speed_u_s` — 부분 수정/보류

사실 확인(초안 진술 지지): `optimizer_request.h:20-21`("seed/derived-arrival speed, distinct from optimization/max_vel"); `optimizer_session.cpp:70`(유한·양수 검증), :85(인자로 전달); `poly_traj_optimizer.cpp:209`(`des_vel = max_vel`, seed 시간 배분), :267(`traj_end_vel = approach_dir * max_vel`, 미지정 도착 속도). 실제 속도 제약은 멤버 `max_vel_`(:3409 `vpen`, :4119-4120 `optimization/max_vel`). 전달 경로 `direct_trajectory_solver.cpp:42`, `route_slice_solver.cpp:40`; 출처 `request.max_vel = max_vel_`(`path_manager.cpp:1644`) ← YAML `manager/max_vel`(:64, :119). 따라서 "max"를 빼는 방향은 맞다.

반론 1 — 접미사: `_u_s` 식별자 선례는 0건이다(유일한 검색 결과는 예제의 로그 문자열 `examples/mission_planner_example.cpp:71`). planner 단위 변화율은 지금까지 `vel_u`(125회)·`acc_u`(43회)처럼 `_u`만 붙였고 SI는 `_mps`(`speed_mps` 125회)다. 대상 struct 세 개의 이웃 필드는 접미사가 없고(`head_vel`, `head_acc`, `cap_ref`: `optimizer_request.h:30-36`, `direct_trajectory_solver.h:45-55`, `route_slice_solver.h:30-37`) 단위는 struct 주석("velocity in u/s")이 정한다. 한 필드에만 `_u_s`를 붙이면 같은 struct 안에 세 번째 스타일이 생긴다. §5가 `_u_s`를 새 규칙으로 세우려면 `vel_u`·`acc_u`를 grandfather로 명시하거나 이전 계획을 적어야 하고, 아니면 `nominal_speed_u`(기존 관례) 또는 `nominal_speed`(struct 단위 주석)로 가야 한다. 결정 필요.

반론 2 — 범위: 같은 manager 값이 `PathManager::maxVel()`(`path_manager.h:221`)과 `RiskProfileSettings.max_velocity`(`planning_visualization_data.h:61` ← `path_manager.cpp:1912`)로도 공개된다. 요청 필드만 바꾸면 설치 헤더에서 옛 이름이 두 곳 더 살아남는다. 묶음에 넣거나 "제외"를 명시해야 한다.

반론 3 — "최대 제약 아님"의 한계: 체인 모드에서 `pm_->maxVel()`은 계약 작성의 cruise 속도이고(`segment_chain_planner.cpp:1259`, :1719 `const double cruise = pm_->maxVel()`), 동시에 worker별 `optimization/max_vel` override를 조화시킬 때의 **기본 ceiling 기준**이다(:2222-2241 `effMaxVel`, :2782-2786 `ceil_v`). 요청 계층에서는 seed/도착 속도가 맞지만, 필드 주석에는 "manager의 명목(순항) 속도; 체인 worker는 optimization/max_vel로 그 아래로 cap할 수 있다"를 적어야 한다. 이름은 `nominal_speed`로 동의하되, 코드가 실제로 쓰는 단어는 `cruise`라는 점은 기록한다(초안이 `cruise`를 뺀 이유도 이해한다).

### C3. `FlightPhaseMetrics::zone_hard_*` — 유지(주석 우선), 소비자 범위 보완

초안이 "따로 확인"하라고 남긴 질문은 헤더에서 바로 답이 나온다: `zone_hard_t_first/t_last/runs/qmin*`는 접촉이 있을 때만 존재하는 접촉 지표(`flight_evaluator.h:112-125`, "zone_hard_qmin, which only exists when a contact occurred"), 전체 구간 최근접은 별도 `zone_clear_*`(h:115-121). HARD_AVOID 한정·샘플 수라는 설명도 이미 있다(h:91-100). 개명 비용 메모(2차 정정, §6): R2 검증 하네스(`docs/validation/planning_visualization_2026_10_10/r2/main.cpp:187-188`)가 직렬화하는 것은 `const auto &v=result.evaluation`, 즉 `FlightVerdict::zone_hard_n`·`zone_standoff_n` 등이고, 후보인 `FlightPhaseMetrics` 상세 필드(`zone_hard_t_first/t_last/runs/qmin*`)는 docs/validation 어디에도 직렬화되지 않는다. JSON 키는 문자열 리터럴이라 소스 필드 개명이 스키마를 자동으로 바꾸지도 않는다. "개명하면 비교 스키마가 바뀐다"는 1차 표현은 철회한다. 남는 사실은 하네스 코드가 `FlightVerdict` 필드의 컴파일 타임 소비자라는 것이며, 그 struct는 이번 후보가 아니다. `zone_hard_mode_`(`dyn_a_star.h:146`)·`zone_hard_contacts`(`waypoint_eval.h:349`)는 다른 개념이 같은 접두사를 쓰는 경우라 그대로 둬도 된다.

### C4. `t_abs` → `time_from_start_s` — 유지

`poly_traj.hpp:81-86`("Absolute time → (piece index, piece-local time). t_abs is clamped into [0,total]"), `evalAt` :119-126; RViz 호출자(`altitude_profile_panel.hpp:282`, `.cpp:1862-1867`, `trajectory_metrics_panel.cpp:670-677`)는 인자명뿐. ROS 선례 `trajectory_msgs/JointTrajectoryPoint.time_from_start`가 정확히 같은 뜻이라 초안보다 근거가 더 강하다. 주석의 "Absolute time"도 같이 고쳐야 한다.

### `nextRouteSearchIdentity` → `nextPlanningIdentity` 보류 — 동의

이미 search가 아닌 곳에서 ID를 뽑는다(`planning_execution.cpp:10` store context, `path_manager.cpp:1265, 1416`). 설치 헤더의 공개 자유 함수(`route_search_backend.h:137`)이고 시험이 직접 부르므로 공개 비용이 있다. 보류 타당.

## 4. 규칙·절차에 대한 반론

1. **§4 동사 사전 `mark…For…`**: A2처럼 1회용. `record`로 흡수 권고.
2. **§3 "동사 + 빠진 대상" vs `computeDirect`**: A4. 예외를 적거나 이름에 대상을 넣거나.
3. **§5 `_u_s`**: C2 반론 1. 새 규칙을 세우려면 기존 `_u` 변화율 필드의 처리를 같은 절에 적어야 한다.
4. **§6 "로그 식별자 유지"**: A1. 로그 태그가 옛 함수명을 유지해야 한다는 뜻이면 반대.
5. **§8 묶음 3 forwarding**: 옛 이름을 영구 forwarding으로 두면 `astar…` 같은 오칭이 설치 헤더에 계속 남는다. `[[deprecated("use …")]]`(C++14) 부착과 제거 시점을 문서에 정해 두기를 권한다. 초안에는 deprecation 표기 정책이 없다.
6. **§2 외부 조사**: Qt 원문 "The number one rule of function naming is that it should be clear from the name whether the function has side-effects"(재조회 확인)는 A3의 "한 함수만 compute로 바꾸면 형제의 부작용 추론이 틀어진다"는 논지를 그대로 뒷받침한다. LLVM "Function names should be verb phrases… camel case, and start with a lowercase letter"도 확인. Core Guidelines NL은 오늘 재조회 실패(기억 대조만).
7. **§5 bool**: `FlightVerdict{evaluated=false, clean=true}`(`flight_evaluation.h:29-30`)와 "direct plans keep evaluated=false"(`mission_plan.h:46-47`)를 확인했다. §7 예제 `evaluated && clean`은 맞고, helper 보류에 동의한다.

## 5. 초안에 반영하기를 바라는 최소 수정

1. A3: `computeRoutePlan` → `planFromRoute`(또는 형제 전체 개명 결정).
2. B4: `cachedVoxelCount` → `storedVoxelCount`(2차 합의), 근거 문장 정정(밀집 시 N, 빈 격자 0).
3. B1: `revokeExecutionForZoneChange` → `revokeExecutionForUnevaluatedPolicy` 또는 `revokeExecution(reason)`.
4. C2: 접미사 결정(`_u` 관례 vs 새 `_u_s`), `maxVel()`/`max_velocity` 포함 여부, 체인 ceiling 역할 주석.
5. A2: `markExecutionForInvalidation` → `recordExecutionInvalidation`(선택).
6. 소소: A1 로그 태그(소비자 확인 뒤)·시험 주석, A4 §3 예외 명시(class + out-param 문맥), B3 선언 주석 4단계, §8 deprecation 표기.

## 6. Codex 2차 반론에 대한 확인·정정 (2026-10-13, 같은 날 2차)

Codex가 보고서를 읽고 보낸 반박에 대한 확인이다. 추가 코드 변경·웹 검색은 하지 않았고, 아래 사실은 현재 소스로 다시 확인했다.

| 항목 | Codex 결정/반박 | 확인 결과 |
|---|---|---|
| A3 | `planFromRoute` 채택. public `plan`/`compute` 규칙과 legacy private 모드명 규칙을 분리 | 동의 |
| A2 | `recordExecutionInvalidation` 채택, 멤버 이동은 안 함 | 동의 |
| B1 | `revokeExecutionForUnevaluatedPolicy` 채택. `revokeExecution(reason)` 일반화는 동작 계약 확장이라 제외 | 동의. 사유 인자화는 이름 묶음 밖이 맞다 |
| B4 | `voxelCount()` 반박: `buildEmpty`(sdf_manager.cpp:177-204)와 점유 없는 빌드(:244-258)는 nx/ny/nz·`has_data=true`인데 `distance_cache.size()==0`. 절충 `storedVoxelCount()` | **반박 인정.** 두 경로 모두 `distance_cache.clear()` 후 `static_empty=true`, `static_free_dist` 스칼라만 남긴다(:195-199, :249-253). "항상 shape.prod()"는 틀렸고 `voxelCount()`는 shape와 어긋나는 0을 돌려주므로 철회. `storedVoxelCount()`와 "밀집 시 N, 암묵 all-free면 0, 동적 패치 제외" 주석에 동의. `cached` 반대와 초안 근거 문장 정정 요구는 유지 |
| C2 | `nominal_speed_u_s`를 새 명시 단위 규칙으로 유지하되 `vel_u`/`acc_u`·무접미사 `head_vel`을 legacy 예외로 명시; 요청 3계층만; `PathManager::maxVel()`·`RiskProfileSettings.max_velocity`는 명시 제외; manager 값의 chain 계약 기준/worker ceiling 역할은 별도 설명 | 동의. 내가 요구한 것은 규칙과 예외를 같은 절에 적는 것이었고 충족된다. "제외"는 설계 문서 본문에 문장으로 남겨야 한다 |
| C3 | harness :188은 `FlightVerdict::zone_hard_n` 등이며 후보 `FlightPhaseMetrics::zone_hard_t_first/…`가 아님. 소스 필드 rename은 JSON 키 리터럴을 자동 변경하지 않음 | **반박 인정.** `r2/main.cpp:187` `const auto &v=result.evaluation`(FlightVerdict). `FlightPhaseMetrics` 상세 필드는 docs/validation에 직렬화된 곳이 없다. "즉시 스키마 변경" 주장 철회(§3 C3 본문도 정정) |
| A1 | 단순 debug 태그면 소비자 확인 뒤 갱신, 파싱 태그는 보존 | 동의. 내 검색은 reports/·experiments/를 제외했으므로 소비자 확인이 실제로 필요하다 |
| B3 | 선언 주석 범위·deprecation 정책 보강 | 동의 |
| A4 | `computeDirect`는 class + out-param 문맥 예외를 §3에 명시 | 동의 |
| §4 | 실제 채택은 `RosPlanningBackend::plan`(cpp:213-217)의 `applyPlanningComputation`이며 `computeDirect`가 아니다 | **확인.** 1차 A4의 "FSM/호환 경로" 표현을 정정했다(`plan()`이 `compute()` 직후 :216-217에서 apply) |

이 2차 확인으로 내 쪽 반론은 모두 처리됐다. 설계 문서 통합은 Codex가 한다.

이 문서는 제안이며, 채택 여부는 Codex의 반박 검토와 사용자의 결정에 따른다.
