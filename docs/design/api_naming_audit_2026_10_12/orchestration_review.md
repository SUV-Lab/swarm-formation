# Runtime/orchestration naming audit — 2026-10-12

작업 범위: 읽기 전용 정적 감사. 저장소 구현·빌드·시험 실행·수정 없음. reports/ 및 experiments/ 내용은 조사 대상에서 제외. 아래 경로는 `src/mmp_path_planning/src/path_manager/` 기준이다.

결론: 큰 클래스·공개 API를 일괄 개명할 필요는 없다. 동작을 잘못 설명하는 내부 이름과 오래된 주석을 먼저 정리하는 편이 좋다. 이름 변경보다 먼저 판단해야 할 환경 갱신의 공개 API 계약 차이도 발견했다.

## 1. 우선 가치가 높은 이름 변경

| 현재 이름 | 확인된 동작·오독 사례 | 권장 처리 | 근거·영향 |
|---|---|---|---|
| ReplanFSM::planFromGlobalTraj(int trial_times) | 계산하지 않고 이미 채택된 궤적을 확인하여 pending_plan_ 또는 실행 저장소에서 ROS 토픽으로 발행한다. trial_times는 미사용. 재계획 N회로 오해 가능. | publishAdoptedTrajectory(), 미사용 인자 삭제 | src/replan_fsm.cpp:516–555, 호출 352,387. private이므로 낮은 위험. 로그 태그 변경은 문자열 기반 검증 영향 확인 필요. |
| SegmentChainPlanner::invalidateStoredTrajectory(computed) | live 실행 궤적을 지우지 않고 computed.invalidate_execution=true만 기록한다. 계산 단계가 실행 저장소를 변경하는 것처럼 읽힌다. | requestExecutionInvalidation 또는 markExecutionInvalidation | src/segment_chain_planner.cpp:222–226, header618. private, 내부 8개 호출. 계산/채택 분리를 가리는 가장 명백한 이름. |
| SegmentChainPlanner::planRouteParallel | run_parallel=false일 때도 호출된다. 실제 의미는 baseline 없이 route에서 경계 상태를 만들어 계획하는 모드. | computeRouteMission | cpp418–453,1937–1944, header385. private. planBaselineMission→computeBaselineMission과 함께 맞추면 일관됨. chain/parallel 및 로그·설정 키 유지. |
| RosPlanningBackend::planDirect | compute() 내부에서 계산 결과와 상태를 준비하고, 채택은 바깥 plan()이 한다. | computeDirect | src/ros_planning_backend.cpp:320,333–402, header67. private. public plan()은 유지. |
| SegmentChainPlanner::planInto / planImpl | 각각 시작·꼬리 경계 검증 및 전이 분기 / multi-waypoint·route·baseline 모드 분기. Into/Impl은 책임을 설명하지 못한다. | 선택적으로 validateAndComputeMission / computeCruiseMission. 역할 주석을 먼저 현재형으로 정리 | cpp264–417,418–453. private. 첫 이름을 computeValidatedMission으로 하면 이미 검증된 입력처럼 읽힐 수 있어 주의. 위 4개보다 우선순위 낮음. |
| ReplanFSM::computeAndPublishPaths | 10ms FSM tick으로 상태 전이·채택 결과 발행·완료 감시·복구 대기 수행. | 선택적으로 tickExecutionState 또는 onExecutionTimer | cpp322–473. public 노출이라 compatibility wrapper 또는 호출자 전환 확인 필요. 지금 반드시 바꿀 항목은 아님. |

planMultiWaypoint도 실제로 chain을 하지 않고 single-shot fallback 후 전체 평가를 한다(cpp456–524). computeMultiWaypointFallback은 도움이 되지만 현재 이름이 직접 거짓이라고 보기는 어렵다.

## 2. 공개 호환 함수는 삭제·대량 개명보다 구분이 우선

- PathManager::planGlobalTraj는 compute→apply 호환 wrapper(cpp1079–1104). front_end_only=true 예외도 기존 계약이다. computeGlobalTrajectory와 이름을 합치면 채택 부작용이 섞인다. 유지하고 legacy compute+adopt 그룹으로 명시한다.
- PathManager::solveSlice는 solveRouteSlice의 bool/out-parameter compatibility wrapper(cpp1705–1722)이며 알고리즘 중복이 아니다. 유지하고 새 사용자는 값 반환 함수를 안내한다.
- SegmentChainPlanner::plan 및 planOverRoute는 계산 후 실행 효과를 적용하는 호환 API(cpp254–261,2495–2524)이다. public compute()와 다르므로 일괄 compute*로 개명하지 않는다.
- SegmentChainPlanner::commitRoute / PathManager::planCommittedRoute는 경로를 선택하여 소유 값으로 고정한다는 도메인 의미가 있다. 실행 채택과 혼동되지 않도록 설명하면 충분하다.
- deliverTrajToOptimizer는 다른 기체들의 swarm trajectory vector 주소를 바인딩(header333–336)한다. bindSwarmTrajectoriesToOptimizer가 정확하지만 public이며 backend 초기화·여러 테스트 사용이 있어 새 이름+기존 wrapper 또는 문서 우선이 적절하다.
- polyTraj2ROSMsg/globalTraj2ROSMsg는 state-based compatibility wrapper(cpp488–514)다. 새 makeTrajectoryMessage가 이미 값 입력·명확한 이름을 제공한다. globalTraj2ROSMsg는 production-src/include 검색에서 정의·선언 외 호출이 없었지만 외부 사용·테스트까지 없다고 단정하여 삭제하면 안 된다.
- EmergencyStop의 핵심은 대문자 스타일보다 실제 행동이 정지 명령/ABORT가 아니라 정지 위치 hover 궤적 생성·채택이라는 점(cpp1762–1795)이다. 필요하면 adoptEmergencyHoldTrajectory라는 wrapper를 검토할 수 있다. ABORT/revoke와 합치지 않는다.

## 3. 유지할 이름

- RosPlanningBackend: 현행 ROS 설정·환경 연결·호환 실행 효과를 조정한다는 사실을 드러낸다.
- PathManager, SegmentChainPlanner, ReplanFSM: 잔여 복잡성은 있으나 클래스명 변경은 경계 개선이 아니다. PlanningCoordinator 등으로 개명하면 diff만 커진다.
- PlanningEnvironmentStore, PlanningExecutionStore: 소유 책임과 일치한다.
- computeGlobalTrajectory, prepareRoute, prepareBaselineSegments, solveBaselineSegments, solveRouteSegments, repairRouteSegments, finishRouteFlight: 현재 목적을 충분히 전달한다.
- beginPlanningComputation, sealPlanningComputation, applyPlanningComputation: 순차 실행/context 검증 프로토콜을 설명한다. apply는 성공 채택뿐 아니라 실패 무효화도 적용하므로 무조건 adopt로 좁히면 부정확하다.
- PlanningExecutionStore::revoke vs invalidatePending: 활성 실행 철회와 미적용 계산 무효화의 구별을 유지한다.
- makeTrajectoryMessage, encodeTerrainGrid, decodeTerrainGrid, inspectTerrainGrid, readRosBuiltinRouteSearchConfig: 변환/검사/설정 읽기가 구별된다.
- RosPlanningVisualization의 frontEndPath/tube/chain 등은 실제 publish만 수행(cpp23–38)한다. publish* 명명 개선은 가능하나 필수는 아니다. clearTrajectory는 시각화 메시지 삭제이며 clearTrajectoryVisualization은 낮은 우선순위 선택안이다.

## 4. 개명보다 먼저 고칠 오래된 주석·로그

1. segment_chain_planner.h:19–21은 ROS command adoption이 FSM에 남았다고 하지만 실제 adoption은 RosPlanningBackend::plan의 applyPlanningComputation(cpp213–217)이다. FSM은 ROS 발행·상태 전이·ABORT 담당이다.
2. segment_chain_planner.h:43–74의 on true, start_vel_synthesized/start_vel_commanded 등 옛 API 설명이 compute() 앞에 남아 있다. 현재 StartHead+PlanningComputation 계약을 짧게 앞에 두고 이력은 설계 기록으로 옮긴다.
3. path_manager.h:424–427의 parks in WAIT_POSITION은 환경 revoke 경로와 맞지 않는다. publishExecutionAbort(cpp934–978)는 WAIT_EXTERNAL_RECOVERY로 전이한다. 모든 invalidation이 ABORT는 아니므로 실패 무효화와 환경 revocation을 구별한다.
4. path_manager.cpp:2178–2183 로그의 cancel layer is an open decision은 현재 구현을 부정한다. execution_->revoke는 observer가 있을 때 FSM ABORT를 유발한다(cpp2138, FSM934–978). standalone backend에 publisher가 없을 수 있으므로 등록된 실행 어댑터가 통지/복구를 담당한다고 표현해야 한다.
5. path_manager.h:400–403은 SDF 전 추가가 -1이라고 설명하나 실제로 -2 queued, malformed/rejected -1이다. planning_environment_store.h:89와 backend contract에는 올바르게 적혀 있다.
6. optimizeStage cpp1604–1606의 Swap optimizeFromPath() to replace the backend 안내는 현재 solveDirectTrajectory→optimizer 경계 및 MINCO/LBFGS 본체 봉인 방침과 어긋나는 수정 지점을 안내한다.
7. replan_fsm.h:100–104는 세 callback group/타이머 전용 그룹을 설명하지만 실제 타이머는 subscription_callback_group_에 붙는다(cpp322). 현재 직렬 계약과 맞춰야 한다.

## 5. 이름 변경과 분리해야 할 계약 검토

### 공개 환경 setter와 ROS callback의 실행 재검증 차이

정적 호출 대조로 차이를 확인했지만 런타임 재현 시험은 하지 않았다. 따라서 이번에 확정 재현한 실행 고장이라고 보고하지 않는다.

- mission_planner.h:10–13: live environment updates가 기존 revalidation/withdrawal 정책을 유지한다고 읽힌다.
- RosPlanningBackend::setTerrain(cpp162–169)→PM::setTerrainData; addDynamicSphere/Box(cpp185–194)→PM add; setRiskZones(cpp181–183)→PM::setRiskZonesRuntime.
- PM::setTerrainData(cpp2033–2063)는 ingest/bind/visualize이며 일반 새 terrain 자체 실행 재검증은 없다. deferred obstacle flush에는 별도 검사 있음.
- PM::addDynamicSphere/Box(cpp2075–2090)는 add/visualize이며 기존 활성 실행 재검증 없음.
- PM::setRiskZonesRuntime(cpp2377–2392)는 field/binding/visualize이며 내용 변경 철회 없음.
- ROS terrain callback cpp1235–1237은 revalidateAfterEnvChange 추가 호출.
- ROS obstacle callback cpp1342–1355는 revalidateStoredTrajectory 추가 호출.
- ROS risk callback cpp1455–1457은 fingerprint 변화 시 invalidateStoredTrajectoryForEnvChange 호출.

구체적 시나리오: 외부 호출자가 plan()으로 결과를 채택한 뒤 addDynamicSphere()로 궤적 앞 장애물을 추가하면 환경 갱신이 실행 재검증도 한다고 기대할 수 있으나 기존 ROS callback의 검사 호출을 거치지 않는다. 반환 궤적 값이 immutable인 것은 정상이다. 쟁점은 backend 내부 활성 실행 상태와 정책 약속이다.

권고:
1. Claude와 intended contract 독립 대조.
2. 환경 데이터 변경과 active execution revalidation을 누가 묶는지 명시.
3. 기능 수정이 필요하면 이름 정리와 별도 단위로 검증. 같은 입력의 공개 setter/ROS callback 경로에서 revoke 이벤트·ID·active duration·동일 재발행 no-op 비교.
4. 일괄 PM setter로 검사를 옮기면 ROS batch가 누적된 뒤 한 번 검사하던 순서, 초기화/deferred 설치, 이벤트 횟수가 달라질 수 있으므로 무작정 이동 금지.

낮은 우선순위 별도 후보: ReplanFSM::callEmergencyStop cpp989가 PM EmergencyStop의 bool 실패를 무시하고 current 궤적을 포장·발행한다. 현재 생성 궤적은 항상 nonempty이고 주 실패 조건은 identity overflow 등 드문 상태다. 이름 변경으로 해결되는 이슈는 아니다.

## 6. 큰 함수·잔여 구조

아직 긴 함수: PM::prepareRoute cpp833–1063, planFrontEnd1384–1592; chain::authorContractsFromRoute1250–1523, planTransitionMission1642–1936; FSM::startMissionPlan593–812, trajectoryCommandCallback1000–1224.

이미 경로/segment solve/assembly/evaluation 경계가 있고 긴 부분에는 현행 정책·검증·진단이 상당량 포함되어 있다. 이름 정리라는 이유로 재분해할 근거는 부족하다. 새 기능으로 변경 압력이 생길 때 입력 변환/정책 선택/진단 경계를 별도로 판단한다. MINCO/LBFGS 본체는 조사·분해 대상으로 삼지 않았다.

낮은 우선순위 결합 후보: PlanningExecutionStore 생성자가 nextRouteSearchIdentity()를 사용(planning_execution.cpp:2,9–10; route_search_backend.cpp:8–10). 공유 atomic ID 생성기인데 탐색 전용 이름/헤더라 execution→search 의존처럼 보인다. 필요할 때 generic internal context-ID helper로 이동 가능하나 현재 동작 문제는 아니다.

## 7. 검토 범위·한계

검토 선언·구현: path_manager, segment_chain_planner, replan_fsm, ros_planning_backend, planning_execution, planning_computation, planning_environment_store, ros_route_search, ros_terrain_grid, ros_planning_visualization, trajectory_message. 계약 대조: mission_planner.h, mission_planning_backend.h, route identity 선언/정의. 관련 테스트의 호출자·현재 assertion 일부.

함수 목록을 추출하고 조정/계산/채택/발행 경계의 구현 및 호출자를 집중 대조했다. 모든 긴 함수의 모든 수치식을 다시 증명한 수치/알고리즘 전수 검수는 아니다. 외부 저장소 호출자, ABI 호환 고객, 실시간 실행은 확인하지 않았다. reports/ 및 experiments/ 내용은 조사 대상에서 제외했다. 저장소 파일은 변경하지 않았다.
