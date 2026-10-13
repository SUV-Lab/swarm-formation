# 함수명·공개 API·내부 경계 전수조사

2026-10-12 현재 작업 트리 기준. 사용자의 요청은 **리팩터링에 앞서 Claude와 조사하고, 지금 이름을 유지할지 판단**하는 것이다. 이번에는 코드·함수명·설정·그림을 수정하지 않았다.

## 판단

**전체 개명이나 추가 전면 재설계는 권하지 않는다. 공개 진입점은 유지하고, 실제 동작을 잘못 짐작하게 하는 내부 이름과 현행 계약에 맞지 않는 설명부터 좁게 정리하는 것이 좋다.**

함수 이름만의 문제가 아닌 항목도 발견했다. 특히 공개 환경 setter와 ROS 토픽 콜백의 실행 재검증 범위가 다른데 일부 문서는 같은 보장처럼 설명한다. 이것은 리네임으로 해결되지 않으며, 실제 구현 범위를 먼저 정확하게 문서화해야 한다. 외부 setter에 같은 실행 정책을 추가할지는 별도의 기능 결정이다.

## 조사 범위와 증거

상위 저장소 `lgh/dev`, 계획 서브모듈 `mmp_dev`의 현재 작업 트리를 조사했다. 서브모듈 HEAD는 `d2ef5496432b86cfb19d10be5658a369c9cc3725`이며, 미커밋 구현 변경을 포함한다.

| 영역 | 조사 내용 | 담당·원문 |
|---|---|---|
| 계획 공개 API·자료형 | 공개 헤더 39개, 입력·결과·소유·실패·설정 규약, 구현 및 호출자 | [공개 API 조사](public_api_review.md), Claude A/B |
| 내부 조정·실행 | 위 39개에서 제외한 PM/chain/FSM 헤더 3개와 구현, backend·두 저장소·ROS adapter | [조정·실행 조사](orchestration_review.md), Claude E |
| 최적화·탐색·모델 | optimizer 요청/결과/환경/프로필, 검색·SDF·GPU 선언, vehicle 공개 헤더 5개와 연결부 | [지원 라이브러리 조사](supporting_review.md), Claude C/D/F |
| 지도·시뮬레이션·시각화 | Python map loader/frame/mission/serializer, simulation session·전략·follower, 표시 준비/발행, 예제·설치 target | 지원·공개 API 조사, Claude F |
| 교차 검토 | 환경 setter 전체 경로, 시간·좌표 의미, CMake 공개 범위, 변경 영향과 호환성 | Codex 본체 + Claude 반박 검토 |

Claude의 [영역별 조사와 상호 검토 원문](claude_review.md), [축약 전 조사 자료](claude_raw_review.json)도 함께 보존한다. 원문의 258건에는 유지 의견·중복 발견·미검증 후보가 포함되므로 결함 수가 아니다. 아래 종합은 그 의견을 그대로 채택한 목록이 아니라, 추가 소스 대조로 표현·우선순위를 조정한 결과다.

`path_manager` 설치 헤더 42개를 목록 기반으로 모두 확인했다. optimizer/planner/vehicle/traj_utils까지 포함한 설치 트리 헤더는 64개다. 수치 라이브러리 내부는 그 개수와 별개로 아래와 같이 범위를 제한했다.

- MINCO·L-BFGS·root finder 수치 본체를 다시 검증하거나 분해하지 않았다. 공개 solver 계약과 연결부를 조사했다.
- terrain 자산 제작·캐시·mosaic 구현은 이번 API 명명 감사의 대상이 아니다. 관련 모듈은 지도 API 의존 관계와 함수 목록 수준으로 확인했다.
- 저장소 밖 소비자와 금지 디렉토리의 참조는 확인하지 않았다. 따라서 검색 결과가 적다는 이유만으로 public symbol의 외부 사용이 없다고 단정하지 않는다.
- `reports/`와 `experiments/`는 읽지 않았다. stash/checkout/build/실행/테스트도 수행하지 않았다.
- [코드 지문](source_snapshot.json)은 조사 중 코드가 변하지 않았는지 확인하기 위한 260개 허용 경로의 해시다. 260개 파일의 모든 수치식을 검증했다는 뜻이 아니다.
- [참조 목록](reference_counts.json)은 명시한 현재 source/test/example 범위의 문자열 참조다. 선언·로그·주석도 포함하므로 호출 횟수나 외부 호환 소비자 수로 해석하지 않는다.

## 1. 그대로 유지할 공개 이름

| 기능 | 유지할 이름 | 이유 |
|---|---|---|
| 전체 계획 | `MissionPlanner::plan`, `MissionPlanRequest`, `MissionPlanResult` | 호출 목적과 입력·결과 소유가 분명함 |
| backend | `MissionPlanningBackend`, `RosPlanningBackend` | 추상 계약과 현재 ROS 구현을 구별함 |
| 최적화 | `PolyTrajOptimizer::solve`, `OptimizerSolveRequest/Result` | 한 solver 단위 호출이라는 사용자 요구에 맞음 |
| 경로·궤적 계산 | `searchMissionRoute`, `prepareRouteProfile`, `solveDirectTrajectory`, `solveRouteSlice`, `solveRouteSliceBatch` | 검색·후처리·직접/구간 계산의 책임이 구별됨 |
| 연결·평가 | `stitchRouteFlight`, `prependFlightPrefix`, `evaluateFlight`, `FlightEvaluationReport` | 조립과 평가를 구별하며 호출자가 계산을 재사용할 수 있음 |
| 구현 교체 | `VehicleModel`, `RouteSearchBackend` | 알고리즘 이름보다 계약을 드러냄 |
| 시뮬레이션 | `SimulationSession`, `simulate`, `SimulationMotionModel`, `TrackingController` | 계획용 모델과 forward simulation·제어기를 구별함 |
| 지도·표시 | `load_map`, `MapLoadInput/Result`, `makeTrajectoryMessage`, 표시 `make*` | 로딩·값 생성·ROS 발행이 이미 구별됨 |
| 상태 적용 | `PlanningComputation`, `PlanningExecutionStore::apply/revoke` | `apply`는 성공 채택뿐 아니라 실패에 따른 무효화도 적용함 |

`PathManager`, `SegmentChainPlanner`, `ReplanFSM` 클래스 전체를 다른 이름으로 바꾸거나, `ego_planner` namespace·패키지·설치 target을 심미적으로 통일할 필요는 없다. 클래스 이름 변경은 의존 관계 개선을 대신하지 않는다.

`route`, `leg`, `slice`, `segment`, `piece`, `phase`도 같은 뜻이 아니다. 경로 기하, 미션 경유점 사이 정책 구간, 계산용 경로 조각, 계획 구간, 다항식 조각, 비행 단계의 차이를 용어표로 설명하는 편이 낫다.

## 2. 우선 확인된 계약·설명 문제

### 2.1 공개 환경 갱신이 ROS 콜백과 같은 실행 정책을 수행하지 않음

정적 호출 관계는 Codex와 Claude가 각각 확인했다.

| 호출 경로 | 환경 데이터 변경 | 활성 실행의 재검증·철회 |
|---|---|---|
| `MissionPlanner::setTerrain/setRiskZones/addDynamic*` → `RosPlanningBackend` → PM setter | 수행 | 일반 경로에는 없음 |
| 기존 FSM terrain/obstacle/risk 토픽 콜백 | 수행 | 콜백이 별도로 수행 |

근거: `src/path_manager/src/ros_planning_backend.cpp:162`, `path_manager.cpp:2033,2075,2377`, `replan_fsm.cpp:1235,1349,1455`. 위 경로는 계획 서브모듈 기준이다.

미채택 계산의 revision을 무효화하는 것과 이미 채택한 실행 궤적을 철회하는 것은 다르다. 새 지형 설치 중 지연 장애물 flush가 재검증을 수행하는 예외 경로도 있지만, 일반 지형 setter의 재검증 보장은 아니다. 이미 반환한 궤적 값이 유지되는 것은 정상적인 값 소유이며, 여기서의 쟁점은 backend의 활성 실행 상태다.

`mission_planner.h:12–13`과 `planning_library_usage_ko.md:170`은 외부 setter에도 기존 재검증·철회가 적용되는 것처럼 읽힌다. 이 설명은 현재 구현보다 넓다. 기존 ROS 콜백 경로가 고장났다는 발견은 아니며, 외부 setter 동작을 이번에 실행 재현한 결과도 아니다.

**권고:** 현행 책임을 먼저 정확하게 설명한다. 외부 호출자에게 동일 실행 정책을 제공하려면 시간 기준·batch 원자성·초기화·지연 장애물·통지 횟수를 정한 별도 작업으로 다룬다. 단순히 모든 PM setter 끝에 재검증을 추가하면 기존 ROS callback의 일괄 적용 순서가 바뀔 수 있다.

앞선 도식 검토에서 공개 환경 setter와 런타임 콜백의 차이를 충분히 구분하지 못했다. 환경 갱신 보장은 이 조사 결과로 정정한다. 사용자가 그림 작업 종료를 요청했으므로 이번에는 그림 파일을 다시 편집하지 않았다.

### 2.2 우선 정리할 오래된 문서·주석

| 위치 | 현재 문제 | 정리 방향 |
|---|---|---|
| `planning_computation_usage_ko.md:113` | 제거한 public `PathManager::traj_`가 남아 있다고 설명 | const 실행 조회와 소유 결과 규약으로 정정 |
| `segment_chain_planner.h:19–21,43–74` | 채택 주체가 FSM이라고 설명, 옛 bool 반환·시작 상태 인자 설명 잔존 | backend의 compute→apply와 현재 `StartHead`/계산 결과 규약으로 정리 |
| `path_manager.h:424–427`, `path_manager.cpp:2178` | 철회 뒤 WAIT_POSITION, 취소 계층 미구현이라는 설명 | 조건부 ABORT observer와 WAIT_EXTERNAL_RECOVERY 책임을 현재형으로 기술 |
| `path_manager.h:400–403` | SDF 미준비 반환을 -1로 설명 | -2는 대기 접수, -1은 거부, 0 이상은 설치 ID로 구별 |
| `mission_planner.h:25`, environment store clear 경로 | `clearTerrain()`가 동적 장애물·SDF도 제거함을 공개면에서 알기 어려움 | 위험영역은 유지된다는 점까지 설명. `clearEnvironment`로 개명하면 더 부정확 |
| optimizer 헤더 `:660`, PM `optimizeStage` 주석 | `optimizeFromPath()`를 재구현해 backend를 교체하라고 안내 | nonvirtual legacy 경계임을 명시하고 신규 호출은 `solve()`로 안내 |
| `flight_evaluation.h:29` | `evaluated=false`를 degenerate 입력에만 한정해 설명 | 정상 direct 계획의 평가 미실행도 포함. 독립 평가에서는 잘못된 입력·설정·환경 질의 부재도 해당하며, `FlightEvaluationReport`를 받은 경우 `issue`로 원인 확인 |
| `planning_computation.h:57` | `ExecutionReceipt.applied`의 실패 효과 적용 의미가 쉽게 보이지 않음 | `applied`가 새 궤적 성공과 같지 않음을 명시 |
| `point_mass_eom.hpp:12` | 현재 MMP 연결의 축을 `mission ENU`라고 지칭 | SI 스케일 변환은 축 회전이 아님. heading의 +X→+Y 의미와 현 MMP +X 북/+Y 서/+Z 위를 설명 |
| `mmp_traj_utils/poly_traj.hpp:81,118` | `t_abs`/absolute time이 ROS 절대시각처럼 읽힘 | 궤적 시작 기준 상대초로 설명하고 인자명 `t_from_start_s` 권고 |

표의 좌표·시간 정리는 설명과 인자 이름의 정리다. 수식·좌표변환·고도 해석·초기값을 바꾸는 작업이 아니다.

현재 헤더의 상당 부분은 실제 계약 설명과 과거 디버깅 경위가 섞여 있다. 공개 선언 앞에는 현재 역할·단위·소유·실패·동시성 규약을 우선 배치하고, 역사적 설명은 관련 설계 기록으로 연결하면 이름 일괄 교체보다 읽기 쉬워진다.

### 2.3 추가 교차검증: 좌표·초기값·입력 수명

| 대상 | 현재 코드에서 확인한 의미 | 권고 |
|---|---|---|
| `TerrainGrid` | row/col 축·column-major·기존 planner units 안내는 이미 있다. 셀 중심은 `x=center_x+length_x/2−(row+0.5)*resolution`, `y=center_y+length_y/2−(col+0.5)*resolution` | 공개 헤더에도 감소 방향의 식과 u 단위를 적는다. 기존 문서에 단위가 전혀 없다는 주장은 채택하지 않음 |
| `FlightVerdict` | 미평가 기본값은 `evaluated=false`, `clean=true`, `unflyable()==false` | `clean`이나 `!unflyable()`만으로 평가 성공을 판단하지 않도록 설명. 현재 생산 gate는 `evaluated`를 별도로 검사함 |
| `ZonePolicySnapshot::valid` | 기본 true지만 이것만으로 측정된 정책·유효 출처를 증명하지 못함. 실제 consumer는 generation/epoch/shape 등도 검사 | 호환 기본값과 producer 책임 명시. 기본 true와 invalid fail-closed 규칙이 논리적으로 모순이라는 주장은 채택하지 않음 |
| `FlightEvaluationInput` | 생성자가 trajectory/phases의 const 참조를 저장하며 수명을 연장하지 않음 | 저장한 input을 사용하는 동안 원본이 살아 있어야 함을 명시. 현재 생산·예제·테스트는 lvalue를 사용 |
| 공개 ground/ceiling 설정 4곳 | 절대 planner z(u), `> -0.5`이면 활성, `<= -0.5`이면 비활성인 legacy sentinel | `DirectAltitudeConfig`, `RouteSliceConfig`, `OptimizerRuntimeSettings`, `RouteQueryConstraints`에 같은 규칙 명시 |
| `TerrainData::getElevationAndGrad` | pure-water stencil에서 true, 높이 0, gradient 0. 헤더의 pure-water false 설명과 다름 | gradient accessor 주석 정정. `getElevation()`의 pure-water `-inf` 동작은 별개이며 유지 |

근거: `terrain_data.h:193,217`, `planning_environment.h:14`, `flight_evaluation.h:29,73`, `flight_evaluator.h:62`, `flight_evaluator.cpp:75,126`, `segment_chain_planner.cpp:491,569,682,2103,3083,3146`, `zone_policy.h:32`, `path_manager.cpp:2214,2258,2285`. 고도 설정의 소비 규칙은 direct/slice solver와 `dyn_a_star.cpp:382`, `poly_traj_optimizer.cpp:3300,3313`에서 확인했다.

지형의 오해 예시는 **x/y 증가 순서로 만든 배열을 감소 방향의 배열로 해석하게 전달하는 경우**다. 모든 GeoTIFF의 자연 배열이 반전된다는 주장은 틀리다. 현재 Python loader의 north-west raster 및 F-order 직렬화는 이 규칙과 연결되어 있다(`mmp_terrain/frame.py:24`, `ros_terrain.py:20`). `validateTerrainGrid`가 rows×resolution과 length의 일치까지 검사하지 않는 것은 확인했지만, 이를 새로 거부하는 것은 문서 정리가 아닌 동작 변경이다.

참조 수명도 brace-init 자체의 결함은 아니다. `FlightEvaluationInput input{temporaryFlight(), temporaryPhases()};`를 다음 문장에서 사용하면 수명이 끝난 참조를 사용할 수 있다. 반면 같은 full expression 안에서 즉시 `evaluateFlight(FlightEvaluationInput{…})`를 끝내는 형태는 수명상 유효할 수 있다. rvalue 생성자 삭제는 유효한 호출까지 제한할 수 있어 별도 계약 강화로 분류한다.

### 2.4 따로 재현할 기본 고도 설정 불일치

`PathManager`의 ground/ceiling 기본값은 둘 다 `-0.1`이다(`path_manager.cpp:107–108`, `path_manager.h:797–798`). 그런데 옛 헤더 설명은 음수면 비활성이라고 쓰고, 현재 소비 규칙은 `> -0.5`면 활성이다. 따라서 **기본값이 설명과 달리 활성 plane으로 전달되는 사실은 확인**했다.

배포 YAML은 ground `0.02`, ceiling `-1.0`으로 덮어쓰고 현재 mission planner 예제도 해당 YAML을 전달한다. 하지만 저장소 내 모든 경로가 YAML을 읽는 것은 아니다. `terrain_risk_mask_test.cpp:58–68,104`는 YAML 없이 manager를 생성하며 CMake도 그 형태로 등록한다. 이 테스트가 기본값의 전체 미션 계획을 검증한다는 뜻은 아니다.

실제 결과가 항상 `no route`라는 주장도 채택하지 않는다. A*는 활성 ceiling 위 후보를 제외하지만 FM2는 해당 mask를 `kFMin`으로 낮추고(`dyn_a_star.cpp:1760–1774`), optimizer는 plane penalty를 적용한다. 입력과 알고리즘별 실행 재현이 필요하다. `-1.0`으로 기본값을 바꾸는 작업은 no-YAML 동작을 바꾸므로 리네임·주석 수정에 끼워 넣지 않는다.

### 2.5 이름보다 입력 규약을 보강할 추가 항목

- `transition_phase::synthesizeCommands`의 속도창에는 `1.02`, `0.98`, `1.1×stall` 규칙이 있고 `waypoint_eval.cpp:573`이 일부를 역산한다. 정확한 식과 순서를 문서화한다. 공유 상수/helper로 옮기는 일도 산술 순서를 보존하는 별도 작은 변경이다.
- `RouteSearchProblem::model`은 optional이다. 겹치는 제약은 이미 해석된 `constraints`를 사용해야 하며 built-in은 그 cone 값을 쓴다(`builtin_route_search.cpp:72`). 모델에서 다시 계산하면 override와 어긋날 수 있다.
- formation은 요청의 solve 성공 뒤 주 optimizer에 설정된다(`ros_planning_backend.cpp:327`, `path_manager.h:341`). 동일 optimizer가 재사용되고 formation 비용 조건이 충족되는 후속 solve에 영향을 줄 수 있지만, 모든 다음 계획·독립 worker에 적용되는 보장은 아니다.
- `OptimizerSolveRequest`의 명시 head와 마지막 waypoint는 trajectory boundary를 정한다. 원래 path endpoint를 일괄 교체하는 것은 아니며 path는 시간 배분·도착 방향에도 쓰인다(`poly_traj_optimizer.cpp:212,246,274`). 입력 사이 일관성과 중간 waypoint의 risk-anchor 역할을 설명한다. 신규 불일치 거부는 별도 정책이다.
- `TerrainHeightGradientQuery`는 planner `(x_u,y_u)`를 받아 `(h_u, ∂h/∂x, ∂h/∂y)`를 제공한다. 콜백 선언에 이 규칙을 붙인다.
- `DirectPlanSettings`는 기존 헤더에 적힌 ROS adapter overlay다. `MissionPlanner::plan`의 직접 입력형은 아니며 chain 내부에서 사용한다. 새로운 이름을 추가할 필요성은 낮다.

## 3. 좁게 바꾸는 것이 유익한 이름

아래는 수정안이며 아직 적용하지 않았다.

| 현재 이름 | 실제 의미 | 권장안 | 영향·우선순위 |
|---|---|---|---|
| private `ReplanFSM::planFromGlobalTraj(int trial_times)` | 이미 채택된 궤적을 발행. 재계산/재시도 없음 | `publishAdoptedTrajectory()`; 미사용 인자 제거 | 생산 호출 2곳. 우선 |
| private `SegmentChainPlanner::invalidateStoredTrajectory(computed)` | 계산 결과에 실행 무효화 요청 flag만 기록 | `requestExecutionInvalidation(computed)` | 즉시 실행 저장소를 지우는 PM 메서드와 혼동 금지. 우선 |
| private `SegmentChainPlanner::planRouteParallel` | 경로 기반 계획을 조정하며 순차 실행도 지원 | `computeRouteMission` | 병렬 설정·3-pass·분기 그대로. 우선 |
| private `RosPlanningBackend::planDirect` | 직접 경로 계산 결과 생성, 실행 적용 전 | `computeDirect` | 외부 `plan()` 이름은 유지. 우선 |
| public `PathManager::invalidateStoredTrajectoryForEnvChange` | zone-set 변경으로 정책을 평가할 수 없을 때 실제 실행을 revoke하고 observer 통지 | `revokeExecutionForZoneChange` | chain의 무효화 요청과 실제 철회를 구별. 기존 wrapper 검토, 호출 전 변경 판정 조건 명시 |
| `validateFlightPhases` | 궤적의 시간·계수와 단계 순서 모두 검증 | `validateTrajectoryAndPhases` | 설정/환경까지 검사하는 이름으로 넓히지 않음. 공개 wrapper 유지 가능 |
| `PathSearcher::astarSearchAndGetSimplePath` | 설정에 따라 FM2 또는 A* 실행 후 단순화 | `searchAndSimplifyPath` | 새 이름 + 기존 forwarding wrapper. 실제 `AstarSearch`는 유지 |
| `SDFManager::numAllocatedBlocks` | static distance cache에 저장된 voxel 수 | `cachedVoxelCount` | 전체 지도 voxel 수와도 다름. 기존 wrapper 유지 가능 |
| `nextRouteSearchIdentity` | 검색·실행 문맥에서 쓰는 공통 원자 ID 생성 | 내부 `nextPlanningIdentity` | 낮은 우선순위. 현재 하나의 sequence를 유지하며 counter 복제 금지 |
| `SimulationSession::controller(t,state)` | 전략 객체가 아니라 model의 realize를 거친 control 값을 반환 | 선택적 `realizedControlAt` | 의미 혼동은 있으나 C++ 이름 충돌은 아님. 낮은 우선순위, 기존 public 함수 호환 고려 |

근거와 호출 위치는 영역별 원문 및 [참조 목록](reference_counts.json)에 있다. private rename은 source 내부 전환으로 충분하지만, 이미 설치된 public 함수는 외부 사용 여부를 알 수 없어 호환 wrapper 또는 명시적 버전 경계가 필요하다.

`validateFlightPhases`, `astarSearchAndGetSimplePath`, `numAllocatedBlocks`, `nextRouteSearchIdentity`의 구체 개명 근거는 Codex 측 코드 검토에서 확인했다. Claude는 방향에 반대하지 않았지만 이 네 항목을 독립 확인했다고 주장하지 않았다.

소스 호환과 바이너리 링크 호환도 구별한다. 기존 out-of-line 함수를 header inline wrapper로만 바꾸면 예전 바이너리가 찾는 symbol이 사라질 수 있다. 기존 바이너리를 지원해야 한다면 기존 symbol을 유지하는 forwarding 정의가 필요하다. 현재 설치 헤더가 존재한다는 사실만으로 장기 ABI 보장을 새로 약속하지 않는다.

`planInto`, `planImpl`, `computeAndPublishPaths`, `deliverTrajToOptimizer`, 표시 adapter의 짧은 메서드 이름도 개선 여지가 있지만 우선순위는 낮다. 현재 책임 주석과 공개/호환 분류를 먼저 정리한 뒤 필요할 때 다룬다. 일괄 이름 규칙 통일은 하지 않는다.

## 4. 공개 필드 개명은 별도 묶음으로 판단

| 대상 | 혼동 | 제안과 결정 |
|---|---|---|
| `EntryCandidate/TransitionResult::route_start_s` | 주변 `_s`는 초인데 이 필드는 경로 호 길이 u | `route_start_arc_u`가 정확함. 다만 공개 aggregate 필드라 source compatibility를 바꿈 |
| direct/slice/optimizer 요청의 `max_vel` | 최대 속도 제약처럼 보이지만 seed·도착 상태 생성 속도 | `seed_speed_u_s` 등으로 3계층을 함께 정리할 가치는 있음. 실제 optimizer 제한 `optimization/max_vel` 및 기존 YAML 키는 유지 |
| `FlightPhaseMetrics::zone_hard_t_first/t_last/runs` 등 상세 필드 | HARD_AVOID 정책에서 authored volume 또는 standoff 접촉을 기록. `zone_hard_n`은 authored volume만 셈 | 우선 각 필드 범위 설명. 개명한다면 SOFT까지 포함하는 듯한 `zone_contact_*` 대신 `hard_avoid_contact_*` 등 정책 한정을 유지 |

이 공개 필드들은 이름 개선의 실익이 있지만 내부 메서드 정리와 같은 위험도는 아니다. `route_start_s`와 `max_vel`은 이미 헤더에 현 의미가 적혀 있으므로 당장 기능 결함으로 분류하지 않는다. 외부 호환을 유지하려고 서로 독립된 alias 데이터 필드를 두면 두 값의 일관성 문제가 생길 수 있다. 소비자를 함께 이전하거나 버전 경계에서 변경해야 한다.

함수 인자 `t_abs`를 `t_from_start_s`로 바꾸는 일은 필드 개명과 다르다. 호출 형식·ABI를 바꾸지 않으며, 같은 의미를 쓰는 RViz 패널 wrapper의 주석도 맞추면 된다.

## 5. 외부 라이브러리 사용성을 위한 경계 정리

### 전역 이름 노출

`poly_traj_optimizer.h:14,23,35,42`는 전역 `LogManager` 별칭과 `LOG_INFO/WARN/ERROR` 매크로를 노출한다. `dyn_a_star.h`에는 전역 `inf`, `path_manager.h:46`에는 전역 `using namespace ego_planner`도 있다. 외부 소스가 헤더를 include하는 것만으로 일반적인 이름과 충돌할 수 있다.

이것은 계산 알고리즘과 무관한 라이브러리 헤더 정리다. 로그 구현을 `.cpp`/비공개 영역으로 옮기거나 프로젝트 접두어를 사용하고, 불필요한 별칭·상수는 namespace 안으로 제한하는 편이 좋다. 모든 namespace·타깃 이름을 바꿀 이유는 없다. 기존 외부 소스가 우연히 이런 전역 이름에 의존했는지는 알 수 없으므로 의도된 공개 API와 구분해서 취급한다.

### 설치된 헤더와 지원 API의 구별

CMake는 현재 `include/` 전체를 설치한다. 이는 호환 조정 헤더를 모두 신규 권장 API로 만든다는 뜻이 아니다. 기존 API 색인은 이 구분을 이미 설명한다. 일반 사용자용 API, 실행 어댑터용 하위 API, legacy wrapper, 내부 구현의 분류를 각 헤더에도 간단히 표시하면 충분하다.

선택 CUDA용 `fm2_gpu.h`도 항상 설치되지만 구현은 CUDA 빌드에서만 포함되고 `PP_HAVE_CUDA`는 PRIVATE다. 일반 `RouteSearchBackend` 사용에는 문제가 없지만 이 하위 헤더를 외부 API처럼 쓰면 CPU 설치에서 링크할 수 없는 조합이 생긴다. 내부 헤더로 한정하거나 CPU용 capability stub을 제공할지 별도로 결정한다. CPU 빌드 재현을 이번에 수행한 것은 아니다.

## 6. 리네임과 섞지 않을 계약 확장 후보

| 항목 | 현재 의미 | 권고 |
|---|---|---|
| 미션 요청의 `length_per_piece/obstacle_clearance` | 양수 값이 이후 호출에도 지속됨. 기존 ROS 호환 의도이며 문서·테스트 존재 | 의미를 유지. 명시 설정 갱신과 per-call option 분리는 별도 설계 |
| direct/slice 실패 사유 | optimizer의 잘못된 입력/미구성/수치 실패를 일부 `OPTIMIZER_FAILED`로 합침 | 더 자세한 원인을 원한다면 result 계약 확장. 이름 변경만으로 해결하지 않음 |
| 채택 직전 stale 결과 | backend가 FAILED + 문자열 detail로 반환하며 기계 판독 stale reason은 없음 | 필요하면 명시 reason 추가. 원래 계산 SUCCESS를 그대로 보존하면 빈 궤적을 성공으로 노출할 수 있어 금지 |
| `addDynamicSphere/Box` int 결과 | 0 이상 ID, -2 정상 대기, -1 거부 | typed result는 개선 가능하지만 virtual interface 계약 변경 |
| `reference_trajectory` | direct seed / baseline 기준 궤적 / route 완성 비행의 모드별 의미 | 이름 유지. 종류를 프로그램으로 구별해야 하면 enum 메타데이터 검토 |
| `compute()` | 새 후보를 채택하지 않음. 환경 준비·일부 진단 등까지 순수한 함수는 아님 | 현재 문서에 있음. 함수명만으로 side-effect-free 계약을 새로 약속하지 않음 |
| 미평가와 clean 구별 | `evaluated && clean`을 명시해야 평가된 clean 결과를 뜻함 | 선택적 `isEvaluatedAndClean()` helper 검토. bool helper를 tri-state라 부르거나 비행 허가 판정으로 설명하지 않음 |
| 정책 기본값·입력 참조 | valid의 호환 기본값과 borrowed 입력 수명 | default false 전환·rvalue 삭제는 동작/소스 계약 변경. 우선 현재 규칙 문서화 |
| no-YAML 고도 기본값 | `-0.1`은 현재 활성 규칙에 해당 | 별도 실행 재현 후 기본값 결정. 문서 수정과 함께 조용히 변경하지 않음 |
| legacy `RosPlanningBackend::setTerrainData(GridMap)` | void adapter가 manager의 `TerrainUpdate`를 버림 | 현재 facade `setTerrain`은 bool 반환. 필요하면 별도 checked 경계 검토; 반환형 교체가 멤버함수 포인터·ABI까지 호환인 것은 아님 |

`PlanReason`의 순서는 기존 저하 사유 우선순위와 관련된다. 보기 좋게 enum을 재정렬하거나 모든 실패 처리 방식을 통일하는 일은 이름 리팩터링이 아니다.

현재 `evaluated && !unflyable()`로 DEGRADED 비행을 허용하는 gate를 `evaluated && clean`으로 교체하면 결과 수락 정책이 바뀐다. helper 추가를 기존 gate 전면 교체의 근거로 삼지 않는다. Claude 원문에 남은 기타 버그 의심 후보도 런타임 재현 없이 확정 결함으로 승격하지 않았다.

## 7. 권장 구현 순서와 검증 범위

1. **현재 계약·주석 정정:** 공개 환경 갱신, clearTerrain 부작용, 실행 채택 주체·ABORT·반환 코드, 격자/시간/좌표·고도 sentinel, 미평가 결과·borrowed 수명, 제거된 필드 문서. 런타임 동작은 바꾸지 않는다.
2. **좁은 내부 이름·헤더 노출 정리:** 우선 private 메서드 4개와 미사용 인자, 일반 전역 로그 매크로·별칭. 함수 호출 순서·설정·로그 메시지의 외부 의미는 보존한다.
3. **필요한 public 명칭 이전:** validator·검색·SDF 이름은 기존 wrapper를 남기는 방식 검토. 공개 필드 2종은 별도 source compatibility 결정 후 진행.
4. **환경 실행 정책 등 계약 확장:** 필요성이 합의될 때 별도 작업으로 설계·재현·구현한다. 큰 함수 재분해나 새 추상 클래스를 명명 정리에 끼워 넣지 않는다.

실제 수정 시에만 영향에 맞춰 검증한다. private rename·헤더 정리는 컨테이너 빌드와 관련 FSM/chain/설치 소비자 검사를 우선하고, 호출 경로나 정책까지 바뀌면 동일 R2 입력 궤적·병렬/3-pass·ABORT/복구를 다시 비교한다. 공개 환경 정책을 확장하면 ROS callback과 공개 API 경로의 활성 실행 ID·철회 통지·no-op 재발행·batch 순서를 함께 검사해야 한다.

이번 결과는 정적 명명·계약 조사다. 기존 실행 성공률, 성능 또는 수치 동일성을 새로 측정했다고 주장하지 않는다.

조사 종료 시 260개 코드 지문이 시작 시점과 모두 일치했고 상위/서브모듈 HEAD도 같았다. 이 요약의 로컬 링크와 보존한 Claude 원문의 동일성도 확인했다. [조사 산출물 확인 기록](artifact_checks.json)에 결과와 파일 해시를 남겼다. 새로 작성한 조사 문서 외에 소스·기존 설계 문서·그림은 수정하지 않았다.
