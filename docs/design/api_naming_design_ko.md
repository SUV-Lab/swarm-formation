# MMP 라이브러리 API 명명 설계

2026-10-13 설계. **같은 날 사용자 승인에 따라 계약 설명과 내부 함수 네 개만
구현·검증했다.** 공개 함수·필드 개명 등 나머지 항목은 제안 상태다.
[적용 범위와 최종 검증](../validation/api_naming_2026_10_13/README.md)을 참고한다.

사용자의 요청에 따라 Claude 세션이 독립 검토했고, 반론을 현재 소스에서 대조해 아래 설계에 반영했다. 공동 검토의 판단과 확인 범위는 §9에 기록한다.

기준: 상위 `lgh/dev`의 `13ced33052fcbe7580b96e79d4f44574fefb6f6d`, 계획 서브모듈 `mmp_dev`의 `068f386f7a3eadd21ab1bd9308727e6eca7ba641` 및 현재 작업 트리. [전날의 명명·계약 조사](api_naming_audit_2026_10_12/README.md)를 출발점으로 삼되, 후보의 실제 동작은 현재 소스에서 다시 확인했다.

## 1. 결정

**기존 C++ 표기법과 핵심 공개 API는 유지한다. 계산·검사·상태 반영·발행의 동사를 구별하고, 잘못된 역할을 암시하는 이름만 바꾼다.** 클래스와 인자가 제공하는 문맥을 활용해 불필요하게 긴 이름을 피한다.

이를 위해 공식 지침 다섯 종류와 실제 계획·최적화 라이브러리 네 종류를 비교했다. 표기법의 유일한 C++ 표준이나 `compute` 같은 동사의 보편적인 부작용 규약은 없었다. 아래 규칙은 그 자료와 MMP의 실제 동작을 바탕으로 정한 **MMP 내부 규칙**이다.

이번 설계에는 ROS 제거, C++ 포팅, MINCO/L-BFGS 분해, 알고리즘·좌표·단위·실패 정책 변경을 포함하지 않는다. `reports/`·`experiments/`는 조사하지 않았다.

## 2. 외부 조사에서 확인한 것

각 링크는 실제로 확인한 공식 지침 또는 프로젝트의 공개 API다. 조사일은 위 날짜이며, 프로젝트별 관례를 모든 C++ 코드의 강제 규칙으로 해석하지 않는다.

| 출처 | 확인한 관례 | MMP에 적용할 판단 |
|---|---|---|
| [C++ Core Guidelines NL.7·8·10](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#Rl-name) | 일관성과 범위에 맞는 이름을 강조. underscore 선호도 기존 스타일보다 우선하지 않음 | 현재 스타일을 유지하고 공개 이름의 명확성을 우선 |
| [LLVM Coding Standards](https://llvm.org/docs/CodingStandards.html#name-types-functions-variables-and-enumerators-properly) | 함수는 lowerCamelCase 동사구, 타입은 명사. 의미와 맞는 이름 및 알려진 약어 사용 | MMP의 함수·타입 표기를 유지할 선례. LLVM 변수 규칙까지 도입하지 않음 |
| [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html#Naming) | 일반 함수는 PascalCase, 일부 접근자는 snake_case. 범위 밖 독자도 이해할 이름 권장 | 대소문자를 베끼기보다 문맥·가독성 원칙 적용 |
| [ROS 2 Humble 개발 지침 원문](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/The-ROS2-Project/Contributing/Code-Style-Language-Versions.rst) | Google 기반에 snake_case 허용. 기존 프로젝트의 소급 스타일 변경을 경계 | ROS 사용이 기존 함수를 일괄 snake_case로 바꿀 이유는 아님 |
| [Qt API Design Principles](https://wiki.qt.io/API_Design_Principles#Naming_Functions_and_Parameters) | 이름의 효과·인자 의미, 일관된 어휘, 실제 사용 코드의 가독성을 중시 | 함수명을 읽고 계산인지 변경인지 구별하도록 설계. 모든 bool에 같은 접두사를 강제하지 않음 |
| [OMPL Planner](https://ompl.kavrakilab.org/classompl_1_1base_1_1Planner.html), [SimpleSetup](https://ompl.kavrakilab.org/classompl_1_1geometric_1_1SimpleSetup.html) | `setup`, `solve`, `getSolutionPath`, `clear`. solve는 상태를 반환하고 해는 내부에 보관할 수 있음 | `solve`는 익숙한 용어지만 결과 소유·무상태성까지 보장하지 않음 |
| [MoveIt MoveGroupInterface](https://moveit.picknik.ai/main/api/html/classmoveit_1_1planning__interface_1_1MoveGroupInterface.html) | `plan`, `execute`, `computeCartesianPath`. 계획과 실행 구별; 출력 인자와 부분 완성 비율도 사용 | 목적별 동사를 참고. MMP `plan`의 실행 저장소 채택까지 MoveIt과 동일하다고 설명하지 않음 |
| [Ceres Solver 공개 헤더](https://raw.githubusercontent.com/ceres-solver/ceres-solver/master/include/ceres/solver.h), [튜토리얼 원문](https://raw.githubusercontent.com/ceres-solver/ceres-solver/master/docs/source/nnls_tutorial.rst) | `Solve`, `Options`, `Problem`, `Summary`. 파라미터 메모리의 해와 진단 요약을 구별 | optimizer의 `solve` 유지. 소유 궤적을 포함한 MMP Result를 Summary로 바꾸지 않음 |
| [Ruckig 튜토리얼](https://docs.ruckig.com/tutorial.html), [API](https://docs.ruckig.com/classruckig_1_1Ruckig.html) | 전체 궤적 `calculate`와 주기별 `update`, 별도 상태 코드·출력 객체 | 시간 진행과 계산을 구별. Result라는 단어 자체가 소유 결과 객체를 뜻하는 것은 아님 |

## 3. 표기와 이름 길이

| 대상 | MMP 규칙 | 예 |
|---|---|---|
| C++ 함수·메서드 | 기존 lowerCamelCase | `solveRouteSlice`, `setVehicleModel` |
| 클래스·구조체·enum 타입 | UpperCamelCase | `MissionPlanRequest`, `VehicleModel` |
| 인자·값 필드·지역 변수 | 기존 snake_case | `trajectory_id`, `time_from_start_s` |
| private 데이터 멤버 | 기존 끝 `_` 표기 | `backend_` |
| namespace·파일·CMake target | 기존 이름·각 언어 규칙 유지 | `path_manager`, `mission_planner.h` |
| 상수·enum 값 | 기존 구성요소의 규칙 유지. 새 scoped enum은 UpperCamelCase를 기본으로 하되 기존 enum에 추가할 때 그 enum의 표기를 따름 | 기존 `PlanOutcome::FAILED` 재철자·재정렬 금지 |
| 매크로 | 가능한 비공개화. 불가피한 프로젝트 매크로는 식별 가능한 접두사 사용 | 일반 이름 `LOG_INFO`를 설치 헤더 밖으로 제한하는 방향 |

함수명은 **동사 + 문맥에서 빠진 대상**으로 정한다. `MissionPlanner::plan()`은 충분하며 `MissionPlanner::planMission()`을 추가하지 않는다. 반면 자유 함수 `solveRouteSlice()`에는 대상이 필요하다. `PathSearcher::searchAndSimplify()`에서는 Path를 반복할 필요가 없다. private 하위 단계는 클래스·호출 위치·결과 인자가 대상을 분명히 하면 `RosPlanningBackend::computeDirect(..., PlanningComputation&)`처럼 대상 반복을 생략할 수 있다.

새 공개 이름에서는 `traj`, `env`, `mgr`처럼 철자를 추측해야 하는 축약을 피한다. 분야에서 통용되는 ROS, SDF, AGL 등은 허용한다. 기존 `PolyTrajOptimizer`나 외부 라이브러리의 이름을 이 원칙만으로 전면 개명하지 않는다.

`Parallel`, `Async`, `Gpu`는 호출 계약이 실제로 그 방식을 보장할 때만 넣는다. 직렬 실행도 가능한 묶음은 `Batch`나 기능 이름을 사용한다. 실제 A* 전용 함수와 FM2/A* 선택 함수도 구별한다.

## 4. 동사 사전

다음은 MMP의 사용 규칙이다. 단어 하나가 소유권·스레드 안전성·순수성을 보장하는 것은 아니며 선언의 계약을 함께 읽어야 한다.

| 이름 | 담당 의미 | 현재 적용·주의점 |
|---|---|---|
| `plan` | 계획 요청 또는 기존 private 계획 모드 | 공개 `MissionPlanner::plan`과 호환 plan wrapper는 계산 후 실행 저장소에 효과를 적용. 기존 private `plan*` 모드 함수들은 계산만 하므로 접두사만으로 채택을 추정하지 않음 |
| `solve` | 설정된 문제에 대한 수치 해 또는 최적화 결과 계산 | `PolyTrajOptimizer::solve`, `solveDirectTrajectory`, `solveRouteSlice`, `solveRouteSliceBatch` 유지 |
| `search` | 환경에서 경로 후보 탐색 | `RouteSearchBackend::search`, `searchMissionRoute` 유지. 알고리즘 교체 경계에 FM2/A* 이름을 고정하지 않음 |
| `compute` | 계산 결과 생성 | 계획 계산 경계에서는 새 후보를 실행 저장소에 적용하지 않음. 캐시·작업 객체·진단·preview까지 불변이라는 뜻은 아님 |
| `validate` | 명시된 입력·구조 규약 검사 | 범위를 이름에 한정. 계수·단계 검사를 전체 비행 안전 평가처럼 부르지 않음. 기존 enum/진단 반환 유지 |
| `evaluate` | 궤적의 측정값·평가 결과 생성 | `evaluateFlight` 유지. 평가 수행과 실행 허가는 별도 |
| `make` / `format` | 값·메시지 구성 / 표시 문자열 생성 | `makeTrajectoryMessage`, `formatFlightEvaluation` 유지. ROS 전송을 포함시키지 않음 |
| `publish` | ROS 메시지 발행 | 후보 계산·실행 저장소 채택과 구별. 발행에 필요한 FSM bookkeeping은 계약에 명시 |
| `apply` | 계산 결과에 기록된 상태 효과 반영 | 성공 채택뿐 아니라 실패 무효화도 포함하므로 `applyPlanningComputation`, store `apply` 유지 |
| `adopt` / `revoke` | 실행 궤적 채택 / 현재 실행 궤적 철회 | store의 제한된 상태 전이에 사용. `revoke`를 진행 중 계산의 `cancel`과 혼용하지 않음 |
| `record` | 결과 객체에 제품·나중에 적용할 효과 기록 | 기존 `recordProduct`와 새 `recordExecutionInvalidation`을 같은 어휘로 설명. 즉시 실행 철회나 ROS 발행이 아님 |
| `set` / `clear` / `cancel` | 값 설정 / 명시 대상 비우기 / 진행 작업 취소 | `setTerrain`만으로 실행 재검증·ABORT까지 약속하지 않음. `clearTerrain`의 SDF·장애물 제거 부수효과는 문서에 명시 |

`prepareRouteProfile`, `stitchRouteFlight`, `prependFlightPrefix`, `roundRouteCorners` 같은 정확한 도메인 동사는 유지한다. 모든 계산 함수를 `computeX`로 다시 부르지 않는다.

공개 plan/compute의 상태 효과 차이와 기존 private 모드 함수의 이름 계열을 구별한다. `SegmentChainPlanner`의 `planImpl`, `planInto`, `planMultiWaypoint`, `planBaselineMission`, `planTransitionMission`, `planRouteFallback`는 실행 채택 없는 계산을 조정한다. 이번에는 같은 계열의 `Parallel` 오칭만 제거하고 나머지 형제 전체를 개명하지 않는다. backend의 `plan`과 store의 `apply`가 실제 적용 지점이며, 최종 ROS 궤적 발행은 FSM 책임이다.

조회 함수는 `storedVoxelCount()`처럼 명사형을 허용한다. `get`을 모든 접근자에 추가하거나 제거하지 않는다. `controlAt(time_s, state)`는 해당 시각·상태에서 control을 계산하는 질의이며 저장 필드를 반환하는 가벼운 getter라고 보장하지 않는다.

## 5. 자료형·bool·단위 규칙

### 자료형

| 역할 | 사용할 어휘 | 현재 결정 |
|---|---|---|
| 작업 입력·출력 묶음 | `Request` / `Result` | `MissionPlanRequest/Result`, `OptimizerSolveRequest/Result` 유지 |
| 설정값 | `Settings`, 기존 작은 구성요소의 `Config` | `PlanSettings`, `RouteSliceConfig` 유지. 설정 지속성은 문서에서 정의 |
| 외부 환경·모델 접근 | `Environment`, `Model`, `Backend` | 기존 세 경계 유지. 추상 타입에 일괄 `I` 접두사를 붙이지 않음 |
| 상태·진단·평가 상세 | `Status`, `Reason`, `Report`, `Verdict` | 상태 코드와 실제 궤적·평가 수치를 구별 |
| 소유·조회 형태 | `Snapshot`, `View` | 실제 복사/참조 계약이 성립할 때만 사용. 현재 입력을 이름만 Snapshot으로 바꾸지 않음 |

기존 `PlanResult`는 전체 궤적이 아니라 outcome/reason/detail을 묶는다. 신규 타입이었다면 `PlanStatus`가 역할을 잘 드러내겠지만, 이번에는 기존 이름을 호환 예외로 설명하고 **동일 타입의 별칭을 하나 더 추가하지 않는다**. `result.status.outcome`이라는 현재 접근은 그대로다.

`Request`라는 이름만으로 모든 필드가 일회성이라고 가정하면 안 된다. 현재 `length_per_piece`·`obstacle_clearance` 양수 override의 지속 규약은 별도로 유지한다. `Result`의 값 소유도 이름이 아닌 각 자료형의 실제 정의와 주석으로 보장한다.

### bool

- 존재 여부는 `hasTrajectory()`, 정해진 성질은 `isValid()` 같은 질문형을 사용한다. 진단 enum을 반환하는 기존 `validate…`를 bool 이름으로 바꾸지 않는다.
- `hasTrajectory()`는 SUCCESS/DEGRADED 궤적 존재 계약이다. 모든 평가 통과나 실행 허가로 확대 해석하지 않는다.
- `FlightVerdict`의 `evaluated`와 `clean`은 유지하고 미평가 기본값을 설명한다. 이번에는 `isClean()`이라는 다른 해석의 중복 접근자를 추가하지 않는다.
- 소비자가 평가된 clean 결과를 원하면 `evaluated && clean`을 명시한다. 현재 coordinator의 `evaluated && !unflyable()`를 이 식으로 대체하지 않는다.
- `isSafe`, `canExecute`, `passesAllChecks`처럼 현재 평가가 보장하지 않는 이름은 도입하지 않는다.

### 단위·시간

새 공개 수치 필드는 **물리적 의미 + 단위**를 쓴다. `_s`는 초, `_u`는 기존 planner 길이, `_u_s`는 u/s, `_m`는 m, `_rad`는 rad처럼 일관되게 사용한다. 속도 크기는 `speed`, 벡터는 `velocity`로 구별한다. 기존 필드를 전부 개명하거나 SI로 변환하는 작업은 아니다.

이는 신규·명시적으로 이전하는 필드의 규칙이다. 기존 `vel_u`·`acc_u`는 각각 u/s·u/s²이며 `_u`가 planner 단위계를 가리키는 호환 표기다. 단위 없는 `head_vel`·`head_acc`도 기존 구조체 주석의 단위를 따른다. `_u_s`가 이미 전체 저장소의 관례라는 주장은 하지 않는다. 이번 공개 필드 이전은 아래 세 계층에 한정하며, 기존 필드를 대량 변환하지 않는다.

- 경로 매개변수 `s`와 초 접미사 `_s`가 충돌하는 `route_start_s`는 `route_start_arc_u`를 권한다.
- 궤적 시작 이후 시각은 `time_from_start_s`로 표현한다. 실제 timestamp와 구별하고, clock 기준이 필요한 timestamp는 주석으로 명시한다.
- scalar `max_vel` 요청값은 `nominal_speed_u_s`를 권한다. seed 시간 배분 및 미지정 도착 속도의 기준이라는 두 용도를 설명한다. 실제 최적화의 최대 속도 제약과 구별한다.
- `seed_speed_u_s`는 도착 경계 역할을 빠뜨리고, `cruise_speed_u_s`는 실제 전 구간 순항 속도 목표처럼 읽힐 수 있다. nominal은 기준값이라는 뜻으로 사용하며 용도를 주석에 고정한다.
- 단위 접미사는 좌표계까지 설명하지 않는다. planner 축, absolute z/AGL, grid 감소 방향, SI 스케일 변환은 헤더 계약에 별도로 남긴다.

`nominal_speed_u_s`의 설명은 direct/slice/optimizer **요청 경계**의 역할이다. 같은 manager 값은 chain 경계 속도를 작성하고 worker별 속도 제한을 조화시킬 때 기본 기준으로도 쓰인다(`segment_chain_planner.cpp:2222–2241,2782–2786`). 따라서 manager 값 전체가 어떤 제한에도 관여하지 않는다고 설명하지 않는다. `PathManager::maxVel()`과 `visualization::RiskSettings::max_velocity`는 이번 이전에서 제외하고 현 역할을 문서화한다. 후자는 표시용 경로 길이 추정에도 사용된다(`planning_visualization_data.cpp:497`).

`route`, `leg`, `slice`, `segment`, `piece`, `phase`는 서로 다른 개념으로 유지한다. 각각 경로 기하, 경유점 사이 정책 구간, 경로에서 잘라낸 부분, 계획 구간, 다항식 조각, 비행 단계다. 이름을 통일하려고 모두 segment로 바꾸지 않는다.

## 6. 구체적인 이름 변경안

### 이번에 적용한 내부 이름

| 변경 전 | 적용한 이름 | 근거 |
|---|---|---|
| `ReplanFSM::planFromGlobalTraj(int trial_times)` | `publishExecutionTrajectory()` | 실제 재계산·retry가 없고 채택 결과를 발행. 기존 `executionTrajectory`·`publishExecutionAbort` 어휘와 연결. 미사용 인자 제거 |
| `SegmentChainPlanner::invalidateStoredTrajectory(computed)` | `recordExecutionInvalidation(computed)` | `computed.invalidate_execution=true`만 기록. 기존 `recordProduct`와 기록 동사를 맞춤. 멤버를 다른 클래스로 옮기지는 않음 |
| `SegmentChainPlanner::planRouteParallel` | `planFromRoute` | 같은 클래스의 private `plan*` 모드 계열 유지. 경로 기반 구성이고 직렬 실행도 지원하므로 `Parallel` 제거 |
| `RosPlanningBackend::planDirect` | `computeDirect` | backend의 계산 하위 단계. 전체 `plan()`의 채택 동작과 구별 |

앞선 감사와 단독 설계 후보를 Claude와 재검토해 위 이름으로 적용했다. 내부 채택 절차어, 요청 송신으로 읽힐 수 있는 동사, private 형제 사이의 불필요한 어휘 차이를 줄였다. 실제 제어 흐름·반환 정책은 유지했다.

`publishExecutionTrajectory`의 bool은 유효한 실행 슬롯을 찾아 발행 경로를 수행했다는 뜻이며 DDS 전달 성공 확인이 아니다. 선택적으로 reference 궤적도 발행하고 FSM bookkeeping을 갱신한다. 옛 함수명인 로그 태그와 직접 참조 주석은 소비자를 확인한 뒤 새 이름에 맞춘다. 파싱되는 로그·검증 스키마의 식별자는 별도 호환 대상으로 유지한다.

### 공개 함수: 호환 이름을 유지하면서 이전할 후보

| 현재 | 설계 이름 | 범위 |
|---|---|---|
| `PathManager::invalidateStoredTrajectoryForEnvChange` | `revokeExecutionForUnevaluatedPolicy` | `POLICY_UNEVALUATED`라는 고정 사유로 철회. zone 변경 감지는 호출자 책임. 이번에는 임의 reason을 받는 새 계약으로 일반화하지 않음 |
| `validateFlightPhases` | `validateTrajectoryAndPhases` | 궤적 계수·시간과 phase partition을 검사. 설정/환경/비행 가능성 전체 검사는 아님 |
| `PathSearcher::astarSearchAndGetSimplePath` | `searchAndSimplify` | A*/FM2 선택·폴백, 설정에 따른 shortcut, 지형 여유 수리, 최종 점유 검사와 빈 결과 조건까지 선언에 명시. 클래스가 Path 문맥을 제공 |
| `SDFManager::numAllocatedBlocks` | `storedVoxelCount` | 저장된 static distance sample 수. 밀집 격자는 `nx*ny*nz`, 배열을 만들지 않는 all-free 표현은 0. 동적 패치 수·메모리 바이트 수·항상 논리 격자 크기를 반환하는 API가 아님 |
| `SimulationSession::controller(t,state)` | `controlAt(t,state)` | controller 전략 객체 대신 control 값을 반환. 내부 `realize` 단계까지 이름에 붙이지 않음. 낮은 우선순위 |

`nextRouteSearchIdentity`의 공통 계획 ID 역할은 향후 내부 이동 시 `nextPlanningIdentity`가 적절하지만 이번 우선 묶음에서 제외한다. 다른 private 이름도 일괄 통일하지 않는다.

### 공개 필드: 별도 소스 호환 결정

| 현재 | 설계 이름·처리 |
|---|---|
| `EntryCandidate/TransitionResult::route_start_s` | `route_start_arc_u` |
| direct/slice/optimizer 요청 설정의 scalar `max_vel` | 세 계층을 함께 `nominal_speed_u_s`로 이전. 기존 YAML `manager/max_vel`, `optimization/max_vel` 및 optimizer 내부 제약은 유지 |
| `FlightPhaseMetrics::zone_hard_*` 상세 접촉 필드 | 우선 HARD_AVOID·authored/standoff 범위를 주석에 명시. 추후 개명한다면 `hard_avoid_contact_*` 계열; 각 필드가 접촉 시각인지 전체 최근접 지표인지 따로 확인 |
| `poly_traj.hpp`와 RViz wrapper의 인자 `t_abs` | `time_from_start_s`. 함수 호출 형식·ABI 변경 없음 |

공개 필드의 낡은 이름과 새 이름을 독립된 데이터 멤버 두 개로 보관하지 않는다. 헤더 밖 소비자를 모르는 상태에서 “검색된 호출자가 적으므로 호환 문제가 없다”고 판단하지 않는다.

소비자 조사에는 직렬화·검증 하네스도 포함한다. 다만 C++ 멤버 이름 변경과 JSON 키 변경은 별개다. 기존 R2 하네스의 `zone_hard_n` 등은 명시적인 `FlightVerdict` 키이며, 위 `FlightPhaseMetrics` 상세 필드 후보와도 구별한다. 이름 정리를 위해 검증용 과거 산출물이나 wire key를 일괄 치환하지 않는다.

## 7. 실제 호출이 읽히는 방식

핵심 외부 사용법은 현재 코드 그대로 유지된다. 다음은 이미 존재하는 API의 사용 형태이며 초기화·오류 표시는 생략했다.

```cpp
MissionPlanResult result = planner.plan(request);
if (!result.hasTrajectory()) {
  // Inspect result.status.outcome, reason and detail.
  return;
}

const auto &trajectory = result.trajectory;  // Owned by result.
const auto &evaluation = result.evaluation;
if (evaluation.evaluated && evaluation.clean) {
  // This evaluation ran and reported clean; not an execution authorization.
}
```

외부에서는 `planner.plan`, 최적화만 필요하면 `optimizer.solve`, 독립 평가가 필요하면 `evaluateFlight`라는 기존 입구를 사용한다. 내부에서는 `compute` → `apply` → `publish`라는 책임 차이를 이름으로 읽을 수 있게 한다. 이것은 각 이름의 역할 설명이며 모든 모드에 동일한 호출 순서를 새로 강제하는 설계가 아니다.

새 공개 함수 선언의 설명은 다음 순서로 짧게 쓴다: 무엇을 계산/변경하는지, 입력 좌표·단위, 결과와 빌린 입력의 수명, 상태 효과, 실패/부분 결과, 직렬 사용 등 동시성 규약. 함수명에 이 내용을 전부 이어 붙이지 않는다.

## 8. 구현 묶음과 완료 기준

1. **규약 설명:** 전날 감사에서 확인한 현행 계약·주석을 고친다. 환경 setter 동작, 고도 기본값, 실패 상태 enum을 이 과정에서 변경하지 않는다.
2. **내부 이름 네 개:** 선언·정의·호출자와 직접 설명을 함께 변경한다. 미사용 `trial_times`만 제거한다. 일반 로그 매크로 등 공개 헤더 노출 정리는 수치 본체를 보존하는 별도 변경으로 묶는다.
3. **공개 함수 이전:** 필요한 새 이름을 정식 입구로 제공하고 기존 이름은 동일 결과의 forwarding 함수로 유지한다. 기존 바이너리 지원이 필요하면 과거 out-of-line symbol도 보존한다. inline wrapper만으로 ABI 보존을 주장하지 않는다. 먼저 문서에서 새 이름을 권장하고, 내부·예제 소비자가 이전된 뒤 `[[deprecated("use …")]]` 적용을 검토한다. 경고를 오류로 처리하는 소비자도 고려한다. 제거는 별도로 공지한 breaking release에서만 결정하며, 현재 제거 날짜나 릴리스는 약속하지 않는다.
4. **공개 필드 이전:** 소비자 범위와 버전 경계를 정한 뒤 진행한다. 상수·enum의 숫자/우선순위, ROS 파라미터 키, 토픽명, 메시지 필드는 별도 프로토콜로 유지한다.

구현할 때의 완료 기준은 **이름을 보고 역할을 잘못 추측하는 지점이 줄고, 기존 계산·상태 전이는 같음**이다. 내부 rename은 컨테이너 빌드와 관련 회귀 시험, 공개 API 이전은 이전/신규 이름의 설치 소비자 확인이 필요하다. 계획·실행 경로를 수정한 묶음은 기존 R2 궤적 계수·시간, 병렬/3-pass, ABORT/복구 비교로 동작을 확인한다. 이름 변경을 위해 MINCO/L-BFGS 내부를 다시 나누거나 새 추상 계층을 추가하지 않는다.

최초 설계 단계에서는 빌드·실행 시험을 수행하지 않았다. 이후 사용자가 승인한
계약 설명과 private 네 함수만 구현하고 빌드·회귀·R2·실제 RViz ABORT/복구를 검증했다.
전역 로그 매크로 등 공개 헤더 노출 정리, 공개 함수·필드 이전 및 계약 확장은 적용하지 않았다.

## 9. Claude 공동 검토와 반론 처리

[Claude 독립 검토 원문](api_naming_design_claude_review_2026_10_13.md)을 별도로 보존한다. 검토자는 후보의 실제 구현을 대조하고 Qt·LLVM 원문을 재확인했다. Core Guidelines NL 절은 Claude의 이번 재조회에서 확인하지 못했으며, Codex 측이 앞선 웹 조사에서 확인한 근거와 구별한다. 두 세션이 모든 외부 자료를 각각 검증했다고 주장하지 않는다.

| 쟁점 | 반영한 판단 |
|---|---|
| private 모드의 `plan`/`compute` 일관성 | Claude 의견 수용. `computeRoutePlan` 대신 `planFromRoute`, 기존 private 계열 예외 명시 |
| flag만 기록하는 함수 | Claude 의견 수용. `markExecutionForInvalidation` 대신 `recordExecutionInvalidation` |
| 실행 철회 이름의 zone 트리거 | Claude 의견 수용. 호출자보다 고정 처리 사유를 드러내는 `revokeExecutionForUnevaluatedPolicy` |
| SDF 개수가 항상 논리 전체 격자 크기인지 | Codex 반박을 Claude가 재확인하고 수용. `buildEmpty` 및 점유 없는 입력에서 `has_data=true`, shape는 양수지만 배열은 비어 있음(`sdf_manager.cpp:179–203,242–259`). 단순 `voxelCount` 대신 `storedVoxelCount`에 합의 |
| `_u_s`가 기존 관례인지 | 새 규칙임을 인정하고 legacy `vel_u`·`acc_u` 예외 명시. 요청 세 계층과 manager/표시 API의 이전 범위 분리 |
| 요청 속도와 chain 제한 기준 | Claude 의견 수용. 요청의 seed/기본 도착 역할과 상위 chain의 기준 역할을 함께 문서화 |
| 로그와 JSON 키 | 무조건 유지/변경 모두 피함. 함수명 디버그 태그·소스 주석은 소비자 확인 후 변경; wire/검증 스키마와 과거 산출물은 별도로 보존 |
| public forwarding 수명 | 문서 권장 → 소비자 이전 → deprecation 검토 → 명시적 breaking release에서 제거 결정 |

Claude 원문의 1차 제안과 반론은 검토 이력이며, 최종 확인은 원문 §6과 이 문서의 변경안에 따른다. 원문의 `RiskProfileSettings.max_velocity` 표기는 실제 타입명인 `visualization::RiskSettings::max_velocity`로 이 문서에서 바로잡았다.

위 원문은 설계 단계의 공동 검토 기록이다. 이후 구현 단계에서는 이 문서의 private
네 함수명과 계약 설명만 정리했다. 수치 계산·ROS 동작은 유지했으며,
[구현 후 Claude 검토](../validation/api_naming_2026_10_13/claude_final_review.md)를 별도로 보존한다.
