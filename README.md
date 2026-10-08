# Path Manager System

ROS 2 기반 드론/로버 경로 계획 및 편대 제어 시스템

### 시뮬레이션 실행

```bash
ros2 launch path_manager rviz_path_manager.launch.py
```

> **참고**:
> - 시작점·목표점은 RViz의 MissionConfig 패널에서 미션 yaml을 로드해 설정합니다
> - 장애물·위험지대는 RViz ObstacleScenario 패널에서 yaml 로 로드합니다
>   (/mission/obstacles · /mission/risk_zones — scenario 런치 파라미터는 폐기)

### 주요 파라미터

| 파라미터 | 설명 | 기본값 |
|---------|------|--------|
| `drone_id` | 실행할 드론 ID | 1 |
| `world` | 지형 맵 이름 (비우면 optimizer_params 기본값) | (없음) |
| `record_bag` | 궤적 토픽 rosbag 기록 | false |
| `disable_file_logging` | 파일 로깅 비활성화 (콘솔만) | false |

### 지형 차폐 위험영역

위험원의 `center.z`를 전파원 고도로 사용합니다. 각 방위각에서 DEM을
가까운 셀부터 훑으며 최대 고도각을 누적하고, 그 고도각이 만드는
`shadow_ceiling`보다 낮은 지점의 위험도를 제거합니다. 따라서 산 뒤에서는
위험도가 사라지지만, 같은 `(x, y)`에서도 항공기가 horizon 위로 상승하면
위험도가 다시 적용됩니다. FM2/A*, shortcut, MINCO가 동일한 precomputed
visibility field를 사용합니다.

관련 파라미터는 `manager/risk_terrain_mask_enable`,
`manager/risk_mask_radial_step`, `manager/risk_mask_softness`이며
`optimizer_params.yaml`에서 조정할 수 있습니다. RViz의 `/viz/risk_field`는
기본적으로 각 위험원 `center.z`에서의 유효 위험영역 단면만 셀로 표시합니다.
표시 고도는 `manager/risk_mask_viz_slice_offset`으로 이동할 수 있습니다.

### C++ 라이브러리 사용

현재 API의 초기화·단위·소유권·교체 구현 계약과 실행 예제는
[`계획 라이브러리 사용법`](docs/design/planning_library_usage_ko.md)을 기준으로 합니다.

전체 미션은 `path_manager::MissionPlanner::plan(request)`로 계산합니다.
`PlanSettings`, `PlanningEnvironment`와 명시적으로 선택한 backend로 planner를 만들고,
`MissionPlanRequest`를 전달하면 상태·실패 사유, 소유하는 최종/참고 궤적과
평가·phase 결과를 받습니다. FSM이나 미션 토픽 없이 동기 호출할 수 있습니다.

일반 입력과 `MissionPlanner` 헤더/구현은 ROS 타입에 의존하지 않습니다.
현재 실제 계산 구현은 `RosPlanningBackend`이며 기존 ROS 설정·시각화·실행 동작을
유지합니다. ROS를 사용하는 구현을 명시적으로 선택하고, caller가 ROS context를
초기화/종료합니다. 새로운 node를 직접 만들 필요는 없습니다.

```cpp
path_manager::PlanSettings settings;
settings.parameters["chain/enable"] = true;
settings.parameters["chain/segments"] = int64_t{3};
path_manager::PlanningEnvironment environment;
environment.terrain = terrain_grid;  // 소유하는 column-major float 데이터
path_manager::RosPlanningOptions runtime;
runtime.node_options.arguments({"--ros-args", "--params-file", config_path});
path_manager::MissionPlanner planner(
    std::move(settings), std::move(environment),
    path_manager::makeRosPlanningBackend(std::move(runtime)));
auto result = planner.plan(request);
```

위 코드는 `mission_planner.h`와 명시적 ROS adapter 헤더인 `ros_planning_backend.h`를
사용합니다. `PlanSettings.parameters`는 기존 파라미터 이름과 정확한 scalar/array 타입을
유지하는 초기 override이며, 숫자 기본값을 별도로 복제하지 않습니다. 명시한 값은
`NodeOptions`/YAML override보다 우선하고 기존 소비자가 필요할 때 선언합니다.
기체와 탐색 구현은 `vehicle_model`, `route_search_backend`로 주입하거나 미션 사이에
각 setter로 교체합니다. 기체 변경이 새로운 VTOL transition 구현을 자동 제공하지는 않습니다.

기존 node에 연결하려면 `makeRosPlanningBackend(node)`를 사용하고
`PlanSettings.parameters`는 비워 둡니다. node가 계속 live 설정의 권한을 갖습니다.
ROS 실행 연결에 필요한 PM은 해당 backend의 `executionManager()`로 접근합니다.
기존 `MissionPlanner(node)` / `legacyManager()` 사용은 이 명시적 adapter 형태로 옮깁니다.

`PlanningEnvironment`는 초기 patch입니다. 없는 항목은 기존 설정을 유지하고,
명시한 빈 위험영역/장애물 목록은 기존 clear API의 범위대로 지웁니다.
`TerrainGrid`는 중심·길이·해상도·행/열 수와 원래 float 배열을 소유하며, NaN과
열 우선 순서, 현재 좌표계·단위를 유지합니다. `setTerrain(grid, &error)`는 잘못된
표현을 거부하고 이전 지형을 유지하며, `clearTerrain()`은 명시적 world-none 갱신입니다.
ROS `GridMap` 연결용 변환은 `ros_terrain_grid.h`에 있고 FSM의 기존 raw callback은 유지합니다.

```cmake
find_package(path_manager REQUIRED)
target_link_libraries(my_planner PRIVATE path_manager::path_manager_lib)
```

사용 예시는 [`mission_planner_example.cpp`](src/path_manager/examples/mission_planner_example.cpp)입니다.
기존 환경을 source한 뒤 아래처럼 실행할 수 있습니다.

```bash
ros2 run path_manager mission_planner_example \
  "$(ros2 pkg prefix path_manager)/share/path_manager/config/optimizer_params.yaml"
```

이미 준비된 경로만 최적화하려면 `ego_planner::PolyTrajOptimizer::solve(request)`를
사용합니다. `setParam(node)`로 기존 수치 설정을 읽고, `setRuntimeSettings(settings)`와
`setEnvironment(environment)`로 실행 설정과 지형·거리장·위험영역 조회를 연결합니다.
`OptimizerSolveRequest`는 경로·경계조건·고도 대역을 소유하며, 결과는 성공 시 seed와
최종 궤적·평가값을, 실패 시 상태와 진단 경로를 반환합니다.
MINCO·L-BFGS 본체는 이 경계 안에서 통째로 유지하며, 내부 알고리즘을 다시 분해하지 않습니다.

```cmake
find_package(path_optimizer REQUIRED)
target_link_libraries(my_optimizer PRIVATE path_optimizer::path_optimizer_lib)
```

두 진입점 모두 동일 인스턴스의 설정 변경·환경 변경·계획 호출은 순차 실행해야 합니다.
결과 데이터는 planner/optimizer 파괴 후에도 유효하지만, 연결한 거리장과 callback이
참조하는 환경은 연결이 유지되는 동안 살아 있고 각 계산 중에는 안정적이어야 합니다.
좌표·단위·AGL 규약은 기존
MMP와 같으며 결과 생성 후 환경이 바뀌었을 때의 실행 유효성은 호출자가 관리합니다.

계산 함수별 경계와 검증 기록은
[`incremental_modularization_ko.md`](docs/design/incremental_modularization_ko.md)에 있습니다.
