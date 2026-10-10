# 계획 시각화 데이터와 ROS 발행 연결

계획 계산과 시각화 메시지 준비를 분리했다. `PathManager`와 `SegmentChainPlanner`의 기존 호출 시점은 유지하고, 필요한 궤적·경로·위험장 조회와 설정만 메시지 준비 함수에 전달한다. 함수는 점 배열, 색, 격자 등을 소유하는 ROS 메시지를 반환한다. `RosPlanningVisualization`은 기존 토픽과 QoS로 메시지를 발행한다. 이번 단계는 ROS 메시지 형식이나 RViz를 제거하지 않는다.

## 전체 계획에서 시각화 연결 끄기

```cpp
path_manager::PlanSettings settings;
settings.attach_visualization = false;
// 기존 설정·환경·RosPlanningBackend로 MissionPlanner를 구성한다.
// planner.plan(request)의 상태·궤적·평가 및 실행 적용 규약은 동일하다.
```

기본값은 `true`다. `false`이면 `PathManager`가 담당하는 `/viz/opt_trajectory`, `/viz/global_trajectory`, `/viz/front_end_path`, `/viz/chain_segments`, `/viz/traj_risk`, `/viz/risk_profile`, `/viz/risk_heatmap`, `/viz/risk_field`, `/viz/dynamic_obstacles`, `/debug/pipeline`, `/planning/front_end_path` 발행 연결과 메시지 준비를 생략한다. ROS 백엔드, 지형/SDF 처리, 실행 메시지·ABORT 처리는 계속 동작한다. 위험장의 LOS 마스크 구축과 장애물 방향 난수 소비도 유지된다. 이 옵션은 초기 구성에 적용하며 실행 중 재연결 API는 제공하지 않는다.

`PathManager`를 직접 구성하는 호환 호출은 `PathManager(node, false)`를 사용할 수 있다. 기본 단일 인자 호출은 기존처럼 시각화를 연결한다.

## 메시지만 준비하기

공개 헤더는 `path_manager/planning_visualization_data.h`, 네임스페이스는 `path_manager::visualization`이다. 설치된 CMake 타깃 `path_manager::path_manager_lib`에 연결한다. `examples/planning_visualization_example.cpp`는 ROS 노드 없이 경로와 궤적 튜브 메시지를 만들고 입력이 소멸한 뒤 결과를 사용한다.

```bash
ros2 run path_manager planning_visualization_example
# route_points=2 tube_points=21 frame=map
```

| 입력 | 준비 함수 | 소유하는 결과 |
|---|---|---|
| 경로점 + 시각 | `makeFrontEndPath` | `nav_msgs::msg::Path` |
| 경로점 또는 궤적 + 표시 속성 + 시계 | `makeTube`, `makeTrajectoryTube` | 선택적 `Marker` |
| 구간 궤적 + 접합점 + 선택적 종말 구간 | `makeChain` | `MarkerArray` |
| 후처리 경로 + 내부 최적화점 | `makePipeline` | 메시지 + 기존 진단문 |
| 궤적 + 위험장 조회 | `makeTrajectoryRisk` | 메시지 + 기존 진단문 |
| 궤적 + 위험장/지형 조회 + 설정 | `makeRiskProfile` | 기존 v4 프로파일 + 진단문 |
| 위험장/지형 조회 + 설정 | `makeRiskHeatmap`, `makeRiskField` | 격자 또는 마커 배열 |
| 장애물 위치·크기·종류·확정 yaw·메시 속성 | `makeDynamicObstacles` | 표시 및 패널용 기하 마커 배열 |
| 없음 | `makeTrajectoryClear` | 기존 `world` 프레임의 DELETEALL |

`RiskView`는 순서가 유지된 위험 구역 목록과 필요한 조회 콜백을 한 번의 동기 호출 동안 빌린다. `TerrainView`는 실제 DEM을 복사하지 않는다. 위험도·가시성·그림자 높이는 계획에 사용하는 정식 계산을 연결해야 한다. `PathManager` 전체나 별도 위험도 알고리즘을 전달하지 않는다. 결과에는 입력 참조나 콜백이 저장되지 않는다.

`RiskSettings`의 0 초기화는 사용할 수 있는 기본 설정 묶음이 아니다. 어댑터가 기존 설정 원본에서 필요한 필드를 채운다. 비어 있지 않은 위험장 출력에는 양수 유한 `vertical_ratio`, 유한 `softness`·`threshold`가 필요하다. 프로파일은 `max_velocity`·`obstacle_clearance`, 히트맵은 양수 `heatmap_max_dim` 및 유한 히트맵 설정, 위험장 마커는 유한 `step`·`slice_offset`·`agl`·`volume_alpha`·`heatmap_offset`을 사용한다. `step <= 0`의 자동 간격 선택은 유지된다. 사용되는 콜백과 유효 지형의 양수 유한 해상도도 필요하다. 구체적인 함수별 콜백 조건은 공개 헤더에 적혀 있다.

표시용 경로 굵기와 구형 장애물 메시 높이는 기존의 유한값을 유지하며, `0`으로 표시를 숨기는 설정도 허용한다.

궤적 조각의 시간은 양수 유한값, 계수와 총 시간은 유한값이어야 한다. 구조적으로 잘못된 입력, 누락된 필수 콜백 또는 표현할 수 없는 샘플 수는 `std::invalid_argument`를 던진다. 콜백 예외는 그대로 전달하며 부분 메시지를 반환하지 않는다. 빈 궤적·빈 위험장에는 기존 지우기 메시지/튜브 미발행 규약을 유지한다. 위험도·가시성 콜백은 유한 `[0,1]` 값이어야 하고, 그림자 높이의 비유한값과 지형의 NaN/음의 무한대는 기존 미가용 처리 의미를 유지한다.

## 발행하기

`path_manager/ros_planning_visualization.h`의 구체 클래스는 준비된 메시지만 받는다. 예를 들어 별도 ROS 어댑터가 소유한 연결에서 `connection.tube(TubeChannel::Optimized, *tube)`를 호출한다. 기본 계획 실행은 내부 연결을 이미 구성하므로 같은 토픽에 추가 연결을 만들 필요가 없다.

기존의 transient-local/reliable/depth-1 QoS, namespace/ID, 색·좌표·샘플 간격과 삭제 범위를 유지한다. `clearTrajectory`는 최적/기준/위험 궤적과 프로파일만 지운다. 전단 경로·구간 표시까지 임의로 함께 지우지 않는다. `makeRiskHeatmap`의 비우기는 1×1 NaN 격자다. 위험장 갱신은 히트맵을 먼저 발행하며, 구간·위험장·장애물은 DELETEALL로 시작하는 완전한 배열을 발행한다. 구독자가 없어도 마지막 표시 상태를 보관한다.
