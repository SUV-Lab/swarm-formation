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
