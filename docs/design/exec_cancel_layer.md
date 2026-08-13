# 실행 취소 계층 — 결정이 필요한 자리

## 요약

비행 중 환경이 바뀌어 **남은 궤적이 막혔을 때**, 이 노드는 그것을 감지하고 자기
슬롯을 무효화한다. 그러나 **이미 발행된 궤적을 회수하지 못한다.** 외부 소비자는
`/planning/trajectory`로 받은 이전 궤적을 계속 실행할 수 있다.

지금 있는 것과 없는 것:

| 계층 | 상태 |
|---|---|
| 재평가 (남은 구간이 아직 통과 가능한가) | **있음** — `revalidateStoredTrajectory`, 회귀 `envchange` |
| 트리거 (장애물 토픽·지연 설치·새 DEM·구역 갱신) | **있음** |
| 이 노드가 그 궤적을 더는 비행 가능으로 취급하지 않음 | **있음** — `duration`/`start_time` 0 |
| **소비자에게 취소를 선언** | **있음** — `/planning/execution_control`, `exec_abort_test` |
| 소비자가 실제로 그것을 구현했는지 | **이 저장소 밖** |

초판에서 "주차한다"고 쓴 것은 이 노드 안에서만 참이었다. 지금은 ABORT가 나간다.

## 왜 기동을 만들지 않는가

취소를 발행하려면 **무엇을 발행할지**를 정해야 하고, 그건 기체 수준 결정이다.

기존 `EmergencyStop`은 쓸 수 없다. `headState << stop_pos, ZERO, ZERO` — 속도·가속도
0의 정지 궤적이다 [읽음: `path_manager.cpp:2158`]. 이 스택의 순항 최저 속도는
122 m/s이고, 정지 궤적은 **날 수 없는 명령**이다. 이 저장소가 반복해서 잡아온
"unflyable product" 부류 그 자체다. 배선하면 감지 못 한 것보다 나빠질 수 있다.

## 결정: 5번 — 명시적 abort (2026-08-13)

플래너에는 기체별 비상 기동을 안전하게 정할 근거가 없다. 정지·선회·상승·대체 목표를
임의로 만들면 다시 실행 불가능한 궤적을 낳을 위험이 있다. 그래서 플래너는 **기존 궤적이
무효임을 선언**하고, 이후 동작은 실행 계층의 비상 정책에 맡긴다.

### 와이어

- `PolyTraj.trajectory_id` (`uint64`) — 노드별 단조 증가. `LocalTrajData::traj_id`가
  이미 있었고 와이어에만 안 나가고 있었다. 2026-07 감사에서 "쓰이지만 아무도 안 읽음"으로
  제거됐던 필드라, **이번엔 읽는 쪽이 생겼다는 사실을 메시지에 적어 두었다.**
  `start_time`은 식별자가 될 수 없다 — 시계값이라 재생·보정에서 움직인다
- `TrajectoryExecutionControl.msg` — `ACTION_ABORT`, `trajectory_id`, `reason`
  (`OBSTACLE_BLOCKED`/`TERRAIN_BLOCKED`/`POLICY_UNEVALUATED`), 진단용 `detected_point`·
  `detected_trajectory_time`·`environment_generation`·`detail`
- 토픽 `/planning/execution_control`, QoS는 궤적과 동일한 **RELIABLE + TRANSIENT_LOCAL**

### 순서

```
남은 구간 재검사 → BLOCKED/UNEVALUATED
  → 해당 trajectory_id의 ABORT 발행      (슬롯을 비우기 전 — id를 아직 읽을 수 있을 때)
  → 로컬 슬롯 무효화
  → FSM → WAIT_EXTERNAL_RECOVERY          (아무것도 발행하지 않는 전용 상태)
```

**여기서 나가는 길**: 새 미션 명령이 `SEQUENTIAL_START`로 보낸다. 초판에는 주석만 그렇게
적혀 있었고 실제 전환표에는 `WAIT_POSITION`·`EXEC_TRAJ`만 있어서, abort 뒤 새 미션은
계획에 성공해도 발행되지 않는 **덫**이었다. 미션 명령은 조종자(또는 실행 계층)가 기체를
다시 통제한다는 선언이고, 이 노드는 odometry가 없어 그것을 스스로 관측할 수 없다.
전용 resume/ACK를 원하면 abort와 같은 제어 토픽에 얹고 이 분기가 그 핸들러가 된다.

`EMERGENCY_STOP`은 쓰지 않는다. 그 상태의 `callEmergencyStop`은 `headState << stop_pos,
ZERO, ZERO` — 정지 PVA이고 이 모델에서 실행 불가능하다.

### 감지와 발행의 분리

`PathManager`가 감지하고(SDF와 저장 궤적을 소유), `ReplanFSM`이 발행한다(퍼블리셔와
상태기계를 소유). 사이는 `setEnvChangeHook`이다. 그래서 **PathManager 내부 트리거**
(지연 장애물 flush)도 같은 배관에 도달한다.

### 생산 배관 회귀 — `fsm_scenarios_test`

실제 `ReplanFSM`을 띄우고 진짜 콜백으로 구동한다. 순서가 중요하고 임의가 아니다:
구역 집합은 **비어 있는 상태로 시작**하므로 첫 비어있지-않은 배치가 곧 변경이다.
궤적이 생기기 **전에** 넣어야 no-op 검사가 그 배치 때문에 오염되지 않는다.

1. 평탄 DEM 투입 → 2. 구역 집합 A 투입(아직 비행 없음, abort 없어야) →
3. 미션 → `/planning/trajectory` 발행 확인, id 기록 → 4. **같은 A 재발행 → abort 0건**
→ 5. **변경된 집합 → abort 정확히 1회**, `POLICY_UNEVALUATED`, 그 id 지목 →
6. **abort 뒤 새 미션 → 발행되고 id가 더 큼**(WAIT_EXTERNAL_RECOVERY 탈출) →
7. **지형을 비행 위로 올림 → abort 1회**, `TERRAIN_BLOCKED`, 현재 id 지목

`drone_id`는 반드시 0이어야 한다 — `SEQUENTIAL_START`가
`drone_id_ <= 0 || (>=1 && have_recv_pre_agent_)`로 막혀 있고 후자는 이 저장소에서
아무도 쓰지 않는다. 파라미터는 FSM·PathManager가 생성자에서 ~90개를 declare 하므로
테스트가 먼저 declare 하면 안 된다(전부 params 파일/NodeOptions로).

**이 테스트가 잡은 실제 결함**: `polylineClear`가 **미션이 지정한 끝점**까지 검사해서,
지형 0.0 위 z=0.15에 있는 드론이 자기 시작점 때문에(여유 0.15 < margin 0.60) geodesic이
통째로 폐기되고 **계획 자체가 실패**했다. 끝점 주변 margin 반경을 면제했다 — A*도 같은
이유로 점유된 시작 셀을 용인한다.

### 회귀

- `exec_abort_test` (신규, 실제 pub/sub): 발행된 궤적 실행 → 같은 id의 abort로 중단 →
  중복 abort 멱등 → abort 후 새 궤적은 실행 → **과거 abort가 새 id를 멈추지 않음** →
  늦게 붙은 소비자가 latched abort된 궤적을 실행하지 않음 → reason 구분
- `chain_experiment_test envchange` (생산자 측): 무해한 변경엔 **선언 없음**, 차단 시
  정확히 1회, **철회되는 궤적의 id로**, 올바른 reason과 지점
- 변이 2종 사망: 훅을 슬롯 정리 **뒤로** 옮기면 id가 0이 되어 죽고, 차단이 아닐 때도
  선언하게 하면 "선언 없음" 3건이 죽는다

### 소비자

**정정**: 초판에 "실제 팔로워 구현이 이 저장소에 없다"고 적었는데 **틀렸다.**
`mmp_vehicle_dynamics`의 `dynamics_sim_node`가 `/planning/trajectory`를 구독하는 실제
소비자다. 그것이 계약을 구현하도록 고쳤다:

- `/planning/execution_control` 구독(궤적과 같은 QoS)
- `current_traj_id_`를 들고 있다가, **그 id를 지목한 abort일 때만** 추종 중단
- 이미 철회된 id의 궤적이 도착하면 실행을 시작하지 않는다(늦은 접속 시 순서 보장이 없다)

플랫폼 실행기는 아니므로 **외부 팔로워와의 합의는 여전히 필요하다.** 다만 이제
저장소 안에 계약을 실제로 구현한 소비자가 하나 있다.

**상태기는 공유한다**: `mmp_vehicle_dynamics/trajectory_abort_state.hpp`. 시뮬레이터와
회귀가 같은 구현을 쓴다 — 전에는 각자 복사본을 들고 있어서 **테스트가 통과하면서
시뮬레이터가 다른 일을 할 수 있었다.** 규칙 네 가지(자기 id만 중단 / 과거 id 무시 /
abort 먼저·궤적 나중이면 시작 거부 / 새 id는 정상)는 토픽이 아니라 그 클래스를 직접
구동해 고정한다 — latched 두 토픽의 전달 순서를 테스트가 정할 수 없기 때문이다.

### 아직 없는 것

> 이 목록의 **차단 항목은 `../abort_layer_backlog.md`로 옮겨 관리한다.** 초기 플래너
> 마일스톤은 닫혔고, 아래는 그 뒤에 오는 실행 계층 작업이다.


- ACK 상태 발행은 넣지 않았다 (계약에 "가능하면")
- FSM 호출부가 넘기는 reason 인자 자체는 회귀가 없다. 매니저가 **호출자가 준 reason을
  그대로 싣는지**는 고정했지만(`envchange`), 지형 콜백이 `TERRAIN_BLOCKED`를 넘기는지는
  FSM 전체를 띄우는 테스트가 필요하다 — 변이를 걸면 살아남는다
- **시뮬레이터의 배선**은 여전히 미커버다. 상태기는 공유하므로 로직 결함은 잡히지만,
  시뮬레이터가 제어 토픽 구독을 지워도 테스트는 통과한다. 실제 노드를 루프에 넣어야 한다
- **ID는 플래너 노드 수명 동안만 고유하다.** 플래너만 재시작하면 다시 1부터 시작하고,
  계속 살아 있는 소비자의 철회 집합과 충돌한다 — 재시작 직후의 궤적 1이 이전 인스턴스의
  철회된 1로 거부될 수 있다. **외부 팔로워 계약 전에** `planner_instance_id + sequence`
  같은 재시작 내성 ID로 넓혀야 한다
- **`fsm_scenarios_test`는 실제 콜백을 호출하지만 토픽 구독은 우회한다.** 생산 로직
  회귀이지 완전한 배선 회귀는 아니다
- **`polylineClear`의 "장애물은 어디서도 면제 없음"** 절은 격리되지 않았다. 시작점
  장애물은 **검색 단계에서** 이미 거부되고(A*가 시작점을 빈 셀로 옮기려다 실패),
  완화 구간 밖 장애물은 어차피 걸리므로, 그 절만 지우는 변이가 죽지 않는다. 방어로
  남기되 검증되지 않았다는 사실은 그대로다
- **`legfail`의 "검색 실패는 epoch을 비운다"**는 이제 `fm2fail`이 덮는다. `legfail`은
  clearance 500으로 실패를 강제했는데, 끝점 면제가 그 반경으로 커져 경로 전체가
  면제된다 — 그래서 back-end 실패 쪽으로 되돌렸다. **leg 0 성공 + leg 1 실패의 prefix
  경우는 여전히 구성 불가**다 (`fm2fail`은 모든 leg가 실패한다)

## 구역 갱신에 대해

구역은 장애물 술어로 재검사하면 안 된다 — 같은 부피가 leg마다 HARD_AVOID이거나
SOFT_ENDPOINT 면제다. 올바른 해법은 **실행 중 궤적을 per-piece 정책으로 재감사**하는
것이고 아직 없다.

그때까지는 fail-closed로 둔다: 실행 중 구역 갱신이 오면 저장 궤적을 무효화한다.
"확인할 수 없었다"가 "아직 괜찮다"로 읽히면 안 되기 때문이다 — 전 비행 감사가 따르는
규칙과 같다. 비용은 실재한다(무해한 구역 재발행도 비행을 세운다). 그 비용이 받아들일 수
없다면 per-piece 재감사를 먼저 지어야 한다.
