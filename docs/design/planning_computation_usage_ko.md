# 계획 계산·실행 채택·궤적 평가 사용 계약

2026-10-10 후속 모듈화 단계 1·2의 현재 API를 설명한다. 기존 전체 계획 사용법은 [planning_library_usage_ko.md](planning_library_usage_ko.md)를 함께 참고한다. 이 문서는 호출 계약을 설명하며, 빌드·실행 검증 결과는 단계별 검증 기록에서 별도로 확인한다.

## 호출 경계

```text
MissionPlanner::plan(request)                 기존 전체 계획 호출
  └─ RosPlanningBackend::plan(request)
       ├─ compute(request)                   소유 계산 결과 생성
       │    ├─ PathManager::computeGlobalTrajectory()
       │    └─ SegmentChainPlanner::compute()
       │         └─ evaluateFlight(input)    기존 chain 평가 범위
       ├─ applyPlanningComputation(result)   실행 효과 명시 적용
       └─ MissionPlanResult                  소유 궤적 + 실행 ID

외부 궤적 → evaluateFlight(input) → FlightEvaluationReport
                                     └─ formatFlightEvaluation(report)
```

기존 `MissionPlanner::plan()`은 실행 ID와 시각을 반환하는 ROS 호환 동작을 유지한다. 실행 저장소에 후보를 채택하지 않고 계산 결과를 확인하려면 구체적인 `RosPlanningBackend::compute()`를 사용한다. 이 함수는 `PlanningComputation`을 반환한다. 직접 최적화와 chain의 내부 계산도 이 결과를 만들기 위해 실행 궤적 저장소를 먼저 쓰지 않는다.

`compute()`는 무상태 함수가 아니다. 경로·정책 진단, 기존 ROS 설정과 환경 준비, 일부 시각화 발행은 유지된다. 지형·장애물 준비 중 기존 실행 궤적의 재검증과 철회가 발생할 수 있다. 새 후보의 채택과 기존 비행의 환경 철회는 서로 다른 효과다.

## 계획 입력과 결과

| 자료형·필드 | 준비할 값과 의미 | 소유권·단위 |
|---|---|---|
| `PlanSettings.parameters` | 기존 ROS 파라미터 이름과 정확한 값 타입의 초기 덮어쓰기 | 값 소유. 기본값은 기존 소비자가 선언하며 새 기본값 표를 만들지 않음 |
| `PlanSettings.vehicle_model` | 계획용 기체 모델. 명시 주입하면 탐색 제약·optimizer·chain 평가가 같은 객체 사용 | `shared_ptr<const VehicleModel>`. 모든 const 질의는 내부 worker의 동시 호출을 허용해야 함 |
| `PlanSettings.route_search_backend` | 경로와 위험 정책·출처를 함께 반환하는 탐색 구현 | 공유 포인터. 임무 사이에 직렬로 교체 |
| `PlanningEnvironment.terrain` | 원본 중심·길이·해상도·행/열과 지형 배열 | 값 소유. 기존 planner 좌표와 단위. `elevation[col * rows + row]`, NaN과 기존 격자 방향 유지 |
| `PlanningEnvironment.risk_zones / dynamic_obstacles` | 초기 환경 수정 입력 | 미지정은 기존 값 유지, 명시한 빈 목록은 기존 삭제 규칙 적용. 불변 환경 스냅샷 아님 |
| `MissionPlanRequest.head` | 위치·속도·가속도와 출처 `StartStateSource`, 가속도 명시 여부 | 기존 planner frame. 위치 `u`, 속도 `u/s`, 가속도 `u/s²` |
| `start_altitude` | `ABSOLUTE`: head z가 기존 전역 z. `COMMAND_AGL`: 지형과 기존 최소 AGL 규칙으로 한 번 해석 | 요청은 값으로 전달하므로 호출자 원본의 AGL 값을 다시 쓰지 않음. 지형 없을 때 기존 절대 고도 처리 유지 |
| `waypoints` | 미션 경유점·목표 | xy는 기존 frame, z는 기존 미션 AGL. head의 고도 모드와 혼동하지 않음 |
| `tail` | 명시한 끝 속도·가속도와 지정 여부 | `u/s`, `u/s²`. 기존 끝 경계 완화 정책 유지 |
| `formation_*` | 기존 편대 정보 | 기존 값과 optimizer 전달 순서 유지 |
| `length_per_piece / obstacle_clearance` | 양수인 경우 기존 manager 덮어쓰기 적용 | 기존 `u`. 이후 비양수 요청은 앞선 설정을 되돌리지 않음 |
| `PlannedTrajectory.trajectory` | 계산된 최종 궤적 | 계수와 조각별 시간을 값 소유. 조각 시간과 궤적 질의 t는 초 |
| `reference_trajectory` | 직접 계획은 MINCO seed, baseline chain은 최적화 baseline, route chain은 전체 비행 | 최종 궤적과 같은 소유권·frame·단위 |
| `start_time / reference_start_time` | 기존 ROS clock 지점에서 기록한 절대 초 | 직접 계획은 최적화 전 참고 시각·후 실행 시각. chain이 최종 조합·선택한 산출물은 기존 평가 후 시각. chain 내부 직접 fallback·다중 경유점 직접 경로는 직접 solve의 전/후 시각. 그 자체로 실행 명령이 아님 |
| `PlanningComputation.status / evaluation / phases` | 기존 성공·저하·실패 판정, 전체 비행 평가, 단계 종료 시각 | 결과 값 소유. 단계의 `t_end`는 궤적 시작부터 상대 초 |
| `identity_advances / invalidate_execution` | 과거 중간 계산들이 소비하던 ID 수와 명시적 무효화 의도 | ROS 실행 호환 메타데이터. 새 알고리즘 입력이나 궤적 품질 지표 아님 |
| `environment_generation`와 채택 문맥 필드 | 계산 완료 시점의 환경·manager·계산/실행 문맥 | 같은 프로세스의 채택 검사 전용. 외부 저장·재생용 영구 식별자나 환경 스냅샷 아님 |
| `ExecutionReceipt` | 실행 효과가 적용됐는지, 적용 후 실행 ID와 drone ID | `applied=false`이면 실행 상태에 적용하지 않았음 |
| `MissionPlanResult` | 기존 전체 계획 API의 결과 | 성공 시 소유 궤적과 채택 ID. 실패 시 과거·거부 궤적을 노출하지 않음 |

현재 기본 환산은 `1 u = 100 m`이지만 경계에서 SI로 일괄 변환하지 않는다. 기존 `dynamics_unit_xy_m`, `dynamics_unit_z_m` 설정이 적용되는 곳과 원래 planner 좌표를 사용하는 곳을 구분한다. `VehicleModel`에 전달하는 물리 PVA만 SI로 환산한다.

## 계산 결과를 확인한 후 실행에 채택

다음은 준비된 입력을 사용하는 최소 형태다. `options`의 ROS context는 이미 초기화돼 있어야 한다. 실제 실행에서는 backend를 발행·재검증·철회가 끝날 때까지 유지한다.

```cpp
#include <utility>
#include "path_manager/path_manager.h"
#include "path_manager/ros_planning_backend.h"

namespace pm = path_manager;

void runPreparedMission(pm::RosPlanningOptions options,
                        pm::PlanSettings settings,
                        pm::PlanningEnvironment environment,
                        pm::MissionPlanRequest request)
{
  auto backend = pm::makeRosPlanningBackend(std::move(options));
  backend->initialize(std::move(settings), std::move(environment));

  auto computed = backend->compute(std::move(request));
  // computed.hasTrajectory()인 경우 product의 값을 여기서 검사할 수 있다.
  // 이 계산만으로 후보가 실행 궤적 저장소에 채택되지는 않는다.

  // 기존 실행 호환 정책을 적용하기로 한 경우 명시적으로 호출한다.
  // 실패 결과에도 '기존 실행 무효화' 효과가 있을 수 있으므로 이 예는
  // 실패 여부를 확인하기 전에 그 명시적 효과까지 적용한다.
  const auto receipt =
      backend->executionManager()->applyPlanningComputation(computed);
  if (!receipt.applied) return;             // 오래됐거나 다른 문맥
  if (!computed.hasTrajectory()) return;  // 실행할 새 궤적 없음

  const auto &flight = computed.product->trajectory;
  const auto position_at_start = flight.getPos(0.0);
  (void)position_at_start;
  // receipt.trajectory_id로 현재 실행을 식별한다.
  // 이 예에는 토픽 발행이 없다. 실제 애플리케이션은 backend를 유지한다.
}
```

계산만 필요한 호출자는 채택을 생략하고 성공 결과의 소유 궤적을 사용하면 된다. `product`가 있다는 이유만으로 성공으로 판단하지 않는다. **`hasTrajectory()`가 기준**이다. 최적화 후 전체 평가에 거부된 경우에도 과거의 ID 소비·실행 무효화 동작을 재현하기 위한 중간 `product`가 남을 수 있다. 이 궤적은 실행 대상으로 제공되지 않는다. `RosPlanningBackend::plan()`은 이런 중간 궤적을 `MissionPlanResult`에 넣지 않는다.

ID는 계산할 때 실행 저장소에서 발급하지 않는다. 명시 채택 시 과거의 증가 수를 적용한다. 직접 계획·route chain의 최종 성공은 보통 1개를 소비하고, baseline chain은 baseline과 성공한 중간 구간, 마지막 결과의 과거 증가 수를 보존한다. 따라서 ID 차이를 새 임무 수나 공개 성공 결과 수로 해석하지 않는다.

계산 후 다음 중 하나가 일어나면 기존 결과의 채택은 거부된다.

- 같은 manager에서 다른 계산을 시작한 경우. 그 새 계산이 초기 입력 검사에서 실패해도 이전 후보는 오래된 문맥이다.
- 다른 결과가 이미 채택됐거나 같은 결과를 다시 적용하는 경우.
- 실행 궤적의 환경 철회·emergency 교체가 발생한 경우.
- 환경 revision, 기체 모델 바인딩 또는 탐색 바인딩이 바뀐 경우.
- 아직 SDF에 반영하지 않은 장애물 입력이나 manager의 계산 설정을 바꾼 경우.
- 다른 manager에서 계산한 결과인 경우.

거부된 채택은 실행 저장소를 수정하지 않는다. 그 결과의 소유 계수는 계속 읽을 수 있지만, 다시 실행하려면 현재 문맥에서 새로 계산해야 한다. `sealPlanningComputation()`은 내부 계산 생산자가 완료 시 호출하는 신뢰된 프로토콜이다. 사용자가 오래된 결과를 다시 seal하거나 문맥 필드를 고쳐 재채택하는 용도가 아니다.

같은 객체의 호출은 직렬로 한다. 특히 **계산과 채택 사이에 live ROS 파라미터나 기타 설정을 변경하지 않는다.** 임의 ROS 파라미터 변경 전체를 추적하는 설정 스냅샷·revision은 제공하지 않는다. 결과를 오래 보관한 뒤 재생하는 기능도 이 채택 API의 범위가 아니다.

미션 사이의 live ROS 기본값 변경도 모든 소비자에 즉시 적용되지는 않는다. primary
optimizer가 사용하는 clearance·altitude weight·headroom과 canonical mission 위험장을
갱신하려면 계산·실행·worker 사용을 끝낸 뒤 `PathManager::initOptimizer(true)`로 다시
초기화한다. 독립 worker와 slice는 생성 시점의 기본값을 읽으며, 그 생성이 primary의
초기화 값을 갱신하지 않는다. 호출별 `DirectPlanSettings` 역시 공유 기본값을 바꾸지 않는다.

기존 호환성을 위해 남은 public `PathManager::traj_`를 외부에서 직접 수정하는 것도
이 프로토콜의 지원 범위가 아니다. FSM은 제한된 const 조회만 사용한다. 그 조회가
반환한 참조는 manager 수명을 빌리므로 장기간 보관할 데이터에는 소유 결과를 사용한다.

## 독립 궤적 평가

`evaluateFlight(const FlightEvaluationInput&)`는 계산한 궤적뿐 아니라 외부에서 준비한 `poly_traj::Trajectory`도 평가한다. ROS node, 실행 저장소, 발행기를 입력으로 받지 않는다. 계산 결과의 로그 문자열은 `formatFlightEvaluation(report)`로 별도 생성하며, 실제 출력은 호출자가 선택한다.

완전한 실행 예제는 [flight_evaluator_example.cpp](../../src/path_manager/examples/flight_evaluator_example.cpp)다.
설치 target `path_manager::path_manager_lib`에 연결하거나 기존 컨테이너에서
`ros2 run path_manager flight_evaluator_example`로 호출할 수 있다. 예제의 계산은
ROS node를 생성하지 않으며, 설치 라이브러리의 패키지 의존성은 아직 ROS를 포함한다.

| 평가 입력 | 의미·단위·사용 조건 |
|---|---|
| `trajectory` | 평가할 계수·시간. 기존 planner frame과 `u`, 초. 동기 호출 동안 const 참조로 빌림 |
| `phases` | `PhaseSpan`의 엄격히 증가하는 상대 종료 초. 마지막 종료는 전체 궤적 길이와 일치. `TRANSITION`은 연속 선행 구간, `TERMINAL`은 마지막 구간 |
| `settings.unit_xy_m / unit_z_m` | 모델 입력 SI 변환용 축별 m/u |
| `settings.obstacle_clearance` | 지형 위 기존 표시·평가 band 높이, `u`. 별도의 전 공간 장애물 충돌 검사를 뜻하지 않음 |
| `envelope_violation_max` | 위반 표본 비율 기준. fraction이며 기존 `>=` 비교 유지 |
| `envelope_peak_max` | 최대 envelope 사용률 기준. 무차원이며 기존 `>` 비교 유지 |
| `model` | 불변 공유 기체 모델. null이면 동역학 평가 비활성화. 실제 계획에 쓴 모델과 일치시키는 책임은 호출자에게 있음 |
| `terrain_elevation(x,y,z*)` | 기존 xy에서 절대 전역 지형 z 조회, `u`. 제공자 없음은 기존 해수면 0 처리. false 반환 시 제공자는 초기화된 출력값을 변경하지 않아야 함 |
| `zone_count`와 위험·접촉 조회 5개 | 위험영역이 있을 때 모두 필요. 기존 effective risk, ellipsoid radius, visibility, routing contact, authored contact 계산 사용 |
| `plan_policy / leg_policies` | 평가 대상 탐색에서 얻은 소유 정책·형상·출처. 모르는 정책을 `HARD_AVOID`나 측정 완료로 새로 만들어 넣지 않음 |
| `piece_leg / piece_epoch / policy_epoch` | 해당 궤적의 조각→탐색 leg 귀속과 같은 탐색에서 온 증거. `piece_leg`는 호출 동안 빌림. 단순 크기 일치는 출처 검증을 대체하지 않음 |

위험영역 접촉 제공자는 정책의 generation·epoch·ordered shape·valid flag를 검증하고 `STALE / INVALID`를 반환할 수 있어야 한다. 여기서 확인 불가를 `CLEAR`로 치환하면 안 된다. 위험영역이 있는데 제공자 일부가 빠지면 `MISSING_ZONE_QUERIES`로 미평가 결과를 반환한다.

각 조각의 시간은 유한한 양수, 전체 시간과 계수는 유한해야 한다. 위반하면
`INVALID_TRAJECTORY`로 거부한다. 단위 배율은 유한한 양수이고 나머지 평가 설정은
유한해야 하며, 위반하면 `INVALID_SETTINGS`를 반환한다. 두 경우 모두 환경 callback을
실행하기 전에 거부한다. 제공자는 기존 의미에 맞는 유한한 값을 반환해야 한다.
이 검사는 임의 크기의 외부 다항식에 대한 수치 안정성이나 연속 시간 안전을 증명하지 않는다.

다음 예제는 위험영역이 없는 평평한 지형을 명시한 최소 평가 함수다. 실제 지도에 적용할 때는 같은 frame의 실제 지형 조회를 제공한다. `model`은 호출자가 준비한 유효 모델이다.

```cpp
#include <utility>
#include <vector>
#include "path_manager/flight_evaluator.h"

path_manager::FlightEvaluationReport evaluateFlatFlight(
    const poly_traj::Trajectory &flight,
    mmp_vehicle_dynamics::VehicleModel::Ptr model)
{
  using namespace path_manager;
  const std::vector<PhaseSpan> phases{
      {flight.getTotalDuration(), PhaseKind::CRUISE, "flight"}};
  FlightEvaluationInput input(flight, phases);
  input.model = std::move(model);
  input.environment.zone_count = 0;
  input.environment.terrain_elevation =
      [](double, double, double *height) {
        *height = 0.0;  // 기존 frame의 절대 지형 z, u
        return true;
      };
  return evaluateFlight(input);
}
```

`FlightEvaluationReport`는 판정의 측정 조건인 `FlightVerdict`, 입력 문제 `issue`, 단계별·전체 측정값, 위험 접촉과 handover 진단을 소유한다. 함수가 반환한 후 입력 궤적·단계 배열·환경 제공자가 없어져도 보고서는 유지된다.

기존 0.1초 표본 간격과 leg 경계 별도 검사, cruise 진입 latch, 위험 접촉 판정 수식을 유지한다. 이 함수는 연속 시간 충돌 증명이나 일반 SDF 충돌 감사 전체를 대체하지 않는다. `TRANSITION`은 기존 규칙대로 cruise envelope 통계에서 제외되지만 지형·위험 평가는 수행한다. `PhaseSpan.name`은 출력 이름이고 `PhaseKind`가 판단 규칙을 선택한다.

`verdict.evaluated`를 먼저 확인한다. `unflyable()`은 미평가 입력에서 false이므로 그것만으로 평가 성공 여부를 판단하지 않는다. 성공·저하·실패로 변환하는 기존 정책은 chain coordinator에 남는다. chain 비활성 직접 계획은 기존 optimizer 검사를 유지하고, 새 전체 비행 평가를 자동으로 추가하지 않는다. chain 내부 직접 fallback과 다중 경유점 직접 경로의 기존 전체 평가는 유지한다.

## 수명·병렬성·예외·남은 ROS 연결

- 계산 결과와 평가 보고서는 값을 소유한다. 계산 중 지형·거리장·위험 조회와 evaluator의 궤적·단계·귀속 포인터는 동기 호출이 끝날 때까지 유효하고 안정적이어야 한다. 결과가 환경의 수명을 연장하거나 불변 복사본을 만들지는 않는다.
- 같은 planner의 외부 호출과 환경·설정 변경은 직렬이다. 내부 구간 worker는 독립 optimizer를 사용하며 공유 환경을 읽는다. 명시 주입한 `VehicleModel`은 불변이고 concurrent const 질의를 지원해야 한다. 모델이 일반적으로 모든 VTOL 상태·전환 제어를 지원한다는 뜻은 아니다.
- 계산 중 기존 전단·궤적 시각화 발행은 일부 남아 있다. 후보의 실행 채택을 생략했다고 모든 ROS 표시가 생략되는 것은 아니다. 표시 준비와 ROS 발행은 후속 단계에서 분리했으며, `PlanSettings.attach_visualization=false`로 manager의 표시 연결을 생략할 수 있다. [표시 API 사용법](planning_visualization_usage_ko.md)을 참고한다.
- 실제 ROS 실행에서는 FSM의 콜백·실행 상태·발행·pending 결과 처리와 ABORT 순서를 유지한다. FSM은 `executionTrajectory()`와 `referenceTrajectory()`의 제한된 const 조회를 사용한다. 환경 ABORT는 철회 대상 ID가 살아 있을 때 발행하고 이어서 실행 유효 상태를 지운다.
- 초기화·잘못된 설정의 기존 예외 계약과 동기 평가 제공자의 예외 전파를 유지한다. 모든 예외를 `PlanResult::FAILED`로 변환하지 않는다. 내부 병렬 worker에서 처리하지 않은 예외가 발생하면 현재 구현은 프로세스를 종료할 수 있다. 이 단계에서 worker 예외 수집·복구 정책을 새로 도입하지 않았다.
- `PolyTrajOptimizer::solve()` 안의 MINCO·L-BFGS는 그대로 유지한다. 이 문서는 계획·실행·평가 계약을 다룬다. 후속 단계에서 분리한 시뮬레이션 계산·Python 지도 처리·표시 준비는 [기능별 API 색인](modularized_function_api_ko.md)을 참고한다. 독립 렌더러와 전체 ROS 제거는 아직 제공 범위에 포함하지 않는다.
