# 모듈화 최종 코드 검토 — 2026-10-11

이 문서는 잔여 경계 구현 전의 검토 기록이다. 이후 세 항목의 구현은
[잔여 모듈화 경계 정리](remaining_modularization_2026_10_11_ko.md)를 참고한다.

검토 시작 기준은 상위 `a297f2342cebe33ff0e9edd2eb55db0172ea65bb`, 계획 서브모듈
`d2ef5496432b86cfb19d10be5658a369c9cc3725`이다. 작업 위치는
`/home/suv/ws/temp/MMP_module/MMP`, 실행 환경은 기존 `mmp_dev` 컨테이너다.
기준 checkout의 저장된 검증 자료를 재확인한 뒤, 현재 호출 경로와 경계 조건을 별도로
검토했다. `reports`와 `experiments` 디렉터리는 읽거나 탐색하지 않았다.
stash·기존 미추적 그림·무관한 로컬 자료는 건드리지 않았다.

## 설계 요구와 실제 구현

장기 설계의 ROS 없는 SDK·전체 SI 전환·C++ 지형 로더를 완료 기준으로 삼지 않는다.
현재 범위는 ROS 실행과 기존 계산을 유지하면서 재사용 가능한 함수·교체 경계를
정리하는 후속 설계의 1~6단계다. 문서에 제안된 클래스 이름을 그대로 옮기는 대신
현재 실행도 같은 계산 경계를 사용하도록 구현되어 있다.

| 항목 | 확인한 구현 | 판단 |
|---|---|---|
| 전체 계획 진입점 | `MissionPlanner::plan(request)` → `RosPlanningBackend` → manager/chain | 외부 호출자가 탐색·분할·병렬 최적화·연결을 직접 조정할 필요 없음 |
| 기존 실행 경로 | `ReplanFSM`이 실제로 같은 `MissionPlanner::plan()` 호출 | 외부용과 ROS용 계획 알고리즘이 중복되지 않음 |
| 설정·환경·미션 | `PlanSettings`, `PlanningEnvironment`, `MissionPlanRequest` | 값 입력 경계 구현. 기존 파라미터·단위·AGL 규약 유지 |
| 결과 수명 | `MissionPlanResult`, `PlanningComputation`, `OptimizerSolveResult` | 궤적 계수·시간을 값으로 소유. 다음 호출·planner 파괴와 독립 |
| 계산과 채택 | `compute()` / seal / apply, context·실행·환경 revision | 오래된 결과·다른 manager 결과·중복 채택 거부. 완전한 무부작용 함수는 아님 |
| 궤적 평가 | `evaluateFlight()`와 소유하는 평가 결과 | 외부 궤적에도 호출 가능. 기존 표본 간격·direct/chain 평가 차이 유지 |
| 기체·탐색 교체 | `VehicleModel`, `RouteSearchBackend` | 탐색·최적화·평가의 모델 연결과 구체 탐색 구현 분리. 범용 VTOL 상태 모델까지 보장하지 않음 |
| 최적화 본체 | `PolyTrajOptimizer::solve()` → 기존 MINCO/L-BFGS | solver 내부 분해 없이 설정·환경·비용 프로필·결과 경계만 정리 |
| 시뮬레이션 | `simulate()` / `SimulationSession::step()` | 기존 ROS 시뮬레이터도 같은 엔진 사용. 모델·제어기 전략 주입 |
| 시각화 | 표시 메시지 `make*()` / ROS 발행 어댑터 | 준비와 발행 분리, 연결 생략 가능. 렌더링은 기존 RViz |
| Python 지도 | `load_map()` / ROS 서비스·직렬화 어댑터 | 같은 로더 계산 재사용. 서비스 상태·rollback·발행은 ROS 소유 |
| 설치·예제 | calculations, manager, optimizer, vehicle exported targets | source include를 직접 지정하지 않는 외부 consumer 구성 제공 |
| 큰 함수·중복 처리 | 경로 후처리·구간 계산·조립, `PlanContext`, worker 설정 view, terrain-risk 계산 | 계산 단위와 호출별 상태가 분리됨. manager/chain의 조정 책임은 여전히 존재 |

따라서 **후속 1~6단계의 기능 경계는 구현되었지만, 내부 결합까지 모두 제거됐다고
평가하지 않는다.** “90%”는 별도 측정 기준이 없는 정성적 표현이다. 완료 항목과 남은
책임을 위처럼 구분하며, 단계 완료를 결함 부재나 일반 임무 성공률로 해석하지 않는다.

## 이번 검토에서 재현하고 수정한 결함

### 1. 낮은 마지막 구간 속도 상한이 지정 종단 상태를 덮어씀

`SegmentChainPlanner`의 baseline 경로는 도착 속도를 구간 상한으로 낮추면서 지정한
속도·가속도를 통째로 바꿀 수 있었다. route 경로는 지정 속도는 보존했지만 가속도만
지정한 경우 이를 0으로 바꿨다. 호출자는 성공 결과를 받으면서 요청한 종단 상태를
잃을 수 있었다. 최근 내부 정리 이전부터 존재한 결함이다.

두 경로의 `completeArrivalTail()`은 **지정되지 않은 성분만** 채운다. 속도·가속도의
독립 지정 플래그, 기존 자유 종단 기본값, solver 내부 정책은 유지한다.

재현: baseline/route의 `segdiff + tailacc`, N=2 baseline의 지정 속도 1.9 u/s와
구간 상한 1.8 u/s. 세 경우 모두 수정 전 실패했고 수정 뒤 통과했다. route의 명시
속도 조합도 통과했다. 구간 수 검사를 추가해 fallback으로 검사가 가려지지 않게 했다.

### 2. 외부 전환 prefix가 direct fallback에서 누락될 수 있음

공개 `planOverRoute()`에 올바른 prefix와 handoff를 전달해도 별도의 legacy flag를
설정하지 않으면 worker 실패 후 prefix 없는 direct flight를 성공 반환할 수 있었다.
이 역시 최근 내부 정리 이전부터 존재했다.

실제 prefix가 있는 요청은 호출별 attempt를 전환 중으로 설정해 기존 fallback 금지
정책을 적용한다. 전체 요청에 전역 상태를 추가하지 않는다. 강제 worker 실패로 수정
전 오류를 재현했고, 수정 뒤 거부 및 다음 prefix 없는 요청의 정상 fallback을 확인한다.

### 3. 비유한 요청 override가 다음 정상 요청까지 오염시킴

`length_per_piece`와 `obstacle_clearance`는 양수면 manager에 지속 적용한다.
`+inf`도 수락되어 뒤의 정상 요청까지 실패할 수 있었다. 기존 설치 라이브러리에서
새 회귀 시험을 실행했을 때 direct/chain 모두 후속 정상 계획이 실패했다.

두 값 모두 finite인지 **어느 것도 적용하기 전에** 검사한다. 유한한 0 이하 값은
기존대로 현재 설정 유지다. 각 필드의 `+inf/-inf/NaN`에 유효한 반대 필드를 조합한
요청을 거부하고, 후속 정상 계획의 최종·참고 계수·시간·상태가 원래와 같은지 검사한다.

## 계약과 문서 보완

- 환경·모델·공유 설정 변경 전 모든 worker solve를 종료하고, 다음 solve 전에 worker를
  다시 생성해야 함을 명시했다. worker의 geometry·sampling pitch 등은 생성 당시
  값이고 조회 callback은 live manager를 참조하기 때문이다.
- 시뮬레이션·Python 로더·표시 준비가 아직 미구현처럼 적힌 과거 사용 문서 문구를
  현재 API 색인과 일치시켰다. 시간순 과거 검증 기록은 그대로 보존한다.
- 선택적 bool 설정의 재설정 시 기존 값 유지 동작은 과거 구현과 같아 변경하지 않았다.

## 검증 결과

| 실제 재실행 | 결과 |
|---|---|
| 별도 build/install 디렉터리의 Release 빌드 | vehicle·optimizer·manager 3개 패키지 통과; 기존 ROS/지도 등 의존성은 `/ws/install` underlay 사용 |
| 수정 후 통합 CTest | **48/48**, 이 안의 chain **73/73**, risk **154/154** |
| 설치 Python 로더 시험 | **69/69**, 설치 모듈 import와 기존 소스 해시 확인 |
| 설치 라이브러리 외부 CMake 소비자 | 계획·평가·시뮬레이션·표시 예제 **4/4** 실행; compile command에 source include 없음 |
| 실제 Python 지도 로딩 + 외부 R2 계획 | **190조각**, double 시간 합 **521.0625247312521초**, 결과 전 필드 기준과 동일 |
| 일반 `/ws/install` 재빌드 + 실제 RViz R2 | float32 궤적 **190조각**, **521.0625259429216초**, 최종·참고 계수·시간과 FE 표시 데이터 동일 |
| 실제 환경 ABORT·복구 | 대상 ID 1을 한 번 철회, 새 명령 ID 2 생성, 복구 궤적 수치 동일 |
| 평가·후처리·비용 프로필 로그 | 각각 38·2·10개 기준과 동일 |
| 실행 중 입력/설치물 동일성 | 명시한 **421개** 소스·입력·설치물의 전후 해시 일치 |
| 독립 수정안 재검수 | 다른 검토자가 종단 성분 4조합·prefix guard 위치/수명·설정 적용 순서 확인, 추가 blocker 없음 |

CTest runner는 소스에서 변형 이름을 자동 수집한다. 기존 71개에 새 `tailcap`,
`prefixfallback`이 더해져 73개가 실행됐다. 처음 요약 스크립트의 71개 기대값 때문에
집계만 false였으며, 실제 CTest는 처음부터 48/48 통과했다. 원본 집계도 보존하고,
소스의 변형 이름 집합·실제 73개 로그·tally를 대조하여 집계를 수정했다. 통과한
시험을 다시 실행하거나 실패 기록을 덮어쓰지 않았다.

R2 직렬화는 절대 실행 stamp를 수치 비교에서 제외한다. double 결과 전체 SHA256은
`33b40dc818378be2343349bd31bff20c309bbb7aa2725d1f00e76b88ce1ebfe5`로 이전과 같다.
RViz는 일반 설치 경로의 실행 파일과 실제 `/proc/<pid>/exe` 해시가 일치함을 확인했다.
수정 전 잘못된 종단/누락 prefix/비유한 입력 동작은 의도적으로 달라졌고, 정상 R2의
결과는 보존됐다. 일반 임무 전체 성공률이나 성능 향상 수치를 뜻하지 않는다.
이번에는 별도 반복 성능 벤치마크를 수행하지 않았다. 기존 compiler warning은 남는다.

원본 로그·재현 전후·소스 patch·외부 consumer·화면·해시와 검사기는
[최종 검토 검증 자료](../validation/final_review_2026_10_11/README.md)에 보존한다.
기존 검증 자료와 이번 실제 재실행 결과를 구분했다. 시험 프로세스와 이번 검토용
격리 display는 정리했으며 정상 `/ws/install`에는 수정본이 빌드되어 있다.

## 남아 있는 책임과 사용 제한

1. **baseline/fallback 설정 전달**: 전체 프로필은 ROS 파라미터를 scoped 변경·복구한다.
   호출별 설정을 경로 준비·탐색·최적화·평가 전체에 전달하는 정리는 남았다. worker의
   제한된 override 경계만으로 이를 대체하면 서로 다른 clearance를 볼 수 있다.
2. **PathManager 책임**: 환경·SDF·위험장·실행 저장소 소유를 유지한다. 다음 구조 변경은
   환경 조회/갱신과 실행 결과 보관/철회를 분리하되 현재 revision·ABORT 순서를 보존해야 한다.
3. **내부 접근**: 호환용 public 헤더·필드가 남는다. 기존 호출자를 옮긴 뒤 접근을 제한한다.
4. 동일 planner의 계획·갱신은 직렬이다. `plan()`은 동기이며 일반적인 비동기 계산 취소
   토큰은 제공하지 않는다. ROS 실행 ABORT의 기존 철회/재계획 경로와 구분한다.
5. FAILED `MissionPlanResult`는 빈 결과이며 해당 계산의 평가도 제공하지 않는다.
   필요하면 `compute()`의 진단이나 별도 `evaluateFlight()`를 사용한다. direct 계획은
   기존 optimizer audit만 거치고 whole-flight 평가는 미평가 상태다.
6. 병렬 worker에서 처리하지 않은 제공자/모델 예외는 프로세스를 종료할 수 있다.
   예외 수집·복구 정책은 이번에 바꾸지 않았다. 평가 역시 연속 시간 충돌 증명이 아니다.
7. 전체 ROS 제거·C++ 지형 이식·독립 렌더러·범용 VTOL/6DOF는 유보 범위다.

후속 우선순위는 **호출별 설정 전달 → manager의 환경/실행 책임 분리 → 내부 접근 제한**이
타당하다. 서로 연결된 호출 경로를 함께 검토하되, 각 단계는 기존 수치·실행 검증을
통과하는 작은 변경으로 나눈다. 이 최종 검토에서는 새 구조 작업으로 범위를 넓히지 않았다.
