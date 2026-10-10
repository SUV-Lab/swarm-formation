# 기능별 API와 설치 target 최종 검증 — 2026-10-10

후속 계획의 단계 1–6을 적용한 최종 설치 상태에서 빌드·실행을 검증했다.
현재 사용법은 [공개 API 색인](../../design/modularized_function_api_ko.md),
변경·설계 검수 기록은 [단계별 구현 기록](../../design/incremental_modularization_ko.md)에 있다.

| 확인 | 최종 결과 |
|---|---|
| 컨테이너 빌드 | 차량·최적화·계획 세 패키지 성공, 기존 설치에서 명시한 경로만 탐색 |
| CTest | **41/41**: 계획 33, 최적화 2, 차량 6; chain 71·risk 154 사례 포함 |
| 외부 설치 소비자 | 계획·평가·시뮬레이션·표시 **4/4** configure·build·실행 성공 |
| 새 외부 R2 빌드 | 원본의 소유 double 결과와 전체 파일 SHA256까지 동일 |
| 실제 RViz | 임무 로드·R2 생성·환경 변경 ABORT·새 임무 복구 성공 |
| 입력 동결 | 소스·입력 **183개**, 외부 소비자가 사용한 설치물·관련 소스 시험 전후 동일 |

계산 14개 소스를 `path_manager_calculations`로 묶고 기존 `path_manager_lib`는
이를 전이 제공한다. ROS backend·manager·chain 조정·FSM·발행은 기존 target에
남는다. 이번 후속 작업의 시작점 `57ac7a8` 이후 `path_optimizer`와 `path_planner`
코드는 바꾸지 않았다. MINCO·L-BFGS를 다시 분할하지 않았다.

외부 예제 프로젝트는 설치된 CMake imported target만 사용한다. `/tmp`에 설치
예제를 복사해 새로 구성했고, 컴파일 명령에 `/ws/src` include나 구현 소스 직접
컴파일이 없음을 확인했다. 기존 workspace의 symlink install은 유지한다.
독립 머신에서 ROS 없이 배포·빌드했다는 뜻은 아니다.

R2는 `r2_ridge_crossing.yaml`이며 최종/참고 궤적, phase, 평가, 상태·사유, ID·환경
정보가 원본과 같다. 소유 double 결과 파일 SHA256은
`33b40dc818378be2343349bd31bff20c309bbb7aa2725d1f00e76b88ce1ebfe5`다.
이 schema는 절대 wall-clock start stamp를 직렬화하지 않는다. 비교 후 임의로
다른 필드를 제외하거나 정규화하지 않았다. 190조각의 double 시간 합은
521.0625247312521초이며 별도 float32 wire 시간 합은 521.0625259429216초다.
각 표현을 같은 표현의 원본과 비교했다.

GUI 시험에서는 기존 RViz 패널로 로드·계획하고 DDS 출력과 로그를 수집했다.
초기/최종 궤적과 전단 경로, 평가 38개·후처리 2개·비용 프로필 10개가 동일하다.
환경 변경으로 철회한 뒤 새 ID의 복구 궤적도 동일하다. GUI 캡처는 실행 증거이며
모든 화면 픽셀의 동등성을 검사한 것은 아니다. 표시 메시지·삭제·지연 구독의
정확한 비교는 아래 단계 4에서 별도로 수행했다.

## 보관 자료

`evidence.tar.gz`에 **253개** 자료를 압축해 보존했고 `manifest.json`이 개별 해시를
정의한다. 실행 바이너리·전체 build 트리·대형 지형 자산은 포함하지 않았다.

- `runtime`: 최종 build/CTest/GUI 명령·로그·JSON·화면과 소스 해시.
- `consumer`: 실제 실행한 외부 CMake 프로젝트·예제·R2 source, compile/link 명령,
  실행 로그, 전후 설치물 해시, 소유 결과와 wire 비교.
- `baseline`: 원본 R2 소유 결과 및 GUI mission/log. 소유 결과 비교는 이 자료만으로 가능하다.
- `source`, `change`: 이번 target·문서·외부 consumer 변경 5개와 적용 전후 해시.
- `harness`: 사용한 GUI 드라이버/도우미와 보관 스크립트.

```bash
python3 verify.py
```

표준 Python만으로 압축본과 모든 파일의 해시, 저장된 성공 기록을 검수하고
원본/최종 소유 R2 결과의 바이트 동등성을 직접 확인한다. 빌드·ROS·GUI를 다시
실행하는 검증은 아니다. 저장된 하네스는 실행 당시 `/tmp` 경로를 유지하므로 재실행은
해당 경로·assets·ROS 설치·컨테이너와 GUI 구성을 준비해야 한다.
GUI 도메인은 82, 외부 예제/R2는 84, CTest는 85로 분리했다.

최종 빌드 명령은 다음과 같다. workspace 전체 탐색을 하지 않는다.

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
cd /ws
CMAKE_BUILD_PARALLEL_LEVEL=4 MAKEFLAGS=-j4 colcon build --symlink-install \
  --base-paths src/mmp_vehicle_dynamics \
    src/mmp_path_planning/src/path_optimizer src/mmp_path_planning/src/path_manager \
  --packages-select mmp_vehicle_dynamics path_optimizer path_manager \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
```

기존 컴파일 경고는 남아 있다. 수행 범위는 현재 컨테이너의 증분 빌드와 새 외부
consumer build이며, 모든 의존성을 지운 clean workspace 또는 다른 머신 시험은 아니다.

## 단계별 증거와 설계 범위

- [계산 결과·채택·평가](../planning_computation_2026_10_10/README.md): 원본 수치,
  소유권·실패·토큰·평가 및 기존 ABORT·복구.
- [시뮬레이션](../../../../../docs/validation/trajectory_simulation_2026_10_10/README.md):
  네 원본 상태 이력과 batch/step, 실제 원본/변경 DDS·스크럽·ABORT 비교.
- [표시 준비·발행](../planning_visualization_2026_10_10/README.md): 62개 표시 payload,
  연결/미연결 물리 상태·R2, 빌더 28개 검사.
- [Python 지도](../../../../../docs/validation/map_loader_2026_10_10/README.md):
  설치 검사 69/69, R2 지형 27개 정확 비교와 별도 실제 GUI 시험.

별도 읽기 전용 검수에서 실제 ROS 호출의 재사용, 값으로 소유하는 결과 수명,
계산/채택과 평가/로그 분리, 단위와 ABORT를 확인했다. 중대한 문제는 발견하지 못했다.
ROS 환경을 이용하는 compute의 부작용, 기존 public 레거시 필드, 직접 계획의 미평가
규약, 3DOF·무상태 제어기 범위는 현재 계약으로 남는다. 전체 ROS 제거·C++ 지형
이식·새 VTOL 상태/제어기를 완료했다고 해석하지 않는다.
