# 내부 조정·설정·지형 계산 분리 검증 — 2026-10-11

기준은 상위 `73d0a69`, 계획 서브모듈 `b14dd9d`다. `mmp_dev` 컨테이너에서 현재
checkout을 사용해 두 단계로 구현·검증했다. stash와 작업과 무관한 파일은 수정하지
않았으며 `reports`, `experiments` 디렉터리는 읽거나 탐색하지 않았다. 패키지 빌드는
항상 명시적 `--base-paths`로 optimizer와 PM만 선택했다.

[구현 기록](../../design/incremental_modularization_ko.md)과
[현재 API](../../design/modularized_function_api_ko.md)에 분리 경계·호출 규약을 기록했다.
MINCO/L-BFGS 본체는 유지하고 chain 호출별 상태·worker 설정·terrain-risk 계산을 정리했다.

| 검사 | 결과 |
|---|---|
| 첫 checkpoint / 최종 CTest | 42/42 → **43/43** |
| 기존 chain 변형 / risk 검사 | **71/71**, **154/154** |
| 지형 위험 계산 검사 | ROS 없이 34/34; 최종 target CTest도 통과 |
| 원본/최종 terrain fixture | 8사례·40mask·203,520질의·12,053,232 bytes 동일 |
| 기존 chain 출력 수치 | 71변형에서 5,160개 printed 진단 동일 |
| 설치된 외부 R2 consumer | 원본/최종 각각 예열 1 + 측정 5, 전체 결과 동일 |
| 실제 RViz | 원본·중간·최종 R2 생성, ABORT 후 새 명령 복구 통과 |
| 최종 소스·설치물 검증 | header/CMake export/archive/node 포함 421개 해시 유지 |

R2는 190개 polynomial piece, double 시간 합 521.0625247312521초다. 결과 전체 SHA256은
`33b40dc818378be2343349bd31bff20c309bbb7aa2725d1f00e76b88ce1ebfe5`이며 wall-clock
start stamp는 이 직렬화 schema에 포함하지 않는다. RViz 메시지는 별도 float32 표현과
비교해 합 521.0625259429216초·계수를 유지했다. 평가 38개·후처리 2개·비용 프로필
10개 비교도 통과했다. 실패한 임무까지 성공했다고 합산한 “일반 임무 성공률”은 아니다.

성능 측정은 지형 준비가 끝난 fresh client에서 `MissionPlanner::plan()`만 계측한다.
각 원본/최종 5개 측정의 시간 중앙값은 **0.762542 → 0.754433초**이고 범위는 각각
0.737676–0.803406, 0.749823–0.796042초다. process lifetime peak RSS 중앙값은
**713552 → 713804 KiB**다. 시간·CPU·메모리 범위가 겹치며 이 R2 표본에서 유의미한
성능 저하 징후가 없다. 일반 속도 향상을 입증하지는 않는다.

RSS는 지도/초기화 상태를 포함한 클라이언트 최대치이고 Python loader는 제외한다.
계획만의 추가 할당량이나 전체 시스템 메모리가 아니다. `/proc` RSS와 `getrusage`
카운터는 갱신 시점·근사 방식이 달라 별도로 보존한다. CPU를 고정하거나 전체 머신을
예약하지 않았고, 최종 타이밍 중 빌드/CTest/GUI를 겹치지 않았다.

비어 있지 않은 위험장 fixture도 원본/최종 교차 5회 측정했다. 전체 fixture 시간은
중앙값 0.135456 → 0.136463초(+0.74%, 약 1ms). PM 초기화·mask 생성·질의·파일 기록·
종료를 포함하며 hot loop만의 성능 검사는 아니다. 준비 실행 포함 12회 모두 수치가 같다.

## 보존 자료

`evidence.tar.gz`는 검증 소스·allowlist·입력 해시·원본 로그·R2 결과·GUI 화면·terrain
원시 결과를 담는다. 원본/최종 실행 바이너리와 큰 static archive는 저장소에 중복하지
않았으며 실행한 컨테이너의 `/tmp/mmp_internal_cleanup_20261011`에 남아 있다.

- `baseline/`: 원본 설치물 동일성, 외부 측정 client 소스·6회 결과·실제 GUI.
- `checkpoint1/`: chain/worker 분리 뒤 빌드·42 tests·R2·GUI.
- `after/`: 최종 빌드·43 tests·6회 결과·성능/메모리 비교·GUI·421개 해시·설계 검수.
- `environment/`: 동일 fixture 소스·원본/최종 binary 결과·교차 시간·해시.
- `final_validation.json`: 종합 색인. 바깥 `summary.json`과 같다.

`manifest.json`에 archive와 내부 787개 파일의 byte 수·SHA256을 저장했다. 보존 자료를
다시 확인하려면 이 디렉터리에서 다음을 실행한다.

```bash
python3 verify.py
```

이 명령은 저장된 자료의 무결성·수치 동일성을 확인하며 ROS/RViz나 성능 시험을 새로
실행하지 않는다. 컨테이너의 실제 재실행 절차와 compile 명령은 archive의 스크립트와
로그에 보존했다. 증분 Release 빌드였고 기존 warning은 남아 있다.

## 남는 구조

순차 baseline / single-shot fallback의 전체 파라미터 적용·복구, PM의 환경/SDF/실행
소유는 유지했다. clearance가 FE·optimizer·코너 검사·표시·최종 평가에 함께 사용되므로
worker용 API로 대체할 수 없다. 후속 변경은 호출별 direct-plan 설정과 위험장 조회를
전체 경로에 전달하고 종료/예외 뒤 mission 조회 복원을 검증해야 한다. 이번 작업으로
모든 내부 상태 결합이나 ROS 의존성이 제거됐다는 의미는 아니다.
