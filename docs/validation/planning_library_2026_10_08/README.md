# 계획 라이브러리 검증 자료

ROS 실행을 유지한 계획 라이브러리 모듈화의 검증 결과와 원본 로그를 보존한다. 2026년 10월 7~8일의 기존 시험 자료와, 10월 8일 현재 소스를 기록하고 다시 빌드·실행한 자료를 구분한다. 원본 `/tmp` 자료를 지우거나 수정하지 않았으며, 이 디렉터리는 원본 중 아래 검증에 필요한 파일만 복사한 것이다.

현재 소스에 대한 기준은 [새 검증 요약](refresh/refresh_summary.json)이다. 147개 소스·설정·테스트 파일을 빌드 전후에 기록했고, 변경 없이 관련 3개 패키지를 다시 빌드했다. 등록 CTest 37개, chain 71개 경우, risk 154개 검사가 통과했다. 새 바이너리로 외부 C++ R2 임무 세 방식과 실제 RViz 임무·ABORT·복구를 다시 실행했으며, 기존 대응 입력의 궤적 계수·시간이 동일했다. 실행 중인 GUI planner의 SHA256도 빌드 후 설치 바이너리와 일치했다.

## 오프라인 검증

Python 3 표준 라이브러리만 사용하며 ROS, NumPy, Docker는 필요 없다. 실행 위치와 관계없이 다음 명령을 사용할 수 있다.

```bash
python3 /path/to/planning_library_2026_10_08/verify.py
```

현재 체크아웃이 새 검증 당시 소스와 일치하는지도 검사하려면 **상위 MMP 저장소 경로**를 지정한다.

```bash
python3 /path/to/planning_library_2026_10_08/verify.py --source-root /path/to/MMP
```

[검증 스크립트](verify.py)는 다음을 직접 확인한다.

- 보존 파일의 추가·누락·SHA256, gzip 해제 후 원본 바이트의 SHA256.
- 기존 최종 검증 색인이 참조한 4개 파일과 새 검증 요약이 참조한 파일의 해시.
- CTest 발견 목록·실행 수·원본 통과 로그, chain 71개 로그·집계와 risk 154개 원본 결과.
- 최종·참고 궤적의 전체 계수와 구간 시간, 직접 계획·기본 계획 결과, 외부 모델·탐색 소비자 검사 수.
- GUI 입력·환경·전단 경로와 궤적의 수치 동일성, ABORT action·reason·identity 및 새 ID 복구 궤적.
- 평가 38줄, 경로 후처리 2줄, 비용 프로필 10줄의 원본 내용 동일성. 병렬 비용 로그는 순서 대신 내용별 개수로 비교한다.
- 새 빌드 전후 소스 동일성과 실제 GUI 바이너리의 설치 바이너리 일치.
- `--source-root` 사용 시 명시적 허용 경로 안의 파일 추가·삭제·해시 변경. 금지된 `reports`와 `experiments` 디렉터리는 순회하지 않는다.

이 명령은 **저장된 증거를 검사**한다. C++ 빌드나 ROS 실행을 새로 수행하지 않는다. 전체 계수와 구간 시간의 정확한 일치를 다시 검사하므로, 별도의 허용오차나 표본화만으로 수치 차이를 숨기지 않는다. 기존 NumPy 기반 10,001시점 비교 스크립트와 결과도 별도로 보존했다.

## 자료 구성과 시점

| 경로 | 의미 |
|---|---|
| [refresh](refresh/refresh_summary.json) | 2026-10-08 06:54~06:59 UTC에 현재 소스·설치 입력을 기록하고 3개 패키지를 증분 빌드한 뒤 재검증한 자료. KST 15:54~15:59 |
| [historical/ext_c](historical/ext_c/final_validation.json) | 2026-10-08 확장 C 당시 최종 결과. 원래 A/B/C 종합 색인과 그 참조 4개 파일을 바이트 그대로 보존 |
| [historical/pre_a](historical/pre_a/baseline_validation.json) | VehicleModel·RouteSearch·일반 설정 입력 확장 전 기준. 당시 소스 목록·설치물 해시·입력 설정·직접/기본/R2 결과 |
| [historical/pre_extensions_final](historical/pre_extensions_final/final_validation.json) | 그 이전 함수 분리 단계의 최종 비교 색인. 최초 기준과의 비교 관계를 남김 |
| [historical/original](historical/original/baseline_mission.json.gz) | 최초 변경 전 GUI R2 궤적, ABORT·복구, planner 로그와 설정 |
| [historical/inputs](historical/inputs/r2_ridge_crossing.yaml) | R2 미션 YAML. 보존 시 현재 파일의 해시가 과거 결과에 기록된 미션 SHA256과 같음을 확인 |
| [historical/harnesses](historical/harnesses/rviz_harness.py) | 당시 GUI 자동화, ROS 수집·ABORT probe, 수치·전단 비교 스크립트 |

원본 결과 안의 `/tmp/...`, `/ws/...` 절대 경로는 당시 실행 이력을 보존하기 위해 바꾸지 않았다. 모든 보존 파일의 **현재 상대 경로, 원래 위치, 원본/보존물 크기와 SHA256, 인코딩**은 [manifest.json](manifest.json)에 연결돼 있다. 원본 스크립트도 경로를 당시 값으로 유지했으므로 다른 환경에서 그대로 실행하는 재현 도구는 아니다. 위 `verify.py`는 상대 경로로 동작하는 별도 오프라인 검증기다.

일부 큰 JSON과 로그는 표준 gzip으로 압축했으며 내용은 변경하지 않았다. 예를 들어 `python3 -c 'import gzip; print(gzip.open("historical/ext_c/r2_consumer/result.json.gz", "rt").read())'`로 원본을 읽을 수 있다. manifest는 스스로를 제외한 모든 보존물과 README·검증기 자체를 해시해 파일 무결성을 점검한다.

## 수치 비교의 정확한 범위

R2는 같은 임무라도 **C++ 결과의 double 값과 ROS 메시지의 float32 직렬화 값**을 구분한다.

| 비교 대상 | 다항식 조각 수 | 시간 합 |
|---|---:|---:|
| GUI가 발행한 PolyTraj 메시지 | 190 | 521.0625259429216 s |
| 외부 C++ API가 소유한 원래 double 결과 | 190 | 521.0625247312521 s |

각 값은 **같은 표현의 변경 전후 결과끼리** 정확히 같다. 두 표현이 서로 같은 실수 배열이라는 뜻은 아니다. GUI의 float32 변환 경계를 유지했기 때문에 생기는 차이다. 표의 시간은 궤적의 시간 길이(비행시간)이며 계산 소요시간이 아니다.

외부 R2 호출의 `bound_node`, `value_input` 방식은 기존 double 결과 전체가 동일하다. `initial_environment`는 생성자에 실제 지형을 바로 넣어 선행 빈 환경 초기화가 생략되므로 `environment_generation`만 기준 6에서 4로 달라진다. 검증기는 이 값이 정확히 6→4인지 먼저 확인한 후 해당 필드만 정규화해 나머지 전체 결과가 동일한지 검사한다. 상태·기체/궤적 ID·궤적·평가·단계별 시간은 그대로 비교한다.

ABORT 자료는 동일한 빈 위험영역 재발행에는 취소가 없고, 변경된 위험영역에는 현재 궤적 ID로 ABORT가 한 번 발생하며, 위험영역을 비운 뒤 같은 미션을 다시 주면 새 ID로 같은 수치 궤적을 생성하는 관측을 포함한다. 모든 가능한 취소·환경 변경 시나리오의 성공률을 나타내지는 않는다.

## 소스와 재현 범위

확장 C의 기존 자료에는 **최종 작업 트리 전체를 식별하는 소스 manifest가 없었다**. 확장 전 `pre_a/source_manifest.json`을 확장 C의 소스 증명으로 사용하지 않는다. 기존 GUI 실행 바이너리는 새 빌드 전 설치물과 해시가 같았지만, 새 증분 빌드에서 manager 바이너리와 archive가 다시 생성되어 해시가 달라졌다. 이 때문에 현재 소스를 새로 기록한 후 CTest와 실제 R2·GUI·ABORT를 다시 실행했다. [역사적 식별 한계](refresh/historical_identity.json)는 그 판단 시점의 원문이며, 재실행 완료 여부는 [새 요약](refresh/refresh_summary.json)의 `historical_identity.fresh_runtime_rerun_resolution`과 `fresh_gui`를 본다.

새 source manifest는 명시한 147개 파일 범위만 식별한다. 전체 저장소, 운영체제, 컴파일러의 모든 파일, 지형 데이터의 완전한 스냅샷을 의미하지 않는다. [빌드 환경](refresh/build_environment.json), [설치 헤더·설정 비교](refresh/installed_input_comparison.json), [설치 바이너리 해시](refresh/installed_manifest_after.json)를 함께 남겼다. `path_planner`를 포함한 기존 의존성은 설치물을 사용했으며, 이번 새 검증은 관련 3개 패키지의 **증분 빌드**이다. 깨끗한 환경에서의 전체 빌드나 다른 머신의 비트 재현성은 시험하지 않았다.

보존 용량을 제한하기 위해 바이너리, 정적 라이브러리 본체, DEM, 빌드 디렉터리 전체, 모든 중간 A/B 실패 시도는 포함하지 않았다. A/B 단계의 요약은 기존 종합 색인에 있지만 그 단계의 모든 원본 로그를 이 폴더가 독립적으로 입증하지는 않는다. 기존 소스 manifest가 열거하는 과거 소스 원문도 전부 보관한 것은 아니다. 현재 새 검증과 최종 C 수치 회귀의 핵심 원본 증거는 포함한다.

향후 소스를 바꾸면 이 디렉터리의 과거 결과를 덮어쓰지 말고 새로운 검증 디렉터리를 만든다. 현재 API 사용 계약은 [계획 라이브러리 사용법](../../design/planning_library_usage_ko.md)을 기준으로 한다.
