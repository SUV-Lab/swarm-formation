# 모듈화 최종 검토 검증 — 2026-10-11

상위 `a297f234`, 계획 서브모듈 `d2ef5496`에서 시작한 최종 검토다.
[항목별 검토 문서](../../design/final_modularization_review_2026_10_11_ko.md)에 실제 호출
관계·수정한 결함·남는 책임을 기록했다. 기존 결과를 다시 인용하는 데 그치지 않고
`mmp_dev` 컨테이너에서 새 빌드·시험·R2·RViz·ABORT 복구를 실행했다.

| 확인 | 결과 |
|---|---|
| 별도 Release build/install | vehicle·optimizer·manager 3개 패키지 성공 |
| 통합 CTest | **48/48**; chain 변형 **73/73**, risk **154/154** 포함 |
| 설치 Python 지도 로더 | **69/69** |
| 설치된 외부 C++ 예제 | 계획·평가·시뮬레이션·표시 **4/4** |
| 외부 C++ R2 전체 결과 | 190조각, double **521.0625247312521초**, 이전 JSON과 byte 단위 동일 |
| 일반 설치본 실제 RViz R2 | 190조각, float32 합 **521.0625259429216초**, 계수·시간·FE 데이터 동일 |
| 환경 ABORT와 새 명령 복구 | ID 1 철회 → ID 2 생성, 복구 수치 동일 |
| 입력/설치물 고정 | 명시한 **421개** 파일의 실행 전후 해시 동일 |

별도 build/install은 세 패키지를 새 디렉터리에서 빌드했으며 나머지 ROS·지도 의존성은
기존 underlay를 사용했다. 모든 패키지를 새 OS에서 재구축했다는 뜻은 아니다. 이후
일반 `/ws/install`에도 optimizer·manager를 다시 빌드하고 해당 경로로 RViz를 실행했다.
실행 중인 노드와 설치 노드의 SHA256도 대조했다. 기존 compiler warning은 남아 있다.

## 발견한 결함과 수정 전후

1. 낮은 마지막 구간 속도 상한이 지정 종단 속도·가속도를 덮어쓸 수 있었다.
   baseline/route의 가속도 전용 조합과 baseline의 지정 속도 조합은 수정 전 실패했다.
   두 성분의 지정 여부를 독립적으로 보존한 후 통과했다.
2. 외부에서 전달한 전환 prefix가 direct fallback에서 누락될 수 있었다.
   prefix 자체로 호출별 보호 상태를 정해 누락을 막고, 다음 일반 요청에 상태가
   남지 않는지 확인했다.
3. 비유한 요청 override가 지속 설정을 오염시켜 다음 정상 요청도 실패했다.
   기존 라이브러리에서 direct/chain 두 경로 모두 재현했다. 두 필드를 적용 전에
   함께 검증한 후 후속 정상 계획의 최종·참고 계수·시간·상태가 원래와 같았다.

종단/prefix의 새 CTest 5개 중 4개는 수정 전에 실패했다. 나머지 route 지정 속도
검사는 이미 맞던 동작의 보존을 확인한다. 두 tail 검사 경로는 예상 구간 수도 검사해
fallback으로 오류가 가려지는 것을 막는다. 최적화 solver 내부와 정상 R2 결과는 유지했다.

## 집계 보정과 검증 범위

chain runner는 소스에서 변형 이름을 자동 수집한다. 새 `tailcap`, `prefixfallback`
두 이름으로 71 → 73개가 되었다. 첫 요약 스크립트만 이전 기대값 71을 사용해 false를
기록했다. **실제 CTest는 처음부터 48/48 통과**했다. 원본 집계
`tests/summary_initial_71_expectation.json`과 원본 로그를 보존하고 소스 변형 이름·
73개 개별 로그·tally를 재대조했다. 보정 스크립트와 이후 summary도 함께 저장했다.

이는 특정 입력과 경계 조건의 회귀 검사다. 일반 임무 전체의 성공률이나 성능 향상을
증명하지 않는다. 이번 검토에서 반복 성능 벤치마크는 새로 수행하지 않았다.
R2 숫자 비교에서 실행 시각 stamp는 제외하며, double 라이브러리 결과와 float32
ROS 메시지는 각각 같은 표현의 기준값과 비교했다.

## 보존 자료와 다시 확인하는 방법

`evidence.tar.gz`에는 281개 파일을 보관했다. `manifest.json`에 각 파일과 archive의
byte 수·SHA256을, `summary.json`에 최종 집계를 저장했다.

- `source_snapshot/`, `reviewed_change.patch`: 검증한 코드·사용 문서 변경 11개.
- `chain_reproductions/`, `repro_*.log`: 수정 전후 경계 조건 재현.
- `tests/`: CTest 원본 로그·결과·73개 chain 로그와 집계 보정 전후.
- `all_tests.xml`, `installed_test_imports.json`: 실제 설치 Python 검사.
- `runtime/`, `baseline/owned_r2.json`: 외부 소비자 소스·실행·원본/현재 전체 결과.
- `gui/`, `baseline/gui/`, `gui_tools/`: RViz DDS 기록·화면·ABORT·비교기.
- 빌드·실행 스크립트, 외부 예제 compile commands, 421개 입력/설치물 전후 해시.

```bash
python3 src/mmp_path_planning/docs/validation/final_review_2026_10_11/verify.py
```

검사기는 archive 무결성, 시험 집계, 수정 전후 실패, R2 계수·시간과 복구 결과를
저장 자료에서 다시 검사한다. **ROS·빌드·실행을 새로 수행하는 명령은 아니다.**
원본 자산·컨테이너·설치 의존성과 실행 당시 `/tmp` 경로가 있어야 실제 하네스를 다시
실행할 수 있다. Python harness는 이전 지도 검증의 `installed_contract.py`를 재사용한다.
대형 DEM과 빌드 바이너리는 archive에 넣지 않았으며 컨테이너의
`/tmp/mmp_final_review_20261011` 및 기존 `/ws/install`에 남아 있다.

`reports`·`experiments` 디렉터리는 읽거나 탐색하지 않았고 stash·기존 미추적 그림·
관련 없는 로컬 자료도 변경하지 않았다. 이번 시험이 만든 ROS 프로세스와 격리 display는
종료했다. 소스 수정과 일반 설치 경로 빌드는 완료했으며 Git commit/push는 수행하지 않았다.
