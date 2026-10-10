# 계획 계산·채택·평가 분리 검증 자료

2026-10-10 후속 계획의 **단계 1·2** 검증이다. 기준은 상위 `0e94aa3`,
계획 서브모듈 `57ac7a8`이며, 같은 `mmp_dev` 컨테이너에서 수정 전 기준 실행을 새로
수행한 뒤 수정 후 결과와 비교했다. 과거 10월 8일 검증을 새 실행으로 대신 표시하지 않는다.

| 검사 | 결과 |
|---|---|
| 수정 전 실제 CTest | 37/37, chain 71/71, risk 154/154 |
| 수정 후 실제 CTest | 39/39, chain 71/71, risk 154/154 |
| 새 독립 evaluator 검사 | 67/67, ROS 없이 production 계산 소스 컴파일·실행 |
| 새 실행 채택 검사 | 62/62, 직접/route/baseline 실제 계산과 저장소·실패·수명 검사 |
| 원본 evaluator 비교 | 152개 입력, 모든 verdict 필드와 로그 문자열 동일 |
| 실제 지형 R2 C++ 호출 | 기존 node/값 설정/초기 환경 생성자 세 방식, 각 기준 수치 동일 |
| 실제 RViz R2 | 최종·참고 궤적 및 전단 경로 동일, 190조각, 5개 병렬 구간 |
| 실제 ABORT·복구 | 기존 ID 1의 ABORT 한 번, 새 ID 2 복구 및 궤적 수치 동일 |
| 설치 target 외부 사용 | evaluator 예제 빌드·실행, 사용 문서 함수 2개 컴파일·링크 |
| 시험 대상 일치 | 소스·설정·테스트 156개와 실행 전후 설치 파일 8개의 해시 유지 |

R2 원본 double 궤적 시간 합은 **521.0625247312521초**, ROS float32 wire 시간 합은
**521.0625259429216초**다. 각각 같은 표현끼리 비교한다. initial-environment 모드는
기존 node 모드와 달리 선행 빈 환경 clear가 없어 환경 번호가 6 대신 4다. 이 이미
존재하던 차이를 별도 검사하며 다른 수치·상태·ID를 무시하지 않는다. 벽시계 시각과
계획 소요 시간은 궤적 수치 동일성의 대상이 아니다.

## 보관 형식과 오프라인 확인

`evidence.tar.gz`는 1,126개의 선택된 원시 로그·JSON·소스 snapshot·비교/실행 스크립트와
실제 RViz 화면을 담는다. `manifest.json`은 압축 파일 및 모든 내부 파일의 SHA256이다.
큰 DEM, ROS 설치 환경, 실행 바이너리와 임시 CMake build tree는 포함하지 않는다.
실제 재실행에는 기존 컨테이너·지형 asset과 그 의존성이 필요하다.

```bash
python3 verify.py
# 해당 단계의 정확한 작업본과도 비교할 때만:
python3 verify.py --source-root /path/to/MMP
```

기본 검사는 **2,493개** 무결성·원본 수치 비교 조건을 확인한다. 새 빌드나 ROS 실행을
대신하지 않는다. 이후 단계에서 코드가 바뀐 작업본에 `--source-root`를 적용하면 소스
차이가 보고되는 것이 정상이다. 압축 파일을 별도 위치에 풀면 다음 자료를 볼 수 있다.

- `baseline/baseline_validation.json`: 새로 확보한 수정 전 결과.
- `after/final_validation.json`: 수정 후 최종 결과와 원시 자료 해시.
- `after/tests/`: CTest 원본 출력·상세 로그·개별 검사 및 71개 chain 로그.
- `after/r2_consumer/`: 설치 target 사용 소비자와 세 방식의 원본 double 결과.
- `after/gui/`: 실제 로드·계획·ABORT·복구 wire 기록과 RViz 화면.
- `evaluator/`: 독립 67개 검사와 원본 대비 152개 비교, 실행 argv·exit·stdout·stderr.
- `after/source/`, `baseline/source/`: 실제 확인한 계산 소스·설정·테스트 snapshot.
- `build_input_validation.log`: 최종 변경 패키지 빌드 로그.

빌드는 `/ws/src/mmp_path_planning/src/path_manager`만 명시적으로 지정했고 기존
의존 패키지 설치를 사용했다. 전체 workspace의 깨끗한 빌드나 다른 머신 검증은 아니다.
최종 빌드 명령은 아래와 같다.

```bash
# 컨테이너 내부 user, /ws
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=4 MAKEFLAGS=-j4 colcon build --symlink-install \
  --base-paths src/mmp_path_planning/src/path_manager --packages-select path_manager \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
```

예비 실행·중단 기록은 최종 결과와 구분해 보존했다. baseline 상세 로그 확보를 위한
재실행, 주석 변경 빌드와 겹친 예비 CTest, evaluator의 잘못된 수치 입력 검사 추가로
중단한 시도, 외부 R2 harness의 generated fixture 헤더 두 개 누락 후 재시도가 포함된다.
최종 CTest·외부 소비자·GUI는 최종 빌드 이후 실행했고 시험 전후 소스와 설치 파일의
일치를 다시 확인했다. 문서의 임무 함수는 컴파일·링크했으며 그 함수 자체를 실행한 것은 아니다.
