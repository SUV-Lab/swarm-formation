# 계약 설명·내부 이름 정리와 최종 검토 — 2026-10-13

사용자가 승인한 **계약 설명 보강과 private 함수 네 개 정리**를 완료했다.
공개 API·필드·인자 이름, 기본값, 계산식, 상태 전이, ROS 파라미터·토픽·메시지는 유지했다.
검증은 상위 `lgh/dev`와 계획 서브모듈 `mmp_dev`의 작업 트리에서 수행했다.
사용자의 후속 승인에 따라 소스와 설계·검토·검증 자료를 함께 커밋해 보존한다.
검증 시작 시 기준 HEAD는 각각 `13ced33052fcbe7580b96e79d4f44574fefb6f6d`,
`068f386f7a3eadd21ab1bd9308727e6eca7ba641`이다.

## 적용 내용

| 변경 전 private 함수 | 적용한 이름 | 실제 책임 |
|---|---|---|
| `ReplanFSM::planFromGlobalTraj(int)` | `publishExecutionTrajectory()` | 채택 궤적 발행·FSM bookkeeping. 쓰지 않던 `trial_times` 제거 |
| `SegmentChainPlanner::invalidateStoredTrajectory` | `recordExecutionInvalidation` | 계산 결과에 무효화 의도만 기록 |
| `SegmentChainPlanner::planRouteParallel` | `planFromRoute` | 순차·병렬 모두 지원하는 경로 기반 계획 조정 |
| `RosPlanningBackend::planDirect` | `computeDirect` | 실행 채택 전 직접 계획 계산 |

선언·정의·모든 호출자·직접 설명을 함께 수정했다. 함수명 디버그 로그 태그 두 곳만
새 이름으로 바꿨고, 기존 상태/실패 로그와 ROS 출력 계약은 유지했다.

계약 설명은 현재 구현을 기준으로 다음을 정리했다.

- 공개 환경 setter와 ROS 콜백의 실행 재검증 책임, deferred obstacle 예외,
  미채택 계산의 stale 처리, `clearTerrain()`의 SDF·장애물 제거 범위.
- 지형 격자의 감소 방향·단위, 고도 plane의 `> -0.5` 활성 규칙과 실제 기본값,
  수면 gradient 질의의 반환값, planner 축과 SI 배율 변환의 차이.
- 미평가 verdict, 평가 입력의 빌린 참조 수명, 실행 효과 적용과 궤적 성공의 차이,
  실패 사유가 합쳐지는 범위, optimizer의 path·경계·waypoint 역할.
- backend의 compute→apply와 FSM 발행, 조건부 ABORT·외부 복구 대기,
  생성된 장애물 yaw, 전환 속도창, formation 설정의 기존 적용 시점.
- 제거된 `PathManager::traj_` 설명, legacy optimizer 경계, 궤적 시작 기준 상대초.

`MissionPlanner::plan`과 `PolyTrajOptimizer::solve` 등 공개 입구는 그대로다.
공개 개명·필드 이전·매크로 정리·새 helper·환경 실행 정책 확장은 이번에 적용하지 않았다.
MINCO/L-BFGS 본체, ROS 연결, Python 지형 로더도 유지했다.

## 변경 파일 전수 검토

286개 소스·설정·시험 파일의 시작 지문을 보존했다. 그중 변경 소스는 27개로,
21개는 주석만, 6개는 위 private 이름의 선언·정의·호출자 변경이다.
주석을 제외하고 네 이름·미사용 인자·함수명 로그 태그만 정규화하면 27개 모두
기준 코드와 토큰이 같다. 검증 기간에는 나머지 259개 지문과 두 저장소 HEAD가 같았다.

계약 주석을 두 담당자가 나눠 구현과 대조했고, 별도 검토자가 모든 개명 호출자를
확인했다. Claude는 자체 기준 지문으로 변경 파일 전체를 독립 확인하고 새 설명을
현재 코드와 대조했다. 발견한 채택 주체·public/private 경계·실패 사유 표현은
수정 후 다시 확인했다. **검토 범위에서 남은 차단 결함은 없다.**

- [개명 독립 검토 원문](rename_review.md)
- [Claude 최종 검토 원문](claude_final_review.md)
- [현재 라이브러리 사용법](../../design/planning_library_usage_ko.md)
- [계산·실행 적용 사용법](../../design/planning_computation_usage_ko.md)

기존 `revalidateAfterEnvChange`의 로그에 남은 “cancel layer is an open decision”은
출력 변경 범위를 제한하기 위해 보존했다. 근접 주석은 현재 observer/ABORT 동작으로
정정했다. 이 옛 로그 문구를 근거로 취소·ABORT가 미구현이라고 해석하면 안 된다.

## 새로 실행한 검증

`mmp_dev` 컨테이너에서 현재 소스를 빌드했다. 별도 Release build/install로 전후를
비교했고, 완료 후 README 실행에서 사용하는 기본 `/ws/install`도 갱신했다.

| 항목 | 결과 |
|---|---|
| 변경 전·후 관련 4개 패키지 증분 빌드 | 통과 |
| 변경 전 계획 CTest | 45/45 |
| 변경 후 CTest | 54/54: 계획 45, optimizer 3, vehicle 6 |
| CTest에 포함된 chain / risk 검사 | 75/75 / 154/154; 순차·병렬·3-pass 관련 기존 회귀 포함 |
| 설치 target만 사용하는 외부 예제 | 4/4, 저장소 source include 없이 빌드·실행 |
| 공개 헤더 개별 translation unit 컴파일 | 21/21 |
| 외부 API R2 | 190조각, 521.0625247312521초. 결과 JSON 바이트 동일 |
| 실제 RViz R2 | 190조각, 521.0625259429216초. 메시지 계수·시간, 환경·명령 및 평가 비교 통과 |
| 실제 ABORT·복구 | 동일 환경 재발행은 ABORT 없음, 변경 환경은 정확한 ID로 1회 ABORT, 새 ID 복구의 계수·시간 동일 |
| 기본 `/ws/install` 빌드 | 4개 패키지 통과, ROS 환경 설정 후 누락 동적 의존성 없음 |

API와 RViz는 **각 호출 경로 안에서 동일 입력의 변경 전후를 비교**했다.
두 경로의 입력 구성이나 결과가 서로 완전히 같다는 검사는 아니다.
RViz에서는 README 순서대로 Start, R2 mission YAML, Run mission을 사용했고 Python
loader의 회랑 로드와 표시를 확인했다. 실행 중 node의 파일 해시도 설치물과 대조했다.
검증용 GUI·ROS 프로세스는 종료했다.

최초 변경 전 API 시험은 궤적 수치는 같았지만 주석 편집과 겹쳐 소스 불변 검사가
실패했다. 해당 원본은 `before/r2`에 남겼다. 소스를 동결하고 변경 전 바이너리로
다시 실행한 `before/r2_frozen`은 입력·소스 불변과 수치 비교가 모두 통과했다.
처음 GUI 실행의 출력 폴더 권한 오류도 기록으로 남기고, 검증 폴더 권한만 바로잡아
정상 실행했다. 어느 경우에도 애플리케이션 동작을 바꾸어 검사를 통과시키지 않았다.

이번 검증은 증분 빌드와 기존 회귀 시험이다. 새 sanitizer·Python 전체 시험·성능
벤치마크를 실행하지 않았으며, 임의 임무의 성공률이나 성능 향상을 주장하지 않는다.
`reports/`와 `experiments/`는 읽거나 수정하지 않았고 stash도 복원하지 않았다.

## 보존 자료

`evidence.tar.gz`에는 전후 빌드·시험·GUI 로그와 캡처, R2 결과, 검증 스크립트,
변경 전후 소스 사본·지문·patch 및 독립 검토를 보존한다. 바이너리·object는 제외했다.
`manifest.json`은 각 파일과 archive의 해시·크기, `summary.json`은 결과 색인이다.
원본 기록의 `committed: false`와 HEAD는 커밋 전 검증 시점의 상태를 보존한 값이다.

```bash
python3 src/mmp_path_planning/docs/validation/api_naming_2026_10_13/verify.py \
  --workspace /home/suv/ws/temp/MMP_module/MMP
```

이 명령은 저장 자료와 선택적으로 현재 소스 지문을 확인한다. 새 빌드·ROS 실행은
하지 않는다. archive의 실행 스크립트는 당시 환경과 경로를 기록한 것이므로 재실행할
때 현재 컨테이너·설치·지형 경로를 확인해야 한다.
