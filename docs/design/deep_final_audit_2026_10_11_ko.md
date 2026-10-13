# 확장 최종 감사 — 2026-10-11

이 문서는 10월 11일 검증 기록이다. 10월 13일 후속 검토에서 발견한 병합 재시도 설정
우선순위 결함과 현재 검수 상태는 [모듈화 체크포인트](../validation/modularization_checkpoint_2026_10_13/README.md)에 기록한다.

이번 검토는 기존 완료 평가를 그대로 인용하지 않고 현재 작업 코드에서 다시 수행했다.
합의한 라이브러리화 범위는 충족한다. 입력·실패 경계를 더 넓게 시험하면서 기존 구현에도
남아 있던 결함을 발견했고 수정했다. 정상 입력의 알고리즘, 좌표·단위·기본값,
MINCO/L-BFGS 본체, ROS/RViz/Python 실행 구성은 유지한다.

## 검토 방법과 범위

영역별 독립 검토를 병행하고 설정/FSM/환경 변경은 서로 교차 검토했다.

| 영역 | 확인한 호출·소유권·실패 경계 | 실행 검증 |
|---|---|---|
| 계획 API | MissionPlanner → RosPlanningBackend → PM/chain, 초기화·요청·반환·모델/탐색 교체 | 외부 설치 소비자, R2 소유 결과 비교, 공개 API 회귀 |
| 설정·chain | baseline/route/parallel/3-pass, per-call 설정, fallback·merge·prefix·tail | 기존 chain 75종, 새 profile 계약 검사 |
| 환경·지도 | 지형/SDF/risk 세대, AGL, borrowed callback, clear/recrop/defer, Python frame/asset/snapshot/rollback | 환경 owner·mask·risk·지도 검사, 실제 지도 서비스 |
| 실행·ROS | seal/apply, stale/foreign/replay, ID, 실패한 교체, observer 수명, callback group, ABORT·재계획 | FSM 시나리오와 실제 RViz ABORT/새 ID 복구 |
| 평가·시뮬레이션·표시 | 값 소유, SI 변환, phase/policy/junction, RK4·chunk·취소·예외, 메시지 준비/발행 | sanitizer, 독립 수치 대조 4,413개 검사, RViz |
| 설치·외부 사용 | exported target, 의존성, C++17, 공개 헤더 독립 사용 | 설치 예제 4개, 헤더 21개를 각각 외부 TU로 컴파일 |

작업 위치는 `/home/suv/ws/temp/MMP_module/MMP`, 상위 `lgh/dev`, 계획 `mmp_dev`다.
기존 미커밋 작업은 보존했다. `reports`·`experiments` 디렉터리를 읽거나 탐색하지 않았고,
stash를 복원하지 않았다. 빌드는 `mmp_dev` 컨테이너에서 명시한 package base-path만 사용했다.

## 발견하고 수정한 사항

| 문제 | 재현된 결과 | 수정·회귀 검사 |
|---|---|---|
| ROS 명령이 공통 요청 검증을 우회 | 거부된 +Inf clearance가 남아 정상 재전송도 실패, 실행 ID가 1에 머묾 | 두 override를 MissionPlanRequest로 전달. 각각 NaN/±Inf와 다른 정상값 조합, 현재 실행 유지, 같은 sequence 수정 재전송·전체 계수 동일 |
| early fallback의 profile 우선순위 불일치 | 중복 max_vel 0.5→2.0인데 0.5 적용, 비활성 phase도 적용 | worker/fallback이 같은 합성 함수 사용. 목록 마지막 값·phase→segment·활성 구간 규칙 일치 |
| 잘못된 hard profile을 허용 | max_vel=nan에서 제한 보존이라고 DEGRADED 반환, 실제 속도 제한 소실 | 최종 hard max의 유한·양수·전체 문자열 검증, 빈 FAILED 결과. baseline/route/parallel/early와 실패 후 복구, 실제 4→3 merge 검사 |
| 거부된 지형이 상태를 바꾸고 ready 발행 | 짧은 payload가 auto voxel 1→2 변경, NaN 길이를 수락, 누락 elevation에도 ready | 값/ROS 입력의 복사 없는 공동 metadata preflight, 거부 시 상태 유지, 실제 TerrainUpdate를 FSM receipt에 사용 |
| 공개 장애물 setter 검증 누락 | 음수 반지름을 정상 장애물로 수락 | 중심·반지름/크기의 finite/positive 검사. 거부 -1, 큐·revision·장애물·yaw RNG 보존 |
| Python 설정 중간 예외 rollback 누락 | 두 번째 parameter callback 예외 후 첫 mission path가 남음 | set_parameters 호출 전에 rollback 준비. 실제 rclpy 재현과 정식 Python 회귀 |

profile의 오류도 값과 함께 우선순위로 합성한다. 잘못된 phase 값을 더 높은 우선순위의
유효 segment 값이 덮으면 유효하다. 최종 segment 값이 잘못되면 이전 값을 되살리지 않고
거부한다. merge retry는 검증된 원 worker들의 hard 제한을 보수적으로 유지한다.

이는 정상 궤적 계산법을 바꾼 작업이 아니라 재사용 가능한 API가 거부·복구 시 지켜야 할
계약을 보강한 작업이다. 잘못된 입력의 결과는 의도적으로 달라진다. 모든 ROS 숫자
파라미터나 사용자 제공 callback 전체를 검증하는 작업으로 확대하지 않았다.

## 이번에 다시 실행한 검증

| 검사 | 결과 |
|---|---|
| 별도 설치 및 일반 `/ws/install` 빌드 | 통과. terrain·optimizer·manager와 별도 vehicle build |
| Release CTest | **54/54**: path_manager 45, optimizer 3, vehicle 6 |
| chain/risk | **75/75** variant, **154/154** risk 검사 |
| 신규·확장 경계 검사 | profile 198개, FSM 39개, 환경 owner 23개 `[OK]` 기록(설정 준비 확인 포함) |
| 설치된 Python 지도 패키지 | **70/70**, 실제 설치 경로와 현재 소스 일치 확인 |
| 설치 외부 예제/공개 헤더 | **4/4** 실행, **21/21** 독립 TU 컴파일, 소스 include 경로 없이 연결 |
| ASan+UBSan | vehicle 계산/시뮬레이션 **41/41**, 누수 검사 포함 |
| UBSan clean build | path_manager **45/45**, halt_on_error 활성. 외부 optimizer/search 의존 라이브러리까지 모두 계측한 시험은 아님 |
| 독립 계산 대조 | **4,413** 검사. 무작위 2-piece 궤적 100개, fixed-wing 상태 151개. 좌표 이동 최대 위치 잔차 1.19371e-12 m |
| R2 외부 API | **190조각, 521.0625247312521초**, 이번 수정 전후 전체 소유 결과 JSON 바이트 동일 |
| 실제 RViz R2 | 지도 자동 로드·계획 실행, float32 계수/시간 동일(합계 521.0625259429216초) |
| 실제 ABORT·복구 | 철회 확인, 새 ID 계획 복구, 원래 궤적과 계수/시간 동일 |
| 평가/표시 관련 로그 | evaluation 38개·postprocessing 2개·cost profile 10개 일치 |
| chain 출력 수치 | 75종 중 74종 그대로 일치. random yaw를 쓰는 dynprobe는 동일 seed 재시험에서 일치 |
| 환경 lifecycle | 11단계·48,125 지점 조회값 전부 동일. 아래 설명한 잘못된 입력 처리 메타데이터만 의도적으로 수정 |
| 검증 중 입력 식별 | 명시한 소스·설치물·입력 **435개 해시** 변경 없음 |
| solver 보존 | `poly_traj_optimizer.cpp`가 체크아웃 HEAD와 바이트 동일 |

R2 소유 결과 SHA-256은
`33b40dc818378be2343349bd31bff20c309bbb7aa2725d1f00e76b88ce1ebfe5`다.
R2/ABORT 수치 증거와 임무 패널·고도 프로필 스크린샷을 함께 보존했다.

환경 lifecycle 전체 바이너리는 **완전히 같다고 주장하지 않는다**. 유효 지도·recrop·
장애물·위험장·최종 복구의 조회값은 모두 같지만, 0.7 해상도를 수락한 뒤 잘못된 0.5
해상도 메시지를 거부하고 clear한 단계에서 SDF의 X/Y voxel 메타데이터가 0.5 대신
0.7을 유지한다. 수정하려던 거부 원자성 결함의 결과다. 3,405,520바이트에서 해당 double
두 개 외에는 모두 같다. 원본/수정 결과와 byte 범위 검증을 보존했다.

테스트 성공률은 선택한 회귀·경계 검사에서의 결과다. 임의 미션의 성공률 100%나
전수 형식 검증을 뜻하지 않는다. 검수용 ROS/GUI/loader/display 프로세스는 종료했다.

테스트 등록 과정의 초기 오류도 보존했다. 새 profile 시험 본문은 0실패였지만 ament의
결과 XML 생성 옵션 누락으로 첫 CTest가 실패했다. 옵션을 추가하고, 이전 설정으로 시작한
실행을 중단한 뒤 재configure와 전체 시험을 다시 수행했다. 최종 성공 수치는 마지막
완료 실행 기준이다. UBSan은 소스 보완 후 clean rebuild한 설치물을 사용했다.

## 유지되는 사용 계약과 범위

- 초기화한 planner의 `plan(request)` 한 번으로 전체 계획 결과를 얻는다. 지도 준비,
  결과 실행, 시뮬레이션, 시각화까지 하나의 필수 함수에 합친 것은 아니다.
- 결과는 궤적·평가 값을 소유한다. 내부 const view와 환경 callback은 정해진 동기 수명
  동안 빌리며 같은 planner의 계획·갱신·설정 변경은 직렬로 사용한다.
- 기존 ROS backend와 ROS 메시지/파라미터·탐색 의존은 남는다. ROS 없는 배포 SDK나
  Python 지형의 C++ 포팅을 완료했다고 주장하지 않는다.
- 외부 전략은 공개된 finite 값·수명·병렬 질의 규약을 지켜야 한다. parallel slice callback은
  예외를 worker 밖으로 던지지 않는 기존 계약을 유지한다. 임의의 외부 전략 구현까지
  오류 없이 동작한다고 보장하지 않는다.
- 평가는 기존 0.1초 표본 및 구간 접점 규칙이다. 연속 충돌 판정으로 바꾸지 않았다.
- 모든 미션의 성공률, 새 성능 향상, 버그의 절대 부재를 의미하는 검증은 아니다.
  새 성능 벤치마크는 수행하지 않았으며 동시 빌드/시험 시간으로 속도를 비교하지 않았다.
- 기존 경고(미사용 값, 주석, 제한된 진단 문자열 길이 등)는 남는다. 빌드 경고 0이라는
  주장은 하지 않는다. 내부 어댑터 헤더는 호환성을 위해 설치되며 일반 호출에는 필요 없다.

이번 합의 범위에서 추가 구조 재설계를 완료 조건으로 남기지 않는다. 새로운 기능이나
지원할 입력 범위가 정해지면 해당 경계를 확장하면 된다. 변경은 작업 폴더에 보존했고
commit/push는 하지 않았다.

[보존된 원본 검증 자료](../validation/deep_final_audit_2026_10_11/README.md)와
[현재 공개 API](modularized_function_api_ko.md)를 함께 참고한다.
