# 잔여 모듈화 세 경계 검증 — 2026-10-11

검증 대상은 호출별 baseline/fallback 설정, 환경/실행 저장소 소유권, 내부 접근 제한이다.
작업 기준은 상위 `lgh/dev`의 `a297f234`, 계획 서브모듈 `mmp_dev`의 `d2ef5496` 및
직전 최종 검토에서 수정한 미커밋 상태다. 변경 전 설치 prefix를 보존하고 `mmp_dev`
컨테이너에서 별도 prefix와 일반 `/ws/install`을 명시적 패키지 경로로 빌드했다.

[구현 설명](../../design/remaining_modularization_2026_10_11_ko.md)과
[현재 API](../../design/modularized_function_api_ko.md)를 함께 참고한다.

| 실제 재실행한 검사 | 결과 |
|---|---|
| 별도 Release 설치 / 일반 실행 설치 | 3개 패키지 / optimizer·manager 빌드 통과 |
| 최종 CTest | **53/53**, 기존 48개 등록 보존 |
| chain 변형 / risk 검사 | **75/75**, **154/154** |
| 신규 경계 검사 | direct 설정 50개, execution owner 22개, environment owner 18개 |
| 공개 접근 제한 | raw 실행 슬롯·owner 비공개와 const 뷰 등 정적 검사 9개 |
| Python 설치 로더 | **69/69** |
| 설치 라이브러리 외부 CMake 예제 | **4/4**, source include 직접 지정 없음 |
| 설치 API R2 | 변경 전 전체 결과 JSON 바이트 동일 |
| 실제 RViz R2 / ABORT·복구 | 동일 궤적 생성, 철회 후 새 ID 복구, 복구 수치 동일 |
| 환경 mask·visibility | 8 지형·40 mask·**203,520회 조회** 바이트 동일 |
| terrain/SDF lifecycle | 11 단계·**48,125점 조회** 바이트 동일 |
| 소스·설치물·입력 고정 확인 | **433개** 명시적 파일 전후 해시 동일 |

R2의 소유 double 결과는 190 pieces, 521.0625247312521초이며 전체 결과 SHA256은
`33b40dc818378be2343349bd31bff20c309bbb7aa2725d1f00e76b88ce1ebfe5`다. RViz float32
메시지는 기존 표현과 비교해 521.0625259429216초와 전체 계수를 유지했다. 평가 38개,
후처리 2개, 비용 프로필 10개 기록도 동일하다. wall-clock 시작 stamp는 소유 결과
직렬화 비교 대상이 아니다. 성공 여부·실패 사유를 포함한 검사를 통과했다는 뜻이며,
모든 임무의 성공률을 통계적으로 추정한 수치는 아니다.

## 단계별 결과와 비교 조건

첫 설정 분리 빌드는 통과했고 R2가 같았다. 새 파라미터 감시자가 최초 ROS 기본값
선언까지 설정 변경으로 집계해 초기 CTest 50개 중 일부가 실패했다. 원본 로그를
`stage1_tests/`에 보존했다. 선언 callback 시점에는 아직 키가 등록되지 않는다는
ROS 계약에 맞춰 최초 선언만 구분했고, 이미 선언된 모든 optimization 키 변경과
타입/최종값은 계속 검사한다. 다음 통합 **52/52**, 최종 **53/53**은 전부 통과했다.

73개 기존 chain 변형의 printed 계산 기록은 72개가 동일했다. `dynprobe`의 일부
geodesic 점 개수는 기본값 `manager/dyn_yaw_seed=-1` 때문에 실행마다 다른 장애물
방향에서 측정된 것이었다. 그 차이를 삭제하지 않고 보존했다. 같은 설정 복사본에서
seed만 `20261011`로 고정해 변경 전/최종 설치물을 다시 실행했으며 모든 비교 기록이
동일했다. ROS timestamp와 병렬 로그 순서는 수치 비교에서 정규화했다. 이 printed
비교는 R2 전체 계수 비교나 테스트 내부의 exact coefficient 검사와 구분한다.

새 환경 owner는 최종 설치 archive에 연결한 별도 fixture로 비교했다. terrain clear,
동일 grid의 고도 변경, 재단·해상도 변경, malformed update, 지연 sphere/box 설치,
AGL grounding, 고정 yaw, 세대·거리·gradient·위험장까지 포함한다. 원본 mask 배열
직렬화를 재현하기 위한 임시 검사 헤더만 접근을 열었으며 제품 헤더의 private 경계는
완화하지 않았다.

MINCO/L-BFGS가 들어 있는 `poly_traj_optimizer.cpp`는 checkout과 동일하다. 새 성능
벤치마크는 수행하지 않았다. 정확성 시험에 부수적으로 기록된 시간은 빌드/다른 검사와
겹칠 수 있어 속도 향상의 근거로 사용하지 않는다. 기존 compiler warning은 남아 있다.

## 자료와 재확인

- `evidence.tar.gz`: 원본 로그, 설치 외부 client·fixture 소스, before/after 결과,
  단계별 테스트, GUI 화면, 입력 해시, 최종 변경 소스 snapshot과 실행 스크립트.
- `summary.json`: 종합 결과. archive의 `final_validation.json`과 같다.
- `manifest.json`: archive와 내부 파일의 byte 수·SHA256.
- `verify.py`: 보존 자료의 무결성과 주요 수치 비교 재검사.

```bash
python3 verify.py
```

이 명령은 저장된 자료를 감사하며 ROS/RViz를 새로 실행하거나 재빌드하지 않는다.
실행한 build/install은 컨테이너 `/tmp/mmp_finish_modularization_20261011`에 남아 있고,
자료 보존본은 이 저장소 디렉터리다. 일반 `/ws/install`도 최종 코드로 빌드했다.
직접 띄운 GUI·planner·loader 프로세스와 격리 display는 검증 후 종료했다.

`reports`와 `experiments` 디렉터리는 읽거나 탐색하지 않았고, stash·기존 그림·무관한
파일은 수정하지 않았다. ROS 제거와 Python 지형 로더 C++ 이식은 이번 범위에 없다.
