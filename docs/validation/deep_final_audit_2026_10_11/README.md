# 확장 최종 감사 검증 자료 — 2026-10-11

현재 작업 폴더의 모듈화 경계를 독립 검토하고, 발견한 입력·실패 경계 결함을 수정한 뒤
다시 실행한 자료다. [상세 검토 문서](../../design/deep_final_audit_2026_10_11_ko.md)에
검토 범위, 여섯 결함 그룹, 변경 이유와 유지되는 계약을 정리했다.

`evidence.tar.gz`는 원본 492개 파일(3,410,291바이트), `manifest.json`은 각 파일과
archive의 SHA-256, `summary.json`은 최종 결과 색인이다. `/tmp` 로그가 사라져도 이
archive로 이번 결과의 값·원본·식별 정보를 확인할 수 있다.

## 최종 결과

- Release CTest 54/54: PM 45, optimizer 3, vehicle 6.
- 기존 chain 75/75, risk 154/154, 설치 Python 70/70.
- 설치 외부 예제 4/4, 공개 헤더 독립 TU 21/21.
- vehicle ASan+UBSan 41/41, PM clean UBSan 45/45.
- 독립 계산 대조 4,413개 검사 통과.
- R2 소유 결과 JSON 바이트 동일, 실제 RViz 계수/시간 동일, ABORT·새 ID 복구 동일.
- 검증 중 명시한 입력·소스·설치물 435개 해시 동일.
- 환경 lifecycle 48,125 지점의 조회값 동일. 거부된 지도 해상도의 잘못된 잔류값을
  수정한 clear 단계의 X/Y voxel metadata 두 double만 의도적으로 변경됐다.
- chain 출력 수치는 74/75가 그대로 같고, random yaw를 쓰는 dynprobe는 동일 seed로
  재시험했을 때 같다. 출력 문자열 비교를 모든 궤적 계수의 증명으로 취급하지 않는다.

## archive 색인

| 위치 | 자료 |
|---|---|
| `before_tests/`, `before_runtime/` | 이번 수정 전 새 빌드 기준 시험과 R2 |
| `tests/` | 최종 54개 CTest 원본 로그/XML·발견된 test 이름·75개 variant |
| `runtime/`, `gui/` | 설치 외부 소비자 결과, 실제 RViz 임무·ABORT 기록·스크린샷 |
| `sanitizer/`, `ubsan_tests.log` | vehicle ASan/UBSan 및 PM UBSan 원본 |
| `computation_probe/` | 독립 수치 대조 소스·CMake·최종 설치물 재실행 로그 |
| `environment_lifecycle/` | 동일 입력의 수정 전후 바이너리·offset·예상 차이 확인 |
| `chain_comparison/` | 이전 75종 로그, 고정 yaw 입력/전후 로그와 비교기 |
| `probes/` | 세 검토자의 수정 전 재현과 표적 수정 후 시험 |
| `checkpoints/` | 초기 CTest 등록 옵션 누락, 중단한 이전 설정 실행, 잘못된 임시 argv 기록 |
| `source_snapshot/` | 이번 검증 코드 32개 snapshot. solver 본체 동일성은 별도 JSON |
| `baseline_source/`, `before_worktree.patch` | 이번 검토 시작 시점의 기존 미커밋 작업 보존 기록 |
| `build*.sh`, `run*.py`, `run*.sh` | 이번 실행에 사용한 빌드·검증 명령 |

초기 profile 시험의 계산 본문은 0실패였으나 ament 결과 XML 생성 옵션이 빠져 CTest가
실패했다. 등록을 고친 뒤 재configure와 전체 검사를 다시 수행했다. 요약은 마지막
완료 실행 기준이며 실패 기록을 제거해 성공률을 만든 자료가 아니다.

## 저장 자료 확인

Python 표준 라이브러리만으로 다음을 실행한다. 파일 해시, 테스트 수, R2/ABORT 기록,
환경 차이 범위, 외부 include 경로, 현재 source snapshot까지 확인한다.

```bash
python3 src/mmp_path_planning/docs/validation/deep_final_audit_2026_10_11/verify.py \
  --workspace /home/suv/ws/temp/MMP_module/MMP
```

`--workspace`를 생략하면 archive 자체를 검사한다. 이 명령은 **저장된 검증 자료 확인**이며
새 빌드나 ROS/RViz 시험을 수행하지 않는다. 새로운 실행은 archive의 빌드·시험 스크립트를
참고해 현재 컨테이너/경로에 맞춰 수행해야 한다. 빌드는 명시된 `--base-paths`를 유지한다.

ROS 제거·C++ 지형 이식, 임의 mission의 100% 성공, 모든 외부 전략의 correctness,
새 성능 향상을 주장하지 않는다. MINCO/L-BFGS 내부는 변경하지 않았다.
