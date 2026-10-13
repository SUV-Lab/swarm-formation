# 명명 구현(계약 설명 보강 + private 4개) — 협업 세션 Claude의 독립 최종 검토

작성: 협업 세션 Claude (Claude Code), 2026-10-13. **협업 세션의 분석이며 사용자의 전달이 아니다.**
대상: 미커밋 작업 트리. 기준 HEAD는 불변(상위 `13ced330`, 계획 서브모듈 `068f386f`).
방법: 변경 전 제가 직접 찍은 해시(서브모듈 `src/` 추적 파일 159개, 상위 관련 패키지 68개)와 `git diff`로 변경 파일을 전수 확인하고, 주석을 제거한 토큰 비교로 동작 변화를 가린 뒤, 새 주석 하나하나를 코드와 대조했다. 빌드·실행·수정은 하지 않았고 `reports/`·`experiments/`는 열지 않았다. 시험은 재실행하지 않고 Codex의 before/after 산출물을 읽어 대조했다(§7).

## 0. 결론

| 항목 | 결과 |
|---|---|
| 변경 파일 전수 | Codex `changed_files.json`과 정확히 일치: 서브모듈 소스 25 + 문서 2, 상위 헤더 2. 그 외 변경 없음 |
| 행동 변화 | 의도된 것만: 함수명 4개, 미사용 `trial_times` 제거, 디버그 로그 태그 2개. 나머지 25개 파일은 주석을 벗기면 토큰이 동일 |
| 범위 초과 | 없음. public 함수·필드·인자명, 매크로, 기본값, 정책, 테스트 코드 불변 |
| 계약 과장 | 새 주석 약 40개 주장을 코드와 대조, 과장 없음(§4). 표현 보완 권고 2건은 2차에서 처리됨(1건 범위 밖 기록, 1건 문서 추가·코드 대조 완료) |
| 누락 | 감사 README §2.1–2.3·2.5 목록은 모두 반영. 의도적 보존 1건(옛 로그 문구) 기록 |
| after 검증(산출물 대조) | Codex의 `after/summary.json` build passed·fresh checks exit 0, `after/r2/summary.json` pass(numeric_exact·inputs_unchanged), `after/gui` summary·abort·comparison pass, `r2_before_after.json` byte_identical(190조각), `source_scope_check.json` pass(286 중 27 변경, HEAD 불변). 제가 재실행하지는 않았다 |

**판정: 차단 결함 없음. Codex의 after 산출물까지 pass로 확인됐으므로 커밋 가능.** (정적 검토 + 산출물 대조이며, 시험을 제가 재실행한 것은 아니다.)

## 1. 변경 파일 전수(독립 확인)

- 제 기준선 해시와 불일치한 파일 = 서브모듈 25개(`src/path_manager` 헤더 16·소스 4·테스트 1, `src/path_optimizer` 헤더 4) + 상위 2개(`mmp_traj_utils/poly_traj.hpp`, `mmp_vehicle_dynamics/point_mass_eom.hpp`). `git status`의 수정 목록과 같고, Codex 목록과도 같다. 문서는 `docs/design/planning_computation_usage_ko.md`, `planning_library_usage_ko.md` 2건.
- `docs/validation/**` 과거 소스 스냅샷, `reports/`, `experiments/`는 변경 목록에 없다.
- `git diff --check` 통과(양 저장소). 변경량: 서브모듈 27 files, +232/−155; 상위 2 files, +14/−8.

## 2. 행동 변화 검사(주석 제거 토큰 비교)

방법: baseline(`git show 068f386f:…`)에 네 이름의 치환만 적용한 뒤 주석 제거·공백 정규화·문자열 보존으로 작업 트리와 비교.

- 27개 중 **25개 토큰 동일**. 차이는 `replan_fsm.h`(`bool publishExecutionTrajectory(int trial_times = 1)` → `()`)와 `replan_fsm.cpp`(호출 2곳 `(1)`→`()`, 정의, 로그 문자열 2개 `[planFromGlobalTraj]`→`[publishExecutionTrajectory]`)뿐.
- 이전 이름 잔존: 활성 `src/`(테스트·예제 포함) 0건. 보류된 public 심볼 `invalidateStoredTrajectoryForEnvChange`는 별개 심볼로 그대로. 새 이름 출현 27회.
- 로그 태그 소비자: `scripts/`·`tools/`·`docs/` 검색에서 파싱 소비자 없음(과거 실행 로그·감사 문서에만 등장). `reports/`·`experiments/`는 검색 제외라 외부 소비자 부재까지 보장하지는 않는다.
- 따라서 제어 흐름·반환값·시그니처(public)·기본값·ROS 파라미터 키·토픽·메시지 필드는 불변이다. 컴파일 검증은 아니며, after 빌드로 확인해야 한다.

## 3. 개명 4건

| 이름 | 선언 / 정의 / 호출자 | 판정 |
|---|---|---|
| `planFromGlobalTraj(int)` → `publishExecutionTrajectory()` | `replan_fsm.h:106`, `replan_fsm.cpp:518`, 호출 `:352`, `:387`, 태그 `:533`, `:556`, 테스트 주석 `mission_sequence_test.cpp:22` | 적합. 본체 불변, 인자만 제거 |
| `invalidateStoredTrajectory` → `recordExecutionInvalidation` | `segment_chain_planner.h:598`, `.cpp:222`, 호출 7곳(`:506,583,699,2120,3069,3149,3181`), 참조 주석 `replan_fsm.cpp:407-409` | 적합. 한 줄 본체 불변 |
| `planRouteParallel` → `planFromRoute` | `segment_chain_planner.h:365`, `.cpp:1935`, 호출 `:445`, 주석 `h:77-80`, `h:543`, `cpp:443`, `cpp:1517-1520` | 적합. commit→computeOverRoute 순서·`run_parallel` 불변 |
| `planDirect` → `computeDirect` | `ros_planning_backend.h:67-69`, `.cpp:333`, 호출 `:320` | 적합 |

## 4. 새 주석의 계약 과장 검사

각 주장을 현재 코드에서 확인했다. ✓ = 코드가 보장함.

| 파일 | 주장 | 근거 | 판정 |
|---|---|---|---|
| `replan_fsm.h:100-102` | FSM 타이머와 계획 구독이 같은 MutuallyExclusive 그룹; `timer_callback_group_`는 생성만 되고 타이머에 미할당 | `replan_fsm.cpp:202-203, 241, 265, 299, 322`(타이머가 `subscription_callback_group_` 사용); 다른 사용처 없음 | ✓ (감사 목록 밖의 정정이지만 정확) |
| `replan_fsm.h:107-111` | 발행 + reference 선택 발행 + bookkeeping, 계획/retry 없음, bool은 DDS 전달 확인 아님 | `replan_fsm.cpp:518-558` | ✓ |
| `ros_planning_backend.h:67-68` | 실행 저장소 적용 없음; 첫 사용 시 완화 파라미터 선언 가능 | `.cpp:373-376`(declare), 적용은 `plan()` `:216-217` | ✓ |
| `segment_chain_planner.h:19-22` | backend가 PathManager를 통해 실행 효과 적용, FSM은 명령·발행·실행 상태 | `ros_planning_backend.cpp:213-217`, `replan_fsm.cpp:518-558` | ✓ |
| `segment_chain_planner.h:44-53` compute() | 미지정 provenance·비유한 상태 거부; status는 PlanResult; baseline chain의 reference는 최적화 baseline, route chain은 full flight | `.cpp:294-301`, `mission_plan.h:43-45` | ✓ |
| `segment_chain_planner.h:67-69` plan() | compute 후 PathManager 적용(실패 무효화 포함), 발행 안 함 | `.cpp:254-259`, `planning_execution.cpp:46-81` | ✓ |
| `segment_chain_planner.h:77-83` | planOverRoute는 적용, private `planFromRoute`·transition은 미적용 합성 | `.cpp:2497-2526`(seal+apply), `:1935-1943`, `:1931-1933` | ✓ |
| `segment_chain_planner.cpp:222-227` | flag만 기록, 적용은 backend/호환 wrapper | `planning_execution.cpp:79` | ✓ |
| `planning_computation.h:58-59` | `applied`는 실패/무제품 효과 수용도 포함 | `planning_execution.cpp:64-81`(무제품 실패도 `{true,…}`) | ✓ |
| `mission_plan.h:28-31` | `formation_type`은 메타데이터; pattern/size는 성공 계산 뒤 primary optimizer에 전달; 독립 worker 미설정 | `path_manager.cpp:1756`(저장만, 읽는 곳 없음), `ros_planning_backend.cpp:314, 327`, chain 소스에 formation 전달 없음 | ✓ |
| `mission_plan.h:43-46` | 채택 거부·무제품은 FAILED + 문자열, 전용 reason 없음; 수용된 실패는 status 유지 | `ros_planning_backend.cpp:218-233` | ✓ |
| `mission_planner.h:12-16` | 일반 setter는 채택 비행 재검증·철회 없음; FSM 콜백이 수행; 지형 설치가 대기 장애물 flush 시 그 배치만 재검증 | `ros_planning_backend.cpp`에 revalidate/revoke 호출 없음; `path_manager.cpp:2034-2051`(flush) → `:2190-2204`(배치 재검증); FSM `:1237, 1351, 1459` | ✓ |
| `mission_planner.h:28-29` clearTerrain | SDF·설치/대기 장애물 제거, 위험영역 유지 | `ros_planning_backend.cpp:171-175`(빈 GridMap), store `:376-378, :530-534`; store에 zone clear 경로 없음 | ✓ |
| `mission_planner.h:32-33`, `path_manager.h:406-409` | ≥0 ID, −2 대기, −1 거부; 수용이 실행 검증 아님; 장애물 추가/삭제는 재검증 없음, ROS 콜백이 배치 뒤 검사 | store `:544-554, :585-593, :614`; `path_manager.cpp:2075-2091, 2144-2148`; `replan_fsm.cpp:1292-1351` | ✓ |
| `path_manager.h:411-412` box yaw | 생성된 yaw를 SDF 프리미티브와 시각 메시가 공유 | store `:599-616`(`spec.yaw = yaw`, obstacles_ 저장) | ✓ |
| `path_manager.h:377-380` setTerrainData | clear 시 SDF·장애물 제거, 일반 재검증 없음, 대기 배치만 재검사 | 위와 동일 | ✓ |
| `path_manager.h:418-427` revalidateStoredTrajectory | 점유 predicate 샘플링(완전 평가 아님), 세 경우, hit 기록, revoke, 선행 구간 무시, observer 선통지, FSM이 ABORT·WAIT_EXTERNAL_RECOVERY | `path_manager.cpp:2097-2141`(`polylineClear`), `planning_execution.cpp:96-108`, `replan_fsm.cpp:234-237, 936-981`(`exec_control_pub_` 없으면 조기 반환 → "available" 조건부 표현이 정확) | ✓ |
| `path_manager.h:447-449` | 경과 ROS 시각 사용; 모든 setter가 호출하지는 않음 | `path_manager.cpp:2164-2171` | ✓ |
| `path_manager.h:450-452` | 고정 사유 POLICY_UNEVALUATED revoke + viz clear; zone 감지는 호출자 | `path_manager.cpp:2157-2162`, `replan_fsm.cpp:1455-1459` | ✓ |
| `path_manager.h:456-460` setRiskZonesRuntime | 즉시 searcher·primary optimizer rebind, viz 갱신, 재검증·철회 없음 | `path_manager.cpp:2384-2397` (옛 주석 "다음 planGlobalTraj에서 rebind"보다 정확) | ✓ |
| `path_manager.h:789-793` | 절대 planner z; `> -0.5` 활성; 기본 −0.1은 ACTIVE; SDF voxel 아님 | `direct_trajectory_solver.cpp:24`, `route_slice_solver.cpp:27`, `dyn_a_star.cpp:382-383, 1575-1578, 1760-1761`, `poly_traj_optimizer.cpp:655, 837, 3313`; `path_manager.cpp:107-108`; store/SDF에 ground_height 없음 | ✓ |
| `planning_environment.h:14-18, 32-33` | 감소 방향 셀 중심식, u 단위; rows×resolution==length 미검사 | 감사 §2.3 공동 확인식; `planning_environment.cpp:20-38`(유한성·양수·payload 길이만) | ✓ |
| `route_search_backend.h:41-42, 78-80` | sentinel; built-in은 constraints만 소비하고 model 미조회 | `builtin_route_search.cpp`에 `problem.model` 사용 없음 | ✓ |
| `path_manager.cpp:1604-1607` optimizeStage | solveDirectTrajectory/solve 경유; optimizeFromPath는 가상 seam 아님 | `:1635-1650`(`solveDirectTrajectory`), `poly_traj_optimizer.h:685`(nonvirtual) | ✓ |
| `path_manager.cpp:2150-2151, 2175-2179, 2192-2194` | 주석 위치 정정(옛 flush 주석이 riskZoneFingerprint 위에 있던 것), observer 선통지, flush 후 재검증 | 각 함수 본체 | ✓ |
| `flight_evaluation.h:29-35` | evaluated=false 사유(direct 미실행, 잘못된 입력/설정, 제공자 누락); clean 기본값은 증거 아님; 두 predicate는 미평가 시 false | `flight_evaluator.cpp:84, 91-93`, `flight_evaluation.h:73-88`, `mission_plan.h:46-47` | ✓ |
| `flight_evaluator.h:57-61` | trajectory/phases 참조 저장, piece_leg 포인터, evaluateFlight 반환까지 수명 | `flight_evaluator.h:64-80` | ✓ |
| `flight_evaluator.h:114-116` | `zone_hard_t_*`·`runs`·`qmin*`는 authored+standoff 모두, `zone_hard_n/s`만 authored | `flight_evaluator.cpp:305-321`(`if (hard || standoff)` 블록 안, `if (hard)`만 n/s) | ✓ |
| `direct_trajectory_solver.h:23-24, 64-65`, `route_slice_solver.h:36-37, 41-43` | sentinel; OPTIMIZER_FAILED가 NOT_CONFIGURED/MALFORMED_INPUT 포함; ROUTE_TOO_SHORT는 slice adapter의 사전 검사 | `direct_trajectory_solver.cpp:49-62`(그 외는 기본값 OPTIMIZER_FAILED `h:70`), `route_slice_solver.cpp:13-14, 44`, `h:48` | ✓ |
| `terrain_data.h:12-13, 118-120` | origin은 최소 X/Y 모서리; 수면은 true/0/0, getElevation은 −inf | `terrain_data.h:15-16, 44-48, 96, 158-199` | ✓ |
| `transition_phase.h:230-234` | v_lo=max(1.02·min, 1.1·stall), v_hi=0.98·max, stall은 상태 고도의 ρ와 CLmax(1−1e−3); waypoint_eval이 역보정 | `transition_phase.cpp:68, 73, 152-157, 201-205`, `waypoint_eval.cpp:573-577` | ✓ |
| `zone_policy.h:29-33` | true는 생성 기본값, 측정 증거 아님; 제공자가 generation/epoch/shape 검사 | `flight_evaluator.h:68-70`(`valid=false`), `path_manager.cpp:2240, 2287` | ✓ |
| `optimizer_environment.h:38-39` | sentinel 양 plane | `poly_traj_optimizer.cpp:655, 837, 3313` | ✓ |
| `optimizer_request.h:25-30` | head와 waypoints.back()이 경계; path는 내부 seed·길이·도착 방향; 끝점 미치환; 끝점 일치 미검사; 중간 waypoint는 barrier/taper 앵커 | `poly_traj_optimizer.cpp:50, 139, 186, 245-275`(`headState << start_pos…`, `tailState << waypoints.back()…`, clean_path 끝점 대입 없음, waypoints는 `prepareRiskBarrier`·back()만 사용), `optimizer_session.cpp:66-80`(유한성만) | ✓ |
| `poly_traj_optimizer.h:666-669` | 비가상 legacy 입구; MINCO/L-BFGS 한 단위 | `h:685` | ✓ |
| `route_cost_profile.h:12-14` | planner 단위 높이·기울기, SI 미변환 | 바인딩 `path_manager.cpp:620, 1303, 1893`(`getElevationAndGrad`, planner 격자), 소비 `route_cost_profile.cpp:243-245`(`hypot(gx,gy)` 그대로) | ✓ |
| `poly_traj.hpp:12-15, 81-84, 120-121` | T_i 구간 표기; t_abs는 궤적 시작 기준 초(인자명은 유지) | 본체 불변; 인자명 미변경은 범위 준수 | ✓ |
| `point_mass_eom.hpp:12-16` | 호출자 우수 z-up 좌표, heading은 +X→+Y, MMP 축 +X북/+Y서/+Z상, 배율은 회전 아님 | `:22, 75-76`(cos ψ, sin ψ); 축 설명은 감사 §2.2에서 공동 확인 | ✓ |
| 문서 2건 | plan()이 적용까지 수행; 기본 고도 −0.1 활성; clearTerrain 범위; setter 정책; observer→ABORT→WAIT_EXTERNAL_RECOVERY; `traj_` 없음; `evaluated && clean` vs `evaluated && !unflyable()` | 위 근거와 동일; `path_manager.h`에 public `traj_` 없음 | ✓ |

표현 보완 권고(차단 아님):
1. `path_manager.cpp:2178-2186`의 **옛 로그 문구**("the cancel layer is an open decision")는 주석으로만 "legacy, predates the observer"라고 달고 문구 자체는 보존했다. 로그는 출력이라 이번 묶음에서 보존한 판단은 일관되지만, 운용자에게 여전히 틀린 상태 설명이 보인다. Codex 결정: 이번 범위에서는 그대로 두고 최종 기록에 명시한다. 동의하며, 파싱 소비자가 없다고 확인되면 별도 작은 변경 후보로 남긴다.
2. (해결) `mission_planner.h:12-16`의 "ordinary setter calls"는 정확하지만, 수락된 환경 변화와 −2 대기 접수가 **미채택 계산 문맥을 stale로 만드는** 효과는 1차 검토 시점에 어디에도 적혀 있지 않았다. Codex가 헤더·코드는 동결한 채 `planning_library_usage_ko.md` 환경 전달 절에 "수락된 환경 변화는 이전에 계산한 미채택 결과를 stale로 만들어 이후 적용을 거부할 수 있다. 장애물 추가가 `-2`로 대기 접수된 경우도 미채택 계산 문맥을 무효화한다."를 추가했다. 코드 대조: `apply()`는 `environment_generation` 불일치 시 거부하고(`planning_execution.cpp:46-52`), 그 값은 zone 정책 generation + 지형 generation + SDF revision의 합이라 수락된 zone·지형·장애물 변경이 이를 올린다(`planning_environment_store.h:111-112`, `.cpp:57`, `sdf_manager.cpp`의 `++revision_`); −2 경로는 `invalidatePending()`을 직접 부른다(`path_manager.cpp:2080, 2089`; 지형 setter `:386, 392`). 두 문장은 코드가 보장하는 범위 안이다.

## 5. 누락 검사(감사 README 대비)

§2.1(`mission_planner.h:12-13`, `planning_library_usage_ko.md:170`), §2.2 표 10항목, §2.3 표 6항목(sentinel 4곳 포함), §2.5 2항목이 모두 이번 diff에 반영됐다. §6의 계약 확장 후보(`maxVel()`/`RiskSettings::max_velocity` 역할 문서화, stale reason 코드, typed 결과 등)는 설계상 이번 묶음 밖이며 변경되지 않았다(정상).

## 6. 범위 초과 검사

- public 함수·필드·인자 개명 없음: `t_abs`, `route_start_s`, `max_vel`, `zone_hard_*`, `validateFlightPhases`, `astarSearchAndGetSimplePath`, `numAllocatedBlocks`, `controller`, `invalidateStoredTrajectoryForEnvChange` 모두 그대로.
- 매크로·helper·정책·기본값: 토큰 비교로 불변 보장(`-0.1` 기본값, enum 순서, `valid{true}`, `clean{true}` 모두 그대로).
- 테스트: `mission_sequence_test.cpp` 주석 1줄만.
- 감사 목록 밖 정정 1건(`replan_fsm.h` 콜백 그룹 주석)은 코드와 일치하며 무해.

## 7. Codex 검증 산출물 대조(재실행 없음)

변경 전(before):
- `before/gui/summary.json` pass(evaluation 38/38, postprocessing 2/2, cost_profile 10/10, abort numeric_exact); `before/gui/abort.json` pass(동일 빈 zone 재발행 무ABORT, 변경 zone ABORT 정확 identity, 복구 미션 수치 동일); `before/gui/comparison.json` pass(2026-10-10 baseline 대비).
- `before/r2/summary.json` pass=false는 `inputs_unchanged=false`뿐(Codex가 밝힌 주석 편집 겹침; 수치는 exact). 소스 동결 후 `before/r2_frozen/summary.json` pass=true, numeric_exact.

변경 후(after, 2026-10-13 수신):
- `after/summary.json`: build "passed", fresh checks(`ctest_path_manager`, `ctest_path_optimizer`, `ctest_mmp_vehicle_dynamics`) exit 0, pass=true. Codex 보고: CTest 54/54(45+3+6), 설치 외부 예제 4/4, 독립 header 21/21. 개별 로그는 `after/*.log`에 있고 `after/build.log`도 있다.
- `after/r2/summary.json`: numeric_exact·inputs_unchanged·keys_equal·installed_only_compile 모두 true, pass=true. binary sha256은 before와 다르다(`70277b6f…` vs `d6304f90…`) — 이름이 바뀐 심볼이 들어간 새 빌드이므로 당연하며, 수치 동일성은 결과 JSON으로 판정됐다.
- `r2_before_after.json`: API 결과 JSON byte_identical=true, 190조각, duration 521.0625247312521 s.
- `after/gui/summary.json` pass, `after/gui/abort.json` pass(복구 궤적 sha256 `8d87546a…`가 before와 동일), `after/gui/comparison.json` pass(before/after mission.json 10001 샘플, 환경·명령 동일).
- 수치 구분: API 경로 duration 521.0625247312521 s와 GUI 메시지 경로 521.0625259429216 s는 **각자 자기 경로의 전후 비교**이며 두 경로 사이의 완전 동일성 주장이 아니다(Codex가 명시). 두 경로 모두 190조각.
- `source_scope_check.json`: 286개 중 259 불변, 27 변경(21 comments-only, 6 rename), 허용 치환 정규화 후 동일, HEAD 불변, pass=true. 제 독립 검사(§1–2: 27 변경, 토큰 동일 25 + 인자 제거·로그 태그 2)와 일치한다.
- `before/build.log`(1478 bytes, "Summary: 4 packages finished [6.79s]", path_planner stderr 출력 1건)와 `default_install/build.log`("4 packages finished [1min 48s]")를 host에서 확인했다. 1차 검토 시점에는 after 수집물과 함께 복사되기 전이라 보이지 않았던 것이며, "폴더에 없다"는 1차 문장은 정정한다.
- 2차 확인(2026-10-13): 변경된 소스 27개의 현재 sha256이 `source_scope_check.json`의 값과 27/27 일치한다. 검토본 이후 소스는 바뀌지 않았으므로 빌드·시험 반복은 필요 없다.

## 8. 요청·권고(모두 차단 아님)

1. (완료) after 결과는 같은 스키마로 수신해 §7에서 대조했다.
2. 커밋 메시지에 "동작 변경 없음: 주석 제거 토큰 비교 25/27 동일, 차이 2파일은 인자 제거·로그 태그 2개"를 적어 두기를 권한다.
3. §4 권고 (a)는 이번 범위 밖으로 기록, (b)는 문서 추가로 해결됐다(코드 대조 완료).
