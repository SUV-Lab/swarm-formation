# Supporting libraries: naming and API audit

Workspace: `/home/suv/ws/temp/MMP_module/MMP`
Date: 2026-10-12
Reviewer: `deep_computation_exports`

지원 라이브러리는 **공개 API 대부분 KEEP**, 실제 의미가 어긋나는 이름 2개와 전역 이름 노출을 좁게 정리하는 것이 적절합니다. 클래스·namespace·ROS 파라미터를 일괄 개명할 근거는 찾지 못했습니다.

아래 경로는 workspace 기준입니다. 정적 코드 검토이며 파일 수정·빌드·테스트·checkout·stash 작업은 하지 않았습니다. 이 문서는 앞서 전달한 감사 결과의 보존본이며 추가 조사 결과를 포함하지 않습니다.

| 분류 | 대상·근거 | 구체적인 오용 가능성 | 제한된 조치·호환성 |
|---|---|---|---|
| **RENAME / 구현으로 숨김** | `src/mmp_path_planning/src/path_optimizer/include/path_optimizer/poly_traj_optimizer.h:14` 전역 `LogManager` 별칭, `:23`, `:35`, `:42` 전역 `LOG_INFO/WARN/ERROR` 매크로. `path_planner/include/path_planner/dyn_a_star.h:18` 전역 `inf`. | optimizer 헤더를 포함하기만 해도 외부 프로그램의 로그 매크로·타입·상수 이름과 충돌한다. 특히 일반적인 `LOG_INFO`를 재정의하며, 이 매크로는 외부에 없는 `node_`, `log_manager_` 같은 멤버를 참조한다. | 로그 매크로는 `.cpp` 또는 비공개 헤더로 이동하고 필요하면 `MMP_OPT_*` 접두어 사용. 불필요한 전역 별칭은 제거하고 `inf`는 search namespace/detail로 제한. 공개 solver 함수·설정·ROS 연결은 유지. 의도하지 않은 전역 이름을 이용하던 외부 소스에는 영향이 있으므로 호환 별칭을 무조건 남기는 방식은 피한다. |
| **RENAME** | `path_planner/include/path_planner/dyn_a_star.h:960` `astarSearchAndGetSimplePath()`. 실제 `path_planner/src/dyn_a_star.cpp:629`, `:652`에서 `FrontEnd::FM2`도 실행한다. 생산 호출은 `path_manager/src/builtin_route_search.cpp:126`. | 이름을 보고 항상 A*를 실행한다고 판단하거나, FM2 교체 작업에서 별도 구현이 필요하다고 오해한다. | `searchAndSimplifyPath()` 같은 알고리즘 중립 이름 추가, 기존 이름은 forwarding wrapper 유지, 내부 adapter만 새 이름 사용. **`AstarSearch()`는 실제 A* 본체이므로 KEEP**. 파일명·설정 키 `manager/front_end`·3-pass 동작까지 변경할 필요 없음. |
| **RENAME** | `path_planner/include/path_planner/sdf/sdf_manager.h:144` `numAllocatedBlocks()`. 구현 `path_planner/src/sdf/sdf_manager.cpp:863`은 `distance_cache.size()` 반환. 현재 로그도 `path_manager/src/planning_environment_store.cpp:250`에서 `blocks=` 출력. | 반환값을 블록 개수로 해석해 메모리 계산을 잘못하거나, all-free 표현의 반환값 0을 지도 없음으로 오해한다. | `cachedVoxelCount()` 또는 `storedVoxelCount()` 추가, 기존 함수는 호환 wrapper. 로그 단어도 대응. 지도 전체 voxel 수와도 다르므로 `voxelCount()`처럼 애매한 이름은 피한다. C++ 호출자·로그 문자열만 관련되고 ROS 메시지/설정 영향 없음. |
| **DOC_ONLY** | `path_planner/include/path_planner/sdf/distance_field.h:22`, `:32`; `path_planner/include/path_planner/dyn_a_star.h:785`, `:796`. | 교체 provider 작성자가 거리와 위치를 metres로 전달하거나 unsigned distance를 구현할 수 있다. 검색의 borrowed SDF/zone vector를 임시 객체로 연결할 수도 있다. 인터페이스 이름만으로는 단위·부호·수명 규약이 충분하지 않다. | `IDistanceField`에 planner-frame 입력/거리, signed distance 부호, 미관측 값, gradient 좌표 미분, false 반환과 출력 규약을 명시. `PathSearcher` binding에는 수명·직렬 갱신·검색 중 안정성 명시. 현 구현 `sdf_manager.cpp:813`은 실패 출력 `+inf/zero` 초기화. 검색 `dyn_a_star.h:408`은 nonfinite 거리를 blocked로 취급하므로 provider가 “멀리 떨어진 free”를 임의로 `+inf`로 표현하지 않도록 설명. 기존 의미를 문서화하며 동작 변경은 금지. |
| **DOC_ONLY** | `src/mmp_vehicle_dynamics/include/mmp_vehicle_dynamics/point_mass_eom.hpp:12`의 `mission ENU`. 실제 `src/trajectory_simulation.cpp:80`은 축 회전 없이 성분별 scale만 적용. `src/mmp_terrain/mmp_terrain/frame.py:24`는 +X north, +Y west. | 외부 follower가 ENU로 한 번 더 회전하거나, MMP heading 0을 east 방향으로 해석해 경로가 90도 어긋난다. | “선택한 수평 직교 world frame, +Z up, heading은 +X에서 +Y 방향”으로 기술하고 기존 MMP 연결은 planner 축을 유지하는 SI 값임을 명시. 수식은 일반 수평 회전에서 유효하므로 **좌표 계산을 바꿀 문제가 아니다**. 공개 타입/토픽/계수 그대로 유지. |
| **DOC_ONLY** | `path_optimizer/include/path_optimizer/poly_traj_optimizer.h:660`의 `optimizeFromPath()` 설명: “single replaceable seam”, custom backend가 이를 reimplement한다는 문구. 새 공개 경계는 `:532`의 `solve()`. `optimizeFromPath()`는 nonvirtual. | 외부 개발자가 상속하여 이 메서드만 대체하면 `solve()`가 자신의 구현으로 dispatch한다고 오해한다. 실제로 그런 교체 계약이 없다. | `optimizeFromPath()`를 기존 low-level/compatibility 진입점이라고 명확히 하고 신규 외부 사용은 `solve(OptimizerSolveRequest)`로 안내. MINCO/L-BFGS 내부 분해나 virtual 전환은 하지 않는다. |
| **CONTRACT_CHANGE — 별도 export 항목** | `path_planner/include/path_planner/fm2_gpu.h:11`, `:22`의 함수 선언은 항상 설치됨. `path_planner/CMakeLists.txt:74`에서 CUDA가 있을 때만 구현 추가, `:101`의 `PP_HAVE_CUDA`는 PRIVATE, `:150`은 모든 헤더 설치. | CPU 전용 설치에서 외부 사용자가 `fm2CudaAvailable()`로 runtime capability를 확인하려 해도 함수 심볼이 없어 링크할 수 없다. 현재 생산 caller는 `dyn_a_star.cpp:1822`의 compile guard 안이라 이 문제가 숨겨진다. | 이름 변경으로 해결되지 않는다. 공개 API로 유지하려면 CPU 빌드용 `false` 반환 stub과 documented capability 제공이 가장 작은 해법. 내부 전용으로 정하려면 별도 비공개 헤더/설치 정책 필요. 현재 CPU 빌드를 다시 실행한 결과가 아닌 **CMake/source 기반 확인**이다. |

표에서 `path_optimizer/`, `path_planner/`, `path_manager/`로 시작하는 축약 경로의 공통 접두어는 `src/mmp_path_planning/src/`입니다. vehicle 행의 `src/trajectory_simulation.cpp`는 `src/mmp_vehicle_dynamics/src/trajectory_simulation.cpp`를 뜻합니다.

추가로 작은 문서 정리가 유효합니다.

- `optimizer_request.h:18`은 **`max_vel`이 hard velocity limit가 아니라 seed/derived-arrival speed**라고 이미 정확히 적었습니다. 현재는 **KEEP + 사용 예제 강조**가 적절합니다. field를 단순 개명하면 aggregate/직접 필드 접근에 영향이 있고 실제 제한값과의 구분은 의미 계약이므로, 새 이름을 선택한다면 상위 direct/slice request와 함께 별도 호환 설계가 필요합니다.
- `optimizer_environment.h:45`와 `route_cost_profile.h:11`의 callback type에 `(x_u, y_u) → h_u`, gradient 포인터 순서, visibility `[0,1]`, optional gradient, `-inf` 의미 등을 직접 적으면 교체 구현 작성에 도움이 됩니다. 현재 필요한 설명 일부가 `poly_traj_optimizer.h`의 private 멤버 주석에만 있습니다.
- `trajectory_simulation.hpp:164`의 `controller(time,state)`는 controller 객체 getter가 아니라 **controller 계산 후 motion model로 실현한 force/control**을 반환합니다(`trajectory_simulation.cpp:269`). 이름을 반드시 바꿀 정도의 오류는 아니며 DOC_ONLY가 우선입니다.
- `trajectory_simulation.hpp:169`의 `stateIndexAt()`는 interpolation이나 직전 sample 선택이 아니라 **가장 가까운 저장 상태**, 범위 밖은 끝점, 동률은 뒤쪽입니다(`trajectory_simulation.cpp:362`). 이 규칙을 선언 옆에 명시하면 재생 caller 오용을 막습니다.
- `mmp_vehicle_dynamics/python/mmp_vehicle_dynamics/__init__.py:10`의 “참고 yaml이 아직 margin 0.02라 stale” 설명은 현재 `config/vehicle_dynamics.yaml:38`의 `0.08` 및 config snapshot test와 모순됩니다. DOC_ONLY.
- `mmp_terrain/frame.py:51`의 `TARGET_CELL_SIZE_M`는 실제 목표 raster resolution이 아니라 **고정 frame metre scale**입니다. 주석에는 이미 명시되어 있어 당장 개명할 필요는 낮지만, 새 이름이 필요하면 `FRAME_UNIT_M`을 canonical 이름으로 추가하고 기존 상수를 alias로 유지하는 방식이 안전합니다. 값 변경은 금지. 실제 해상도는 `MapLoadInput.resolution_m`입니다.

유지하는 편이 좋은 공개 경계도 확인했습니다.

| 범위 | KEEP 판단 |
|---|---|
| Optimizer | `PolyTrajOptimizer::solve`, `OptimizerSolveRequest/Result/Status`, `OptimizerEnvironment`, `OptimizerRuntimeSettings`, `buildPieceAltitudeCaps`, `buildPieceRoughness`는 역할과 일치. 값 결과·borrowed provider·serial session 규약이 구분됨. `setParam`은 ROS 구성 adapter라는 기존 이름을 유지하고 신규 진입점으로 오해하지 않게 안내하면 충분. |
| Vehicle | `VehicleModel`과 `SimulationMotionModel`은 계획 역동역학/제약 판정과 forward simulation의 서로 다른 경계를 명시한다. `FixedWingModel`, `FixedWingMotionModel`, `TrackingController`, `SimulationSession`, `simulate`를 하나로 통합하거나 같은 이름으로 맞출 필요 없음. |
| 결과 소유권 | `OptimizerSolveResult` 및 `SimulationResult`는 값을 소유하고, `SimulationSession::takeResult() &&`는 이동을 이름·형식으로 명시. `states()/metrics()/reference()` borrowed 참조는 README에도 설명됨. |
| Terrain | `MapLoadInput/Result`, `load_map`, `resolve_map_mission`, `normalize_elevation`, `ros_terrain.to_grid_map`은 역할이 구분된다. `mission`이 snapshot인 점, writable owned grid, mixed XY frame / Z metres AGL, ROS adapter의 채택 책임이 문서화되어 있다. 일괄 개명 근거 없음. |
| Namespace·target | `ego_planner`, `path_planner`, `mmp_vehicle_dynamics`와 기존 exported target명을 심미적 통일 목적으로 바꾸지 않는 것이 좋음. ROS 유지 범위에서 현재 package 이름의 존재 자체는 결함이 아님. |

검토 범위는 optimizer 공개 요청/결과/환경/프로필/설정 overlay·legacy container와 caller, path_planner search/SDF/GPU 공개 선언·CMake, vehicle의 5개 공개 헤더·simulation/controller/follower ROS wrapper·Python binding/예제/CMake, terrain map loader/frame/mission/ROS serializer/문서·예제·패키징입니다. `risk_shape.h`·`eikonal_godunov.h`의 공개 helper 표면도 확인했습니다.

MINCO/L-BFGS·root finder 수치 본체는 감사/분해하지 않았습니다. terrain 자산 제작 도구(`source_coverage`, `windowed_mosaic`, `full_map_assembler`)는 함수 목록만 확인했으며 내부 처리 전수 검토에 포함하지 않습니다. 별도 `src/mmp_dynamics_sim` 패키지는 이 checkout에 없어서, 현재 존재하는 C++ follower와 Python model binding까지만 확인했습니다. 금지 디렉토리는 읽지 않았습니다.
