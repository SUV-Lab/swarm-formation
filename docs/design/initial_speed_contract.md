# 설계안 — 초기 상태 입력 계약 (`use_initial_speed`)

> **상태: 제안. 구현 없음.** 2차 검토(§9.3) 권장 순서 1번.
> A/B 측정이 고정 커밋에서 도는 동안 **설계만** 진행한 산출물이며,
> 주 작업공간의 소스는 건드리지 않았다.
>
> 결정이 필요한 항목은 `docs/design/open_questions.md`에 모았다.

> ## RECOMMENDATION — initial-state contract, revision 2 (supersedes r1)
>
> The wire decision from r1 survives review unchanged and is **not** re-opened: ONE appended `bool use_initial_speed`, fail-closed default, no enum on the wire, append-only layout rule in the header. The three high findings were all on the C++/dispatch/migration side and all three are real; r2 changes the design rather than defending it.
>
> ### What changed vs r1, in one line each
> 1. **Head policy is data carried in a struct, applied at the two sites that own a route** — not "dispatched inside `planGlobalTraj`". *(finding 1 — real: the default config bypasses `planGlobalTraj` for the mission head.)*
> 2. **A stated initial state outranks an in-flight trajectory.** `TRAJECTORY_DERIVED` is now the fallback for commands that state nothing, not the winner. *(finding 2 — real: the panel's constant `mission_id` made r1 inert.)*
> 3. **Two evaluation stages, explicitly.** Pure-message validation always runs first and unconditionally; context can only override a *well-formed* claim. *(finding 3 — real: r1's ordering was unimplementable at its own stated site.)*
> 4. **`statedStartSpeedProblem` is deleted.** Both stated forms are judged by `stateEnvelopeProblem` + `classifyStartState`. *(findings 4, 8, 9 — real: floor-only gating, dynamics-off asymmetry, and transition unreachable from the scalar form are all one root cause: two validators for one quantity.)*
> 5. **Validate before mutating.** The callback is reordered so a refusal touches no state. *(finding 7 — real.)*
> 6. `junction_head` is deleted; `CHAIN_JUNCTION` is the sole selector. *(finding 6 — real, and the same argument r1 used against the wire enum.)*
> 7. `MissionStartClaim` gains `double speed_u`. *(finding 5 — real.)*
> 8. Malformed-input tests get their own binary. *(finding 10 — real.)*
> 9. The panel's doc example splits into two mutually exclusive blocks. *(finding 11 — real.)*
> 10. A startup unit-parameter consistency check makes the msg's "never changes the magnitude" promise enforceable. *(finding 12 — real.)*
>
> ---
>
> ## A. The wire change — complete
>
> `mmp_mission_msgs/msg/TrajectoryCommand.msg`, **one appended field**, plus comment rewrites. No field is inserted, moved, or removed.
>
> **A1. Header note (after the QoS block, before `int32 drone_id`):**
> ```
> # WIRE LAYOUT RULE: this message is APPEND-ONLY. ROS 2 serializes fields
> # positionally, so inserting or removing one re-reads every field after it
> # when an un-updated publisher is on the topic. New fields go at the END.
> # Removing a field is a breaking change and requires a topic rename.
> ```
>
> **A2. Replace the `initial_speed` comment block (lines 43-46), value semantics only:**
> ```
> # Initial speed at trajectory start (m/s). READ ONLY when
> # use_initial_speed is true. Direction is the first route leg, level; the
> # planner may re-aim that direction onto the front-end route, never the
> # magnitude.
> # Judged by the SAME envelope gate as initial_velocity: stall floor,
> # handoff ceiling, flight-path cone, transition classification. The two
> # forms differ in what you may SAY, never in what is accepted.
> #   > 0   a stated entry speed
> #   = 0   an explicitly stated REST start. Legal to STATE; this stack
> #         cannot FLY it and fails the plan with INITIAL_MODE_UNSUPPORTED.
> #         Never silently raised to the stall floor.
> #   < 0 / non-finite   malformed. Publishers must NOT clamp: a clamped -5
> #         is indistinguishable from a stated rest start.
> # Publishers MUST write 0.0 here when use_initial_speed is false.
> float64 initial_speed
> ```
>
> **A3. Rewrite the `use_initial_velocity` preamble (lines 48-50)** — delete "An enabled velocity overrides initial_speed":
> ```
> # Optional map-frame initial-state vectors (SI; converted to planner units
> # by ReplanFSM). MUTUALLY EXCLUSIVE with use_initial_speed: two stated
> # sources for one quantity is an error, not a precedence puzzle. The old
> # "velocity overrides speed" rule existed only because the scalar had no
> # way to assert itself. Acceleration carries its own presence bit and may
> # accompany either.
> ```
>
> **A4. Append at the very end (after `final_acceleration`):**
> ```
> # --- initial-state presence (2026-08 input contract) ---------------------
> # Is `initial_speed` a STATED value? Without this bit three distinct inputs
> # collapsed into one refusal: never set, an explicit 0 m/s ("I start from
> # rest", which this message DEFINES as legal), and a negative value the
> # publisher clamped to 0.
> #
> # false is the fail-closed default and the whole compatibility story: a
> # publisher predating this field reads as "said nothing about the initial
> # speed" whatever `initial_speed` holds, so it is REFUSED, never misread.
> # A NON-ZERO `initial_speed` with this false and no velocity claimed is
> # MALFORMED, not merely unspecified: the only ways to produce it are a
> # stale publisher and a forgotten claim bit, and both want a diagnosis that
> # names the fix.
> bool use_initial_speed
> ```
>
> **Why not an enum on the wire** (unchanged from r1, four reasons, all still hold): replacing `use_initial_velocity` shifts all four `final_*` fields; appending an enum beside the surviving bool gives the wire two selectors for one decision; `TRAJECTORY_DERIVED` is not publishable (it would be a forgeable claim that skips the head gate); and the enum cannot swallow the acceleration presence bit, which must stay independent. The enum's value is internal — that is where it goes.
>
> ---
>
> ## B. Decision table — TWO stages, both normative
>
> Symbols: **S**=`use_initial_speed`, **v**=`initial_speed`, **V**=`use_initial_velocity`, **vec**=`initial_velocity`, **A**=`use_initial_acceleration`, **a**=`initial_acceleration`.
>
> ### Stage 1 — `parseStartClaim(const TrajectoryCommand&, double unit_m)`
> Pure function of the message. Runs in `trajectoryCommandCallback`, **unconditionally, before any FSM state is read or written**. No node parameter, no FSM member, no context. First match wins; the table is total.
>
> | # | Condition | Result |
> |---|---|---|
> | 1 | `S && V` | MALFORMED "two stated initial-state sources (use_initial_speed and use_initial_velocity) — one quantity, one claim" |
> | 2 | `S && !finite(v)` | MALFORMED "non-finite initial_speed" |
> | 3 | `S && v < 0` | MALFORMED "negative initial_speed %.3f — publishers must not clamp" |
> | 4 | `S` (so `v >= 0`, finite) | `STATED_SPEED`, `speed_u = v/unit_m` (**includes v == 0**) |
> | 5 | `V && !finite(vec)` | MALFORMED "non-finite initial_velocity" *(today: WARN + downgrade → refused as UNSPECIFIED, the wrong diagnosis, replan_fsm.cpp:1038-1043)* |
> | 6 | `V` | `STATED_VECTOR`, `vel_u = vec/unit_m` (**includes the zero vector**) |
> | 7 | `!S && !V && v != 0` | MALFORMED "initial_speed=%.1f with use_initial_speed=false — publisher predates the initial-state contract, or the claim bit was forgotten" |
> | 8 | `!S && !V && v == 0` | `UNSPECIFIED` |
> | — | `A && !finite(a)` (checked after the above, independently) | MALFORMED "non-finite initial_acceleration" *(today: WARN + silent downgrade)* |
> | — | `!A && a != 0` | MALFORMED, same stale-publisher rule as row 7 |
> | — | `A` (incl. `a == 0`) | `acc_prescribed = true`, `acc_u = a/unit_m` |
>
> **Deliberately NOT malformed:** `V && v != 0 && !S` — a stale publisher that left a scalar beside a claimed vector. Old and new contracts agree the vector wins, so nothing is misread; WARN naming the ignored scalar, mirroring the panel's existing `ignored_velocity_vector` doctrine (mission_config_panel.cpp:207-209).
>
> **This is the fix for finding 3.** Rows 1-3, 5, 7 are now reachable in *every* configuration — with `test/inject_init_state` on, with a local trajectory in flight, always. r1's "layer 3, the positive stale-publisher check" is no longer dead exactly where migration needs it.
>
> ### Stage 2 — `resolveStartSource(claim, ctx)` in `startMissionPlan`
> `ctx = {inject_init_state_, have_local_traj_}`. Context may override a **well-formed** claim; it can never rescue a malformed one.
>
> | # | Condition | Resolved source | Notes |
> |---|---|---|---|
> | 1 | `claim.malformed` | — | **FAILED(`INITIAL_STATE_MALFORMED`)**, unconditionally. Injection overrides the *state*, not the *validity of the command*. |
> | 2 | `inject_init_state_` | `TEST_INJECTED` | full PVA judged, never re-aimed, never clamped (as today) |
> | 3 | `claim.src == STATED_SPEED` | `STATED_SPEED` | head = first-leg direction × `speed_u`, level |
> | 4 | `claim.src == STATED_VECTOR` | `STATED_VECTOR` | |
> | 5 | `have_local_traj_` | `TRAJECTORY_DERIVED` | message initial-state fields ignored; `acc_prescribed := false`; INFO log naming the ignore (today it is silent) |
> | 6 | otherwise | `UNSPECIFIED` | **FAILED(`INITIAL_STATE_UNSPECIFIED`)** — now a true "nothing was said" |
>
> **Rows 3-4 above row 5 is the fix for finding 2.** A command that states an initial state is honored whether or not a trajectory is in flight. r1 put `TRAJECTORY_DERIVED` first and keyed it on `mission_changed`, which the panel's constant `mission_id = "mission_config_panel"` (mission_config_panel.cpp:359) makes false for every Run after the first — so under r1 the entire new claim, *and* the new start position (the adoption block at replan_fsm.cpp:1060-1075 is behind the same `mission_changed`), would have been discarded for the one flow the contract exists to fix. The panel `mission_id` fix ships too (§E), but the contract must not *depend* on publisher hygiene for its central guarantee.
>
> ### Stage 3 — head judgment (one gate, both stated forms, both planning modes)
>
> | Source | Judgment | Re-aim onto route | [STALL-FLOOR] clamp |
> |---|---|---|---|
> | `UNSPECIFIED` / malformed | refuse before planning | — | — |
> | `STATED_SPEED` | `classifyStartState(pos, head_vel, acc, acc_prescribed)` | yes, **unless** `acc_prescribed` | **never** |
> | `STATED_VECTOR` | `classifyStartState(...)` | no | never |
> | `TEST_INJECTED` | `classifyStartState(...)` | no | never |
> | `TRAJECTORY_DERIVED` | none (our own state) | no | **yes** — the only source that may be repaired |
> | `CHAIN_JUNCTION` | none (judged as the previous segment's tail) | no | never (kept verbatim, path_manager.cpp:1137-1143) |
>
> `classifyStartState` returns CRUISE_VALID / TRANSITION_REQUIRED / UNSUPPORTED exactly as today (segment_chain_planner.cpp:1344-1395), and internally calls `pvaEnvelopeProblem` when acc is prescribed, `stateEnvelopeProblem` otherwise.
>
> **`PathManager::statedStartSpeedProblem` is DELETED.** It is the root of three separate findings: it checks only `stallFloorUnits()` — no handoff ceiling, no cone — so `initial_speed_mps: 400.0` planned while `initial_velocity_mps: [400,0,0]` was refused ("above the effective maximum 200.0 m/s"); it returns `{}` when the floor is non-positive while `stateEnvelopeProblem` returns `{}` when `dynamicsEnabled()` is false, so the two forms disagreed with the model off; and because it is reached from the `else if (start_vel_synthesized)` branch, a scalar-stated head never reached `classifyStartState`, making TRANSITION_REQUIRED unreachable from the scalar form even with `transition/enable:=true` while the identical level 60 m/s stated as a vector dispatched to the coordinator. **One quantity, one gate.** The friendlier "stated initial speed …, and the planner does not raise a stated one" wording survives as a prefix argument to the existing validator, not as a second validator.
>
> **New model-independent rule, stated explicitly:** `‖v‖ == 0` from *any* stated source is `INITIAL_MODE_UNSUPPORTED` regardless of `dynamics_enable`. Same doctrine as the finiteness check already at path_manager.cpp:620-623 — a zero head has no direction, the first-leg synthesis and every downstream normalization divide by it, and no configuration makes it flyable. This is a **new hardcoded rule**, not a consequence of the 131.8 m/s floor; r1 justified rest-refusal with numbers (`dynamics_speed_min_mps` 122.0 × 1.08, `model_activation_speed_mps` 40.0) that do not exist under `optimization/dynamics_enable: false`. Rows 4 and 6 of stage 1 now partition identically at zero in every configuration.
>
> ### What a stated REST start does: refuse with `INITIAL_MODE_UNSUPPORTED`
> Grounded: the stall floor under the shipped config is `dynamics_speed_min_mps` 122.0 × (1 + `dynamics_margin` 0.08) = **131.8 m/s** (optimizer_params.yaml:476, 502); the cruise pipeline is *measured* to fail on rest ("dynamics_cost 62M, 4× -1005, audit clearance -0.79 → nothing published", path_manager.cpp:1120-1128); and `classifyStartState` returns UNSUPPORTED below `model_activation_speed_mps` = 40 m/s, which the scalar form now actually reaches. `INITIAL_MODE_UNSUPPORTED`'s existing comment already describes this case (planning_result.h:45-49) — no new reason code for rest.
>
> **Rejected: an opt-in that accepts rest.** `planning/allow_final_boundary_relaxation` *drops a soft requirement* and still flies a trajectory satisfying everything else. There is no "drop" at the head — it is a boundary condition the solver must satisfy, so relaxing it means rewriting the operator's stated speed to 131.8 m/s, the exact fabrication this contract exists to stop. The legitimate answer is a launch-phase planner that prepends 0 → activation speed. That is a planner, not a flag.
>
> ---
>
> ## C. Where head policy lives — the fix for finding 1
>
> r1 claimed the [STALL-FLOOR] clamp becomes "unreachable by construction" by dispatching on `src` inside `PathManager::planGlobalTraj`. **That is false under the shipped configuration.** optimizer_params.yaml:74-75 ships `chain/author_from_route: true` + `chain/parallel: true`, so `planImpl` (segment_chain_planner.cpp:380-397) dispatches every single-goal mission to `planRouteParallel` → `planOverRoute`, which re-implements both rules on segment 0's head *outside* `PathManager`: [VEL-ALIGN] at segment_chain_planner.cpp:1965-1977 and a full [STALL-FLOOR] raise at 1988-2018 with its own duplicated floor arithmetic, gated only on `transition == nullptr` — no source, no junction flag. r1 edited neither, and could not have compiled: `start_vel_synthesized` is carried through `plan` / `planImpl` / `planRouteParallel` / `planOverRoute` signatures (segment_chain_planner.h:76-84, 118, 271, 280), all omitted from r1's change list.
>
> **r2's rule: the duplicate is deleted, not gated.**
>
> - New free function in `path_manager/start_state.h`:
>   `void applyHeadPolicy(const std::vector<Eigen::Vector3d>& route, const StartHead& head, Eigen::Vector3d* vel_eff, LogManager* log);`
>   It contains the single implementation of [VEL-ALIGN] and [STALL-FLOOR], keyed on `head.src` and `head.acc_prescribed` via `mayReaim()` / `mayClamp()`, and reads the floor from `poly_traj_opt_->dynamicsMinSpeedFloorUnits()` — the one accessor, never the re-derived `speed_min*(1+margin)/um` arithmetic currently duplicated at segment_chain_planner.cpp:2005-2010.
> - `PathManager::planGlobalTraj` (path_manager.cpp:1090-1160) calls it. The 70 lines there are replaced by the call.
> - `SegmentChainPlanner::planOverRoute` (segment_chain_planner.cpp:1965-2018) calls it. Those 54 lines are replaced by the call. The `transition == nullptr` guard is subsumed: a transition-prescribed head arrives as `TRAJECTORY_DERIVED`, which `mayReaim()` returns false for; the structural exemption stays structural.
> - `bool start_vel_synthesized` is deleted from `plan`, `planImpl`, `planRouteParallel`, `planOverRoute`, and `commitRoute`; each takes `const StartHead&`.
> - `PathManager::setStartVelSynthesized` and the member `start_vel_synthesized_` are deleted (path_manager.h:418, 714).
> - `bool junction_head` is deleted from `planGlobalTraj` (path_manager.h:282-287). `CHAIN_JUNCTION` is the source. **Fix for finding 6** — r1 would have left `mayClamp(src) && !junction_head`, two independent inputs encoding one fact that can disagree, which is precisely the argument r1 used to reject the wire enum. `junction_goal` is a different fact and stays.
>
> Now the clamp really is unreachable for operator input by construction, because there is exactly one implementation and it is keyed on the source.
>
> ---
>
> ## D. New/changed C++ — complete list
>
> **NEW `src/path_manager/include/path_manager/start_state.h`** (beside `planning_result.h`, so planners do not depend on the state machine):
> - `enum class StartStateSource { UNSPECIFIED, STATED_SPEED, STATED_VECTOR, TRAJECTORY_DERIVED, TEST_INJECTED, CHAIN_JUNCTION };`
> - `struct MissionStartClaim { StartStateSource src{UNSPECIFIED}; double speed_u{0.0}; Eigen::Vector3d vel_u{Zero}; bool acc_prescribed{false}; Eigen::Vector3d acc_u{Zero}; std::string problem; };` — **`speed_u` is the fix for finding 5.** For `STATED_SPEED` the velocity vector does not exist at callback time: the direction is the first route leg, resolved in `startMissionPlan` from `start_pt_` and the waypoint list (replan_fsm.cpp:598-622), and `start_pt_` is not AGL-resolved until `resolveCommandedStartAgl`. r1 deleted `commanded_initial_speed_` with no replacement and no place to put the magnitude. The claim carries the magnitude; the head vector is built later; `[VEL-ALIGN]` re-aim is a *route* refinement of an already-real first-leg direction, never a repair of a placeholder axis.
> - `struct StartHead { StartStateSource src; bool acc_prescribed; };` — the value threaded through the planners.
> - Free predicates: `isStated`, `isOperatorInput`, `judgeFullPva`, `mayReaim`, `mayClamp`, `sourceName`.
> - `MissionStartClaim parseStartClaim(const TrajectoryCommand&, double unit_m);` — stage 1 verbatim, no ROS spin.
> - `void applyHeadPolicy(...);` — §C.
>
> **`planning_result.h`** — add `INITIAL_STATE_MALFORMED` adjacent to `INITIAL_STATE_UNSPECIFIED`, FAILED-only: "the command could not be read" vs "the command said nothing". Update the `INITIAL_STATE_UNSPECIFIED` comment: an `initial_speed` value with `use_initial_speed` false is MALFORMED, not this.
>
> **`replan_fsm.h`** (lines 145-173) — delete `commanded_initial_speed_`, `use_commanded_initial_velocity_`, `use_commanded_initial_acceleration_`, `commanded_initial_velocity_`, `commanded_initial_acceleration_`, `start_vel_synthesized_`, `start_vel_commanded_`, `start_state_stated_`. Add `MissionStartClaim mission_start_claim_;` and `StartHead start_head_;`. Keep `initial_speed_unit_m_`.
>
> **`replan_fsm.cpp`**
> - **Callback reordered — the fix for finding 7.** New order: drone filter → dedup gate → `parseStartClaim` (+ any other pure-message validation) → **refuse and `return` here** → *then* consume `last_received_sequence_`, overwrite `current_mission_id_`, adopt the start position, clear `have_local_traj_`. r1 put the new MALFORMED refusals downstream of the adoption block at replan_fsm.cpp:1060-1075, which sets `start_pt_`/`current_pos_`/`start_position_received_` and `have_local_traj_ = false` — while the rollback at 1143-1152 restores only `last_received_sequence_` and `current_mission_id_`. Today a non-finite commanded acceleration is a WARN and the mission still plans; r1 converted that (plus negative speed, both-sources, unclaimed non-zero) into FAILED, so each new refusal would have dropped a flying trajectory and overwritten the start point with the rejected mission's, irrecoverably. Validate-before-mutate removes the whole class.
> - Delete `std::max(0.0, msg->initial_speed)` (line 1002) — that clamp is literally the third collapsed input.
> - Delete both "Ignoring non-finite …" WARN-and-downgrade blocks (1038-1049); they become MALFORMED.
> - `startMissionPlan` 558-659: three-flag assignments (570-572, 583-588, 594-596, 627-628, 653-655) collapse to `resolveStartSource`. `commanded_initial_speed_ > 0.0` (597) becomes `src == STATED_SPEED` so a stated 0 takes the same branch and produces a zero head vector that the gate then refuses. First-leg aiming (598-622) unchanged. `vel_src` becomes `sourceName(src)`. The latent trap at 558-579 — the `have_local_traj_` branch never resets `use_commanded_initial_acceleration_`, so a stale bool reaches `chain.plan(start_acc_commanded=…)`, harmless today only because `start_vel_commanded_` is false there — becomes structurally impossible.
> - `triggerGlobalPlan` 771-796: the single gate becomes two ordered checks (`!problem.empty()` → MALFORMED; `src == UNSPECIFIED` → UNSPECIFIED), same bookkeeping and rollback.
> - 804-835: the `start_vel_commanded_ ? … : start_vel_synthesized_ ? …` ladder becomes the stage-3 table. This also closes a live hole: today a stated *speed* with a 51-g commanded acceleration reaches the optimizer head with the acceleration never envelope-judged, because both replan_fsm.cpp:829-835 and segment_chain_planner.cpp:261-301 select full-PVA only on `start_vel_commanded_`.
> - **New startup consistency check** (finding 12, real): error at construction if `manager/initial_speed_unit_m`, `optimization/dynamics_unit_xy_m` and `optimization/dynamics_unit_z_m` are not all equal. The FSM converts every component — including z — with `initial_speed_unit_m` (replan_fsm.cpp:83-87, 1005-1007) while the gates convert back with the two `dynamics_unit_*` parameters (path_manager.cpp:629-633, segment_chain_planner.cpp:1369-1375), and nothing cross-checks them. Drift silently makes the magnitude judged differ from the magnitude stated — the exact fabrication the new msg text forbids. r1 added the promise without the enforcement.
>
> **`path_manager.h` / `.cpp`** — `statedStartSpeedProblem` deleted; `setStartVelSynthesized` and `start_vel_synthesized_` deleted; `planGlobalTraj` takes `const StartHead&` (required, no default) and loses `junction_head`; the [VEL-ALIGN]/[STALL-FLOOR] block (1090-1160) becomes an `applyHeadPolicy` call. Making the source a required parameter turns "forgetting it" from a convention (six call sites currently get it right by hand) into a compile error.
>
> **`segment_chain_planner.h` / `.cpp`** — `plan` / `planImpl` / `planRouteParallel` / `planOverRoute` / `commitRoute` take `const StartHead&` in place of the three booleans (three bools with four legal combinations out of eight become one value with exactly the legal set); the entry gate at 261-301 becomes a switch on `src` where every stated source reaches `classifyStartState`; line 635 passes `i == 0 ? head : StartHead{CHAIN_JUNCTION, false}`; 1690/1729/1887 pass the explicit source; 1965-2018 replaced by the shared call.
>
> **`optimizer_params.yaml:488`** — the comment on `dynamics_activation_speed_mps` says "REPORTING/BUCKETING ONLY … Changing it has no trajectory effect". It is the UNSUPPORTED/TRANSITION_REQUIRED boundary in `classifyStartState` (segment_chain_planner.cpp:1377). r2 leans on that boundary from *both* stated forms, so the stale comment becomes load-bearing misinformation. Corrected in the same commit.
>
> **Tests — the fix for finding 10.** `chain_experiment_test.cpp` contains no `ReplanFSM` and no `TrajectoryCommand` (grep: zero hits); every variant drives `chain.plan(...)` directly. After the signature change, "both sources stated", "negative", "NaN" and "unclaimed non-zero" are **not expressible** there — they are message-level states rejected in `parseStartClaim`. So:
> - **NEW `test/start_claim_test.cpp`** (gtest, no ROS spin): all eleven stage-1 rows, one case each. This is where `malformedstart` actually lives.
> - `chain_experiment_test.cpp`: 10 call sites take the new signature; `synthclamp` (1925-1979) re-expressed in enum terms; new `reststart` variant (stated 0 → `INITIAL_MODE_UNSUPPORTED`, not UNSPECIFIED, not clamped, still refused with `transition/enable:=true`); new `statedspeedtransition` variant (level 60 m/s stated as a scalar dispatches to the coordinator when enabled — the divergence r2 closes).
>
> ---
>
> ## E. Publishers
>
> **`mission_config_panel`**
> - `mission_id` becomes `"<yaml stem>#<sequence>"` instead of the constant `"mission_config_panel"`. Every Run is genuinely a new mission, which fixes the start-position adoption bug at replan_fsm.cpp:1060-1075 as a side effect (mission B currently plans from mission A's trajectory point).
> - `parseYaml` 462-464: delete `std::max(0.0, …)`; **key presence is the opt-in** for the scalar (`if (m["initial_speed_mps"])`) — the parser can tell presence from absence, which is exactly what the wire cannot, and a redundant `use_initial_speed:` key would only add a sync failure mode. The vector keeps its explicit `use_initial_velocity:` opt-in for its documented historical reason.
> - New checks after 515: both stated → parse error naming the line to delete; neither stated → parse error with the copy-pasteable fix. Both run in `onRunClicked` **before** `load_map`, so a bad mission fails in milliseconds instead of after the 30 s ingestion gate.
> - `publish` 396-400: set `use_initial_speed`; **zero `initial_speed` when the bit is false** so the stale-publisher diagnostic stays meaningful.
> - `mission_config_panel.hpp:78`: `initial_speed_mps = 200.0` → `0.0`; add `bool use_initial_speed = false`.
> - **hpp:30-40 schema comment split into two mutually exclusive example blocks** (finding 11, real). r1 left one block containing `initial_speed_mps: 200.0` *and* `use_initial_velocity: true` — copying it verbatim becomes a hard parse failure under the new both-sources rule. Same shape hits anyone editing a shipped mission (all 8 carry both keys, e.g. r1_coastal_terrain_following.yaml:19-20) to try a vector start: flipping the bool without deleting the scalar line now refuses. Kept as an error, but the message says "delete the initial_speed_mps line".
> - Status line 195-210 appends the resolved claim ("start: 200.0 m/s along the first leg" / "start: vector (0,-160,100) m/s").
>
> **`scripts/e2e_smoke.py`** (12, 106-112) — drop the 200.0 default, derive the bit from key presence, exit non-zero **before publishing** when neither source is stated.
> **`scripts/ab_difficulty_balance.py`** (174-184) — add `use_initial_speed` to the `topic pub` dict, drop `m.get("initial_speed_mps", 0.0)`, abort the arm when neither source is stated; otherwise every row silently becomes a refusal and the sweep measures nothing.
> **8 shipped mission yamls** — no value change; only the comment ("200 m/s = 계획 상한") corrected to say 200 is a neutral benchmark default, not a platform-validated value.
>
> ---
>
> ## F. Un-updated publishers: broken loudly, four layers
>
> 1. **Fail-closed default.** `use_initial_speed` defaults false; ignorance produces "said nothing", which is refused, never misread.
> 2. **Append-only layout.** The field is last, so an old payload is short: the middleware either errors on deserialization or reads false. Both are refusals. No `final_*` field can be re-read as something else — which inserting the field, or replacing `use_initial_velocity` with an enum, would have caused.
> 3. **In-band stale-publisher detection.** Layer 2 is middleware-dependent and must not be trusted alone. Stage-1 row 7 is the positive check: non-zero `initial_speed`, no claim, no vector → MALFORMED with a message naming the stale-publisher hypothesis. **In r2 this check is always reachable** (stage 1 is unconditional); under r1 it was dead whenever a local trajectory existed or injection was on, i.e. precisely when an old publisher keeps resending.
> 4. **Publisher hygiene rule in the .msg** ("MUST write 0.0 when the bit is false"), which makes layer 3's signal meaningful rather than noisy.
>
> **Honest limit:** an old publisher on the *vector* path is byte- and semantics-compatible and still plans. Intended (its claim is unambiguous under both contracts), but it means "old publisher" is not globally detectable — only "old publisher relying on the scalar" is.
>
> **Bags:** `ros2 bag play` replays stored bytes, so a pre-change bag hits the new subscriber as a short payload → dropped or read-false → refused. Repair by republishing through a rewrite script, i.e. **outside** the planner. **Rejected: a `planning/legacy_initial_speed_compat` parameter** that reads `initial_speed > 0` as stated — it re-introduces behind a flag exactly the inference the contract removes, and flags of that kind end up set in a launch file and never audited.
>
> ---
>
> ## G. Findings judged NOT to require a design change
>
> None. All twelve are real and all twelve are addressed above. The wire decision (append-only bool, no enum, fail-closed default) is retained unchanged because no finding attacked it.
>
> ## H. Known gaps that remain after r2 (unchanged from r1, still true)
>
> Expressiveness ≠ capability (the interface can now say "rest"; no launch planner exists). No "speed + heading" form — a mission meaning "200 m/s on heading 135" must use the vector. No measured initial state — `TRAJECTORY_DERIVED` is our own authored trajectory, not telemetry; there is no odometry subscription. Point claims only, no uncertainty: 131.9 plans, 131.7 refuses. No freshness — `header` was dropped, `sequence` is per-publisher. `use_initial_acceleration` is not source-scoped on the wire; "TRAJECTORY_DERIVED ⇒ not prescribed" is a code rule, not an interface guarantee. No per-drone defaults. **And the big one: refusals are still invisible outside the log** — `last_plan_outcome_`/`last_plan_reason_` are write-only members (replan_fsm.cpp:792-793, 894-895 are the only writes in the tree). r2 improves the *diagnosis*, not the *delivery*. **The `PlanStatus` channel proposed in the terrain-receipt contract closes this for both contracts and should be reviewed as shared infrastructure, not as terrain-specific.**