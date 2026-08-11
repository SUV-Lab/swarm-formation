# 두 계약에서 사람이 결정해야 하는 것

설계자가 임의로 정할 수 없다고 판단해 남긴 항목들. 각 항목은 "어느 쪽을
고르든 설계가 성립하되 결과가 달라지는" 지점이다.

- initial-speed / SCOPE: r2 deletes statedStartSpeedProblem and routes both stated forms through stateEnvelopeProblem + classifyStartState. This is the fix for four separate findings, but it is a behaviour change beyond 'add a presence bit': a stated initial_speed above the handoff ceiling (effectiveHandoffMaxMps() = optimization/max_vel 2.0 u/s = 200 m/s by default) now REFUSES where it used to plan. All 8 shipped missions sit exactly at 200.0 so nothing in-tree breaks, but any per-segment optimization/max_vel override (the commented idiom at optimizer_params.yaml:37, 99) moves the ceiling. Approve the unification, or keep two validators and accept that the two 'mutually exclusive, equal' forms disagree above 200 m/s.

- initial-speed / SEMANTICS: r2 makes a stated initial state OUTRANK an in-flight local trajectory (stage-2 rows 3-4 above row 5). This is what makes the contract non-inert given the panel's constant mission_id, but it changes mid-flight replan semantics: a command arriving during a flight now replans from the commanded start instead of continuing from the trajectory point. Confirm that is intended for every publisher, not only the panel. The alternative is to rely solely on the mission_id fix, which makes the guarantee depend on publisher hygiene.

- initial-speed / POLICY: a stated REST start (speed 0 from either form) is refused as INITIAL_MODE_UNSUPPORTED, model-independently, with NO opt-in flag. The only legitimate acceptance path is a launch-phase planner that prepends 0 -> activation speed. Confirm nobody needs launch-from-rest in this cycle. A 'planning/allow_launch_from_rest' flag is explicitly rejected: unlike allow_final_boundary_relaxation it cannot drop a requirement, it can only rewrite the operator's stated speed to 131.8 m/s.

- initial-speed / TEST FLOW: under r2 a MALFORMED message is refused even with test/inject_init_state enabled (injection overrides the state, not the validity of the command). README.md:113 documents the injection flow; confirm no existing test rig publishes a command that is malformed under the new rules and relies on injection to rescue it.

- initial-speed / PUBLISHER IDENTITY: the panel's mission_id changes from the constant "mission_config_panel" to "<yaml stem>#<sequence>". This also fixes the start-position adoption bug (mission B currently plans from mission A's trajectory point). Confirm no consumer, log parser, or recorded-bag analysis keys on the constant string.

- initial-speed / TRANSITION REACH: routing the scalar form through classifyStartState means a level 60 m/s stated as initial_speed_mps can dispatch to the transition coordinator when transition/enable is true — today only the vector form can. Confirm the transition contract owner accepts the scalar form as a legitimate entry into that path.

- terrain / SCOPE, BLOCKING: r2 requires a new PlanStatus message and topic to ship with this contract. Without it the new TERRAIN_MAP_MISMATCH refusal does not merely fail silently — it presents as SUCCESS (the panel prints 'published', /planning/trajectory keeps latching the PREVIOUS mission's PolyTraj at QoS(5) TL, RViz shows a valid trajectory). Accept the scope growth, or accept shipping a refusal that looks like a success. There is no third option. Note this channel also closes the equivalent gap in the initial-state contract, so it is shared infrastructure, not terrain-specific.

- terrain / TOPIC RENAME: /planning/terrain_ready -> /planning/terrain_ingest is mandatory (reusing the name with a new type gives a silent no-match indistinguishable from 'topic absent'). Confirm no external tooling, launch file, bag recording profile, or downstream consumer outside this tree subscribes to the old name.

- terrain / ENFORCEMENT STRENGTH: expected_map_token = 0 permanently means 'no requirement', which is what keeps ab_difficulty_balance.py, e2e CLI runs and every ad-hoc ros2 topic pub working. The resulting guarantee is 'a gated publisher cannot silently plan on the wrong map', NOT 'nothing can'. Accept that, or add a launch parameter planning/require_map_token (default off) that upgrades a zero token to a refusal for production runs.

- terrain / OPERATOR BYPASS: does the operator get a bypass button at all, and is a bypassed run allowed to fly? r2 offers it on 'no receipt / stale build', 'budget expired', and 'server too old', and REFUSES it on an explicit REJECTED verdict (the planner answered; there is nothing to override) and when no planner is in the graph. Confirm that split, and confirm a bypassed command publishing expected_map_token = 0 — visibly stating no terrain requirement on the wire — is the accepted audit trail.

- terrain / SIM TIME: token uniqueness holds within one publisher lifetime under any clock, but under use_sim_time stamps restart near 0 each run, so tokens can repeat across a publisher restart and a stale receipt could in principle match a replayed token. Nothing in the tree sets use_sim_time today. Accept as a documented boundary, or add a per-boot salt (publisher boot ns mixed into the token) at the cost of the token no longer being literally header.stamp.

- terrain / MULTI-DRONE TIMING: /planning/terrain_ingest ships flat with a drone_id field for filtering, matching the existing single-drone flat-topic choice. Decide whether it moves under a per-drone prefix now (together with /mission/trajectory_command) or waits for the swarm work.

- BOTH CONTRACTS / SHARED PREREQUISITE: both require the same reordering of trajectoryCommandCallback — validate before mutating, with the admission and parse checks sitting after the dedup gate and above last_received_sequence_, current_mission_id_, and the start-position adoption block at replan_fsm.cpp:1060-1075. Whichever branch lands first must do the reordering; the second rebases onto it. Both also append fields to TrajectoryCommand.msg, so they must not append concurrently. Decide the landing order and assign the reordering to one branch.
---

# 착지 순서

## Cross-contract sequencing (decide first)

Both contracts append to `TrajectoryCommand.msg` and both need the same `trajectoryCommandCallback` reordering (validate-before-mutate). **They must not land concurrently.** Recommended order: **initial-speed first, terrain-receipt second** — the initial-speed change is wire-compatible in isolation (one appended bool, fail-closed) whereas the terrain change forces a whole-workspace rebuild, so doing the cheap one first keeps the expensive one to a single coordinated restart. Whichever lands first owns the callback reordering; the second rebases onto it.

---

## Contract 1 — initial-speed: three commits, the middle one atomic

**Commit A — `mmp_mission_msgs` only.**
Append `bool use_initial_speed` to `TrajectoryCommand.msg`; add the append-only header rule; rewrite the `initial_speed` and `use_initial_velocity` comment blocks. Behaviour-neutral: nothing reads the bit yet, and the appended field defaults false for every existing publisher. Safe to land and sit.

**Commit B — planner + panel + scripts, ONE commit.**
Not separable: the moment the FSM reads the bit, every publisher that does not set it is refused. In-tree publishers of `/mission/trajectory_command` are exactly four (verified):

| Publisher | Today | After | If left behind |
|---|---|---|---|
| `mission_config_panel.cpp:396` | always sends `initial_speed` (default 200) | sets the claim bit, zeroes unclaimed, refuses at parse time, per-Run `mission_id` | **every Run refused with a green "published" status** — the one genuinely bad outcome, and the reason this is one commit |
| `scripts/e2e_smoke.py:106` | `m.get('initial_speed_mps', 200.0)` | bit from key presence; pre-publish abort | exits 1 with "no trajectory" and no reason |
| `scripts/ab_difficulty_balance.py:181` | `m.get("initial_speed_mps", 0.0)` | bit in the `topic pub` dict; abort the arm when nothing is stated | **every A/B row becomes a refusal** and the sweep measures nothing |
| hand-written `ros2 topic pub` | omitted fields default 0/false | must add `use_initial_speed: true` | refused; the message names the missing bit |

Ordering *within* commit B (so intermediate states compile):
1. `start_state.h` — enum, `MissionStartClaim` (with `speed_u`), `StartHead`, predicates, `parseStartClaim`, `applyHeadPolicy`.
2. `planning_result.h` — `INITIAL_STATE_MALFORMED`.
3. `path_manager.h/.cpp` — delete `statedStartSpeedProblem`, `setStartVelSynthesized`, `start_vel_synthesized_`, `junction_head`; `planGlobalTraj` takes `const StartHead&`; the 1090-1160 block becomes an `applyHeadPolicy` call; add the `‖v‖ == 0` model-independent rule to `stateEnvelopeProblem`.
4. `segment_chain_planner.h/.cpp` — `StartHead` through `plan`/`planImpl`/`planRouteParallel`/`planOverRoute`/`commitRoute`; entry gate 261-301 routes every stated source to `classifyStartState`; **delete the duplicated [VEL-ALIGN]/[STALL-FLOOR] at 1965-2018** in favour of the shared call. *Nothing compiles until 3 and 4 are both done — they are one edit.*
5. `replan_fsm.h/.cpp` — member purge, callback reordering, two-stage resolution, unit-parameter consistency check at construction.
6. `test/start_claim_test.cpp` (new) + `chain_experiment_test.cpp` 10 call sites + `reststart` / `statedspeedtransition` variants.
7. Panel (`.hpp` + `.cpp`), then `e2e_smoke.py`, then `ab_difficulty_balance.py`.

**Commit C — docs and comments.** `optimizer_params.yaml:488` (`dynamics_activation_speed_mps` is NOT inert — it is the UNSUPPORTED/TRANSITION boundary and r2 makes it load-bearing from both stated forms), `docs/TOPICS.md:69`, the 8 mission yamls' "200 m/s = 계획 상한" comment (benchmark default, not platform-validated). No value changes.

**Rollout:** rebuild `mmp_mission_msgs` → `mmp_path_planning` → `mmp_rviz_plugins`, restart planner and rviz2 together. Old bags of `/mission/trajectory_command` replay as short payloads → dropped or read-false → refused; repair with a republish script **outside** the planner (a `legacy_initial_speed_compat` parameter is explicitly rejected — it re-introduces the removed inference behind a flag nobody audits).

**Shipped scenarios:** 8 missions already state `initial_speed_mps: 200.0` → unchanged. 2 (`r5_transition_success_probe`, `r5_transition_reject_probe`) state `use_initial_velocity: true` and no scalar → today the panel silently attaches its 200 default beside the vector; after the change the scalar is zeroed and unclaimed, so the two-source rule never fires. Behaviour unchanged, and the wire finally says what the yaml says. **No shipped scenario breaks.** Third-party yamls break deliberately, at parse time, with the fix printed.

---

## Contract 2 — terrain receipt: ONE release, five steps, no partial rollout

Adding fields changes the type hash, so `mmp_mission_msgs`, `mmp_path_planning` and `mmp_rviz_plugins` rebuild and restart together. Sequence within the release (each step compiles on the previous):

1. **`mmp_mission_msgs`** — `TerrainIngestReceipt.msg` (new), `PlanStatus.msg` (new), `map_token` appended to the `LoadMap` response, `expected_map_token` appended to `TrajectoryCommand`, both added to `rosidl_generate_interfaces`.
2. **`terrain_publisher.py`** — becomes the owner of map identity: `_stamp()` monotone helper called from both publish paths, `response.map_token` filled after the publish. *First, because everything downstream needs real tokens to test against.* If skipped: every reply carries 0, and under r2 the panel fails **closed** with "the gate is unavailable" (r1 fails open here — the one place its prose and its code spec contradicted).
3. **`path_manager`** — `terrain_ingest.h`, `last_applied_token_`, `heldMapToken()`, `TerrainIngestResult setTerrainData(...)`, the nine-row table, **`sdf_voxel_size_` moved into the ingest tail**, `finite_cells` folded into the existing self-check loop, `TERRAIN_MAP_MISMATCH` in `planning_result.h`.
4. **`ReplanFSM`** — receipt publisher on `/planning/terrain_ingest` created and `STATUS_NONE` published before `terrain_sub_`; `terrainCallback` collapses to translate-and-publish (delete 1188-1218, the cloned predicates); `PlanStatus` publisher wired to the two existing outcome write sites; admission check after the dedup gate and above every mutation. If only 3-4 move: the panel sees no publisher on the new topic and fails closed with the "stale build" diagnosis — the intended half-migration behaviour.
5. **`mission_config_panel`** then **`e2e_smoke.py`** — the only in-tree receipt consumers (verified by grep: the other references are the FSM and the panel's own header). Panel gates on `held_token`, `publish()` takes the token as an argument, `graphQuery()` wraps both graph calls, amber status added, `map_token == 0` fails closed. Smoke test drops the log-offset scraping and the `MMP_E2E_SETTLE` sleep, keeps `MMP_E2E_REQUIRE_FINE` asserting on `receipt.resolution_u`, and asserts the `PlanStatus` verdict — making it the executable test of this contract.

**Topic rename** `/planning/terrain_ready` → `/planning/terrain_ingest` happens in the same commit as step 4. Reusing the name with a new type gives a silent no-match indistinguishable from "topic absent"; renaming makes `ros2 topic list` diagnostic and lets the panel's `count_publishers` check honestly report "no planner publishes the receipt topic".

**Unchanged, verified:** `ab_difficulty_balance.py` (`expected_map_token` defaults 0 = no requirement — this harness is why the 0 convention is mandatory rather than a nicety), `map_selector_panel.cpp` (never touches receipt or command; it *causes* the planner restarts the token identity is designed to survive), `altitude_profile_panel` / `trajectory_metrics_panel` (subscribe to `/terrain/grid_map` only).

**Bags:** a replayed `/terrain/grid_map` carries an old stamp, so the planner emits a receipt quoting a token no live LoadMap reply can match. The gate holds and fails closed — correct — which means bag-driven replay must use the `expected_map_token = 0` path.