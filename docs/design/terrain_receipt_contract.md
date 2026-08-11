# 설계안 — 지형 수신 영수증 계약 (generation/status)

> **상태: 제안. 구현 없음.** 2차 검토(§9.3) 권장 순서 2번.
> 초기 상태 계약과 **동시에 착지시키면 안 된다** — 둘 다
> `TrajectoryCommand.msg`에 추가하고 같은 콜백 재정렬을 요구한다.

> ## RECOMMENDATION — terrain ingestion receipt, revision 2 (supersedes r1)
>
> The identity model from r1 survives review and is **not** re-opened: the token is `GridMap.header.stamp` in nanoseconds, **owned by the terrain publisher, echoed by the planner, learned by the requester from the LoadMap reply**. Nobody counts. The restart argument holds and is confirmed in code — `node.cpp:11/14` constructs `ReplanFSM` before `executor.spin()`, and `/terrain/grid_map` is RELIABLE+TRANSIENT_LOCAL on both ends (terrain_publisher.py:46-51, replan_fsm.cpp:206-212), so a restarted planner re-receives and re-ingests the latched map.
>
> The critique found three hard self-contradictions, one false claim at the core of the decision table, and one place where the migration prose said "fail closed" while the code spec said fail open. All five are real; r2 fixes each with a normative rule.
>
> ### What changed vs r1, in one line each
> 1. **`sdf_voxel_size_` assignment moves after validation**, so "held state unchanged" on rejects becomes true instead of asserted. *(finding 1 — real.)*
> 2. **`held_token` is a separate member that survives the clear**, so `world=none` no longer certifies a state the FSM immediately refuses. *(finding 2 — real.)*
> 3. **The panel's release key is `held_token`, not `subject_token`.** *(finding 3 — real: a later malformed map would have deadlocked the gate for 30 s on a planner whose own receipt said it held T.)*
> 4. **`publish()` takes the token as an argument.** *(finding 4 — real: a sticky member leaks the previous corridor's token into an un-gated Run.)*
> 5. **A `PlanStatus` channel ships with this contract, blocking.** *(finding 5 — real, and the most important: without it a `TERRAIN_MAP_MISMATCH` refusal presents as SUCCESS.)*
> 6. **`map_token == 0` fails closed.** *(finding 6 — real: r1's code spec reused the fail-open skip path in exactly the version-skew state where a wrong-DEM plan is most likely.)*
> 7. **One normative insertion point: after dedup, before any mutation.** *(finding 7 — real, and the same reordering the initial-state contract needs.)*
> 8. **`resolution_u` is asserted in the smoke test.** *(finding 8 — real: a token match proves identity, not that the ladder produced a fine corridor.)*
> 9. **Every graph query goes through one guarded helper.** *(finding 9 — real: `count_publishers` in a Qt slot on a dead context is the `std::terminate` the panel's own comment at mission_config_panel.cpp:556-578 exists to prevent.)*
> 10. **The constructor-reorder rationale is corrected.** *(finding 10 — real: the race r1 cited cannot occur; the reorder is kept for a different, honest reason.)*
> 11. **`bool retryable` added to the msg**, so "no bypass on REJECTED" has a wire basis rather than a convention.
>
> ---
>
> ## 1. NEW `mmp_mission_msgs/msg/TerrainIngestReceipt.msg`
>
> ```
> # Planner ingestion receipt for /terrain/grid_map — the single statement of
> # what terrain the planner actually holds. Latched (RELIABLE +
> # TRANSIENT_LOCAL, depth 1) on /planning/terrain_ingest. ReplanFSM publishes
> # exactly one per delivered GridMap, plus one at startup.
> #
> # Read it as TWO things at once:
> #   * a VERDICT on the last GridMap delivered  (status, reason, subject_token)
> #   * a STATE statement about what is held now (holds_terrain, held_token, geometry)
> # Both are required because depth-1 latching means a late joiner sees only the
> # newest receipt: a REJECTED verdict must not read as "the planner has no map".
> # CONSUMERS GATE ON held_token. subject_token is for fail-fast diagnosis only.
>
> uint8 STATUS_NONE=0       # planner up, no GridMap has ever been delivered
> uint8 STATUS_INGESTED=1   # the subject map IS the planning terrain now
> uint8 STATUS_CLEARED=2    # subject was the empty clear verb (world=none)
> uint8 STATUS_REJECTED=3   # subject refused; the previously held state stands
> uint8 status
>
> uint8 REASON_NONE=0
> uint8 REASON_BAD_RESOLUTION=1         # non-empty map, info.resolution <= 0
> uint8 REASON_NO_PAYLOAD=2             # layers present, data[] empty
> uint8 REASON_NO_ELEVATION_LAYER=3     # no layer named "elevation"
> uint8 REASON_LAYER_PAYLOAD_MISSING=4  # layers[]/data[] length mismatch
> uint8 REASON_BAD_LAYOUT=5             # elevation layout has < 2 dims
> uint8 REASON_DEGENERATE_DIMS=6        # zero or overflowing cols|rows
> uint8 REASON_PAYLOAD_TRUNCATED=7      # data.size() < cols*rows
> uint8 reason
> string detail   # operator text. NEVER parsed — consumers branch on `reason`.
>
> # Is this rejection worth retrying with the SAME map? Every REASON_* above
> # describes a malformed map, so this is false for all of them and a consumer
> # must not offer "retry". It exists so that the day a transient reject is
> # added (allocation failure during SDF build), the assumption breaks LOUDLY
> # at the consumer instead of silently.
> bool retryable
>
> # Identity of the GridMap this receipt JUDGES: its header.stamp in
> # nanoseconds. 0 = none (STATUS_NONE only). The PRODUCER of the map owns this
> # number; the planner only echoes what arrived, which is what makes the
> # receipt survive a planner restart.
> uint64 subject_token
>
> # --- what the planner holds AFTER applying the verdict above ------------
> # held_token is the token of the last map the planner APPLIED — a DEM or a
> # clear. It is NOT cleared by world=none: "I applied your clear" is a state
> # the requester must be able to certify. holds_terrain distinguishes the two.
> bool holds_terrain
> uint64 held_token
> float64 resolution_u     # planner frame units; 0 when !holds_terrain
> float64 origin_x_u       # LOW CORNER (pose - length/2) — the panel's convention
> float64 origin_y_u
> float64 length_x_u
> float64 length_y_u
> uint32 cols
> uint32 rows
> uint64 finite_cells      # non-NaN elevation cells. 0 = ingested but all
>                          # water/nodata: geometrically fine, planning-useless.
>
> # Which planner wrote this. drone_id lets a consumer ignore another agent's
> # receipt when this topic stops being single-drone-flat. instance_id (the
> # planner's boot time in ns) changes on every restart, so a waiting consumer
> # can tell "the planner restarted, re-ingestion is coming" from "I was ignored".
> int32 drone_id
> uint64 instance_id
> ```
>
> **Why typed, not `Float64MultiArray`** (unchanged from r1): the untyped array cannot carry a tri-state verdict plus a machine-readable reason plus operator text — the first two only as magic float constants, the third not at all. It already needs an arity compatibility branch (`data.size() >= 5 ? data[3] : -1.0`, mission_config_panel.cpp:109) and already abuses sentinels (`resolution == 0` means "clear" on the wire; `ready_res_u_ <= 0` means "no receipt yet" in the panel — two facts, one encoding). `ros2 topic echo` currently prints five anonymous doubles. Cost is verified near zero: `mmp_mission_msgs` is already a `<depend>` of `mmp_rviz_plugins` and already used by `replan_fsm.cpp`. It must live in `mmp_mission_msgs` and **not** in a path_manager-owned package — `mmp_rviz_plugins/package.xml` deliberately refuses a `path_manager` dependency so a `--packages-up-to` viz build does not drag the planner in.
>
> ## 2. NEW `mmp_mission_msgs/msg/PlanStatus.msg` — **blocking, not optional**
>
> ```
> # The planner's verdict on one mission command. Latched (RELIABLE +
> # TRANSIENT_LOCAL, depth 1) on /planning/plan_status. Published exactly once
> # per command that reaches the FSM, INCLUDING admission refusals that never
> # reach a planner.
> # PlanOutcome/PlanReason (planning_result.h) have existed as a rich planner
> # contract and never left the process: last_plan_outcome_/last_plan_reason_
> # are write-only members (replan_fsm.cpp:792-793 and 894-895 are the only
> # writes in the tree). This message is what makes them read.
> int32 drone_id
> int32 sequence          # the command this judges
> string mission_id
> uint8 OUTCOME_SUCCESS=0
> uint8 OUTCOME_DEGRADED=1
> uint8 OUTCOME_FAILED=2
> uint8 outcome
> uint16 reason           # PlanReason ordinal; values pinned to planning_result.h
> string detail
> ```
>
> Without this, the new `TERRAIN_MAP_MISMATCH` refusal is not merely silent — **it presents as success.** §D returns before replan_fsm.cpp:1140-1152, so neither `[PLAN REJECTED]` nor the sequence rollback runs; `exec_state_` and `traj_` are untouched; `/planning/trajectory` keeps latching the *previous* mission's PolyTraj (QoS(5) reliable + transient_local, replan_fsm.cpp:172-178). The panel prints a green "ingested → mission published" and RViz draws a valid trajectory. r1 filed this as known-gap 3 and shipped anyway. That is a contract that makes the front of the pipeline honest and the back actively misleading. **`PlanStatus` ships in the same release or the terrain contract does not ship.**
>
> ## 3. `srv/LoadMap.srv` — append to the RESPONSE
> ```
> # Identity of the GridMap this call published: its header.stamp in
> # nanoseconds. Hand it to the planner-ingestion gate — a receipt quoting this
> # exact token is the only proof the planner holds THIS map, and the only
> # thing that stays true across a planner restart. 0 = nothing was published,
> # or the server predates this field.
> uint64 map_token
> ```
>
> ## 4. `msg/TrajectoryCommand.msg` — append
> ```
> # Terrain this mission was prepared for: the map_token returned by the
> # LoadMap call that produced it. 0 = the command states no terrain
> # requirement (CLI/headless run on whatever is loaded, or a deliberate,
> # WIRE-VISIBLE operator bypass of the ingestion gate).
> # Non-zero and different from the token the planner holds => the plan is
> # REFUSED (PlanReason::TERRAIN_MAP_MISMATCH) and the refusal is reported on
> # /planning/plan_status — never silently planned on a different DEM.
> uint64 expected_map_token
> ```
> This field is **read**, not decorative (§D). The repo has a documented allergy to write-only wire fields — TrajectoryCommand.msg:6-10 records the purge of three.
>
> **Cross-contract note:** `TrajectoryCommand` receives appended fields from both contracts. Whichever lands first goes first; both are append-only so the order only has to be fixed once. Do not let the two branches append concurrently.
>
> ## 5. `CMakeLists.txt` — add both new msgs. No new `find_package`: every field is a primitive. `uint64` ns over `builtin_interfaces/Time` on purpose — no dependency, one `==` instead of two, one number under `topic echo`, and a natural zero ("no map stated") whereas a zero `Time` is a legal timestamp.
>
> ---
>
> ## A. Identity — who owns it
>
> **The publisher owns it. The planner echoes it. The requester learns it from the reply. Nobody counts.** `terrain_publisher.py` already stamps both publish paths — `build_grid_map` (~522) and the `world=none` empty map in `load_tif` (~440) — so the clear verb is tokened too and the identity travels *inside* the message being judged, with zero new coordination.
>
> **Not the planner:** a planner-local counter restarts at 0, and because the planner re-receives the latched map on restart, a panel holding "accept only > 7" rejects every receipt the new planner will ever publish — deadlock, precisely when the operator most needs the tool. Persisting it needs a state file nothing else in this tree has; special-casing it needs an instance id, at which point the instance id is doing the work. Separately, `terrain_data_.generation` is bumped on the *clear* path too (path_manager.cpp:3374-3376) — it is a cache-invalidation tick for the ELEV-MEMO, not a map identity. Two jobs, one counter.
>
> **Not the requester:** a nonce would have to route through `terrain_publisher` into the GridMap (no field) or onto a second correlated topic (reintroducing the race), and it is unsound for any terrain change the panel did not request — startup publish, CLI `ros2 service call`, second operator, `map_selector_panel` relaunching with `world:=X`. Those carry no nonce and would be permanently uncertifiable.
>
> **Uniqueness guarantee (3 lines, must be added):** one `_stamp(msg)` helper used by both publish paths — take `clock.now()` in ns; if `ns <= self._last_map_token`, use `self._last_map_token + 1`; store and stamp. Immune to clock non-monotonicity and to two publishes in one tick, without requiring the clock to be trustworthy.
>
> **No content hash:** the token is assigned by the sole producer at the instant the message exists and is never reused. A digest would only catch a producer lying about its own publish — not a failure the wire can police — and would cost a full pass over a ~64 MB payload on the ingestion path. What a digest would genuinely add ("this map has usable content") is `finite_cells`, which the alignment self-check loop (path_manager.cpp:3557-3575) computes for free while scanning for the peak cell.
>
> ### Restart behaviour
> | Event | What happens | Gate outcome |
> |---|---|---|
> | Planner restarts, terrain unchanged | TL redelivers map T → INGESTED, `held_token=T`, new `instance_id` | **Releases.** Correct: the new planner really does hold T. A strictly-greater counter hangs here. |
> | Planner restarts during the wait | Same, plus `instance_id` change | Panel resets its patience budget **once**, then matches. |
> | Planner dies and does not return | Cached receipt is stale | Release is additionally gated on a live `replan_fsm_drone_*` **evaluated at release time**. No planner ⇒ fail, no bypass (publishing to nobody is not a bypass). |
> | Planner dies, new one starts, panel armed for T | New FSM publishes `STATUS_NONE` in its constructor, overwriting the stale latch; then TL redelivery produces a genuine INGESTED(T) | Panel waits through the gap, then releases on the new planner's own receipt. |
> | Terrain publisher restarts | Republishes with a **new** stamp; planner ingests; receipt quotes the new token | Panel armed for old T holds, then fails closed. Correct: the map it asked for was replaced. |
> | Same corridor loaded twice | Fresh stamp ⇒ fresh token | Second Run cannot be released by the first Run's receipt. Geometry no longer identifies anything. |
>
> ---
>
> ## B. `setTerrainData` decision table — evaluation order is normative
>
> New signature: `TerrainIngestResult PathManager::setTerrainData(const grid_map_msgs::msg::GridMap::SharedPtr&)`.
>
> **Normative rule 1 — no mutation before the last validation gate.** `sdf_voxel_size_` is currently assigned at path_manager.cpp:3431-3439, *before* the elevation-layer search at 3441-3451, so rejects 4-8 (returning at 3450, 3455, 3460, 3468) already overwrote the SDF voxel pitch from the refused map — and `sdf_built_` is not reset, so the poisoning stays latent until the next `buildSDFForBounds` via `flushPendingObstacles()` or the geometry-change branch. r1's "held state unchanged" column was false for five of nine rows, and r1's own central rule ("read geometry out of `terrain_data_`, never out of `msg`") made the receipt *structurally blind* to it, since `sdf_voxel_size_` is not in `TerrainData`. **Fix: move the assignment into the ingest tail**, beside `terrain_data_.valid = true`. It needs no information the tail lacks. Rejects then genuinely have zero side effects and the column is true rather than asserted.
>
> **Normative rule 2 — the receipt's geometry block is read out of `terrain_data_` after the ingest, never out of `msg`.** This is the fix for the primary hole: today the FSM builds the receipt from `msg->info` (replan_fsm.cpp:1197-1202), which is why a map with a good `info` header and a truncated payload gets certified.
>
> | # | Condition (in order) | Result | reason | Held state after | Receipt |
> |---|---|---|---|---|---|
> | 0 | `msg == nullptr` | REJECTED | (internal) | unchanged | **none** — no token to answer; unreachable from rclcpp |
> | 1 | `layers.empty() && data.empty()` | CLEARED | NONE | DEM/SDF/dyn-obstacles wiped; `holds_terrain=false`; **`last_applied_token_ = subject_token`** | yes |
> | 2 | `!(info.resolution > 0)` | REJECTED | BAD_RESOLUTION | unchanged | yes |
> | 3 | `data.empty()` (layers non-empty) | REJECTED | NO_PAYLOAD | unchanged | yes |
> | 4 | no layer named `elevation` | REJECTED | NO_ELEVATION_LAYER | unchanged | yes |
> | 5 | `elev_idx >= data.size()` | REJECTED | LAYER_PAYLOAD_MISSING | unchanged | yes |
> | 6 | `layout.dim.size() < 2` | REJECTED | BAD_LAYOUT | unchanged | yes |
> | 7 | `cols==0 \|\| rows==0 \|\| cols > SIZE_MAX/rows` | REJECTED | DEGENERATE_DIMS | unchanged | yes |
> | 8 | `elev.data.size() < cols*rows` | REJECTED | PAYLOAD_TRUNCATED | unchanged | yes |
> | 9 | otherwise | INGESTED | NONE | new DEM; `sdf_voxel_size_` updated here; `last_applied_token_ = subject_token` | yes |
>
> Cases 2/3 are one test today (line 3428) and 7/8 are one test today (line 3465); splitting costs nothing and separates "the publisher produced a broken map" from "the publisher produced an empty non-clear" — different bugs. All six early returns keep their existing `warnf`; only the return value is added.
>
> **The INGESTED receipt is returned after the SDF build and `rebuildTerrainRiskMasks()` (past line 3607).** The receipt must not be publishable before the digest the gate exists to wait for is finished.
>
> **Normative rule 3 — `held_token` is `last_applied_token_`, a PathManager member that survives `terrain_data_ = TerrainData{}`.** `heldMapToken()` returns it unconditionally; `holds_terrain` is `terrain_data_.valid`. **This is the fix for finding 2**, which is a guaranteed deterministic failure of r1's own documented flow: r1 put the token in `TerrainData::source_token` and defined `heldMapToken()` as `valid ? source_token : 0`, while the clear branch does `terrain_data_ = TerrainData{}`. So LoadMap(world=none) → token C → receipt CLEARED/held_token=C → panel calls it "the success case", releases, publishes `expected_map_token = C` → §D compares C against `heldMapToken()` = 0 → TERRAIN_MAP_MISMATCH. The gate certified a state and the admission rule refused the command it released. With `last_applied_token_`, C == C and the world=none flow works. `TerrainData::source_token` is dropped — one member, one meaning.
>
> `finite_cells == 0` is **not** a rejection. An all-water crop is a legal map; the receipt reports it and the panel warns while still releasing.
>
> Publishing on REJECTED is a change from today (a malformed map currently emits nothing) and is safe **only because** the receipt carries the held-state block and consumers gate on it — see §C.
>
> ---
>
> ## C. Panel gate table — armed with token T after a successful LoadMap
>
> **Normative rule 4 — the release key is `held_token`, never `subject_token`.**
>
> This is the fix for finding 3, which r1 created by its own change. Today a malformed map emits nothing, so the last INGESTED receipt survives the depth-1 latch and the panel releases. Under r1, with the panel armed for T: planner ingests T (INGESTED, subject=T), then *any* later malformed GridMap arrives → the latch is overwritten with REJECTED(subject=M, holds_terrain=true, held_token=T). r1's table had exactly one applicable row — "`subject_token != T` → Wait" — so the panel waited out the full budget and then failed, on a planner whose own receipt said `held_token == T`. r1 advertised a verdict/state duality that its only consumer did not consume.
>
> | Observed | Panel action |
> |---|---|
> | No receipt ever **and** no publisher on `/planning/terrain_ingest` | **Fail immediately** — do not burn 30 s. "planner publishes no ingestion receipt (stale build or message-type skew)". Bypass offered. |
> | No receipt ever, publisher exists | Wait. "planner is up but has ingested no terrain yet". |
> | `holds_terrain && held_token == T` | **RELEASE.** Publish with `expected_map_token = T`. (Whatever `status`/`subject_token` say — the planner is stating what it holds.) |
> | Same, and `finite_cells == 0` | Release, **amber**: "ingested, but the map has no terrain data (all water/nodata)". |
> | `!holds_terrain && held_token == T` | Release **iff** the request was `world=none` (the clear succeeded). Otherwise fail. |
> | `status == REJECTED && subject_token == T` | **Fail immediately.** Red, with `reason` text + `detail`. Run re-enabled. **No bypass** — the planner answered; there is nothing to override. (Justified by `retryable == false`; if a future receipt sets it true, offer retry instead.) |
> | `held_token != T` (any status) | Wait. Status names what the planner *does* hold, from the held block. |
> | `instance_id` changed while waiting | Reset the patience budget **once**; keep waiting. |
> | Budget expired, publisher exists, planner in graph | **Fail.** "planner never confirmed map <T>; it holds <held geometry>". Bypass offered. |
> | Any release decision while no `replan_fsm_drone_*` is in the graph | Fail. No bypass. |
> | `drone_id != 0` | Ignore the receipt entirely. |
> | LoadMap reply `map_token == 0` | **Fail closed.** Red: "terrain server predates the ingestion contract — the gate is unavailable". Run re-enabled, bypass offered. **Never publish silently.** |
>
> That last row is the fix for finding 6. r1's migration prose said a half-migrated server makes the gate "unavailable (loudly) rather than silently open", while r1's code spec said "treat it exactly like the existing `expect_res_u_ <= 0` skip path" — and that path (mission_config_panel.cpp:320-326) calls `publish(pending_mission_)` and reports non-error. So the only surviving fail-open was triggered by precisely the partial rollout the migration section said had "no partial-rollout story", and the command it emitted carried `expected_map_token = 0` ("no requirement"), so the FSM would not catch it either. Both ends disarmed by one skew. The skip path is **deleted**, not reused.
>
> **Normative rule 5 — every graph query goes through one guarded helper.** `template <class F> bool MissionConfigPanel::graphQuery(F&&)`: `if (!rclcpp::ok()) return false;` + `try/catch(...)`. `pathManagerUp()` (mission_config_panel.cpp:556-578) and the new `count_publishers` call both route through it. The existing comment there records a deterministic `std::terminate` reproduced under valgrind — the readiness QTimer keeps firing after the SIGINT handler invalidates the rcl context, `get_node_names()` throws, and an uncaught throw in a Qt slot terminates. r1 added a second graph query in a second QTimer slot and never mentioned the guard, reintroducing the exact abort that comment exists to prevent.
>
> Geometry from the LoadMap reply is **demoted to a cross-check**: if the token matches but `resolution_u`/origin/length disagree with the reply, log a loud warning — it should be impossible, and if it happens you want to know rather than silently trust either side.
>
> The "don't clear the cached receipt" hack (panel 330-346, and the 33 s measurement behind it) stops being a hack: a pre-arm receipt quoting T can only exist because *this* request created T, so accepting an early receipt is safe **by construction** rather than by an argument about corner+resolution+extent aliasing.
>
> QoS stays depth-1 TL reliable both ends. A dropped intermediate receipt cannot cause a false release: the only receipt that releases states `held_token == T`, and any newer receipt overwriting it means the planner moved to a different map, where holding is correct.
>
> ---
>
> ## D. FSM admission rule for `expected_map_token`
>
> **Normative rule 6 — one insertion point: after the `drone_id` filter, after the dedup gate, before any state mutation.**
>
> Fix for finding 7: r1's rules section and code section named two different places. Dedup must come first — otherwise a `ros2 bag play` of the command topic, or the panel's own re-publish, is terrain-checked and writes FAILED/TERRAIN_MAP_MISMATCH over the state of the mission that actually ran. And the check must precede every mutation, so the whole block moves above `last_received_sequence_ = msg->sequence` (line 990), above `current_mission_id_ = msg->mission_id`, and above the start-position adoption block at 1060-1075. A refusal then touches nothing and a corrected resend retries without needing the rollback path at all. **This is the same reordering the initial-state contract requires; do it once, in whichever branch lands first.**
>
> | `cmd.expected_map_token` | `heldMapToken()` | Action |
> |---|---|---|
> | 0 | anything | Plan (today's behaviour). Log `[TERRAIN-GATE] command states no map — planning on token %llu`. Publish `PlanStatus` normally at the plan tail. |
> | T | T | Plan. |
> | T | U ≠ T (including 0 = nothing ever applied) | **Refuse.** `PlanOutcome::FAILED`, `PlanReason::TERRAIN_MAP_MISMATCH`, log naming both tokens, **publish `PlanStatus`**, return before consuming the sequence. |
>
> `0 = no requirement stated` is load-bearing for the headless publishers in this tree (§migration) and is what makes an operator bypass self-documenting: a bypassed command is visibly a command that states no terrain.
>
> The `PlanStatus` publish on the refusal row is not decoration — it is the only thing that stops the refusal from reading as success (§2).
>
> ---
>
> ## E. Code plan — files and edits
>
> **NEW `path_manager/include/path_manager/terrain_ingest.h`** (mirrors `planning_result.h` in shape and placement doctrine):
> `enum class TerrainIngest { REJECTED, INGESTED, CLEARED };`, `enum class TerrainReject { … }` with values numerically identical to the msg's `REASON_*` and a comment in both files saying so, `struct TerrainIngestResult { … }`, static factories `ingested/cleared/rejected` in the `PlanResult::failedBecause` style.
>
> **`path_manager.h`** — include it; `TerrainIngestResult setTerrainData(...)` (line 399); new member `uint64_t last_applied_token_{0};` (**not** inside `TerrainData`); `uint64_t heldMapToken() const { return last_applied_token_; }` and `bool holdsTerrain() const { return terrain_data_.valid; }`; private `TerrainIngestResult heldStateResult(TerrainIngest, TerrainReject, std::string, uint64_t subject_token) const` filling the entire held block from `terrain_data_`, so no call site can hand-assemble it from `msg` again.
>
> **`path_manager.cpp` `setTerrainData` (3362)** — compute `subject_token` from `msg->header.stamp` once at the top; **move the `sdf_voxel_size_` block (3431-3439) into the ingest tail**; split 3428 into rows 2/3 and 3465 into rows 7/8; convert 3450/3455/3460/3468 to typed rejects; set `last_applied_token_` in both the clear branch and the ingest tail; add `uint64_t finite` to the existing alignment self-check double loop (3557-3575) — zero extra passes; `return heldStateResult(INGESTED, …)` after the risk-channel refresh at 3607.
>
> **`planning_result.h`** — append `TERRAIN_MAP_MISMATCH` after `INITIAL_STATE_UNSPECIFIED`, FAILED-only (safe under the documented priority rule): "the command named the terrain it was prepared for and the planner holds a different map (or none)."
>
> **`replan_fsm.h`** — publisher type at 122 becomes `TerrainIngestReceipt`; new `plan_status_pub_`; new `uint64_t instance_id_{0};`; rewrite the `[TERRAIN-READY]` comment block (119-122) for the verdict/state duality and the held_token gating rule.
>
> **`replan_fsm.cpp`**
> - Constructor: create `terrain_receipt_pub_` on `/planning/terrain_ingest` (`QoS(1).reliable().transient_local()`) and publish `STATUS_NONE` **before** creating `terrain_sub_`; set `instance_id_` from the node clock. **Corrected rationale** (finding 10): r1 justified this with a race that cannot occur — `node.cpp:11` constructs `ReplanFSM` and `node.cpp:14` calls `executor.spin()`, with no intra-constructor spin, so `terrainCallback` cannot run before the constructor returns in either order. The reorder is kept because it makes the latch's *content* well-defined from the first instant the topic exists — not because it fixes a race. The restart table's row 4 rests on the same correct fact (the publish is unconditional in the constructor), which is worth stating accurately since that row carries the whole restart argument.
> - `terrainCallback` (1166): call `setTerrainData`, translate the result field-for-field, stamp `drone_id_`/`instance_id_`, publish. **Delete 1188-1218 entirely** — the duplicated `ingested`/`cleared` predicates are the bug, and the "verbatim copy of setTerrainData's own clear test" comment is an admission that the predicate was cloned rather than returned.
> - `trajectoryCommandCallback` (965): the admission check per rule 6.
> - New `publishPlanStatus(outcome, reason, detail, seq, mission_id)`, called from the two existing `last_plan_outcome_` write sites (792-793, 894-895) and from the admission refusal.
>
> **`mission_config_panel.hpp`** — replace the five `ready_*` doubles with `struct Receipt`, `bool have_receipt_`, `Receipt last_receipt_`, `uint64_t expect_token_`, `uint64_t last_instance_seen_`, `bool budget_extended_once_`. Keep `expect_*` geometry doubles **only** as cross-check inputs and status text. **No `released_token_` member.** New `QPushButton* bypass_button_` (hidden by default). Rewrite the `[TERRAIN-READY]` comment block (140-149) — delete "Fail-open on timeout (old behavior + a loud status)".
>
> **`mission_config_panel.cpp`**
> - `onInitialize` (103-114): subscribe to `/planning/terrain_ingest` typed; the callback stores the whole receipt (the "res last" ordering comment at 111-113 exists only because the untyped decode was not atomic — it goes away). Subscribe to `/planning/plan_status`; a FAILED status for our `sequence` sets red with the reason text.
> - `pollCorridor` ready phase (215-246): replaced by §C, keyed on `held_token`. Geometry matching (219-231) deleted as the release key, re-added as a mismatch warning.
> - Load-success tail (307-348): store `expect_token_ = resp->map_token`; `map_token == 0` → fail closed per §C.
> - **`publish(const ParsedMission&, uint64_t expected_map_token)`** — the token becomes a required argument. Fix for finding 4: `publish()` is called from three sites with nothing distinguishing them (the un-gated path at 200, the gated release at 236, the skip path at 323) plus the new bypass; r1 specified `cmd.expected_map_token = released_token_` inside `publish()` with no assignment site and no reset point, so a gated Run releasing T1 followed by an un-gated Run would silently carry T1. A required parameter makes the omission a compile error. Sites pass: `0` (no corridor), `expect_token_` (gated release), `0` (bypass).
> - New `onBypassClicked()`; `refreshRunButtonEnabled()` hides the bypass on any state change.
> - `setStatus` gains a third amber state so "ingested but all-water" is neither green nor red.
> - New `graphQuery()` helper; both graph calls routed through it.
>
> **`terrain_publisher.py`** — `self._last_map_token = 0`; new `_stamp(msg)` helper implementing the strict-monotone rule, called from `build_grid_map` (~522) and the `world=none` branch of `load_tif` (~440), replacing both direct `header.stamp` assignments — one place mints the token; `load_map_callback` sets `response.map_token = self._last_map_token` after `self.pub.publish(grid_map)` (line 187), safe because the node runs a single-threaded executor so no second load can interleave between build and reply (worth stating in the comment — it is the only invariant holding the two together).
>
> **`scripts/e2e_smoke.py`** — replace the `[DEM-GATE]` block (the `_gate_log0` / `os.path.getsize` log-offset scraping, the `MMP_E2E_SETTLE` sleep, and the "DEM cell: X" parse) with: subscribe to `/planning/terrain_ingest` (R+TL), spin until a receipt states `held_token == resp.map_token`, fail with a distinct exit code on REJECTED, then publish with `expected_map_token` set, then assert on `/planning/plan_status`. **Keep `MMP_E2E_REQUIRE_FINE`, asserting on `receipt.resolution_u`** — fix for finding 8: a token match proves the planner holds the map LoadMap published, and says nothing about whether that map is the 30 m crop or a ladder-coarsened 250 m one (LoadMap.srv's own comment: "the auto ladder may have coarsened resolution"). r1 deleted the only resolution assertion in the tree. Asserting on a typed field is strictly better than scraping the planner's log.
>
> **Docs** — `docs/review_handoff_20260811.md` §2.7 supersession note; new `docs/adr/0003-terrain-map-identity.md` ("map identity is owned by the map's publisher"), since the restart argument is the kind of reasoning that gets re-litigated.
>
> ---
>
> ## F. Un-updated publishers: what breaks and how it is detected
>
> Nothing here is wire-compatible — adding a field changes the type hash — so `mmp_mission_msgs`, `mmp_path_planning` and `mmp_rviz_plugins` rebuild and restart **together**. There is no partial-rollout story, and pretending otherwise produces a panel and a planner that see each other as "no publisher".
>
> - **Old terrain server** (no `map_token`): reply carries 0 → panel **fails closed** with "the gate is unavailable", Run re-enabled, explicit bypass. Never a silent publish.
> - **Old planner** (no receipt topic): panel's `count_publishers` sees zero → immediate fail with "stale build or message-type skew", not a 30 s burn.
> - **Old panel** (no receipt subscription): publishes `expected_map_token = 0` → the FSM plans, exactly as today. Detected only as an absence, which is the honest limit of a 0-means-no-requirement convention.
> - **Topic rename `/planning/terrain_ready` → `/planning/terrain_ingest`** is mandatory: reusing the name with a new type gives a silent no-match indistinguishable from "topic absent". Renaming makes `ros2 topic list` diagnostic and lets the panel honestly say "no planner publishes the receipt topic".
> - **`ros2 bag play` of `/terrain/grid_map`** replays an old stamp, so the planner emits a receipt quoting a token no live LoadMap reply can match. The gate holds and fails closed — correct; bag-driven replay must use the `expected_map_token = 0` path.
> - **`ab_difficulty_balance.py`** publishes via `ros2 topic pub -1` with a partial YAML dict; unspecified fields default, so `expected_map_token = 0` ⇒ plans as today. **This harness is why the 0 convention is mandatory rather than a nicety.**
> - **`map_selector_panel.cpp`** never touches the receipt or the command; it does cause planner restarts, which is exactly what token-based identity is designed to survive.
> - **`altitude_profile_panel` / `trajectory_metrics_panel`** subscribe to `/terrain/grid_map` only. Unaffected — but they matter indirectly: their 38.5 MB deserialization on rviz2's main thread starves the Qt timer and produced the measured 33 s pathology.
> - **`use_sim_time`**: nothing in the tree sets it today. Under sim time the publisher's stamps restart near 0 each run, so tokens are not unique *across* publisher restarts. The per-lifetime monotone guarantee holds; cross-lifetime collision is the residual (see human decisions).
> - **Multi-drone**: `/planning/terrain_ingest` is flat, matching the deliberate single-drone flat-topic choice (replan_fsm.cpp:161-164). `drone_id` makes the panel's filter correct today; the topic must move under a per-drone prefix when the swarm returns, together with `/mission/trajectory_command`.
>
> ---
>
> ## G. Findings judged NOT to require a design change
>
> None. All ten are real. The identity model is retained unchanged because no finding attacked it; finding 10 attacked only the *rationale* for one constructor edit, and the rationale is corrected in place.
>
> ## H. Known gaps that remain after r2
>
> **1. The receipt certifies ingestion, not usability.** A map can ingest perfectly and still be cropped short of the mission line, all NaN, or covering the wrong region. `finite_cells` and the new `resolution_u` assertion catch the crudest cases. Nothing says "this map contains the mission" — the panel has the mission bbox and could check. Deliberately out of scope; the obvious next hole.
>
> **2. Terrain only. Zones and obstacles have no receipt.** `rebuildTerrainRiskMasks()` depends on the DEM *and* `risk_zones_`. The current ordering argument for zones (panel publishes zones then command, reliable FIFO, shared MutuallyExclusive callback group — replan_fsm.cpp:229-249) does **not** cover a restarted planner collecting two TL latches in arbitrary order. A mission can still be released against a correct DEM and a half-applied zone set. A single "planner world state" receipt (terrain token + zone generation + obstacle generation) would close it.
>
> **3. The gate is a release gate with a by-construction opt-out.** `expected_map_token = 0` must mean "no requirement" for the CLI/A-B harnesses to survive, so enforcement is only as strong as publishers choosing to state a token. The guarantee is "a gated publisher cannot silently plan on the wrong map", not "nothing can".
>
> **4. Token uniqueness rests on a clock the design cannot audit.** Unique within one publisher lifetime under any clock. Across a publisher restart under sim time, stamps can repeat. The identity must be derivable from the GridMap itself (that is what removes the side channel) and `header.stamp` is the only field available — a real boundary, not an oversight.
>
> **5. The "planner died between receipt and release" window is bounded, not closed.** Release requires a live planner checked at release time, but the graph query and the publish are not atomic. `/mission/trajectory_command` is RELIABLE but VOLATILE, so a planner dying in that window takes the command with it. With `PlanStatus` latched, the panel now shows "no verdict for sequence N" instead of r1's flat "success" — improved, not eliminated.
>
> **6. `STATUS_CLEARED` correctness rests entirely on token uniqueness.** There is no geometry to cross-check a clear against.
>
> **7. The 30 s patience budget is still a guess.** Now fail-closed rather than fail-open, so being wrong costs a false failure instead of a silent wrong-DEM flight — a much better failure — but the budget does not scale with crop size and the digest it waits on (SDF + risk masks on a 9.2M-point corridor) demonstrably does.