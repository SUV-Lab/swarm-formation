#!/usr/bin/env python3
"""Zone-loaded A/B for chain/difficulty_balance, run from a PINNED workspace.

Why this exists as a script rather than a shell loop: the previous two
measurement rounds were both invalidated by process errors, not by the
planner — one measured a stale binary after a branch switch, the other ran
the "zone missions" with no zones because risk zones reach the planner only
via /mission/risk_zones. Both are now structural: the workspace is pinned and
verified before anything runs, every row records the zone count the planner
actually installed so a zero can never pass as a measurement again, and the
scenario yamls are read from the pinned workspace and digested into meta.json
— pinning the binary is worth little while the inputs stay editable.

Emits one CSV row per run and keeps every planner log. Nothing is averaged
away: repetitions are kept individually so median/p95 can be recomputed and
so a single anomalous run stays visible.

  ./ab_difficulty_balance.py --ws /home/user/ab --reps 5 --out /home/user/ab_out
"""
import argparse
import csv
import hashlib
import json
import os
import re
import shutil
import signal
import subprocess
import time

# Obstacle scenarios name their own mission in a comment; that pairing is the
# authority, not a guess from the filename.
MISSIONS = [
    # (mission, scenario or None)
    ("r1_coastal_terrain_following", "r1_dynamic_surface_obstacles"),
    ("r2_high_altitude_probe", None),          # no scenario names this mission
    ("r2_ridge_crossing", "r2_ridge_gauntlet"),
    ("r3_mountain_terrain_following", "r3_dense_corridor"),
    ("r3_mountain_terrain_following", "r3_dense_overlap"),
    ("r3_mountain_terrain_following", "r3_goal_in_zone"),
    ("r3_mountain_terrain_following", "r3_mixed_terrain_zones"),
    ("r3_mountain_terrain_following", "r3_multiple_zones"),
    ("r3_mountain_terrain_following", "r3_nested_zones"),
    ("r3_mountain_terrain_following", "r3_single_large_zone"),
    ("r4_long_range_traverse", "r4_traverse_chain"),
    ("r5_extended_corridor", "r5_corridor_wall"),
    ("r5_transition_success_probe", "r5_corridor_wall"),
    ("r5_transition_reject_probe", "r5_corridor_wall"),
    ("r6_zone_slalom", "r6_slalom_gates"),
    ("r7_encircled_goal", "r7_goal_ring"),
]

FIELDS = [
    "rep", "arm", "mission", "scenario", "zones_installed", "obstacles_installed",
    "outcome", "reason", "plan_mode", "zone_pass", "plan_total_ms",
    "chain_total_ms", "frontend_ms", "max_solve_ms",
    "eikonal_ms", "grid", "ceiling_u", "geo_z_max_u", "pieces", "flight_s",
    "min_agl_u", "env_viol_pct", "env_peak_pct", "env_peak_limit",
    "risk_max", "risk_exposure_s", "hard_zone_contacts", "hard_zone_contact_s",
    "soft_zone_contacts", "zone_policy_measurable", "zone_sample_dt",
    "obstacles_expected", "obstacles_added", "obstacles_deferred",
    "obstacles_skipped", "retry_fallback", "seam_worst",
    "zones_in_search", "leaked_procs", "log", "log_src",
]

RX = {
    "chain_total_ms": r"=> TOTAL (\d+) ms",
    "frontend_ms": r"front-end (\d+) ms",
    "max_solve_ms": r"\(wall \d+, max (\d+)\)",
    "eikonal_ms": r"eikonal=([0-9.]+)ms",
    "grid": r"grid=([0-9x]+)",
    "ceiling_u": r"-> ceiling ([0-9.]+)",
    "geo_z_max_u": r"geo_z=\[[0-9.eE+-]+,\s*([0-9.eE+-]+)\]",
    "pieces": r"(\d+) pieces, [0-9.]+ s flight",
    "flight_s": r"\d+ pieces, ([0-9.]+) s flight",
    "min_agl_u": r"verdict: [A-Z]+ — AGL min ([0-9.]+) u",
    "env_viol_pct": r"env viol ([0-9.]+)%",
    "env_peak_pct": r"peak ([0-9.]+)%",
    "env_peak_limit": r"peak [0-9.]+% ([a-z ]+?)\)",
    # Two DIFFERENT events, and conflating them cost a dry run: the planner
    # logs loadRiskZones when it ACCEPTS a zone set off the topic, and
    # setRiskZones when a plan hands that set to the searcher. Waiting for
    # the second before publishing the mission waits for something that
    # cannot have happened yet.
    "zones_installed": r"loadRiskZones: replaced with (\d+) zones",
    "zones_in_search": r"setRiskZones: (\d+) zones",
    "zone_pass": r"\[ZONE-AVOID\] pass=(\d+)",
    "seam_worst": r"worst \|d\(P,V,A\)\| = ([0-9.eE+-]+)",
    # NOT zone_hard_contacts — that string does not exist in a planner log,
    # so the column it fed was empty in all 160 rows of the first sweep while
    # the document claimed zone contact was among the safety metrics. What
    # FINAL-EVAL actually reports is peak risk and time spent exposed.
    # From [ZONE-AUDIT], which carries the WHOLE-FLIGHT values. The bare
    # "risk max" string appears on every per-span line, so taking the last
    # match returned the final span — usually arrival — not the flight.
    "risk_max": r"\[ZONE-AUDIT\].* risk_max=([0-9.]+)",
    "risk_exposure_s": r"\[ZONE-AUDIT\].* risk_exposure_s=([0-9.]+)",
    # The actual question — did the delivered flight enter a hard zone —
    # measured by the planner against the same primitive the hard passes use.
    # risk_max/exposure are field statistics and cannot answer it.
    # ONE machine-readable line, parsed positionally. Deriving safety from
    # the ABSENCE of a warning word is fail-open by construction: a reworded
    # log or a dropped call both read as "measurable, no contact". A missing
    # [ZONE-AUDIT] line is a missing measurement and fails the run.
    "hard_zone_contacts": r"\[ZONE-AUDIT\] hard=(\d+)",
    "hard_zone_contact_s": r"\[ZONE-AUDIT\] hard=\d+ hard_s=([0-9.]+)",
    "soft_zone_contacts": r"\[ZONE-AUDIT\].* soft=(\d+)",
    "zone_policy_measurable": r"\[ZONE-AUDIT\].* measurable=(true|false)",
    "zone_sample_dt": r"\[ZONE-AUDIT\].* sample_dt=([0-9.]+)",
    # Common to BOTH planning modes. The chain-only "=> TOTAL n ms" is absent
    # on the direct-fallback path, which is why 10 successful rows had no time
    # at all in the first sweep.
    "plan_total_ms": r"Global trajectory planning took (\d+) ms",
    # Explicit rather than inferred from the absence of a chain line.
    "plan_mode": r"\[PLAN-MODE\] (direct|chain)",
}


def last(rx, text, cast=str, default=""):
    m = re.findall(rx, text)
    if not m:
        return default
    try:
        return cast(m[-1])
    except ValueError:
        return default


def sh(cmd, ws, timeout=180):
    """Run inside the pinned workspace only."""
    full = f"source /opt/ros/humble/setup.bash && source {ws}/install/setup.bash && {cmd}"
    return subprocess.run(["bash", "-lc", full], capture_output=True, text=True,
                          timeout=timeout)


def reap(name):
    """Kill every process whose cmdline matches, by PID, from Python.

    NOT `bash -lc "pkill -f <name>"`: that shell's own cmdline contains the
    pattern, so pkill kills the shell before it reaches the next statement.
    That is how 279 terrain_publisher processes accumulated during a run —
    the first pkill in a two-pkill line killed its own shell every time, the
    second never executed, and the survivors slowly starved the machine until
    zone installs and mission commands stopped being delivered at all.
    """
    me = str(os.getpid())
    out = subprocess.run(["pgrep", "-f", name], capture_output=True,
                         text=True).stdout.split()
    for pid in out:
        if pid == me:
            continue
        try:
            os.kill(int(pid), signal.SIGKILL)
        except (ProcessLookupError, ValueError, PermissionError):
            pass


def kill_group(proc):
    """Kill a detached launch and everything it spawned."""
    if proc is None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    except (ProcessLookupError, PermissionError):
        pass


def sh_bg(cmd, ws):
    """Launch a node and return immediately.

    subprocess.run(capture_output=True) blocks on a backgrounded child even
    with nohup, because the child inherits the pipes and they never close.
    Detach the session and give it no inherited streams; the command redirects
    its own output to a file.
    """
    full = f"source /opt/ros/humble/setup.bash && source {ws}/install/setup.bash && {cmd}"
    return subprocess.Popen(["bash", "-lc", full], stdin=subprocess.DEVNULL,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            start_new_session=True)


def wait_for(pred, timeout_s, poll=0.5):
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        if pred():
            return True
        time.sleep(poll)
    return False


def newest_log(logdir, after_ts):
    try:
        c = [os.path.join(logdir, f) for f in os.listdir(logdir)
             if f.startswith("path_manager_")]
    except FileNotFoundError:
        return None
    c = [f for f in c if os.path.getmtime(f) >= after_ts]
    return max(c, key=os.path.getmtime) if c else None


def run_one(ws, out, mission, scenario, arm, rep, missions_dir, obstacles_dir, logdir):
    for nm in ("path_manager_node", "terrain_publisher", "topic pub"):
        reap(nm)
    time.sleep(4)
    t_start = time.time()
    launched = []

    mpath = f"{missions_dir}/{mission}.yaml"
    import yaml as _y
    m = _y.safe_load(open(mpath))["mission"]
    s, g = m["start"], m["goal"]

    launched.append(sh_bg(
        f"exec ros2 run mmp_terrain terrain_publisher --ros-args -p world:=full_map "
        f"-p corridor_mission:={mpath} > /tmp/ab_terr.log 2>&1", ws))
    dbal = "true" if arm in ("on", "direct_on") else "false"
    # A direct-mode probe: an enormous per-segment target makes auto-N pick
    # the single-shot plan, which is the only way to exercise the direct
    # path's timing and its whole-flight gate on demand. Until that gate was
    # made unconditional this probe would have "passed" without any safety
    # evaluation at all.
    pieces_target = 100000 if arm.startswith("direct") else 35
    launched.append(sh_bg(
        f"exec ros2 run path_manager path_manager_node --ros-args "
          f"--params-file {ws}/install/path_manager/share/path_manager/config/optimizer_params.yaml "
          f"--params-file {ws}/install/path_manager/share/path_manager/config/drone_hardware.yaml "
          f"-p drone_id:=0 -p manager/world:=full_map "
          f"-p chain/difficulty_balance:={dbal} "
          f"-p chain/auto_pieces_per_segment:={pieces_target} "
        f"> /tmp/ab_pm.log 2>&1", ws))

    # Readiness poll instead of a fixed sleep: the planner must have INGESTED
    # the corridor, which is the state a fixed sleep only assumed.
    def ready():
        lg = newest_log(logdir, t_start)
        return lg is not None and "Terrain data received and forwarded" in open(
            lg, errors="ignore").read()
    def bail(outcome, reason=""):
        """Every early return must clean up, or the next run inherits it."""
        for h in launched:
            kill_group(h)
        for nm in ("path_manager_node", "terrain_publisher", "topic pub"):
            reap(nm)
        return {"rep": rep, "arm": arm, "mission": mission,
                "scenario": scenario or "", "outcome": outcome,
                "reason": reason, "log": ""}

    if not wait_for(ready, 240):
        return bail("NO_TERRAIN")

    n_zones = n_obs = 0
    if scenario:
        pub = subprocess.run(
            ["python3", os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                     "scenario_pub.py"),
             f"{obstacles_dir}/{scenario}.yaml"],
            capture_output=True, text=True)
        mm = re.search(r"(\d+) zones, (\d+) obstacles", pub.stderr)
        if mm:
            n_zones, n_obs = int(mm.group(1)), int(mm.group(2))

        # `ros2 topic pub -1` exits the moment it has published, and
        # TRANSIENT_LOCAL durability is served BY THE PUBLISHER — once it is
        # gone a subscriber that had not matched yet gets nothing, ever. The
        # planner is busy ingesting a ~38 MB corridor right here, so that race
        # is lost often: 12 of 30 scenario runs installed zero zones while the
        # run still looked healthy. Hold the publisher open and REQUIRE the
        # planner's own confirmation before the mission goes out; a run that
        # cannot confirm is not a datapoint, it is a failed setup.
        # CLEAR FIRST, and confirm the clear. A ghost publisher from an
        # earlier run put "loadRiskZones: replaced with 10 zones" into an r1
        # run whose scenario has 2 — the count was then read off the wrong
        # line and the run looked healthy. Chasing where that publisher came
        # from is the wrong fix; starting from a state the planner has just
        # CONFIRMED as empty makes any leftover irrelevant, because the
        # install below is then the only thing that can raise the count.
        clear = ('exec ros2 topic pub -r 2.0 /mission/risk_zones '
                 'mmp_mission_msgs/msg/RiskZoneArray '
                 '"{replace: true, zones: []}" >/dev/null 2>&1')
        ch = sh_bg(clear, ws)

        def zones_cleared():
            lg2 = newest_log(logdir, t_start)
            if not lg2:
                return False
            t = open(lg2, errors="ignore").read()
            m = re.findall(r"loadRiskZones: replaced with (\d+) zones", t)
            return bool(m) and m[-1] == "0"

        cleared = wait_for(zones_cleared, 60)
        kill_group(ch)
        reap("topic pub")
        if not cleared:
            return bail("ZONE_CLEAR_FAILED",
                        "planner never confirmed an empty zone set")
        time.sleep(2)

        holders = [sh_bg(line.replace("timeout 60 ros2 topic pub -1",
                                      "exec ros2 topic pub -r 0.5"), ws)
                   for line in pub.stdout.splitlines()]

        def zones_in():
            lg2 = newest_log(logdir, t_start)
            if not lg2:
                return False
            t = open(lg2, errors="ignore").read()
            return bool(re.search(rf"loadRiskZones: replaced with {n_zones} zones", t))

        def obstacle_counts():
            """(added, deferred, skipped, total) from the planner's own line."""
            lg2 = newest_log(logdir, t_start)
            if not lg2:
                return None
            t = open(lg2, errors="ignore").read()
            m = re.findall(r"loadObstacles: added=(\d+) deferred=(\d+) "
                           r"skipped=(\d+) \(total in msg=(\d+)\)", t)
            return tuple(int(x) for x in m[-1]) if m else None

        def obstacles_in():
            if n_obs == 0:
                return True
            c = obstacle_counts()
            # "the string appeared" is not the same as "they went in": a run
            # where 9 of 10 were rejected would pass that test.
            return (c is not None and c[3] == n_obs and c[2] == 0
                    and c[0] + c[1] == n_obs)

        installed = wait_for(zones_in, 90) and wait_for(obstacles_in, 60)
        for h in holders:
            kill_group(h)
        reap("topic pub")
        if not installed:
            return bail("SCENARIO_INSTALL_FAILED",
                        f"planner never accepted {n_zones} zones "
                        f"and {n_obs} obstacles off the topic")
        time.sleep(3)

    cmd = (
        '{drone_id: 0, mission_id: "%s", '
        "start_position: {x: %g, y: %g, z: %g}, target_position: {x: %g, y: %g, z: %g}, "
        'formation_type: "none", use_initial_velocity: %s, initial_speed: %g, '
        "initial_velocity: {x: %g, y: %g, z: %g}, use_initial_acceleration: %s, "
        "initial_acceleration: {x: %g, y: %g, z: %g}}"
        % (mission, s[0], s[1], s[2] / 100.0, g[0], g[1], g[2] / 100.0,
           str(m.get("use_initial_velocity", False)).lower(),
           m.get("initial_speed_mps", 0.0),
           *(m.get("initial_velocity_mps") or [0, 0, 0]),
           str(m.get("use_initial_acceleration", False)).lower(),
           *(m.get("initial_acceleration_mps2") or [0, 0, 0]))
    )
    sh(f"timeout 120 ros2 topic pub -1 /mission/trajectory_command "
       f"mmp_mission_msgs/msg/TrajectoryCommand \"{cmd}\" >/dev/null 2>&1", ws, timeout=150)

    lg = newest_log(logdir, t_start)

    def settled():
        t = open(lg, errors="ignore").read()
        return ("startMissionPlan COMPLETED" in t) or ("PLAN REJECTED" in t)
    wait_for(settled, 300)
    time.sleep(3)

    text = open(lg, errors="ignore").read()
    row = {"rep": rep, "arm": arm, "mission": mission, "scenario": scenario or "",
           "obstacles_installed": n_obs, "log": os.path.basename(lg)}
    for k, rx in RX.items():
        row[k] = last(rx, text, float if k.endswith(("_ms", "_u", "_pct", "_s")) else str)
    if not row.get("zones_installed"):
        row["zones_installed"] = 0
    row["retry_fallback"] = len(re.findall(r"CHAIN-RETRY|falling back|DEGRADED", text))
    if "PLAN REJECTED" in text:
        row["outcome"] = "REJECTED"
        row["reason"] = last(r"\[PLAN\] FAILED: (.{0,120})", text)
    elif "Successfully generated global trajectory" in text:
        row["outcome"] = last(r"FINAL-EVAL\] verdict: ([A-Z]+)", text, default="OK")
        row["reason"] = ""
    else:
        row["outcome"] = "UNKNOWN"
        row["reason"] = ""
    archived = f"{arm}_{mission}_{scenario or 'nozone'}_r{rep}.log"
    shutil.copy(lg, os.path.join(out, "logs", archived))
    oc = None
    if scenario and n_obs:
        m = re.findall(r"loadObstacles: added=(\d+) deferred=(\d+) "
                       r"skipped=(\d+) \(total in msg=(\d+)\)", text)
        oc = tuple(int(x) for x in m[-1]) if m else None
    row["obstacles_expected"] = n_obs
    row["obstacles_added"] = oc[0] if oc else ""
    row["obstacles_deferred"] = oc[1] if oc else ""
    row["obstacles_skipped"] = oc[2] if oc else ""
    row["log_src"] = os.path.basename(lg)   # planner's own timestamped name
    row["log"] = archived                   # what is actually in the archive
    for h in launched:
        kill_group(h)
    for nm in ("path_manager_node", "terrain_publisher", "topic pub"):
        reap(nm)
    # A leak here degrades every later run, so it is recorded per row rather
    # than discovered at the end by counting corpses.
    time.sleep(1)
    # pgrep never matches its own process, and this runs without a shell, so
    # the count is the true survivor count — no self-match to subtract.
    row["leaked_procs"] = sum(
        int(subprocess.run(["pgrep", "-cf", nm], capture_output=True,
                           text=True).stdout.strip() or 0)
        for nm in ("terrain_publisher", "path_manager_node"))
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ws", required=True, help="PINNED workspace (never the dev tree)")
    ap.add_argument("--reps", type=int, default=5)
    # The dry run used to be a hand-edited COPY of this file with a shorter
    # mission list and an extra arm. Two instruments means the gate passes on
    # one and the measurement runs on the other — so the subset is an
    # argument now and the copy is gone.
    ap.add_argument("--only", type=int, default=0,
                    help="use only the first N mission/scenario pairs (0=all)")
    # The direct-mode probe is not part of the comparison — it exists so the
    # measurement itself contains evidence that a single-shot product is
    # audited and labelled. One rep's worth is enough; running it in every
    # rep would inflate the sweep by half for no pairwise value.
    ap.add_argument("--direct-reps", type=int, default=1,
                    help="reps that also run the direct-mode probe arm")
    ap.add_argument("--out", required=True)
    # Inputs default INSIDE the pinned workspace. They used to default to
    # /ws — the dev tree — so the binary was pinned while the missions,
    # obstacles and log directory it read were still editable mid-measurement.
    # That is the same isolation hole the pin exists to close, one level down.
    ap.add_argument("--logdir", default=None)
    ap.add_argument("--missions", default=None)
    ap.add_argument("--obstacles", default=None)
    a = ap.parse_args()
    scen = os.path.join(a.ws, "src/mmp_terrain/data/scenarios")
    a.logdir = a.logdir or os.path.join(a.ws, "logs/runtime")
    a.missions = a.missions or os.path.join(scen, "missions")
    a.obstacles = a.obstacles or os.path.join(scen, "obstacles")
    for d in (a.missions, a.obstacles):
        if not os.path.isdir(d):
            raise SystemExit(f"scenario input missing from the pinned "
                             f"workspace: {d}")
        if os.path.realpath(d).startswith("/ws/"):
            raise SystemExit(f"scenario input resolves into the dev tree: {d}")

    os.makedirs(os.path.join(a.out, "logs"), exist_ok=True)
    sha = open(os.path.join(a.ws, "PINNED_SHA")).read().strip() \
        if os.path.exists(os.path.join(a.ws, "PINNED_SHA")) else "unknown"
    # The scenario yamls are measurement INPUTS; a run that cannot say which
    # bytes it fed the planner is not reproducible, so they are digested here
    # exactly like the source commit is.
    h = hashlib.sha256()
    for d in (a.missions, a.obstacles):
        for f in sorted(os.listdir(d)):
            h.update(f.encode())
            h.update(open(os.path.join(d, f), "rb").read())
    meta = {"pinned_sha": sha, "ws": a.ws, "reps": a.reps,
            "only": a.only, "direct_reps": a.direct_reps,
            "missions": a.missions, "obstacles": a.obstacles,
            "scenario_sha256": h.hexdigest(),
            "started": time.strftime("%Y-%m-%d %H:%M:%S")}
    json.dump(meta, open(os.path.join(a.out, "meta.json"), "w"), indent=2)

    missions = MISSIONS[:a.only] if a.only else MISSIONS
    print(f"{len(missions)} combos x {a.reps} reps, "
          f"direct probe in reps 1..{a.direct_reps}")

    csv_path = os.path.join(a.out, "runs.csv")
    with open(csv_path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=FIELDS, extrasaction="ignore")
        w.writeheader()
        for rep in range(1, a.reps + 1):
            for mission, scenario in missions:
                # Alternate, so a warm-up or drift effect cannot masquerade as
                # an arm effect in the timing columns.
                arms = ("off", "on") if rep % 2 else ("on", "off")
                if rep <= a.direct_reps:
                    arms = arms + ("direct_off",)
                for arm in arms:
                    try:
                        row = run_one(a.ws, a.out, mission, scenario, arm, rep,
                                      a.missions, a.obstacles, a.logdir)
                    except Exception as e:  # a crashed run is data, not a stop
                        row = {"rep": rep, "arm": arm, "mission": mission,
                               "scenario": scenario or "", "outcome": "HARNESS_ERROR",
                               "reason": str(e)[:120]}
                    w.writerow(row)
                    fh.flush()
                    print(f"[{rep}/{a.reps}] {arm:3s} {mission} / {scenario or '-'}"
                          f" -> {row.get('outcome')} zones={row.get('zones_installed')}",
                          flush=True)
    for nm in ("path_manager_node", "terrain_publisher", "topic pub"):
        reap(nm)
    print("AB_DONE", csv_path)


if __name__ == "__main__":
    main()
