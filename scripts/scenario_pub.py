#!/usr/bin/env python3
"""Publish a scenario yaml the way the RViz ObstacleScenario panel does.

Headless verification kept measuring the "zone missions" with no zones in
them: risk zones and dynamic obstacles do not live in the mission yaml, they
live in scenarios/obstacles/*.yaml and reach the planner only because the
panel publishes them. A smoke test that skips this step gets
`setRiskZones: 0 zones` and silently measures a different problem.

Emits the two `ros2 topic pub -1` command lines the panel's Load button
sends, in the same order (zones first, then obstacles — the planner installs
the zone set before the SDF patches reference it).

Unit convention is the panel's, verified against
mmp_rviz_plugins/src/obstacle_scenario_panel.cpp:
  risk_zones : flat [x, y, z, reach, peak] * n, all in FRAME UNITS
  obstacles  : center xy in FRAME UNITS; center z and every physical
               dimension in METRES, converted by 1/100 (kMetresToUnits)

Usage:
  scenario_pub.py <scenario.yaml> [--zones-only] [--obstacles-only]
  eval "$(scenario_pub.py .../r7_goal_ring.yaml)"
"""
import sys
import yaml

M_TO_U = 0.01  # kMetresToUnits, obstacle_scenario_panel.cpp


def zones_msg(root):
    flat = root.get("risk_zones") or []
    if len(flat) % 5:
        raise SystemExit(f"risk_zones length {len(flat)} is not a multiple of 5")
    z = [flat[i:i + 5] for i in range(0, len(flat), 5)]
    body = ", ".join(
        "{center: {x: %g, y: %g, z: %g}, reach: %g, peak: %g}" % tuple(e) for e in z
    )
    return len(z), "{replace: true, zones: [%s]}" % body


def obstacles_msg(root):
    specs = []
    for node in root.get("dynamic_obstacles") or []:
        t = node.get("type", "")
        if t not in ("sphere", "box"):
            continue  # panel skips unknown kinds silently
        cx, cy, cz = node["center"]
        if t == "box":
            sx, sy, sz = node["size"]
            if min(sx, sy, sz) <= 0.0:
                raise SystemExit("box entry has non-positive size")
            specs.append(
                "{kind: 1, center: {x: %g, y: %g, z: %g}, radius: 0.0, "
                "size: {x: %g, y: %g, z: %g}, model: '%s'}"
                % (cx, cy, cz * M_TO_U, sx * M_TO_U, sy * M_TO_U, sz * M_TO_U,
                   node.get("model", ""))
            )
        else:
            r = float(node.get("radius", 0.0))
            if r <= 0.0:
                raise SystemExit("sphere entry has non-positive radius")
            specs.append(
                "{kind: 0, center: {x: %g, y: %g, z: %g}, radius: %g, "
                "size: {x: 0.0, y: 0.0, z: 0.0}, model: '%s'}"
                % (cx, cy, cz * M_TO_U, r * M_TO_U, node.get("model", ""))
            )
    return len(specs), "{replace: true, obstacles: [%s]}" % ", ".join(specs)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    flags = {a for a in sys.argv[1:] if a.startswith("--")}
    if len(args) != 1:
        raise SystemExit(__doc__)
    root = yaml.safe_load(open(args[0])) or {}

    nz, zm = zones_msg(root)
    no, om = obstacles_msg(root)
    print(f"# {args[0]}: {nz} zones, {no} obstacles", file=sys.stderr)

    if "--obstacles-only" not in flags:
        print("timeout 60 ros2 topic pub -1 /mission/risk_zones "
              f"mmp_mission_msgs/msg/RiskZoneArray \"{zm}\" >/dev/null 2>&1")
    if "--zones-only" not in flags:
        print("timeout 60 ros2 topic pub -1 /mission/obstacles "
              f"mmp_mission_msgs/msg/DynamicObstacleArray \"{om}\" >/dev/null 2>&1")


if __name__ == "__main__":
    main()
