#!/bin/bash
# Pin the isolated measurement workspace at a committed sha and build it there.
#
# Measurement runs from a workspace that the dev tree cannot reach into. Three
# rounds were thrown away before this was written down, each to a different
# leak: a stale binary after a branch switch, scenario yamls read from /ws
# while the binary was pinned, and terrain data resolved through MMP_WS —
# which defaults to /ws — no matter what the pinned src/ looked like.
#
# The source tree is REPLACED from git archive, never patched, so the
# workspace can hold only what the commit holds. The scenario yamls are
# snapshotted here because pin time is the one moment they may change; the
# rasters stay symlinked (2.4 GB of immutable input) and the harness records
# their size, mtime and realpath in meta.json instead.
#
#   ./pin_measurement_ws.sh <sha> [workspace]
set -e
SHA=${1:?usage: pin_measurement_ws.sh <sha> [workspace]}
AB=${2:-/home/user/ab}
SRC=${MMP_SRC:-/ws/src/mmp_path_planning}
TERRAIN=${MMP_TERRAIN:-/ws/src/mmp_terrain}
source /opt/ros/humble/setup.bash

rm -rf "$AB/src/mmp_path_planning"
mkdir -p "$AB/src/mmp_path_planning"
git -C "$SRC" archive "$SHA" | tar -x -C "$AB/src/mmp_path_planning"
git -C "$SRC" rev-parse "$SHA" > "$AB/PINNED_SHA"

DD=$AB/src/mmp_terrain/data
rm -rf "$DD"; mkdir -p "$DD"
for t in "$TERRAIN"/data/*.tif; do ln -sf "$t" "$DD/"; done
cp -a "$TERRAIN"/data/scenarios "$DD/"
cp -a "$TERRAIN"/data/risk_scenarios "$DD/" 2>/dev/null || true

for f in ab_difficulty_balance:ab ab_analyze:analyze ab_dry_assert:dry_assert \
         scenario_pub:scenario_pub; do
  cp "$AB/src/mmp_path_planning/scripts/${f%%:*}.py" "$AB/${f##*:}.py"
done

cd "$AB"
rm -rf build/path_manager install/path_manager
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -4

# The pin is worth nothing unless the built binary carries what is being
# measured, so say so out loud rather than assuming the build ran.
BIN=$AB/install/path_manager/lib/path_manager/path_manager_node
for s in ZONE-AUDIT PLAN-MODE STITCH-GATE; do
  printf '  %-12s %s\n' "$s" "$(strings "$BIN" | grep -c "$s")"
done
echo "PINNED $(cat "$AB/PINNED_SHA")"
