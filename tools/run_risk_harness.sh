#!/bin/bash
# Canonical invocation of the risk_scenarios_test harness (the "154/0" gate).
#
# The harness resolves its fixture RELATIVE TO CWD
# ("src/mmp_terrain/data/risk_scenarios/synthetic_flat.yaml"), and its argv is
# positional: yaml, alpha, h_weight, front_end, fm2_k, fm2_star, barrier.
# That tribal knowledge lived in throwaway scratch scripts; this pins it.
#
# Run inside the dev container from anywhere:
#   bash src/mmp_path_planning/tools/run_risk_harness.sh [outfile]
# Exit code is the harness exit code. Output goes to stdout and, when an
# outfile is given, is tee'd there for baseline diffing:
#   bash .../run_risk_harness.sh /tmp/baseline_p0.txt
set -euo pipefail

WS="${MMP_WS:-/ws}"
[ -d "$WS/src" ] || WS="$(cd "$(dirname "$0")/../../.." && pwd)"
# Self-contained: source ROS + the overlay so the binary's rosidl typesupport
# resolves even from a bare shell (set -u off around setup scripts — they
# reference unset vars).
set +u
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
set -u
BIN="$WS/install/path_manager/lib/path_manager/risk_scenarios_test"
FIXTURE="$WS/src/mmp_terrain/data/risk_scenarios/synthetic_flat.yaml"
[ -x "$BIN" ] || { echo "harness not built: $BIN (colcon build --packages-up-to path_manager)" >&2; exit 2; }
[ -f "$FIXTURE" ] || { echo "fixture missing: $FIXTURE" >&2; exit 2; }

# Defaults = the harness's own defaults, made explicit and absolute:
# full alpha sweep {0.1,0.3,1.0,3.0,10.0} x fm2 front-end. cwd is pinned to the
# workspace root so the relative default inside the binary would also resolve.
cd "$WS"
OUT="${1:-}"
if [ -n "$OUT" ]; then
  "$BIN" "$FIXTURE" | tee "$OUT"
else
  "$BIN" "$FIXTURE"
fi
