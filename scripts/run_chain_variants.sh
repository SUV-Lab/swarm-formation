#!/bin/bash
# Run every chain_experiment_test variant, one process each, and tally.
#
# The test binary takes ONE variant per invocation and mutates parameters as
# it goes, so the variants cannot share a process. There is no "all" switch —
# the list is derived from the source so it can never drift from what the
# binary actually accepts, which is how a new variant silently stopped being
# run before.
#
#   ./run_chain_variants.sh [workspace] [outdir]
# No `set -u`: the ROS setup scripts read unbound variables by design and
# would abort the run before a single variant started.
#
# The workspace defaults to this script's own location rather than a literal
# /ws, so ctest can invoke it with no arguments from a build directory. The
# literal is kept as the last resort for the container shell.
# A caller that passes a wrong path is corrected rather than obeyed: ctest
# drops an empty argument entirely, which shifts the positional args and made
# the outdir arrive here as the workspace.
WS=${1:-}
if [ -z "$WS" ] || [ ! -d "$WS/src" ]; then
  WS=$(cd "$(dirname "$0")/../../.." && pwd)
  [ -d "$WS/src" ] || WS=/ws
fi
OUT=${2:-/tmp/chain_variants}
SRC=$WS/src/mmp_path_planning
# The binary is supplied by CMake ($<TARGET_FILE:...>) so the sweep runs the
# artefact of the tree ctest was invoked in. It used to be hardcoded to
# $WS/install, which meant a ctest run over ANY other build tree silently
# swept the Release install instead: a sanitizer build reported 71/71 green
# while never executing an instrumented binary. Bare-shell callers still get
# the install path.
BIN=${3:-}
if [ -z "$BIN" ] || [ ! -x "$BIN" ]; then
  BIN=$WS/install/path_manager/lib/path_manager/chain_experiment_test
fi
[ -x "$BIN" ] || { echo "ABORT: test binary not found: $BIN" >&2; exit 2; }
echo "binary: $BIN"
CFG=src/mmp_path_planning/src/path_manager/config/optimizer_params.yaml

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
cd "$WS" || exit 1

rm -rf "$OUT"; mkdir -p "$OUT"
grep -oE 'if \(v == "[a-z0-9_]+"\)' \
  "$SRC/src/path_manager/test/chain_experiment_test.cpp" |
  sed 's/.*"\(.*\)".*/\1/' | sort -u > "$OUT/variants.txt"
total=$(grep -c . "$OUT/variants.txt")
echo "$total variants"
# Zero variants used to be a PASS: the tally ended with pass=0, total=0 and
# `[ "$pass" = "$total" ]` is true. A moved source file, a renamed variant
# idiom, or a bad workspace path would report a green sweep that ran nothing —
# which is the exact failure this script exists to prevent.
if [ "$total" -lt 40 ]; then
  echo "ABORT: only $total variants found in $SRC — the list is derived by" >&2
  echo "  grepping the test source, so this means the source moved or the" >&2
  echo "  variant idiom changed. Refusing to report a sweep that ran nothing." >&2
  exit 2
fi

: > "$OUT/tally.txt"
while read -r v; do
  [ -z "$v" ] && continue
  timeout 900 "$BIN" "$CFG" 3 "$v" > "$OUT/$v.log" 2>&1
  rc=$?
  n=$(grep -cE '^\[FAIL\]' "$OUT/$v.log")
  if [ $rc -eq 0 ] && [ "$n" = "0" ]; then
    echo "PASS $v" >> "$OUT/tally.txt"
  else
    echo "FAIL $v rc=$rc fails=$n" >> "$OUT/tally.txt"
  fi
done < "$OUT/variants.txt"

pass=$(grep -c '^PASS' "$OUT/tally.txt")
echo "DONE $pass/$total" | tee -a "$OUT/tally.txt"
[ "$pass" = "$total" ]
