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
WS=${1:-/ws}
OUT=${2:-/tmp/chain_variants}
SRC=$WS/src/mmp_path_planning
BIN=$WS/install/path_manager/lib/path_manager/chain_experiment_test
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
