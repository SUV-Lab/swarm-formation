#!/bin/bash
# [PA-2] Undefined-behaviour check, in a SEPARATE build tree.
#
# Why this exists: the Release regression cannot see an uninitialised member.
# Measured — deleting all four initialisers in poly_traj_optimizer.h
# (enable_obstacles_, enable_debug_logs_, enable_lbfgs_detail_logs_,
# drone_id_) leaves `colcon test` at 86/0/0 and the chain sweep at 71/71,
# while a UBSan build of the SAME code reports 13 diagnostics. A green suite
# is not evidence about this class of defect; only a sanitizer is.
#
# Usage (inside the dev container):
#   bash src/mmp_path_planning/tools/run_ubsan_check.sh [workspace]
# Exit 0 = zero sanitizer diagnostics. Exit 1 = diagnostics (listed).
# Exit 2 = the check could not run, which is NOT a pass.
#
# Builds into build_ubsan/ + install_ubsan/ so the Release tree is untouched:
# mixing sanitizer objects into build/ would silently change what every other
# regression measures.
set -u

WS=${1:-}
if [ -z "$WS" ] || [ ! -d "$WS/src" ]; then
  WS=$(cd "$(dirname "$0")/../../.." && pwd)
  [ -d "$WS/src" ] || WS=/ws
fi
cd "$WS" || { echo "workspace not found: $WS" >&2; exit 2; }

CFG=$WS/src/mmp_path_planning/src/path_manager/config/optimizer_params.yaml
[ -f "$CFG" ] || { echo "params not found: $CFG" >&2; exit 2; }

set +u
source /opt/ros/humble/setup.bash
# Dependencies resolve from the ORDINARY Release install; only the two
# packages under test are rebuilt with the sanitizer. UBSan works fine across
# an uninstrumented boundary, and instrumenting the whole workspace would
# multiply the build for no extra coverage of THIS defect class.
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
set -u
if [ ! -d "$WS/install/path_planner" ]; then
  echo "ABORT: the Release install is missing — build the workspace first." >&2
  echo "  This check instruments path_optimizer + path_manager only and" >&2
  echo "  takes every other package from install/." >&2
  exit 2
fi

echo "== [PA-2] UBSan build (separate tree: build_ubsan/) =="
# The log target has to exist before colcon writes to it.
mkdir -p "$WS/build_ubsan" || { echo "cannot create build_ubsan" >&2; exit 2; }
colcon build \
  --packages-select path_optimizer path_manager \
  --build-base "$WS/build_ubsan" --install-base "$WS/install_ubsan" \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DMMP_SANITIZE=ON \
  > "$WS/build_ubsan/build.log" 2>&1 || {
    echo "ABORT: sanitizer build failed — see build_ubsan/build.log" >&2
    tail -20 "$WS/build_ubsan/build.log" >&2
    exit 2
  }

BIN=$WS/build_ubsan/path_manager/chain_experiment_test
[ -x "$BIN" ] || { echo "ABORT: $BIN not built" >&2; exit 2; }

# The instrumentation has to actually BE there. A build that quietly dropped
# the flags would report zero diagnostics and read exactly like a pass.
# poly_traj_optimizer.cpp is where the members under test live, so its
# object file is the one that MUST carry the handlers — a run that only
# instrumented the harness would report zero and mean nothing.
OPT_OBJ=$(find "$WS/build_ubsan/path_optimizer" -name 'poly_traj_optimizer.cpp.o' 2>/dev/null | head -1)
[ -n "$OPT_OBJ" ] || { echo "ABORT: poly_traj_optimizer object not found" >&2; exit 2; }
for obj in "$BIN" "$OPT_OBJ" \
           "$WS/build_ubsan/path_manager/libpath_manager_lib.a"; do
  [ -e "$obj" ] || { echo "ABORT: missing $obj" >&2; exit 2; }
  if ! nm -C "$obj" 2>/dev/null | grep -q "__ubsan_handle"; then
    echo "ABORT: $(basename "$obj") carries no __ubsan_handle symbols —" >&2
    echo "  the sanitizer flags did not reach it, so a clean run would be" >&2
    echo "  meaningless." >&2
    exit 2
  fi
  echo "  instrumented: $(basename "$obj")"
done

# ASAN_OPTIONS is REQUIRED, not decoration. Without it the process aborts at
# exit 134 on an unrelated new-delete mismatch inside uninstrumented
# librcutils, BEFORE PathManager is constructed, producing zero UBSan output
# — which reads as "does not reproduce".
export ASAN_OPTIONS=new_delete_type_mismatch=0:detect_leaks=0
export UBSAN_OPTIONS=print_stacktrace=0

OUT=$WS/build_ubsan/ubsan_run.log
echo "== running chain_experiment_test under UBSan =="
( cd "$WS/build_ubsan" && timeout 900 "$BIN" "$CFG" 3 > "$OUT" 2>&1 )
rc=$?
if [ $rc -ne 0 ] && [ $rc -ne 1 ]; then
  echo "ABORT: harness exited $rc (not a pass/fail) — see $OUT" >&2
  tail -20 "$OUT" >&2
  exit 2
fi

n=$(grep -c "runtime error" "$OUT")
echo "== sanitizer diagnostics: $n =="
if [ "$n" != "0" ]; then
  grep "runtime error" "$OUT" | sed 's/^/  /' | head -20
  echo "FAIL: $n undefined-behaviour diagnostic(s)"
  exit 1
fi

# The single binary above is the fast signal. The ACTUAL automatic entry point
# is `colcon test` over the sanitized tree: MMP_SANITIZE puts halt_on_error in
# every registered test's environment, so any diagnostic fails the test that
# produced it rather than scrolling past on stderr.
echo "== colcon test over the sanitized tree =="
colcon test --build-base "$WS/build_ubsan" --install-base "$WS/install_ubsan" \
  --packages-select path_manager --event-handlers console_direct- \
  > "$WS/build_ubsan/test.log" 2>&1 || true
summary=$(colcon test-result --test-result-base "$WS/build_ubsan" 2>&1 | tail -3)
echo "$summary" | sed 's/^/  /'
if echo "$summary" | grep -qE "[1-9][0-9]* (error|failure)"; then
  echo "FAIL: sanitized colcon test reported failures"
  exit 1
fi
echo "PASS: 0 undefined-behaviour diagnostics; sanitized suite green"
exit 0
