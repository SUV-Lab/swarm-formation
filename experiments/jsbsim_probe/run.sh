#!/usr/bin/env bash
# [CONTRACT-2 EVAL] Fetch the pinned JSBSim, build the probes, run them.
# Everything lands under .jsbsim/ next to this script (gitignored).
#
#   ./run.sh                     # pinned version, f16
#   ./run.sh --version v1.2.1    # another tag
#   ./run.sh --model X15         # another bundled example aircraft
set -euo pipefail

# Pin rationale (README.md): v1.3.1 is the current stable release and the
# probe's numbers are recorded against it. v1.2.1 was the first version
# measured; both build and run identically here, so there is no reason to
# hold the older one.
VERSION="v1.3.1"
MODEL="f16"
DURATION="12"
JOBS="$(nproc 2>/dev/null || echo 4)"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --version) VERSION="$2"; shift 2 ;;
    --model)   MODEL="$2";   shift 2 ;;
    --duration) DURATION="$2"; shift 2 ;;
    -h|--help) sed -n '2,10p' "$0"; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORK="$HERE/.jsbsim"
# EVERY per-version artifact is suffixed (review find: a shared inst/ meant
# "--version v1.2.1" after a v1.3.1 run silently reused the NEWER library
# with the OLDER aircraft data — a cross-version chimera, not a re-check).
SRC="$WORK/src-$VERSION"
INST="$WORK/inst-$VERSION"
PROBE_BUILD="$WORK/build-$VERSION"

mkdir -p "$WORK"
if [[ ! -d "$SRC" ]]; then
  echo "[jsbsim] fetching $VERSION"
  git clone --depth 1 --branch "$VERSION" \
    https://github.com/JSBSim-Team/jsbsim.git "$SRC"
fi
# Tag AND resolved SHA, so a re-run years later can prove it used the same
# code even if the tag were moved.
SHA="$(git -C "$SRC" rev-parse HEAD)"
echo "[jsbsim] $VERSION @ $SHA"
if [[ ! -f "$INST/lib/libJSBSim.so" ]]; then
  echo "[jsbsim] building $VERSION ($JOBS jobs)"
  cmake -S "$SRC" -B "$SRC/build" -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_PYTHON_MODULE=OFF -DBUILD_SHARED_LIBS=ON \
        -DCMAKE_INSTALL_PREFIX="$INST" >/dev/null
  cmake --build "$SRC/build" -j "$JOBS" >/dev/null
  cmake --install "$SRC/build" >/dev/null
fi

echo "[probe] building against $VERSION"
cmake -S "$HERE" -B "$PROBE_BUILD" -DJSBSIM_ROOT="$INST" >/dev/null
cmake --build "$PROBE_BUILD" -j "$JOBS" >/dev/null

# $SRC is the aircraft/engine/systems data root. No LD_LIBRARY_PATH: RPATH
# is baked in by CMakeLists (per-version, since JSBSIM_ROOT differs).
echo
"$PROBE_BUILD/jsbsim_probe" "$SRC" "$MODEL" "$DURATION"
echo
"$PROBE_BUILD/hermite_check" "$SRC" "$MODEL" "$DURATION"
