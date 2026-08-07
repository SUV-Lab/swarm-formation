#!/usr/bin/env bash
# [CONTRACT-2 EVAL] Fetch the pinned JSBSim, build the probes, run them.
# Everything lands under .jsbsim/ next to this script (gitignored).
#
#   ./run.sh                     # pinned version, f16
#   ./run.sh --version v1.2.1    # another pinned tag
#   ./run.sh --model X15         # another bundled example aircraft
set -euo pipefail

# Pin rationale (README.md): v1.3.1 is the current stable release and the
# probe's numbers are recorded against it. Tag AND expected SHA are pinned —
# a moved tag fails loudly instead of silently measuring different code.
VERSION="v1.3.1"
declare -A PINNED_SHA=(
  [v1.3.1]="3b25f25e49b42d0489c04ac805674fc1450ca579"
  [v1.2.1]="9b95d1b5ccff59916c79a0e3eb8f548377910598"
)
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

# The cache may have been produced in another environment (review find: a
# container run as root left .jsbsim root-owned, and the host user's git
# then failed with "dubious ownership" and nothing could be rebuilt).
# Detect both problems up front with actionable messages.
mkdir -p "$WORK" 2>/dev/null || {
  echo "ERROR: cannot create $WORK" >&2
  echo "  The parent directory is not writable by $(id -un) ($(id -u))." >&2
  exit 1
}
if [[ ! -w "$WORK" ]]; then
  echo "ERROR: cache $WORK exists but is not writable by $(id -un) ($(id -u))." >&2
  echo "  It was probably created by another user (e.g. a root container shell)." >&2
  echo "  Fix ownership or remove it and re-run:" >&2
  echo "    sudo chown -R $(id -u):$(id -g) '$WORK'     # keep the cache" >&2
  echo "    sudo rm -rf '$WORK'                          # or rebuild from scratch" >&2
  exit 1
fi

if [[ ! -d "$SRC" ]]; then
  echo "[jsbsim] fetching $VERSION"
  git clone --depth 1 --branch "$VERSION" \
    https://github.com/JSBSim-Team/jsbsim.git "$SRC"
fi
# safe.directory: the clone may legitimately be owned by a different uid
# than the one running now (host vs container); git's dubious-ownership
# guard would otherwise fail rev-parse on a cache we made ourselves.
SHA="$(git -c safe.directory="$SRC" -C "$SRC" rev-parse HEAD)"
if [[ -n "${PINNED_SHA[$VERSION]:-}" ]]; then
  if [[ "$SHA" != "${PINNED_SHA[$VERSION]}" ]]; then
    echo "ERROR: $VERSION resolved to $SHA" >&2
    echo "       expected ${PINNED_SHA[$VERSION]} (pinned)." >&2
    echo "  The upstream tag moved or the cache is stale. Remove $SRC to refetch." >&2
    exit 1
  fi
  echo "[jsbsim] $VERSION @ $SHA (pin OK)"
else
  echo "[jsbsim] $VERSION @ $SHA (WARNING: no pinned SHA for this version)"
fi

if ! command -v cmake >/dev/null; then
  # No toolchain (typically the HOST, outside the dev container). A warm
  # cache is still runnable: the binaries carry $ORIGIN-relative RPATHs.
  if [[ -x "$PROBE_BUILD/jsbsim_probe" && -x "$PROBE_BUILD/hermite_check" ]]; then
    echo "[probe] cmake not found — reusing existing binaries (no rebuild)"
    echo
    "$PROBE_BUILD/jsbsim_probe" "$SRC" "$MODEL" "$DURATION"
    echo
    "$PROBE_BUILD/hermite_check" "$SRC" "$MODEL" "$DURATION"
    exit 0
  fi
  echo "ERROR: cmake not found and no prebuilt probes for $VERSION." >&2
  echo "  Build once inside the dev container (docker/run_docker.sh)." >&2
  exit 1
fi

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

# $SRC is the aircraft/engine/systems data root. No LD_LIBRARY_PATH: the
# binaries carry an $ORIGIN-relative RPATH, so they run from the container
# path (/ws/...) and the host path of the same bind mount alike.
echo
"$PROBE_BUILD/jsbsim_probe" "$SRC" "$MODEL" "$DURATION"
echo
"$PROBE_BUILD/hermite_check" "$SRC" "$MODEL" "$DURATION"
