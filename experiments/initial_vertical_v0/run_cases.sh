#!/usr/bin/env bash
# v0 사례 전부 실행. **비생산.**
#
# 기대 종료 코드를 여기 적어 두고 어긋나면 실패한다 — 사례가 조용히
# 성격을 바꾸면(양성대조가 거절로 뒤집히는 등) 여기서 걸린다.
#
#   0 인계 도달   1 미도달   2 실행 오류   3 계약 위반
set -u
cd "$(dirname "$0")"
BIN=./.build/v0_harness
SCRATCH=${TMPDIR:-/tmp}/v0_case.csv
fails=0

# JSBSim 은 공유 라이브러리이고, 바이너리에 박히는 RUNPATH 는 **설정 시점의
# 절대경로**다. 컨테이너에서 설정하면 /ws/... 가 박히므로 호스트 셸에서는
# 그대로 실행할 수 없다 — 사례 7개가 전부 127 로 끝나고, 그 127 이
# "기대와 다름"으로만 보여 원인이 가려진다.
#
# 그래서 스크립트가 저장소 상대 경로로 직접 찾는다. 못 찾으면 사례를 돌리기
# 전에 멈춘다: 재현 명령이 참이 아니면 결과도 참이 아니다.
JSBSIM_LIB=$(cd "../jsbsim_probe/.jsbsim/inst-v1.3.1/lib" 2>/dev/null && pwd)
if [ -z "${JSBSIM_LIB:-}" ] || [ ! -e "$JSBSIM_LIB/libJSBSim.so.1" ]; then
  echo "중단: libJSBSim.so.1 을 찾지 못했다." >&2
  echo "  기대 위치: experiments/jsbsim_probe/.jsbsim/inst-v1.3.1/lib" >&2
  echo "  JSBSim 을 먼저 빌드·설치하거나 LD_LIBRARY_PATH 를 직접 지정하라." >&2
  exit 2
fi
export LD_LIBRARY_PATH="$JSBSIM_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

if [ ! -x "$BIN" ]; then
  echo "중단: $BIN 이 없다. 먼저 빌드하라:" >&2
  echo "  cmake -S . -B .build -DJSBSIM_ROOT=<inst> -DMMP_DYN_ROOT=<dyn>" >&2
  echo "  cmake --build .build -j4" >&2
  exit 2
fi

# vertical 사례의 적용범위 집계를 **값으로** 단언한다. 출력만 하고
# 단언하지 않으면 집계 코드를 지워도 사례가 통과한다 — 그러면 이 수치는
# 영구 증거가 못 된다.
#
# 경계는 출처가 확정한 유효범위가 아니라 보수적으로 택한 진단 정책
# 경계다. 값이 바뀌면 여기서 깨지고, 그때 왜 바뀌었는지 적어야 한다.
EXPECT_APPLICABILITY=(
  "window=CL steps=60001 in_mach=12786 ok=106 best_s=0.2120 below=7604 above=5076"
  "window=Cm steps=60001 in_mach=12786 ok=126 best_s=0.2520 below=7584 above=5076"
)

assert_applicability() {
  local out="$1"
  for want in "${EXPECT_APPLICABILITY[@]}"; do
    if ! grep -qF "[APPLICABILITY] $want" <<<"$out"; then
      echo "!! 적용범위 집계가 기대와 다르다"
      echo "   기대: $want"
      echo "   실제: $(grep -F '[APPLICABILITY]' <<<"$out" | head -2)"
      fails=$((fails + 1))
      return
    fi
  done
  echo "적용범위 집계 2줄 단언 통과 (전 적분 스텝)"
}

run() {
  local expect="$1" name="$2"; shift 2
  echo
  echo "════════════════ $name  (기대 exit=$expect) ════════════════"
  "$BIN" --root . --case "$name" "$@" 2>&1 \
    | grep -vE '^$|JSBSim Flight|JSBSim-ML|startup beginning|aerodynamic axis|aerodynamic moment'
  local got=${PIPESTATUS[0]}
  if [ "$name" = "vertical" ]; then
    assert_applicability "$("$BIN" --root . --case vertical --tmax 120 \
      --csv "$SCRATCH" 2>&1)"
  fi
  if [ "$got" != "$expect" ]; then
    echo "!! 기대 exit=$expect 인데 $got — 사례 성격이 바뀌었다"
    fails=$((fails + 1))
  else
    echo "exit=$got (기대와 일치)"
  fi
}

# ── 측정 대상 ────────────────────────────────────────────────
run 1 vertical --tmax 120 --csv v0_vertical.csv

# ── 진단: 6DOF 준비 7항목은 통과하는데 중기가 거절하는 두 경로 ──
# 7항목이 필요조건일 뿐이라는 것을 서로 다른 **이름 있는 사유**로 보인다.
run 1 seven_pass_speed_ceiling --ic-gamma-deg 25 --tmax 60 \
    --csv "$SCRATCH"
run 1 seven_pass_thrust_reject --ic-gamma-deg 2.0 --gamma-target-deg 0.0 \
    --ic-alt-m 1000 --ic-speed-mps 170 --thrust-n 3200 --tmax 20 \
    --csv "$SCRATCH"

# ── 진짜 양성대조 ────────────────────────────────────────────
# 결합 관문이 늘 거절하는 장식이 아님을 보인다: 중기 모델이 받아들이는
# PVA 를 6DOF 가 실제로 지나가는 동안 두 단계가 함께 성립한다.
run 0 true_positive --ic-gamma-deg 2.0 --gamma-target-deg 0.0 \
    --ic-alt-m 1000 --ic-speed-mps 170 --thrust-n 0 --tmax 20 \
    --csv v0_positive.csv

# ── 음성대조 ─────────────────────────────────────────────────
run 3 neg_missing_param --negative missing_param --tmax 5 --csv "$SCRATCH"
run 1 neg_weak_control --negative weak_control --ic-gamma-deg 2.0 \
    --gamma-target-deg 0.0 --ic-alt-m 1000 --ic-speed-mps 170 --thrust-n 0 \
    --tmax 20 --csv "$SCRATCH"
run 3 neg_nonfinite --negative nonfinite --ic-gamma-deg 2.0 \
    --gamma-target-deg 0.0 --ic-alt-m 1000 --ic-speed-mps 170 --thrust-n 0 \
    --tmax 20 --csv "$SCRATCH"

echo
if [ "$fails" -eq 0 ]; then
  echo "═══ 모든 사례가 기대한 종료 코드로 끝났다 ═══"
else
  echo "═══ $fails 개 사례가 기대와 다르다 ═══"
fi
exit "$fails"
