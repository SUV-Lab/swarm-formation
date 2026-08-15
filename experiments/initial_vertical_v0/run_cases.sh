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

run() {
  local expect="$1" name="$2"; shift 2
  echo
  echo "════════════════ $name  (기대 exit=$expect) ════════════════"
  "$BIN" --root . --case "$name" "$@" 2>&1 \
    | grep -vE '^$|JSBSim Flight|JSBSim-ML|startup beginning|aerodynamic axis|aerodynamic moment'
  local got=${PIPESTATUS[0]}
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
