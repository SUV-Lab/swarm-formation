#!/usr/bin/env bash
# 강제응답 시험 전체를 재현한다. 비생산.
#
#   ./run_forced.sh [T] [dt]      (출력 간격 = dt 고정)
#
# 표 1  동일 입력에서 AB2 대 AB4 (임펄스 미정규화)
# 표 2  실측 ZOH 임펄스를 step 에 맞춘 매끄러움 시험
#
# 두 표를 섞지 않는다 — 섞으면 "매끄러움 덕분"과 "자극량이 작아서"를
# 다시 가를 수 없다.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
T="${1:-6}"; DT="${2:-0.001}"
# 강제응답은 전 스텝 기록을 강제한다 — 일-에너지와 최악값이 성긴 격자에서
# 계산되면 순위가 바뀐다 (측정). 생성기도 out_dt != dt 를 거절한다.
ODT="$DT"
BIN="$HERE/.build/candidate_acc02"
export LD_LIBRARY_PATH="$HERE/../jsbsim_probe/.jsbsim/inst-v1.3.1/lib:${LD_LIBRARY_PATH:-}"
cd "$HERE"
W=$(mktemp -d); trap 'rm -rf "$W"' EXIT

row() {  # $1 프로파일  $2 이름  $3 rate  $4 att  $5 스케줄
  "$BIN" . "$DT" "$T" "$ODT" "$W/o.csv" "$3" "$4" "forced_$1" "$5" >/dev/null 2>&1
  python3 forced_reference.py "$W/o.csv.hex" "$W/o.csv.mom" "$DT" "$1" "$T"
}

echo "### 표 1 — 동일 입력에서 AB2 대 AB4 (T=$T, dt=$DT)"
printf "%-8s %-5s %12s %12s %12s %12s %14s\n" \
  "프로파일" "적분기" "각거리max" "RMS" "Δω max" "일-에너지" "ZOH임펄스"
for pr in step c0ramp c1ramp c2ramp; do
  python3 forced_profile.py "$pr" "$T" "$DT" > "$W/s.txt"
  for c in "AB2:3:7" "AB4:5:7"; do
    n=${c%%:*}; r=$(echo "$c"|cut -d: -f2); a=$(echo "$c"|cut -d: -f3)
    m=$(row "$pr" "$n" "$r" "$a" "$W/s.txt")
    printf "%-8s %-5s %12s %12s %12s %12s %14s\n" "$pr" "$n" \
      $(echo "$m"|tr ' ' '\n'|sed -E 's/.*=//'|sed -n '1p;2p;4p;7p;8p'|tr '\n' ' ')
  done
done

echo
echo "### 표 2 — 실측 ZOH 임펄스를 step 에 맞춤 (AB4/Buss2)"
printf "%-8s %12s %12s %12s %14s\n" \
  "프로파일" "각거리max" "RMS" "Δω max" "ZOH임펄스"
for pr in step c0ramp c1ramp c2ramp; do
  python3 forced_profile.py "$pr" "$T" "$DT" --normalize step > "$W/sn.txt"
  m=$(row "$pr" AB4 5 7 "$W/sn.txt")
  printf "%-8s %12s %12s %12s %14s\n" "$pr" \
    $(echo "$m"|tr ' ' '\n'|sed -E 's/.*=//'|sed -n '1p;2p;4p;8p'|tr '\n' ' ')
done
