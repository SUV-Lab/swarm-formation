#!/usr/bin/env bash
# NASA NESC 6DOF 검증 사례 자료 회수. 비생산 실험용.
#
# 받는 것은 전부 NESC Academy가 공개 배포하는 것이며 이 저장소에
# 커밋하지 않는다 (.gitignore). 사례 정의·초기조건·모델 상세는 부록
# 보고서에 전문이 있고, 기준 시계열은 사례별 페이지의 동적 표에서
# 제공된다 (README의 "남은 것" 참조).
set -euo pipefail
BASE="https://nescacademy.nasa.gov"
OUT="${1:-$(dirname "$0")/.nesc}"
mkdir -p "$OUT/spec" "$OUT/cases"

echo "[1/3] 보고서 — 사례 정의·모델 상세·초기조건"
for r in NASA-TM-2015-218675-EOM_checkcase_summary \
         NASA-TM-2015-218675-EOM_checkcase_appendices \
         aiaa-13-5071-EOM_chkcases \
         aiaa-15-1810-EOM_chkcases-II; do
  curl -fsSL --max-time 300 "$BASE/src/flightsim/Reports/$r.pdf" \
       -o "$OUT/spec/$r.pdf" && echo "  $r.pdf"
done

echo "[2/3] 사양 페이지 — 비행체·좌표계·중력·대기·출력"
for p in bodies datum_coordinate_system gravity atmosphere output_specs; do
  curl -fsSL --max-time 60 "$BASE/flightsim/2015/$p" \
       -o "$OUT/spec/$p.html" && echo "  $p"
done

echo "[3/3] 대기권 사례 페이지 (초기조건 포함)"
for c in acc01 acc02 acc03 acc04 acc05 acc06 acc07 acc08 acc09 acc10; do
  curl -fsSL --max-time 60 "$BASE/flightsim/2015/atmospheric/$c" \
       -o "$OUT/cases/$c.html" && echo "  $c"
done

cat <<'NOTE'

받지 않은 것: 기준 시계열 CSV.
사례 페이지의 "Latest Results" 표가 JS 번들로 채워지므로 정적 URL이
없다. README의 "남은 것"을 참조.
NOTE
