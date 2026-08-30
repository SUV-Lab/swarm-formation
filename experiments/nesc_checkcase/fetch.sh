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

echo "[3/4] 대기권 사례 페이지 (초기조건 포함)"
for c in acc01 acc02 acc03 acc04 acc05 acc06 acc07 acc08 acc09 acc10; do
  curl -fsSL --max-time 60 "$BASE/flightsim/2015/atmospheric/$c" \
       -o "$OUT/cases/$c.html" && echo "  $c"
done

echo "[4/4] 참여 도구 기준 시계열 CSV"
# 경로는 사이트의 매니페스트에서 뽑는다 — 손으로 적으면 조용히 낡는다.
DATES="$OUT/spec/dates.js"
curl -fsSL --max-time 60 "$BASE/workshop/FlightSim/js/2015/dates.js" -o "$DATES"

# acc02 = 회전 직육면체(무항력), acc03 = 공력 감쇠 추가. 자세 전파 검증용.
CSV_BASE="$BASE/workshop/FlightSim/2015"
: > "$OUT/manifest.txt"
total=0
for scn in 02 03; do
  mkdir -p "$OUT/ref/atmos_scn_$scn"
  paths=$(grep -oE "/atmos_scn_${scn}/Atmos_${scn}_sim_[0-9]+\\.csv" "$DATES" | sort -u)
  # 개수만 세면 {01,02,04,05,06} 이 아닌 다섯 개도 통과한다. 집합을 고정한다.
  got=$(printf '%s\n' "$paths" | sed -E 's#.*_sim_([0-9]+)\.csv#\1#' | sort | tr '\n' ',')
  if [ "$got" != "01,02,04,05,06," ]; then
    echo "FAIL: atmos_scn_$scn 시뮬레이터 집합이 '$got' — '01,02,04,05,06,' 기대" >&2
    exit 3
  fi
  for rel in $paths; do
    f="$OUT/ref$rel"
    curl -fsSL --max-time 120 "$CSV_BASE$rel" -o "$f"
    sz=$(wc -c < "$f")
    if [ "$sz" -lt 1000 ]; then
      echo "FAIL: $rel 가 $sz 바이트 — 내용 없음" >&2; exit 3
    fi
    # 헤더는 CSV 토큰으로 정확히 대조한다. 부분 문자열 일치는
    # eulerAngle_deg_Roll 을 찾을 때 eulerAngle_deg_RollRate 도 통과시킨다.
    # 참여 도구 5종의 열 구성이 27~38열로 서로 다르므로(실측), 아래는
    # 공통 부분집합 중 자세 전파 검증에 필요한 것만이다.
    python3 - "$f" <<'PYCHK' || exit 3
import csv, sys
need = {"time", "feVelocity_ft_s_X", "altitudeMsl_ft", "latitude_deg",
        "longitude_deg", "eulerAngle_deg_Roll", "eulerAngle_deg_Pitch",
        "eulerAngle_deg_Yaw", "bodyAngularRateWrtEi_deg_s_Roll",
        "bodyAngularRateWrtEi_deg_s_Pitch", "bodyAngularRateWrtEi_deg_s_Yaw"}
with open(sys.argv[1], newline="") as fh:
    hdr = next(csv.reader(fh))
cols = {c.strip() for c in hdr}
missing = need - cols
if missing:
    print(f"FAIL: {sys.argv[1]} 에 필수 열 없음: {sorted(missing)}", file=sys.stderr)
    raise SystemExit(3)
PYCHK
    sha=$(sha256sum "$f" | cut -d" " -f1)
    printf '%s  %s  %s\n' "$sha" "$CSV_BASE$rel" "$rel" >> "$OUT/manifest.txt"
    echo "  $(basename "$rel")  ${sz}B"
    total=$((total + 1))
  done
done
[ "$total" -eq 10 ] || { echo "FAIL: CSV $total개 (10개 기대)" >&2; exit 3; }

cat <<NOTE

기준 시계열 $total개 확보. URL과 SHA-256은 $OUT/manifest.txt 에 있고
원시 CSV는 커밋하지 않는다 (.gitignore).

주의 — 비교기가 다뤄야 하는 실측 사실:
  * 참여 도구 5종의 열 구성이 다르다 (27~38열). 공통은 25열이며 위치는
    지구고정 직교좌표가 아니라 고도·위경도로만 공통이다.
  * 출력 주기가 다르다 (대부분 302 표본, 하나는 3002). 공통 시각에서
    비교하거나 재표본해야 한다.
NOTE
