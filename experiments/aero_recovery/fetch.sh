#!/usr/bin/env bash
# ADR-0003 §4-2 ② 원문 회수. 비생산. 원문 PDF 는 커밋하지 않는다.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="$HERE/.src"; mkdir -p "$OUT"
URL="https://ntrs.nasa.gov/api/citations/20130003336/downloads/20130003336.pdf"
WANT="9e3b026b7684f9e118a1379cb228b8418728b140f5c106df3bb58b979a97d85d"
F="$OUT/CR-2012-217475.pdf"

curl -fsSL --max-time 300 "$URL" -o "$F"
GOT="$(sha256sum "$F" | cut -d' ' -f1)"
if [ "$GOT" != "$WANT" ]; then
  echo "FAIL: SHA-256 불일치 — 감사 문서가 다른 판을 근거로 삼게 된다" >&2
  echo "  기대 $WANT" >&2
  echo "  실제 $GOT" >&2
  exit 3
fi
echo "OK: $F  SHA-256 일치"
command -v pdftotext >/dev/null && pdftotext -layout "$F" "$OUT/full.txt" \
  && echo "OK: $OUT/full.txt ($(wc -l < "$OUT/full.txt")줄)"
