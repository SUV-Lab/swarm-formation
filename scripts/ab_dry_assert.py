#!/usr/bin/env python3
"""Gate a dry run before the full A/B is allowed to start.

Six sweeps have been thrown away in this project because a harness looked
healthy while measuring the wrong thing — a stale binary, zones that were
never installed, obstacles that were never confirmed, a metric whose log
string does not exist. Each was found by reading output afterwards. This
asserts the conditions instead, and a full run is only justified once it
passes:

  1. no required metric missing on a successful row
  2. zone and obstacle counts match the scenario exactly (added+deferred,
     nothing skipped)
  3. the hard-zone contact metric is present — the thing the flight is
     actually judged on, not a field statistic standing in for it
  4. no leaked processes on any row
  5. every archived log named in the CSV exists on disk
  6. the analyzer itself exits 0

Exit code is the number of failed checks.
"""
import csv
import os
import subprocess
import sys

REQUIRED = ["min_agl_u", "env_peak_pct", "env_viol_pct", "risk_max",
            "risk_exposure_s", "hard_zone_contacts", "zone_policy_measurable",
            "zone_sample_dt", "plan_total_ms"]
USABLE = ("CLEAN", "DEGRADED")


def main():
    if len(sys.argv) < 3:
        raise SystemExit("usage: ab_dry_assert.py <out_dir> <analyze.py>")
    out, analyzer = sys.argv[1], sys.argv[2]
    rows = list(csv.DictReader(open(os.path.join(out, "runs.csv"))))
    fails = []

    def check(ok, label, detail=""):
        print(f"  [{'OK ' if ok else 'FAIL'}] {label}" + (f" — {detail}" if detail else ""))
        if not ok:
            fails.append(label)

    print(f"시범 {len(rows)}행 검사")

    ok_rows = [r for r in rows if r["outcome"] in USABLE]
    check(bool(ok_rows), "성공 행이 하나 이상", f"{len(ok_rows)}행")

    holes = {}
    for r in ok_rows:
        mode = r.get("plan_mode") or ""
        for c in REQUIRED:
            if mode == "direct" and c in ("chain_total_ms", "frontend_ms"):
                continue
            if not (r.get(c) or "").strip():
                holes[c] = holes.get(c, 0) + 1
    check(not holes, "필수 지표 결측 0", str(holes) if holes else "")

    zbad = [r for r in rows
            if r["scenario"] and r["outcome"] in USABLE
            and r.get("zones_installed") in ("", "0", "-1")]
    check(not zbad, "구역 설치 확인", f"{len(zbad)}행 미설치" if zbad else "")

    obad = []
    for r in rows:
        if not r["scenario"] or r["outcome"] not in USABLE:
            continue
        exp = int(r.get("obstacles_expected") or 0)
        if exp == 0:
            continue
        add = r.get("obstacles_added")
        dfr = r.get("obstacles_deferred")
        skp = r.get("obstacles_skipped")
        if add == "" or skp == "" or int(skp) != 0 or int(add) + int(dfr) != exp:
            obad.append(f"{r['arm']}/{r['mission'][:16]} exp={exp} "
                        f"add={add} def={dfr} skip={skp}")
    check(not obad, "장애물 개수 일치 (added+deferred==expected, skipped==0)",
          "; ".join(obad[:3]) if obad else "")

    nocontact = [r for r in ok_rows
                 if r["scenario"] and not (r.get("hard_zone_contacts") or "").strip()]
    check(not nocontact, "하드 구역 접촉 지표 존재",
          f"{len(nocontact)}행 없음" if nocontact else "")

    # Presence is not the invariant. A successful flight must have entered NO
    # hard zone and must have had a policy that could be judged — otherwise
    # the row is not a datapoint about a safe flight, it is a bug report.
    hardhit = [f"{r['arm']}/{r['mission'][:18]}={r['hard_zone_contacts']}"
               for r in ok_rows if (r.get("hard_zone_contacts") or "0") not in ("0", "")]
    check(not hardhit, "성공 행의 하드 접촉 == 0", "; ".join(hardhit[:3]))

    unmeas = [f"{r['arm']}/{r['mission'][:18]}" for r in ok_rows
              if (r.get("zone_policy_measurable") or "true") != "true"]
    check(not unmeas, "성공 행의 구역 정책이 판정 가능", "; ".join(unmeas[:3]))

    modes = {r.get("plan_mode") for r in ok_rows}
    check("direct" in modes, "direct 모드가 최소 1행 (공통 시간 계측 검증)",
          f"모드: {sorted(m for m in modes if m)}")

    leaked = [r for r in rows if (r.get("leaked_procs") or "0") not in ("0", "")]
    check(not leaked, "프로세스 누수 0", f"{len(leaked)}행" if leaked else "")

    missing_logs = [r["log"] for r in rows
                    if r.get("log") and not os.path.exists(
                        os.path.join(out, "logs", r["log"]))]
    check(not missing_logs, "CSV의 아카이브 로그가 실제로 존재",
          f"{len(missing_logs)}개 없음" if missing_logs else "")

    rc = subprocess.run([sys.executable, analyzer,
                         os.path.join(out, "runs.csv")],
                        capture_output=True, text=True).returncode
    check(rc == 0, "분석기 종료코드 0", f"rc={rc}")

    print()
    if fails:
        print(f"시범 실패 {len(fails)}건 — 본 측정을 시작하면 안 된다:")
        for f in fails:
            print("   -", f)
    else:
        print("시범 통과 — 본 측정 가능")
    return len(fails)


if __name__ == "__main__":
    sys.exit(main())
