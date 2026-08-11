#!/usr/bin/env python3
"""Decide chain/difficulty_balance from the zone-loaded A/B, by the stated rule.

The rule was fixed before the data existed, so it cannot be bent to fit it:

    if ANY safety metric is worse with the flag on, it stays off,
    whatever it does for time.

Safety is minimum AGL (higher is better), envelope violation percentage and
peak utilisation (lower is better), hard-zone contacts (lower), and the
outcome itself — a combination that plans with the flag off and is REFUSED
with it on is the worst regression available and is reported first.

Everything is compared PAIRWISE within (rep, mission, scenario), because the
missions differ from each other by far more than the arms differ within one.
Rows that are not measurements — a failed zone install, a harness error — are
excluded and counted separately rather than silently averaged in.
"""
import argparse
import collections
import csv
import statistics as st

SAFETY_HIGHER_BETTER = ["min_agl_u"]
SAFETY_LOWER_BETTER = ["env_viol_pct", "env_peak_pct", "hard_zone_contacts"]
TIME = ["total_ms", "max_solve_ms", "frontend_ms"]
USABLE = ("CLEAN", "DEGRADED")


def num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def pct(vals, p):
    if not vals:
        return None
    s = sorted(vals)
    if p == 50:
        return st.median(s)
    k = max(0, min(len(s) - 1, int(round((p / 100.0) * (len(s) - 1)))))
    return s[k]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--tol", type=float, default=0.02,
                    help="relative tolerance below which a difference is noise")
    a = ap.parse_args()
    rows = list(csv.DictReader(open(a.csv)))

    print(f"총 {len(rows)}행")
    print("판정 분포:", dict(collections.Counter(r["outcome"] for r in rows)))
    invalid = [r for r in rows if r["outcome"] not in USABLE + ("REJECTED",)]
    if invalid:
        print(f"측정 아님(제외): {len(invalid)}행",
              dict(collections.Counter(r["outcome"] for r in invalid)))
    leaked = [r for r in rows if (r.get("leaked_procs") or "0") not in ("0", "")]
    print(f"프로세스 누수 있는 행: {len(leaked)}")
    zbad = [r for r in rows if r["scenario"] and r["zones_installed"] in ("0", "", "-1")]
    print(f"시나리오 있는데 구역 미설치: {len(zbad)}")
    print()

    # --- worst regression first: outcome flips ------------------------------
    by_key = collections.defaultdict(dict)
    for r in rows:
        by_key[(r["rep"], r["mission"], r["scenario"])][r["arm"]] = r
    flips = []
    for k, v in by_key.items():
        if "off" not in v or "on" not in v:
            continue
        off_ok = v["off"]["outcome"] in USABLE
        on_ok = v["on"]["outcome"] in USABLE
        if off_ok and not on_ok:
            flips.append((k, v["off"]["outcome"], v["on"]["outcome"], "on이 잃음"))
        elif on_ok and not off_ok:
            flips.append((k, v["off"]["outcome"], v["on"]["outcome"], "off가 잃음"))
    print("=== 결과 뒤집힘 (한쪽만 계획에 성공) ===")
    if flips:
        for (rep, m, sc), o, n, who in sorted(flips):
            print(f"  rep{rep} {m[:26]}/{sc[:18]}: off={o} on={n}  <- {who}")
    else:
        print("  없음")
    print()

    # --- pairwise metric comparison ----------------------------------------
    pairs = [(k, v) for k, v in by_key.items()
             if "off" in v and "on" in v
             and v["off"]["outcome"] in USABLE and v["on"]["outcome"] in USABLE]
    print(f"=== 양팔 모두 계획 성공한 쌍: {len(pairs)} ===")
    combos = sorted({(m, sc) for (_, m, sc), _ in pairs})
    print(f"조합 {len(combos)}개, 조합당 반복 "
          f"{collections.Counter(len([1 for (r_, m_, s_), _ in pairs if (m_, s_) == c]) for c in combos)}")
    print()

    verdict_bad = list(flips)
    for metric in SAFETY_HIGHER_BETTER + SAFETY_LOWER_BETTER + TIME:
        higher_better = metric in SAFETY_HIGHER_BETTER
        is_safety = metric in SAFETY_HIGHER_BETTER + SAFETY_LOWER_BETTER
        per_combo = collections.defaultdict(list)
        for (rep, m, sc), v in pairs:
            o, n = num(v["off"].get(metric)), num(v["on"].get(metric))
            if o is None or n is None:
                continue
            per_combo[(m, sc)].append((o, n))
        if not per_combo:
            continue
        print(f"--- {metric} ({'높을수록 좋음' if higher_better else '낮을수록 좋음'})"
              f"{' [안전]' if is_safety else ' [시간]'}")
        worse = []
        for c, vals in sorted(per_combo.items()):
            offs = [x[0] for x in vals]
            ons = [x[1] for x in vals]
            mo, mn = pct(offs, 50), pct(ons, 50)
            po, pn = pct(offs, 95), pct(ons, 95)
            if mo in (None, 0) and mn in (None, 0):
                continue
            base = abs(mo) if mo else 1.0
            delta = (mn - mo) / base
            bad = (delta < -a.tol) if higher_better else (delta > a.tol)
            mark = "  <<< 악화" if (bad and is_safety) else ""
            print(f"    {c[0][:24]:24s}/{c[1][:16]:16s} n={len(vals)} "
                  f"median {mo:8.3f} -> {mn:8.3f} ({delta:+6.1%})  p95 {po:8.3f} -> {pn:8.3f}{mark}")
            if bad and is_safety:
                worse.append((metric, c, mo, mn))
        verdict_bad.extend(worse)
        print()

    print("=" * 70)
    if verdict_bad:
        print("판정: chain/difficulty_balance = false 유지")
        print(f"이유: 안전 지표 후퇴 {len(verdict_bad)}건")
        for item in verdict_bad[:12]:
            print("   -", item)
    else:
        print("판정: 안전 지표 후퇴 없음 — true 채택을 검토할 수 있음")
        print("     (시간 이득은 위 [시간] 항목 참조)")
    print("=" * 70)


if __name__ == "__main__":
    main()
