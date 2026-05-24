#!/usr/bin/env python3
"""
Validate fusion experiment results.
Usage: python3 validate_results.py [--methods baseline_tf icp probabilistic gnn icp_probabilistic icp_gnn]
"""

import json
import os
import argparse
import glob

RESULTS_DIR = "/ros2_ws/results2"
METRICS_DIR = "/tmp/fusion_metrics"

# ── thresholds ────────────────────────────────────────────────────────────────
MIN_SNAPSHOTS         = 18      # fewer = run too short
MIN_DURATION_S        = 600     # less = likely didn't converge
CONVERGENCE_WINDOW    = 5       # last N snapshots checked for stability
MAX_CONVERGENCE_DRIFT = 1.0     # unknown% allowed to vary in last N snapshots
MIN_OCCUPIED          = 10.0    # % — too low = robots didn't explore
MAX_UNKNOWN           = 80.0    # % — too high = map mostly unexplored
MIN_SHARPNESS         = 3.0     # too low = very blurry map
MAX_POINT_DROP_PCT    = 50.0    # robot point count drop > this = suspect

PASS = "✓ PASS"
WARN = "⚠ WARN"
FAIL = "✗ FAIL"


def load_metrics(method):
    """Load metrics.jsonl for a method — prefer /ros2_ws/results2, fallback /tmp."""
    run_dirs = sorted(glob.glob(f"{RESULTS_DIR}/{method}/run_*"))
    source = None

    if run_dirs:
        candidate = os.path.join(run_dirs[-1], "metrics.jsonl")
        if os.path.exists(candidate):
            source = candidate

    if source is None:
        tmp_dirs = sorted(glob.glob(f"{METRICS_DIR}/{method}_*"))
        if tmp_dirs:
            candidate = os.path.join(tmp_dirs[-1], "metrics.jsonl")
            if os.path.exists(candidate):
                source = candidate

    if source is None:
        return None, None

    records = []
    with open(source) as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    records.append(json.loads(line))
                except json.JSONDecodeError:
                    pass
    return records, source


def check(label, condition, message, level=FAIL):
    """Print a check result. Returns True if passed."""
    status = PASS if condition else level
    print(f"  {status}  {label}: {message}")
    return condition


def validate_method(method):
    print(f"\n{'='*60}")
    print(f"  METHOD: {method}")
    print(f"{'='*60}")

    records, source = load_metrics(method)

    if records is None:
        print(f"  {FAIL}  No metrics found for '{method}'")
        return False

    print(f"  Source: {source}")
    print(f"  Snapshots loaded: {len(records)}")

    passes = 0
    total  = 0

    # ── 1. Enough snapshots ───────────────────────────────────────────────────
    total += 1
    ok = len(records) >= MIN_SNAPSHOTS
    passes += check("Snapshot count",
                    ok,
                    f"{len(records)} snapshots "
                    f"({'OK' if ok else f'need >= {MIN_SNAPSHOTS}'})",
                    WARN)

    # ── 2. Run duration ───────────────────────────────────────────────────────
    total += 1
    duration = records[-1].get("elapsed_sec", 0)
    ok = duration >= MIN_DURATION_S
    passes += check("Run duration",
                    ok,
                    f"{duration:.0f}s "
                    f"({'OK' if ok else f'need >= {MIN_DURATION_S}s — map may not have converged'})",
                    WARN)

    # ── 3. Map convergence ────────────────────────────────────────────────────
    total += 1
    last_n   = records[-CONVERGENCE_WINDOW:]
    unknowns = [r.get("unknown_pct", 100) for r in last_n]
    drift    = max(unknowns) - min(unknowns)
    ok = drift <= MAX_CONVERGENCE_DRIFT
    passes += check("Convergence (last 5)",
                    ok,
                    f"unknown% drift = {drift:.2f}pp "
                    f"({'stable' if ok else 'still changing — run longer'})",
                    WARN)

    # ── 4. Spiral / drift collapse detection ─────────────────────────────────
    total += 1
    occ_series = [r.get("occupied_pct", 0) for r in records]
    if len(occ_series) >= 6:
        peak_idx = occ_series.index(max(occ_series))
        peak_val = occ_series[peak_idx]
        final_val = occ_series[-1]
        drop = peak_val - final_val
        ok = drop < 5.0 or peak_idx > len(occ_series) * 0.7
        passes += check("No spiral/drift collapse",
                        ok,
                        f"peak occ={peak_val:.1f}% at snapshot {peak_idx}, "
                        f"final={final_val:.1f}% "
                        f"({'OK' if ok else f'dropped {drop:.1f}pp — possible loop closure spiral'})",
                        FAIL)
    else:
        passes += check("No spiral/drift collapse", True,
                        "too few snapshots to check — skipped", WARN)

    # ── 5. Final map coverage ─────────────────────────────────────────────────
    total += 1
    final = records[-1]
    occ   = final.get("occupied_pct", 0)
    unk   = final.get("unknown_pct", 100)
    ok = occ >= MIN_OCCUPIED and unk <= MAX_UNKNOWN
    passes += check("Final map coverage",
                    ok,
                    f"occupied={occ:.1f}%  unknown={unk:.1f}% "
                    f"({'OK' if ok else 'poor exploration — robots may not have moved enough'})",
                    FAIL)

    # ── 6. Wall sharpness ─────────────────────────────────────────────────────
    total += 1
    sharpness = final.get("wall_sharpness_mean_run", 0)
    ok = sharpness >= MIN_SHARPNESS
    passes += check("Wall sharpness",
                    ok,
                    f"{sharpness:.3f} "
                    f"({'OK' if ok else f'< {MIN_SHARPNESS} — walls very blurry'})",
                    WARN)

    # ── 7. All robots contributing ────────────────────────────────────────────
    total += 1
    ppr = final.get("points_per_robot", {})
    all_contributing = all(v > 1000 for v in ppr.values()) if ppr else False
    passes += check("All robots contributing",
                    all_contributing,
                    f"{ppr}" if all_contributing else
                    f"{ppr}  ← some robots have < 1000 points",
                    FAIL)

    # ── 8. Robot point stability (no large drops) ─────────────────────────────
    total += 1
    if len(records) >= 3 and ppr:
        drops = []
        for robot in ppr.keys():
            pts = [r.get("points_per_robot", {}).get(robot, 0) for r in records]
            pts = [p for p in pts if p > 0]
            if len(pts) >= 2:
                peak      = max(pts)
                final_pts = pts[-1]
                drop_pct  = (peak - final_pts) / peak * 100 if peak > 0 else 0
                if drop_pct > MAX_POINT_DROP_PCT:
                    drops.append(f"{robot}: dropped {drop_pct:.0f}% from peak")
        ok = len(drops) == 0
        passes += check("Robot point stability",
                        ok,
                        "OK" if ok else " | ".join(drops) +
                        " (possible crash/restart)",
                        WARN)
    else:
        passes += 1  # skip if not enough data

    # ── 9. ICP acceptance rate (ICP methods only) ─────────────────────────────
    if "icp" in method:
        total += 1
        attempts = final.get("icp_attempts", 0)
        accepted = final.get("icp_accepted", 0)
        rate     = accepted / attempts * 100 if attempts > 0 else 0
        ok = attempts > 0 and rate >= 10.0
        passes += check("ICP acceptance rate",
                        ok,
                        f"{accepted}/{attempts} ({rate:.0f}%) "
                        f"{'OK' if ok else '— ICP never fired or always rejected'}",
                        WARN)

    # ── 10. GNN updates (GNN methods only) ───────────────────────────────────
    if "gnn" in method:
        total += 1
        gnn_updates = final.get("gnn_updates", 0)
        ok = gnn_updates > 0
        passes += check("GNN weight updates",
                        ok,
                        f"{gnn_updates} online updates "
                        f"{'OK' if ok else '— GNN model never updated (check logs)'}",
                        WARN)

    # ── Summary ───────────────────────────────────────────────────────────────
    pct = passes / total * 100 if total > 0 else 0
    verdict = "VALID ✓" if passes == total else \
              "ACCEPTABLE" if pct >= 70 else \
              "INVALID ✗ — do not use for comparison"

    print(f"\n  Result: {passes}/{total} checks passed  ← {verdict}")
    print(f"  Final metrics: occ={occ:.1f}%  unk={unk:.1f}%  "
          f"sharp={sharpness:.2f}  t={duration:.0f}s")

    return passes, total


def main():
    parser = argparse.ArgumentParser(
        description="Validate multi-robot fusion experiment results")
    parser.add_argument("--methods", nargs="+",
                        default=["baseline_tf", "probabilistic", "icp",
                                 "icp_probabilistic", "gnn", "icp_gnn"])
    args = parser.parse_args()

    print("\nFUSION EXPERIMENT RESULT VALIDATOR")
    print("===================================")
    print(f"Results dir : {RESULTS_DIR}")
    print(f"Fallback dir: {METRICS_DIR}")

    summary = {}
    for method in args.methods:
        result = validate_method(method)
        if isinstance(result, tuple):
            summary[method] = result

    if not summary:
        print("\nNo results found for any method.")
        return

    print(f"\n{'='*60}")
    print("SUMMARY")
    print(f"{'='*60}")
    print(f"  {'Method':<25} {'Checks':>8}  {'Status'}")
    print(f"  {'-'*55}")
    for method, (p, t) in summary.items():
        pct    = p / t * 100 if t > 0 else 0
        status = "VALID" if p == t else \
                 "ACCEPTABLE" if pct >= 70 else "INVALID"
        print(f"  {method:<25} {p}/{t:<4}      {status}")

    print()


if __name__ == "__main__":
    main()