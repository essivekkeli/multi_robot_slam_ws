#!/usr/bin/env python3
"""
validate_results.py  —  Validate fusion experiment results
===========================================================
Usage:
  # validate all methods, all runs
  python3 validate_results.py

  # validate specific methods
  python3 validate_results.py --methods baseline_tf icp

  # validate a specific run directory
  python3 validate_results.py --run_dir /tmp/fusion_metrics/icp_20260601_120000

Changes from v1:
  FIX1: wall_sharpness check was inverted (lower = sharper, not higher).
        Renamed MIN_SHARPNESS -> MAX_SHARPNESS, check is now sharpness <= MAX.
  FIX2: MIN_DURATION_S aligned with MIN_SNAPSHOTS (both imply >= 300s / 10 runs).
        Changed to 300s to match 5-minute runs. If you run 10 min, set to 600.
  FIX3: load_metrics now loads ALL run_* directories, not just the last one.
        validate_method() reports mean +/- std across runs for thesis table.
  FIX4: spiral/drift collapse check — removed incorrect 'or peak_idx > 0.7'
        condition. A late peak that collapses is still a collapse.
"""

import json
import os
import argparse
import glob
import numpy as np

RESULTS_DIR = "/ros2_ws/results2"
METRICS_DIR = "/tmp/fusion_metrics"

# -- Thresholds ---------------------------------------------------------------
MIN_SNAPSHOTS         = 10      # fewer = run too short (10 x 30s = 300s)
MIN_DURATION_S        = 300     # FIX2: 5-minute runs = 300s (was 600)
CONVERGENCE_WINDOW    = 5       # last N snapshots checked for stability
MAX_CONVERGENCE_DRIFT = 1.0     # unknown% allowed to vary in last N snapshots
MIN_OCCUPIED          = 10.0    # % — too low = robots didn't explore
MAX_UNKNOWN           = 80.0    # % — too high = map mostly unexplored
MAX_SHARPNESS         = 6.0     # FIX1: UPPER bound — higher = blurrier walls
                                 # (metric is mean run-length; lower = sharper)
MAX_POINT_DROP_PCT    = 50.0    # robot point count drop > this = suspect

PASS = "✓ PASS"
WARN = "⚠ WARN"
FAIL = "✗ FAIL"


def load_metrics_single(path):
    """Load a single metrics.jsonl file. Returns list of records."""
    records = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    records.append(json.loads(line))
                except json.JSONDecodeError:
                    pass
    return records


def load_metrics(method):
    """
    FIX3: Load metrics.jsonl for ALL run_* directories under a method,
    not just the last one. Returns (list_of_run_records, list_of_sources).
    Falls back to /tmp/fusion_metrics if /ros2_ws/results2 has nothing.
    """
    all_runs   = []
    all_sources = []

    # Primary: results2/method/run_*/metrics.jsonl
    run_dirs = sorted(glob.glob(f"{RESULTS_DIR}/{method}/run_*"))
    for rd in run_dirs:
        candidate = os.path.join(rd, "metrics.jsonl")
        if os.path.exists(candidate):
            records = load_metrics_single(candidate)
            if records:
                all_runs.append(records)
                all_sources.append(candidate)

    # Fallback: /tmp/fusion_metrics/method_*/metrics.jsonl
    if not all_runs:
        tmp_dirs = sorted(glob.glob(f"{METRICS_DIR}/{method}_*"))
        for td in tmp_dirs:
            candidate = os.path.join(td, "metrics.jsonl")
            if os.path.exists(candidate):
                records = load_metrics_single(candidate)
                if records:
                    all_runs.append(records)
                    all_sources.append(candidate)

    return all_runs, all_sources


def check(label, condition, message, level=FAIL):
    """Print a check result. Returns 1 if passed, 0 if not."""
    status = PASS if condition else level
    print(f"  {status}  {label}: {message}")
    return 1 if condition else 0


def _snapshot_at(records, target_sec):
    """
    Return the snapshot whose elapsed_sec is closest to target_sec.
    If target_sec is None, returns the last snapshot (default behaviour).
    Also returns the actual elapsed time of the chosen snapshot.
    """
    if target_sec is None:
        return records[-1], records[-1].get("elapsed_sec", 0)
    best = min(records, key=lambda r: abs(r.get("elapsed_sec", 0) - target_sec))
    return best, best.get("elapsed_sec", 0)


def validate_run(records, source, method, run_idx, n_runs, compare_at_sec=None):
    """
    Validate a single run's metrics. Returns (passes, total, final_record).
    compare_at_sec: if set, metrics are taken from the snapshot closest to
                    this elapsed time instead of the last snapshot.
                    Enables fair comparison across runs with different durations.
    """
    label = f"Run {run_idx+1}/{n_runs}"
    print(f"\n  --- {label}  ({source}) ---")
    print(f"  Snapshots: {len(records)}")
    if compare_at_sec is not None:
        print(f"  Comparing at: t={compare_at_sec}s")

    passes = 0
    total  = 0

    # Select the snapshot to use for metric comparison
    final, actual_t = _snapshot_at(records, compare_at_sec)
    if compare_at_sec is not None:
        print(f"  Using snapshot at t={actual_t:.0f}s "
              f"(closest to {compare_at_sec}s)")

    # 1. Enough snapshots
    total += 1
    ok = len(records) >= MIN_SNAPSHOTS
    passes += check("Snapshot count", ok,
                    f"{len(records)} ({'OK' if ok else f'need >= {MIN_SNAPSHOTS}'})",
                    WARN)

    # 2. Run duration — always check full run, not comparison snapshot
    total += 1
    duration = records[-1].get("elapsed_sec", 0)
    ok = duration >= MIN_DURATION_S
    passes += check("Run duration", ok,
                    f"{duration:.0f}s ({'OK' if ok else f'need >= {MIN_DURATION_S}s'})",
                    WARN)

    # 3. Map convergence — always use last N snapshots of full run
    total += 1
    last_n   = records[-CONVERGENCE_WINDOW:]
    unknowns = [r.get("unknown_pct", 100) for r in last_n]
    drift    = max(unknowns) - min(unknowns)
    ok = drift <= MAX_CONVERGENCE_DRIFT
    passes += check("Convergence (last 5)", ok,
                    f"unknown% drift = {drift:.2f}pp "
                    f"({'stable' if ok else 'still changing — run longer'})",
                    WARN)

    # 4. FIX4: Spiral/drift collapse — drop from peak, no peak_idx escape hatch
    total += 1
    occ_series = [r.get("occupied_pct", 0) for r in records]
    if len(occ_series) >= 6:
        peak_val  = max(occ_series)
        final_val = occ_series[-1]
        drop      = peak_val - final_val
        ok        = drop < 5.0   # FIX4: removed incorrect 'or peak_idx > 0.7'
        passes += check("No drift collapse", ok,
                        f"peak={peak_val:.1f}% final={final_val:.1f}% "
                        f"drop={drop:.1f}pp "
                        f"({'OK' if ok else 'dropped > 5pp — possible GLIM spiral'})",
                        FAIL)
    else:
        total -= 1  # not enough data — don't penalise

    # 5. Final map coverage
    total += 1
    occ   = final.get("occupied_pct", 0)
    unk   = final.get("unknown_pct", 100)
    ok    = occ >= MIN_OCCUPIED and unk <= MAX_UNKNOWN
    passes += check("Final map coverage", ok,
                    f"occupied={occ:.1f}%  unknown={unk:.1f}% "
                    f"({'OK' if ok else 'poor exploration'})",
                    FAIL)

    # 6. FIX1: Wall sharpness — lower is BETTER (thinner walls)
    total += 1
    sharpness = final.get("wall_sharpness_mean_run", 0)
    ok        = sharpness <= MAX_SHARPNESS   # FIX1: was >= MIN_SHARPNESS
    passes += check("Wall sharpness", ok,
                    f"{sharpness:.3f} (lower=sharper) "
                    f"({'OK' if ok else f'> {MAX_SHARPNESS} — walls very blurry'})",
                    WARN)

    # 7. All robots contributing
    total += 1
    ppr = final.get("points_per_robot", {})
    all_contributing = all(v > 1000 for v in ppr.values()) if ppr else False
    passes += check("All robots contributing",
                    all_contributing,
                    f"{ppr}" if all_contributing else
                    f"{ppr}  <- some robots have < 1000 points",
                    FAIL)

    # 8. Robot point stability
    total += 1
    if len(records) >= 3 and ppr:
        drops = []
        for robot in ppr.keys():
            pts      = [r.get("points_per_robot", {}).get(robot, 0) for r in records]
            pts      = [p for p in pts if p > 0]
            if len(pts) >= 2:
                peak      = max(pts)
                final_pts = pts[-1]
                drop_pct  = (peak - final_pts) / peak * 100 if peak > 0 else 0
                if drop_pct > MAX_POINT_DROP_PCT:
                    drops.append(f"{robot}: dropped {drop_pct:.0f}% from peak")
        ok = len(drops) == 0
        passes += check("Robot point stability", ok,
                        "OK" if ok else " | ".join(drops) + " (possible crash)",
                        WARN)
    else:
        # not enough data — count as pass silently
        passes += 1

    # 9. ICP acceptance rate (ICP methods only)
    if "icp" in method:
        total += 1
        attempts = final.get("icp_attempts", 0)
        accepted = final.get("icp_accepted", 0)
        rate     = accepted / attempts * 100 if attempts > 0 else 0
        ok       = attempts > 0 and rate >= 10.0
        passes += check("ICP acceptance rate", ok,
                        f"{accepted}/{attempts} ({rate:.0f}%) "
                        f"{'OK' if ok else '— ICP never fired or always rejected'}",
                        WARN)

    # 10. GNN updates (GNN methods only)
    if "gnn" in method:
        total += 1
        gnn_updates = final.get("gnn_updates", 0)
        ok          = gnn_updates > 0
        passes += check("GNN weight updates", ok,
                        f"{gnn_updates} online updates "
                        f"{'OK' if ok else '— GNN never updated (check logs)'}",
                        WARN)

    return passes, total, final


def validate_method(method, compare_at_sec=None):
    print(f"\n{'='*60}")
    print(f"  METHOD: {method}")
    if compare_at_sec is not None:
        print(f"  Normalised to t={compare_at_sec}s per run")
    print(f"{'='*60}")

    all_runs, all_sources = load_metrics(method)

    if not all_runs:
        print(f"  {FAIL}  No metrics found for '{method}'")
        return None

    print(f"  Found {len(all_runs)} run(s)")

    run_results = []
    for i, (records, source) in enumerate(zip(all_runs, all_sources)):
        p, t, final = validate_run(records, source, method, i, len(all_runs),
                                   compare_at_sec=compare_at_sec)
        run_results.append((p, t, final))

    # -- Cross-run summary (thesis table values) ------------------------------
    print(f"\n  --- Cross-run summary ({len(all_runs)} runs) ---")

    def stat(key):
        vals = [r[2].get(key, 0) for r in run_results]
        return np.mean(vals), np.std(vals), vals

    occ_m,   occ_s,   occ_v   = stat("occupied_pct")
    unk_m,   unk_s,   unk_v   = stat("unknown_pct")
    sharp_m, sharp_s, sharp_v = stat("wall_sharpness_mean_run")
    dur_m,   dur_s,   dur_v   = stat("elapsed_sec")

    print(f"  occupied%   : {occ_m:.2f} ± {occ_s:.2f}   {[f'{v:.1f}' for v in occ_v]}")
    print(f"  unknown%    : {unk_m:.2f} ± {unk_s:.2f}   {[f'{v:.1f}' for v in unk_v]}")
    print(f"  sharpness   : {sharp_m:.3f} ± {sharp_s:.3f}  {[f'{v:.2f}' for v in sharp_v]}")
    print(f"  duration(s) : {dur_m:.0f} ± {dur_s:.0f}")

    if "icp" in method:
        attempts_v = [r[2].get("icp_attempts", 0) for r in run_results]
        accepted_v = [r[2].get("icp_accepted", 0) for r in run_results]
        rates      = [a/t*100 if t > 0 else 0 for a, t in zip(accepted_v, attempts_v)]
        print(f"  ICP accept% : {np.mean(rates):.1f} ± {np.std(rates):.1f}   "
              f"{[f'{r:.0f}%' for r in rates]}")

    # -- Per-run check summary ------------------------------------------------
    total_p = sum(r[0] for r in run_results)
    total_t = sum(r[1] for r in run_results)
    pct     = total_p / total_t * 100 if total_t > 0 else 0
    verdict = "VALID ✓" if pct == 100 else \
              "ACCEPTABLE" if pct >= 70 else \
              "INVALID ✗ — do not use for comparison"

    print(f"\n  Result: {total_p}/{total_t} checks passed across all runs  <- {verdict}")

    return method, occ_m, occ_s, unk_m, unk_s, sharp_m, sharp_s, total_p, total_t


def main():
    parser = argparse.ArgumentParser(
        description="Validate multi-robot fusion experiment results")
    parser.add_argument("--methods", nargs="+",
                        default=["baseline_tf", "probabilistic", "icp",
                                 "icp_probabilistic", "gnn", "icp_gnn"])
    parser.add_argument("--run_dir", default=None,
                        help="Validate a single run directory directly")
    parser.add_argument("--compare_at_sec", type=int, default=None,
                        help="Compare all runs at this elapsed sim-time (seconds) "
                             "instead of run end. Enables fair cross-run comparison "
                             "when run durations vary. Example: --compare_at_sec 300")
    args = parser.parse_args()

    print("\nFUSION EXPERIMENT RESULT VALIDATOR")
    print("====================================")
    print(f"Results dir : {RESULTS_DIR}")
    print(f"Fallback dir: {METRICS_DIR}")
    print(f"Sharpness   : lower is better (mean occupied run-length in cells)")
    if args.compare_at_sec:
        print(f"Normalised  : metrics taken at t={args.compare_at_sec}s per run")

    if args.run_dir:
        # Single run mode
        candidate = os.path.join(args.run_dir, "metrics.jsonl")
        if not os.path.exists(candidate):
            print(f"No metrics.jsonl in {args.run_dir}")
            return
        records = load_metrics_single(candidate)
        method  = os.path.basename(args.run_dir).split("_")[0]
        validate_run(records, candidate, method, 0, 1,
                     compare_at_sec=args.compare_at_sec)
        return

    summary = []
    for method in args.methods:
        result = validate_method(method, compare_at_sec=args.compare_at_sec)
        if result is not None:
            summary.append(result)

    if not summary:
        print("\nNo results found for any method.")
        return

    # -- Final thesis table ---------------------------------------------------
    print(f"\n{'='*60}")
    print("THESIS COMPARISON TABLE")
    print(f"{'='*60}")
    print(f"  {'Method':<20} {'occ% (mean±std)':>18} {'unk% (mean±std)':>18} "
          f"{'sharp (mean±std)':>18}  {'Checks'}")
    print(f"  {'-'*90}")
    for method, om, os_, um, us, sm, ss, p, t in summary:
        status = "VALID" if p == t else "ACCEPTABLE" if p/t >= 0.7 else "INVALID"
        print(f"  {method:<20} {om:>6.2f} ± {os_:<6.2f}  "
              f"{um:>6.2f} ± {us:<6.2f}  "
              f"{sm:>6.3f} ± {ss:<6.3f}  "
              f"{p}/{t}  {status}")
    print()


if __name__ == "__main__":
    main()