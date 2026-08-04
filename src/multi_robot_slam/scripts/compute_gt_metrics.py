#!/usr/bin/env python3
"""
compute_gt_metrics.py

Computes wall-detection AND free-space-detection accuracy metrics
(Precision / Recall / F1 / IoU) for each map-fusion method against the
Gazebo-SDF-derived ground truth grid.

WHY THIS EXISTS
----------------
The saved global maps (global_map_<method>.npy) and the ground truth grid
(ground_truth.npy) have DIFFERENT shapes and DIFFERENT origins:

    predicted (icp_gnn): shape (461, 464), origin (-11.503, -11.594)
    ground truth:        shape (444, 448), origin (-11.2,   -11.1)

At 0.05 m resolution that's an offset of ~6.06 cells in x and ~9.87 cells
in y -- NOT a whole number of cells. This script aligns the two grids on
WORLD COORDINATES, rounds to the nearest cell (reporting the resulting
sub-cell error so it isn't silently hidden), crops to the overlapping
region, and only then builds the confusion matrix.

If you don't do this -- e.g. compare the two arrays index-by-index despite
the shape mismatch -- every wall in the comparison is misaligned by several
cells and the resulting precision/recall/F1/IoU numbers are close to
meaningless. This script exists specifically to avoid that failure mode
and to make the alignment quality visible/auditable.

FREE-SPACE DEFINITION (per supervisor's navigation-oriented framing)
----------------------------------------------------------------------
  predicted "free"     = cell value == 0
  predicted "not free" = cell value == 100 (occupied)  OR  -1 (unknown)

Unknown cells are folded into the NEGATIVE class for this metric, because
from a navigation standpoint unmapped space cannot be treated as safely
traversable regardless of what it turns out to contain. This is a
deliberate, different scoring convention from the existing wall-detection
metric, which only evaluates the occupied/wall class and does not treat
unknown specially.

CAVEAT THIS SCRIPT DOES NOT FIX
----------------------------------
Each global_map_<method>.npy currently represents exactly ONE run (the
save script overwrites the same filename every time it's called with no
run index), not an average over 3 runs like the operational metrics in
Table 2. The numbers this script produces are therefore single-run point
estimates. If you want a proper mean +/- std to match Table 2's
methodology, you need to re-run save_final_map.py with a run-indexed
filename (e.g. global_map_<method>_run{N}.npy) across 3 fresh runs per
method, then average this script's output across those 3 files.

Usage
-----
  python3 compute_gt_metrics.py \
      --gt /ros2_ws/results/gnn/ground_truth.npy \
      --gt-meta /ros2_ws/results/gnn/ground_truth_meta.json \
      --maps-dir /ros2_ws/results2/maps \
      --out /ros2_ws/results2/gt_accuracy_report.json
"""
import argparse
import glob
import json
import os

import numpy as np


def load_grid(npy_path, meta_path):
    arr = np.load(npy_path)
    with open(meta_path) as f:
        meta = json.load(f)
    return arr, meta


def world_to_gt_offset(pred_meta, gt_meta):
    """
    Float cell offset (row, col) needed to shift the predicted grid so its
    coordinate frame lines up with the ground truth grid's frame.
    Positive offset means the predicted grid's origin is to the
    right/above (in cell terms) of the GT grid's origin.
    """
    res = gt_meta["resolution"]
    dx = pred_meta["origin_x"] - gt_meta["world_min_x"]
    dy = pred_meta["origin_y"] - gt_meta["world_min_y"]
    col_off = dx / res
    row_off = dy / res
    return row_off, col_off


def align_and_crop(pred, pred_meta, gt, gt_meta):
    """
    Returns (aligned_pred, subcell_error_m, overlap_coverage_pct).

    aligned_pred has the SAME shape as gt. Cells with no corresponding
    predicted data (outside the predicted grid's footprint) are filled
    with -1 (unknown), which is the conservative choice for both the
    wall metric (counts as not-predicted-wall) and the free-space metric
    (counts as not-predicted-free, consistent with the "unknown is not
    traversable" framing).
    """
    row_off_f, col_off_f = world_to_gt_offset(pred_meta, gt_meta)
    row_off = int(round(row_off_f))
    col_off = int(round(col_off_f))
    subcell_err_m = (
        abs(row_off_f - row_off) + abs(col_off_f - col_off)
    ) * gt_meta["resolution"]

    gh, gw = gt.shape
    ph, pw = pred.shape

    aligned_pred = np.full((gh, gw), -1, dtype=np.int8)

    src_row_start = max(0, -row_off)
    src_col_start = max(0, -col_off)
    dst_row_start = max(0, row_off)
    dst_col_start = max(0, col_off)

    rows = min(ph - src_row_start, gh - dst_row_start)
    cols = min(pw - src_col_start, gw - dst_col_start)

    if rows <= 0 or cols <= 0:
        raise ValueError(
            "No overlap between predicted grid and ground truth grid "
            "-- check origins/resolution in the meta files."
        )

    aligned_pred[
        dst_row_start : dst_row_start + rows, dst_col_start : dst_col_start + cols
    ] = pred[
        src_row_start : src_row_start + rows, src_col_start : src_col_start + cols
    ]

    coverage_pct = 100.0 * rows * cols / (gh * gw)
    return aligned_pred, subcell_err_m, coverage_pct


def confusion(pred_positive, gt_positive):
    tp = int(np.sum(pred_positive & gt_positive))
    fp = int(np.sum(pred_positive & ~gt_positive))
    fn = int(np.sum(~pred_positive & gt_positive))
    precision = tp / (tp + fp) if (tp + fp) > 0 else float("nan")
    recall = tp / (tp + fn) if (tp + fn) > 0 else float("nan")
    f1 = (
        2 * precision * recall / (precision + recall)
        if (precision + recall) > 0
        else float("nan")
    )
    iou = tp / (tp + fp + fn) if (tp + fp + fn) > 0 else float("nan")
    return {
        "tp": tp,
        "fp": fp,
        "fn": fn,
        "precision": round(float(precision), 4),
        "recall": round(float(recall), 4),
        "f1": round(float(f1), 4),
        "iou": round(float(iou), 4),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gt", required=True)
    ap.add_argument("--gt-meta", required=True)
    ap.add_argument("--maps-dir", required=True)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    gt, gt_meta = load_grid(args.gt, args.gt_meta)
    gt_wall = gt == 100
    gt_free = gt == 0

    results = {}
    npy_files = sorted(glob.glob(os.path.join(args.maps_dir, "global_map_*.npy")))
    for npy_path in npy_files:
        base = os.path.basename(npy_path)
        if not base.startswith("global_map_") or not base.endswith(".npy"):
            continue
        method = base[len("global_map_") : -len(".npy")]
        meta_path = os.path.join(args.maps_dir, f"global_map_{method}_meta.json")
        if not os.path.exists(meta_path):
            print(f"[skip] no meta file for {method}")
            continue

        pred, pred_meta = load_grid(npy_path, meta_path)
        aligned, subcell_err_m, coverage_pct = align_and_crop(
            pred, pred_meta, gt, gt_meta
        )

        pred_wall = aligned == 100
        pred_free = aligned == 0  # unknown (-1) is deliberately NOT free

        wall_metrics = confusion(pred_wall, gt_wall)
        free_metrics = confusion(pred_free, gt_free)

        results[method] = {
            "alignment": {
                "subcell_error_m": round(subcell_err_m, 4),
                "overlap_coverage_pct": round(coverage_pct, 2),
            },
            "wall_detection": wall_metrics,
            "free_space_detection": free_metrics,
        }

        print(f"\n=== {method} ===")
        print(
            f"  alignment: subcell_err={subcell_err_m*100:.1f}cm  "
            f"overlap={coverage_pct:.1f}% of GT grid"
        )
        print(
            f"  wall:  P={wall_metrics['precision']:.3f} "
            f"R={wall_metrics['recall']:.3f} "
            f"F1={wall_metrics['f1']:.3f} IoU={wall_metrics['iou']:.3f}"
        )
        print(
            f"  free:  P={free_metrics['precision']:.3f} "
            f"R={free_metrics['recall']:.3f} "
            f"F1={free_metrics['f1']:.3f} IoU={free_metrics['iou']:.3f}"
        )

    if args.out:
        with open(args.out, "w") as f:
            json.dump(results, f, indent=2)
        print(f"\nSaved -> {args.out}")


if __name__ == "__main__":
    main()
