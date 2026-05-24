#!/usr/bin/env python3
import argparse
import json
import os
import shutil
import glob
from datetime import datetime


def find_metrics_dir(method):
    import os
    pattern = f"/tmp/fusion_metrics/{method}_*"
    matches = glob.glob(pattern)
    # Filter exact prefix matches only (e.g. 'icp_' should not match 'icp_probabilistic_')
    matches = [m for m in matches
               if os.path.basename(m).startswith(method + '_')
               and not any(os.path.basename(m).startswith(method + '_' + suffix)
                           for suffix in ['probabilistic', 'gnn'])]
    matches = sorted(matches)
    if not matches:
        raise FileNotFoundError(f"No metrics found for method '{method}'")
    return matches[-1]


def load_jsonl(path):
    records = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                records.append(json.loads(line))
    return records


def export_csv(records, path):
    if not records:
        return
    keys = ["timestamp", "elapsed_sec", "method",
            "unknown_pct", "occupied_pct", "free_pct",
            "wall_sharpness_mean_run", "map_width", "map_height", "total_cells"]
    robots = list(records[0].get("points_per_robot", {}).keys())
    with open(path, "w") as f:
        header = keys + [f"pts_{r}" for r in robots]
        f.write(",".join(header) + "\n")
        for r in records:
            pts = r.get("points_per_robot", {})
            row = [str(r.get(k, "")) for k in keys]
            row += [str(pts.get(rb, 0)) for rb in robots]
            f.write(",".join(row) + "\n")


def print_table(records, path):
    if not records:
        return
    robots = sorted(records[0].get("points_per_robot", {}).keys())
    hdr = (f"{'t(s)':<8} {'unknown%':<10} {'occupied%':<11} "
           f"{'free%':<8} {'sharpness':<11} " +
           " ".join(f"{r:<10}" for r in robots))
    sep = "-" * len(hdr)
    lines = [hdr, sep]
    for r in records:
        pts = r.get("points_per_robot", {})
        line = (f"{r['elapsed_sec']:<8} "
                f"{r['unknown_pct']:<10} "
                f"{r['occupied_pct']:<11} "
                f"{r['free_pct']:<8} "
                f"{r['wall_sharpness_mean_run']:<11} " +
                " ".join(f"{pts.get(rb,0):<10}" for rb in robots))
        lines.append(line)
    lines.append(sep)
    if len(records) >= 5:
        last5 = records[-5:]
        avg_unk  = sum(r['unknown_pct']  for r in last5) / 5
        avg_occ  = sum(r['occupied_pct'] for r in last5) / 5
        avg_free = sum(r['free_pct']     for r in last5) / 5
        avg_shp  = sum(r['wall_sharpness_mean_run'] for r in last5) / 5
        lines.append(f"FINAL AVG (last 5): unknown={avg_unk:.2f}% "
                     f"occupied={avg_occ:.2f}% free={avg_free:.2f}% "
                     f"sharpness={avg_shp:.3f}")
    table = "\n".join(lines)
    with open(path, "w") as f:
        f.write(table + "\n")
    print(table)


def write_notes(path, method, run, notes, records, src_dir):
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    duration = f"{records[-1]['elapsed_sec']:.0f}s" if records else "unknown"
    last = records[-1] if records else {}
    pts = last.get("points_per_robot", {})
    active_robots = [r for r, p in pts.items() if p > 0]
    content = f"""Run Notes
=========
Method:          {method}
Run:             {run}
Date:            {now}
Duration:        {duration}
Active robots:   {', '.join(active_robots) if active_robots else 'unknown'}
Source dir:      {src_dir}

Notes:
{notes}

Final metrics:
  unknown%:   {last.get('unknown_pct', 'N/A')}
  occupied%:  {last.get('occupied_pct', 'N/A')}
  free%:      {last.get('free_pct', 'N/A')}
  sharpness:  {last.get('wall_sharpness_mean_run', 'N/A')}

TODO (do manually):
  [ ] Save RViz screenshot (full map)     -> global_map_full.png
  [ ] Save RViz screenshot (junction)     -> global_map_junction.png
  [ ] Note any GLIM drift observed
"""
    with open(path, "w") as f:
        f.write(content)
    print(content)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--method", required=True,
                        choices=["baseline_tf", "icp", "probabilistic", "gnn", "icp_probabilistic", "gnn_probabilistic", "icp_gnn"])
    parser.add_argument("--run", default="001")
    parser.add_argument("--notes", default="")
    args = parser.parse_args()

    src_dir = find_metrics_dir(args.method)
    print(f"Found metrics: {src_dir}")

    out_dir = f"/ros2_ws/results/{args.method}/run_{args.run}"
    os.makedirs(out_dir, exist_ok=True)
    print(f"Saving to: {out_dir}")

    jsonl_src = os.path.join(src_dir, "metrics.jsonl")
    if not os.path.exists(jsonl_src):
        print("ERROR: metrics.jsonl not found")
        return

    records = load_jsonl(jsonl_src)
    print(f"Loaded {len(records)} snapshots")

    shutil.copy(jsonl_src, os.path.join(out_dir, "metrics.jsonl"))

    info_src = os.path.join(src_dir, "run_info.json")
    if os.path.exists(info_src):
        shutil.copy(info_src, os.path.join(out_dir, "run_info.json"))

    export_csv(records, os.path.join(out_dir, "metrics.csv"))
    print_table(records, os.path.join(out_dir, "results_table.txt"))
    write_notes(os.path.join(out_dir, "notes.txt"),
                args.method, args.run, args.notes, records, src_dir)

    if records:
        last5 = records[-5:] if len(records) >= 5 else records
        avg_unk = sum(r['unknown_pct']  for r in last5) / len(last5)
        avg_occ = sum(r['occupied_pct'] for r in last5) / len(last5)
        avg_shp = sum(r['wall_sharpness_mean_run'] for r in last5) / len(last5)
        print(f"\nTHESIS TABLE ROW ({args.method}):")
        print(f"  unknown%={avg_unk:.2f}  occupied%={avg_occ:.2f}  sharpness={avg_shp:.3f}")

    print(f"\nSaved to: {out_dir}")
    print("Next: take RViz screenshots and copy to that folder")


if __name__ == "__main__":
    main()
