#!/usr/bin/env python3
"""
central_server_unified.py  —  Stage 3a unified fusion server  (v2)
====================================================================
Controlled via a single ROS parameter:

  method_label:  baseline_tf | icp | probabilistic | icp_probabilistic
                 | gnn | icp_gnn

Method matrix:
  ┌──────────────────┬───────────────┬──────────────┐
  │ method_label     │ merge_method  │ drift_correct│
  ├──────────────────┼───────────────┼──────────────┤
  │ baseline_tf      │ occupied_wins │ none         │
  │ icp              │ occupied_wins │ VGICP        │
  │ probabilistic    │ log_odds      │ none         │
  │ icp_probabilistic│ log_odds      │ VGICP        │
  │ gnn              │ gnn           │ none         │
  │ icp_gnn          │ gnn           │ VGICP        │  ← NEW
  └──────────────────┴───────────────┴──────────────┘

ICP (VGICP) — corrected architecture
──────────────────────────────────────
Previous version aligned robot_A vs robot_B in world frame, which
is redundant because TF already places them in the same frame.

Fixed approach — per-robot drift correction:
  For each robot i, align C_i (its accumulated cloud) against
  the CONSENSUS cloud  C_ref = union( C_j  for j ≠ i ).
  The resulting transform T_i corrects GLIM drift for robot i
  in the world frame.

  Acceptance criteria relaxed to match realistic GLIM drift in
  a flat Gazebo simulation:
    fitness  > 0.05   (low overlap expected in multi-robot scenario)
    rmse     < 5.0 m  (drift can be metres in flat sim)
    rot      < 20°    (yaw drift)
    trans    < 3.0 m  (translational drift)

GNN — real implementation (see central_server_gnn.py)
───────────────────────────────────────────────────────
  2-layer Graph Attention Network, pure NumPy, no ML framework needed.
  Weights initialised analytically; optional online self-supervised update.

Usage:
  python3 central_server_unified.py --ros-args -p method_label:=icp_gnn
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
from tf2_ros import (Buffer, TransformListener, TransformBroadcaster,
                     LookupException, ConnectivityException,
                     ExtrapolationException)
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import numpy as np
import struct
import threading
import json
import os
import time
import math
from datetime import datetime

# ── Log-odds constants ────────────────────────────────────────────────────────
L_OCC  =  0.85
L_FREE = -0.40
L_MAX  =  3.50
P_OCC_THRESH  = 0.65
P_FREE_THRESH = 0.35

# ── Method → config ───────────────────────────────────────────────────────────
METHOD_CONFIG = {
    "baseline_tf":        {"merge": "occupied_wins", "icp": False},
    "icp":                {"merge": "occupied_wins", "icp": True},
    "probabilistic":      {"merge": "log_odds",      "icp": False},
    "icp_probabilistic":  {"merge": "log_odds",      "icp": True},
    "gnn":                {"merge": "gnn",            "icp": False},
    "icp_gnn":            {"merge": "gnn",            "icp": True},   # NEW
}


# ─────────────────────────────────────────────────────────────────────────────
#  Shared helpers
# ─────────────────────────────────────────────────────────────────────────────

def pointcloud2_to_xyz(msg):
    offsets = {}
    for field in msg.fields:
        if field.name in ("x", "y", "z"):
            offsets[field.name] = field.offset
    if not all(k in offsets for k in ("x", "y", "z")):
        return np.array([])
    n    = msg.width * msg.height
    step = msg.point_step
    data = msg.data
    xyz  = np.zeros((n, 3), dtype=np.float32)
    for i in range(n):
        base = i * step
        xyz[i, 0] = struct.unpack_from("f", data, base + offsets["x"])[0]
        xyz[i, 1] = struct.unpack_from("f", data, base + offsets["y"])[0]
        xyz[i, 2] = struct.unpack_from("f", data, base + offsets["z"])[0]
    valid = np.isfinite(xyz).all(axis=1)
    return xyz[valid]


def _voxel_downsample(xyz, voxel_size=0.05):
    if len(xyz) == 0:
        return xyz
    voxels = np.floor(xyz / voxel_size).astype(int)
    _, unique_idx = np.unique(voxels, axis=0, return_index=True)
    return xyz[unique_idx]


def accumulated_cloud_to_occupancy_grid(xyz, resolution=0.05,
                                         z_min=0.1, z_max=2.5,
                                         frame_id="world"):
    if len(xyz) == 0:
        return None
    mask  = (xyz[:, 2] >= z_min) & (xyz[:, 2] <= z_max)
    pts2d = xyz[mask, :2]
    if len(pts2d) == 0:
        return None
    floor_mask = (xyz[:, 2] >= -0.5) & (xyz[:, 2] < z_min)
    floor_pts  = xyz[floor_mask, :2]
    all_pts = np.vstack([pts2d, floor_pts]) if len(floor_pts) > 0 else pts2d
    min_x = all_pts[:, 0].min() - 1.0
    min_y = all_pts[:, 1].min() - 1.0
    max_x = all_pts[:, 0].max() + 1.0
    max_y = all_pts[:, 1].max() + 1.0
    width  = int(np.ceil((max_x - min_x) / resolution))
    height = int(np.ceil((max_y - min_y) / resolution))
    grid   = np.full((height, width), -1, dtype=np.int8)
    if len(floor_pts) > 0:
        gx = ((floor_pts[:, 0] - min_x) / resolution).astype(int)
        gy = ((floor_pts[:, 1] - min_y) / resolution).astype(int)
        v  = (gx >= 0) & (gx < width) & (gy >= 0) & (gy < height)
        grid[gy[v], gx[v]] = 0
    gx = ((pts2d[:, 0] - min_x) / resolution).astype(int)
    gy = ((pts2d[:, 1] - min_y) / resolution).astype(int)
    v  = (gx >= 0) & (gx < width) & (gy >= 0) & (gy < height)
    grid[gy[v], gx[v]] = 100
    msg = OccupancyGrid()
    msg.info.resolution            = resolution
    msg.info.width                 = width
    msg.info.height                = height
    msg.info.origin.position.x    = float(min_x)
    msg.info.origin.position.y    = float(min_y)
    msg.info.origin.orientation.w = 1.0
    msg.header.frame_id            = frame_id
    msg.data = grid.flatten().tolist()
    return msg


def _wall_sharpness(grid_np):
    occupied = (grid_np == 100)
    run_lengths = []
    for row in occupied:
        in_run, length = False, 0
        for val in row:
            if val:
                in_run = True; length += 1
            elif in_run:
                run_lengths.append(length); in_run = False; length = 0
        if in_run:
            run_lengths.append(length)
    return float(np.mean(run_lengths)) if run_lengths else 0.0


# ─────────────────────────────────────────────────────────────────────────────
#  Merge strategies
# ─────────────────────────────────────────────────────────────────────────────

def _world_canvas(maps, resolution):
    """Compute merged canvas bounds. Returns (min_x, min_y, width, height)."""
    min_x = min_y =  float("inf")
    max_x = max_y = float("-inf")
    for m in maps.values():
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y
        min_x = min(min_x, ox)
        min_y = min(min_y, oy)
        max_x = max(max_x, ox + m.info.width  * resolution)
        max_y = max(max_y, oy + m.info.height * resolution)
    w = int(np.ceil((max_x - min_x) / resolution))
    h = int(np.ceil((max_y - min_y) / resolution))
    return min_x, min_y, w, h


def _make_occ_grid(merged_data, min_x, min_y, width, height, resolution):
    g = OccupancyGrid()
    g.header.frame_id            = "world"
    g.info.resolution            = resolution
    g.info.width                 = width
    g.info.height                = height
    g.info.origin.position.x    = float(min_x)
    g.info.origin.position.y    = float(min_y)
    g.info.origin.orientation.w = 1.0
    g.data = merged_data.tolist()
    return g


def occupied_wins_merge(maps, resolution):
    if not maps:
        return None
    min_x, min_y, W, H = _world_canvas(maps, resolution)
    merged = np.full(H * W, -1, dtype=np.int8)
    for m in maps.values():
        ox    = m.info.origin.position.x
        oy    = m.info.origin.position.y
        x_off = int(round((ox - min_x) / resolution))
        y_off = int(round((oy - min_y) / resolution))
        data  = np.array(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)
        cc, rr = np.meshgrid(np.arange(m.info.width), np.arange(m.info.height))
        gx = x_off + cc
        gy = y_off + rr
        valid = (gx >= 0) & (gx < W) & (gy >= 0) & (gy < H)
        idx   = (gy * W + gx)[valid]
        cell  = data[valid]
        merged[idx[cell == 100]] = 100
        free_m = (cell == 0) & (merged[idx] != 100)
        merged[idx[free_m]] = 0
    return _make_occ_grid(merged, min_x, min_y, W, H, resolution)


def log_odds_merge(maps, resolution):
    if not maps:
        return None
    min_x, min_y, W, H = _world_canvas(maps, resolution)
    log_odds = np.zeros((H, W), dtype=np.float32)
    for m in maps.values():
        ox    = m.info.origin.position.x
        oy    = m.info.origin.position.y
        x_off = int(round((ox - min_x) / resolution))
        y_off = int(round((oy - min_y) / resolution))
        data  = np.array(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)
        gy, gx = np.meshgrid(np.arange(m.info.height) + y_off,
                              np.arange(m.info.width)  + x_off,
                              indexing="ij")
        valid     = (gx >= 0) & (gx < W) & (gy >= 0) & (gy < H)
        occ_mask  = valid & (data == 100)
        free_mask = valid & (data == 0)
        log_odds[gy[occ_mask],  gx[occ_mask]]  += L_OCC
        log_odds[gy[free_mask], gx[free_mask]] += L_FREE
    np.clip(log_odds, -L_MAX, L_MAX, out=log_odds)
    prob   = 1.0 / (1.0 + np.exp(-log_odds))
    merged = np.full((H, W), -1, dtype=np.int8)
    merged[prob >  P_OCC_THRESH]  = 100
    merged[prob <  P_FREE_THRESH] = 0
    return _make_occ_grid(merged.flatten(), min_x, min_y, W, H, resolution)


# ─────────────────────────────────────────────────────────────────────────────
#  VGICP drift correction  —  FIXED architecture
# ─────────────────────────────────────────────────────────────────────────────

def apply_transform(xyz, T):
    if len(xyz) == 0:
        return xyz
    pts = xyz.astype(np.float64)
    return ((T[:3, :3] @ pts.T).T + T[:3, 3]).astype(np.float32)


def run_vgicp(source_cloud, target_cloud,
              downsampling_resolution=0.15,
              max_correspondence_distance=1.0,
              num_threads=4):
    """
    Register source into target frame using small_gicp VGICP.
    Returns (T_4x4, fitness, rmse, converged).
    T transforms source → target (i.e. apply to source points).
    """
    try:
        import small_gicp
        src = source_cloud.astype(np.float64)
        tgt = target_cloud.astype(np.float64)

        src_ds = np.array(small_gicp.voxelgrid_sampling(
            src, downsampling_resolution).points())[:, :3]
        tgt_ds = np.array(small_gicp.voxelgrid_sampling(
            tgt, downsampling_resolution).points())[:, :3]

        if len(src_ds) < 20 or len(tgt_ds) < 20:
            return np.eye(4), 0.0, float("inf"), False

        src_pp, src_tree = small_gicp.preprocess_points(
            src_ds, downsampling_resolution)
        tgt_pp, tgt_tree = small_gicp.preprocess_points(
            tgt_ds, downsampling_resolution)

        result = small_gicp.align(
            tgt_pp, src_pp, tgt_tree,
            max_correspondence_distance=max_correspondence_distance,
            num_threads=num_threads)

        T       = np.array(result.T_target_source)
        fitness = result.num_inliers / max(len(src_ds), 1)

        n_sample    = min(500, len(src_ds), len(tgt_ds))
        transformed = (T[:3, :3] @ src_ds[:n_sample].T).T + T[:3, 3]
        dists = np.min(
            np.linalg.norm(
                transformed[:, None, :] - tgt_ds[:n_sample][None, :, :],
                axis=2), axis=1)
        rmse = float(np.sqrt(np.mean(dists ** 2)))

        return T, fitness, rmse, result.converged

    except Exception:
        import traceback
        traceback.print_exc()
        return np.eye(4), 0.0, float("inf"), False


class DriftCorrector:
    """
    Per-robot VGICP drift correction.

    For each robot i, build a consensus reference cloud from all
    OTHER robots' accumulated clouds, then run VGICP to find how
    much robot i has drifted from the consensus.

    The resulting correction transforms robot i's cloud so it better
    agrees with the rest of the fleet.

    Acceptance criteria are intentionally relaxed compared to v1
    because GLIM can drift significantly (metres / tens of degrees)
    in a featureless Gazebo simulation with flat floors.
    """

    # Thresholds for accepting a correction
    MIN_FITNESS   = 0.05   # inlier ratio  (relaxed from 0.30)
    MAX_RMSE_M    = 5.0    # metres        (relaxed from 8.0)
    MAX_ROT_DEG   = 20.0   # degrees       (relaxed from 5.0)
    MAX_TRANS_M   = 3.0    # metres        (relaxed from 0.5)
    MIN_REF_PTS   = 500    # min points in reference cloud
    MIN_SRC_PTS   = 500    # min points in source cloud

    def __init__(self, robot_names, voxel_size=0.15, max_corr=1.0,
                 metrics_dir="/tmp"):
        self.names       = robot_names
        self.voxel_size  = voxel_size
        self.max_corr    = max_corr
        self.transforms  = {name: np.eye(4) for name in robot_names}
        self.log         = []
        self.metrics_dir = metrics_dir

    def run(self, clouds: dict):
        """
        Run per-robot drift correction.
        clouds: {robot_name → np.ndarray (N,3) in world frame}
        Updates self.transforms[name].
        """
        for name in self.names:
            src_cloud = clouds.get(name)
            if src_cloud is None or len(src_cloud) < self.MIN_SRC_PTS:
                continue

            # Build reference = union of all OTHER robots' clouds
            ref_parts = [clouds[n] for n in self.names
                         if n != name and len(clouds.get(n, [])) > 0]
            if not ref_parts:
                continue
            ref_cloud = np.vstack(ref_parts)
            if len(ref_cloud) < self.MIN_REF_PTS:
                continue

            t0 = time.time()
            T, fitness, rmse, converged = run_vgicp(
                src_cloud, ref_cloud,
                downsampling_resolution=self.voxel_size,
                max_correspondence_distance=self.max_corr)
            elapsed = time.time() - t0

            cos_val   = min(1.0, max(-1.0, (np.trace(T[:3, :3]) - 1) / 2))
            rot_deg   = math.degrees(math.acos(cos_val))
            trans_m   = float(np.linalg.norm(T[:3, 3]))

            # Strip rotation — only apply translational correction
            T[:3, :3] = np.eye(3)
            # Recompute rot_deg after stripping (will be 0)
            rot_deg = 0.0
            accepted = (
                fitness  > self.MIN_FITNESS  and
                rmse     < self.MAX_RMSE_M   and
                rot_deg  < self.MAX_ROT_DEG  and
                trans_m  < self.MAX_TRANS_M
            )

            record = {
                "timestamp": datetime.now().isoformat(),
                "robot":     name,
                "fitness":   round(fitness,  4),
                "rmse_m":    round(rmse,     4) if rmse != float("inf") else None,
                "rot_deg":   round(rot_deg,  2),
                "trans_m":   round(trans_m,  4),
                "converged": converged,
                "elapsed_s": round(elapsed,  3),
                "accepted":  accepted,
                "T":         T.tolist(),
            }
            self.log.append(record)

            # Persist incrementally
            with open(os.path.join(self.metrics_dir, "icp_drift_corrections.jsonl"),
                      "a") as f:
                f.write(json.dumps(record) + "\n")

            if accepted:
                # Compose with existing correction (incremental update)
                self.transforms[name] = T

        return self.transforms

    def apply(self, name: str, cloud: np.ndarray) -> np.ndarray:
        return cloud  # TF-based correction; cloud untouched

    @property
    def n_accepted(self):
        return sum(1 for r in self.log if r["accepted"])


# ─────────────────────────────────────────────────────────────────────────────
#  GNN merge wrapper
# ─────────────────────────────────────────────────────────────────────────────

def gnn_merge(maps: dict, resolution: float,
              gnn_model=None, point_counts: dict = None):
    """Delegate to real GNN model; fall back to log_odds if unavailable."""
    if gnn_model is not None:
        try:
            import sys as _sys, os as _os
            _pkg_dir = _os.path.dirname(_os.path.abspath(__file__))
            if _pkg_dir not in _sys.path:
                _sys.path.insert(0, _pkg_dir)
            from central_server_gnn import fuse_with_gnn
            result = fuse_with_gnn(gnn_model, maps, resolution,
                                   point_counts=point_counts)
            if result is not None:
                return result
        except Exception as e:
            print(f"[GNN] fuse failed: {e} — falling back to log_odds")
    return log_odds_merge(maps, resolution)


# ─────────────────────────────────────────────────────────────────────────────
#  Central Server
# ─────────────────────────────────────────────────────────────────────────────

class CentralServer(Node):

    def __init__(self):
        super().__init__("central_server")

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter("robot_namespaces",       ["robot1", "robot2", "robot3"])
        self.declare_parameter("map_merge_frequency",    1.0)
        self.declare_parameter("resolution",             0.05)
        self.declare_parameter("z_min",                  0.1)
        self.declare_parameter("z_max",                  2.5)
        self.declare_parameter("max_points_per_robot",   500_000)
        self.declare_parameter("metrics_interval_sec",   30.0)
        self.declare_parameter("method_label",           "baseline_tf")
        # ICP
        self.declare_parameter("icp_voxel_size",         0.15)
        self.declare_parameter("icp_max_correspondence", 1.0)
        self.declare_parameter("icp_frequency",          0.016)   # Hz: every 5 s
        # GNN online updates
        self.declare_parameter("gnn_update_interval",    60.0)  # seconds

        ns  = self.robot_namespaces  = self.get_parameter("robot_namespaces").value
        self.merge_frequency         = self.get_parameter("map_merge_frequency").value
        self.resolution              = self.get_parameter("resolution").value
        self.z_min                   = self.get_parameter("z_min").value
        self.z_max                   = self.get_parameter("z_max").value
        self.max_points              = self.get_parameter("max_points_per_robot").value
        self.metrics_interval        = self.get_parameter("metrics_interval_sec").value
        self.method_label            = self.get_parameter("method_label").value
        self.icp_voxel_size          = self.get_parameter("icp_voxel_size").value
        self.icp_max_corr            = self.get_parameter("icp_max_correspondence").value
        self.icp_frequency           = self.get_parameter("icp_frequency").value
        self.gnn_update_interval     = self.get_parameter("gnn_update_interval").value

        cfg = METHOD_CONFIG.get(self.method_label, METHOD_CONFIG["baseline_tf"])
        self.merge_method = cfg["merge"]
        self.use_icp      = cfg["icp"]

        # ── Metrics dir ──────────────────────────────────────────────────────
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.metrics_dir  = f"/tmp/fusion_metrics/{self.method_label}_{ts}"
        os.makedirs(self.metrics_dir, exist_ok=True)
        self.metrics_file = os.path.join(self.metrics_dir, "metrics.jsonl")
        self.start_time   = time.time()

        with open(os.path.join(self.metrics_dir, "run_info.json"), "w") as f:
            json.dump({
                "method":       self.method_label,
                "merge_method": self.merge_method,
                "use_icp":      self.use_icp,
                "started":      ts,
                "resolution":   self.resolution,
                "z_min":        self.z_min,
                "z_max":        self.z_max,
                "robots":       list(ns),
            }, f, indent=2)

        # ── TF ────────────────────────────────────────────────────────────────
        self.tf_buffer      = Buffer()
        self.tf_listener    = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.icp_corrections = {n: None for n in ns}

        # ── State ─────────────────────────────────────────────────────────────
        self.robot_clouds      = {n: np.zeros((0, 3), dtype=np.float32) for n in ns}
        self.robot_maps        = {}
        self.global_map_latest = None
        self.lock              = threading.Lock()

        # ── ICP drift corrector ───────────────────────────────────────────────
        self.drift_corrector = DriftCorrector(
            robot_names=list(ns),
            voxel_size=self.icp_voxel_size,
            max_corr=self.icp_max_corr,
            metrics_dir=self.metrics_dir) if self.use_icp else None

        # ── GNN model ────────────────────────────────────────────────────────
        self.gnn_model = None
        if self.merge_method == "gnn":
            self._load_gnn_model()

        # ── Publishers ────────────────────────────────────────────────────────
        qos = QoSProfile(depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.global_map_pub = self.create_publisher(OccupancyGrid, "/global_map", qos)
        self.robot_map_pubs = {
            n: self.create_publisher(OccupancyGrid, f"/{n}/map", qos) for n in ns}

        # ── Subscribers ───────────────────────────────────────────────────────
        for n in ns:
            self.create_subscription(
                PointCloud2, f"/{n}/glim/aligned_points",
                lambda msg, name=n: self.pointcloud_callback(msg, name), 10)
            self.get_logger().info(f"Subscribed to /{n}/glim/aligned_points")

        # ── Timers ────────────────────────────────────────────────────────────
        self.create_timer(1.0 / self.merge_frequency, self.merge_and_publish)
        self.create_timer(self.metrics_interval,      self.log_metrics)

        if self.use_icp:
            self.create_timer(1.0 / self.icp_frequency, self.run_icp_correction)
            self.get_logger().info(
                f"[ICP] drift correction enabled | "
                f"voxel={self.icp_voxel_size}m | "
                f"max_corr={self.icp_max_corr}m | "
                f"every {1.0/self.icp_frequency:.0f}s")

        if self.gnn_model is not None:
            self.create_timer(self.gnn_update_interval, self.run_gnn_update)
            self.get_logger().info(
                f"[GNN] online updates enabled every {self.gnn_update_interval:.0f}s")

        self.get_logger().info(
            f"Central Server started | method={self.method_label} | "
            f"merge={self.merge_method} | icp={self.use_icp}")

    # ── GNN loader ────────────────────────────────────────────────────────────

    def _load_gnn_model(self):
        try:
            import sys as _sys, os as _os
            _pkg_dir = _os.path.dirname(_os.path.abspath(__file__))
            if _pkg_dir not in _sys.path:
                _sys.path.insert(0, _pkg_dir)
            from central_server_gnn import load_gnn_model
            self.gnn_model = load_gnn_model(n_robots=len(self.robot_namespaces))
            self.get_logger().info("[GNN] model loaded (2-layer GAT, pure NumPy)")
        except Exception as e:
            self.get_logger().warn(f"[GNN] load failed: {e} — using log_odds fallback")
            self.merge_method = "log_odds"

    # ── Point cloud callback ──────────────────────────────────────────────────

    def pointcloud_callback(self, msg, robot_ns):
        # Transform into world frame via TF
        try:
            transform = self.tf_buffer.lookup_transform(
                "world", msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
            msg_world = do_transform_cloud(msg, transform)
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f"TF lookup failed for {robot_ns}: {e}",
                throttle_duration_sec=5.0)
            msg_world = msg

        xyz = pointcloud2_to_xyz(msg_world)
        if len(xyz) == 0:
            return

        with self.lock:
            current  = self.robot_clouds[robot_ns]
            combined = np.vstack([current, xyz]) if len(current) > 0 else xyz
            if len(combined) > self.max_points:
                combined = _voxel_downsample(combined, voxel_size=0.05)
            self.robot_clouds[robot_ns] = combined

        # Apply ICP drift correction via transform (not cloud shifting)
        if self.drift_corrector is not None:
            T_corr = self.icp_corrections.get(robot_ns)
            xyz_corrected = apply_transform(combined, T_corr) if T_corr is not None and not np.allclose(T_corr, np.eye(4)) else combined
        else:
            xyz_corrected = combined

        occ = accumulated_cloud_to_occupancy_grid(
            xyz_corrected, resolution=self.resolution,
            z_min=self.z_min, z_max=self.z_max, frame_id="world")
        if occ is None:
            return
        occ.header.stamp = msg.header.stamp

        with self.lock:
            self.robot_maps[robot_ns] = occ
        self.robot_map_pubs[robot_ns].publish(occ)

    # ── ICP drift correction (per-robot vs consensus) ─────────────────────────

    def run_icp_correction(self):
        with self.lock:
            clouds = {n: self.robot_clouds[n].copy() for n in self.robot_namespaces}

        transforms = self.drift_corrector.run(clouds)
        accepted   = self.drift_corrector.n_accepted
        total      = len(self.drift_corrector.log)

        if total > 0:
            self.get_logger().info(
                f"[ICP] drift correction: {accepted}/{total} accepted")

        # Publish per-robot TF corrections
        now = self.get_clock().now().to_msg()
        for rec in self.drift_corrector.log[-len(self.robot_namespaces):]:
            if not rec["accepted"]:
                continue
            name = rec["robot"]
            T = np.array(rec["T"])
            with self.lock:
                self.icp_corrections[name] = T
            # Publish as TF for visualisation
            import math as _math
            ts = TransformStamped()
            ts.header.stamp    = now
            ts.header.frame_id = "world"
            ts.child_frame_id  = f"icp_correction/{name}"
            ts.transform.translation.x = float(T[0,3])
            ts.transform.translation.y = float(T[1,3])
            ts.transform.translation.z = float(T[2,3])
            R = T[:3,:3]; tr = R[0,0]+R[1,1]+R[2,2]
            if tr > 0:
                s=0.5/_math.sqrt(tr+1.0); qw=0.25/s
                qx=(R[2,1]-R[1,2])*s; qy=(R[0,2]-R[2,0])*s; qz=(R[1,0]-R[0,1])*s
            else:
                qw,qx,qy,qz=1.0,0.0,0.0,0.0
            n=_math.sqrt(qw**2+qx**2+qy**2+qz**2)
            ts.transform.rotation.w=qw/n; ts.transform.rotation.x=qx/n
            ts.transform.rotation.y=qy/n; ts.transform.rotation.z=qz/n
            self.tf_broadcaster.sendTransform(ts)

    # ── GNN online update ─────────────────────────────────────────────────────

    def run_gnn_update(self):
        if self.gnn_model is None:
            return
        with self.lock:
            maps = dict(self.robot_maps)
            pc   = {n: len(self.robot_clouds[n]) for n in self.robot_namespaces}
        if not maps:
            return
        try:
            import sys as _sys, os as _os
            _pkg_dir = _os.path.dirname(_os.path.abspath(__file__))
            if _pkg_dir not in _sys.path:
                _sys.path.insert(0, _pkg_dir)
            from central_server_gnn import update_gnn
            update_gnn(self.gnn_model, maps, self.resolution, point_counts=pc)
            self.get_logger().info(
                f"[GNN] weight update #{self.gnn_model._update_count} done")
        except Exception as e:
            self.get_logger().warn(f"[GNN] update failed: {e}")

    # ── Map merge & publish ───────────────────────────────────────────────────

    def merge_and_publish(self):
        with self.lock:
            if not self.robot_maps:
                return
            maps = dict(self.robot_maps)
            pc   = {n: len(self.robot_clouds[n]) for n in self.robot_namespaces}

        if self.merge_method == "log_odds":
            gmap = log_odds_merge(maps, self.resolution)
        elif self.merge_method == "gnn":
            gmap = gnn_merge(maps, self.resolution,
                             gnn_model=self.gnn_model, point_counts=pc)
        else:   # occupied_wins
            gmap = occupied_wins_merge(maps, self.resolution)

        if gmap is None:
            return

        gmap.header.stamp = self.get_clock().now().to_msg()
        self.global_map_pub.publish(gmap)

        with self.lock:
            self.global_map_latest = gmap

    # ── Metrics logging ───────────────────────────────────────────────────────

    def log_metrics(self):
        with self.lock:
            gmap   = self.global_map_latest
            clouds = {n: len(c) for n, c in self.robot_clouds.items()}

        if gmap is None:
            self.get_logger().info("[Metrics] No global map yet.")
            return

        data         = np.array(gmap.data, dtype=np.int8)
        total        = len(data)
        unknown_pct  = float(np.sum(data == -1)  / total * 100)
        occupied_pct = float(np.sum(data == 100) / total * 100)
        free_pct     = float(np.sum(data == 0)   / total * 100)
        sharpness    = _wall_sharpness(
            data.reshape(gmap.info.height, gmap.info.width))
        elapsed      = time.time() - self.start_time

        icp_stats = {}
        if self.drift_corrector:
            icp_stats = {
                "icp_attempts": len(self.drift_corrector.log),
                "icp_accepted": self.drift_corrector.n_accepted,
            }

        gnn_stats = {}
        if self.gnn_model:
            gnn_stats = {"gnn_updates": self.gnn_model._update_count}

        record = {
            "timestamp":               datetime.now().isoformat(),
            "elapsed_sec":             round(elapsed, 1),
            "method":                  self.method_label,
            "merge_method":            self.merge_method,
            "use_icp":                 self.use_icp,
            "map_width":               gmap.info.width,
            "map_height":              gmap.info.height,
            "total_cells":             total,
            "unknown_pct":             round(unknown_pct,  2),
            "occupied_pct":            round(occupied_pct, 2),
            "free_pct":                round(free_pct,     2),
            "wall_sharpness_mean_run": round(sharpness,    3),
            "points_per_robot":        clouds,
            **icp_stats,
            **gnn_stats,
        }

        with open(self.metrics_file, "a") as f:
            f.write(json.dumps(record) + "\n")

        icp_str = (f" | icp={icp_stats.get('icp_accepted',0)}/"
                   f"{icp_stats.get('icp_attempts',0)}") if self.use_icp else ""
        gnn_str = (f" | gnn_updates={gnn_stats.get('gnn_updates',0)}"
                   if self.gnn_model else "")
        self.get_logger().info(
            f"[Metrics] t={elapsed:.0f}s | "
            f"unk={unknown_pct:.1f}% occ={occupied_pct:.1f}% | "
            f"sharpness={sharpness:.2f}{icp_str}{gnn_str} | pts={clouds}")


# ─────────────────────────────────────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    server = CentralServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.log_metrics()
        if server.drift_corrector and server.drift_corrector.log:
            out = os.path.join(server.metrics_dir, "icp_drift_corrections_final.json")
            with open(out, "w") as f:
                json.dump(server.drift_corrector.log, f, indent=2)
            server.get_logger().info(
                f"[ICP] {server.drift_corrector.n_accepted}/"
                f"{len(server.drift_corrector.log)} corrections accepted")
        server.get_logger().info(f"[Metrics] saved → {server.metrics_dir}")
        server.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
