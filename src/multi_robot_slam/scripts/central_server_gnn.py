#!/usr/bin/env python3
"""
Central Server — Stage 3a Week 4: GNN-based Map Fusion

Uses data-derived optimal fusion weights learned from 12 training runs.
Each robot's map contribution is weighted by its average quality relative
to the SDF ground truth, computed via constrained least-squares optimisation
across all collected training samples.

Learned weights (mean optimal across 12 samples):
  robot1: 0.403  (highest — most spatially consistent with GT)
  robot2: 0.284
  robot3: 0.314

Thesis context
--------------
  Training on 12 samples revealed systematic quality differences between
  robots. The GNN converged to predicting mean optimal weights rather than
  per-scene weights, indicating the quality difference is consistent across
  runs. These data-driven weights replace the hand-designed rules used in
  baseline (occupied-wins), ICP (occupied-wins after correction), and
  probabilistic (log-odds) methods.

Difference from other methods
------------------------------
  Baseline:      equal implicit weight, deterministic occupied-wins
  ICP:           equal implicit weight after geometric correction
  Probabilistic: weight by observation count (log-odds)
  GNN (this):    weight by learned data-driven quality score per robot
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from tf2_ros import (Buffer, TransformListener,
                     LookupException, ConnectivityException,
                     ExtrapolationException)
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import numpy as np
import struct, threading, json, os, time
from datetime import datetime


# ─────────────────────────────────────────────
#  Learned fusion weights
#  Loaded from file; fallback to hardcoded values
# ─────────────────────────────────────────────
WEIGHTS_PATH = "/ros2_ws/results/gnn/learned_weights.npy"
FALLBACK_WEIGHTS = np.array([0.403, 0.284, 0.314], dtype=np.float32)


def load_weights(path):
    if os.path.exists(path):
        w = np.load(path).astype(np.float32)
        w = np.clip(w, 0, 1)
        w /= w.sum()
        return w
    return FALLBACK_WEIGHTS.copy()


# ─────────────────────────────────────────────
#  Shared helpers (identical to other methods)
# ─────────────────────────────────────────────

def pointcloud2_to_xyz(msg):
    offsets = {}
    for field in msg.fields:
        if field.name in ("x","y","z"):
            offsets[field.name] = field.offset
    if not all(k in offsets for k in ("x","y","z")):
        return np.array([])
    n, step, data = msg.width*msg.height, msg.point_step, msg.data
    xyz = np.zeros((n,3), dtype=np.float32)
    for i in range(n):
        base = i*step
        xyz[i,0] = struct.unpack_from("f",data,base+offsets["x"])[0]
        xyz[i,1] = struct.unpack_from("f",data,base+offsets["y"])[0]
        xyz[i,2] = struct.unpack_from("f",data,base+offsets["z"])[0]
    return xyz[np.isfinite(xyz).all(axis=1)]


def accumulated_cloud_to_occupancy_grid(xyz, resolution=0.05,
                                        z_min=0.1, z_max=2.5,
                                        frame_id="world"):
    if len(xyz) == 0: return None
    mask  = (xyz[:,2]>=z_min)&(xyz[:,2]<=z_max)
    pts2d = xyz[mask,:2]
    if len(pts2d) == 0: return None
    floor_mask = (xyz[:,2]>=-0.5)&(xyz[:,2]<z_min)
    floor_pts  = xyz[floor_mask,:2]
    all_pts = np.vstack([pts2d,floor_pts]) if len(floor_pts)>0 else pts2d
    mn_x=all_pts[:,0].min()-1.0; mn_y=all_pts[:,1].min()-1.0
    mx_x=all_pts[:,0].max()+1.0; mx_y=all_pts[:,1].max()+1.0
    W=int(np.ceil((mx_x-mn_x)/resolution))
    H=int(np.ceil((mx_y-mn_y)/resolution))
    grid=np.full((H,W),-1,dtype=np.int8)
    if len(floor_pts)>0:
        gx=((floor_pts[:,0]-mn_x)/resolution).astype(int)
        gy=((floor_pts[:,1]-mn_y)/resolution).astype(int)
        v=(gx>=0)&(gx<W)&(gy>=0)&(gy<H)
        grid[gy[v],gx[v]]=0
    gx=((pts2d[:,0]-mn_x)/resolution).astype(int)
    gy=((pts2d[:,1]-mn_y)/resolution).astype(int)
    v=(gx>=0)&(gx<W)&(gy>=0)&(gy<H)
    grid[gy[v],gx[v]]=100
    msg=OccupancyGrid()
    msg.info.resolution=resolution
    msg.info.width=W; msg.info.height=H
    msg.info.origin.position.x=float(mn_x)
    msg.info.origin.position.y=float(mn_y)
    msg.info.origin.orientation.w=1.0
    msg.header.frame_id=frame_id
    msg.data=grid.flatten().tolist()
    return msg


def _voxel_downsample(xyz, voxel_size=0.05):
    voxels=np.floor(xyz/voxel_size).astype(int)
    _,idx=np.unique(voxels,axis=0,return_index=True)
    return xyz[idx]


def _wall_sharpness(grid_np):
    occupied=(grid_np==100)
    run_lengths=[]
    for row in occupied:
        in_run,length=False,0
        for val in row:
            if val: in_run=True; length+=1
            elif in_run: run_lengths.append(length); in_run,length=False,0
        if in_run: run_lengths.append(length)
    for col in occupied.T:
        in_run,length=False,0
        for val in col:
            if val: in_run=True; length+=1
            elif in_run: run_lengths.append(length); in_run,length=False,0
        if in_run: run_lengths.append(length)
    return float(np.mean(run_lengths)) if run_lengths else 0.0


# ─────────────────────────────────────────────
#  GNN weighted merge
# ─────────────────────────────────────────────

def gnn_weighted_merge(maps: dict, weights: np.ndarray,
                       robot_namespaces: list, resolution: float):
    """
    Merge per-robot OccupancyGrids using learned fusion weights.

    For each cell:
      If multiple robots observed it:
        weighted_score = sum(w_i * value_i) / sum(w_i for observed robots)
        score > 0.5  → occupied (100)
        score < 0.4  → free (0)
        otherwise    → unknown (-1)
      If only one robot observed it:
        use that robot's value directly (with its weight as confidence)
      If no robot observed it:
        unknown (-1)

    This differs from baseline occupied-wins in that a high-weight robot
    marking a cell as free can override a low-weight robot marking it
    as occupied — capturing the learned reliability differences.
    """
    if not maps: return None

    # Global bounding box
    mn_x=mn_y= float("inf")
    mx_x=mx_y= float("-inf")
    for m in maps.values():
        ox=m.info.origin.position.x; oy=m.info.origin.position.y
        mn_x=min(mn_x,ox); mn_y=min(mn_y,oy)
        mx_x=max(mx_x,ox+m.info.width*resolution)
        mx_y=max(mx_y,oy+m.info.height*resolution)

    W=int(np.ceil((mx_x-mn_x)/resolution))
    H=int(np.ceil((mx_y-mn_y)/resolution))

    # Accumulators: weighted sum and weight sum per cell
    weighted_sum = np.zeros((H,W), dtype=np.float32)
    weight_total = np.zeros((H,W), dtype=np.float32)

    for robot_name, m in maps.items():
        # Get this robot's learned weight
        if robot_name in robot_namespaces:
            idx = robot_namespaces.index(robot_name)
            w_i = float(weights[idx])
        else:
            w_i = 1.0 / len(robot_namespaces)

        ox    = m.info.origin.position.x
        oy    = m.info.origin.position.y
        mw,mh = m.info.width, m.info.height
        x_off = int(round((ox-mn_x)/resolution))
        y_off = int(round((oy-mn_y)/resolution))

        data = np.array(m.data, dtype=np.int8).reshape(mh,mw)

        # Convert to float: occupied=1.0, free=0.0, unknown=skip
        gy,gx = np.meshgrid(np.arange(mh)+y_off,
                             np.arange(mw)+x_off, indexing="ij")
        valid = (gx>=0)&(gx<W)&(gy>=0)&(gy<H)

        # Only update cells with known observations (not -1)
        known = valid & (data != -1)

        # Normalise: 100→1.0, 0→0.0
        cell_val = np.where(data==100, 1.0, 0.0).astype(np.float32)

        weighted_sum[gy[known],gx[known]] += w_i * cell_val[known]
        weight_total[gy[known],gx[known]] += w_i

    # Convert to occupancy
    merged = np.full((H,W), -1, dtype=np.int8)
    observed = weight_total > 0
    score = np.zeros((H,W), dtype=np.float32)
    score[observed] = weighted_sum[observed] / weight_total[observed]

    merged[observed & (score > 0.5)] = 100
    merged[observed & (score < 0.4)] = 0
    # score in [0.4, 0.5] stays -1 (uncertain/conflicted)

    gmap = OccupancyGrid()
    gmap.header.frame_id            = "world"
    gmap.info.resolution            = resolution
    gmap.info.width                 = W
    gmap.info.height                = H
    gmap.info.origin.position.x    = mn_x
    gmap.info.origin.position.y    = mn_y
    gmap.info.origin.orientation.w = 1.0
    gmap.data = merged.flatten().tolist()
    return gmap


# ─────────────────────────────────────────────
#  Central Server node
# ─────────────────────────────────────────────

class CentralServer(Node):
    def __init__(self):
        super().__init__("central_server")

        self.declare_parameter("robot_namespaces",     ["robot1","robot2","robot3"])
        self.declare_parameter("map_merge_frequency",  1.0)
        self.declare_parameter("resolution",           0.05)
        self.declare_parameter("z_min",                0.1)
        self.declare_parameter("z_max",                2.5)
        self.declare_parameter("max_points_per_robot", 500000)
        self.declare_parameter("metrics_interval_sec", 30.0)
        self.declare_parameter("method_label",         "gnn")
        self.declare_parameter("weights_path",         WEIGHTS_PATH)

        self.robot_namespaces = self.get_parameter("robot_namespaces").value
        self.merge_frequency  = self.get_parameter("map_merge_frequency").value
        self.resolution       = self.get_parameter("resolution").value
        self.z_min            = self.get_parameter("z_min").value
        self.z_max            = self.get_parameter("z_max").value
        self.max_points       = self.get_parameter("max_points_per_robot").value
        self.metrics_interval = self.get_parameter("metrics_interval_sec").value
        self.method_label     = self.get_parameter("method_label").value
        weights_path          = self.get_parameter("weights_path").value

        # Load learned weights
        self.fusion_weights = load_weights(weights_path)
        self.get_logger().info(
            f"Fusion weights loaded from {weights_path}: "
            + "  ".join(f"{r}={self.fusion_weights[i]:.3f}"
                        for i,r in enumerate(self.robot_namespaces)))

        # Metrics
        run_ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.metrics_dir  = f"/tmp/fusion_metrics/{self.method_label}_{run_ts}"
        os.makedirs(self.metrics_dir, exist_ok=True)
        self.metrics_file = os.path.join(self.metrics_dir, "metrics.jsonl")
        self.start_time   = time.time()
        with open(os.path.join(self.metrics_dir,"run_info.json"),"w") as f:
            json.dump({
                "method":          self.method_label,
                "started":         run_ts,
                "resolution":      self.resolution,
                "z_min":           self.z_min,
                "z_max":           self.z_max,
                "robots":          list(self.robot_namespaces),
                "fusion_weights":  self.fusion_weights.tolist(),
                "weights_source":  weights_path,
            }, f, indent=2)
        self.get_logger().info(f"Metrics → {self.metrics_dir}")

        # TF
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State
        self.robot_clouds = {n: np.zeros((0,3),dtype=np.float32)
                             for n in self.robot_namespaces}
        self.robot_maps        = {}
        self.global_map_latest = None
        self.lock = threading.Lock()

        # Publishers
        qos = QoSProfile(depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.global_map_pub = self.create_publisher(OccupancyGrid,"/global_map",qos)
        self.robot_map_pubs = {
            n: self.create_publisher(OccupancyGrid,f"/{n}/map",qos)
            for n in self.robot_namespaces}

        # Subscribers
        for name in self.robot_namespaces:
            self.create_subscription(
                PointCloud2, f"/{name}/glim/aligned_points",
                lambda msg,n=name: self.pointcloud_callback(msg,n), 10)
            self.get_logger().info(f"Subscribed to /{name}/glim/aligned_points")

        # Timers
        self.merge_timer   = self.create_timer(
            1.0/self.merge_frequency, self.merge_and_publish)
        self.metrics_timer = self.create_timer(
            self.metrics_interval, self.log_metrics)

        self.get_logger().info(
            f"Central Server (GNN learned-weight fusion) started  "
            f"method={self.method_label}")

    # ── upstream pipeline (identical to all other methods) ────────────────

    def pointcloud_callback(self, msg, robot_ns):
        try:
            transform = self.tf_buffer.lookup_transform(
                "world", msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
            msg_world = do_transform_cloud(msg, transform)
        except (LookupException,ConnectivityException,ExtrapolationException) as e:
            self.get_logger().warn(
                f"TF lookup failed for {robot_ns}: {e}",
                throttle_duration_sec=5.0)
            msg_world = msg

        xyz = pointcloud2_to_xyz(msg_world)
        if len(xyz) == 0: return

        with self.lock:
            cur = self.robot_clouds[robot_ns]
            combined = np.vstack([cur,xyz]) if len(cur)>0 else xyz
            if len(combined) > self.max_points:
                combined = _voxel_downsample(combined, voxel_size=0.05)
            self.robot_clouds[robot_ns] = combined

        occ = accumulated_cloud_to_occupancy_grid(
            combined, resolution=self.resolution,
            z_min=self.z_min, z_max=self.z_max, frame_id="world")
        if occ is None: return
        occ.header.stamp = msg.header.stamp

        with self.lock:
            self.robot_maps[robot_ns] = occ
        self.robot_map_pubs[robot_ns].publish(occ)

    # ── GNN weighted merge ────────────────────────────────────────────────

    def merge_and_publish(self):
        with self.lock:
            if not self.robot_maps: return
            maps = dict(self.robot_maps)

        global_map = gnn_weighted_merge(
            maps, self.fusion_weights,
            list(self.robot_namespaces), self.resolution)
        if global_map is None: return

        global_map.header.stamp = self.get_clock().now().to_msg()
        self.global_map_pub.publish(global_map)

        with self.lock:
            self.global_map_latest = global_map

    # ── metrics (identical schema to baseline, ICP, probabilistic) ────────

    def log_metrics(self):
        with self.lock:
            gmap   = self.global_map_latest
            clouds = {n: len(c) for n,c in self.robot_clouds.items()}

        if gmap is None:
            self.get_logger().info("[Metrics] No map yet")
            return

        data         = np.array(gmap.data, dtype=np.int8)
        total        = len(data)
        unknown_pct  = float(np.sum(data==-1)  /total*100)
        occupied_pct = float(np.sum(data==100) /total*100)
        free_pct     = float(np.sum(data==0)   /total*100)
        sharpness    = _wall_sharpness(data.reshape(gmap.info.height,
                                                    gmap.info.width))
        elapsed      = time.time()-self.start_time

        record = {
            "timestamp":               datetime.now().isoformat(),
            "elapsed_sec":             round(elapsed,1),
            "method":                  self.method_label,
            "map_width":               gmap.info.width,
            "map_height":              gmap.info.height,
            "total_cells":             total,
            "unknown_pct":             round(unknown_pct,2),
            "occupied_pct":            round(occupied_pct,2),
            "free_pct":                round(free_pct,2),
            "wall_sharpness_mean_run": round(sharpness,3),
            "points_per_robot":        clouds,
            "fusion_weights":          self.fusion_weights.tolist(),
        }
        with open(self.metrics_file,"a") as f:
            f.write(json.dumps(record)+"\n")
        self.get_logger().info(
            f"[Metrics] t={elapsed:.0f}s | "
            f"unknown={unknown_pct:.1f}% occ={occupied_pct:.1f}% "
            f"free={free_pct:.1f}% sharpness={sharpness:.2f} | "
            f"pts={clouds}")


# ─────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    server = CentralServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.log_metrics()
        server.get_logger().info(
            f"Final snapshot → {server.metrics_dir}")
        server.destroy_node()
        try: rclpy.shutdown()
        except: pass

if __name__ == "__main__":
    main()
