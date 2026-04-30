# ICP alignmetn new 

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import numpy as np
import struct
import threading
import json
import os
import time
import math
from datetime import datetime
from itertools import combinations

import small_gicp


# ── helpers ───────────────────────────────────────────────────────────────────

def pointcloud2_to_xyz(msg):
    offsets = {}
    for field in msg.fields:
        if field.name in ("x", "y", "z"):
            offsets[field.name] = field.offset
    if not all(k in offsets for k in ("x", "y", "z")):
        return np.array([])
    n = msg.width * msg.height
    step = msg.point_step
    data = msg.data
    xyz = np.zeros((n, 3), dtype=np.float32)
    for i in range(n):
        base = i * step
        xyz[i, 0] = struct.unpack_from("f", data, base + offsets["x"])[0]
        xyz[i, 1] = struct.unpack_from("f", data, base + offsets["y"])[0]
        xyz[i, 2] = struct.unpack_from("f", data, base + offsets["z"])[0]
    valid = np.isfinite(xyz).all(axis=1)
    return xyz[valid]


def accumulated_cloud_to_occupancy_grid(xyz, resolution=0.05,
                                        z_min=0.1, z_max=2.5, frame_id="world"):
    if len(xyz) == 0:
        return None
    mask = (xyz[:, 2] >= z_min) & (xyz[:, 2] <= z_max)
    pts2d = xyz[mask, :2]
    if len(pts2d) == 0:
        return None
    floor_mask = (xyz[:, 2] >= -0.5) & (xyz[:, 2] < z_min)
    floor_pts = xyz[floor_mask, :2]
    all_pts = np.vstack([pts2d, floor_pts]) if len(floor_pts) > 0 else pts2d
    min_x = all_pts[:, 0].min() - 1.0
    min_y = all_pts[:, 1].min() - 1.0
    max_x = all_pts[:, 0].max() + 1.0
    max_y = all_pts[:, 1].max() + 1.0
    width  = int(np.ceil((max_x - min_x) / resolution))
    height = int(np.ceil((max_y - min_y) / resolution))
    grid = np.full((height, width), -1, dtype=np.int8)
    if len(floor_pts) > 0:
        gx = ((floor_pts[:, 0] - min_x) / resolution).astype(int)
        gy = ((floor_pts[:, 1] - min_y) / resolution).astype(int)
        valid = (gx >= 0) & (gx < width) & (gy >= 0) & (gy < height)
        grid[gy[valid], gx[valid]] = 0
    gx = ((pts2d[:, 0] - min_x) / resolution).astype(int)
    gy = ((pts2d[:, 1] - min_y) / resolution).astype(int)
    valid = (gx >= 0) & (gx < width) & (gy >= 0) & (gy < height)
    grid[gy[valid], gx[valid]] = 100
    msg = OccupancyGrid()
    msg.info.resolution = resolution
    msg.info.width = width
    msg.info.height = height
    msg.info.origin.position.x = float(min_x)
    msg.info.origin.position.y = float(min_y)
    msg.info.origin.orientation.w = 1.0
    msg.header.frame_id = frame_id
    msg.data = grid.flatten().tolist()
    return msg


def _voxel_downsample(xyz, voxel_size=0.05):
    voxels = np.floor(xyz / voxel_size).astype(int)
    _, unique_idx = np.unique(voxels, axis=0, return_index=True)
    return xyz[unique_idx]


def _wall_sharpness(grid_np):
    occupied = (grid_np == 100)
    run_lengths = []
    for row in occupied:
        in_run, length = False, 0
        for val in row:
            if val:
                in_run = True
                length += 1
            elif in_run:
                run_lengths.append(length)
                in_run, length = False, 0
        if in_run:
            run_lengths.append(length)
    for col in occupied.T:
        in_run, length = False, 0
        for val in col:
            if val:
                in_run = True
                length += 1
            elif in_run:
                run_lengths.append(length)
                in_run, length = False, 0
        if in_run:
            run_lengths.append(length)
    return float(np.mean(run_lengths)) if run_lengths else 0.0


# ── ICP alignment functions ───────────────────────────────────────────────────

def voxel_overlap(cloud_a, cloud_b, voxel_size=0.1):
    """
    Compute fraction of cloud_a voxels that overlap with cloud_b voxels.
    Borrowed from GLIM's submap overlap criterion (threshold = 0.05).
    """
    if len(cloud_a) == 0 or len(cloud_b) == 0:
        return 0.0
    voxels_b = set()
    voxels_b_arr = np.floor(cloud_b / voxel_size).astype(int)
    for v in voxels_b_arr:
        voxels_b.add((v[0], v[1], v[2]))
    voxels_a_arr = np.floor(cloud_a / voxel_size).astype(int)
    hits = sum(1 for v in voxels_a_arr if (v[0], v[1], v[2]) in voxels_b)
    return hits / len(cloud_a)


def run_vgicp(source_cloud, target_cloud,
              downsampling_resolution=0.1,
              max_correspondence_distance=0.5,
              num_threads=4):
    """
    Run V-GICP alignment using small_gicp.
    Same algorithm GLIM uses internally for scan-to-map registration.

    Returns: T (4x4), fitness (float), rmse (float), converged (bool)
    """
    try:
        src = source_cloud.astype(np.float64)
        tgt = target_cloud.astype(np.float64)

        # Downsample — voxelgrid_sampling returns PointCloud
        # .points() returns (N,4) XYZW array, take first 3 columns
        src_pc = small_gicp.voxelgrid_sampling(src, downsampling_resolution)
        tgt_pc = small_gicp.voxelgrid_sampling(tgt, downsampling_resolution)
        src_ds = np.array(src_pc.points())[:, :3]
        tgt_ds = np.array(tgt_pc.points())[:, :3]

        if len(src_ds) < 20 or len(tgt_ds) < 20:
            return np.eye(4), 0.0, float('inf'), False

        # Preprocess: normals + covariances required for GICP
        src_pp, src_tree = small_gicp.preprocess_points(
            src_ds, downsampling_resolution)
        tgt_pp, tgt_tree = small_gicp.preprocess_points(
            tgt_ds, downsampling_resolution)

        # Run V-GICP
        result = small_gicp.align(
            tgt_pp, src_pp, tgt_tree,
            max_correspondence_distance=max_correspondence_distance,
            num_threads=num_threads)

        T           = np.array(result.T_target_source)
        converged   = result.converged
        num_inliers = result.num_inliers
        fitness     = num_inliers / max(len(src_ds), 1)

        # Compute RMSE on downsampled points
        n_sample    = min(500, len(src_ds), len(tgt_ds))
        src_sample  = src_ds[:n_sample]
        tgt_sample  = tgt_ds[:n_sample]
        transformed = (T[:3, :3] @ src_sample.T).T + T[:3, 3]
        dists = np.min(np.linalg.norm(
            transformed[:, None, :] - tgt_sample[None, :, :], axis=2), axis=1)
        rmse = float(np.sqrt(np.mean(dists ** 2)))

        return T, fitness, rmse, converged

    except Exception as e:
        import traceback
        print(f"[run_vgicp] EXCEPTION: {type(e).__name__}: {e}")
        traceback.print_exc()
        print(f"[run_vgicp] src shape={source_cloud.shape} dtype={source_cloud.dtype}")
        print(f"[run_vgicp] tgt shape={target_cloud.shape} dtype={target_cloud.dtype}")
        return np.eye(4), 0.0, float('inf'), False


def apply_transform(xyz, T):
    """Apply 4x4 homogeneous transform to Nx3 point cloud."""
    if len(xyz) == 0:
        return xyz
    pts = xyz.astype(np.float64)
    transformed = (T[:3, :3] @ pts.T).T + T[:3, 3]
    return transformed.astype(np.float32)


# ── main node ─────────────────────────────────────────────────────────────────

class CentralServer(Node):
    def __init__(self):
        super().__init__("central_server")

        self.declare_parameter("robot_namespaces", ["robot1", "robot2", "robot3"])
        self.declare_parameter("map_merge_frequency", 1.0)
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("z_min", 0.1)
        self.declare_parameter("z_max", 2.5)
        self.declare_parameter("max_points_per_robot", 500000)
        self.declare_parameter("metrics_interval_sec", 30.0)
        self.declare_parameter("method_label", "icp")
        self.declare_parameter("overlap_threshold", 0.05)
        self.declare_parameter("icp_voxel_size", 0.1)
        self.declare_parameter("icp_max_correspondence", 0.5)
        self.declare_parameter("icp_min_points", 1000)
        self.declare_parameter("icp_frequency", 5.0)

        self.robot_namespaces  = self.get_parameter("robot_namespaces").value
        self.merge_frequency   = self.get_parameter("map_merge_frequency").value
        self.resolution        = self.get_parameter("resolution").value
        self.z_min             = self.get_parameter("z_min").value
        self.z_max             = self.get_parameter("z_max").value
        self.max_points        = self.get_parameter("max_points_per_robot").value
        self.metrics_interval  = self.get_parameter("metrics_interval_sec").value
        self.method_label      = self.get_parameter("method_label").value
        self.overlap_threshold = self.get_parameter("overlap_threshold").value
        self.icp_voxel_size    = self.get_parameter("icp_voxel_size").value
        self.icp_max_corr      = self.get_parameter("icp_max_correspondence").value
        self.icp_min_points    = self.get_parameter("icp_min_points").value
        self.icp_frequency     = self.get_parameter("icp_frequency").value

        run_ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.metrics_dir  = f"/tmp/fusion_metrics/{self.method_label}_{run_ts}"
        os.makedirs(self.metrics_dir, exist_ok=True)
        self.metrics_file = os.path.join(self.metrics_dir, "metrics.jsonl")
        self.start_time   = time.time()

        with open(os.path.join(self.metrics_dir, "run_info.json"), "w") as f:
            json.dump({
                "method":                 self.method_label,
                "started":                run_ts,
                "resolution":             self.resolution,
                "z_min":                  self.z_min,
                "z_max":                  self.z_max,
                "robots":                 list(self.robot_namespaces),
                "overlap_threshold":      self.overlap_threshold,
                "icp_voxel_size":         self.icp_voxel_size,
                "icp_max_correspondence": self.icp_max_corr,
            }, f, indent=2)

        self.get_logger().info(
            f"[ICP Server] method={self.method_label} | "
            f"overlap_threshold={self.overlap_threshold} | "
            f"icp_voxel={self.icp_voxel_size}m | "
            f"metrics -> {self.metrics_dir}")

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.robot_clouds = {name: np.zeros((0, 3), dtype=np.float32)
                             for name in self.robot_namespaces}
        self.robot_maps        = {}
        self.global_map_latest = None
        self.lock              = threading.Lock()

        # Correction transforms — replace, never compose
        self.robot_transforms = {name: np.eye(4)
                                 for name in self.robot_namespaces}

        self.icp_log = []

        qos = QoSProfile(depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.global_map_pub = self.create_publisher(OccupancyGrid, "/global_map", qos)
        self.robot_map_pubs = {
            name: self.create_publisher(OccupancyGrid, f"/{name}/map", qos)
            for name in self.robot_namespaces
        }

        for name in self.robot_namespaces:
            self.create_subscription(
                PointCloud2, f"/{name}/glim/aligned_points",
                lambda msg, n=name: self.pointcloud_callback(msg, n), 10)
            self.get_logger().info(f"Subscribed to /{name}/glim/aligned_points")

        self.merge_timer   = self.create_timer(1.0 / self.merge_frequency,
                                               self.merge_and_publish)
        self.icp_timer     = self.create_timer(self.icp_frequency,
                                               self.run_icp_alignment)
        self.metrics_timer = self.create_timer(self.metrics_interval,
                                               self.log_metrics)

        self.get_logger().info("Central Server (V-GICP ICP fusion) started")

    # ── point cloud callback ──────────────────────────────────────────────────

    def pointcloud_callback(self, msg, robot_ns):
        try:
            transform = self.tf_buffer.lookup_transform(
                'world', msg.header.frame_id,
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

        T = self.robot_transforms[robot_ns]
        xyz_corrected = apply_transform(combined, T) \
                        if not np.allclose(T, np.eye(4)) else combined

        occ = accumulated_cloud_to_occupancy_grid(
            xyz_corrected, resolution=self.resolution,
            z_min=self.z_min, z_max=self.z_max, frame_id='world')
        if occ is None:
            return
        occ.header.stamp = msg.header.stamp

        with self.lock:
            self.robot_maps[robot_ns] = occ
        self.robot_map_pubs[robot_ns].publish(occ)

    # ── V-GICP alignment ─────────────────────────────────────────────────────

    def run_icp_alignment(self):
        with self.lock:
            clouds = {n: self.robot_clouds[n].copy()
                      for n in self.robot_namespaces}

        for robot_a, robot_b in combinations(self.robot_namespaces, 2):
            cloud_a = clouds[robot_a]
            cloud_b = clouds[robot_b]

            if len(cloud_a) < self.icp_min_points or \
               len(cloud_b) < self.icp_min_points:
                self.get_logger().info(
                    f"[ICP] {robot_a}/{robot_b}: insufficient points "
                    f"({len(cloud_a)}/{len(cloud_b)}) — skipping",
                    throttle_duration_sec=10.0)
                continue

            # Step 1: voxel overlap check
            overlap = voxel_overlap(cloud_a, cloud_b,
                                    voxel_size=self.icp_voxel_size)

            if overlap < self.overlap_threshold:
                self.get_logger().info(
                    f"[ICP] {robot_a}/{robot_b}: overlap={overlap:.3f} "
                    f"< {self.overlap_threshold} — skipping",
                    throttle_duration_sec=10.0)
                continue

            self.get_logger().info(
                f"[ICP] {robot_a}/{robot_b}: overlap={overlap:.3f} — running V-GICP")

            # Step 2: V-GICP alignment
            t0 = time.time()
            T, fitness, rmse, converged = run_vgicp(
                cloud_a, cloud_b,
                downsampling_resolution=self.icp_voxel_size,
                max_correspondence_distance=self.icp_max_corr)
            elapsed = time.time() - t0

            # Validate transform — reject large rotations or translations
            rot_angle  = math.degrees(math.acos(
                min(1.0, max(-1.0, (np.trace(T[:3, :3]) - 1) / 2))))
            trans_dist = np.linalg.norm(T[:3, 3])

            self.get_logger().info(
                f"[ICP] {robot_a}/{robot_b}: "
                f"fitness={fitness:.3f} rmse={rmse:.4f}m "
                f"rot={rot_angle:.1f}deg trans={trans_dist:.3f}m "
                f"converged={converged} t={elapsed:.2f}s")

            icp_record = {
                "timestamp":   datetime.now().isoformat(),
                "pair":        f"{robot_a}/{robot_b}",
                "overlap":     round(overlap, 4),
                "fitness":     round(fitness, 4),
                "rmse_m":      round(rmse, 4) if rmse != float('inf') else None,
                "converged":   converged,
                "rot_deg":     round(rot_angle, 2),
                "trans_m":     round(trans_dist, 4),
                "elapsed_s":   round(elapsed, 3),
                "accepted":    False,
                "T":           T.tolist(),
            }

            # Accept only small, reasonable corrections
            if (converged
                    and fitness > 0.3
                    and rmse < 3.0
                    and rot_angle < 15.0
                    and trans_dist < 1.0):
                with self.lock:
                    # Replace transform — never compose to avoid drift
                    self.robot_transforms[robot_a] = T
                icp_record["accepted"] = True
                self.get_logger().info(
                    f"[ICP] ✓ Applied transform to {robot_a}")
            else:
                self.get_logger().warn(
                    f"[ICP] ✗ Rejected transform {robot_a}/{robot_b}")

            self.icp_log.append(icp_record)
            with open(os.path.join(self.metrics_dir, "icp_alignments.jsonl"), "a") as f:
                f.write(json.dumps(icp_record) + "\n")

    # ── map merge ─────────────────────────────────────────────────────────────

    def merge_and_publish(self):
        with self.lock:
            if not self.robot_maps:
                return
            maps = dict(self.robot_maps)

        resolution = self.resolution
        min_x = min_y =  float("inf")
        max_x = max_y = float("-inf")
        for m in maps.values():
            ox = m.info.origin.position.x
            oy = m.info.origin.position.y
            min_x = min(min_x, ox)
            min_y = min(min_y, oy)
            max_x = max(max_x, ox + m.info.width  * resolution)
            max_y = max(max_y, oy + m.info.height * resolution)

        width  = int(np.ceil((max_x - min_x) / resolution))
        height = int(np.ceil((max_y - min_y) / resolution))
        merged = np.full(height * width, -1, dtype=np.int8)

        for m in maps.values():
            ox    = m.info.origin.position.x
            oy    = m.info.origin.position.y
            x_off = int(round((ox - min_x) / resolution))
            y_off = int(round((oy - min_y) / resolution))
            data  = np.array(m.data, dtype=np.int8).reshape(
                m.info.height, m.info.width)
            cc, rr = np.meshgrid(np.arange(m.info.width),
                                 np.arange(m.info.height))
            gx = x_off + cc
            gy = y_off + rr
            valid = (gx >= 0) & (gx < width) & (gy >= 0) & (gy < height)
            idx   = (gy * width + gx)[valid]
            cell  = data[valid]
            merged[idx[cell == 100]] = 100
            free_mask = (cell == 0) & (merged[idx] != 100)
            merged[idx[free_mask]] = 0

        global_map = OccupancyGrid()
        global_map.header.stamp              = self.get_clock().now().to_msg()
        global_map.header.frame_id           = "world"
        global_map.info.resolution           = resolution
        global_map.info.width                = width
        global_map.info.height               = height
        global_map.info.origin.position.x    = min_x
        global_map.info.origin.position.y    = min_y
        global_map.info.origin.orientation.w = 1.0
        global_map.data = merged.tolist()
        self.global_map_pub.publish(global_map)

        with self.lock:
            self.global_map_latest = global_map

    # ── metrics logging ───────────────────────────────────────────────────────

    def log_metrics(self):
        with self.lock:
            gmap       = self.global_map_latest
            clouds     = {n: len(c) for n, c in self.robot_clouds.items()}
            recent_icp = self.icp_log[-3:] if self.icp_log else []

        if gmap is None:
            return

        data         = np.array(gmap.data, dtype=np.int8)
        total        = len(data)
        unknown_pct  = float(np.sum(data == -1)  / total * 100)
        occupied_pct = float(np.sum(data == 100) / total * 100)
        free_pct     = float(np.sum(data == 0)   / total * 100)
        sharpness    = _wall_sharpness(data.reshape(gmap.info.height,
                                                    gmap.info.width))
        elapsed      = time.time() - self.start_time

        accepted = sum(1 for r in self.icp_log if r.get("accepted"))

        record = {
            "timestamp":               datetime.now().isoformat(),
            "elapsed_sec":             round(elapsed, 1),
            "method":                  self.method_label,
            "map_width":               gmap.info.width,
            "map_height":              gmap.info.height,
            "total_cells":             total,
            "unknown_pct":             round(unknown_pct, 2),
            "occupied_pct":            round(occupied_pct, 2),
            "free_pct":                round(free_pct, 2),
            "wall_sharpness_mean_run": round(sharpness, 3),
            "points_per_robot":        clouds,
            "icp_attempts":            len(self.icp_log),
            "icp_accepted":            accepted,
            "recent_icp":              recent_icp[-3:],
        }

        with open(self.metrics_file, "a") as f:
            f.write(json.dumps(record) + "\n")

        self.get_logger().info(
            f"[Metrics] t={elapsed:.0f}s | "
            f"unknown={unknown_pct:.1f}% occupied={occupied_pct:.1f}% | "
            f"sharpness={sharpness:.2f} | "
            f"icp={accepted}/{len(self.icp_log)} accepted | "
            f"pts={clouds}")


# ── entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    server = CentralServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.log_metrics()
        icp_final = os.path.join(server.metrics_dir, "icp_alignments_final.json")
        with open(icp_final, "w") as f:
            json.dump(server.icp_log, f, indent=2)
        server.get_logger().info(
            f"[ICP] {len(server.icp_log)} attempts, "
            f"{sum(1 for r in server.icp_log if r.get('accepted'))} accepted "
            f"-> {icp_final}")
        server.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()