#collect_training_data.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
import numpy as np
import json, os, glob, time
from datetime import datetime

SAMPLE_DIR    = "/ros2_ws/results/gnn/samples"
ROBOTS        = ["robot1", "robot2", "robot3"]
SAVE_COOLDOWN = 30.0   # minimum seconds between saves — prevents duplicates

def next_sample_id(base_dir):
    existing = sorted(glob.glob(os.path.join(base_dir, "sample_*")))
    if not existing: return "0000"
    last = os.path.basename(existing[-1]).split("_")[1]
    return f"{int(last)+1:04d}"

def load_latest_icp_fitness():
    matches = sorted(glob.glob("/tmp/fusion_metrics/icp_*/icp_alignments.jsonl"))
    if not matches: return {}
    fitness = {}
    try:
        with open(matches[-1]) as f:
            for line in f:
                line = line.strip()
                if not line: continue
                rec = json.loads(line)
                if rec.get("accepted"):
                    fitness[rec.get("pair","")] = rec.get("fitness", 0.0)
    except Exception:
        pass
    return fitness

class TrainingDataCollector(Node):
    def __init__(self):
        super().__init__("training_data_collector")
        os.makedirs(SAMPLE_DIR, exist_ok=True)
        self.latest_maps    = {r: None for r in ROBOTS}
        self.latest_meta    = {r: None for r in ROBOTS}
        self.saved_count    = 0
        self.last_save_time = 0.0

        qos = QoSProfile(depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        for robot in ROBOTS:
            self.create_subscription(OccupancyGrid, f"/{robot}/map",
                lambda msg, r=robot: self._map_cb(msg, r), qos)
            self.get_logger().info(f"Subscribed to /{robot}/map")

        self.create_subscription(String, "/save_sample",
            self._save_trigger_cb, 10)
        self.create_timer(30.0, self._status_log)
        self.create_timer(60.0, self._auto_save)
        self.get_logger().info(
            f"Collector ready. SAVE_COOLDOWN={SAVE_COOLDOWN}s")

    def _map_cb(self, msg, robot):
        data = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)
        self.latest_maps[robot] = data
        self.latest_meta[robot] = {
            "resolution": msg.info.resolution,
            "origin_x":   msg.info.origin.position.x,
            "origin_y":   msg.info.origin.position.y,
            "width":      msg.info.width,
            "height":     msg.info.height,
        }

    def _save_trigger_cb(self, msg):
        now = time.time()
        since_last = now - self.last_save_time
        if since_last < SAVE_COOLDOWN:
            self.get_logger().warn(
                f"Save trigger ignored — cooldown {since_last:.0f}s "
                f"< {SAVE_COOLDOWN}s")
            return
        self.get_logger().info(f"Save trigger received: '{msg.data}'")
        self.save_sample(label=msg.data)

    def _status_log(self):
        ready = [r for r in ROBOTS if self.latest_maps[r] is not None]
        occ   = {r: int(np.sum(self.latest_maps[r]==100))
                 for r in ready}
        self.get_logger().info(
            f"Maps: {ready}  occ={occ}  saved={self.saved_count}",
            throttle_duration_sec=30.0)

    def _auto_save(self):
        self.save_sample(label="auto")

    def save_sample(self, label="manual"):
        if not all(self.latest_maps[r] is not None for r in ROBOTS):
            self.get_logger().warn("Not all maps available — skipping")
            return None
        sample_id  = next_sample_id(SAMPLE_DIR)
        sample_dir = os.path.join(SAMPLE_DIR, f"sample_{sample_id}")
        os.makedirs(sample_dir, exist_ok=True)
        for robot in ROBOTS:
            np.save(os.path.join(sample_dir, f"{robot}_map.npy"),
                    self.latest_maps[robot])
        meta = {
            "sample_id":      sample_id,
            "timestamp":      datetime.now().isoformat(),
            "label":          label,
            "resolution":     0.05,
            "icp_fitness":    load_latest_icp_fitness(),
            "map_info":       {r: self.latest_meta[r] for r in ROBOTS
                               if self.latest_meta[r] is not None},
            "occupied_cells": {r: int(np.sum(self.latest_maps[r]==100))
                               for r in ROBOTS},
            "free_cells":     {r: int(np.sum(self.latest_maps[r]==0))
                               for r in ROBOTS},
        }
        json.dump(meta, open(os.path.join(sample_dir,"meta.json"),"w"),
                  indent=2)
        self.saved_count    += 1
        self.last_save_time  = time.time()
        self.get_logger().info(
            f"Saved sample_{sample_id}  total={self.saved_count}  "
            f"occ={[meta['occupied_cells'][r] for r in ROBOTS]}")
        return sample_dir

def main(args=None):
    rclpy.init(args=args)
    node = TrainingDataCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown — saving final sample")
        node.save_sample(label="shutdown")
    finally:
        node.destroy_node()
        try: rclpy.shutdown()
        except: pass

if __name__ == "__main__":
    main()
