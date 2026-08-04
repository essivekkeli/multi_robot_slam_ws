#!/usr/bin/env python3
"""
Continuously logs robot3's global costmap (costmap_raw) metadata and the cost
value at two fixed candidate points, timestamped by sim time.

Run this CONCURRENTLY with coordination_node.py during a live MAPPO/IPPO/Random
run, then cross-reference this log against coordination_node.py's fast-abort
log lines by sim-time (NOT wall-clock time — Gazebo runs at ~3% real time on
this system, per your own confirmed finding this session).

Topic: /robot3/global_costmap/costmap_raw
Type:  nav2_msgs/msg/Costmap  (data field is uint8[], 0-254 valid cost range,
       255 = unknown per Nav2 docs: docs.nav2.org "Mapping and Localization")
QoS:   RELIABLE + TRANSIENT_LOCAL, confirmed live via
       `ros2 topic info /robot3/global_costmap/costmap_raw -v`. Subscriber
       QoS below is set explicitly to match; ROS2/DDS will silently deliver
       nothing on a QoS mismatch with no error (Maruyama, Kato, Azumi,
       "Exploring the Performance of ROS2", ACM EMSOFT 2016).

World -> robot3/map static offset used below: (0, 5, 0) — confirmed live via
tf2_echo in a prior session. If that offset is ever re-verified differently,
update WORLD_TO_ROBOT3_OFFSET accordingly; do not assume it's still correct
indefinitely, since costmap ORIGIN/SIZE have already been shown this session
to change between queries.
"""
import sys
import csv
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav2_msgs.msg import Costmap

WORLD_TO_ROBOT3_OFFSET = (0.0, 5.0)  # (dx, dy) world -> robot3/map

# world-frame candidate points — update/add as new abort goals are logged
CANDIDATE_POINTS = {
    "cylinder_side": (-4.23, -9.26),
    "wall_side":      (-7.1,  -9.2),
}

OUT_PATH = "/ros2_ws/robot3_costmap_log.csv"


class CostmapLogger(Node):
    def __init__(self):
        super().__init__("robot3_costmap_logger")
        self.declare_parameter("use_sim_time", True)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub = self.create_subscription(
            Costmap, "/robot3/global_costmap/costmap_raw", self.cb, qos
        )

        self.csv_file = open(OUT_PATH, "w", newline="")
        self.writer = csv.writer(self.csv_file)
        header = ["sim_stamp_sec", "sim_stamp_nsec", "origin_x", "origin_y",
                  "resolution", "size_x", "size_y"]
        for name in CANDIDATE_POINTS:
            header += [f"{name}_col", f"{name}_row", f"{name}_in_bounds", f"{name}_cost"]
        self.writer.writerow(header)
        self.csv_file.flush()

        self.frame_count = 0
        self.get_logger().info(f"Logging to {OUT_PATH} — waiting for first costmap_raw message...")

    def cb(self, msg: Costmap):
        m = msg.metadata
        row = [msg.header.stamp.sec, msg.header.stamp.nanosec,
               m.origin.position.x, m.origin.position.y,
               m.resolution, m.size_x, m.size_y]

        for name, (wx, wy) in CANDIDATE_POINTS.items():
            mx = wx - WORLD_TO_ROBOT3_OFFSET[0]
            my = wy - WORLD_TO_ROBOT3_OFFSET[1]
            col = int((mx - m.origin.position.x) / m.resolution)
            r = int((my - m.origin.position.y) / m.resolution)
            in_bounds = 0 <= col < m.size_x and 0 <= r < m.size_y
            cost = msg.data[r * m.size_x + col] if in_bounds else -1
            row += [col, r, in_bounds, cost]

        self.writer.writerow(row)
        self.csv_file.flush()
        self.frame_count += 1
        if self.frame_count % 20 == 0:
            self.get_logger().info(f"{self.frame_count} frames logged "
                                    f"(sim_t={msg.header.stamp.sec}.{msg.header.stamp.nanosec})")

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = CostmapLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()