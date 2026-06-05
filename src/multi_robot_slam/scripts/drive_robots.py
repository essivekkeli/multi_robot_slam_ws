#!/usr/bin/env python3
"""
drive_robots.py  —  Waypoint-following driver for 3 robots
===========================================================
Drives robot1, robot2, robot3 through pre-defined routes using
world-frame TF pose feedback.

Changes from v1:
  FIX1: time.time() replaced with ROS sim clock throughout.
        Gazebo + 3x GLIM runs at 0.5-0.8x real-time under load.
        Wall-clock timeouts would cause robots to skip reachable
        waypoints prematurely and spin phases to be cut short.
        self.clock = self.get_clock() stored in __init__; all
        time comparisons use self._now() -> float seconds.
"""

import rclpy
import rclpy.parameter
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
import math
import time

ROUTES = {
    "robot1": [
        (-7.0, -4.0, "sw_nw_corner",    4.0),
        (-5.0, -7.0, "sw_south",        3.0),
        (-5.0, -4.0, "sw_centre",       3.0),
        (-1.0, -5.0, "east_of_niche",   2.0),
        ( 0.0, -4.5, "gap_approach",    2.0),
        ( 0.0, -2.0, "in_corridor",     3.0),
        (-1.0,  0.5, "junction_west",   5.0),
        ( 1.0,  0.5, "junction_east",   5.0),
        ( 0.0, -1.0, "junction_south",  5.0),
        ( 0.0, -4.5, "gap_return",      1.0),
        (-5.0, -5.0, "home",            2.0),
    ],
    "robot2": [
        ( 7.0, -4.0, "se_ne_corner",    4.0),
        ( 5.0, -7.0, "se_south",        3.0),
        ( 5.0, -4.0, "se_centre",       3.0),
        ( 1.0, -5.0, "west_approach",   2.0),
        ( 0.0, -4.5, "gap_approach",    2.0),
        ( 0.0, -2.0, "in_corridor",     3.0),
        ( 1.0,  0.5, "junction_east",   5.0),
        (-1.0,  0.5, "junction_west",   5.0),
        ( 0.0, -1.0, "junction_south",  5.0),
        ( 0.0, -4.5, "gap_return",      1.0),
        ( 5.0, -5.0, "home",            2.0),
    ],
    "robot3": [
        (-2.0,  7.0, "nc_west",         4.0),
        ( 2.0,  7.0, "nc_east",         4.0),
        ( 0.0,  5.5, "nc_centre",       3.0),
        ( 0.0,  4.0, "gap_approach",    2.0),
        ( 0.0,  2.0, "in_corridor",     3.0),
        ( 0.0,  1.0, "junction_north",  5.0),
        ( 1.0, -0.5, "junction_east",   5.0),
        (-1.0, -0.5, "junction_west",   5.0),
        ( 0.0,  4.0, "gap_return",      1.0),
        ( 0.0,  5.0, "home",            2.0),
    ],
}

LINEAR_SPEED   = 0.4
ANGULAR_SPEED  = 0.7
GOAL_TOLERANCE = 0.4
STUCK_TIMEOUT  = 30.0   # sim seconds (FIX1: was wall seconds)
MISSION_TIMEOUT = 180.0   # 3 minutes (sim time)


class MultiRobotDriver(Node):
    def __init__(self):
        super().__init__(
            "multi_robot_driver",
            parameter_overrides=[
                rclpy.parameter.Parameter(
                    "use_sim_time",
                    rclpy.Parameter.Type.BOOL,
                    True)
            ])

        # FIX1: store ROS clock for sim-time-aware timing
        self.clock = self.get_clock()

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.mission_start = self._now()
        self.robots = {}
        for name in ROUTES:
            self.robots[name] = {
                "waypoints": ROUTES[name],
                "wp_index":  0,
                "done":      False,
                "phase":     "wait_tf",
                "spin_end":  0.0,
                "wp_start":  0.0,
                "cmd_pub":   self.create_publisher(Twist, f"/{name}/cmd_vel", 10),
            }
            self.get_logger().info(f"[{name}] registered")
        self.create_timer(0.1, self.tick)



    def _now(self) -> float:
        """Current time in seconds using ROS sim clock (respects use_sim_time)."""
        return self.clock.now().nanoseconds / 1e9

    def get_world_pose(self, robot_name):
        try:
            t = self.tf_buffer.lookup_transform(
                "world", f"{robot_name}/base_footprint",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
            x   = t.transform.translation.x
            y   = t.transform.translation.y
            q   = t.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return x, y, yaw
        except Exception:
            return None, None, None

    def stop(self, name):
        self.robots[name]["cmd_pub"].publish(Twist())

    def tick(self):

        if self._now() - self.mission_start >= MISSION_TIMEOUT:#
            self.get_logger().info(#
                f"Mission timeout ({MISSION_TIMEOUT}s) reached")#

            for name in self.robots:#
                self.stop(name)#

            raise SystemExit

        all_done = True
        for name, r in self.robots.items():
            if r["done"]:
                continue
            all_done = False
            x, y, yaw = self.get_world_pose(name)

            if r["phase"] == "wait_tf":
                if x is not None:
                    self.get_logger().info(
                        f"[{name}] TF ready at world({x:.2f},{y:.2f}) — spinning")
                    r["phase"]    = "spinning"
                    r["spin_end"] = self._now() + 5.0  # FIX1
                else:
                    self.get_logger().info(
                        f"[{name}] waiting for TF...",
                        throttle_duration_sec=3.0)
                continue

            if x is None:
                continue

            if r["phase"] == "spinning":
                if self._now() < r["spin_end"]:  # FIX1
                    msg = Twist()
                    msg.angular.z = ANGULAR_SPEED
                    r["cmd_pub"].publish(msg)
                else:
                    self.stop(name)
                    if r["wp_index"] >= len(r["waypoints"]):
                        r["done"] = True
                        self.get_logger().info(f"[{name}] route complete")
                    else:
                        r["phase"]    = "driving"
                        r["wp_start"] = self._now()  # FIX1
                        wx, wy, lbl, _ = r["waypoints"][r["wp_index"]]
                        self.get_logger().info(
                            f"[{name}] -> '{lbl}' world({wx:.1f},{wy:.1f})")
                continue

            if r["phase"] == "driving":
                wx, wy, lbl, spin_t = r["waypoints"][r["wp_index"]]
                dx   = wx - x
                dy   = wy - y
                dist = math.sqrt(dx * dx + dy * dy)

                if dist < GOAL_TOLERANCE:
                    self.stop(name)
                    self.get_logger().info(f"[{name}] reached '{lbl}'")
                    r["wp_index"] += 1
                    r["phase"]    = "spinning"
                    r["spin_end"] = self._now() + spin_t  # FIX1
                    continue

                if self._now() - r["wp_start"] > STUCK_TIMEOUT:  # FIX1
                    self.stop(name)
                    self.get_logger().warn(
                        f"[{name}] timeout '{lbl}' — skipping")
                    r["wp_index"] += 1
                    r["phase"]    = "spinning"
                    r["spin_end"] = self._now() + 1.0  # FIX1
                    continue

                goal_yaw = math.atan2(dy, dx)
                yaw_err  = goal_yaw - yaw
                while yaw_err >  math.pi: yaw_err -= 2 * math.pi
                while yaw_err < -math.pi: yaw_err += 2 * math.pi

                cmd = Twist()
                if abs(yaw_err) > 0.35:
                    cmd.angular.z = ANGULAR_SPEED * (1.0 if yaw_err > 0 else -1.0)
                    cmd.linear.x  = 0.08
                else:
                    cmd.linear.x  = min(LINEAR_SPEED, dist * 0.6)
                    cmd.angular.z = 1.2 * yaw_err
                r["cmd_pub"].publish(cmd)

        if all_done:
            self.get_logger().info("All robots finished!")
            raise SystemExit


def main():
    print("=" * 60)
    print("MULTI-ROBOT DRIVE — world frame TF positioning")
    print("Starting in 3 seconds...")
    print("=" * 60)
    time.sleep(3)   # wall-clock delay before rclpy.init() is fine
    rclpy.init()
    node = MultiRobotDriver()
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        for name in ROUTES:
            try:
                node.robots[name]["cmd_pub"].publish(Twist())
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()
    print("Drive complete!")


if __name__ == "__main__":
    main()