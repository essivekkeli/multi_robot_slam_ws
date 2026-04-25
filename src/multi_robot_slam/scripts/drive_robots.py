#!/usr/bin/env python3
import rclpy
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
STUCK_TIMEOUT  = 30.0


class MultiRobotDriver(Node):
    def __init__(self):
        super().__init__("multi_robot_driver")
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
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

    def get_world_pose(self, robot_name):
        try:
            t = self.tf_buffer.lookup_transform(
                'world', f'{robot_name}/base_footprint',
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
                    r["spin_end"] = time.time() + 5.0
                else:
                    self.get_logger().info(
                        f"[{name}] waiting for TF...", throttle_duration_sec=3.0)
                continue

            if x is None:
                continue

            if r["phase"] == "spinning":
                if time.time() < r["spin_end"]:
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
                        r["wp_start"] = time.time()
                        wx, wy, lbl, _ = r["waypoints"][r["wp_index"]]
                        self.get_logger().info(
                            f"[{name}] -> '{lbl}' world({wx:.1f},{wy:.1f})")
                continue

            if r["phase"] == "driving":
                wx, wy, lbl, spin_t = r["waypoints"][r["wp_index"]]
                dx   = wx - x
                dy   = wy - y
                dist = math.sqrt(dx*dx + dy*dy)

                if dist < GOAL_TOLERANCE:
                    self.stop(name)
                    self.get_logger().info(f"[{name}] reached '{lbl}'")
                    r["wp_index"] += 1
                    r["phase"]    = "spinning"
                    r["spin_end"] = time.time() + spin_t
                    continue

                if time.time() - r["wp_start"] > STUCK_TIMEOUT:
                    self.stop(name)
                    self.get_logger().warn(f"[{name}] timeout '{lbl}' — skipping")
                    r["wp_index"] += 1
                    r["phase"]    = "spinning"
                    r["spin_end"] = time.time() + 1.0
                    continue

                goal_yaw = math.atan2(dy, dx)
                yaw_err  = goal_yaw - yaw
                while yaw_err >  math.pi: yaw_err -= 2*math.pi
                while yaw_err < -math.pi: yaw_err += 2*math.pi

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
    time.sleep(3)
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
