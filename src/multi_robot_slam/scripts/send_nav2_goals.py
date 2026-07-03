#!/usr/bin/env python3
"""
send_nav2_goals.py — Step 2, Stage 4 Gazebo deployment.
Sends NavigateToPose goals to all three robots concurrently via raw
rclpy action clients (one node + executor per robot thread).
Logs timing to CSV. Does NOT touch lifecycle managers.
"""
import argparse
import csv
import math
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


DEFAULT_GOALS = {
    "robot1": (0.0, 1.0, 0.0),
    "robot2": (0.0, 1.0, 0.0),
    "robot3": (0.0, 0.5, 0.0),
}


def make_pose(x, y, yaw, frame_id):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def run_robot_goal(robot_name, x, y, yaw, results, results_lock):
    record = {
        "robot": robot_name,
        "goal_x": x, "goal_y": y,
        "time_to_accepted_s": None,
        "time_to_complete_s": None,
        "result": None,
    }

    context = rclpy.Context()
    rclpy.init(context=context)
    node = Node(f"{robot_name}_goal_sender",
                namespace=f"/{robot_name}",
                context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)

    try:
        client = ActionClient(node, NavigateToPose,
                              f"/{robot_name}/navigate_to_pose")

        node.get_logger().info(f"Waiting for action server...")
        if not client.wait_for_server(timeout_sec=10.0):
            record["result"] = "TIMEOUT_NO_SERVER"
            return

        goal = NavigateToPose.Goal()
        goal.pose = make_pose(x, y, yaw, f"{robot_name}/map")
        goal.pose.header.stamp = node.get_clock().now().to_msg()

        t_send = time.monotonic()
        send_future = client.send_goal_async(goal)

        # Spin until goal accepted
        while not send_future.done():
            executor.spin_once(timeout_sec=0.1)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            record["result"] = "REJECTED"
            return

        record["time_to_accepted_s"] = time.monotonic() - t_send
        node.get_logger().info(f"Goal accepted, navigating...")

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            executor.spin_once(timeout_sec=0.1)

        record["time_to_complete_s"] = time.monotonic() - t_send
        status = result_future.result().status
        # status 4 = SUCCEEDED, 5 = CANCELED, 6 = ABORTED
        status_map = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}
        record["result"] = status_map.get(status, f"STATUS_{status}")

        node.get_logger().info(
            f"Done: {record['result']} "
            f"accepted={record['time_to_accepted_s']:.2f}s "
            f"total={record['time_to_complete_s']:.2f}s"
        )

    except Exception as e:
        record["result"] = f"EXCEPTION: {e}"
        node.get_logger().error(str(e))
    finally:
        with results_lock:
            results.append(record)
        node.destroy_node()
        rclpy.shutdown(context=context)


def parse_goal_arg(s):
    parts = s.split(":")
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(f"must be robot_name:x:y")
    name, x, y = parts
    return name, float(x), float(y)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--goals", nargs="*", type=parse_goal_arg)
    parser.add_argument("--out", default="nav2_step2_results.csv")
    args = parser.parse_args()

    goal_map = {name: (x, y, 0.0) for name, x, y in args.goals} \
        if args.goals else DEFAULT_GOALS

    print(f"Sending goals: {goal_map}")

    results = []
    results_lock = threading.Lock()
    threads = [
        threading.Thread(
            target=run_robot_goal,
            args=(robot, x, y, yaw, results, results_lock),
            daemon=True,
        )
        for robot, (x, y, yaw) in goal_map.items()
    ]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    if results:
        with open(args.out, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(results[0].keys()))
            writer.writeheader()
            writer.writerows(results)
        print(f"Results written to {args.out}")

    print("\n--- Summary ---")
    for r in results:
        print(f"{r['robot']}: {r['result']} "
              f"accepted={r['time_to_accepted_s']}s "
              f"total={r['time_to_complete_s']}s")


if __name__ == "__main__":
    main()
