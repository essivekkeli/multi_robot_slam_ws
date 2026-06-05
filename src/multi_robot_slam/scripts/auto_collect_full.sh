#!/bin/bash
# auto_collect_full.sh — full restart between every run
# Usage: bash auto_collect_full.sh 30

N=${1:-50}
SAMPLE_DIR="/ros2_ws/results/gnn/samples"
SCRIPTS="/ros2_ws/src/multi_robot_slam/scripts"
GLIM_CFG="/ros2_ws/install/multi_robot_slam/share/multi_robot_slam/config/glim"
LOG_DIR="/ros2_ws/results/gnn/collection_logs"

export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
source /ros2_ws/install/setup.bash

mkdir -p $LOG_DIR $SAMPLE_DIR

echo "========================================================"
echo "Full-restart auto collection: $N runs"
echo "Sample dir: $SAMPLE_DIR"
echo "Started: $(date)"
echo "Estimated time: $((N * 12)) minutes = $((N * 12 / 60)) hours"
echo "========================================================"

success=0
failed=0

for i in $(seq 1 $N); do
    echo ""
    echo "━━━ Run $i / $N  ($(date))  success=$success failed=$failed ━━━"
    LOG="$LOG_DIR/run_$(printf '%03d' $i).log"
    before=$(ls $SAMPLE_DIR 2>/dev/null | wc -l)

    # ── Kill everything from previous run ─────────────────────────────
    echo "  [1/6] Killing previous processes..."
    pkill -f "glim_rosnode"   2>/dev/null; sleep 1
    pkill -f "gz sim"         2>/dev/null; sleep 1
    pkill -f "gz_multi_robot" 2>/dev/null; sleep 1
    pkill -f "gzserver"       2>/dev/null; sleep 1
    pkill -f "central_server" 2>/dev/null; sleep 1
    pkill -f "imu_relay"      2>/dev/null; sleep 1
    pkill -f "drive_robots"   2>/dev/null; sleep 1
    sleep 3

    # ── Clean GLIM state ──────────────────────────────────────────────
    echo "  [2/6] Cleaning GLIM state..."
    rm -rf /tmp/dump /tmp/fusion_metrics
    sleep 1

    # ── Start Gazebo ──────────────────────────────────────────────────
    echo "  [3/6] Starting Gazebo..."
    ros2 launch multi_robot_slam gz_multi_robot.launch.py >> $LOG 2>&1 &
    sleep 40

    # ── Start IMU relays ──────────────────────────────────────────────
    echo "  [4/6] Starting IMU relays..."
    ros2 run multi_robot_slam imu_relay.py --ros-args -p robot_name:=robot1 >> $LOG 2>&1 &
    ros2 run multi_robot_slam imu_relay.py --ros-args -p robot_name:=robot2 >> $LOG 2>&1 &
    ros2 run multi_robot_slam imu_relay.py --ros-args -p robot_name:=robot3 >> $LOG 2>&1 &
    sleep 3

    # ── Start GLIM x3 ─────────────────────────────────────────────────
    echo "  [5/6] Starting GLIM..."
    ros2 run glim_ros glim_rosnode --ros-args \
      -p config_path:=$GLIM_CFG/robot1 -p use_sim_time:=true \
      --remap /glim_ros/map:=/robot1/glim/map \
      --remap /glim_ros/odom:=/robot1/glim/odom \
      --remap /glim_ros/aligned_points:=/robot1/glim/aligned_points >> $LOG 2>&1 &

    ros2 run glim_ros glim_rosnode --ros-args \
      -p config_path:=$GLIM_CFG/robot2 -p use_sim_time:=true \
      --remap /glim_ros/map:=/robot2/glim/map \
      --remap /glim_ros/odom:=/robot2/glim/odom \
      --remap /glim_ros/aligned_points:=/robot2/glim/aligned_points >> $LOG 2>&1 &

    ros2 run glim_ros glim_rosnode --ros-args \
      -p config_path:=$GLIM_CFG/robot3 -p use_sim_time:=true \
      --remap /glim_ros/map:=/robot3/glim/map \
      --remap /glim_ros/odom:=/robot3/glim/odom \
      --remap /glim_ros/aligned_points:=/robot3/glim/aligned_points >> $LOG 2>&1 &

    sleep 60

    # ── Start central server (baseline_tf) ────────────────────────────
    python3 /ros2_ws/src/multi_robot_slam_py/multi_robot_slam_py/central_server_unified.py \
      --ros-args -p method_label:=baseline_tf >> $LOG 2>&1 &
    sleep 5

    # ── Drive robots ──────────────────────────────────────────────────
    echo "  [6/6] Driving robots..."
    timeout 900 python3 $SCRIPTS/drive_robots.py \
      --ros-args -p use_sim_time:=true >> $LOG 2>&1
    drive_exit=$?
    [ $drive_exit -eq 124 ] && echo "  WARNING: drive timed out — saving anyway"

    # ── Trigger sample save ───────────────────────────────────────────
    sleep 8
    ros2 topic pub --once /save_sample std_msgs/msg/String \
      "{data: 'auto_run_$(printf '%03d' $i)'}" >> $LOG 2>&1
    sleep 5

    # ── Check result ──────────────────────────────────────────────────
    after=$(ls $SAMPLE_DIR 2>/dev/null | wc -l)
    new=$((after - before))
    if [ $new -gt 0 ]; then
        echo "  ✓ Sample saved. Total: $after"
        success=$((success + 1))
    else
        echo "  ✗ No sample saved — check $LOG"
        failed=$((failed + 1))
    fi
    echo "  Progress: $success successful, $failed failed, $((N-i)) remaining"
done

echo ""
echo "========================================================"
echo "Collection complete"
echo "  Successful runs: $success / $N"
echo "  Failed runs:     $failed / $N"
echo "  Total samples:   $(ls $SAMPLE_DIR 2>/dev/null | wc -l)"
echo "  Finished: $(date)"
echo "========================================================"
