#!/bin/bash
N=${1:-30}
SAMPLE_DIR="/ros2_ws/results/gnn/samples"
SCRIPTS="/ros2_ws/src/multi_robot_slam/scripts"
GLIM_CFG="/ros2_ws/install/multi_robot_slam/share/multi_robot_slam/config/glim"

echo "========================================================"
echo "Auto collect v2: $N runs  Started: $(date)"
echo "========================================================"

success=0; failed=0

check_glim_ready() {
    # Check if all 3 robots are publishing aligned_points
    # Use timeout 3 to avoid blocking
    r1=$(timeout 3 ros2 topic echo /robot1/glim/aligned_points \
         --once 2>/dev/null | grep -c "header" || echo 0)
    r2=$(timeout 3 ros2 topic echo /robot2/glim/aligned_points \
         --once 2>/dev/null | grep -c "header" || echo 0)
    r3=$(timeout 3 ros2 topic echo /robot3/glim/aligned_points \
         --once 2>/dev/null | grep -c "header" || echo 0)
    echo "$r1 $r2 $r3"
}

for i in $(seq 1 $N); do
    echo ""
    echo "=== Run $i/$N  $(date) ==="
    before=$(ls $SAMPLE_DIR 2>/dev/null | wc -l)

    # Step 1: Kill and restart GLIM only
    echo "  Restarting GLIM..."
    pkill -f glim_rosnode 2>/dev/null
    sleep 3
    rm -rf /tmp/dump
    sleep 2

    ros2 run glim_ros glim_rosnode --ros-args \
      -p config_path:=$GLIM_CFG/robot1 -p use_sim_time:=true \
      --remap /glim_ros/map:=/robot1/glim/map \
      --remap /glim_ros/odom:=/robot1/glim/odom \
      --remap /glim_ros/aligned_points:=/robot1/glim/aligned_points > /dev/null 2>&1 &

    ros2 run glim_ros glim_rosnode --ros-args \
      -p config_path:=$GLIM_CFG/robot2 -p use_sim_time:=true \
      --remap /glim_ros/map:=/robot2/glim/map \
      --remap /glim_ros/odom:=/robot2/glim/odom \
      --remap /glim_ros/aligned_points:=/robot2/glim/aligned_points > /dev/null 2>&1 &

    ros2 run glim_ros glim_rosnode --ros-args \
      -p config_path:=$GLIM_CFG/robot3 -p use_sim_time:=true \
      --remap /glim_ros/map:=/robot3/glim/map \
      --remap /glim_ros/odom:=/robot3/glim/odom \
      --remap /glim_ros/aligned_points:=/robot3/glim/aligned_points > /dev/null 2>&1 &

    # Step 2: Wait for all 3 GLIM nodes to publish data
    echo "  Waiting for GLIM data on all 3 robots..."
    max_wait=180
    elapsed=0
    ready=0

    while [ $elapsed -lt $max_wait ]; do
        sleep 10
        elapsed=$((elapsed + 10))
        read r1 r2 r3 <<< $(check_glim_ready)
        echo "  t=${elapsed}s  publishing: r1=$r1 r2=$r2 r3=$r3"
        if [ "$r1" -gt "0" ] && [ "$r2" -gt "0" ] && [ "$r3" -gt "0" ]; then
            sleep 30
            echo "  All 3 GLIM nodes publishing after ${elapsed}s"
            ready=1
            break
        fi
    done

    if [ $ready -eq 0 ]; then
        echo "  GLIM not ready in ${max_wait}s — skipping"
        failed=$((failed + 1))
        continue
    fi

    # Step 3: Drive
    echo "  Driving..."
    timeout 900 python3 $SCRIPTS/drive_robots.py
    drive_exit=$?
    [ $drive_exit -eq 124 ] && echo "  WARNING: drive timed out"

    # Step 4: Save
    echo "  Saving sample..."
    sleep 5
    ros2 topic pub --once /save_sample std_msgs/msg/String \
      "{data: 'auto_v2_$(printf '%03d' $i)'}" 2>/dev/null
    sleep 8

    # Step 5: Verify
    after=$(ls $SAMPLE_DIR 2>/dev/null | wc -l)
    new=$((after - before))
    if [ $new -gt 0 ]; then
        latest=$(ls -t $SAMPLE_DIR | head -1)
        min_occ=$(python3 -c "
import json
meta = json.load(open('$SAMPLE_DIR/$latest/meta.json'))
occ = meta.get('occupied_cells', {})
print(min(occ.values()) if occ else 0)
" 2>/dev/null || echo 0)
        echo "  SAVED $latest min_occ=$min_occ total=$after"
        success=$((success + 1))
    else
        echo "  FAILED — no sample saved"
        failed=$((failed + 1))
    fi

    echo "  Progress: $success ok $failed failed $((N-i)) remaining"
done

echo ""
echo "========================================================"
echo "Done: $success/$N  Total: $(ls $SAMPLE_DIR 2>/dev/null | wc -l) samples"
echo "Finished: $(date)"
echo "========================================================"
