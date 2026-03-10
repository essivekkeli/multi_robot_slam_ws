#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash 2>/dev/null || true
export FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/fastdds_no_shm.xml
exec "$@"
