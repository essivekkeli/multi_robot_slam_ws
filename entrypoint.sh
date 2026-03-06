#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash 2>/dev/null || true
exec "$@"
