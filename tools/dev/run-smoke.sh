#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash
# 1) launch RealSense (background)
ros2 launch realsense2_camera rs_launch.py align_depth:=true >/tmp/rs.log 2>&1 &
sleep 2
# 2) open rqt (image view)
rqt
