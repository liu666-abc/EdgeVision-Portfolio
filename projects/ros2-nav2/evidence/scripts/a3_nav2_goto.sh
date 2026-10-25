#!/usr/bin/env bash
# 发一个 NavigateToPose 目标并跟随反馈。用法：a3_nav2_goto.sh [x] [y] [yaw_deg]
set +u

source /opt/ros/humble/setup.bash
source /home/wheeltec/wheeltec_ros2/install/setup.bash

X=${1:-1.0}
Y=${2:-0.0}
YAW=${3:-0.0}
QZ=$(python3 -c 'import math,sys;print(math.sin(math.radians(float(sys.argv[1]))/2.0))' "$YAW")
QW=$(python3 -c 'import math,sys;print(math.cos(math.radians(float(sys.argv[1]))/2.0))' "$YAW")

echo "goal: x=$X y=$Y yaw=${YAW}deg (frame=map)"
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: $X, y: $Y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: $QZ, w: $QW}}}}" --feedback
