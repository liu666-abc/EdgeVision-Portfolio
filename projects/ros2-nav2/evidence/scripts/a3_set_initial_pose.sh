#!/usr/bin/env bash
# 给 AMCL 一个初始位姿（无 GUI 也能用）。用法：a3_set_initial_pose.sh [x] [y] [yaw_deg]
set +u

source /opt/ros/humble/setup.bash
source /home/wheeltec/wheeltec_ros2/install/setup.bash

X=${1:-0.0}
Y=${2:-0.0}
YAW=${3:-0.0}
QZ=$(python3 -c 'import math,sys;print(math.sin(math.radians(float(sys.argv[1]))/2.0))' "$YAW")
QW=$(python3 -c 'import math,sys;print(math.cos(math.radians(float(sys.argv[1]))/2.0))' "$YAW")

ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: map}, pose: {pose: {position: {x: $X, y: $Y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: $QZ, w: $QW}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.07]}}"
echo "initial pose sent: x=$X y=$Y yaw=${YAW}deg"
