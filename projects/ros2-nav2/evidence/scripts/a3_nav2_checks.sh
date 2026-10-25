#!/usr/bin/env bash
# A4 起栈后只读自检（不发任何速度指令）
set +u

source /opt/ros/humble/setup.bash
source /home/wheeltec/wheeltec_ros2/install/setup.bash

echo "=== date / uptime ==="; date -Ins; cat /proc/uptime
echo "=== nodes ==="; ros2 node list
echo "=== /scan hz ==="; timeout 10 ros2 topic hz /scan --window 20
echo "=== /odom_combined hz ==="; timeout 10 ros2 topic hz /odom_combined --window 20
echo "=== /amcl_pose hz ==="; timeout 10 ros2 topic hz /amcl_pose --window 10
echo "=== /map info ==="; ros2 topic info /map
echo "=== /cmd_vel publishers ==="; ros2 topic info /cmd_vel
echo "=== tf map->base_footprint ==="; timeout 8 ros2 run tf2_ros tf2_echo map base_footprint
echo "=== /local_costmap/costmap ==="; ros2 topic info /local_costmap/costmap
echo "=== serial holders ==="; fuser -v /dev/wheeltec_controller
echo "=== done ==="
