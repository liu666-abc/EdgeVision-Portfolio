#!/usr/bin/env bash
set -eo pipefail

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
unset PYTHONPATH LD_LIBRARY_PATH
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION TROS_DISTRO
unset ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION

source /opt/tros/humble/setup.bash
source /home/wheeltec/wheeltec_ros2/install/setup.bash
export ROS_DOMAIN_ID="${EVI_ROS_DOMAIN_ID:-59}"
export ROS_LOCALHOST_ONLY="${EVI_ROS_LOCALHOST_ONLY:-1}"

ROOT=/home/wheeltec/evi/evi_dashboard
CONFIG_JSON="${1:?missing bridge config JSON}"
SOCKET_PATH="${2:?missing bridge socket path}"

cd "$ROOT/nav_console"
exec python3 "$ROOT/nav_console/tools/bridge_socket_proxy.py" \
  --bridge "$ROOT/nav_console/ros_bridge_process.py" \
  --socket "$SOCKET_PATH" \
  "$CONFIG_JSON"
