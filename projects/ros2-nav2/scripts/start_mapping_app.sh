#!/usr/bin/env bash
set -eo pipefail

ROOT=/home/wheeltec/evi/evi_dashboard
USART_PORT="${EVI_BASE_USART:-/dev/ttyACM0}"

source /opt/tros/humble/setup.bash
source /home/wheeltec/wheeltec_ros2/install/setup.bash
export ROS_DOMAIN_ID="${EVI_ROS_DOMAIN_ID:-59}"
export ROS_LOCALHOST_ONLY="${EVI_ROS_LOCALHOST_ONLY:-1}"

pkill -INT -f '[m]ap_server' || true
pkill -INT -f '[w]heeltec_lidar.launch.py' || true
pkill -INT -f '[l]slidar_driver_node' || true
sleep 1
if ! pgrep -f '[d]ds_warmup.py|[r]os_bridge_process.py' >/dev/null 2>&1; then
  rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
fi

exec ros2 launch "$ROOT/board_agent/evi_mapping_bringup.launch.py" \
  usart_port:="$USART_PORT"

