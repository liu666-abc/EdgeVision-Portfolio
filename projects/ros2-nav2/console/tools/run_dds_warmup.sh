#!/usr/bin/env bash
set -eo pipefail

source /opt/tros/humble/setup.bash
source /home/wheeltec/wheeltec_ros2/install/setup.bash
export ROS_DOMAIN_ID="${EVI_ROS_DOMAIN_ID:-59}"
export ROS_LOCALHOST_ONLY="${EVI_ROS_LOCALHOST_ONLY:-1}"

exec python3 /home/wheeltec/evi/evi_dashboard/nav_console/tools/dds_warmup.py
