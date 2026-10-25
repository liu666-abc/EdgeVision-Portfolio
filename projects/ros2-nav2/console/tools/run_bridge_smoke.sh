#!/usr/bin/env bash
set -eo pipefail

ROOT=/home/wheeltec/evi/evi_dashboard
SECONDS_TO_RUN="${1:-12}"
shift || true

source /opt/tros/humble/setup.bash
source /home/wheeltec/wheeltec_ros2/install/setup.bash
export ROS_DOMAIN_ID="${EVI_ROS_DOMAIN_ID:-59}"
export ROS_LOCALHOST_ONLY="${EVI_ROS_LOCALHOST_ONLY:-1}"

cd "${EVI_BRIDGE_CWD:-$ROOT/nav_console}"
exec python3 "$ROOT/nav_console/tools/bridge_smoke_test.py" \
  --seconds "$SECONDS_TO_RUN" "$@"
