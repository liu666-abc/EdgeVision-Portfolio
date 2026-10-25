#!/usr/bin/env bash
set -eo pipefail

ROOT=/home/wheeltec/evi/evi_dashboard
PORT="${EVI_NAV_CONSOLE_PORT:-8091}"
PID_FILE=/tmp/evi_nav_console.pid
LOG_FILE=/tmp/evi_nav_console.log
BRIDGE_SOCKET="${EVI_NAV_BRIDGE_SOCKET:-/tmp/evi_nav_bridge.sock}"
BRIDGE_PID_FILE=/tmp/evi_nav_bridge.pid
BRIDGE_LOG=/tmp/evi_nav_bridge.log
WARMUP_PID_FILE=/tmp/evi_nav_warmup.pid
WARMUP_LOG=/tmp/evi_nav_warmup.log
CONFIG="${EVI_NAV_CONSOLE_CONFIG:-$ROOT/nav_console/config.json}"
if [ ! -f "$CONFIG" ]; then
  CONFIG="$ROOT/nav_console/config.example.json"
fi

source /opt/tros/humble/setup.bash
if [ -f /home/wheeltec/wheeltec_ros2/install/setup.bash ]; then
  source /home/wheeltec/wheeltec_ros2/install/setup.bash
fi

export ROS_DOMAIN_ID="${EVI_ROS_DOMAIN_ID:-59}"
export ROS_LOCALHOST_ONLY="${EVI_ROS_LOCALHOST_ONLY:-1}"
export EVI_NAV_BRIDGE_SOCKET="$BRIDGE_SOCKET"

if [ -f "$PID_FILE" ]; then
  PID="$(cat "$PID_FILE")"
  if kill -0 "$PID" 2>/dev/null; then
    echo "nav console already running: pid=$PID"
    exit 0
  fi
fi

HEALTH="$(curl -fsS "http://127.0.0.1:$PORT/healthz" 2>/dev/null || true)"
if printf '%s' "$HEALTH" | grep -q '"mock"[[:space:]]*:[[:space:]]*true'; then
  echo "port $PORT is occupied by a mock console; stop it before starting real Nav2 console" >&2
  exit 2
fi
if printf '%s' "$HEALTH" | grep -q '"ok"[[:space:]]*:[[:space:]]*true'; then
  echo "nav console already reachable on port $PORT"
  exit 0
fi

WARMUP_RUNNING=0
if [ -f "$WARMUP_PID_FILE" ]; then
  WARMUP_PID="$(cat "$WARMUP_PID_FILE")"
  if kill -0 "$WARMUP_PID" 2>/dev/null; then
    WARMUP_RUNNING=1
  fi
fi

if [ "$WARMUP_RUNNING" -eq 0 ]; then
  rm -f "$WARMUP_PID_FILE"
  nohup bash "$ROOT/nav_console/tools/run_dds_warmup.sh" \
    > "$WARMUP_LOG" 2>&1 &
  echo $! > "$WARMUP_PID_FILE"
  sleep 1
  if ! kill -0 "$(cat "$WARMUP_PID_FILE")" 2>/dev/null; then
    echo "FastDDS warmup failed; log: $WARMUP_LOG" >&2
    tail -n 20 "$WARMUP_LOG" >&2 || true
    exit 2
  fi
fi

BRIDGE_RUNNING=0
if [ -f "$BRIDGE_PID_FILE" ]; then
  BRIDGE_PID="$(cat "$BRIDGE_PID_FILE")"
  if kill -0 "$BRIDGE_PID" 2>/dev/null && [ -S "$BRIDGE_SOCKET" ]; then
    BRIDGE_RUNNING=1
  fi
fi

if [ "$BRIDGE_RUNNING" -eq 0 ]; then
  rm -f "$BRIDGE_SOCKET" "$BRIDGE_PID_FILE"
  CONFIG_JSON="$(python3 -c \
    'import json,sys; print(json.dumps(json.load(open(sys.argv[1]))))' \
    "$CONFIG")"
  nohup bash "$ROOT/nav_console/tools/run_ros_bridge.sh" \
    "$CONFIG_JSON" "$BRIDGE_SOCKET" \
    > "$BRIDGE_LOG" 2>&1 &
  echo $! > "$BRIDGE_PID_FILE"

  for _ in $(seq 1 80); do
    if [ -S "$BRIDGE_SOCKET" ]; then
      break
    fi
    if ! kill -0 "$(cat "$BRIDGE_PID_FILE")" 2>/dev/null; then
      break
    fi
    sleep 0.25
  done

  if [ ! -S "$BRIDGE_SOCKET" ]; then
    echo "ROS bridge failed to start; log: $BRIDGE_LOG" >&2
    tail -n 20 "$BRIDGE_LOG" >&2 || true
    exit 2
  fi
fi

nohup python3 "$ROOT/nav_console/server.py" \
  --host 0.0.0.0 \
  --port "$PORT" \
  --config "$CONFIG" \
  --web-root "$ROOT/nav_console/web" \
  --ros-domain-id "$ROS_DOMAIN_ID" \
  > "$LOG_FILE" 2>&1 &

echo $! > "$PID_FILE"

for _ in $(seq 1 30); do
  if curl -fsS "http://127.0.0.1:$PORT/healthz" >/dev/null 2>&1; then
    break
  fi
  sleep 0.5
done

echo "nav console: http://192.168.128.10:$PORT"
echo "pid: $(cat "$PID_FILE") log: $LOG_FILE"
echo "bridge: pid=$(cat "$BRIDGE_PID_FILE") socket=$BRIDGE_SOCKET"
echo "warmup: pid=$(cat "$WARMUP_PID_FILE") log=$WARMUP_LOG"

