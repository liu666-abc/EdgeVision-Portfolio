#!/usr/bin/env bash
set -eo pipefail

PID_FILE=/tmp/evi_nav_console.pid
BRIDGE_PID_FILE=/tmp/evi_nav_bridge.pid
BRIDGE_SOCKET="${EVI_NAV_BRIDGE_SOCKET:-/tmp/evi_nav_bridge.sock}"
WARMUP_PID_FILE=/tmp/evi_nav_warmup.pid
SERVER_PIDS=''
BRIDGE_PIDS=''
WARMUP_PIDS=''

if [ -f "$PID_FILE" ]; then
  PID="$(cat "$PID_FILE")"
  if kill -0 "$PID" 2>/dev/null; then
    SERVER_PIDS="$PID"
  fi
fi

for PID in $(pgrep -f '[n]av_console/server.py' || true); do
  SERVER_PIDS="$SERVER_PIDS $PID"
done
SERVER_PIDS="$(printf '%s\n' $SERVER_PIDS | sort -u | tr '\n' ' ')"

if [ -f "$BRIDGE_PID_FILE" ]; then
  PID="$(cat "$BRIDGE_PID_FILE")"
  if kill -0 "$PID" 2>/dev/null; then
    BRIDGE_PIDS="$BRIDGE_PIDS $PID"
  fi
fi

for PID in $(pgrep -f '[r]os_bridge_process.py' || true); do
  BRIDGE_PIDS="$BRIDGE_PIDS $PID"
done
BRIDGE_PIDS="$(printf '%s\n' $BRIDGE_PIDS | sort -u | tr '\n' ' ')"

if [ -f "$WARMUP_PID_FILE" ]; then
  PID="$(cat "$WARMUP_PID_FILE")"
  if kill -0 "$PID" 2>/dev/null; then
    WARMUP_PIDS="$WARMUP_PIDS $PID"
  fi
fi

for PID in $(pgrep -f '[d]ds_warmup.py' || true); do
  WARMUP_PIDS="$WARMUP_PIDS $PID"
done
WARMUP_PIDS="$(printf '%s\n' $WARMUP_PIDS | sort -u | tr '\n' ' ')"

if [ -z "$SERVER_PIDS$BRIDGE_PIDS$WARMUP_PIDS" ]; then
  rm -f "$PID_FILE" "$BRIDGE_PID_FILE" "$BRIDGE_SOCKET" "$WARMUP_PID_FILE"
  echo "nav console not running"
  exit 0
fi

stop_targets() {
  TARGETS="$1"
  if [ -z "$TARGETS" ]; then
    return 0
  fi
  for _ in $(seq 1 8); do
    ACTIVE=0
    for PID in $TARGETS; do
      if kill -0 "$PID" 2>/dev/null; then
        ACTIVE=1
        break
      fi
    done
    if [ "$ACTIVE" -eq 0 ]; then
      return 0
    fi
    sleep 0.25
  done
  return 1
}

for PID in $SERVER_PIDS; do
  kill -INT "$PID" 2>/dev/null || true
done

if ! stop_targets "$SERVER_PIDS"; then
  for PID in $SERVER_PIDS; do
    kill -TERM "$PID" 2>/dev/null || true
  done
  if ! stop_targets "$SERVER_PIDS"; then
    for PID in $SERVER_PIDS; do
      kill -KILL "$PID" 2>/dev/null || true
    done
  fi
fi

for PID in $BRIDGE_PIDS; do
  kill -INT "$PID" 2>/dev/null || true
done

if ! stop_targets "$BRIDGE_PIDS"; then
  for PID in $BRIDGE_PIDS; do
    kill -TERM "$PID" 2>/dev/null || true
  done
  if ! stop_targets "$BRIDGE_PIDS"; then
    for PID in $BRIDGE_PIDS; do
      kill -KILL "$PID" 2>/dev/null || true
    done
  fi
fi

for PID in $WARMUP_PIDS; do
  kill -INT "$PID" 2>/dev/null || true
done

if ! stop_targets "$WARMUP_PIDS"; then
  for PID in $WARMUP_PIDS; do
    kill -TERM "$PID" 2>/dev/null || true
  done
  if ! stop_targets "$WARMUP_PIDS"; then
    for PID in $WARMUP_PIDS; do
      kill -KILL "$PID" 2>/dev/null || true
    done
  fi
fi

rm -f "$PID_FILE" "$BRIDGE_PID_FILE" "$BRIDGE_SOCKET" "$WARMUP_PID_FILE"

echo "nav console stopped"

