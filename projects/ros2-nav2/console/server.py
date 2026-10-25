#!/usr/bin/env python3
"""Low-latency ROS2 Nav2 web console.

The server intentionally exposes a narrow command surface instead of a generic
ROS bridge:

- subscribe to map, TF, scan, paths, odometry and Nav2 action state;
- push telemetry over one WebSocket;
- send NavigateToPose goals and /initialpose updates;
- run mapping/navigation processes from an explicit command allowlist.

It never exposes arbitrary shell execution or teleop commands.
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import json
import math
import os
import queue
import re
import signal
import socket
import struct
import subprocess
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

from aiohttp import WSMsgType, web


MAP_STRUCT = struct.Struct("<4sBBIIIffffI")
MAP_MAGIC = b"EVM1"
SAFE_MAP_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$")


DEFAULT_CONFIG: dict[str, Any] = {
    "title": "EVI Nav2 Mission Console",
    "ros_domain_id": 59,
    "map_frame": "map",
    "base_frame": "base_footprint",
    "ws_hz": 20.0,
    "scene_hz": 10.0,
    "map_hz": 2.0,
    "camera_stream_url": "http://127.0.0.1:8080/api/stream.mjpg",
    "topics": {
        "map": "/map",
        "scan": "/scan",
        "amcl_pose": "/amcl_pose",
        "global_path": "/plan",
        "local_path": "/local_plan",
        "odom": "/odom_combined",
    },
    "actions": {"navigate_to_pose": "/navigate_to_pose"},
    "operations": {
        "mapping_start": {
            "label": "开始建图",
            "mode": "process",
            "command": [
                "bash",
                "/home/wheeltec/evi/evi_dashboard/board_agent/start_mapping_app.sh",
            ],
        },
        "mapping_stop": {
            "label": "停止建图",
            "mode": "signal",
            "target": "mapping_start",
        },
        "navigation_start": {
            "label": "启动导航",
            "mode": "process",
            "command": [
                "bash",
                "/home/wheeltec/evi/evi_dashboard/board_agent/start_nav2_app.sh",
            ],
        },
        "navigation_stop": {
            "label": "停止导航",
            "mode": "signal",
            "target": "navigation_start",
        },
        "save_map": {
            "label": "保存地图",
            "mode": "save_map",
            "output_dir": "/home/wheeltec/evi/a3_nav2",
        },
    },
}


def _deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    result = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = value
    return result


def _load_config(path: Path | None) -> dict[str, Any]:
    if path is None:
        return _deep_merge(DEFAULT_CONFIG, {})
    payload = json.loads(path.read_text(encoding="utf-8"))
    return _deep_merge(DEFAULT_CONFIG, payload)


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= math.tau
    while angle < -math.pi:
        angle += math.tau
    return angle


def _compose_transform(
    first: tuple[float, float, float],
    second: tuple[float, float, float],
) -> tuple[float, float, float]:
    x1, y1, yaw1 = first
    x2, y2, yaw2 = second
    cos_yaw = math.cos(yaw1)
    sin_yaw = math.sin(yaw1)
    return (
        x1 + cos_yaw * x2 - sin_yaw * y2,
        y1 + sin_yaw * x2 + cos_yaw * y2,
        _normalize_angle(yaw1 + yaw2),
    )


def _invert_transform(
    transform: tuple[float, float, float],
) -> tuple[float, float, float]:
    x, y, yaw = transform
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        -cos_yaw * x - sin_yaw * y,
        sin_yaw * x - cos_yaw * y,
        _normalize_angle(-yaw),
    )


def _lookup_tf_chain(
    edges: dict[tuple[str, str], tuple[float, float, float]],
    target: str,
    source: str,
) -> tuple[float, float, float] | None:
    if target == source:
        return (0.0, 0.0, 0.0)
    queue: deque[tuple[str, tuple[float, float, float]]] = deque(
        [(target, (0.0, 0.0, 0.0))]
    )
    visited = {target}
    while queue:
        current, accumulated = queue.popleft()
        for (parent, child), transform in edges.items():
            if parent == current and child not in visited:
                visited.add(child)
                composed = _compose_transform(accumulated, transform)
                if child == source:
                    return composed
                queue.append((child, composed))
            elif child == current and parent not in visited:
                visited.add(parent)
                composed = _compose_transform(
                    accumulated,
                    _invert_transform(transform),
                )
                if parent == source:
                    return composed
                queue.append((parent, composed))
    return None


def _finite_float(value: Any, name: str, limit: float = 100000.0) -> float:
    number = float(value)
    if not math.isfinite(number) or abs(number) > limit:
        raise ValueError(f"{name} must be a finite number within +/-{limit}")
    return number


class StateStore:
    """Thread-safe state shared by the ROS thread and aiohttp tasks."""

    def __init__(self, config: dict[str, Any]) -> None:
        self.config = config
        self._lock = threading.RLock()
        self._pose = {
            "x": 0.0,
            "y": 0.0,
            "yaw": 0.0,
            "frame": config["map_frame"],
            "updated_monotonic": 0.0,
        }
        self._speed = {"linear_x": 0.0, "angular_z": 0.0}
        self._scan: list[list[float]] = []
        self._scan_polar: list[list[float]] = []
        self._scan_frame = ""
        self._scan_transformed = False
        self._global_path: list[list[float]] = []
        self._local_path: list[list[float]] = []
        self._footprint: list[list[float]] = []
        self._goal: dict[str, float] | None = None
        self._initial_pose: dict[str, float] | None = None
        self._nav = {
            "mode": "IDLE",
            "server_ready": False,
            "goal_active": False,
            "distance_remaining": None,
            "eta_s": None,
            "recoveries": 0,
            "last_result": "",
        }
        self._map_meta = {
            "revision": 0,
            "width": 0,
            "height": 0,
            "resolution": 0.0,
            "origin_x": 0.0,
            "origin_y": 0.0,
            "origin_yaw": 0.0,
            "updated_monotonic": 0.0,
        }
        self._map_packet: bytes | None = None
        self._scan_revision = 0
        self._path_revision = 0
        self._scene_revision = 0
        self._status_revision = 0
        self._telemetry_revision = 0
        self._last_scan_monotonic = 0.0
        self._last_amcl_monotonic = 0.0
        self._last_path_monotonic = 0.0
        self._ros_connected = False
        self._bridge_publishers: dict[str, int] = {}
        self._bridge_callback_counts: dict[str, int] = {}
        self._events: deque[dict[str, Any]] = deque(maxlen=120)
        self._processes: dict[str, dict[str, Any]] = {}
        self._started_monotonic = time.monotonic()
        self.add_event("info", "Nav2 控制台状态服务已启动")

    def _bump_scene_locked(self) -> None:
        self._scene_revision += 1

    def add_event(self, level: str, message: str) -> None:
        with self._lock:
            self._events.appendleft(
                {
                    "time": time.strftime("%H:%M:%S"),
                    "level": level,
                    "message": message,
                }
            )
            self._status_revision += 1

    def set_ros_connected(self, connected: bool) -> None:
        with self._lock:
            if self._ros_connected != connected:
                self._ros_connected = connected
                self._status_revision += 1
                self.add_event(
                    "info" if connected else "warn",
                    "ROS2 数据桥已连接" if connected else "ROS2 数据桥已断开",
                )

    def set_pose(
        self,
        x: float,
        y: float,
        yaw: float,
        age_ms: float | None = None,
    ) -> None:
        with self._lock:
            age_s = max(0.0, age_ms or 0.0) / 1000.0
            self._pose.update(
                {
                    "x": round(x, 4),
                    "y": round(y, 4),
                    "yaw": round(yaw, 5),
                    "frame": self.config["map_frame"],
                    "updated_monotonic": time.monotonic() - age_s,
                }
            )
            self._telemetry_revision += 1

    def set_amcl_pose(self) -> None:
        with self._lock:
            self._last_amcl_monotonic = time.monotonic()
            self._status_revision += 1

    def set_sensor_ages(
        self,
        scan_age_ms: float | None = None,
        amcl_age_ms: float | None = None,
        path_age_ms: float | None = None,
    ) -> None:
        now = time.monotonic()
        with self._lock:
            if scan_age_ms is not None:
                self._last_scan_monotonic = now - max(0.0, scan_age_ms) / 1000.0
            if amcl_age_ms is not None:
                self._last_amcl_monotonic = now - max(0.0, amcl_age_ms) / 1000.0
            if path_age_ms is not None:
                self._last_path_monotonic = now - max(0.0, path_age_ms) / 1000.0

    def set_bridge_publishers(self, publishers: dict[str, Any]) -> None:
        cleaned: dict[str, int] = {}
        for name, count in publishers.items():
            try:
                cleaned[str(name)] = max(0, int(count))
            except (TypeError, ValueError):
                continue
        with self._lock:
            if cleaned != self._bridge_publishers:
                self._bridge_publishers = cleaned
                self._telemetry_revision += 1

    def set_bridge_callback_counts(self, counts: dict[str, Any]) -> None:
        cleaned: dict[str, int] = {}
        for name, count in counts.items():
            try:
                cleaned[str(name)] = max(0, int(count))
            except (TypeError, ValueError):
                continue
        with self._lock:
            if cleaned != self._bridge_callback_counts:
                self._bridge_callback_counts = cleaned
                self._telemetry_revision += 1

    def set_speed(self, linear_x: float, angular_z: float) -> None:
        with self._lock:
            self._speed = {
                "linear_x": round(float(linear_x), 4),
                "angular_z": round(float(angular_z), 4),
            }
            self._telemetry_revision += 1

    def set_scan(
        self,
        points: list[list[float]],
        polar_points: list[list[float]] | None = None,
        frame: str = "",
        transformed: bool = True,
    ) -> None:
        with self._lock:
            self._scan = points
            self._scan_polar = polar_points or []
            self._scan_frame = frame
            self._scan_transformed = transformed
            self._last_scan_monotonic = time.monotonic()
            self._scan_revision += 1
            self._bump_scene_locked()

    def set_paths(
        self,
        global_path: list[list[float]] | None = None,
        local_path: list[list[float]] | None = None,
    ) -> None:
        with self._lock:
            if global_path is not None:
                self._global_path = global_path
            if local_path is not None:
                self._local_path = local_path
            self._last_path_monotonic = time.monotonic()
            self._path_revision += 1
            self._bump_scene_locked()

    def set_footprint(self, points: list[list[float]]) -> None:
        with self._lock:
            self._footprint = points
            self._bump_scene_locked()

    def set_goal(self, x: float, y: float, yaw: float | None = None) -> None:
        with self._lock:
            self._goal = {
                "x": round(x, 4),
                "y": round(y, 4),
                "yaw": round(0.0 if yaw is None else yaw, 5),
            }
            self._bump_scene_locked()

    def clear_goal(self) -> None:
        with self._lock:
            self._goal = None
            self._bump_scene_locked()

    def set_initial_pose(self, x: float, y: float, yaw: float) -> None:
        with self._lock:
            self._initial_pose = {
                "x": round(x, 4),
                "y": round(y, 4),
                "yaw": round(yaw, 5),
            }
            self._bump_scene_locked()

    def update_nav(self, **values: Any) -> None:
        with self._lock:
            changed = any(self._nav.get(key) != value for key, value in values.items())
            self._nav.update(values)
            self._telemetry_revision += 1
            if changed:
                self._status_revision += 1

    def set_map(
        self,
        width: int,
        height: int,
        resolution: float,
        origin_x: float,
        origin_y: float,
        origin_yaw: float,
        data: list[int] | tuple[int, ...] | bytes | bytearray,
    ) -> None:
        if width <= 0 or height <= 0 or len(data) != width * height:
            raise ValueError("invalid occupancy grid dimensions")
        if isinstance(data, (bytes, bytearray)):
            raw = bytes(data)
        else:
            raw = bytes((int(value) & 0xFF) for value in data)
        with self._lock:
            revision = int(self._map_meta["revision"]) + 1
            header = MAP_STRUCT.pack(
                MAP_MAGIC,
                1,
                MAP_STRUCT.size,
                int(width),
                int(height),
                revision,
                float(resolution),
                float(origin_x),
                float(origin_y),
                float(origin_yaw),
                len(raw),
            )
            self._map_packet = header + raw
            self._map_meta.update(
                {
                    "revision": revision,
                    "width": int(width),
                    "height": int(height),
                    "resolution": round(float(resolution), 6),
                    "origin_x": round(float(origin_x), 5),
                    "origin_y": round(float(origin_y), 5),
                    "origin_yaw": round(float(origin_yaw), 5),
                    "updated_monotonic": time.monotonic(),
                }
            )
            self._telemetry_revision += 1
            self._status_revision += 1

    def set_process(self, name: str, **values: Any) -> None:
        with self._lock:
            current = self._processes.get(name, {})
            current.update(values)
            current["updated_at"] = time.time()
            self._processes[name] = current
            self._status_revision += 1

    def get_process(self, name: str) -> dict[str, Any]:
        with self._lock:
            return dict(self._processes.get(name, {}))

    def get_map_packet(self) -> tuple[bytes | None, int]:
        with self._lock:
            return self._map_packet, int(self._map_meta["revision"])

    def _age_ms(self, timestamp: float) -> float | None:
        if timestamp <= 0.0:
            return None
        return round((time.monotonic() - timestamp) * 1000.0, 1)

    def snapshot_telemetry(self) -> dict[str, Any]:
        with self._lock:
            return {
                "type": "telemetry",
                "revision": self._telemetry_revision,
                "server_time": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
                "uptime_s": round(time.monotonic() - self._started_monotonic, 1),
                "ros_connected": self._ros_connected,
                "frame": self.config["map_frame"],
                "pose": {
                    **self._pose,
                    "age_ms": self._age_ms(self._pose["updated_monotonic"]),
                },
                "speed": dict(self._speed),
                "nav": dict(self._nav),
                "map": {
                    key: value
                    for key, value in self._map_meta.items()
                    if key != "updated_monotonic"
                }
                | {
                    "age_ms": self._age_ms(
                        float(self._map_meta["updated_monotonic"])
                    )
                },
                "sensor_age_ms": {
                    "scan": self._age_ms(self._last_scan_monotonic),
                    "amcl": self._age_ms(self._last_amcl_monotonic),
                    "path": self._age_ms(self._last_path_monotonic),
                },
                "bridge_publishers": dict(self._bridge_publishers),
                "bridge_callback_counts": dict(self._bridge_callback_counts),
            }

    def snapshot_scene(self) -> dict[str, Any]:
        with self._lock:
            return {
                "type": "scene",
                "revision": self._scene_revision,
                "scan": self._scan,
                "scan_polar": self._scan_polar,
                "scan_frame": self._scan_frame,
                "scan_transformed": self._scan_transformed,
                "global_path": self._global_path,
                "local_path": self._local_path,
                "footprint": self._footprint,
                "goal": self._goal,
                "initial_pose": self._initial_pose,
            }

    def snapshot_status(self) -> dict[str, Any]:
        with self._lock:
            operations = {
                key: {
                    "label": value.get("label", key),
                    "mode": value.get("mode", "oneshot"),
                }
                for key, value in self.config["operations"].items()
            }
            return {
                "type": "status",
                "revision": self._status_revision,
                "ros_connected": self._ros_connected,
                "nav": dict(self._nav),
                "processes": {
                    key: dict(value) for key, value in self._processes.items()
                },
                "operations": operations,
                "events": list(self._events),
            }

    def public_config(self) -> dict[str, Any]:
        camera = self.config.get("camera_stream_url", "")
        return {
            "title": self.config.get("title", DEFAULT_CONFIG["title"]),
            "map_frame": self.config["map_frame"],
            "base_frame": self.config["base_frame"],
            "ws_hz": self.config["ws_hz"],
            "camera_stream_url": camera,
            "topics": self.config["topics"],
            "actions": self.config["actions"],
        }


class ProcessManager:
    """Runs only operations declared in the loaded JSON config."""

    def __init__(
        self,
        state: StateStore,
        config: dict[str, Any],
        mock: bool = False,
    ) -> None:
        self.state = state
        self.config = config
        self.mock = mock
        self._lock = threading.RLock()
        self._processes: dict[str, subprocess.Popen[str]] = {}

    def _operation(self, name: str) -> dict[str, Any]:
        operation = self.config.get("operations", {}).get(name)
        if not isinstance(operation, dict):
            raise ValueError(f"unknown operation: {name}")
        return operation

    def _spawn(
        self,
        name: str,
        command: list[str],
        mode: str,
    ) -> dict[str, Any]:
        if self.mock:
            self.state.set_process(
                name,
                running=mode == "process",
                mock=True,
                pid=None,
                command=command,
            )
            self.state.add_event("info", f"[mock] 已执行 {name}")
            return {"ok": True, "mock": True, "name": name}
        if not command or not all(isinstance(item, str) for item in command):
            raise ValueError(f"invalid command for {name}")
        kwargs: dict[str, Any] = {
            "stdout": subprocess.PIPE,
            "stderr": subprocess.STDOUT,
            "text": True,
            "bufsize": 1,
        }
        if os.name != "nt":
            kwargs["start_new_session"] = True
        process = subprocess.Popen(command, **kwargs)
        with self._lock:
            self._processes[name] = process
        self.state.set_process(
            name,
            running=True,
            pid=process.pid,
            command=command,
            mode=mode,
            last_result="",
        )
        self.state.add_event("info", f"启动 {name}，pid={process.pid}")

        def monitor() -> None:
            output_tail: deque[str] = deque(maxlen=20)
            if process.stdout is not None:
                for line in process.stdout:
                    line = line.rstrip()
                    if line:
                        output_tail.append(line)
            code = process.wait()
            with self._lock:
                if self._processes.get(name) is process:
                    self._processes.pop(name, None)
            self.state.set_process(
                name,
                running=False,
                pid=None,
                exit_code=code,
                last_result="ok" if code == 0 else "failed",
                output=list(output_tail),
            )
            self.state.add_event(
                "info" if code == 0 else "warn",
                f"{name} 退出，code={code}",
            )

        threading.Thread(
            target=monitor,
            name=f"process-monitor-{name}",
            daemon=True,
        ).start()
        return {"ok": True, "name": name, "pid": process.pid}

    def start(self, name: str) -> dict[str, Any]:
        operation = self._operation(name)
        mode = str(operation.get("mode", "oneshot"))
        if mode == "signal":
            raise ValueError(f"{name} is not a start operation")
        if mode == "save_map":
            raise ValueError("save_map requires a map name")
        with self._lock:
            current = self.state.get_process(name)
            if current.get("running"):
                return {"ok": False, "error": f"{name} already running"}
        command = operation.get("command")
        if not isinstance(command, list):
            raise ValueError(f"{name} has no command list")
        return self._spawn(name, command, mode)

    def stop_process(self, name: str) -> dict[str, Any]:
        if self.mock:
            self.state.set_process(name, running=False, last_result="canceled")
            self.state.add_event("info", f"[mock] 已停止 {name}")
            return {"ok": True, "mock": True, "name": name}
        with self._lock:
            process = self._processes.get(name)
        if process is None or process.poll() is not None:
            self.state.set_process(name, running=False, last_result="not_running")
            return {"ok": False, "error": f"{name} is not running"}
        try:
            if os.name != "nt":
                os.killpg(os.getpgid(process.pid), signal.SIGINT)
            else:
                process.send_signal(signal.SIGINT)
        except ProcessLookupError:
            pass
        self.state.add_event("warn", f"已向 {name} 发送 SIGINT")
        return {"ok": True, "name": name, "signal": "SIGINT"}

    def signal_operation(self, name: str) -> dict[str, Any]:
        operation = self._operation(name)
        if operation.get("mode") != "signal":
            raise ValueError(f"{name} is not a signal operation")
        target = str(operation.get("target", ""))
        if not target:
            raise ValueError(f"{name} has no target")
        return self.stop_process(target)

    def is_running(self, name: str) -> bool:
        return bool(self.state.get_process(name).get("running"))

    def save_map(self, name: str) -> dict[str, Any]:
        operation = self._operation("save_map")
        if operation.get("mode") != "save_map":
            raise ValueError("save_map operation is not configured")
        if not SAFE_MAP_NAME.fullmatch(name):
            raise ValueError("map name must match [A-Za-z0-9_-]{1,64}")
        output_dir = Path(str(operation.get("output_dir", ".")))
        output = output_dir / name
        command = [
            "ros2",
            "run",
            "nav2_map_server",
            "map_saver_cli",
            "-f",
            str(output),
        ]
        return self._spawn("save_map", command, "save_map")


class MockSource:
    def __init__(self, state: StateStore) -> None:
        self.state = state
        self._stop = threading.Event()
        self._lock = threading.RLock()
        self._started = time.monotonic()
        self._pose = [2.0, 1.8, 0.0]
        self._goal: list[float] | None = [25.0, 3.2, 0.1]
        self._resolution = 0.1
        self._width = 300
        self._height = 100
        self._origin_x = -2.0
        self._origin_y = -2.0
        self._map = self._build_map()

    def _build_map(self) -> bytearray:
        values = bytearray([255]) * (self._width * self._height)

        def set_cell(x: int, y: int, value: int = 0) -> None:
            if 0 <= x < self._width and 0 <= y < self._height:
                values[y * self._width + x] = value

        def fill_rect(x0: int, y0: int, x1: int, y1: int, value: int) -> None:
            for y in range(max(0, y0), min(self._height, y1)):
                for x in range(max(0, x0), min(self._width, x1)):
                    set_cell(x, y, value)

        fill_rect(0, 0, self._width, 3, 100)
        fill_rect(0, self._height - 3, self._width, self._height, 100)
        fill_rect(0, 0, 3, self._height, 100)
        fill_rect(self._width - 3, 0, self._width, self._height, 100)
        fill_rect(78, 3, 80, 34, 100)
        fill_rect(78, 52, 80, self._height - 3, 100)
        fill_rect(145, 3, 148, 28, 100)
        fill_rect(145, 44, 148, self._height - 3, 100)
        fill_rect(218, 3, 220, 37, 100)
        fill_rect(218, 55, 220, self._height - 3, 100)
        fill_rect(112, 31, 124, 43, 100)
        fill_rect(176, 61, 192, 74, 100)
        fill_rect(245, 20, 259, 35, 100)
        return values

    def start(self) -> None:
        self.state.set_ros_connected(True)
        self.state.set_map(
            self._width,
            self._height,
            self._resolution,
            self._origin_x,
            self._origin_y,
            0.0,
            self._map,
        )

        def run() -> None:
            while not self._stop.is_set():
                now = time.monotonic()
                elapsed = now - self._started
                with self._lock:
                    if self._goal is not None:
                        dx = self._goal[0] - self._pose[0]
                        dy = self._goal[1] - self._pose[1]
                        distance = math.hypot(dx, dy)
                        desired_yaw = math.atan2(dy, dx)
                        if distance > 0.08:
                            step = min(0.035, distance)
                            self._pose[0] += step * dx / distance
                            self._pose[1] += step * dy / distance
                            self._pose[2] = desired_yaw
                            nav_mode = "NAVIGATING"
                        else:
                            self._pose[2] = self._goal[2]
                            self._goal = None
                            distance = 0.0
                            nav_mode = "SUCCEEDED"
                    else:
                        self._pose[0] = 2.0 + 8.0 * (0.5 + 0.5 * math.sin(elapsed * 0.12))
                        self._pose[1] = 3.4 + 0.55 * math.sin(elapsed * 0.45)
                        self._pose[2] = 0.07 * math.sin(elapsed * 0.45)
                        distance = None
                        nav_mode = "IDLE"
                self.state.set_pose(*self._pose)
                self.state.set_speed(0.12 if nav_mode == "NAVIGATING" else 0.0, 0.08)
                self.state.update_nav(
                    mode=nav_mode,
                    server_ready=True,
                    goal_active=self._goal is not None,
                    distance_remaining=distance,
                    eta_s=(distance / 0.12) if distance else 0.0,
                    recoveries=0,
                )
                if self._goal is not None:
                    self.state.set_goal(*self._goal)
                else:
                    self.state.clear_goal()
                scan: list[list[float]] = []
                pose_x, pose_y, pose_yaw = self._pose
                for index in range(180):
                    angle = pose_yaw - math.pi + (index / 179.0) * math.pi * 2.0
                    distance = 1.0 + 0.15 * math.sin(angle * 7.0 + elapsed)
                    scan.append(
                        [
                            round(pose_x + distance * math.cos(angle), 3),
                            round(pose_y + distance * math.sin(angle), 3),
                        ]
                    )
                self.state.set_scan(scan)
                path = [
                    [2.0, 1.8],
                    [7.0, 2.4],
                    [12.0, 3.5],
                    [18.0, 3.8],
                    [23.0, 3.1],
                    [27.0, 2.6],
                ]
                self.state.set_paths(global_path=path, local_path=path[1:4])
                self.state.set_footprint([[-0.16, -0.12], [0.16, -0.12], [0.16, 0.12], [-0.16, 0.12]])
                time.sleep(1.0 / 30.0)

        threading.Thread(target=run, name="mock-source", daemon=True).start()

    def submit_goal(self, x: float, y: float, yaw: float) -> None:
        with self._lock:
            self._goal = [x, y, yaw]
        self.state.set_goal(x, y, yaw)
        self.state.update_nav(mode="ACCEPTED", goal_active=True)
        self.state.add_event("info", "Mock 导航目标已接收")

    def cancel_goal(self) -> None:
        with self._lock:
            self._goal = None
        self.state.clear_goal()
        self.state.update_nav(mode="CANCELED", goal_active=False)

    def stop(self) -> None:
        self._stop.set()


class RosBridge:
    """Runs rclpy in a worker thread and accepts commands through a queue."""

    def __init__(self, state: StateStore, config: dict[str, Any]) -> None:
        self.state = state
        self.config = config
        self.commands: queue.Queue[dict[str, Any]] = queue.Queue()
        self._thread: threading.Thread | None = None
        self._node: Any = None
        self._stop_event = threading.Event()
        self._rclpy: Any = None

    def start(self) -> None:
        self._thread = threading.Thread(
            target=self._run,
            name="rclpy-nav-console",
            daemon=True,
        )
        self._thread.start()

    def submit(self, command: dict[str, Any]) -> None:
        self.commands.put(command)

    def _run(self) -> None:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor

        self._rclpy = rclpy
        rclpy.init()
        self._node = self._make_node()
        executor = SingleThreadedExecutor()
        executor.add_node(self._node)
        self.state.set_ros_connected(True)
        self._node.get_logger().info("nav console ROS spin loop started")
        spin_cycles = 0
        try:
            while rclpy.ok() and not self._stop_event.is_set():
                executor.spin_once(timeout_sec=0.05)
                spin_cycles += 1
                if spin_cycles % 100 == 0:
                    self._node.get_logger().info(
                        f"nav console ROS spin cycles={spin_cycles}"
                    )
        finally:
            self.state.set_ros_connected(False)
            executor.remove_node(self._node)
            executor.shutdown()
            if self._node is not None:
                self._node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    def _make_node(self) -> Any:
        import rclpy

        state = self.state
        config = self.config
        commands = self.commands
        map_frame = config["map_frame"]
        base_frame = config["base_frame"]
        topics = config["topics"]

        from action_msgs.msg import GoalStatus
        from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
        from nav2_msgs.action import NavigateToPose
        from nav_msgs.msg import OccupancyGrid, Odometry, Path
        from rclpy.action import ActionClient
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from sensor_msgs.msg import LaserScan
        from std_srvs.srv import Empty
        from tf2_msgs.msg import TFMessage

        outer = self

        class ConsoleNode(Node):
            def __init__(self) -> None:
                super().__init__("evi_nav_console")
                self.goal_handle: Any = None
                self.goal_total_distance: float | None = None
                self.last_base_lookup_error = ""
                self.map_callback_count = 0
                self.scan_callback_count = 0
                self.last_teleop_monotonic = 0.0
                self.control_stop_event = threading.Event()
                self.navigation_enabled = False
                self.tf_edges: dict[
                    tuple[str, str],
                    tuple[float, float, float],
                ] = {}

                map_qos = QoSProfile(
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                )
                static_qos = QoSProfile(
                    history=HistoryPolicy.KEEP_LAST,
                    depth=20,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                )
                self.create_subscription(
                    OccupancyGrid,
                    topics["map"],
                    self.on_map,
                    map_qos,
                )
                self.create_subscription(
                    LaserScan,
                    topics["scan"],
                    self.on_scan,
                    10,
                )
                self.create_subscription(
                    PoseWithCovarianceStamped,
                    topics["amcl_pose"],
                    self.on_amcl_pose,
                    10,
                )
                self.create_subscription(
                    Path,
                    topics["global_path"],
                    self.on_global_path,
                    10,
                )
                self.create_subscription(
                    Path,
                    topics["local_path"],
                    self.on_local_path,
                    10,
                )
                self.create_subscription(
                    Odometry,
                    topics["odom"],
                    self.on_odom,
                    10,
                )
                self.create_subscription(
                    TFMessage,
                    "/tf",
                    self.on_tf,
                    100,
                )
                self.create_subscription(
                    TFMessage,
                    "/tf_static",
                    self.on_tf_static,
                    static_qos,
                )

                self.initial_pose_pub = self.create_publisher(
                    PoseWithCovarianceStamped,
                    "/initialpose",
                    10,
                )
                self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
                self.global_localization_client = self.create_client(
                    Empty,
                    "/reinitialize_global_localization",
                )
                self.nomotion_update_client = self.create_client(
                    Empty,
                    "/request_nomotion_update",
                )
                self.nav_client: Any = None
                self.get_logger().info(
                    f"nav console map={topics['map']} scan={topics['scan']} "
                    f"action={config['actions']['navigate_to_pose']}"
                )
                self.control_thread = threading.Thread(
                    target=self.control_loop,
                    name="nav-console-control",
                    daemon=True,
                )
                self.control_thread.start()

            def destroy_node(self) -> bool:
                self.control_stop_event.set()
                if self.control_thread.is_alive():
                    self.control_thread.join(timeout=1.0)
                return super().destroy_node()

            def control_loop(self) -> None:
                while not self.control_stop_event.is_set():
                    if self.navigation_enabled:
                        if self.nav_client is None:
                            self.nav_client = ActionClient(
                                self,
                                NavigateToPose,
                                config["actions"]["navigate_to_pose"],
                            )
                        state.update_nav(
                            server_ready=self.nav_client.server_is_ready()
                        )
                    else:
                        state.update_nav(server_ready=False)
                    self.process_commands()
                    self.enforce_teleop_deadman()
                    time.sleep(0.05)

            def on_map(self, msg: OccupancyGrid) -> None:
                self.map_callback_count += 1
                if self.map_callback_count == 1:
                    self.get_logger().info(
                        f"map callback received {msg.info.width}x{msg.info.height}"
                    )
                try:
                    origin = msg.info.origin
                    yaw = _yaw_from_quaternion(
                        origin.orientation.x,
                        origin.orientation.y,
                        origin.orientation.z,
                        origin.orientation.w,
                    )
                    state.set_map(
                        int(msg.info.width),
                        int(msg.info.height),
                        float(msg.info.resolution),
                        float(origin.position.x),
                        float(origin.position.y),
                        yaw,
                        msg.data,
                    )
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().error(f"map conversion failed: {exc}")

            def _record_tf(self, message: TFMessage) -> None:
                for item in message.transforms:
                    parent = item.header.frame_id.lstrip("/")
                    child = item.child_frame_id.lstrip("/")
                    if not parent or not child:
                        continue
                    translation = item.transform.translation
                    rotation = item.transform.rotation
                    self.tf_edges[(parent, child)] = (
                        float(translation.x),
                        float(translation.y),
                        _yaw_from_quaternion(
                            rotation.x,
                            rotation.y,
                            rotation.z,
                            rotation.w,
                        ),
                    )
                self._update_pose_from_tf()

            def on_tf(self, message: TFMessage) -> None:
                self._record_tf(message)

            def on_tf_static(self, message: TFMessage) -> None:
                self._record_tf(message)

            def _lookup_map_scan(self, msg: LaserScan) -> Any:
                transform = _lookup_tf_chain(
                    self.tf_edges,
                    map_frame,
                    msg.header.frame_id.lstrip("/"),
                )
                if transform is None:
                    raise RuntimeError("map frame is not available")
                return transform

            def on_scan(self, msg: LaserScan) -> None:
                self.scan_callback_count += 1
                if self.scan_callback_count == 1:
                    self.get_logger().info(
                        f"scan callback received {len(msg.ranges)} points"
                    )
                finite_ranges: list[tuple[float, float]] = []
                angle = msg.angle_min
                for distance in msg.ranges:
                    if (
                        math.isfinite(distance)
                        and msg.range_min <= distance <= msg.range_max
                    ):
                        finite_ranges.append((angle, distance))
                    angle += msg.angle_increment
                stride = max(1, len(finite_ranges) // 720)
                sampled_ranges = finite_ranges[::stride]
                polar_points = [
                    [round(math.degrees(angle), 2), round(distance, 3)]
                    for angle, distance in sampled_ranges
                ]
                state.set_scan(
                    [],
                    polar_points,
                    msg.header.frame_id,
                    False,
                )
                try:
                    tx, ty, yaw = self._lookup_map_scan(msg)
                    cos_yaw = math.cos(yaw)
                    sin_yaw = math.sin(yaw)
                    points: list[list[float]] = []
                    for angle, distance in sampled_ranges:
                        lx = distance * math.cos(angle)
                        ly = distance * math.sin(angle)
                        points.append(
                            [
                                round(tx + cos_yaw * lx - sin_yaw * ly, 3),
                                round(ty + sin_yaw * lx + cos_yaw * ly, 3),
                            ]
                        )
                    state.set_scan(
                        points,
                        polar_points,
                        msg.header.frame_id,
                        True,
                    )
                    self.last_base_lookup_error = ""
                except Exception as exc:  # noqa: BLE001
                    state.set_scan(
                        [],
                        polar_points,
                        msg.header.frame_id,
                        False,
                    )
                    error = str(exc)
                    if error != self.last_base_lookup_error:
                        self.get_logger().warn(f"scan transform unavailable: {error}")
                        self.last_base_lookup_error = error

            def on_amcl_pose(self, _msg: PoseWithCovarianceStamped) -> None:
                pose = _msg.pose.pose
                q = pose.orientation
                state.set_pose(
                    float(pose.position.x),
                    float(pose.position.y),
                    _yaw_from_quaternion(q.x, q.y, q.z, q.w),
                )
                state.set_amcl_pose()

            def _path_points(self, msg: Path) -> list[list[float]]:
                points = [
                    [round(float(pose.pose.position.x), 3), round(float(pose.pose.position.y), 3)]
                    for pose in msg.poses
                ]
                if len(points) > 2000:
                    stride = max(1, len(points) // 2000)
                    points = points[::stride]
                return points

            def on_global_path(self, msg: Path) -> None:
                state.set_paths(global_path=self._path_points(msg))

            def on_local_path(self, msg: Path) -> None:
                state.set_paths(local_path=self._path_points(msg))

            def on_odom(self, msg: Odometry) -> None:
                state.set_speed(
                    float(msg.twist.twist.linear.x),
                    float(msg.twist.twist.angular.z),
                )

            def _update_pose_from_tf(self) -> None:
                for frame in (base_frame, "base_link"):
                    transform = _lookup_tf_chain(
                        self.tf_edges,
                        map_frame,
                        frame.lstrip("/"),
                    )
                    if transform is not None:
                        state.set_pose(*transform)
                        return

            def process_commands(self) -> None:
                while True:
                    try:
                        command = commands.get_nowait()
                    except queue.Empty:
                        return
                    action = command.get("action")
                    try:
                        if action == "goal":
                            self.send_goal(
                                float(command["x"]),
                                float(command["y"]),
                                float(command["yaw"]),
                            )
                        elif action == "initial_pose":
                            self.send_initial_pose(
                                float(command["x"]),
                                float(command["y"]),
                                float(command["yaw"]),
                                bool(command.get("coarse", False)),
                            )
                        elif action == "cancel":
                            self.cancel_goal()
                        elif action == "zero_velocity":
                            self.publish_zero_velocity()
                        elif action == "teleop":
                            self.send_teleop(
                                float(command["linear_x"]),
                                float(command["angular_z"]),
                            )
                        elif action == "enable_navigation":
                            self.navigation_enabled = True
                        elif action == "disable_navigation":
                            self.navigation_enabled = False
                            state.update_nav(server_ready=False)
                        elif action == "global_localize":
                            self.call_empty_service(
                                self.global_localization_client,
                                "全局重定位",
                            )
                        elif action == "nomotion_update":
                            self.call_empty_service(
                                self.nomotion_update_client,
                                "静止重定位更新",
                            )
                    except Exception as exc:  # noqa: BLE001
                        state.add_event("warn", f"{action} 失败: {exc}")

            def send_initial_pose(
                self,
                x: float,
                y: float,
                yaw: float,
                coarse: bool = False,
            ) -> None:
                message = PoseWithCovarianceStamped()
                message.header.frame_id = map_frame
                message.header.stamp = self.get_clock().now().to_msg()
                message.pose.pose.position.x = x
                message.pose.pose.position.y = y
                qx, qy, qz, qw = _quaternion_from_yaw(yaw)
                message.pose.pose.orientation.x = qx
                message.pose.pose.orientation.y = qy
                message.pose.pose.orientation.z = qz
                message.pose.pose.orientation.w = qw
                covariance = [0.0] * 36
                covariance[0] = 1.0 if coarse else 0.25
                covariance[7] = 1.0 if coarse else 0.25
                covariance[35] = 0.25 if coarse else 0.07
                message.pose.covariance = covariance
                self.initial_pose_pub.publish(message)
                state.set_initial_pose(x, y, yaw)
                state.add_event(
                    "info",
                    f"初始位姿已发送 ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f}°)"
                    + ("，粗略协方差" if coarse else ""),
                )

            def call_empty_service(self, client: Any, label: str) -> None:
                if not client.service_is_ready():
                    state.add_event("warn", f"{label}服务未就绪")
                    return
                future = client.call_async(Empty.Request())
                future.add_done_callback(
                    lambda result: state.add_event(
                        "info" if result.exception() is None else "warn",
                        f"{label}请求完成"
                        if result.exception() is None
                        else f"{label}失败: {result.exception()}",
                    )
                )

            def send_goal(self, x: float, y: float, yaw: float) -> None:
                if self.nav_client is None:
                    state.add_event("warn", "导航 action client 尚未启用")
                    state.update_nav(mode="ACTION_UNAVAILABLE")
                    return
                if not self.nav_client.server_is_ready():
                    state.add_event("warn", "NavigateToPose action server 未就绪")
                    state.update_nav(mode="ACTION_UNAVAILABLE")
                    return
                goal = NavigateToPose.Goal()
                goal.pose.header.frame_id = map_frame
                goal.pose.header.stamp = self.get_clock().now().to_msg()
                goal.pose.pose.position.x = x
                goal.pose.pose.position.y = y
                qx, qy, qz, qw = _quaternion_from_yaw(yaw)
                goal.pose.pose.orientation.x = qx
                goal.pose.pose.orientation.y = qy
                goal.pose.pose.orientation.z = qz
                goal.pose.pose.orientation.w = qw
                try:
                    pose = state.snapshot_telemetry()["pose"]
                    self.goal_total_distance = math.hypot(
                        x - float(pose["x"]),
                        y - float(pose["y"]),
                    )
                except Exception:
                    self.goal_total_distance = None
                state.set_goal(x, y, yaw)
                state.update_nav(
                    mode="SENDING",
                    goal_active=True,
                    last_result="",
                    goal_total_distance=self.goal_total_distance,
                )
                state.add_event(
                    "info",
                    f"发送导航目标 ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f}°)",
                )
                future = self.nav_client.send_goal_async(
                    goal,
                    feedback_callback=self.on_goal_feedback,
                )
                future.add_done_callback(self.on_goal_response)

            def on_goal_response(self, future: Any) -> None:
                try:
                    goal_handle = future.result()
                except Exception as exc:  # noqa: BLE001
                    state.update_nav(mode="FAILED", goal_active=False, last_result=str(exc))
                    state.add_event("warn", f"目标发送失败: {exc}")
                    return
                if not goal_handle.accepted:
                    state.update_nav(mode="REJECTED", goal_active=False)
                    state.add_event("warn", "导航目标被拒绝")
                    return
                self.goal_handle = goal_handle
                state.update_nav(mode="NAVIGATING", goal_active=True)
                state.add_event("info", "导航目标已接收")
                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(self.on_goal_result)

            def on_goal_feedback(self, feedback_msg: Any) -> None:
                feedback = feedback_msg.feedback
                distance = getattr(feedback, "distance_remaining", None)
                eta = getattr(feedback, "estimated_time_remaining", None)
                eta_s = None
                if eta is not None:
                    eta_s = float(eta.sec) + float(eta.nanosec) / 1e9
                state.update_nav(
                    mode="NAVIGATING",
                    goal_active=True,
                    distance_remaining=(
                        round(float(distance), 3) if distance is not None else None
                    ),
                    eta_s=eta_s,
                    recoveries=int(getattr(feedback, "number_of_recoveries", 0)),
                )

            def on_goal_result(self, future: Any) -> None:
                try:
                    status = int(future.result().status)
                except Exception as exc:  # noqa: BLE001
                    state.update_nav(mode="FAILED", last_result=str(exc))
                    return
                self.goal_handle = None
                labels = {
                    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
                    GoalStatus.STATUS_ABORTED: "ABORTED",
                    GoalStatus.STATUS_CANCELED: "CANCELED",
                }
                label = labels.get(status, f"STATUS_{status}")
                state.update_nav(
                    mode=label,
                    goal_active=False,
                    distance_remaining=0.0 if label == "SUCCEEDED" else None,
                    last_result=label,
                )
                state.add_event(
                    "info" if label == "SUCCEEDED" else "warn",
                    f"导航结束: {label}",
                )

            def cancel_goal(self) -> None:
                if self.goal_handle is None:
                    state.update_nav(mode="IDLE", goal_active=False)
                    state.add_event("warn", "当前没有活动导航目标")
                    return
                state.update_nav(mode="CANCELING")
                future = self.goal_handle.cancel_goal_async()
                future.add_done_callback(
                    lambda _future: state.add_event("warn", "已请求取消导航目标")
                )

            def publish_zero_velocity(self) -> None:
                self.cmd_vel_pub.publish(Twist())
                self.last_teleop_monotonic = 0.0

            def send_teleop(self, linear_x: float, angular_z: float) -> None:
                linear_x = max(0.0, min(0.12, linear_x))
                angular_z = max(-0.45, min(0.45, angular_z))
                message = Twist()
                message.linear.x = linear_x
                message.angular.z = angular_z
                self.cmd_vel_pub.publish(message)
                self.last_teleop_monotonic = time.monotonic()
                state.set_speed(linear_x, angular_z)

            def enforce_teleop_deadman(self) -> None:
                if (
                    self.last_teleop_monotonic > 0.0
                    and time.monotonic() - self.last_teleop_monotonic > 0.35
                ):
                    self.cmd_vel_pub.publish(Twist())
                    self.last_teleop_monotonic = 0.0
                    state.set_speed(0.0, 0.0)
                    state.add_event("warn", "遥控超时，已自动发送零速")

        return ConsoleNode()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)


class RosBridgeProcess:
    """Connect to the standalone rclpy bridge over a local Unix socket."""

    def __init__(self, state: StateStore, config: dict[str, Any]) -> None:
        self.state = state
        self.config = config
        self.socket_path = os.environ.get(
            "EVI_NAV_BRIDGE_SOCKET",
            "/tmp/evi_nav_bridge.sock",
        )
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._socket: socket.socket | None = None
        self._socket_lock = threading.RLock()
        self._write_lock = threading.Lock()
        self._connected = False
        self._last_connect_error = ""

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._thread = threading.Thread(
            target=self._supervise,
            name="ros-bridge-supervisor",
            daemon=True,
        )
        self._thread.start()

    def submit(self, command: dict[str, Any]) -> None:
        payload = json.dumps(command, separators=(",", ":")) + "\n"
        with self._write_lock:
            with self._socket_lock:
                bridge_socket = self._socket
            if bridge_socket is None:
                raise RuntimeError("ROS bridge socket is not connected")
            try:
                bridge_socket.sendall(payload.encode("utf-8"))
            except OSError as exc:
                raise RuntimeError("ROS bridge socket is not writable") from exc

    def stop(self) -> None:
        self._stop_event.set()
        with self._socket_lock:
            bridge_socket = self._socket
        if bridge_socket is not None:
            try:
                bridge_socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                bridge_socket.close()
            except OSError:
                pass
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=3.0)

    def _supervise(self) -> None:
        while not self._stop_event.is_set():
            try:
                self._connect_once()
            except Exception as exc:  # noqa: BLE001
                message = str(exc)
                if message != self._last_connect_error:
                    print(f"[ros-bridge] connect failed: {message}", flush=True)
                    self._last_connect_error = message
            if self._stop_event.is_set():
                break
            self._set_connected(False)
            self._stop_event.wait(0.5)

    def _connect_once(self) -> None:
        bridge_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        bridge_socket.connect(self.socket_path)
        with self._socket_lock:
            self._socket = bridge_socket
        self._last_connect_error = ""
        print(f"[ros-bridge] connected socket={self.socket_path}", flush=True)
        try:
            stream = bridge_socket.makefile(
                "r",
                encoding="utf-8",
                errors="replace",
            )
            for line in stream:
                line = line.strip()
                if not line:
                    continue
                try:
                    event = json.loads(line)
                except json.JSONDecodeError:
                    print(f"[ros-bridge] invalid JSON: {line[:200]}", flush=True)
                    continue
                if not isinstance(event, dict):
                    continue
                self._handle_event(event)
        finally:
            with self._socket_lock:
                if self._socket is bridge_socket:
                    self._socket = None
            self._set_connected(False)
            try:
                bridge_socket.close()
            except OSError:
                pass

    def _set_connected(self, connected: bool) -> None:
        if self._connected == connected:
            return
        self._connected = connected
        self.state.set_ros_connected(connected)

    @staticmethod
    def _points(value: Any, limit: int = 5000) -> list[list[float]]:
        if not isinstance(value, list):
            return []
        result: list[list[float]] = []
        for point in value[:limit]:
            if not isinstance(point, (list, tuple)) or len(point) < 2:
                continue
            try:
                x = float(point[0])
                y = float(point[1])
            except (TypeError, ValueError):
                continue
            if math.isfinite(x) and math.isfinite(y):
                result.append([x, y])
        return result

    def _handle_event(self, event: dict[str, Any]) -> None:
        event_type = event.get("type")
        if event_type == "map":
            meta = event.get("meta")
            if not isinstance(meta, dict):
                return
            try:
                raw = base64.b64decode(str(event.get("data", "")), validate=True)
                self.state.set_map(
                    int(meta["width"]),
                    int(meta["height"]),
                    float(meta["resolution"]),
                    float(meta["origin_x"]),
                    float(meta["origin_y"]),
                    float(meta["origin_yaw"]),
                    raw,
                )
            except (KeyError, TypeError, ValueError) as exc:
                self.state.add_event("warn", f"ROS bridge map invalid: {exc}")
            self._set_connected(True)
            return

        if event_type == "telemetry":
            pose = event.get("pose")
            if isinstance(pose, dict):
                try:
                    pose_age_ms = self._optional_float(pose.get("age_ms"))
                    if pose_age_ms is not None:
                        self.state.set_pose(
                            float(pose["x"]),
                            float(pose["y"]),
                            float(pose["yaw"]),
                            age_ms=pose_age_ms,
                        )
                except (KeyError, TypeError, ValueError):
                    pass
            speed = event.get("speed")
            if isinstance(speed, dict):
                try:
                    self.state.set_speed(
                        float(speed["linear_x"]),
                        float(speed["angular_z"]),
                    )
                except (KeyError, TypeError, ValueError):
                    pass
            nav = event.get("nav")
            if isinstance(nav, dict):
                allowed = {
                    "mode",
                    "server_ready",
                    "goal_active",
                    "distance_remaining",
                    "eta_s",
                    "recoveries",
                    "last_result",
                    "goal_total_distance",
                }
                self.state.update_nav(
                    **{key: nav[key] for key in allowed if key in nav}
                )
            self.state.set_sensor_ages(
                scan_age_ms=self._optional_float(event.get("scan_age_ms")),
                amcl_age_ms=self._optional_float(event.get("amcl_age_ms")),
                path_age_ms=self._optional_float(event.get("path_age_ms")),
            )
            publishers = event.get("publishers")
            if isinstance(publishers, dict):
                self.state.set_bridge_publishers(publishers)
            callback_counts = event.get("callback_counts")
            if isinstance(callback_counts, dict):
                self.state.set_bridge_callback_counts(callback_counts)
            self._set_connected(True)
            return

        if event_type == "scene":
            scan = self._points(event.get("scan"))
            polar = self._points(event.get("scan_polar"))
            self.state.set_scan(
                scan,
                polar,
                str(event.get("scan_frame", "")),
                bool(event.get("scan_transformed", False)),
            )
            current = self.state.snapshot_scene()
            path_updates: dict[str, list[list[float]]] = {}
            global_path = self._points(event.get("global_path"), limit=2000)
            local_path = self._points(event.get("local_path"), limit=2000)
            if global_path != current["global_path"]:
                path_updates["global_path"] = global_path
            if local_path != current["local_path"]:
                path_updates["local_path"] = local_path
            if path_updates:
                self.state.set_paths(**path_updates)
            self._set_connected(True)
            return

        if event_type == "event":
            self.state.add_event(
                str(event.get("level", "info")),
                str(event.get("message", "")),
            )
            self._set_connected(True)

    @staticmethod
    def _optional_float(value: Any) -> float | None:
        if value is None:
            return None
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None
        return number if math.isfinite(number) else None


def _public_config(config: dict[str, Any]) -> dict[str, Any]:
    return {
        "title": config["title"],
        "map_frame": config["map_frame"],
        "base_frame": config["base_frame"],
        "ws_hz": config["ws_hz"],
        "camera_stream_url": config["camera_stream_url"],
        "topics": config["topics"],
        "actions": config["actions"],
    }


def _resolve_camera_url(url: str, request: web.Request) -> str:
    if not url:
        return ""
    parsed = urlparse(url)
    if parsed.hostname not in ("127.0.0.1", "localhost"):
        return url
    host = request.host.split(":", 1)[0]
    port = parsed.port or 80
    return f"{parsed.scheme}://{host}:{port}{parsed.path}"


def create_app(
    config: dict[str, Any],
    state: StateStore,
    manager: ProcessManager,
    bridge: RosBridge | RosBridgeProcess | None,
    mock_source: MockSource | None,
    web_root: Path,
) -> web.Application:
    app = web.Application(client_max_size=1024 * 1024)
    app["config"] = config
    app["state"] = state
    app["manager"] = manager
    app["bridge"] = bridge
    app["mock_source"] = mock_source
    app["web_root"] = web_root

    async def index(_request: web.Request) -> web.FileResponse:
        return web.FileResponse(web_root / "index.html")

    async def health(_request: web.Request) -> web.Response:
        return web.json_response(
            {
                "ok": True,
                "ros_connected": state.snapshot_telemetry()["ros_connected"],
                "mock": mock_source is not None,
            }
        )

    async def api_config(request: web.Request) -> web.Response:
        payload = _public_config(config)
        payload["camera_stream_url"] = _resolve_camera_url(
            str(payload["camera_stream_url"]),
            request,
        )
        return web.json_response(payload)

    async def api_state(_request: web.Request) -> web.Response:
        return web.json_response(
            {
                "telemetry": state.snapshot_telemetry(),
                "scene": state.snapshot_scene(),
                "status": state.snapshot_status(),
            }
        )

    async def api_command(request: web.Request) -> web.Response:
        if request.content_type != "application/json":
            return web.json_response(
                {"ok": False, "error": "Content-Type must be application/json"},
                status=415,
            )
        origin = request.headers.get("Origin")
        if origin:
            origin_host = urlparse(origin).netloc
            if origin_host and origin_host != request.host:
                return web.json_response(
                    {"ok": False, "error": "cross-origin command rejected"},
                    status=403,
                )
        try:
            payload = await request.json()
        except Exception:
            return web.json_response({"ok": False, "error": "invalid JSON"}, status=400)
        action = str(payload.get("action", ""))
        if not action:
            return web.json_response({"ok": False, "error": "missing action"}, status=400)
        try:
            if action in ("goal", "initial_pose"):
                x = _finite_float(payload.get("x"), "x")
                y = _finite_float(payload.get("y"), "y")
                yaw = _finite_float(payload.get("yaw", 0.0), "yaw", limit=math.tau)
                if action == "goal" and payload.get("safety_confirmed") is not True:
                    return web.json_response(
                        {
                            "ok": False,
                            "error": "goal requires explicit safety confirmation",
                        },
                        status=409,
                    )
                if mock_source is not None:
                    if action == "goal":
                        mock_source.submit_goal(x, y, yaw)
                    else:
                        state.set_initial_pose(x, y, yaw)
                        state.add_event(
                            "info",
                            f"Mock 初始位姿 ({x:.2f}, {y:.2f}, "
                            f"{math.degrees(yaw):.1f}°)"
                            + ("，粗略协方差" if payload.get("coarse") else ""),
                        )
                elif bridge is not None:
                    bridge.submit(
                        {
                            "action": action,
                            "x": x,
                            "y": y,
                            "yaw": yaw,
                            "coarse": bool(payload.get("coarse", False)),
                        }
                    )
                else:
                    return web.json_response(
                        {"ok": False, "error": "ROS bridge unavailable"},
                        status=503,
                    )
                return web.json_response({"ok": True, "action": action})

            if action == "cancel":
                if mock_source is not None:
                    mock_source.cancel_goal()
                elif bridge is not None:
                    bridge.submit({"action": "cancel"})
                state.add_event("warn", "取消导航目标")
                return web.json_response({"ok": True, "action": action})

            if action == "teleop":
                if payload.get("safety_confirmed") is not True:
                    return web.json_response(
                        {
                            "ok": False,
                            "error": "teleop requires explicit safety confirmation",
                        },
                        status=409,
                    )
                if mock_source is None and not manager.is_running("mapping_start"):
                    return web.json_response(
                        {
                            "ok": False,
                            "error": "teleop is only allowed in mapping mode",
                        },
                        status=409,
                    )
                linear_x = _finite_float(
                    payload.get("linear_x", 0.0),
                    "linear_x",
                    limit=0.12,
                )
                angular_z = _finite_float(
                    payload.get("angular_z", 0.0),
                    "angular_z",
                    limit=0.45,
                )
                if linear_x < 0.0 or linear_x > 0.12:
                    raise ValueError("linear_x must be within 0.0..0.12 m/s")
                if angular_z < -0.45 or angular_z > 0.45:
                    raise ValueError("angular_z must be within -0.45..0.45 rad/s")
                if mock_source is not None:
                    state.set_speed(linear_x, angular_z)
                    state.add_event("info", "[mock] 收到遥控速度")
                elif bridge is not None:
                    bridge.submit(
                        {
                            "action": "teleop",
                            "linear_x": linear_x,
                            "angular_z": angular_z,
                        }
                    )
                return web.json_response({"ok": True, "action": action})

            if action in ("global_localize", "nomotion_update"):
                if mock_source is not None:
                    state.add_event("info", f"[mock] {action}")
                elif bridge is not None:
                    bridge.submit({"action": action})
                else:
                    return web.json_response(
                        {"ok": False, "error": "ROS bridge unavailable"},
                        status=503,
                    )
                return web.json_response({"ok": True, "action": action})

            if action == "mapping_start":
                if manager.is_running("navigation_start"):
                    if bridge is not None:
                        bridge.submit({"action": "cancel"})
                        bridge.submit({"action": "zero_velocity"})
                    manager.stop_process("navigation_start")
                    await asyncio.sleep(1.5)
                if bridge is not None:
                    bridge.submit({"action": "disable_navigation"})
                return web.json_response(manager.start("mapping_start"))

            if action == "navigation_start":
                if manager.is_running("mapping_start"):
                    if bridge is not None:
                        bridge.submit({"action": "zero_velocity"})
                    manager.stop_process("mapping_start")
                    await asyncio.sleep(1.5)
                result = manager.start("navigation_start")
                if bridge is not None:
                    bridge.submit({"action": "enable_navigation"})
                return web.json_response(result)

            if action == "mapping_stop":
                if bridge is not None:
                    bridge.submit({"action": "zero_velocity"})
                    await asyncio.sleep(0.15)
                return web.json_response(manager.signal_operation(action))

            if action == "navigation_stop":
                if bridge is not None:
                    bridge.submit({"action": "cancel"})
                    bridge.submit({"action": "zero_velocity"})
                    await asyncio.sleep(0.25)
                result = manager.signal_operation(action)
                return web.json_response(result)

            if action == "safe_stop":
                if bridge is not None:
                    bridge.submit({"action": "cancel"})
                    bridge.submit({"action": "zero_velocity"})
                    await asyncio.sleep(0.2)
                results = []
                if manager.is_running("mapping_start"):
                    results.append(manager.stop_process("mapping_start"))
                if manager.is_running("navigation_start"):
                    results.append(manager.stop_process("navigation_start"))
                state.add_event("warn", "执行安全停止")
                return web.json_response(
                    {"ok": True, "action": action, "results": results}
                )

            if action == "save_map":
                name = str(payload.get("name", "")).strip()
                return web.json_response(manager.save_map(name))

            return web.json_response(
                {"ok": False, "error": f"unsupported action: {action}"},
                status=400,
            )
        except (TypeError, ValueError) as exc:
            return web.json_response({"ok": False, "error": str(exc)}, status=400)
        except Exception as exc:  # noqa: BLE001
            state.add_event("warn", f"命令执行失败: {exc}")
            return web.json_response({"ok": False, "error": str(exc)}, status=500)

    async def websocket_handler(request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse(max_msg_size=0, heartbeat=10.0)
        await ws.prepare(request)
        config_payload = _public_config(config)
        config_payload["camera_stream_url"] = _resolve_camera_url(
            str(config_payload["camera_stream_url"]),
            request,
        )
        await ws.send_json({"type": "config", **config_payload})
        last_scene = -1
        last_status = -1
        last_map = -1
        interval = 1.0 / max(1.0, float(config["ws_hz"]))
        scene_interval = 1.0 / max(0.5, float(config["scene_hz"]))
        map_interval = 1.0 / max(0.1, float(config["map_hz"]))
        last_scene_sent = 0.0
        last_map_sent = 0.0
        pong_queue: asyncio.Queue[str] = asyncio.Queue()
        closed = asyncio.Event()

        async def receive_loop() -> None:
            try:
                async for message in ws:
                    if message.type == WSMsgType.TEXT:
                        if message.data.startswith("ping:"):
                            pong_queue.put_nowait(
                                "pong:" + message.data.removeprefix("ping:")
                            )
                        elif message.data == "ping":
                            pong_queue.put_nowait("pong")
                    elif message.type in (
                        WSMsgType.CLOSE,
                        WSMsgType.CLOSED,
                        WSMsgType.ERROR,
                    ):
                        break
            finally:
                closed.set()

        receiver = asyncio.create_task(receive_loop())
        next_tick = time.monotonic()
        try:
            while not ws.closed and not closed.is_set():
                next_tick += interval
                now = time.monotonic()
                telemetry = state.snapshot_telemetry()
                await ws.send_json(telemetry)
                status = state.snapshot_status()
                if status["revision"] != last_status:
                    await ws.send_json(status)
                    last_status = status["revision"]
                scene = state.snapshot_scene()
                if (
                    scene["revision"] != last_scene
                    and now - last_scene_sent >= scene_interval
                ):
                    await ws.send_json(scene)
                    last_scene = scene["revision"]
                    last_scene_sent = now
                packet, revision = state.get_map_packet()
                if (
                    packet is not None
                    and revision != last_map
                    and now - last_map_sent >= map_interval
                ):
                    await ws.send_bytes(packet)
                    last_map = revision
                    last_map_sent = now
                while not pong_queue.empty():
                    await ws.send_str(pong_queue.get_nowait())
                try:
                    await asyncio.wait_for(
                        closed.wait(),
                        timeout=max(0.0, next_tick - time.monotonic()),
                    )
                except asyncio.TimeoutError:
                    pass
        except (ConnectionResetError, RuntimeError):
            pass
        finally:
            receiver.cancel()
            await asyncio.gather(receiver, return_exceptions=True)
            await ws.close()
        return ws

    app.router.add_get("/", index)
    app.router.add_get("/healthz", health)
    app.router.add_get("/api/config", api_config)
    app.router.add_get("/api/state", api_state)
    app.router.add_post("/api/command", api_command)
    app.router.add_get("/ws", websocket_handler)
    app.router.add_static("/", web_root, show_index=False)
    return app


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8091)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--web-root", type=Path)
    parser.add_argument("--mock", action="store_true")
    parser.add_argument("--ros-domain-id", type=int, default=-1)
    return parser.parse_args()


async def run_server(args: argparse.Namespace) -> int:
    config = _load_config(args.config)
    if args.ros_domain_id >= 0:
        config["ros_domain_id"] = args.ros_domain_id
    web_root = (
        args.web_root.resolve()
        if args.web_root
        else Path(__file__).resolve().parent / "web"
    )
    if not (web_root / "index.html").is_file():
        raise FileNotFoundError(f"missing web root: {web_root}")
    state = StateStore(config)
    manager = ProcessManager(state, config, mock=args.mock)
    mock_source = MockSource(state) if args.mock else None
    bridge = None
    if mock_source is not None:
        mock_source.start()
    else:
        bridge = RosBridgeProcess(state, config)
        bridge.start()
    app = create_app(config, state, manager, bridge, mock_source, web_root)
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, args.host, args.port)
    await site.start()
    print(
        f"nav console listening on http://{args.host}:{args.port} "
        f"mock={args.mock} web_root={web_root}",
        flush=True,
    )
    stop_event = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, stop_event.set)
        except (NotImplementedError, RuntimeError):
            pass
    try:
        await stop_event.wait()
    except asyncio.CancelledError:
        pass
    finally:
        if mock_source is not None:
            mock_source.stop()
        if bridge is not None:
            bridge.stop()
        await runner.cleanup()
    return 0


def main() -> int:
    args = parse_args()
    try:
        return asyncio.run(run_server(args))
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())

