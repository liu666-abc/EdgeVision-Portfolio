#!/usr/bin/env python3
"""Sample camera/detection rates and resources for two ROS2 streams."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import statistics
import subprocess
import time
from collections import deque
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from evi_cpp_detector.msg import DetectionArray


FIELDS = [
    "elapsed_s",
    "camera_a_fps",
    "camera_b_fps",
    "detector_a_fps",
    "detector_b_fps",
    "detector_b_over_a",
    "preprocess_a_ms",
    "preprocess_b_ms",
    "infer_a_ms",
    "infer_b_ms",
    "decode_a_ms",
    "decode_b_ms",
    "total_a_ms",
    "total_b_ms",
    "tracks_a",
    "tracks_b",
    "cpu_ratio",
    "bpu_ratio",
    "cpu_temp_c",
    "bpu_temp_c",
    "ddr_temp_c",
    "bpu_freq_mhz",
    "ddr_freq_mhz",
    "mem_used_mb",
]


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    return ordered[max(0, min(len(ordered) - 1, math.ceil(fraction * len(ordered)) - 1))]


def stats(values: list[float]) -> dict[str, float | None]:
    if not values:
        return {"min": None, "mean": None, "p50": None, "p95": None, "max": None}
    return {
        "min": min(values),
        "mean": sum(values) / len(values),
        "p50": statistics.median(values),
        "p95": percentile(values, 0.95),
        "max": max(values),
    }


def read_cpu_times() -> tuple[int, int]:
    fields = Path("/proc/stat").read_text(encoding="utf-8").splitlines()[0].split()
    values = [int(value) for value in fields[1:]]
    idle = values[3] + (values[4] if len(values) > 4 else 0)
    return sum(values), idle


def read_mem_used_mb() -> int:
    values = {}
    for line in Path("/proc/meminfo").read_text(encoding="utf-8").splitlines():
        key, raw = line.split(":", 1)
        values[key] = int(raw.strip().split()[0])
    return (values.get("MemTotal", 0) - values.get("MemAvailable", 0)) // 1024


def read_system_status() -> dict[str, float | int | None]:
    result = {
        "bpu_ratio": None,
        "cpu_temp_c": None,
        "bpu_temp_c": None,
        "ddr_temp_c": None,
        "bpu_freq_mhz": None,
        "ddr_freq_mhz": None,
        "mem_used_mb": read_mem_used_mb(),
    }
    try:
        process = subprocess.run(
            ["hrut_somstatus", "-n", "1"],
            capture_output=True,
            text=True,
            timeout=4,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return result
    text = process.stdout + process.stderr
    import re

    for key, label in (
        ("ddr_temp_c", "DDR"),
        ("bpu_temp_c", "BPU"),
        ("cpu_temp_c", "CPU"),
    ):
        match = re.search(rf"{label}\s*:\s*([0-9.]+)", text)
        if match:
            result[key] = float(match.group(1))
    match = re.search(r"bpu0:\s*(\d+)\s+(\d+)\s+(\d+)\s+(\d+)", text)
    if match:
        result["bpu_freq_mhz"] = int(match.group(2))
        result["bpu_ratio"] = float(match.group(4))
    match = re.search(r"ddr:\s+(\d+)\s+(\d+)\s+(\d+)", text)
    if match:
        result["ddr_freq_mhz"] = int(match.group(2))
    return result


class DualStreamMonitor(Node):
    def __init__(self) -> None:
        super().__init__("dual_stream_monitor")
        self.camera_a_times: deque[float] = deque(maxlen=5000)
        self.camera_b_times: deque[float] = deque(maxlen=5000)
        self.detector_a_times: deque[float] = deque(maxlen=5000)
        self.detector_b_times: deque[float] = deque(maxlen=5000)
        self.detector_a: dict[str, float | int] = {}
        self.detector_b: dict[str, float | int] = {}
        self.create_subscription(
            Image,
            "/camera_a/color/image_raw",
            lambda _msg: self.camera_a_times.append(time.monotonic()),
            10,
        )
        self.create_subscription(
            Image,
            "/camera_b/color/image_raw",
            lambda _msg: self.camera_b_times.append(time.monotonic()),
            10,
        )
        self.create_subscription(
            DetectionArray,
            "/evi/detections_a",
            lambda msg: self.on_detection(msg, "a"),
            10,
        )
        self.create_subscription(
            DetectionArray,
            "/evi/detections_b",
            lambda msg: self.on_detection(msg, "b"),
            10,
        )

    def on_detection(self, message: DetectionArray, side: str) -> None:
        now = time.monotonic()
        target_times = self.detector_a_times if side == "a" else self.detector_b_times
        target_state = self.detector_a if side == "a" else self.detector_b
        target_times.append(now)
        target_state.update(
            {
                "preprocess_ms": float(message.preprocess_ms),
                "infer_ms": float(message.infer_ms),
                "decode_ms": float(message.decode_ms),
                "total_ms": float(message.total_ms),
                "tracks": sum(
                    1 for item in message.detections if int(item.track_id) > 0
                ),
            }
        )

    @staticmethod
    def rate(timestamps: deque[float], now: float, window_s: float = 2.0) -> float:
        recent = [value for value in timestamps if now - value <= window_s]
        if len(recent) < 2:
            return 0.0
        span = recent[-1] - recent[0]
        return (len(recent) - 1) / span if span > 0 else 0.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--minutes", type=float, default=10.0)
    parser.add_argument("--interval", type=float, default=5.0)
    parser.add_argument("--warmup-s", type=float, default=5.0)
    parser.add_argument("--max-bpu-temp", type=float, default=65.0)
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--summary", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.csv.parent.mkdir(parents=True, exist_ok=True)
    args.summary.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = DualStreamMonitor()
    rows: list[dict[str, float | int | None]] = []
    started = time.monotonic()
    next_sample = started
    previous_cpu = read_cpu_times()
    stop_reason = "completed"
    with args.csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        writer.writeheader()
        while time.monotonic() - started < args.minutes * 60.0:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if now < next_sample:
                continue
            next_sample += args.interval
            cpu_now = read_cpu_times()
            total_delta = cpu_now[0] - previous_cpu[0]
            idle_delta = cpu_now[1] - previous_cpu[1]
            cpu_ratio = (
                100.0 * (total_delta - idle_delta) / total_delta
                if total_delta > 0
                else None
            )
            previous_cpu = cpu_now
            system = read_system_status()
            rate_a = node.rate(node.detector_a_times, now)
            rate_b = node.rate(node.detector_b_times, now)
            row = {
                "elapsed_s": round(now - started, 3),
                "camera_a_fps": round(node.rate(node.camera_a_times, now), 3),
                "camera_b_fps": round(node.rate(node.camera_b_times, now), 3),
                "detector_a_fps": round(rate_a, 3),
                "detector_b_fps": round(rate_b, 3),
                "detector_b_over_a": round(rate_b / rate_a, 4) if rate_a else None,
                "preprocess_a_ms": node.detector_a.get("preprocess_ms"),
                "preprocess_b_ms": node.detector_b.get("preprocess_ms"),
                "infer_a_ms": node.detector_a.get("infer_ms"),
                "infer_b_ms": node.detector_b.get("infer_ms"),
                "decode_a_ms": node.detector_a.get("decode_ms"),
                "decode_b_ms": node.detector_b.get("decode_ms"),
                "total_a_ms": node.detector_a.get("total_ms"),
                "total_b_ms": node.detector_b.get("total_ms"),
                "tracks_a": node.detector_a.get("tracks", 0),
                "tracks_b": node.detector_b.get("tracks", 0),
                "cpu_ratio": round(cpu_ratio, 2) if cpu_ratio is not None else None,
                **system,
            }
            rows.append(row)
            writer.writerow(row)
            handle.flush()
            print(
                f"t={row['elapsed_s']:7.1f}s "
                f"camA={row['camera_a_fps']} camB={row['camera_b_fps']} "
                f"detA={row['detector_a_fps']} detB={row['detector_b_fps']} "
                f"cpu={row['cpu_ratio']}% bpu={row['bpu_ratio']}% "
                f"temp={row['cpu_temp_c']}/{row['bpu_temp_c']}/{row['ddr_temp_c']}",
                flush=True,
            )
            temp = row.get("bpu_temp_c")
            if isinstance(temp, (int, float)) and temp >= args.max_bpu_temp:
                stop_reason = "thermal_limit"
                break
    node.destroy_node()
    rclpy.shutdown()

    steady_rows = [
        row for row in rows if float(row["elapsed_s"]) >= args.warmup_s
    ]
    metrics = {}
    for field in (
        "camera_a_fps",
        "camera_b_fps",
        "detector_a_fps",
        "detector_b_fps",
        "detector_b_over_a",
        "total_a_ms",
        "total_b_ms",
        "cpu_ratio",
        "bpu_ratio",
        "cpu_temp_c",
        "bpu_temp_c",
        "ddr_temp_c",
        "mem_used_mb",
    ):
        metrics[field] = stats(
            [
                float(row[field])
                for row in steady_rows
                if isinstance(row.get(field), (int, float))
            ]
        )
    summary = {
        "stop_reason": stop_reason,
        "samples": len(rows),
        "steady_samples": len(steady_rows),
        "warmup_s": args.warmup_s,
        "actual_duration_s": rows[-1]["elapsed_s"] if rows else 0.0,
        "metrics": metrics,
    }
    args.summary.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
    return 0 if stop_reason == "completed" else 2


if __name__ == "__main__":
    main()

