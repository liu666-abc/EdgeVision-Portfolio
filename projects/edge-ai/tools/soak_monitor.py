#!/usr/bin/env python3
"""Monitor the live dashboard for a sustained pipeline soak test."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import signal
import statistics
import subprocess
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any


FIELDS = [
    "elapsed_s",
    "camera_online",
    "camera_fps",
    "detector_online",
    "detector_fps",
    "preprocess_ms",
    "infer_ms",
    "decode_ms",
    "total_ms",
    "detections",
    "active_tracks",
    "ddr_temp_c",
    "bpu_temp_c",
    "cpu_temp_c",
    "cpu_ratio",
    "bpu_ratio",
    "bpu_freq_mhz",
    "ddr_freq_mhz",
    "mem_used_mb",
    "uptime_s",
]


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, math.ceil(fraction * len(ordered)) - 1))
    return ordered[index]


def stop_demo() -> None:
    for name in ("dashboard", "detector", "video_source"):
        pid_file = Path(f"/tmp/evi_video_{name}.pid")
        try:
            pid = int(pid_file.read_text(encoding="utf-8").strip())
            os.kill(pid, signal.SIGINT)
        except (OSError, ValueError):
            pass
    for pattern in ("[d]etector_node", "[v]ideo_source_node"):
        subprocess.run(
            ["pkill", "-INT", "-f", pattern],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )


def flatten(status: dict[str, Any], elapsed: float) -> dict[str, Any]:
    camera = status.get("camera", {})
    detector = status.get("detector", {})
    tracker = status.get("tracker", {})
    system = status.get("system", {})
    return {
        "elapsed_s": round(elapsed, 3),
        "camera_online": bool(camera.get("online")),
        "camera_fps": camera.get("fps"),
        "detector_online": bool(detector.get("online")),
        "detector_fps": detector.get("fps"),
        "preprocess_ms": detector.get("preprocess_ms"),
        "infer_ms": detector.get("infer_ms"),
        "decode_ms": detector.get("decode_ms"),
        "total_ms": detector.get("total_ms"),
        "detections": detector.get("count"),
        "active_tracks": tracker.get("active_tracks"),
        "ddr_temp_c": system.get("ddr_temp_c"),
        "bpu_temp_c": system.get("bpu_temp_c"),
        "cpu_temp_c": system.get("cpu_temp_c"),
        "cpu_ratio": system.get("cpu_ratio"),
        "bpu_ratio": system.get("bpu_ratio"),
        "bpu_freq_mhz": system.get("bpu_freq_mhz"),
        "ddr_freq_mhz": system.get("ddr_freq_mhz"),
        "mem_used_mb": system.get("mem_used_mb"),
        "uptime_s": system.get("uptime_s"),
    }


def numeric_summary(rows: list[dict[str, Any]], field: str) -> dict[str, float | None]:
    values = [
        float(row[field])
        for row in rows
        if row.get(field) is not None and isinstance(row[field], (int, float))
    ]
    if not values:
        return {"min": None, "mean": None, "p50": None, "p95": None, "max": None}
    return {
        "min": min(values),
        "mean": sum(values) / len(values),
        "p50": statistics.median(values),
        "p95": percentile(values, 0.95),
        "max": max(values),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="http://127.0.0.1:8080/api/status")
    parser.add_argument("--minutes", type=float, default=30.0)
    parser.add_argument("--interval", type=float, default=5.0)
    parser.add_argument("--max-bpu-temp", type=float, default=65.0)
    parser.add_argument("--max-api-misses", type=int, default=3)
    parser.add_argument("--frame-budget-ms", type=float, default=40.0)
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--summary", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.csv.parent.mkdir(parents=True, exist_ok=True)
    args.summary.parent.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    duration = args.minutes * 60.0
    rows: list[dict[str, Any]] = []
    api_misses = 0
    consecutive_misses = 0
    stop_reason = "completed"

    with args.csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        writer.writeheader()
        while True:
            elapsed = time.monotonic() - started
            if elapsed >= duration:
                break
            try:
                with urllib.request.urlopen(args.url, timeout=3) as response:
                    status = json.loads(response.read().decode("utf-8"))
                consecutive_misses = 0
            except (OSError, ValueError, urllib.error.URLError):
                api_misses += 1
                consecutive_misses += 1
                if consecutive_misses >= args.max_api_misses:
                    stop_reason = "api_unavailable"
                    break
                time.sleep(args.interval)
                continue

            row = flatten(status, elapsed)
            rows.append(row)
            writer.writerow(row)
            handle.flush()
            print(
                f"t={row['elapsed_s']:7.1f}s "
                f"cam={row['camera_fps']} det={row['detector_fps']} "
                f"total={row['total_ms']}ms "
                f"temp={row['cpu_temp_c']}/{row['bpu_temp_c']}/{row['ddr_temp_c']} "
                f"cpu={row['cpu_ratio']}% bpu={row['bpu_ratio']}% "
                f"mem={row['mem_used_mb']}MB",
                flush=True,
            )
            bpu_temp = row.get("bpu_temp_c")
            if isinstance(bpu_temp, (int, float)) and bpu_temp >= args.max_bpu_temp:
                stop_reason = "thermal_limit"
                break
            time.sleep(args.interval)

    if stop_reason != "completed":
        stop_demo()

    metrics = {}
    for field in FIELDS:
        if field in (
            "elapsed_s",
            "camera_online",
            "detector_online",
            "camera_fps",
            "detector_fps",
            "preprocess_ms",
            "infer_ms",
            "decode_ms",
            "total_ms",
            "detections",
            "active_tracks",
            "ddr_temp_c",
            "bpu_temp_c",
            "cpu_temp_c",
            "cpu_ratio",
            "bpu_ratio",
            "bpu_freq_mhz",
            "ddr_freq_mhz",
            "mem_used_mb",
        ):
            metrics[field] = numeric_summary(rows, field)

    camera_online_ratio = (
        sum(bool(row["camera_online"]) for row in rows) / len(rows)
        if rows
        else 0.0
    )
    detector_online_ratio = (
        sum(bool(row["detector_online"]) for row in rows) / len(rows)
        if rows
        else 0.0
    )
    memory_start = rows[0]["mem_used_mb"] if rows else None
    memory_end = rows[-1]["mem_used_mb"] if rows else None
    total_latency_values = [
        float(row["total_ms"])
        for row in rows
        if isinstance(row.get("total_ms"), (int, float))
    ]
    deadline_misses = sum(
        1
        for value in total_latency_values
        if value > args.frame_budget_ms
    )
    summary = {
        "stop_reason": stop_reason,
        "requested_minutes": args.minutes,
        "interval_s": args.interval,
        "samples": len(rows),
        "actual_duration_s": rows[-1]["elapsed_s"] if rows else 0.0,
        "api_misses": api_misses,
        "camera_online_ratio": camera_online_ratio,
        "detector_online_ratio": detector_online_ratio,
        "memory_start_mb": memory_start,
        "memory_end_mb": memory_end,
        "memory_delta_mb": (
            memory_end - memory_start
            if memory_start is not None and memory_end is not None
            else None
        ),
        "frame_budget_ms": args.frame_budget_ms,
        "deadline_missed_samples": deadline_misses,
        "deadline_miss_rate": (
            deadline_misses / len(total_latency_values)
            if total_latency_values
            else 0.0
        ),
        "metrics": metrics,
    }
    args.summary.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
    return 0 if stop_reason == "completed" else 2


if __name__ == "__main__":
    raise SystemExit(main())

