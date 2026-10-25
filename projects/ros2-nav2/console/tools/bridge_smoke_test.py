#!/usr/bin/env python3
"""Run the standalone ROS bridge briefly and summarize its JSON events."""

from __future__ import annotations

import argparse
import json
import os
import queue
import subprocess
import sys
import threading
import time
from collections import Counter
from pathlib import Path
from typing import Any


def read_stream(stream: Any, target: queue.Queue[tuple[str, str]]) -> None:
    for line in iter(stream.readline, ""):
        target.put(("ok", line.rstrip("\r\n")))
    target.put(("eof", ""))


def summarize_event(
    event: dict[str, Any],
    counts: Counter[str],
    summary: dict[str, Any],
) -> None:
    event_type = str(event.get("type", "unknown"))
    counts[event_type] += 1
    if event_type == "map":
        summary["map_meta"] = event.get("meta")
        summary["map_payload_chars"] = len(str(event.get("data", "")))
    elif event_type == "telemetry":
        summary["telemetry"] = {
            "pose": event.get("pose"),
            "speed": event.get("speed"),
            "map_age_ms": event.get("map_age_ms"),
            "scan_age_ms": event.get("scan_age_ms"),
            "amcl_age_ms": event.get("amcl_age_ms"),
            "nav_mode": (event.get("nav") or {}).get("mode"),
            "publishers": event.get("publishers"),
            "callback_counts": event.get("callback_counts"),
            "resubscribe_count": event.get("resubscribe_count"),
            "tf_frames": event.get("tf_frames"),
        }
    elif event_type == "scene":
        summary["scene"] = {
            "revision": event.get("revision"),
            "scan_points": len(event.get("scan") or []),
            "raw_scan_points": len(event.get("scan_polar") or []),
            "scan_frame": event.get("scan_frame"),
            "scan_transformed": event.get("scan_transformed"),
            "global_path_points": len(event.get("global_path") or []),
            "local_path_points": len(event.get("local_path") or []),
        }


def main() -> int:
    here = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        type=Path,
        default=here.parent / "config.example.json",
    )
    parser.add_argument(
        "--script",
        type=Path,
        default=here.parent / "ros_bridge_process.py",
    )
    parser.add_argument("--seconds", type=float, default=12.0)
    parser.add_argument("--python", default=sys.executable)
    parser.add_argument("--spawn-from-thread", action="store_true")
    args = parser.parse_args()

    config = json.loads(args.config.read_text(encoding="utf-8"))
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(config.get("ros_domain_id", 0))
    env["ROS_LOCALHOST_ONLY"] = "1"
    command = [
        args.python,
        str(args.script),
        json.dumps(config, separators=(",", ":")),
    ]
    launch_options = {
        "stdin": subprocess.PIPE,
        "stdout": subprocess.PIPE,
        "stderr": subprocess.PIPE,
        "text": True,
        "encoding": "utf-8",
        "errors": "replace",
        "bufsize": 1,
        "env": env,
    }
    if args.spawn_from_thread:
        launched: queue.Queue[subprocess.Popen[str]] = queue.Queue()

        def launch() -> None:
            launched.put(subprocess.Popen(command, **launch_options))

        launcher = threading.Thread(target=launch)
        launcher.start()
        launcher.join()
        process = launched.get()
    else:
        process = subprocess.Popen(command, **launch_options)
    assert process.stdout is not None
    assert process.stderr is not None
    assert process.stdin is not None

    stdout_lines: queue.Queue[tuple[str, str]] = queue.Queue()
    stderr_lines: queue.Queue[tuple[str, str]] = queue.Queue()
    threading.Thread(
        target=read_stream,
        args=(process.stdout, stdout_lines),
        daemon=True,
    ).start()
    threading.Thread(
        target=read_stream,
        args=(process.stderr, stderr_lines),
        daemon=True,
    ).start()

    counts: Counter[str] = Counter()
    summary: dict[str, Any] = {}
    errors: list[str] = []
    invalid_json = 0
    deadline = time.monotonic() + max(1.0, args.seconds)

    while time.monotonic() < deadline and process.poll() is None:
        try:
            _, line = stdout_lines.get(timeout=0.1)
        except queue.Empty:
            continue
        if not line:
            continue
        try:
            event = json.loads(line)
        except json.JSONDecodeError:
            invalid_json += 1
            continue
        if isinstance(event, dict):
            summarize_event(event, counts, summary)
        while True:
            try:
                _, error_line = stderr_lines.get_nowait()
            except queue.Empty:
                break
            if error_line:
                errors.append(error_line)
                del errors[:-20]

    try:
        process.stdin.write('{"action":"shutdown"}\n')
        process.stdin.flush()
    except (BrokenPipeError, OSError):
        pass
    try:
        process.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2.0)

    while True:
        try:
            _, error_line = stderr_lines.get_nowait()
        except queue.Empty:
            break
        if error_line:
            errors.append(error_line)
            del errors[:-20]

    result = {
        "exit_code": process.returncode,
        "duration_s": round(args.seconds, 1),
        "event_counts": dict(sorted(counts.items())),
        "invalid_json_lines": invalid_json,
        "map_received": counts["map"] > 0,
        "scan_received": counts["scene"] > 0
        and int((summary.get("scene") or {}).get("raw_scan_points", 0)) > 0,
        "latest": summary,
        "stderr": errors,
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["map_received"] and result["scan_received"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
