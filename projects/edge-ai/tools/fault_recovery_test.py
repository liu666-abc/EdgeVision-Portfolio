#!/usr/bin/env python3
"""Fault-injection test for detector and video-source recovery."""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import time
import urllib.request
from pathlib import Path
from typing import Any, Callable


ROOT = Path("/home/wheeltec/evi")
START_SCRIPT = ROOT / "evi_dashboard/board_agent/start_video_demo.sh"
PID_FILES = {
    "video_source": Path("/tmp/evi_video_source.pid"),
    "detector": Path("/tmp/evi_video_detector.pid"),
    "dashboard": Path("/tmp/evi_video_dashboard.pid"),
}
PATTERNS = {
    "video_source": "[v]ideo_source_node",
    "detector": "[d]etector_node",
}


def read_status(url: str) -> dict[str, Any]:
    with urllib.request.urlopen(url, timeout=3) as response:
        return json.loads(response.read().decode("utf-8"))


def wait_for(
    description: str,
    predicate: Callable[[dict[str, Any]], bool],
    url: str,
    timeout_s: float,
    interval_s: float = 0.5,
) -> tuple[bool, float, dict[str, Any] | None]:
    started = time.monotonic()
    latest = None
    while time.monotonic() - started < timeout_s:
        try:
            latest = read_status(url)
            if predicate(latest):
                return True, time.monotonic() - started, latest
        except OSError:
            pass
        time.sleep(interval_s)
    return False, time.monotonic() - started, latest


def kill_component(name: str) -> None:
    pid_file = PID_FILES[name]
    if pid_file.exists():
        try:
            os.kill(int(pid_file.read_text(encoding="utf-8").strip()), signal.SIGINT)
        except (OSError, ValueError):
            pass
    subprocess.run(
        ["pkill", "-INT", "-f", PATTERNS[name]],
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    for _ in range(20):
        output = subprocess.run(
            ["pgrep", "-f", PATTERNS[name]],
            capture_output=True,
            text=True,
            check=False,
        ).stdout.strip()
        if not output:
            break
        time.sleep(0.25)
    pid_file.unlink(missing_ok=True)


def start_demo(video_file: str | None, fps: float | None) -> None:
    environment = os.environ.copy()
    environment["EVI_ROS_DOMAIN_ID"] = "59"
    environment["EVI_ROS_LOCALHOST_ONLY"] = "1"
    environment["EVI_DASHBOARD_PORT"] = "8080"
    if video_file:
        environment["EVI_VIDEO_FILE"] = video_file
    if fps is not None:
        environment["EVI_VIDEO_FPS"] = str(fps)
    result = subprocess.run(
        ["bash", str(START_SCRIPT)],
        env=environment,
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(
            f"start_video_demo failed rc={result.returncode}\n"
            f"stdout={result.stdout}\nstderr={result.stderr}"
        )


def record(
    results: list[dict[str, Any]],
    step: str,
    passed: bool,
    elapsed_s: float,
    detail: str,
) -> None:
    results.append(
        {
            "step": step,
            "status": "PASS" if passed else "FAIL",
            "elapsed_s": round(elapsed_s, 3),
            "detail": detail,
        }
    )
    print(
        f"{'PASS' if passed else 'FAIL'} {step} {elapsed_s:.2f}s {detail}",
        flush=True,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="http://127.0.0.1:8080/api/status")
    parser.add_argument("--video-file")
    parser.add_argument("--fps", type=float)
    parser.add_argument("--timeout", type=float, default=12.0)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    results: list[dict[str, Any]] = []
    started = time.monotonic()
    try:
        start_demo(args.video_file, args.fps)
        ok, elapsed, status = wait_for(
            "initial_online",
            lambda s: s["camera"]["online"]
            and s["detector"]["online"]
            and s["tracker"]["active_tracks"] > 0,
            args.url,
            args.timeout,
        )
        record(results, "initial_online", ok, elapsed, "camera+detector+tracks")
        if not ok:
            raise RuntimeError("initial pipeline did not become online")

        fault_started = time.monotonic()
        kill_component("detector")
        ok, elapsed, status = wait_for(
            "detector_offline_detected",
            lambda s: s["camera"]["online"] and not s["detector"]["online"],
            args.url,
            args.timeout,
        )
        elapsed = time.monotonic() - fault_started
        record(
            results,
            "detector_offline_detected",
            ok,
            elapsed,
            "camera online, detector offline",
        )

        recovery_started = time.monotonic()
        start_demo(args.video_file, args.fps)
        ok, elapsed, status = wait_for(
            "detector_recovered",
            lambda s: s["camera"]["online"]
            and s["detector"]["online"]
            and s["tracker"]["active_tracks"] > 0,
            args.url,
            args.timeout,
        )
        elapsed = time.monotonic() - recovery_started
        record(results, "detector_recovered", ok, elapsed, "detections resumed")

        fault_started = time.monotonic()
        kill_component("video_source")
        ok, elapsed, status = wait_for(
            "video_loss_detected",
            lambda s: not s["camera"]["online"] and not s["detector"]["online"],
            args.url,
            args.timeout,
        )
        elapsed = time.monotonic() - fault_started
        record(
            results,
            "video_loss_detected",
            ok,
            elapsed,
            "camera and detector offline",
        )

        recovery_started = time.monotonic()
        start_demo(args.video_file, args.fps)
        ok, elapsed, status = wait_for(
            "video_recovered",
            lambda s: s["camera"]["online"]
            and s["detector"]["online"]
            and s["tracker"]["active_tracks"] > 0,
            args.url,
            args.timeout,
        )
        elapsed = time.monotonic() - recovery_started
        record(results, "video_recovered", ok, elapsed, "pipeline resumed")
    except Exception as exc:  # noqa: BLE001
        record(results, "exception", False, 0.0, str(exc))
    finally:
        try:
            start_demo(args.video_file, args.fps)
            restore_ok, restore_elapsed, _ = wait_for(
                "final_online",
                lambda s: s["camera"]["online"] and s["detector"]["online"],
                args.url,
                args.timeout,
            )
            record(
                results,
                "final_restore",
                restore_ok,
                restore_elapsed,
                "camera and detector online",
            )
        except Exception as exc:  # noqa: BLE001
            record(results, "final_restore", False, 0.0, str(exc))

    passed = all(item["status"] == "PASS" for item in results)
    output = {
        "status": "PASS" if passed else "FAIL",
        "duration_s": round(time.monotonic() - started, 3),
        "video_file": args.video_file,
        "fps": args.fps,
        "results": results,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(output, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    print(json.dumps(output, ensure_ascii=False, indent=2), flush=True)
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())

