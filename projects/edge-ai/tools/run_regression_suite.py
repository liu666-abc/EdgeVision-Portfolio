#!/usr/bin/env python3
"""Run the repeatable EdgeVision Lab regression suite on the RDK."""

from __future__ import annotations

import argparse
import csv
import json
import os
import subprocess
import time
import urllib.request
from pathlib import Path
from typing import Any


TROS_SETUP = "/opt/tros/humble/setup.bash"
WORKSPACE = Path("/home/wheeltec/evi/upper_computer_ws")
EVi_ROOT = Path("/home/wheeltec/evi/evi_dashboard")
START_SCRIPT = EVi_ROOT / "board_agent/start_video_demo.sh"
MODEL = "/opt/hobot/model/x5/basic/yolov8_640x640_nv12.bin"
STATIC_IMAGES = Path("/home/wheeltec/evi/cpp_pipeline_ws/images_test")
EXPECTED_COUNTS = [6, 1, 4, 1, 3, 2, 2, 2, 4, 10]


def shell(command: str, timeout: float, workdir: Path | None = None) -> subprocess.CompletedProcess:
    return subprocess.run(
        ["bash", "-lc", command],
        cwd=str(workdir) if workdir else None,
        capture_output=True,
        text=True,
        timeout=timeout,
        check=False,
    )


def add_result(
    results: list[dict[str, Any]],
    name: str,
    passed: bool,
    elapsed: float,
    detail: str,
    log_path: Path | None = None,
    artifacts: list[str] | None = None,
) -> None:
    results.append(
        {
            "name": name,
            "status": "PASS" if passed else "FAIL",
            "elapsed_s": round(elapsed, 3),
            "detail": detail,
            "log": str(log_path) if log_path else None,
            "artifacts": artifacts or [],
        }
    )
    print(f"{'PASS' if passed else 'FAIL'} {name} {elapsed:.2f}s {detail}", flush=True)


def run_tracker_selftest(output_dir: Path) -> tuple[bool, float, str, Path]:
    log_path = output_dir / "tracker_selftest.log"
    command = (
        f"source {TROS_SETUP} && "
        f"source {WORKSPACE}/install/setup.bash && "
        "ros2 run evi_cpp_detector tracker_selftest"
    )
    started = time.monotonic()
    result = shell(command, 30)
    elapsed = time.monotonic() - started
    log_path.write_text(result.stdout + result.stderr, encoding="utf-8")
    passed = result.returncode == 0 and "TRACKER_SELFTEST_OK" in result.stdout
    return passed, elapsed, result.stdout.strip(), log_path


def run_evaluator_selftest(output_dir: Path) -> tuple[bool, float, str, Path]:
    log_path = output_dir / "evaluator_selftest.log"
    command = (
        f"python3 {EVi_ROOT}/tools/evaluate_mot.py "
        f"--gt {EVi_ROOT}/assets/mot17-mini/gt.txt "
        f"--tracks {EVi_ROOT}/assets/mot17-mini/gt.txt --selftest"
    )
    started = time.monotonic()
    result = shell(command, 20)
    elapsed = time.monotonic() - started
    log_path.write_text(result.stdout + result.stderr, encoding="utf-8")
    passed = result.returncode == 0 and "EVALUATOR_SELFTEST_OK" in result.stdout
    return passed, elapsed, result.stdout.strip(), log_path


def run_static_detection_regression(output_dir: Path) -> tuple[bool, float, str, Path]:
    log_path = output_dir / "static_detector.log"
    csv_path = output_dir / "static_detector.csv"
    command = (
        f"source {TROS_SETUP} && "
        f"source {WORKSPACE}/install/setup.bash && "
        "export ROS_DOMAIN_ID=61 ROS_LOCALHOST_ONLY=1 && "
        "ros2 run evi_cpp_detector detector_node --ros-args "
        "-p mode:=static "
        f"-p model:={MODEL} "
        f"-p image_dir:={STATIC_IMAGES} "
        "-p confidence:=0.25 -p iou:=0.45 -p top_k:=300 "
        "-p period_s:=0.001 -p loop:=false "
        f"-p csv_path:={csv_path}"
    )
    started = time.monotonic()
    result = shell(command, 60)
    elapsed = time.monotonic() - started
    log_path.write_text(result.stdout + result.stderr, encoding="utf-8")
    counts = []
    if csv_path.exists():
        with csv_path.open("r", newline="", encoding="utf-8") as handle:
            counts = [int(row["detections"]) for row in csv.DictReader(handle)]
    passed = result.returncode == 0 and counts == EXPECTED_COUNTS
    detail = f"counts={counts} expected={EXPECTED_COUNTS}"
    return passed, elapsed, detail, log_path


def start_live_demo() -> None:
    environment = os.environ.copy()
    environment.update(
        {
            "EVI_ROS_DOMAIN_ID": "59",
            "EVI_ROS_LOCALHOST_ONLY": "1",
            "EVI_DASHBOARD_PORT": "8080",
            "EVI_VIDEO_FILE": "/home/wheeltec/evi/evi_dashboard/assets/vtest.avi",
            "EVI_VIDEO_FPS": "25.0",
        }
    )
    subprocess.run(
        ["bash", str(START_SCRIPT)],
        env=environment,
        capture_output=True,
        text=True,
        check=False,
    )


def run_live_contract(output_dir: Path, timeout_s: float) -> tuple[bool, float, str, Path]:
    log_path = output_dir / "live_contract.json"
    start_live_demo()
    started = time.monotonic()
    latest = None
    deadline = started + timeout_s
    while time.monotonic() < deadline:
        try:
            with urllib.request.urlopen(
                "http://127.0.0.1:8080/api/status",
                timeout=3,
            ) as response:
                latest = json.loads(response.read().decode("utf-8"))
            if (
                latest.get("camera", {}).get("online")
                and latest.get("detector", {}).get("online")
                and latest.get("tracker", {}).get("active_tracks", 0) > 0
                and latest.get("system", {}).get("cpu_ratio") is not None
            ):
                break
        except (OSError, ValueError):
            pass
        time.sleep(0.5)
    elapsed = time.monotonic() - started
    if latest is None:
        latest = {}
    log_path.write_text(
        json.dumps(latest, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    passed = (
        latest.get("camera", {}).get("online", False)
        and latest.get("detector", {}).get("online", False)
        and latest.get("tracker", {}).get("active_tracks", 0) > 0
        and latest.get("system", {}).get("cpu_ratio") is not None
    )
    detail = (
        f"camera={latest.get('camera', {}).get('online')} "
        f"detector={latest.get('detector', {}).get('online')} "
        f"tracks={latest.get('tracker', {}).get('active_tracks')} "
        f"cpu={latest.get('system', {}).get('cpu_ratio')}"
    )
    return passed, elapsed, detail, log_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--live-timeout", type=float, default=20.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    stamp = time.strftime("%Y%m%d_%H%M%S")
    output_dir = args.output_dir or Path(f"/tmp/evi_regression_{stamp}")
    output_dir.mkdir(parents=True, exist_ok=True)
    results: list[dict[str, Any]] = []
    overall_started = time.monotonic()

    checks = (
        ("tracker_selftest", run_tracker_selftest),
        ("evaluator_selftest", run_evaluator_selftest),
        ("static_detection_regression", run_static_detection_regression),
    )
    for name, function in checks:
        try:
            passed, elapsed, detail, log_path = function(output_dir)
        except Exception as exc:  # noqa: BLE001
            passed, elapsed, detail, log_path = False, 0.0, str(exc), None
        add_result(results, name, passed, elapsed, detail, log_path)

    try:
        passed, elapsed, detail, log_path = run_live_contract(
            output_dir,
            args.live_timeout,
        )
    except Exception as exc:  # noqa: BLE001
        passed, elapsed, detail, log_path = False, 0.0, str(exc), None
    add_result(results, "live_dashboard_contract", passed, elapsed, detail, log_path)

    summary = {
        "status": "PASS" if all(item["status"] == "PASS" for item in results) else "FAIL",
        "duration_s": round(time.monotonic() - overall_started, 3),
        "output_dir": str(output_dir),
        "results": results,
    }
    summary_path = output_dir / "summary.json"
    summary_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
    return 0 if summary["status"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())

