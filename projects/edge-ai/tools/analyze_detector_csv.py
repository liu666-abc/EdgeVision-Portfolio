#!/usr/bin/env python3
"""Summarize detector_node per-frame CSV files."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    return ordered[max(0, min(len(ordered) - 1, math.ceil(fraction * len(ordered)) - 1))]


def summarize(path: Path) -> dict:
    with path.open("r", newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    total = [float(row["total_ms"]) for row in rows]
    infer = [float(row["infer_ms"]) for row in rows]
    preprocess = [float(row["preprocess_ms"]) for row in rows]
    decode = [float(row["decode_ms"]) for row in rows]
    mean_total = sum(total) / len(total) if total else 0.0
    return {
        "path": str(path),
        "frames": len(rows),
        "latency_implied_fps": 1000.0 / mean_total if mean_total else 0.0,
        "total_ms": {
            "mean": mean_total,
            "p50": percentile(total, 0.50),
            "p95": percentile(total, 0.95),
            "max": max(total) if total else None,
        },
        "preprocess_ms": {
            "mean": sum(preprocess) / len(preprocess) if preprocess else 0.0,
            "p95": percentile(preprocess, 0.95),
        },
        "infer_ms": {
            "mean": sum(infer) / len(infer) if infer else 0.0,
            "p95": percentile(infer, 0.95),
        },
        "decode_ms": {
            "mean": sum(decode) / len(decode) if decode else 0.0,
            "p95": percentile(decode, 0.95),
        },
        "over_40ms": sum(1 for value in total if value > 40.0),
        "over_40ms_rate": (
            sum(1 for value in total if value > 40.0) / len(total)
            if total
            else 0.0
        ),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", nargs="+", type=Path)
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    result = [summarize(path) for path in args.csv]
    payload = json.dumps(result, ensure_ascii=False, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload, encoding="utf-8")
    print(payload)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

