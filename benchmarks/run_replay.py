#!/usr/bin/env python3
"""Run MobileROS benchmark replay or compare measured logs with paper baselines."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from mobile_ros.experiments import (
    compare_against_baseline,
    load_baselines,
    paper_slam_policy_trace,
    replay_from_paper_baseline,
    summarize_comparisons,
    write_report,
)


def load_observed(path: Path) -> Dict[str, Dict[str, Dict[str, Any]]]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def main() -> int:
    parser = argparse.ArgumentParser(description="Replay or compare MobileROS paper experiments")
    parser.add_argument("--mode", choices=["replay", "hardware"], default="replay")
    parser.add_argument("--observed", type=Path, help="Measured JSON file for hardware mode")
    parser.add_argument("--baselines", type=Path, default=Path("benchmarks/paper_baselines.json"))
    parser.add_argument("--output", type=Path, default=Path("benchmarks/results/replay_report.json"))
    parser.add_argument("--tolerance", type=float, default=0.01)
    args = parser.parse_args()

    baselines = load_baselines(args.baselines)
    if args.mode == "replay":
        observed = replay_from_paper_baseline(baselines)
    else:
        if not args.observed:
            raise SystemExit("--observed is required in hardware mode")
        observed = load_observed(args.observed)

    comparisons = compare_against_baseline(
        observed=observed,
        baselines=baselines,
        tolerance=args.tolerance,
        mode=args.mode,
    )
    summary = summarize_comparisons(comparisons)
    summary["mode"] = args.mode
    summary["baseline_source"] = baselines["metadata"]
    summary["slam_policy_trace"] = paper_slam_policy_trace(baselines)
    summary["academic_integrity"] = {
        "replay_mode": "uses paper reference values to exercise comparison and reporting code paths",
        "hardware_mode": "requires measured logs collected from OAI/USRP or OAI RF simulator",
    }

    report_path = write_report(summary, args.output)
    print(f"wrote {report_path}")
    print(f"passed {summary['passed']}/{summary['total']} comparisons")
    return 0 if summary["failed"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
