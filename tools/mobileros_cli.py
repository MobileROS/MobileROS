#!/usr/bin/env python3
"""Command-line entry point for MobileROS cases."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from mobile_ros.cells import CameraAdaptiveCell, LidarAdaptiveCell, SlamElasticStreamCell
from mobile_ros.runtime import MobileRosRuntime, SequenceMetricProvider
from mobile_ros.slicing import RecordingSliceClient
from mobile_ros.tasks.lidar_perception import AdaptiveVoxelCase
from mobile_ros.tasks.multi_robot import MultiRobotCoordinator, RobotPeer
from mobile_ros.tasks.partition import PartitionPlanner
from mobile_ros.tasks.v2x import V2XSafetyCase
from mobile_ros.tasks.visual_slam import ElasticVisualSlamCase
from mobile_ros.types import RuntimeMode, TaskState, TaskType


def load_jsonl(path: Path) -> List[Mapping[str, Any]]:
    records = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if line:
                records.append(json.loads(line))
    if not records:
        raise ValueError(f"no metric records found in {path}")
    return records


def run_runtime(records: Iterable[Mapping[str, Any]], steps: int) -> Dict[str, Any]:
    slice_client = RecordingSliceClient()
    tick = {"value": 0}

    def clock() -> float:
        value = float(tick["value"])
        tick["value"] += 1
        return value

    runtime = MobileRosRuntime(
        SequenceMetricProvider(records),
        mode=RuntimeMode.RFSIM,
        task_state=TaskState("slam", TaskType.VISUAL_SLAM, criticality=0.75, deadline_ms=70.0),
        slice_client=slice_client,
        clock=clock,
    )
    runtime.register_cell(CameraAdaptiveCell("camera"))
    runtime.register_cell(SlamElasticStreamCell("slam"))
    runtime.register_cell(LidarAdaptiveCell("lidar"))
    runtime.run_steps(steps)
    report = runtime.report()
    report["slice_commands"] = [command.to_dict() for command in slice_client.commands]
    return report


def run_all(records: List[Mapping[str, Any]], frames: int) -> Dict[str, Any]:
    slam_case = ElasticVisualSlamCase()
    slam = slam_case.run(records, frame_count=frames)
    lidar_case = AdaptiveVoxelCase()
    lidar = lidar_case.run(records, frames=frames)
    v2x_case = V2XSafetyCase()
    v2x = v2x_case.run(records, distances_m=[5.0, 3.0, 2.2, 1.4, 0.8, 2.5, 4.5])
    coordinator = MultiRobotCoordinator(
        [
            RobotPeer("robot-0", 4200, role="leader"),
            RobotPeer("robot-1", 3600),
            RobotPeer("robot-2", 2400),
        ]
    )
    fragments = coordinator.run(records, frames=frames)
    partition = PartitionPlanner().decide_from_mapping(records[0], task_criticality=0.75)
    return {
        "runtime": run_runtime(records, min(frames, 10)),
        "slam": {"summary": slam_case.summarize(slam), "observations": slam_case.as_dicts(slam)},
        "lidar": {"summary": lidar_case.summarize(lidar), "observations": lidar_case.as_dicts(lidar)},
        "v2x": {"summary": v2x_case.summarize(v2x), "observations": v2x_case.as_dicts(v2x)},
        "multi_robot": {"summary": coordinator.summarize(fragments), "fragments": coordinator.as_dicts(fragments)},
        "partition": partition.to_dict(),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Run MobileROS runtime and task cases")
    parser.add_argument("--metrics", type=Path, default=REPO_ROOT / "benchmarks" / "oai_rfsim_metrics.jsonl")
    parser.add_argument("--case", choices=["all", "runtime", "slam", "lidar", "v2x", "multi_robot", "partition"], default="all")
    parser.add_argument("--frames", type=int, default=20)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    records = load_jsonl(args.metrics)
    if args.case == "all":
        result = run_all(records, args.frames)
    elif args.case == "runtime":
        result = run_runtime(records, args.frames)
    elif args.case == "slam":
        case = ElasticVisualSlamCase()
        obs = case.run(records, frame_count=args.frames)
        result = {"summary": case.summarize(obs), "observations": case.as_dicts(obs)}
    elif args.case == "lidar":
        case = AdaptiveVoxelCase()
        obs = case.run(records, frames=args.frames)
        result = {"summary": case.summarize(obs), "observations": case.as_dicts(obs)}
    elif args.case == "v2x":
        case = V2XSafetyCase()
        obs = case.run(records, distances_m=[5.0, 3.0, 2.2, 1.4, 0.8, 2.5, 4.5])
        result = {"summary": case.summarize(obs), "observations": case.as_dicts(obs)}
    elif args.case == "multi_robot":
        coordinator = MultiRobotCoordinator([RobotPeer("robot-0", 4200, role="leader"), RobotPeer("robot-1", 3600)])
        fragments = coordinator.run(records, frames=args.frames)
        result = {"summary": coordinator.summarize(fragments), "fragments": coordinator.as_dicts(fragments)}
    else:
        result = PartitionPlanner().decide_from_mapping(records[0], task_criticality=0.75).to_dict()

    text = json.dumps(result, indent=2, sort_keys=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + "\n", encoding="utf-8")
        print(f"wrote {args.output}")
    else:
        print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
