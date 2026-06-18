"""LiDAR perception adapter using Adaptive Voxel filtering."""
from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Sequence

from mobile_ros.cells import LidarAdaptiveCell, synthetic_cloud
from mobile_ros.policies import build_stream_policy
from mobile_ros.runtime import snapshot_from_record
from mobile_ros.types import PointCloudFrame


@dataclass(frozen=True)
class LidarObservation:
    sequence: int
    input_points: int
    output_points: int
    voxel_leaf_size_m: float
    estimated_bytes: int


def sample_point_cloud(sequence: int = 0, count: int = 2000) -> PointCloudFrame:
    return synthetic_cloud(sequence, count=count)


def openpcdet_command(openpcdet_root: str | Path, cfg_file: str | Path, ckpt: str | Path, data_path: str | Path) -> List[str]:
    root = Path(openpcdet_root)
    return [
        "python",
        str(root / "tools" / "demo.py"),
        "--cfg_file",
        str(Path(cfg_file)),
        "--ckpt",
        str(Path(ckpt)),
        "--data_path",
        str(Path(data_path)),
    ]


class AdaptiveVoxelCase:
    def __init__(self) -> None:
        self.cell = LidarAdaptiveCell("lidar_perception")

    def run(self, metric_records: Iterable[Mapping[str, float]], frames: int = 20, points_per_frame: int = 2500) -> List[LidarObservation]:
        records = list(metric_records)
        if not records:
            raise ValueError("metric_records cannot be empty")
        observations: List[LidarObservation] = []
        for index in range(frames):
            snapshot = snapshot_from_record(records[index % len(records)])
            policy = build_stream_policy(snapshot, task_criticality=0.55)
            self.cell.update_policy(policy)
            cloud = sample_point_cloud(index, points_per_frame)
            output = self.cell.process(cloud, now=float(index) / 8.0)
            if not output.accepted:
                continue
            reduced = output.output
            observations.append(
                LidarObservation(
                    sequence=index,
                    input_points=len(cloud.points),
                    output_points=len(reduced.points),
                    voxel_leaf_size_m=policy.voxel_leaf_size_m,
                    estimated_bytes=reduced.bytes_per_frame,
                )
            )
        return observations

    @staticmethod
    def summarize(observations: Sequence[LidarObservation]) -> Dict[str, float]:
        if not observations:
            return {"frames": 0, "mean_reduction_ratio": 0.0, "bytes": 0}
        ratios = [1.0 - item.output_points / max(1, item.input_points) for item in observations]
        return {
            "frames": float(len(observations)),
            "mean_reduction_ratio": round(sum(ratios) / len(ratios), 4),
            "bytes": float(sum(item.estimated_bytes for item in observations)),
        }

    @staticmethod
    def as_dicts(observations: Sequence[LidarObservation]) -> List[Dict[str, float]]:
        return [asdict(item) for item in observations]
