"""Visual SLAM task adapter with Elastic Stream behavior."""
from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Sequence

from mobile_ros.cells import CameraAdaptiveCell, SlamElasticStreamCell, synthetic_image
from mobile_ros.policies import build_stream_policy
from mobile_ros.runtime import snapshot_from_record


@dataclass(frozen=True)
class SlamObservation:
    sequence: int
    keyframe: bool
    feature_count: int
    map_points: int
    estimated_bytes: int
    pose_error_m: float


def orb_slam3_command(
    orb_slam3_root: str | Path,
    vocabulary: str | Path,
    settings_yaml: str | Path,
    image_topic: str = "/camera/mobile_ros/image",
    use_ros: bool = True,
) -> List[str]:
    root = Path(orb_slam3_root)
    if use_ros:
        return [
            "rosrun",
            "ORB_SLAM3",
            "Mono",
            str(Path(vocabulary)),
            str(Path(settings_yaml)),
            image_topic,
        ]
    return [
        str(root / "Examples" / "Monocular" / "mono_euroc"),
        str(Path(vocabulary)),
        str(Path(settings_yaml)),
    ]


class ElasticVisualSlamCase:
    def __init__(self) -> None:
        self.camera = CameraAdaptiveCell("slam_camera")
        self.slam = SlamElasticStreamCell("slam_frontend")

    def run(
        self,
        metric_records: Iterable[Mapping[str, float]],
        frame_count: int = 30,
        width: int = 640,
        height: int = 480,
    ) -> List[SlamObservation]:
        records = list(metric_records)
        if not records:
            raise ValueError("metric_records cannot be empty")
        observations: List[SlamObservation] = []
        pose_error = 0.02
        for index in range(frame_count):
            snapshot = snapshot_from_record(records[index % len(records)])
            policy = build_stream_policy(snapshot, task_criticality=0.75)
            self.camera.update_policy(policy)
            self.slam.update_policy(policy)
            frame = synthetic_image(index, width=width, height=height)
            camera_output = self.camera.process(frame, now=float(index) / 10.0)
            if not camera_output.accepted:
                pose_error += 0.002
                continue
            pose_delta = 0.05 + (index % 5) * 0.03
            slam_output = self.slam.process(camera_output.output, pose_delta_m=pose_delta, now=float(index) / 10.0)
            packet = slam_output.output
            if packet.keyframe:
                pose_error = max(0.01, pose_error * 0.93)
            else:
                pose_error += 0.0015
            observations.append(
                SlamObservation(
                    sequence=index,
                    keyframe=packet.keyframe,
                    feature_count=packet.feature_count,
                    map_points=packet.map_points,
                    estimated_bytes=packet.estimated_bytes,
                    pose_error_m=round(pose_error, 4),
                )
            )
        return observations

    @staticmethod
    def summarize(observations: Sequence[SlamObservation]) -> Dict[str, float]:
        if not observations:
            return {"frames": 0, "keyframes": 0, "mean_pose_error_m": 0.0, "bytes": 0}
        return {
            "frames": float(len(observations)),
            "keyframes": float(sum(1 for item in observations if item.keyframe)),
            "mean_pose_error_m": round(sum(item.pose_error_m for item in observations) / len(observations), 4),
            "bytes": float(sum(item.estimated_bytes for item in observations)),
        }

    @staticmethod
    def as_dicts(observations: Sequence[SlamObservation]) -> List[Dict[str, float]]:
        return [asdict(item) for item in observations]
