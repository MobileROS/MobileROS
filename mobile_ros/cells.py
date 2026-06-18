"""Runtime cells for camera, LiDAR, SLAM, V2X, and map synchronization."""
from __future__ import annotations

import math
import time
from collections import defaultdict
from dataclasses import dataclass
from typing import DefaultDict, Iterable, List, Optional, Sequence, Tuple

from mobile_ros.policies import NetworkSnapshot, StreamPolicy, build_stream_policy
from mobile_ros.types import (
    CellOutput,
    CellStats,
    FeaturePacket,
    FrameMeta,
    ImageFrame,
    PointCloudFrame,
    PointXYZ,
    TaskState,
    TaskType,
)


def _now(value: Optional[float] = None) -> float:
    return time.time() if value is None else value


class AdaptiveCell:
    def __init__(self, cell_id: str, task_state: Optional[TaskState] = None) -> None:
        self.cell_id = cell_id
        self.task_state = task_state or TaskState(cell_id)
        self.policy: StreamPolicy = build_stream_policy(
            snapshot=NetworkSnapshot(
                throughput_mbps=10.0,
                latency_ms=30.0,
                packet_loss_pct=0.0,
                jitter_ms=2.0,
                prb_allocation_pct=70.0,
            )
        )
        self.stats = CellStats()
        self._last_publish: Optional[float] = None

    def update_policy(self, policy: StreamPolicy) -> None:
        self.policy = policy

    def update_task_state(self, task_state: TaskState) -> None:
        self.task_state = task_state

    def publish_due(self, now: Optional[float] = None) -> bool:
        now = _now(now)
        rate = max(0.1, float(self.policy.publish_rate_hz))
        if self._last_publish is None:
            return True
        return now - self._last_publish >= 1.0 / rate

    def _record(self, accepted: bool, emitted_bytes: int = 0, now: Optional[float] = None) -> CellStats:
        timestamp = _now(now)
        self.stats = self.stats.update(accepted=accepted, emitted_bytes=emitted_bytes, now=timestamp)
        if accepted:
            self._last_publish = timestamp
        return self.stats


class CameraAdaptiveCell(AdaptiveCell):
    def __init__(self, cell_id: str = "camera", task_state: Optional[TaskState] = None) -> None:
        super().__init__(cell_id, task_state or TaskState(cell_id, TaskType.VISUAL_SLAM, criticality=0.6))

    def process(self, frame: ImageFrame, now: Optional[float] = None) -> CellOutput:
        if not self.publish_due(now):
            stats = self._record(False, now=now)
            return CellOutput(self.cell_id, False, reason="rate_limited", stats=stats)
        output = frame.resized(self.policy.resolution_scale, self.policy.jpeg_quality)
        stats = self._record(True, output.bytes_per_frame, now=now)
        return CellOutput(self.cell_id, True, output=output, reason="published", stats=stats)


class LidarAdaptiveCell(AdaptiveCell):
    def __init__(self, cell_id: str = "lidar", task_state: Optional[TaskState] = None) -> None:
        super().__init__(cell_id, task_state or TaskState(cell_id, TaskType.LIDAR_PERCEPTION, criticality=0.55))

    @staticmethod
    def voxel_filter(points: Sequence[PointXYZ], leaf_size_m: float) -> Tuple[PointXYZ, ...]:
        if leaf_size_m <= 0 or not points:
            return tuple(points)
        buckets: DefaultDict[Tuple[int, int, int], List[PointXYZ]] = defaultdict(list)
        for x, y, z in points:
            key = (
                math.floor(x / leaf_size_m),
                math.floor(y / leaf_size_m),
                math.floor(z / leaf_size_m),
            )
            buckets[key].append((float(x), float(y), float(z)))
        reduced: List[PointXYZ] = []
        for members in buckets.values():
            count = float(len(members))
            reduced.append(
                (
                    sum(point[0] for point in members) / count,
                    sum(point[1] for point in members) / count,
                    sum(point[2] for point in members) / count,
                )
            )
        return tuple(reduced)

    def process(self, cloud: PointCloudFrame, now: Optional[float] = None) -> CellOutput:
        if not self.publish_due(now):
            stats = self._record(False, now=now)
            return CellOutput(self.cell_id, False, reason="rate_limited", stats=stats)
        points = self.voxel_filter(cloud.points, self.policy.voxel_leaf_size_m)
        output = cloud.with_points(points)
        stats = self._record(True, output.bytes_per_frame, now=now)
        return CellOutput(self.cell_id, True, output=output, reason="voxel_filtered", stats=stats)


class SlamElasticStreamCell(AdaptiveCell):
    def __init__(self, cell_id: str = "slam", task_state: Optional[TaskState] = None) -> None:
        super().__init__(cell_id, task_state or TaskState(cell_id, TaskType.VISUAL_SLAM, criticality=0.75))
        self._last_keyframe_time: Optional[float] = None
        self._keyframes_this_second = 0
        self._keyframe_window = 0
        self.map_points = 0

    def _keyframe_budget_available(self, now: float) -> bool:
        window = int(now)
        if window != self._keyframe_window:
            self._keyframe_window = window
            self._keyframes_this_second = 0
        if self._keyframes_this_second >= self.policy.orb_max_keyframes_per_second:
            return False
        if self._last_keyframe_time is None:
            return True
        return now - self._last_keyframe_time >= 1.0 / max(0.1, self.policy.keyframe_rate_hz)

    def process(self, frame: ImageFrame, pose_delta_m: float = 0.0, now: Optional[float] = None) -> CellOutput:
        timestamp = _now(now)
        force_keyframe = pose_delta_m >= 0.15
        keyframe = self._keyframe_budget_available(timestamp) or force_keyframe
        if keyframe:
            self._last_keyframe_time = timestamp
            self._keyframes_this_second += 1
        feature_count = max(60, int(frame.width * frame.height * self.policy.resolution_scale / 4200.0))
        if not keyframe:
            feature_count = max(20, feature_count // 3)
        estimated_bytes = int(feature_count * (24 if keyframe else 10))
        self.map_points += feature_count if keyframe else max(1, feature_count // 8)
        packet = FeaturePacket(
            meta=frame.meta,
            keyframe=keyframe,
            feature_count=feature_count,
            estimated_bytes=estimated_bytes,
            pose_delta_m=pose_delta_m,
            map_points=self.map_points,
        )
        stats = self._record(True, estimated_bytes, now=timestamp)
        return CellOutput(self.cell_id, True, output=packet, reason="keyframe" if keyframe else "tracking", stats=stats)


@dataclass(frozen=True)
class SafetyDecision:
    allow_motion: bool
    target_speed_mps: float
    criticality: float
    reason: str


class V2XSafetyCell(AdaptiveCell):
    def __init__(self, cell_id: str = "v2x", task_state: Optional[TaskState] = None) -> None:
        super().__init__(cell_id, task_state or TaskState(cell_id, TaskType.V2X_SAFETY, criticality=0.9, deadline_ms=20.0))

    @staticmethod
    def criticality_from_distance(distance_m: float, relative_speed_mps: float = 0.0) -> float:
        distance_score = 1.0 - min(1.0, max(0.0, distance_m) / 8.0)
        speed_score = min(1.0, max(0.0, relative_speed_mps) / 6.0)
        return round(max(distance_score, 0.5 * distance_score + 0.5 * speed_score), 3)

    def decide(self, obstacle_distance_m: float, requested_speed_mps: float, relative_speed_mps: float = 0.0) -> SafetyDecision:
        criticality = self.criticality_from_distance(obstacle_distance_m, relative_speed_mps)
        loss_sensitive = self.policy.quality.value in {"very_poor", "poor"} or self.policy.priority >= 90
        if obstacle_distance_m < 1.2 or (loss_sensitive and obstacle_distance_m < 2.0):
            return SafetyDecision(False, 0.0, criticality, "emergency_stop")
        if obstacle_distance_m < 3.0:
            return SafetyDecision(True, min(requested_speed_mps, 0.3), criticality, "slow_zone")
        return SafetyDecision(True, requested_speed_mps, criticality, "clear")


@dataclass(frozen=True)
class MapFragment:
    robot_id: str
    sequence: int
    points: int
    bytes_estimate: int
    priority: int


class MapSyncCell(AdaptiveCell):
    def __init__(self, cell_id: str = "map_sync", task_state: Optional[TaskState] = None) -> None:
        super().__init__(cell_id, task_state or TaskState(cell_id, TaskType.MULTI_ROBOT_MAP, criticality=0.5))
        self.sequence = 0

    def package_fragment(self, robot_id: str, map_points: int, now: Optional[float] = None) -> CellOutput:
        if not self.publish_due(now):
            stats = self._record(False, now=now)
            return CellOutput(self.cell_id, False, reason="rate_limited", stats=stats)
        self.sequence += 1
        points = max(1, int(map_points * self.policy.resolution_scale))
        fragment = MapFragment(
            robot_id=robot_id,
            sequence=self.sequence,
            points=points,
            bytes_estimate=points * 16,
            priority=self.policy.priority,
        )
        stats = self._record(True, fragment.bytes_estimate, now=now)
        return CellOutput(self.cell_id, True, output=fragment, reason="fragment", stats=stats)


def synthetic_image(sequence: int, width: int = 640, height: int = 480) -> ImageFrame:
    size = width * height * 3
    payload = bytes((sequence + offset) % 251 for offset in range(min(size, 65536)))
    return ImageFrame(meta=FrameMeta("camera", sequence=sequence), width=width, height=height, payload=payload)


def synthetic_cloud(sequence: int, count: int = 2000) -> PointCloudFrame:
    points = []
    for index in range(count):
        x = (index % 80) * 0.04
        y = ((index // 80) % 40) * 0.04
        z = ((index + sequence) % 15) * 0.03
        points.append((x, y, z))
    return PointCloudFrame(meta=FrameMeta("lidar", sequence=sequence), points=tuple(points))
