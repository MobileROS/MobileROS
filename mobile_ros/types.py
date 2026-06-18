"""Shared data types for the MobileROS runtime."""
from __future__ import annotations

import time
from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple


class TaskType(str, Enum):
    VISUAL_SLAM = "visual_slam"
    LIDAR_PERCEPTION = "lidar_perception"
    V2X_SAFETY = "v2x_safety"
    MULTI_ROBOT_MAP = "multi_robot_map"
    PARTITIONED_PIPELINE = "partitioned_pipeline"
    GENERIC = "generic"


class RuntimeMode(str, Enum):
    REPLAY = "replay"
    RFSIM = "rfsim"
    HARDWARE = "hardware"


@dataclass(frozen=True)
class FrameMeta:
    frame_id: str
    timestamp: float = field(default_factory=time.time)
    sequence: int = 0
    source: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class ImageFrame:
    meta: FrameMeta
    width: int
    height: int
    encoding: str = "rgb8"
    payload: bytes = b""

    @property
    def bytes_per_frame(self) -> int:
        return len(self.payload) if self.payload else self.width * self.height * 3

    def resized(self, scale: float, jpeg_quality: int) -> "ImageFrame":
        width = max(1, int(round(self.width * scale)))
        height = max(1, int(round(self.height * scale)))
        if self.payload:
            ratio = max(0.01, min(1.0, scale * scale * jpeg_quality / 95.0))
            payload = self.payload[: max(1, int(len(self.payload) * ratio))]
        else:
            payload = b""
        return ImageFrame(
            meta=self.meta,
            width=width,
            height=height,
            encoding=f"{self.encoding};q={jpeg_quality}",
            payload=payload,
        )


PointXYZ = Tuple[float, float, float]


@dataclass(frozen=True)
class PointCloudFrame:
    meta: FrameMeta
    points: Sequence[PointXYZ]
    fields: Tuple[str, ...] = ("x", "y", "z")

    @property
    def bytes_per_frame(self) -> int:
        return len(self.points) * len(self.fields) * 4

    def with_points(self, points: Sequence[PointXYZ]) -> "PointCloudFrame":
        return PointCloudFrame(meta=self.meta, points=tuple(points), fields=self.fields)


@dataclass(frozen=True)
class FeaturePacket:
    meta: FrameMeta
    keyframe: bool
    feature_count: int
    estimated_bytes: int
    pose_delta_m: float = 0.0
    map_points: int = 0

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class TaskState:
    task_id: str
    task_type: TaskType = TaskType.GENERIC
    criticality: float = 0.5
    deadline_ms: float = 100.0
    backlog: int = 0
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        data = asdict(self)
        data["task_type"] = self.task_type.value
        return data


@dataclass(frozen=True)
class RuntimeEvent:
    name: str
    timestamp: float = field(default_factory=time.time)
    details: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class CellStats:
    accepted: int = 0
    dropped: int = 0
    emitted_bytes: int = 0
    last_emit_time: Optional[float] = None

    def update(self, accepted: bool, emitted_bytes: int = 0, now: Optional[float] = None) -> "CellStats":
        return CellStats(
            accepted=self.accepted + (1 if accepted else 0),
            dropped=self.dropped + (0 if accepted else 1),
            emitted_bytes=self.emitted_bytes + emitted_bytes,
            last_emit_time=now if accepted else self.last_emit_time,
        )


@dataclass(frozen=True)
class CellOutput:
    cell_id: str
    accepted: bool
    output: Any = None
    reason: str = ""
    stats: Optional[CellStats] = None

    def to_dict(self) -> Dict[str, Any]:
        output = self.output.to_dict() if hasattr(self.output, "to_dict") else self.output
        return {
            "cell_id": self.cell_id,
            "accepted": self.accepted,
            "output": output,
            "reason": self.reason,
            "stats": asdict(self.stats) if self.stats else None,
        }


def dict_without_none(data: Mapping[str, Any]) -> Dict[str, Any]:
    return {key: value for key, value in data.items() if value is not None}


def merge_metadata(*items: Mapping[str, Any]) -> Dict[str, Any]:
    merged: Dict[str, Any] = {}
    for item in items:
        merged.update(dict(item))
    return merged
