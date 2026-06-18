"""Partition planner for robot perception pipelines."""
from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Dict, List, Mapping, Sequence

from mobile_ros.policies import NetworkSnapshot, build_stream_policy


@dataclass(frozen=True)
class StageProfile:
    name: str
    local_ms: float
    edge_ms: float
    upload_mb: float
    critical: bool = False


@dataclass(frozen=True)
class PartitionDecision:
    local_stages: List[str]
    edge_stages: List[str]
    estimated_latency_ms: float
    estimated_upload_mb: float
    reason: str

    def to_dict(self) -> Dict[str, object]:
        return asdict(self)


class PartitionPlanner:
    def __init__(self, stages: Sequence[StageProfile] | None = None) -> None:
        self.stages = list(
            stages
            or [
                StageProfile("sensor_preprocess", 8.0, 5.0, 0.1, critical=True),
                StageProfile("feature_extract", 24.0, 11.0, 0.8),
                StageProfile("tracking", 16.0, 9.0, 0.25, critical=True),
                StageProfile("mapping", 55.0, 19.0, 1.5),
                StageProfile("loop_closure", 90.0, 28.0, 2.2),
            ]
        )

    def decide(self, snapshot: NetworkSnapshot, deadline_ms: float = 120.0, task_criticality: float = 0.5) -> PartitionDecision:
        policy = build_stream_policy(snapshot, task_criticality)
        mb_per_ms = max(0.001, snapshot.throughput_mbps / 8.0 / 1000.0)
        local: List[str] = []
        edge: List[str] = []
        latency = 0.0
        upload = 0.0
        for stage in self.stages:
            edge_cost = stage.edge_ms + stage.upload_mb / mb_per_ms + snapshot.latency_ms
            local_cost = stage.local_ms
            if stage.critical or policy.quality.value in {"very_poor", "poor"} or edge_cost > local_cost + 15.0:
                local.append(stage.name)
                latency += local_cost
            else:
                edge.append(stage.name)
                latency += edge_cost
                upload += stage.upload_mb
        if latency > deadline_ms and edge:
            moved = edge.pop()
            local.append(moved)
            stage = next(item for item in self.stages if item.name == moved)
            latency -= stage.edge_ms + stage.upload_mb / mb_per_ms + snapshot.latency_ms
            latency += stage.local_ms
            upload -= stage.upload_mb
            reason = "deadline_guard"
        else:
            reason = f"network_{policy.quality.value}"
        return PartitionDecision(
            local_stages=local,
            edge_stages=edge,
            estimated_latency_ms=round(latency, 3),
            estimated_upload_mb=round(upload, 3),
            reason=reason,
        )

    def decide_from_mapping(self, data: Mapping[str, float], deadline_ms: float = 120.0, task_criticality: float = 0.5) -> PartitionDecision:
        prb_value = float(data.get("prb_allocation_pct", data.get("prb_util", 0.0)))
        if prb_value <= 1.0:
            prb_value *= 100.0
        snapshot = NetworkSnapshot(
            throughput_mbps=float(data.get("throughput_mbps", 0.0)),
            latency_ms=float(data.get("latency_ms", 0.0)),
            packet_loss_pct=float(data.get("packet_loss_pct", 0.0)),
            jitter_ms=float(data.get("jitter_ms", 0.0)),
            prb_allocation_pct=prb_value,
        )
        return self.decide(snapshot, deadline_ms=deadline_ms, task_criticality=task_criticality)
