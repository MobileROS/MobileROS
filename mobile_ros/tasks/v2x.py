"""V2X safety task adapter."""
from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Dict, Iterable, List, Mapping, Sequence

from mobile_ros.cells import V2XSafetyCell
from mobile_ros.policies import build_stream_policy
from mobile_ros.runtime import snapshot_from_record


@dataclass(frozen=True)
class V2XObservation:
    sequence: int
    obstacle_distance_m: float
    requested_speed_mps: float
    target_speed_mps: float
    allow_motion: bool
    criticality: float
    reason: str


class V2XSafetyCase:
    def __init__(self) -> None:
        self.cell = V2XSafetyCell("v2x_safety")

    def run(
        self,
        metric_records: Iterable[Mapping[str, float]],
        distances_m: Sequence[float],
        requested_speed_mps: float = 1.0,
    ) -> List[V2XObservation]:
        records = list(metric_records)
        if not records:
            raise ValueError("metric_records cannot be empty")
        observations: List[V2XObservation] = []
        for index, distance in enumerate(distances_m):
            snapshot = snapshot_from_record(records[index % len(records)])
            criticality = V2XSafetyCell.criticality_from_distance(distance, relative_speed_mps=1.0)
            policy = build_stream_policy(snapshot, task_criticality=criticality)
            self.cell.update_policy(policy)
            decision = self.cell.decide(distance, requested_speed_mps, relative_speed_mps=1.0)
            observations.append(
                V2XObservation(
                    sequence=index,
                    obstacle_distance_m=distance,
                    requested_speed_mps=requested_speed_mps,
                    target_speed_mps=decision.target_speed_mps,
                    allow_motion=decision.allow_motion,
                    criticality=decision.criticality,
                    reason=decision.reason,
                )
            )
        return observations

    @staticmethod
    def summarize(observations: Sequence[V2XObservation]) -> Dict[str, float]:
        stops = sum(1 for item in observations if not item.allow_motion)
        slow = sum(1 for item in observations if item.allow_motion and item.target_speed_mps < item.requested_speed_mps)
        return {
            "samples": float(len(observations)),
            "stops": float(stops),
            "slowdowns": float(slow),
        }

    @staticmethod
    def as_dicts(observations: Sequence[V2XObservation]) -> List[Dict[str, float]]:
        return [asdict(item) for item in observations]
