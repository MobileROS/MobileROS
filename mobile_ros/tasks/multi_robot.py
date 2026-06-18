"""Multi-robot map sharing adapter."""
from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Dict, Iterable, List, Mapping, Sequence

from mobile_ros.cells import MapFragment, MapSyncCell
from mobile_ros.policies import build_stream_policy
from mobile_ros.runtime import snapshot_from_record


@dataclass(frozen=True)
class RobotPeer:
    robot_id: str
    map_points: int
    battery_pct: float = 100.0
    role: str = "peer"


class MultiRobotCoordinator:
    def __init__(self, peers: Sequence[RobotPeer]) -> None:
        if not peers:
            raise ValueError("at least one peer is required")
        self.peers = list(peers)
        self.cell = MapSyncCell("multi_robot_map")

    def select_peer_order(self) -> List[RobotPeer]:
        return sorted(self.peers, key=lambda peer: (peer.role != "leader", -peer.map_points, -peer.battery_pct))

    def run(self, metric_records: Iterable[Mapping[str, float]], frames: int = 10) -> List[MapFragment]:
        records = list(metric_records)
        if not records:
            raise ValueError("metric_records cannot be empty")
        fragments: List[MapFragment] = []
        order = self.select_peer_order()
        for index in range(frames):
            snapshot = snapshot_from_record(records[index % len(records)])
            policy = build_stream_policy(snapshot, task_criticality=0.5)
            self.cell.update_policy(policy)
            peer = order[index % len(order)]
            output = self.cell.package_fragment(peer.robot_id, peer.map_points, now=float(index) / 5.0)
            if output.accepted:
                fragments.append(output.output)
        return fragments

    @staticmethod
    def summarize(fragments: Sequence[MapFragment]) -> Dict[str, float]:
        return {
            "fragments": float(len(fragments)),
            "bytes": float(sum(item.bytes_estimate for item in fragments)),
            "points": float(sum(item.points for item in fragments)),
        }

    @staticmethod
    def as_dicts(fragments: Sequence[MapFragment]) -> List[Dict[str, float]]:
        return [asdict(item) for item in fragments]
