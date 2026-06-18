"""Slice-control clients and policy mapping."""
from __future__ import annotations

import json
import socket
import time
from dataclasses import asdict, dataclass
from typing import Any, Dict, List, Mapping, Optional, Tuple

from mobile_ros.policies import SliceAction, StreamPolicy
from mobile_ros.types import TaskState, TaskType


@dataclass(frozen=True)
class SliceCommand:
    ue_id: str
    action: SliceAction
    priority: int
    min_prb_pct: float
    max_latency_ms: float
    task_type: TaskType
    reason: str
    timestamp: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        data = asdict(self)
        data["action"] = self.action.value
        data["task_type"] = self.task_type.value
        data["timestamp"] = self.timestamp or time.time()
        return data

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), sort_keys=True)


class SliceClientBase:
    def send(self, command: SliceCommand) -> None:
        raise NotImplementedError


class RecordingSliceClient(SliceClientBase):
    def __init__(self) -> None:
        self.commands: List[SliceCommand] = []

    def send(self, command: SliceCommand) -> None:
        self.commands.append(command)


class UdpSliceClient(SliceClientBase):
    """Send one JSON command per UDP datagram to the OAI slice executor."""

    def __init__(self, address: Tuple[str, int] = ("127.0.0.1", 63000)) -> None:
        self.address = address
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, command: SliceCommand) -> None:
        self.sock.sendto(command.to_json().encode("utf-8"), self.address)


class LocalSliceExecutor(SliceClientBase):
    """Apply commands to an in-process state table for simulation and tests."""

    def __init__(self) -> None:
        self.state: Dict[str, Dict[str, Any]] = {}

    def send(self, command: SliceCommand) -> None:
        self.state[command.ue_id] = command.to_dict()


class PolicySliceMapper:
    def __init__(
        self,
        default_ue_id: str = "ue-0",
        latency_targets_ms: Optional[Mapping[TaskType, float]] = None,
    ) -> None:
        self.default_ue_id = default_ue_id
        self.latency_targets_ms = dict(
            latency_targets_ms
            or {
                TaskType.VISUAL_SLAM: 70.0,
                TaskType.LIDAR_PERCEPTION: 80.0,
                TaskType.V2X_SAFETY: 20.0,
                TaskType.MULTI_ROBOT_MAP: 90.0,
                TaskType.PARTITIONED_PIPELINE: 60.0,
                TaskType.GENERIC: 100.0,
            }
        )

    def map_policy(
        self,
        policy: StreamPolicy,
        task_state: TaskState,
        ue_id: Optional[str] = None,
    ) -> SliceCommand:
        criticality = max(0.0, min(1.0, task_state.criticality))
        min_prb = max(10.0, min(95.0, 20.0 + criticality * 60.0 + max(0, policy.priority - 50) * 0.3))
        if policy.slice_action == SliceAction.DEMOTE:
            min_prb = min(min_prb, 25.0)
        elif policy.slice_action == SliceAction.PROMOTE:
            min_prb = max(min_prb, 65.0)
        latency_target = min(task_state.deadline_ms, self.latency_targets_ms.get(task_state.task_type, 100.0))
        return SliceCommand(
            ue_id=ue_id or str(task_state.metadata.get("ue_id", self.default_ue_id)),
            action=policy.slice_action,
            priority=policy.priority,
            min_prb_pct=round(min_prb, 2),
            max_latency_ms=round(latency_target, 2),
            task_type=task_state.task_type,
            reason=f"{task_state.task_id}:{policy.quality.value}",
        )
