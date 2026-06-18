"""End-to-end MobileROS runtime orchestration."""
from __future__ import annotations

import time
from dataclasses import asdict, dataclass, replace
from typing import Any, Callable, Dict, Iterable, List, Mapping, Optional

from mobile_ros.cells import AdaptiveCell
from mobile_ros.oai import OaiMetricProvider, OaiMetricRecord
from mobile_ros.policies import NetworkSnapshot, StreamPolicy, build_stream_policy
from mobile_ros.slicing import PolicySliceMapper, RecordingSliceClient, SliceClientBase, SliceCommand
from mobile_ros.types import RuntimeEvent, RuntimeMode, TaskState, TaskType


@dataclass(frozen=True)
class RuntimeStep:
    snapshot: NetworkSnapshot
    policy: StreamPolicy
    slice_command: SliceCommand
    timestamp: float
    mode: RuntimeMode

    def to_dict(self) -> Dict[str, Any]:
        data = asdict(self)
        data["policy"]["quality"] = self.policy.quality.value
        data["policy"]["slice_action"] = self.policy.slice_action.value
        data["slice_command"] = self.slice_command.to_dict()
        data["mode"] = self.mode.value
        return data


class MobileRosRuntime:
    def __init__(
        self,
        metric_provider: OaiMetricProvider,
        mode: RuntimeMode = RuntimeMode.REPLAY,
        task_state: Optional[TaskState] = None,
        slice_client: Optional[SliceClientBase] = None,
        slice_mapper: Optional[PolicySliceMapper] = None,
        clock: Callable[[], float] = time.time,
    ) -> None:
        self.metric_provider = metric_provider
        self.mode = mode
        self.task_state = task_state or TaskState("default", TaskType.VISUAL_SLAM, criticality=0.6)
        self.slice_client = slice_client or RecordingSliceClient()
        self.slice_mapper = slice_mapper or PolicySliceMapper()
        self.clock = clock
        self.cells: Dict[str, AdaptiveCell] = {}
        self.events: List[RuntimeEvent] = []
        self.last_step: Optional[RuntimeStep] = None

    def register_cell(self, cell: AdaptiveCell) -> None:
        self.cells[cell.cell_id] = cell
        cell.update_task_state(self.task_state)

    def set_task_state(self, task_state: TaskState) -> None:
        self.task_state = task_state
        for cell in self.cells.values():
            cell.update_task_state(task_state)

    def step(self) -> RuntimeStep:
        record = self.metric_provider.read_record()
        snapshot = snapshot_from_record(record)
        policy = build_stream_policy(snapshot, self.task_state.criticality)
        for cell in self.cells.values():
            cell.update_policy(policy)
        command = self.slice_mapper.map_policy(policy, self.task_state, ue_id=record.ue_id)
        timestamp = self.clock()
        command = replace(command, timestamp=timestamp)
        self.slice_client.send(command)
        step = RuntimeStep(snapshot, policy, command, timestamp, self.mode)
        self.last_step = step
        self.events.append(RuntimeEvent("runtime_step", timestamp=timestamp, details=step.to_dict()))
        return step

    def run_steps(self, count: int) -> List[RuntimeStep]:
        return [self.step() for _ in range(count)]

    def report(self) -> Dict[str, Any]:
        return {
            "mode": self.mode.value,
            "task_state": self.task_state.to_dict(),
            "last_step": self.last_step.to_dict() if self.last_step else None,
            "cells": {cell_id: asdict(cell.stats) for cell_id, cell in self.cells.items()},
            "events": [event.to_dict() for event in self.events],
        }


def snapshot_from_record(record: OaiMetricRecord | Mapping[str, Any]) -> NetworkSnapshot:
    if hasattr(record, "as_policy_mapping"):
        data = record.as_policy_mapping()
    else:
        data = dict(record)
    return NetworkSnapshot(
        throughput_mbps=float(data.get("throughput_mbps", data.get("link_rate_mbps", 0.0))),
        latency_ms=float(data.get("latency_ms", data.get("ul_latency_ms", 0.0))),
        packet_loss_pct=float(data.get("packet_loss_pct", data.get("packet_loss_rate_pct", 0.0))),
        jitter_ms=float(data.get("jitter_ms", 0.0)),
        prb_allocation_pct=float(data.get("prb_allocation_pct", data.get("prb_pct", 0.0))),
        snr_db=float(data.get("snr_db", data.get("sinr_db", 0.0))),
        rsrp_dbm=float(data.get("rsrp_dbm", -95.0)),
    )


class SequenceMetricProvider(OaiMetricProvider):
    def __init__(self, records: Iterable[Mapping[str, Any]], loop: bool = True) -> None:
        self.records = [OaiMetricRecord.from_mapping(dict(record)) for record in records]
        if not self.records:
            raise ValueError("at least one metric record is required")
        self.loop = loop
        self.index = 0

    def read_record(self) -> OaiMetricRecord:
        if self.index >= len(self.records):
            self.index = 0 if self.loop else len(self.records) - 1
        record = self.records[self.index]
        self.index += 1
        return record
