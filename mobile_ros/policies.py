"""Adaptation policies used by MobileROS runtime and experiments."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Mapping


class NetworkQuality(str, Enum):
    VERY_POOR = "very_poor"
    POOR = "poor"
    MODERATE = "moderate"
    GOOD = "good"
    EXCELLENT = "excellent"


class SliceAction(str, Enum):
    HOLD = "hold"
    PROMOTE = "promote"
    DEMOTE = "demote"


@dataclass(frozen=True)
class NetworkSnapshot:
    throughput_mbps: float
    latency_ms: float
    packet_loss_pct: float
    jitter_ms: float
    prb_allocation_pct: float
    snr_db: float = 0.0
    rsrp_dbm: float = -95.0


@dataclass(frozen=True)
class StreamPolicy:
    quality: NetworkQuality
    slice_action: SliceAction
    keyframe_rate_hz: float
    orb_max_keyframes_per_second: int
    jpeg_quality: int
    resolution_scale: float
    voxel_leaf_size_m: float
    publish_rate_hz: float
    priority: int


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def classify_network(snapshot: NetworkSnapshot) -> NetworkQuality:
    score = 0.0
    score += clamp(snapshot.throughput_mbps / 10.0, 0.0, 1.0) * 0.35
    score += (1.0 - clamp(snapshot.latency_ms / 150.0, 0.0, 1.0)) * 0.25
    score += (1.0 - clamp(snapshot.packet_loss_pct / 10.0, 0.0, 1.0)) * 0.20
    score += (1.0 - clamp(snapshot.jitter_ms / 30.0, 0.0, 1.0)) * 0.10
    score += clamp(snapshot.prb_allocation_pct / 90.0, 0.0, 1.0) * 0.10

    if score < 0.25:
        return NetworkQuality.VERY_POOR
    if score < 0.45:
        return NetworkQuality.POOR
    if score < 0.65:
        return NetworkQuality.MODERATE
    if score < 0.82:
        return NetworkQuality.GOOD
    return NetworkQuality.EXCELLENT


def elastic_stream_keyframe_limit(
    throughput_mbps: float,
    alpha: float = 2.0,
    k_min: int = 5,
    k_max: int = 20,
) -> int:
    return int(round(clamp(alpha * throughput_mbps, k_min, k_max)))


def effective_keyframe_rate(throughput_mbps: float, latency_ms: float) -> float:
    rate = 0.55 + 0.085 * throughput_mbps - 0.0015 * max(0.0, latency_ms - 15.0)
    return round(clamp(rate, 0.45, 1.5), 2)


def adaptive_jpeg_quality(throughput_mbps: float) -> int:
    return int(round(clamp(30.0 + throughput_mbps * 6.6, 30.0, 95.0)))


def adaptive_resolution_scale(throughput_mbps: float) -> float:
    if throughput_mbps < 4.0:
        return 0.50
    if throughput_mbps < 7.0:
        return 0.75
    return 1.00


def adaptive_voxel_leaf_size(
    available_bandwidth_mbps: float,
    base_leaf_m: float = 0.10,
    beta: float = 1.5,
    max_bandwidth_mbps: float = 20.0,
) -> float:
    ratio = clamp(available_bandwidth_mbps / max_bandwidth_mbps, 0.0, 1.0)
    return round(base_leaf_m * (1.0 + beta * (1.0 - ratio)), 3)


def choose_slice_action(task_criticality: float, snapshot: NetworkSnapshot) -> SliceAction:
    if task_criticality >= 0.80:
        return SliceAction.PROMOTE
    if snapshot.prb_allocation_pct >= 80.0 and task_criticality < 0.30:
        return SliceAction.DEMOTE
    return SliceAction.HOLD


def build_stream_policy(snapshot: NetworkSnapshot, task_criticality: float = 0.5) -> StreamPolicy:
    quality = classify_network(snapshot)
    priority_by_quality = {
        NetworkQuality.VERY_POOR: 95,
        NetworkQuality.POOR: 85,
        NetworkQuality.MODERATE: 70,
        NetworkQuality.GOOD: 55,
        NetworkQuality.EXCELLENT: 45,
    }
    priority = min(100, priority_by_quality[quality] + int(task_criticality * 20))
    return StreamPolicy(
        quality=quality,
        slice_action=choose_slice_action(task_criticality, snapshot),
        keyframe_rate_hz=effective_keyframe_rate(snapshot.throughput_mbps, snapshot.latency_ms),
        orb_max_keyframes_per_second=elastic_stream_keyframe_limit(snapshot.throughput_mbps),
        jpeg_quality=adaptive_jpeg_quality(snapshot.throughput_mbps),
        resolution_scale=adaptive_resolution_scale(snapshot.throughput_mbps),
        voxel_leaf_size_m=adaptive_voxel_leaf_size(snapshot.throughput_mbps),
        publish_rate_hz=round(clamp(snapshot.throughput_mbps * 1.8, 2.0, 20.0), 1),
        priority=priority,
    )


def policy_from_mapping(data: Mapping[str, float], task_criticality: float = 0.5) -> StreamPolicy:
    snapshot = NetworkSnapshot(
        throughput_mbps=float(data.get("throughput_mbps", data.get("link_rate_mbps", 0.0))),
        latency_ms=float(data.get("latency_ms", data.get("ul_latency_ms", 0.0))),
        packet_loss_pct=float(data.get("packet_loss_pct", data.get("packet_loss_rate_pct", 0.0))),
        jitter_ms=float(data.get("jitter_ms", 0.0)),
        prb_allocation_pct=float(data.get("prb_allocation_pct", data.get("prb_pct", 0.0))),
        snr_db=float(data.get("snr_db", 0.0)),
        rsrp_dbm=float(data.get("rsrp_dbm", -95.0)),
    )
    return build_stream_policy(snapshot, task_criticality=task_criticality)
