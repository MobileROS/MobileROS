"""Experiment replay and comparison helpers."""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional

from mobile_ros.policies import NetworkSnapshot, build_stream_policy


REPO_ROOT = Path(__file__).resolve().parent.parent
BASELINE_PATH = REPO_ROOT / "benchmarks" / "paper_baselines.json"


@dataclass(frozen=True)
class ComparisonItem:
    table: str
    row: str
    metric: str
    expected: Any
    observed: Any
    tolerance: Optional[float]
    passed: bool
    mode: str


def load_baselines(path: str | Path = BASELINE_PATH) -> Dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as handle:
        return json.load(handle)


def scalar_mean(value: Any) -> Any:
    if isinstance(value, list) and value:
        return value[0]
    return value


def is_numeric_tree(value: Any) -> bool:
    if isinstance(value, list):
        return all(is_numeric_tree(item) for item in value)
    return isinstance(value, (int, float))


def compare_scalar(expected: Any, observed: Any, tolerance: Optional[float]) -> bool:
    if isinstance(expected, list) and isinstance(observed, list):
        if len(expected) != len(observed):
            return False
        return all(compare_scalar(exp, obs, tolerance) for exp, obs in zip(expected, observed))
    expected = scalar_mean(expected)
    observed = scalar_mean(observed)
    if isinstance(expected, str) or expected is None:
        return expected == observed
    if isinstance(expected, list) or isinstance(observed, list):
        return expected == observed
    if tolerance is None:
        return expected == observed
    return abs(float(expected) - float(observed)) <= tolerance


def replay_from_paper_baseline(baselines: Mapping[str, Any]) -> Dict[str, Dict[str, Dict[str, Any]]]:
    replay: Dict[str, Dict[str, Dict[str, Any]]] = {}
    for table, payload in baselines.items():
        if table == "metadata":
            continue
        replay[table] = {}
        for row, metrics in payload["rows"].items():
            replay[table][row] = dict(metrics)
    return replay


def compare_against_baseline(
    observed: Mapping[str, Mapping[str, Mapping[str, Any]]],
    baselines: Optional[Mapping[str, Any]] = None,
    tolerance: float = 0.01,
    mode: str = "replay",
) -> List[ComparisonItem]:
    if baselines is None:
        baselines = load_baselines()
    results: List[ComparisonItem] = []
    for table, table_observed in observed.items():
        if table not in baselines:
            continue
        rows = baselines[table]["rows"]
        for row, metrics in table_observed.items():
            if row not in rows:
                continue
            expected_metrics = rows[row]
            for metric, value in metrics.items():
                if metric not in expected_metrics:
                    continue
                expected = expected_metrics[metric]
                item_tolerance = tolerance if is_numeric_tree(expected) else None
                results.append(
                    ComparisonItem(
                        table=table,
                        row=row,
                        metric=metric,
                        expected=expected,
                        observed=value,
                        tolerance=item_tolerance,
                        passed=compare_scalar(expected, value, item_tolerance),
                        mode=mode,
                    )
                )
    return results


def paper_slam_policy_trace(baselines: Optional[Mapping[str, Any]] = None) -> List[Dict[str, Any]]:
    if baselines is None:
        baselines = load_baselines()
    rows = baselines["table_iii_orb_slam3_prb"]["rows"]
    trace: List[Dict[str, Any]] = []
    for name, row in rows.items():
        snapshot = NetworkSnapshot(
            throughput_mbps=row["link_rate_mbps"][0],
            latency_ms=row["ul_latency_ms"][0],
            packet_loss_pct=row["packet_loss_rate_pct"][0],
            jitter_ms=row["jitter_ms"][0],
            prb_allocation_pct=float(name.split("_")[-1]),
        )
        policy = build_stream_policy(snapshot)
        trace.append(
            {
                "condition": name,
                "snapshot": asdict(snapshot),
                "policy": asdict(policy),
                "paper_keyframe_rate_hz": row["keyframe_rate_hz"][0],
                "paper_pose_error_m": row["mean_abs_pose_error_m"][0],
            }
        )
    return trace


def summarize_comparisons(items: Iterable[ComparisonItem]) -> Dict[str, Any]:
    serialized = [asdict(item) for item in items]
    failed = [item for item in serialized if not item["passed"]]
    return {
        "total": len(serialized),
        "passed": len(serialized) - len(failed),
        "failed": len(failed),
        "items": serialized,
    }


def write_report(report: Mapping[str, Any], path: str | Path) -> Path:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    return target
