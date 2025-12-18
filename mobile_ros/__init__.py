"""MobileROS Python package bootstrap.

This lightweight package adapter makes the repository's existing modules
importable under the ``mobile_ros`` namespace without reorganizing the
source tree. It avoids hard dependencies on ROS when possible so that
lightweight demos can run in pure Python.
"""
from __future__ import annotations

import pathlib
import sys

# Ensure repository root is on the Python path so module-level imports work
_repo_root = pathlib.Path(__file__).resolve().parent.parent
if str(_repo_root) not in sys.path:
    sys.path.insert(0, str(_repo_root))

# Optional ROS-backed imports guarded to keep pure-Python demos runnable
try:  # pragma: no cover - optional dependency
    from hub.hub import Hub as RosHub  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    RosHub = None

try:  # pragma: no cover - optional dependency
    from cells.cell_base import Cell as CellBase  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    CellBase = None

try:  # pragma: no cover - optional dependency
    from engines.radio_information_engine import RadioInformationEngine  # type: ignore
    from engines.cross_domain_engine import CrossDomainEngine  # type: ignore
    from engines.physical_adaptive_engine import PhysicalAdaptiveEngine  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    RadioInformationEngine = CrossDomainEngine = PhysicalAdaptiveEngine = None

# Pure-Python primitives
from mobile_ros.core import (
    ChannelObserver,
    ChannelUpdate,
    Hub,
    SensorCell,
    HUB_UNREACHABLE,
    HUB_RECOVERED,
    SIGNAL_LOST,
)
from mobile_ros.radio import RadioInfoEngine, RadioDriverBase, MockRadioDriver, OAIRadioDriver, RadioMetrics

__all__ = [
    "ChannelObserver",
    "ChannelUpdate",
    "Hub",
    "SensorCell",
    "HUB_UNREACHABLE",
    "HUB_RECOVERED",
    "SIGNAL_LOST",
    "RadioInfoEngine",
    "RadioDriverBase",
    "MockRadioDriver",
    "OAIRadioDriver",
    "RadioMetrics",
    "RosHub",
    "CellBase",
    "RadioInformationEngine",
    "CrossDomainEngine",
    "PhysicalAdaptiveEngine",
]
