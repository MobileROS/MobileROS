"""MobileROS Python package bootstrap.

This lightweight package adapter makes the repository's existing modules
importable under the ``mobile_ros`` namespace without reorganizing the
source tree. It ensures the examples that use ``from mobile_ros import ...``
resolve correctly after the legacy namespace rename.
"""
from __future__ import annotations

import pathlib
import sys

# Ensure repository root is on the Python path so module-level imports work
_repo_root = pathlib.Path(__file__).resolve().parent.parent
if str(_repo_root) not in sys.path:
    sys.path.insert(0, str(_repo_root))

from hub.hub import Hub  # noqa: E402
from cells.cell_base import Cell as CellBase  # noqa: E402
from engines.radio_information_engine import RadioInformationEngine  # noqa: E402
from engines.cross_domain_engine import CrossDomainEngine  # noqa: E402
from engines.physical_adaptive_engine import PhysicalAdaptiveEngine  # noqa: E402
from mobile_ros.core import ChannelObserver  # noqa: E402

__all__ = [
    "Hub",
    "CellBase",
    "RadioInformationEngine",
    "CrossDomainEngine",
    "PhysicalAdaptiveEngine",
    "ChannelObserver",
]
