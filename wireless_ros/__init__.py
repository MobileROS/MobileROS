"""Compatibility shim for legacy imports.

The project namespace has moved from ``wireless_ros`` to ``mobile_ros``. This
module re-exports the new package to preserve compatibility with existing
scripts while emitting a deprecation warning.
"""
from __future__ import annotations

import warnings

warnings.warn(
    "wireless_ros is deprecated; please migrate imports to mobile_ros",
    DeprecationWarning,
    stacklevel=2,
)

from mobile_ros import *  # noqa: F401,F403
