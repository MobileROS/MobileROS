"""Compatibility shim for the legacy namespace.

This package re-exports :mod:`mobile_ros` so existing imports continue to
work. The shim is deprecated; migrate to :mod:`mobile_ros` directly.
"""
from __future__ import annotations

import warnings

warnings.warn(
    "Legacy namespace is deprecated; please migrate imports to mobile_ros",
    DeprecationWarning,
    stacklevel=2,
)

from mobile_ros import *  # noqa: F401,F403
