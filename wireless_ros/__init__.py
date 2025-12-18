"""Compatibility shim for the legacy :mod:`wireless_ros` namespace.

The primary implementation now lives under :mod:`mobile_ros`. This shim only
re-exports that public API and emits a deprecation warning on import to help
migration.
"""
from __future__ import annotations

import warnings

import mobile_ros as _mobile_ros

warnings.warn(
    "wireless_ros is deprecated; please migrate imports to mobile_ros",
    DeprecationWarning,
    stacklevel=2,
)

from mobile_ros import *  # noqa: F401,F403

__all__ = getattr(_mobile_ros, "__all__", [])


def __getattr__(name: str):
    if hasattr(_mobile_ros, name):
        return getattr(_mobile_ros, name)
    raise AttributeError(
        "wireless_ros is a deprecated shim; attribute "
        f"{name!r} is not available in mobile_ros"
    )
