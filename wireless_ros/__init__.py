"""Deprecated compatibility shim for the legacy :mod:`wireless_ros` namespace.

The canonical implementation lives in :mod:`mobile_ros`. Importing this module
re-exports the public ``mobile_ros`` API and raises a :class:`DeprecationWarning`
to guide callers to update their imports.
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
    return getattr(_mobile_ros, name)
