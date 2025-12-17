"""Lightweight message stubs for development environments.

These classes mirror the names used throughout the codebase so imports under
``mobile_ros.msgs.msg`` succeed even when ROS message generation is not
available. The objects are simple attribute containers and are not intended to
replace full ROS message types in production deployments.
"""
from __future__ import annotations

from types import SimpleNamespace


class _BaseMsg:
    def __init__(self, **kwargs):
        for key, value in kwargs.items():
            setattr(self, key, value)


class ChannelState(_BaseMsg):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        # Provide a header field compatible with ROS-style timestamp usage
        if not hasattr(self, "header"):
            self.header = SimpleNamespace(stamp=None)


class TransmissionStrategy(_BaseMsg):
    pass


class Constraint(_BaseMsg):
    pass


class PhyLayerMetrics(_BaseMsg):
    pass


class SpectrumAnalysis(_BaseMsg):
    pass


__all__ = [
    "ChannelState",
    "TransmissionStrategy",
    "Constraint",
    "PhyLayerMetrics",
    "SpectrumAnalysis",
]
