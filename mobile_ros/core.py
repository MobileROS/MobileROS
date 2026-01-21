"""Core utilities for MobileROS with lightweight runtime primitives.

This module implements the Hub/Engines/Cells/Observer interfaces
referenced in the P1 specification so they can run without ROS.
"""
from __future__ import annotations

import os
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Optional


# Event names used throughout the failsafe path
HUB_UNREACHABLE = "HUB_UNREACHABLE"
HUB_RECOVERED = "HUB_RECOVERED"
SIGNAL_LOST = "SIGNAL_LOST"


@dataclass
class ChannelUpdate:
    """Represents a channel update delivered to observers."""

    metrics: Any
    event: Optional[str] = None
    timestamp: float = field(default_factory=time.time)


class ChannelObserver:
    """Base class providing a consistent interface for channel-aware nodes."""

    def __init__(self, name: str = "observer") -> None:
        self.name = name
        self._callbacks = []

    def on_channel_update(self, channel_state: ChannelUpdate):  # pragma: no cover - interface hook
        """Handle channel state updates (override in subclasses)."""
        raise NotImplementedError("ChannelObserver subclasses must implement on_channel_update")

    def on_update(self, callback: Callable[[ChannelUpdate], None]) -> None:
        """Register an additional callback that fires after ``on_channel_update``."""

        self._callbacks.append(callback)

    def _emit(self, update: ChannelUpdate) -> None:
        """Internal helper for emitting updates to the observer and callbacks."""

        try:
            self.on_channel_update(update)
        finally:
            for cb in list(self._callbacks):
                cb(update)


class SensorCell:
    """Represents an edge cell with policy caching and failsafe logic."""

    def __init__(
        self,
        cell_id: str,
        conservative_policy: Optional[Dict[str, Any]] = None,
    ) -> None:
        self.cell_id = cell_id
        self.last_known_good_policy: Optional[Dict[str, Any]] = None
        self.active_policy: Dict[str, Any] = conservative_policy or {"allow": False, "mode": "conservative"}
        self.conservative_policy = self.active_policy.copy()
        self.degraded_mode = False
        self.conservative_mode = False
        self._hub: Optional["Hub"] = None

    def bind_hub(self, hub: "Hub") -> None:
        self._hub = hub

    def should_transmit(self, frame: Any) -> bool:
        """Return whether a frame should be transmitted under the active policy."""

        allow = self.active_policy.get("allow", True)
        return bool(allow)

    def apply_policy(self, policy: Dict[str, Any]) -> None:
        """Apply a new policy from the Hub and cache it as last-known-good."""

        self.last_known_good_policy = policy
        self.active_policy = dict(policy)
        self.degraded_mode = False
        self.conservative_mode = False

    def apply_cached_policy(self) -> None:
        """Use the last-known-good policy during degraded mode."""

        if self.last_known_good_policy:
            self.active_policy = dict(self.last_known_good_policy)
        self.degraded_mode = True
        self.conservative_mode = False

    def apply_conservative_policy(self) -> None:
        """Switch to a conservative fallback policy after extended partition."""

        self.active_policy = dict(self.conservative_policy)
        self.degraded_mode = True
        self.conservative_mode = True

    def resync_from_hub(self, policy: Optional[Dict[str, Any]] = None) -> None:
        """Resynchronize with the hub after recovery."""

        if policy is None:
            policy = self.last_known_good_policy or self.conservative_policy
        self.apply_policy(policy)

    def handle_event(self, event: str) -> None:
        """Hook for demo purposes; subclasses can override for logging."""

        # Default implementation keeps silent; examples print explicitly.
        return None


class Hub:
    """Simplified hub coordinating engines and cells with heartbeat checks."""

    def __init__(self, heartbeat_timeout: float = 0.5, conservative_timeout: float = 5.0) -> None:
        self.engines: Dict[str, Any] = {}
        self.cells: Dict[str, SensorCell] = {}
        self.heartbeat_timeout = heartbeat_timeout
        self.conservative_timeout = conservative_timeout
        self._last_heartbeat: Dict[str, float] = {}
        self._stop = threading.Event()
        self._monitor_thread = threading.Thread(target=self._monitor, daemon=True)
        self._monitor_thread.start()

    def attach_engine(self, name: str, engine: Any) -> None:
        """Attach an engine to the hub; the engine may optionally bind back."""

        self.engines[name] = engine
        bind = getattr(engine, "bind_hub", None)
        if callable(bind):
            bind(self)

    def deploy_cell(self, cell: SensorCell) -> None:
        """Register a new cell with the hub and bind for coordination."""

        self.cells[cell.cell_id] = cell
        cell.bind_hub(self)

    def record_heartbeat(self, cell_id: str) -> None:
        """Record a heartbeat from a cell indicating hub connectivity."""

        now = time.time()
        self._last_heartbeat[cell_id] = now
        cell = self.cells.get(cell_id)
        if cell and cell.degraded_mode:
            cell.handle_event(HUB_RECOVERED)
            cell.resync_from_hub()

    def stop(self) -> None:
        self._stop.set()
        self._monitor_thread.join(timeout=1)

    def _monitor(self) -> None:
        force_degraded = os.getenv("FORCE_DEGRADED_MODE", "").lower() == "true"
        if force_degraded:
            print("[Hub] FORCE_DEGRADED_MODE=true detected; triggering immediate degraded mode")
            for cell in self.cells.values():
                cell.apply_cached_policy()
                cell.handle_event(HUB_UNREACHABLE)
            time.sleep(self.conservative_timeout + 1.0)
            for cell in self.cells.values():
                if not cell.conservative_mode:
                    cell.apply_conservative_policy()
                    cell.handle_event(HUB_UNREACHABLE)

        while not self._stop.is_set():
            now = time.time()
            for cell_id, cell in list(self.cells.items()):
                last = self._last_heartbeat.get(cell_id)
                if last is None or now - last > self.heartbeat_timeout:
                    if not cell.degraded_mode:
                        cell.apply_cached_policy()
                        cell.handle_event(HUB_UNREACHABLE)
                if last is None or now - last > self.conservative_timeout:
                    if not cell.conservative_mode:
                        cell.apply_conservative_policy()
                        cell.handle_event(HUB_UNREACHABLE)
            time.sleep(self.heartbeat_timeout / 2.0)


__all__ = [
    "ChannelObserver",
    "ChannelUpdate",
    "SensorCell",
    "Hub",
    "HUB_UNREACHABLE",
    "HUB_RECOVERED",
    "SIGNAL_LOST",
]
