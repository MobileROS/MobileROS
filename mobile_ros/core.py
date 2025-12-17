"""Core utilities for MobileROS."""
from __future__ import annotations


class ChannelObserver:
    """Base class providing a consistent interface for channel-aware nodes."""

    def on_channel_update(self, channel_state):  # pragma: no cover - interface hook
        """Handle channel state updates (override in subclasses)."""
        raise NotImplementedError("ChannelObserver subclasses must implement on_channel_update")
