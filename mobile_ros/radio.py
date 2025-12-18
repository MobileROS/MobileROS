"""Radio Information Engine with driver abstractions and SIGNAL_LOST detection."""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Callable, Dict, Iterable, Optional

from mobile_ros.core import ChannelObserver, ChannelUpdate, SIGNAL_LOST


@dataclass
class RadioMetrics:
    rsrp_dbm: float
    snr_db: float
    prb_util: float
    timestamp: float


class RadioDriverBase:
    """Base driver API; implementations must be independent of ROS."""

    def read_metrics(self) -> RadioMetrics:
        raise NotImplementedError


class MockRadioDriver(RadioDriverBase):
    """Mock driver cycling through a list of metrics for demos/tests."""

    def __init__(self, samples: Iterable[Dict[str, float]], interval: float = 0.2) -> None:
        self.samples = list(samples)
        self.interval = interval
        self._index = 0
        self._lock = threading.Lock()
        self._last_time = time.time()

    def read_metrics(self) -> RadioMetrics:
        with self._lock:
            if not self.samples:
                now = time.time()
                return RadioMetrics(rsrp_dbm=-120.0, snr_db=5.0, prb_util=0.2, timestamp=now)
            data = self.samples[self._index % len(self.samples)]
            self._index += 1
            self._last_time = time.time()
            time.sleep(self.interval)
            return RadioMetrics(
                rsrp_dbm=data.get("rsrp_dbm", -120.0),
                snr_db=data.get("snr_db", 0.0),
                prb_util=data.get("prb_util", 0.0),
                timestamp=self._last_time,
            )


class OAIRadioDriver(RadioDriverBase):
    """Placeholder for an OpenAirInterface-backed driver."""

    def __init__(self, reader: Callable[[], Dict[str, float]]) -> None:
        self.reader = reader

    def read_metrics(self) -> RadioMetrics:
        data = self.reader()
        now = time.time()
        return RadioMetrics(
            rsrp_dbm=data.get("rsrp_dbm", -120.0),
            snr_db=data.get("snr_db", 0.0),
            prb_util=data.get("prb_util", 0.0),
            timestamp=data.get("timestamp", now),
        )


class RadioInfoEngine:
    """Engine polling radio drivers and notifying observers of channel changes."""

    def __init__(
        self,
        driver: RadioDriverBase,
        heartbeat_timeout: float = 0.5,
        signal_threshold: float = -140.0,
    ) -> None:
        self.driver = driver
        self.heartbeat_timeout = heartbeat_timeout
        self.signal_threshold = signal_threshold
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._observer: Optional[ChannelObserver] = None
        self._last_update: Optional[float] = None
        self._hub = None

    def bind_hub(self, hub) -> None:
        self._hub = hub

    def start_monitoring(self, observer: ChannelObserver) -> None:
        self._observer = observer
        self._stop.clear()
        if self._thread is None or not self._thread.is_alive():
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1)

    def _loop(self) -> None:
        while not self._stop.is_set():
            metrics = self.driver.read_metrics()
            now = time.time()
            self._last_update = now
            update = ChannelUpdate(metrics=metrics, event=None, timestamp=now)

            if self._observer:
                self._observer._emit(update)

            if metrics.rsrp_dbm < self.signal_threshold or (now - metrics.timestamp) > self.heartbeat_timeout:
                if self._observer:
                    lost_update = ChannelUpdate(metrics=metrics, event=SIGNAL_LOST, timestamp=now)
                    self._observer._emit(lost_update)
            elif self._observer and self._last_update and now - self._last_update < self.heartbeat_timeout:
                # heartbeat path to keep hub informed if bound
                if self._hub:
                    self._hub.record_heartbeat(getattr(self._observer, "cell_id", "unknown"))

    def __del__(self):
        self.stop()


__all__ = [
    "RadioMetrics",
    "RadioDriverBase",
    "MockRadioDriver",
    "OAIRadioDriver",
    "RadioInfoEngine",
]
