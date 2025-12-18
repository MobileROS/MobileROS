"""Network-aware camera example using the Hub/Engine/Cell/Observer stack.

Run with:
    python -m examples.network_aware_camera
"""
from __future__ import annotations

import threading
import time

from mobile_ros.core import ChannelObserver, Hub, SensorCell, HUB_UNREACHABLE, HUB_RECOVERED, SIGNAL_LOST
from mobile_ros.radio import MockRadioDriver, RadioInfoEngine


class CameraObserver(ChannelObserver):
    def __init__(self, cell: SensorCell):
        super().__init__(name=f"observer-{cell.cell_id}")
        self.cell = cell

    def on_channel_update(self, channel_state):
        metrics = channel_state.metrics
        if channel_state.event == SIGNAL_LOST:
            print(f"[Observer] SIGNAL_LOST detected at {metrics.timestamp:.3f}; applying conservative policy")
            self.cell.apply_conservative_policy()
            return
        # Simple policy adaption based on SNR
        if metrics.snr_db < 0:
            policy = {"allow": False, "mode": "pause", "reason": "low_snr"}
        else:
            policy = {"allow": True, "mode": "normal", "bitrate": max(1, metrics.snr_db) * 1.5}
        self.cell.apply_policy(policy)
        print(f"[Observer] Updated policy based on SNR={metrics.snr_db:.1f}: {policy}")


class LoggingCell(SensorCell):
    def handle_event(self, event: str) -> None:
        if event == HUB_UNREACHABLE:
            print(f"[Cell {self.cell_id}] Hub unreachable -> degraded: applying cached policy {self.active_policy}")
        elif event == HUB_RECOVERED:
            print(f"[Cell {self.cell_id}] Hub recovered -> resyncing policy {self.active_policy}")


class HeartbeatThread(threading.Thread):
    def __init__(self, hub: Hub, cell_id: str, period: float = 0.2, drop_after: float = None):
        super().__init__(daemon=True)
        self.hub = hub
        self.cell_id = cell_id
        self.period = period
        self.drop_after = drop_after
        self.start_time = time.time()
        self._stop = threading.Event()

    def run(self):
        while not self._stop.is_set():
            elapsed = time.time() - self.start_time
            if self.drop_after is not None and elapsed > self.drop_after:
                time.sleep(self.period)
                continue
            self.hub.record_heartbeat(self.cell_id)
            time.sleep(self.period)

    def stop(self):
        self._stop.set()


def main():
    hub = Hub(heartbeat_timeout=0.5, conservative_timeout=5.0)
    cell = LoggingCell(cell_id="camera")
    hub.deploy_cell(cell)

    observer = CameraObserver(cell)
    observer.cell_id = cell.cell_id  # heartbeat identification

    samples = [
        {"rsrp_dbm": -90, "snr_db": 5, "prb_util": 0.2},
        {"rsrp_dbm": -95, "snr_db": 1, "prb_util": 0.4},
        {"rsrp_dbm": -150, "snr_db": -5, "prb_util": 0.8},  # triggers SIGNAL_LOST
        {"rsrp_dbm": -100, "snr_db": 10, "prb_util": 0.3},
    ]
    driver = MockRadioDriver(samples, interval=0.5)
    engine = RadioInfoEngine(driver=driver)
    hub.attach_engine("radio_info", engine)

    engine.start_monitoring(observer)

    heartbeat = HeartbeatThread(hub, cell.cell_id, period=0.2)
    heartbeat.start()

    try:
        time.sleep(3)
    finally:
        heartbeat.stop()
        engine.stop()
        hub.stop()


if __name__ == "__main__":
    main()
