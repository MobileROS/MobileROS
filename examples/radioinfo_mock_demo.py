"""Demonstrate SIGNAL_LOST detection with the mock radio driver.

Run with:
    python -m examples.radioinfo_mock_demo
"""
from __future__ import annotations

import time

from mobile_ros.core import (
    ChannelObserver,
    ChannelUpdate,
    Hub,
    SensorCell,
    HUB_RECOVERED,
    HUB_UNREACHABLE,
    SIGNAL_LOST,
)
from mobile_ros.radio import MockRadioDriver, RadioInfoEngine


class DemoCell(SensorCell):
    def handle_event(self, event: str) -> None:
        if event == HUB_UNREACHABLE:
            print(f"[RadioCell] Hub unreachable; policy={self.active_policy}")
        elif event == HUB_RECOVERED:
            print(f"[RadioCell] Hub recovered; policy={self.active_policy}")


class RadioObserver(ChannelObserver):
    def __init__(self, cell: DemoCell):
        super().__init__(name="radio-observer")
        self.cell = cell

    def on_channel_update(self, channel_state: ChannelUpdate) -> None:
        metrics = channel_state.metrics
        if channel_state.event == SIGNAL_LOST:
            print(
                f"[Observer] SIGNAL_LOST @ {channel_state.timestamp:.3f}"
                f" (rsrp={metrics.rsrp_dbm} snr={metrics.snr_db})"
            )
            self.cell.apply_conservative_policy()
        else:
            print(
                f"[Observer] metrics rsrp={metrics.rsrp_dbm} snr={metrics.snr_db}"
                f" prb={metrics.prb_util}"
            )
            self.cell.apply_policy({"allow": True, "mode": "normal", "bitrate": 5.0})


def run_demo() -> None:
    samples = [
        {"rsrp_dbm": -110, "snr_db": 10, "prb_util": 0.2},
        {"rsrp_dbm": -150, "snr_db": -5, "prb_util": 0.8},  # triggers SIGNAL_LOST
        {"rsrp_dbm": -115, "snr_db": 5, "prb_util": 0.4},
        {"rsrp_dbm": -145, "snr_db": -8, "prb_util": 0.6},  # triggers SIGNAL_LOST again
    ]

    hub = Hub(heartbeat_timeout=0.5, conservative_timeout=5.0)
    cell = DemoCell(cell_id="radio-cell")
    hub.deploy_cell(cell)

    observer = RadioObserver(cell)
    driver = MockRadioDriver(samples, interval=0.1)
    engine = RadioInfoEngine(driver, heartbeat_timeout=0.5, signal_threshold=-140.0)
    engine.bind_hub(hub)
    engine.start_monitoring(observer)

    time.sleep(2.5)
    engine.stop()
    hub.stop()


if __name__ == "__main__":
    run_demo()
