"""Demonstrate failsafe transitions: degraded -> conservative -> resync.

Run with:
    python -m examples.failsafe_demo
"""
from __future__ import annotations

import time

from mobile_ros.core import Hub, SensorCell, HUB_UNREACHABLE, HUB_RECOVERED


class DemoCell(SensorCell):
    def handle_event(self, event: str) -> None:
        if event == HUB_UNREACHABLE:
            mode = "conservative" if self.conservative_mode else "degraded"
            print(f"[DemoCell] Hub unreachable; entering {mode} mode with policy {self.active_policy}")
        elif event == HUB_RECOVERED:
            print(f"[DemoCell] Hub recovered; resynced policy {self.active_policy}")


def simulate():
    hub = Hub(heartbeat_timeout=0.5, conservative_timeout=5.0)
    cell = DemoCell(cell_id="demo", conservative_policy={"allow": False, "mode": "conservative"})
    cell.apply_policy({"allow": True, "mode": "normal", "bitrate": 10})
    hub.deploy_cell(cell)

    start = time.time()
    # warmup heartbeats to keep hub healthy
    while time.time() - start < 1.0:
        hub.record_heartbeat(cell.cell_id)
        time.sleep(0.2)

    print("[Demo] Stopping heartbeats to trigger degraded mode (500ms)...")
    # no heartbeats for >0.5s will trigger degraded
    time.sleep(1.0)

    print("[Demo] Waiting past 5s to trigger conservative fallback...")
    time.sleep(5.2)

    print("[Demo] Restoring heartbeats to resync from hub...")
    for _ in range(5):
        hub.record_heartbeat(cell.cell_id)
        time.sleep(0.2)

    hub.stop()


if __name__ == "__main__":
    simulate()
