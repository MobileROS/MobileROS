#!/usr/bin/env python3
import argparse
import json
import logging
import socket
import threading
import time

logging.basicConfig(level=logging.INFO, format="[HUB-SLICE] %(asctime)s %(levelname)s: %(message)s")

class SlicingController:
    def __init__(self, gnb_addr, promote_timeout=5.0):
        self.gnb_addr = gnb_addr
        self.promote_timeout = promote_timeout
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.active_promotions = {}
        self.lock = threading.Lock()

    def send_command(self, action, ue_id, intent="LOW_LATENCY"):
        payload = {"action": action, "ue_id": ue_id, "intent": intent}
        self.socket.sendto(json.dumps(payload).encode("utf-8"), self.gnb_addr)
        logging.info("Sent RRM command %s for %s -> %s:%d", action, ue_id, self.gnb_addr[0], self.gnb_addr[1])

    def promote_slice(self, ue_id):
        with self.lock:
            self.active_promotions[ue_id] = time.time() + self.promote_timeout
        self.send_command("PROMOTE_SLICE", ue_id)

    def retract_slice(self, ue_id):
        with self.lock:
            self.active_promotions.pop(ue_id, None)
        self.send_command("RESET_SLICE", ue_id, intent="BEST_EFFORT")

    def monitor_timeouts(self):
        while True:
            now = time.time()
            expired = []
            with self.lock:
                for ue_id, expiry in list(self.active_promotions.items()):
                    if now > expiry:
                        expired.append(ue_id)
            for ue_id in expired:
                logging.info("Promotion window expired for %s; reverting", ue_id)
                self.retract_slice(ue_id)
            time.sleep(1.0)

    def run_mock_triggers(self, interval):
        counter = 0
        while True:
            ue_id = f"ue-{counter%3}"
            logging.info("CrossDomainEngine emitted SLICE_PROMOTE for %s", ue_id)
            self.promote_slice(ue_id)
            counter += 1
            time.sleep(interval)


def main():
    parser = argparse.ArgumentParser(description="Hub slicing controller -> RRM translator")
    parser.add_argument("--gnb-addr", default="127.0.0.1:60001", help="gNB slice manager host:port")
    parser.add_argument("--promote-timeout", type=float, default=5.0)
    parser.add_argument("--trigger-interval", type=float, default=10.0)
    args = parser.parse_args()

    host, port = args.gnb_addr.split(":")
    controller = SlicingController((host, int(port)), args.promote_timeout)

    threading.Thread(target=controller.monitor_timeouts, daemon=True).start()
    threading.Thread(target=controller.run_mock_triggers, args=(args.trigger_interval,), daemon=True).start()

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        logging.info("Stopping slicing controller")


if __name__ == "__main__":
    main()
