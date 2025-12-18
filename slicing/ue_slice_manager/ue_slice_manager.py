#!/usr/bin/env python3
import argparse
import json
import logging
import random
import socket
import threading
import time
from typing import Tuple

logging.basicConfig(level=logging.INFO, format="[UE-SLICE] %(asctime)s %(levelname)s: %(message)s")


def parse_host_port(text: str) -> Tuple[str, int]:
    host, port = text.split(":")
    return host, int(port)


def duplex_forward(ue_socket: socket.socket, ue_peer: Tuple[str, int], core_socket: socket.socket, core_peer: Tuple[str, int]):
    def ue_to_core():
        while True:
            data, addr = ue_socket.recvfrom(65535)
            logging.info("UE->Core %d bytes", len(data))
            core_socket.sendto(data, core_peer)

    def core_to_ue():
        while True:
            data, addr = core_socket.recvfrom(65535)
            logging.info("Core->UE %d bytes", len(data))
            ue_socket.sendto(data, ue_peer)

    threading.Thread(target=ue_to_core, daemon=True).start()
    threading.Thread(target=core_to_ue, daemon=True).start()


def send_reports(report_socket: socket.socket, report_addr: Tuple[str, int], ue_id: str, stop: threading.Event):
    while not stop.is_set():
        criticality = round(random.random(), 3)
        intent = random.choice(["BEST_EFFORT", "LOW_LATENCY", "ULTRA_RELIABLE"])
        report = {
            "ue_id": ue_id,
            "criticality_score": criticality,
            "semantic_intent": intent,
            "timestamp": time.time(),
        }
        encoded = json.dumps(report).encode("utf-8")
        report_socket.sendto(encoded, report_addr)
        logging.info("Sent UE state report to %s:%d => %s", report_addr[0], report_addr[1], report)
        time.sleep(2.0)


def main():
    parser = argparse.ArgumentParser(description="UE slice manager with UDP tunneling and reporting")
    parser.add_argument("--listen", default="127.0.0.1:50000", help="UE-facing host:port")
    parser.add_argument("--remote", default="127.0.0.1:50001", help="Core-facing host:port")
    parser.add_argument("--report-addr", default="127.0.0.1:60000", help="gNB slice manager host:port")
    parser.add_argument("--ue-id", default="ue-mock-01")
    args = parser.parse_args()

    listen_host, listen_port = parse_host_port(args.listen)
    remote_host, remote_port = parse_host_port(args.remote)
    report_host, report_port = parse_host_port(args.report_addr)

    ue_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ue_socket.bind((listen_host, listen_port))

    core_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    core_socket.bind((remote_host, remote_port))

    logging.info("UE Slice Manager listening on %s:%d -> %s:%d", listen_host, listen_port, remote_host, remote_port)

    stop_event = threading.Event()
    duplex_forward(ue_socket, (remote_host, remote_port), core_socket, (listen_host, listen_port))

    report_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    threading.Thread(target=send_reports, args=(report_socket, (report_host, report_port), args.ue_id, stop_event), daemon=True).start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        stop_event.set()
        logging.info("Shutting down UE Slice Manager")


if __name__ == "__main__":
    main()
