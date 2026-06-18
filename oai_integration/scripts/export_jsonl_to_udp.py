#!/usr/bin/env python3
from __future__ import annotations

import argparse
import socket
import time
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description="Replay JSONL OAI metrics to UDP")
    parser.add_argument("--input", type=Path, default=Path("benchmarks/oai_rfsim_metrics.jsonl"))
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=62000)
    parser.add_argument("--interval", type=float, default=0.2)
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    lines = [line.strip().encode("utf-8") for line in args.input.read_text(encoding="utf-8").splitlines() if line.strip()]
    if not lines:
        raise SystemExit(f"no metrics in {args.input}")

    while True:
        for payload in lines:
            sock.sendto(payload, (args.host, args.port))
            time.sleep(args.interval)


if __name__ == "__main__":
    raise SystemExit(main())
