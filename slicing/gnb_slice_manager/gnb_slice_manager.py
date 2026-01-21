#!/usr/bin/env python3
import argparse
import json
import logging
import os
import socket
import threading
from typing import Dict, Optional

logging.basicConfig(level=logging.INFO, format="[GNB-SLICE] %(asctime)s %(levelname)s: %(message)s")

class PolicyExecutor:
    def __init__(self, policy_file: Optional[str] = None):
        self.weights: Dict[str, float] = {}
        self.prb_alloc: Dict[str, int] = {}
        self.rrm_policy = {}
        if policy_file and os.path.exists(policy_file):
            self.load_rrm_policy(policy_file)

    def load_rrm_policy(self, policy_file: str):
        try:
            with open(policy_file, "r") as f:
                self.rrm_policy = json.load(f)
            logging.info("Loaded RRM policy from %s", policy_file)
            if "rrmPolicyRatio" in self.rrm_policy:
                logging.info("  Found %d rrmPolicyRatio entries", len(self.rrm_policy["rrmPolicyRatio"]))
            if "subSlicePolicy" in self.rrm_policy and self.rrm_policy["subSlicePolicy"].get("enabled"):
                logging.info("  Sub-slice policy enabled with %d sub-slices",
                            len(self.rrm_policy.get("subSlicePolicy", {}).get("subSlices", [])))
            if "ueSliceMapping" in self.rrm_policy:
                logging.info("  UE slice mapping entries: %d", len(self.rrm_policy["ueSliceMapping"]))
        except Exception as e:
            logging.error("Failed to load RRM policy from %s: %s", policy_file, e)

    def update_policy(self, ue_id: str, criticality: float, intent: str):
        base_weight = 1.0 + criticality
        if intent == "LOW_LATENCY":
            base_weight += 0.5
        elif intent == "ULTRA_RELIABLE":
            base_weight += 0.75
        self.weights[ue_id] = base_weight
        self.prb_alloc[ue_id] = max(1, int(base_weight * 10))
        logging.info("[policy_executor] Updated UE %s -> weight %.2f, prbs %d", ue_id, base_weight, self.prb_alloc[ue_id])

    def snapshot(self):
        return {ue: {"weight": w, "prbs": self.prb_alloc.get(ue, 0)} for ue, w in self.weights.items()}


def listen_for_reports(bind_host: str, bind_port: int, executor: PolicyExecutor):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((bind_host, bind_port))
    logging.info("gNB Slice Manager listening on %s:%d", bind_host, bind_port)
    while True:
        data, addr = sock.recvfrom(65535)
        try:
            report = json.loads(data.decode("utf-8"))
        except json.JSONDecodeError:
            logging.warning("Invalid report from %s", addr)
            continue
        ue_id = report.get("ue_id", "unknown")
        criticality = float(report.get("criticality_score", 0.0))
        intent = report.get("semantic_intent", "BEST_EFFORT")
        logging.info("Received report from %s: score=%.3f intent=%s", ue_id, criticality, intent)
        executor.update_policy(ue_id, criticality, intent)
        logging.info("Scheduler weights/PRB updated => %s", executor.snapshot())


def listen_for_rrm_commands(bind_host: str, bind_port: int, executor: PolicyExecutor):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((bind_host, bind_port))
    logging.info("gNB RRM command listener on %s:%d", bind_host, bind_port)
    while True:
        data, addr = sock.recvfrom(4096)
        try:
            cmd = json.loads(data.decode("utf-8"))
        except json.JSONDecodeError:
            logging.warning("Invalid RRM command from %s", addr)
            continue
        if cmd.get("action") == "PROMOTE_SLICE":
            ue_id = cmd.get("ue_id", "unknown")
            logging.info("Received SLICE_PROMOTE for %s", ue_id)
            executor.update_policy(ue_id, criticality=1.0, intent=cmd.get("intent", "LOW_LATENCY"))
        elif cmd.get("action") == "RESET_SLICE":
            ue_id = cmd.get("ue_id", "unknown")
            logging.info("Received slice reset for %s", ue_id)
            executor.update_policy(ue_id, criticality=0.1, intent="BEST_EFFORT")
        logging.info("Scheduler weights/PRB updated => %s", executor.snapshot())


def main():
    parser = argparse.ArgumentParser(description="Mock gNB slice manager + scheduler hooks")
    parser.add_argument("--report-bind", default="0.0.0.0:60000")
    parser.add_argument("--rrm-bind", default="0.0.0.0:60001")
    parser.add_argument("--policy-file", default="rrmPolicy.json", help="Path to RRM policy JSON")
    args = parser.parse_args()

    report_host, report_port = args.report_bind.split(":")
    rrm_host, rrm_port = args.rrm_bind.split(":")

    policy_path = args.policy_file if os.path.isabs(args.policy_file) else os.path.join(os.getcwd(), args.policy_file)
    executor = PolicyExecutor(policy_file=policy_path)
    threading.Thread(target=listen_for_reports, args=(report_host, int(report_port), executor), daemon=True).start()
    threading.Thread(target=listen_for_rrm_commands, args=(rrm_host, int(rrm_port), executor), daemon=True).start()

    try:
        while True:
            threading.Event().wait(1.0)
    except KeyboardInterrupt:
        logging.info("Shutting down gNB slice manager")


if __name__ == "__main__":
    main()
