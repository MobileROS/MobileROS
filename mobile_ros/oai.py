"""OpenAirInterface metric providers for MobileROS.

The providers in this module keep the runtime independent from ROS and from
USRP hardware. Hardware deployments can feed the same interface from OAI E2,
shared-memory hooks, or log exporters. Test and RF-simulator deployments can
feed JSON records from files or UDP.
"""
from __future__ import annotations

import json
import socket
import time
from dataclasses import dataclass
from multiprocessing import shared_memory
from pathlib import Path
from typing import Any, Dict, Iterable, Iterator, Optional

from mobile_ros.radio import RadioDriverBase, RadioMetrics


@dataclass(frozen=True)
class OaiMetricRecord:
    timestamp: float
    ue_id: str
    rnti: str
    sinr_db: float
    rsrp_dbm: float
    rsrq_db: float
    prb_util: float
    mcs: int
    throughput_mbps: float
    latency_ms: float
    packet_loss_pct: float
    jitter_ms: float

    @classmethod
    def from_mapping(cls, data: Dict[str, Any]) -> "OaiMetricRecord":
        now = time.time()
        return cls(
            timestamp=float(data.get("timestamp", now)),
            ue_id=str(data.get("ue_id", data.get("ue", "ue-0"))),
            rnti=str(data.get("rnti", "0x0000")),
            sinr_db=float(data.get("sinr_db", data.get("snr_db", 0.0))),
            rsrp_dbm=float(data.get("rsrp_dbm", -95.0)),
            rsrq_db=float(data.get("rsrq_db", -10.0)),
            prb_util=float(data.get("prb_util", data.get("prb_utilization", 0.0))),
            mcs=int(data.get("mcs", data.get("mcs_index", 0))),
            throughput_mbps=float(data.get("throughput_mbps", data.get("link_rate_mbps", 0.0))),
            latency_ms=float(data.get("latency_ms", data.get("ul_latency_ms", 0.0))),
            packet_loss_pct=float(data.get("packet_loss_pct", data.get("packet_loss_rate_pct", 0.0))),
            jitter_ms=float(data.get("jitter_ms", 0.0)),
        )

    def as_radio_metrics(self) -> RadioMetrics:
        return RadioMetrics(
            rsrp_dbm=self.rsrp_dbm,
            snr_db=self.sinr_db,
            prb_util=self.prb_util,
            timestamp=self.timestamp,
        )

    def as_policy_mapping(self) -> Dict[str, float]:
        return {
            "throughput_mbps": self.throughput_mbps,
            "latency_ms": self.latency_ms,
            "packet_loss_pct": self.packet_loss_pct,
            "jitter_ms": self.jitter_ms,
            "prb_allocation_pct": self.prb_util * 100.0 if self.prb_util <= 1.0 else self.prb_util,
            "snr_db": self.sinr_db,
            "rsrp_dbm": self.rsrp_dbm,
        }


class OaiMetricProvider:
    def read_record(self) -> OaiMetricRecord:
        raise NotImplementedError


class OaiMetricDriver(RadioDriverBase):
    def __init__(self, provider: OaiMetricProvider) -> None:
        self.provider = provider

    def read_metrics(self) -> RadioMetrics:
        return self.provider.read_record().as_radio_metrics()


class JsonlMetricProvider(OaiMetricProvider):
    """Replay OAI metrics from JSON Lines files."""

    def __init__(self, path: str | Path, loop: bool = True, interval_s: float = 0.0) -> None:
        self.path = Path(path)
        self.loop = loop
        self.interval_s = interval_s
        self._records = list(self._load())
        if not self._records:
            raise ValueError(f"no metric records found in {self.path}")
        self._index = 0

    def _load(self) -> Iterable[OaiMetricRecord]:
        with self.path.open("r", encoding="utf-8") as handle:
            for line in handle:
                line = line.strip()
                if line:
                    yield OaiMetricRecord.from_mapping(json.loads(line))

    def read_record(self) -> OaiMetricRecord:
        if self._index >= len(self._records):
            if not self.loop:
                self._index = len(self._records) - 1
            else:
                self._index = 0
        record = self._records[self._index]
        self._index += 1
        if self.interval_s > 0:
            time.sleep(self.interval_s)
        return record


class UdpMetricProvider(OaiMetricProvider):
    """Receive one JSON metric record per UDP datagram."""

    def __init__(self, bind: tuple[str, int] = ("127.0.0.1", 62000), timeout_s: float = 1.0) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(bind)
        self.sock.settimeout(timeout_s)
        self.last_record: Optional[OaiMetricRecord] = None

    def read_record(self) -> OaiMetricRecord:
        try:
            payload, _ = self.sock.recvfrom(8192)
            self.last_record = OaiMetricRecord.from_mapping(json.loads(payload.decode("utf-8")))
        except socket.timeout:
            if self.last_record is None:
                self.last_record = OaiMetricRecord.from_mapping({})
        return self.last_record


class SharedMemoryMetricProvider(OaiMetricProvider):
    """Read a JSON metric blob from a shared-memory segment.

    The OAI-side hook writes a UTF-8 JSON object followed by NUL bytes into the
    segment. This keeps the contract stable across C, C++, and Python writers.
    """

    def __init__(self, name: str = "mobileros_oai_metrics", size: int = 4096) -> None:
        self.name = name
        self.size = size
        self.shm = shared_memory.SharedMemory(name=name)

    def read_record(self) -> OaiMetricRecord:
        raw = bytes(self.shm.buf[: self.size]).split(b"\0", 1)[0]
        if not raw:
            return OaiMetricRecord.from_mapping({})
        return OaiMetricRecord.from_mapping(json.loads(raw.decode("utf-8")))


def records_from_sequence(records: Iterable[Dict[str, Any]]) -> Iterator[OaiMetricRecord]:
    for record in records:
        yield OaiMetricRecord.from_mapping(record)
