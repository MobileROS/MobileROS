import unittest
from pathlib import Path

from mobile_ros.oai import JsonlMetricProvider, OaiMetricRecord


class OaiProviderTests(unittest.TestCase):
    def test_metric_record_mapping(self):
        record = OaiMetricRecord.from_mapping(
            {
                "ue_id": "ue-a",
                "rnti": "0x1",
                "snr_db": 12.5,
                "rsrp_dbm": -88,
                "prb_util": 0.4,
                "link_rate_mbps": 6.2,
                "ul_latency_ms": 25.0,
                "packet_loss_rate_pct": 1.1,
            }
        )
        self.assertEqual(record.sinr_db, 12.5)
        self.assertEqual(record.as_radio_metrics().snr_db, 12.5)
        self.assertEqual(record.as_policy_mapping()["prb_allocation_pct"], 40.0)

    def test_jsonl_provider_replays_records(self):
        provider = JsonlMetricProvider(Path("benchmarks/oai_rfsim_metrics.jsonl"), loop=False)
        first = provider.read_record()
        second = provider.read_record()
        self.assertEqual(first.ue_id, "ue-slam-01")
        self.assertGreater(second.throughput_mbps, first.throughput_mbps)


if __name__ == "__main__":
    unittest.main()
