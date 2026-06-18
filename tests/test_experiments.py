import unittest

from mobile_ros.experiments import (
    compare_against_baseline,
    load_baselines,
    paper_slam_policy_trace,
    replay_from_paper_baseline,
    summarize_comparisons,
)


class ExperimentReplayTests(unittest.TestCase):
    def test_replay_matches_paper_baseline(self):
        baselines = load_baselines()
        observed = replay_from_paper_baseline(baselines)
        comparisons = compare_against_baseline(observed, baselines, mode="replay")
        summary = summarize_comparisons(comparisons)
        self.assertGreater(summary["total"], 50)
        self.assertEqual(summary["failed"], 0)

    def test_slam_policy_trace_contains_all_prb_conditions(self):
        trace = paper_slam_policy_trace()
        self.assertEqual([item["condition"] for item in trace], ["low_prb_30", "medium_prb_60", "high_prb_90"])
        self.assertLess(trace[0]["paper_pose_error_m"], 0.5)
        self.assertGreater(trace[-1]["snapshot"]["throughput_mbps"], trace[0]["snapshot"]["throughput_mbps"])


if __name__ == "__main__":
    unittest.main()
