import unittest

from mobile_ros.policies import (
    NetworkQuality,
    NetworkSnapshot,
    SliceAction,
    adaptive_jpeg_quality,
    adaptive_resolution_scale,
    adaptive_voxel_leaf_size,
    build_stream_policy,
    elastic_stream_keyframe_limit,
)


class PolicyTests(unittest.TestCase):
    def test_elastic_stream_limits(self):
        self.assertEqual(elastic_stream_keyframe_limit(1.0), 5)
        self.assertEqual(elastic_stream_keyframe_limit(7.2), 14)
        self.assertEqual(elastic_stream_keyframe_limit(99.0), 20)

    def test_adaptive_visual_policy_bounds(self):
        self.assertEqual(adaptive_jpeg_quality(0.0), 30)
        self.assertEqual(adaptive_jpeg_quality(20.0), 95)
        self.assertEqual(adaptive_resolution_scale(3.0), 0.50)
        self.assertEqual(adaptive_resolution_scale(6.0), 0.75)
        self.assertEqual(adaptive_resolution_scale(9.0), 1.00)

    def test_adaptive_voxel_matches_formula(self):
        self.assertEqual(adaptive_voxel_leaf_size(20.0), 0.100)
        self.assertEqual(adaptive_voxel_leaf_size(3.0), 0.228)

    def test_critical_task_promotes_slice(self):
        snapshot = NetworkSnapshot(
            throughput_mbps=4.67,
            latency_ms=44.7,
            packet_loss_pct=3.2,
            jitter_ms=14.0,
            prb_allocation_pct=30.0,
        )
        policy = build_stream_policy(snapshot, task_criticality=0.9)
        self.assertIn(policy.quality, {NetworkQuality.POOR, NetworkQuality.MODERATE})
        self.assertEqual(policy.slice_action, SliceAction.PROMOTE)
        self.assertLessEqual(policy.jpeg_quality, 65)


if __name__ == "__main__":
    unittest.main()
