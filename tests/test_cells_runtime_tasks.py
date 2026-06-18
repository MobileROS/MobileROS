import unittest

from mobile_ros.cells import (
    CameraAdaptiveCell,
    LidarAdaptiveCell,
    SlamElasticStreamCell,
    V2XSafetyCell,
    synthetic_cloud,
    synthetic_image,
)
from mobile_ros.policies import NetworkSnapshot, build_stream_policy
from mobile_ros.runtime import MobileRosRuntime, SequenceMetricProvider
from mobile_ros.slicing import RecordingSliceClient
from mobile_ros.tasks.lidar_perception import AdaptiveVoxelCase
from mobile_ros.tasks.multi_robot import MultiRobotCoordinator, RobotPeer
from mobile_ros.tasks.partition import PartitionPlanner
from mobile_ros.tasks.v2x import V2XSafetyCase
from mobile_ros.tasks.visual_slam import ElasticVisualSlamCase
from mobile_ros.types import RuntimeMode, TaskState, TaskType


METRICS = [
    {
        "ue_id": "ue-test",
        "throughput_mbps": 4.67,
        "latency_ms": 44.7,
        "packet_loss_pct": 3.2,
        "jitter_ms": 14.0,
        "prb_util": 0.3,
    },
    {
        "ue_id": "ue-test",
        "throughput_mbps": 9.9,
        "latency_ms": 15.3,
        "packet_loss_pct": 0.5,
        "jitter_ms": 4.3,
        "prb_util": 0.9,
    },
]


class CellsRuntimeTasksTests(unittest.TestCase):
    def test_camera_and_slam_cells_emit_adapted_outputs(self):
        policy = build_stream_policy(
            NetworkSnapshot(throughput_mbps=4.67, latency_ms=44.7, packet_loss_pct=3.2, jitter_ms=14.0, prb_allocation_pct=30.0)
        )
        camera = CameraAdaptiveCell()
        camera.update_policy(policy)
        camera_output = camera.process(synthetic_image(1), now=0.0)
        self.assertTrue(camera_output.accepted)
        self.assertLessEqual(camera_output.output.width, 640)

        slam = SlamElasticStreamCell()
        slam.update_policy(policy)
        slam_output = slam.process(camera_output.output, pose_delta_m=0.2, now=0.0)
        self.assertTrue(slam_output.output.keyframe)
        self.assertGreater(slam_output.output.feature_count, 0)

    def test_lidar_voxel_filter_reduces_points(self):
        cell = LidarAdaptiveCell()
        policy = build_stream_policy(
            NetworkSnapshot(throughput_mbps=3.0, latency_ms=50.0, packet_loss_pct=2.0, jitter_ms=10.0, prb_allocation_pct=30.0)
        )
        cell.update_policy(policy)
        cloud = synthetic_cloud(0, count=1000)
        output = cell.process(cloud, now=0.0)
        self.assertTrue(output.accepted)
        self.assertLess(len(output.output.points), len(cloud.points))

    def test_v2x_cell_stops_near_obstacle(self):
        cell = V2XSafetyCell()
        decision = cell.decide(obstacle_distance_m=0.9, requested_speed_mps=1.0, relative_speed_mps=1.0)
        self.assertFalse(decision.allow_motion)
        self.assertEqual(decision.target_speed_mps, 0.0)

    def test_runtime_sends_slice_commands(self):
        slice_client = RecordingSliceClient()
        runtime = MobileRosRuntime(
            SequenceMetricProvider(METRICS),
            mode=RuntimeMode.RFSIM,
            task_state=TaskState("slam", TaskType.VISUAL_SLAM, criticality=0.8, deadline_ms=70.0),
            slice_client=slice_client,
        )
        runtime.register_cell(CameraAdaptiveCell("camera"))
        steps = runtime.run_steps(2)
        self.assertEqual(len(steps), 2)
        self.assertEqual(len(slice_client.commands), 2)
        self.assertEqual(slice_client.commands[0].ue_id, "ue-test")

    def test_task_cases_run(self):
        slam_case = ElasticVisualSlamCase()
        slam = slam_case.run(METRICS, frame_count=6)
        self.assertGreater(slam_case.summarize(slam)["frames"], 0)

        lidar_case = AdaptiveVoxelCase()
        lidar = lidar_case.run(METRICS, frames=4, points_per_frame=800)
        self.assertGreaterEqual(lidar_case.summarize(lidar)["mean_reduction_ratio"], 0.0)

        v2x_case = V2XSafetyCase()
        v2x = v2x_case.run(METRICS, distances_m=[4.0, 0.8])
        self.assertEqual(v2x_case.summarize(v2x)["stops"], 1.0)

        coordinator = MultiRobotCoordinator([RobotPeer("robot-0", 1000, role="leader"), RobotPeer("robot-1", 900)])
        fragments = coordinator.run(METRICS, frames=4)
        self.assertGreater(coordinator.summarize(fragments)["fragments"], 0)

        decision = PartitionPlanner().decide_from_mapping(METRICS[1], task_criticality=0.7)
        self.assertIn("sensor_preprocess", decision.local_stages)


if __name__ == "__main__":
    unittest.main()
