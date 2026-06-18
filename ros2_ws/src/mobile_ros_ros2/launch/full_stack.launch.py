from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("metrics_source", default_value="jsonl"),
            DeclareLaunchArgument("metrics_jsonl", default_value="benchmarks/oai_rfsim_metrics.jsonl"),
            DeclareLaunchArgument("camera_in", default_value="/camera/image_raw"),
            DeclareLaunchArgument("camera_out", default_value="/camera/mobile_ros/image"),
            DeclareLaunchArgument("points_in", default_value="/points_raw"),
            DeclareLaunchArgument("points_out", default_value="/points/mobile_ros"),
            DeclareLaunchArgument("slice_host", default_value="127.0.0.1"),
            DeclareLaunchArgument("slice_port", default_value="63000"),
            Node(
                package="mobile_ros_ros2",
                executable="mobile_ros_hub",
                name="mobile_ros_hub",
                output="screen",
                parameters=[
                    {
                        "metrics_source": LaunchConfiguration("metrics_source"),
                        "metrics_jsonl": LaunchConfiguration("metrics_jsonl"),
                        "rate_hz": 10.0,
                    }
                ],
            ),
            Node(
                package="mobile_ros_ros2",
                executable="camera_cell",
                name="mobile_ros_camera_cell",
                output="screen",
                parameters=[
                    {
                        "input_topic": LaunchConfiguration("camera_in"),
                        "output_topic": LaunchConfiguration("camera_out"),
                    }
                ],
            ),
            Node(
                package="mobile_ros_ros2",
                executable="slam_task",
                name="mobile_ros_slam_task",
                output="screen",
                parameters=[{"input_topic": LaunchConfiguration("camera_out")}],
            ),
            Node(
                package="mobile_ros_ros2",
                executable="lidar_cell",
                name="mobile_ros_lidar_cell",
                output="screen",
                parameters=[
                    {
                        "input_topic": LaunchConfiguration("points_in"),
                        "output_topic": LaunchConfiguration("points_out"),
                    }
                ],
            ),
            Node(
                package="mobile_ros_ros2",
                executable="slice_client",
                name="mobile_ros_slice_client",
                output="screen",
                parameters=[
                    {
                        "slice_host": LaunchConfiguration("slice_host"),
                        "slice_port": LaunchConfiguration("slice_port"),
                    }
                ],
            ),
            Node(package="mobile_ros_ros2", executable="v2x_safety", name="mobile_ros_v2x_safety", output="screen"),
            Node(package="mobile_ros_ros2", executable="partition_node", name="mobile_ros_partition", output="screen"),
        ]
    )
