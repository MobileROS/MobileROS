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
        ]
    )
