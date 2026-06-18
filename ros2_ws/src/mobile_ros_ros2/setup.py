from setuptools import setup

package_name = "mobile_ros_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/slam_elastic_stream.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MobileROS Maintainers",
    maintainer_email="maintainer@example.com",
    description="ROS 2 adapters for MobileROS radio-aware policies.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "mobile_ros_hub = mobile_ros_ros2.mobile_ros_hub_node:main",
            "camera_cell = mobile_ros_ros2.camera_cell_node:main",
        ],
    },
)
