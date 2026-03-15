"""Launch the traversability mapping node."""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory("frodobot_mapping")
    default_cfg = os.path.join(pkg, "config", "mapping.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config", default_value=default_cfg,
            description="Path to mapping.yaml"
        ),
        Node(
            package="frodobot_mapping",
            executable="traversability_map_node",
            name="traversability_map_node",
            output="screen",
            parameters=[LaunchConfiguration("config")],
        ),
    ])
