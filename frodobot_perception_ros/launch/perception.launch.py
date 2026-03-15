"""Launch perception node in live mode (with frodobot_bridge)."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('frodobot_perception_ros')
    default_config = os.path.join(pkg_share, 'config', 'perception.yaml')

    return LaunchDescription([
        DeclareLaunchArgument("checkpoint", default_value="",
                              description="Path to model checkpoint"),
        DeclareLaunchArgument("config_file", default_value=default_config,
                              description="Path to perception.yaml"),

        Node(
            package="frodobot_perception_ros",
            executable="perception_node",
            name="perception_node",
            parameters=[
                LaunchConfiguration("config_file"),
                {
                    "checkpoint": LaunchConfiguration("checkpoint"),
                }
            ],
            output="screen",
        ),
    ])
