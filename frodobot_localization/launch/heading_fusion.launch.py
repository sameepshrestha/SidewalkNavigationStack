from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('frodobot_localization')
    default_config = os.path.join(pkg_share, 'config', 'heading_fusion.yaml')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to the YAML config file for heading_fusion_node.'
    )

    node = Node(
        package='frodobot_localization',
        executable='heading_fusion_node',
        name='heading_fusion_node',
        output='screen',
        parameters=[LaunchConfiguration('config')],
    )

    return LaunchDescription([config_arg, node])
