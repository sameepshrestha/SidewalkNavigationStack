import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    loc_pkg = FindPackageShare('sidewalk_localization')
    
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('sidewalk_localization'), 'config', 'ekf.yaml')],
            remappings=[
                ('odometry/filtered', '/odometry/local'),
            ]
        ),
        
        Node(
            package='frodobot_localization',
            executable='heading_fusion_node',
            name='heading_fusion_node',
            output='screen',
            parameters=[
                os.path.join(get_package_share_directory('frodobot_localization'), 'config', 'heading_fusion.yaml')
            ]
        ),
    ])
