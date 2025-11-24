from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_name = 'sidewalk_localization'
    pkg_share = get_package_share_directory(pkg_name)

    ekf_config_file = os.path.join(pkg_share, 'config', 'ekf.yaml')


    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[ekf_config_file],
        remappings=[
            ('imu/data', '/imu/data'),          # Input IMU
            ('gps/fix', '/gps/data'),            # Input GPS
            ('odometry/gps', '/odometry/gps')   # Output X/Y (Intermediate topic)
        ]
    )


    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file],
        remappings=[
            ('odometry/filtered', '/odometry/global')  
        ]
    )

    return LaunchDescription([
        navsat_transform_node,
        ekf_node
    ])