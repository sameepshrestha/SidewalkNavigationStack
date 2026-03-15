from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_name = 'sidewalk_localization'
    pkg_share = get_package_share_directory(pkg_name)

    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')

    ekf_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            ('odometry/filtered', '/odometry/local'),
        ]
    )

    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            ('gps/fix', '/earth_rover/gps/fix'),
            ('imu/data', '/earth_rover/imu/data_raw'),       # harmless if use_odometry_yaw=true
            ('odometry/filtered', '/odometry/local'),
            ('odometry/gps', '/odometry/gps'),
        ]
    )


    ekf_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            ('odometry/filtered', '/odometry/global'),
        ]
    )

    return LaunchDescription([
        ekf_local_node,
        navsat_transform_node,
        ekf_global_node,
    ])