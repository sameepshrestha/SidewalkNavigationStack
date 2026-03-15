#!/usr/bin/env python3
"""
Master Navigation Launch File — FrodoBots Competition

Starts all nodes required for autonomous navigation:
  1. Bridge (GPS, IMU, cameras, wheel odom) + Static TF
  2. Localization (heading_fusion & simple robot_localization)
  3. Perception (DA3 segmentation -> traversable cloud)
  4. Mapping (Traversability occupancy grid)
  5. Waypoint Follower (MPPI Goal Director)
  6. Nav2 MPPI
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    nav_pkg = FindPackageShare('frodobot_navigation')
    perc_pkg = FindPackageShare('frodobot_perception_ros')
    loc_pkg = FindPackageShare('frodobot_localization')

    route_file = LaunchConfiguration('route_file')
    route_file_arg = DeclareLaunchArgument(
        'route_file',
        default_value=PathJoinSubstitution([nav_pkg, 'config', 'route.yaml']),
        description='Path to the route config file (start/goal GPS points)'
    )

    nav2_params = PathJoinSubstitution([nav_pkg, 'config', 'nav2_params.yaml'])

    return LaunchDescription([
        route_file_arg,

        # ── 1. FrodoBots SDK Bridge ────────────────────────────────────────────────
        Node(
            package='frodobot_bridge',
            executable='sdk_bridges',
            name='earth_rover_bridge',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('frodobot_bridge'), 'config', 'bridge_params.yaml')]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('frodobot_bridge'), 'urdf', 'earthrover.urdf.xacro')])
            }]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([loc_pkg, 'launch', 'localization.launch.py']))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([perc_pkg, 'launch', 'perception.launch.py']))
        ),

        Node(
            package='frodobot_mapping',
            executable='traversability_map_node',
            name='traversability_map_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('frodobot_mapping'), 'config', 'mapping.yaml')]
        ),

        Node(
            package='frodobot_navigation',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[{
                'route_file': route_file,
                'goal_tolerance_m': 5.0,
                'gps_topic': '/earth_rover/gps/fix'
            }]
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params],
            remappings=[('cmd_vel', '/cmd_vel')],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_controller',
            output='screen',
            parameters=[{'use_sim_time': False, 'autostart': True, 'node_names': ['controller_server']}],
        ),
    ])
