#!/usr/bin/env python3
"""
frodobot_navigation — MPPI test launch.

Starts ONLY the Nav2 controller server (MPPI) + costmap.
Does NOT start perception/mapping/bridge — those run in their own terminals.

Prerequisites running (in other terminals):
  1.  frodobot_perception_ros perception_node  (publishes traversable_cloud)
  2.  frodobot_mapping        mapping.launch   (publishes /mapping/traversability)
  3.  static_transform_publisher map→odom→base_link  (for testing without GPS)

To test: publish a goal manually:
  ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
    '{header: {frame_id: "map"}, pose: {position: {x: 5.0, y: 0.0}}}'

Watch /cmd_vel for output.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg = FindPackageShare("frodobot_navigation")
    nav2_params = PathJoinSubstitution([pkg, "config", "nav2_params.yaml"])

    # fake_tf:=true  → collapse map=odom=base_link at origin (testing without robot)
    # fake_tf:=false → robot_localization publishes odom→base_link + navsat publishes map→odom
    fake_tf_arg = DeclareLaunchArgument(
        "fake_tf", default_value="true",
        description="Publish fake map→odom→base_link TF for standalone testing. "
                    "Set false when robot_localization is running."
    )

    return LaunchDescription([
        fake_tf_arg,

        # ── Fake TF: map → odom → base_link ───────────────────────────────────
        # ONLY active when fake_tf:=true (standalone testing without robot).
        # When robot_localization is running (live robot), set fake_tf:=false.
        # frodobot_bridge/static_tf already handles: base_link→cameras/IMU/GPS.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            condition=IfCondition(LaunchConfiguration("fake_tf")),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="odom_to_base_link",
            arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
            condition=IfCondition(LaunchConfiguration("fake_tf")),
        ),

        # ── Nav2 Controller Server (MPPI) ─────────────────────────────────────
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[nav2_params],
            remappings=[
                # MPPI reads the cmd_vel topic — remap if needed
                ("cmd_vel", "/cmd_vel"),
            ],
        ),

        # ── Lifecycle Manager: starts Nav2 nodes in correct order ─────────────
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_controller",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "autostart": True,
                "node_names": ["controller_server"],
            }],
        ),
    ])
