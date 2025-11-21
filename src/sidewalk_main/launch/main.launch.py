from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import os

def generate_launch_description():

    robot_mode = LaunchConfiguration('robot_mode')
    perception_mode = LaunchConfiguration('perception_mode')

    arg_robot = DeclareLaunchArgument(
        'robot_mode',
        default_value='boxx',
        description='Options: frodobot, boxx'
    )

    arg_perception = DeclareLaunchArgument(
        'perception_mode',
        default_value='ipm',
        description='Options: ipm, depth, hybrid'
    )

    # FIX 1: Correct syntax for checking conditions
    node_boxx = Node(
        package='sidewalk_bridge',
        executable='bridge_node',
        name='boxx_main',
        condition=IfCondition(
            PythonExpression(["'", robot_mode, "' == 'boxx'"]) 
        )
    )

    node_frodobot = Node(
        package='robot_description',
        executable='frodobot_node',
        name='frodobot_main',   
        condition=IfCondition(
            PythonExpression(["'", robot_mode, "' == 'frodobot'"])
        )
    )

    
    rviz_config_path = '/home/sameep/phd_research/sidewalkauto_ws/src/sidewalk_main/launch/sidewalk_nav.rviz'
    
    # Only use the argument if the file actually exists, otherwise run empty
    rviz_args = ['-d', rviz_config_path] if os.path.exists(rviz_config_path) else []

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=rviz_args
    )

    return LaunchDescription([
        arg_robot,
        arg_perception,
        node_boxx,
        node_frodobot,
        rviz,
    ])