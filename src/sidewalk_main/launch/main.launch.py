from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    robot_mode = LaunchConfiguration('robot_mode')
    perception_mode = LaunchConfiguration('perception_mode')
    pkg_share = get_package_share_directory('sidewalk_main')
    joy_config = os.path.join(pkg_share, 'config', 'ps5controller.yaml')
    mux_config = os.path.join(pkg_share, 'config', 'mux.yaml')

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
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'dev': '/dev/input/js0', 'deadzone': 0.1, 'autorepeat_rate': 20.0}]
    )

    # B. Converts raw USB to Velocity commands (on /cmd_vel_joy)
# B. Converts raw USB to Velocity commands (on /cmd_vel_joy)
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_ps5_driver',  # CHANGED NAME TO KILL ZOMBIES
        parameters=[{
            # Main mapping
            'axis_linear.x': 1,     # Force Specific Forward Axis
            'axis_linear': 1,       # Backup
            
            'scale_linear.x': 0.5,  # Force Specific Scale
            'scale_linear': 0.5,

            'axis_angular.yaw': 3,  # Force Specific Turn Axis
            'axis_angular': 3,      # Backup
            
            'scale_angular.yaw': 1.0, # Force Specific Scale
            'scale_angular': 1.0,

            # Button Config
            'require_enable_button': True,
            'enable_button': 4,     # L1
            'deadzone': 0.1
        }],
        remappings=[('/cmd_vel', '/cmd_vel_joy')]
    )
    # C. The Traffic Cop (Mux)
    # Listens to /cmd_vel_auto (Bridge) and /cmd_vel_joy (PS5)
    # Outputs to /cmd_vel (Real Wheels)
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        parameters=[mux_config],
        remappings=[('/cmd_vel_out', '/cmd_vel')])
    
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
        joy_node,
        teleop_node,
        twist_mux_node,
        node_boxx,
        node_frodobot,
        rviz,

    ])