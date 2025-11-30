from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription      # CORRECT
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
def generate_launch_description():

    robot_mode = LaunchConfiguration('robot_mode')
    perception_mode = LaunchConfiguration('perception_mode')
    pkg_share = get_package_share_directory('sidewalk_main')
    pkg_share_loc = get_package_share_directory('sidewalk_localization')
    pkg_share_ui = get_package_share_directory('sidewalk_ui')
    joy_config = os.path.join(pkg_share, 'config', 'ps5controller.yaml')
    mux_config = os.path.join(pkg_share, 'config', 'mux.yaml')
    boxx_urdf = os.path.join(pkg_share, 'config', 'boxx.urdf')  # 

    arg_robot = DeclareLaunchArgument(
        'robot_mode',
        default_value='boxx',
        description='Options: frodobot, boxx'
    )
    tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['0.24', '0', '0.355', '0', '0', '0', 'base_link', 'imu_link']
    )
    
    tf_gps = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['0', '0', '0.5', '0', '0', '0', 'base_link', 'gps_link']
    )

    arg_perception = DeclareLaunchArgument(
        'perception_mode',
        default_value='ipm',
        description='Options: ipm, depth, hybrid'
    )
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': ParameterValue(Command(['cat ', boxx_urdf]),value_type = str)}],
        condition=IfCondition(PythonExpression(["'", robot_mode, "' == 'boxx'"]))
    )
    
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sidewalk_localization'), 
                         'launch', 'localization.launch.py')
        )
    )
    ui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share_ui, 'launch', 'sidewalk_ui.launch.py')
        )
    )
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'dev': '/dev/input/js0', 'deadzone': 0.1, 'autorepeat_rate': 20.0}]
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_ps5_driver',  
        parameters=[{
            'axis_linear.x': 1,    
            'axis_linear': 1,    
            
            'scale_linear.x': 0.5,  
            'scale_linear': 0.5,

            'axis_angular.yaw': 0, 
            'axis_angular': 0,      
            
            'scale_angular.yaw': 1.0, 
            'scale_angular': 1.0,

            'require_enable_button': True,
            'enable_button': 4,     # L1
            'deadzone': 0.1
        }],
        remappings=[('/cmd_vel', '/cmd_vel_joy')]
    )

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        parameters=[mux_config],
        remappings=[('/cmd_vel_out', '/cmd_vel')])
    
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
    node_osm_planner = Node(
        package='sidewalk_osmplanner',
        executable='osm_planner',
        name='osm_planner',
        output='screen'  # Added so you can see your prints
    )
    
    
    
    rviz_config_path = '/home/sameep/phd_research/sidewalkauto_ws/src/sidewalk_main/launch/sidewalk_nav.rviz'
    rviz_args = ['-d', rviz_config_path] if os.path.exists(rviz_config_path) else []

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=rviz_args
    )

    node_depth_estimation = Node(
        package='sidewalk_depthestimation',
        executable='depth_node', 
        name='depth_estimator',
        output='screen',
        # Check python script for subscriber topic. Usually /camera/front/image_raw
        remappings=[
            ('/camera/front/image_raw', '/camera/front/image_raw') 
        ],
    )
    
    return LaunchDescription([
        arg_robot,
        arg_perception,
        joy_node,
        teleop_node,
        twist_mux_node,
        localization,
        ui_launch,
        tf_imu,
        tf_gps,
        node_robot_state_publisher,
        node_boxx,
        node_frodobot,
        node_depth_estimation,
        rviz,
        node_osm_planner

    ])
