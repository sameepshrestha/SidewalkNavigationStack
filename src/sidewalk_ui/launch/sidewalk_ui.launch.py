import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    rosbridge_launch_dir = os.path.join(
        get_package_share_directory('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml')

    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(rosbridge_launch_dir)
    )

    web_video_server = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        parameters=[{
            'port': 8081,              
            'server_threads': 2,
            'ros_transport': 'best_effort' 
        }]
    ) # <--- THIS WAS MISSING BEFORE

    gps_converter = Node(
        package='sidewalk_ui',
        executable='gps_goal_sender',
        name='gps_goal_sender',
        output='screen'
    )

    return LaunchDescription([
        rosbridge,
        web_video_server,
        gps_converter
    ])