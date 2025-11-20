from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


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

    node_boxx = Node(
        package='robot_description',
        executable='boxx_node',
        name='boxx_main',
        condition=IfCondition(
            PythonExpression([f'"{robot_mode}" == "boxx"'])
        )
    )

    node_frodobot = Node(
        package='robot_description',
        executable='frodobot_node',
        name='frodobot_main',   
        condition=IfCondition(
            PythonExpression([f'"{robot_mode}" == "frodobot"'])
        )
    )

    node_ipm = Node(
        package='perception_stack',
        executable='ipm_node',
        name='ipm_perception',
        condition=IfCondition(
            PythonExpression([f'"{perception_mode}" == "ipm"'])
        )
    )

    node_depth = Node(
        package='perception_stack',
        executable='depth_node',
        name='depth_perception',
        condition=IfCondition(
            PythonExpression([f'"{perception_mode}" == "depth"'])
        )
    )

    node_hybrid = Node(
        package='perception_stack',
        executable='hybrid_node',
        name='hybrid_perception',
        condition=IfCondition(
            PythonExpression([f'"{perception_mode}" == "hybrid"'])
        )
    )
    rviz = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', '/path/to/sidewalk_nav.rviz']
)

    return LaunchDescription([
        arg_robot,
        arg_perception,

        node_boxx,
        node_frodobot,
        rviz,
        node_ipm,
        node_depth,
        node_hybrid
    ])
