from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='waypoint_manager',
            executable='waypoint_manager_node',
            name='waypoint_manager',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'map_frame': LaunchConfiguration('map_frame'),
            }],
            output='screen',
        ),
    ])
