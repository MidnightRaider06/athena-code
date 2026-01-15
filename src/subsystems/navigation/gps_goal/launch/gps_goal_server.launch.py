#!/usr/bin/env python3
"""Launch file for GPS Goal Action Server."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for GPS goal server."""

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('gps_goal'),
            'config',
            'gps_goal_params.yaml'
        ]),
        description='Path to GPS goal server configuration file'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')

    gps_goal_server_node = Node(
        package='gps_goal',
        executable='gps_goal_server',
        name='gps_goal_server',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        respawn=True,
        respawn_delay=2.0,
    )

    return LaunchDescription([
        use_sim_time_arg,
        config_file_arg,
        gps_goal_server_node,
    ])
