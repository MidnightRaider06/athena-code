from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'waypoints_file',
            default_value='~/.ros/waypoints.yaml',
            description='Path to YAML file for waypoint persistence'
        ),
        DeclareLaunchArgument(
            'load_from_file',
            default_value='true',
            description='Load saved waypoints from file on startup'
        ),
        Node(
            package='waypoint_manager',
            executable='waypoint_manager_node',
            name='waypoint_manager',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'waypoints_file': LaunchConfiguration('waypoints_file'),
                'load_from_file': LaunchConfiguration('load_from_file'),
            }],
            output='screen',
        ),
    ])
