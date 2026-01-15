from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_sim = get_package_share_directory('simulation')

    return LaunchDescription([
        DeclareLaunchArgument(
            'child_frame_id',
            default_value='base_footprint',
            description='Child frame ID for the transform (default: base_footprint)'
        ),

        Node(
            package='simulation',
            executable='ground_truth_tf_publisher.py',
            name='ground_truth_tf_publisher',
            output='screen',
            parameters=[{
                'child_frame_id': LaunchConfiguration('child_frame_id')
            }]
        ),
    ])
