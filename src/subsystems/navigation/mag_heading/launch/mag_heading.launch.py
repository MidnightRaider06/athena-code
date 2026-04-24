from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('mag_heading'), 'config', 'mag_heading_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='mag_heading',
            executable='mag_heading_node',
            name='mag_heading_node',
            output='screen',
            parameters=[params],
            respawn=True,
            respawn_delay=2.0,
        )
    ])
