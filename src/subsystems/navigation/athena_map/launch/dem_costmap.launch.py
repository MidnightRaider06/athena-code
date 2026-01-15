#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('athena_map')
    config_file = os.path.join(pkg_dir, 'config', 'dem_costmap.yaml')

    dem_node = Node(
        package='athena_map',
        executable='map_node',
        name='dem_costmap_converter',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        dem_node
    ])
