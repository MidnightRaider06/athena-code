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

    # you can still expose the DEM path (if you ever want to override it)
    dem_file_arg = DeclareLaunchArgument(
        'dem_file_path',
        default_value='',
        description='Path to the DEM TIFF file'
    )

    dem_node = Node(
        package='athena_map',
        executable='map_node',
        name='dem_costmap_converter',
        output='screen',
        parameters=[config_file,  # load everything from YAML
                    { 'dem_file_path': LaunchConfiguration('dem_file_path') }  # override just this one
                   ]
    )

    return LaunchDescription([
        dem_file_arg,
        dem_node
    ])
