from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

import os


def generate_launch_description():
    sim = LaunchConfiguration('sim')
    camera_model = LaunchConfiguration('camera_model')

    declare_sim = DeclareLaunchArgument(
        'sim',
        default_value='false',
        choices=['true', 'false'],
        description='Use simulation bridges instead of real hardware'
    )
    declare_camera_model = DeclareLaunchArgument(
        'camera_model',
        default_value='zed2i',
        description='ZED camera model'
    )

    athena_sensors_launch_dir = os.path.join(
        get_package_share_directory('athena_sensors'), 'launch'
    )
    athena_gps_launch_dir = os.path.join(
        get_package_share_directory('athena_gps'), 'launch'
    )

    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(athena_sensors_launch_dir, 'zed.launch.py')
        ),
        launch_arguments={
            'sim':             sim,
            'camera_model':    camera_model,
            'publish_tf':      'true',
            'publish_map_tf':  'false',
            'publish_urdf':   'false',
        }.items(),
        condition=UnlessCondition(sim),
    )

    gps_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(athena_gps_launch_dir, 'gps_launch.py')
        ),
        launch_arguments={
            'sim': sim,
        }.items(),
    )

    return LaunchDescription([
        declare_sim,
        declare_camera_model,
        zed_launch,
        gps_launch,
    ])
