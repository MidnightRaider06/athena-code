from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    sim = LaunchConfiguration('sim')

    declare_sim = DeclareLaunchArgument(
        'sim',
        default_value='false',
        choices=['true', 'false'],
        description='Use Gazebo bridge instead of real ZED hardware'
    )
    declare_camera_model = DeclareLaunchArgument(
        'camera_model',
        default_value='zed2i',
        description='ZED camera model'
    )
    declare_camera_name = DeclareLaunchArgument(
        'camera_name',
        default_value='zed',
        description='The name of the camera'
    )
    declare_node_name = DeclareLaunchArgument(
        'node_name',
        default_value='zed_node',
        description='The name of the zed_wrapper node'
    )
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Node namespace (defaults to camera_name if empty)'
    )
    declare_publish_urdf = DeclareLaunchArgument(
        'publish_urdf',
        default_value='true',
        choices=['true', 'false'],
        description='Enable URDF processing and Robot State Publisher'
    )
    declare_publish_tf = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        choices=['true', 'false'],
        description='Enable publication of odom -> camera_link TF'
    )
    declare_publish_map_tf = DeclareLaunchArgument(
        'publish_map_tf',
        default_value='true',
        choices=['true', 'false'],
        description='Enable publication of map -> odom TF'
    )
    declare_publish_imu_tf = DeclareLaunchArgument(
        'publish_imu_tf',
        default_value='false',
        choices=['true', 'false'],
        description='Enable publication of the IMU TF'
    )
    declare_svo_path = DeclareLaunchArgument(
        'svo_path',
        default_value='live',
        description='Path to an input SVO file'
    )
    declare_publish_svo_clock = DeclareLaunchArgument(
        'publish_svo_clock',
        default_value='false',
        choices=['true', 'false'],
        description='Publish SVO timestamp as clock'
    )
    declare_enable_gnss = DeclareLaunchArgument(
        'enable_gnss',
        default_value='false',
        choices=['true', 'false'],
        description='Enable ZED GNSS fusion'
    )
    declare_gnss_antenna_offset = DeclareLaunchArgument(
        'gnss_antenna_offset',
        default_value='[]',
        description='GNSS antenna offset [x,y,z] relative to ZED mounting point'
    )
    declare_enable_ipc = DeclareLaunchArgument(
        'enable_ipc',
        default_value='true',
        choices=['true', 'false'],
        description='Enable intra-process communication'
    )
    declare_sim_mode = DeclareLaunchArgument(
        'sim_mode',
        default_value='false',
        choices=['true', 'false'],
        description='Enable ZED SDK simulation mode (separate from sim bridge)'
    )
    declare_sim_address = DeclareLaunchArgument(
        'sim_address',
        default_value='127.0.0.1',
        description='ZED SDK simulation server address'
    )
    declare_sim_port = DeclareLaunchArgument(
        'sim_port',
        default_value='30000',
        description='ZED SDK simulation server port'
    )
    declare_stream_address = DeclareLaunchArgument(
        'stream_address',
        default_value='',
        description='Input streaming server address'
    )
    declare_stream_port = DeclareLaunchArgument(
        'stream_port',
        default_value='30000',
        description='Input streaming server port'
    )
    declare_serial_number = DeclareLaunchArgument(
        'serial_number',
        default_value='0',
        description='Serial number of the camera to open'
    )
    declare_camera_id = DeclareLaunchArgument(
        'camera_id',
        default_value='-1',
        description='ID of the camera to open'
    )
    declare_ros_params_override_path = DeclareLaunchArgument(
        'ros_params_override_path',
        default_value='',
        description='Path to a ROS params override YAML file'
    )
    declare_node_log_type = DeclareLaunchArgument(
        'node_log_type',
        default_value='both',
        choices=['screen', 'log', 'both'],
        description='Log output type for the ZED node'
    )

    zed_wrapper_launch_dir = os.path.join(
        get_package_share_directory('zed_wrapper'), 'launch'
    )

    zed_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_wrapper_launch_dir, 'zed_camera.launch.py')
        ),
        launch_arguments={
            'camera_model':             LaunchConfiguration('camera_model'),
            'camera_name':              LaunchConfiguration('camera_name'),
            'node_name':                LaunchConfiguration('node_name'),
            'namespace':                LaunchConfiguration('namespace'),
            'publish_urdf':             LaunchConfiguration('publish_urdf'),
            'publish_tf':               LaunchConfiguration('publish_tf'),
            'publish_map_tf':           LaunchConfiguration('publish_map_tf'),
            'publish_imu_tf':           LaunchConfiguration('publish_imu_tf'),
            'svo_path':                 LaunchConfiguration('svo_path'),
            'publish_svo_clock':        LaunchConfiguration('publish_svo_clock'),
            'enable_gnss':              LaunchConfiguration('enable_gnss'),
            'gnss_antenna_offset':      LaunchConfiguration('gnss_antenna_offset'),
            'enable_ipc':               LaunchConfiguration('enable_ipc'),
            'use_sim_time':             sim,
            'sim_mode':                 LaunchConfiguration('sim_mode'),
            'sim_address':              LaunchConfiguration('sim_address'),
            'sim_port':                 LaunchConfiguration('sim_port'),
            'stream_address':           LaunchConfiguration('stream_address'),
            'stream_port':              LaunchConfiguration('stream_port'),
            'serial_number':            LaunchConfiguration('serial_number'),
            'camera_id':                LaunchConfiguration('camera_id'),
            'ros_params_override_path': LaunchConfiguration('ros_params_override_path'),
            'node_log_type':            LaunchConfiguration('node_log_type'),
        }.items(),
        condition=UnlessCondition(sim),
    )

    return LaunchDescription([
        declare_sim,
        declare_camera_model,
        declare_camera_name,
        declare_node_name,
        declare_namespace,
        declare_publish_urdf,
        declare_publish_tf,
        declare_publish_map_tf,
        declare_publish_imu_tf,
        declare_svo_path,
        declare_publish_svo_clock,
        declare_enable_gnss,
        declare_gnss_antenna_offset,
        declare_enable_ipc,
        declare_sim_mode,
        declare_sim_address,
        declare_sim_port,
        declare_stream_address,
        declare_stream_port,
        declare_serial_number,
        declare_camera_id,
        declare_ros_params_override_path,
        declare_node_log_type,
        zed_real,
    ])
