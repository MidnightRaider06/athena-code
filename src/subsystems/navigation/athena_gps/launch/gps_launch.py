from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_dir = get_package_share_directory('athena_gps')
    config_file = os.path.join(pkg_dir, 'config', 'pixhawk_params.yaml')

    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        choices=['true', 'false'],
        description='Use simulation bridge instead of real hardware'
    )

    pixhawk_node = Node(
        package='athena_gps',
        executable='athena_gps',
        name='athena_gps',
        output='screen',
        parameters=[
            config_file,
        ],
        respawn=True,
        respawn_delay=2.0,
        condition=UnlessCondition(LaunchConfiguration('sim'))
    )

    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='imu_bridge',
        output='screen',
        arguments=[
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        condition=IfCondition(LaunchConfiguration('sim'))
    )

    gps_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gps_bridge',
        output='screen',
        arguments=[
            '/gps/fix@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat',
        ],
        condition=IfCondition(LaunchConfiguration('sim'))
    )
    

    return LaunchDescription([
        sim_arg,
        pixhawk_node,
        imu_bridge,
        gps_bridge,
    ])
