import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    launch_file_dir = os.path.dirname(os.path.realpath(__file__))
    sim_config = os.path.join(launch_file_dir, '..', 'config', 'localizer.yaml')
    real_config = os.path.join(launch_file_dir, '..', 'config', 'localizer_real.yaml')

    sim = LaunchConfiguration('sim')
    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim',
            default_value='false',
            choices=['true', 'false'],
            description='Select sim config (localizer.yaml) or real config (localizer_real.yaml)'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Robot namespace'
        ),
        Node(
            package='localizer',
            executable='localizer_node',
            namespace=namespace,
            name='localizer_node',
            parameters=[
                sim_config,
                {'use_sim_time': sim},
            ],
            output='screen',
            emulate_tty=True,
            condition=IfCondition(sim),
        ),
        Node(
            package='localizer',
            executable='localizer_node',
            namespace=namespace,
            name='localizer_node',
            parameters=[
                real_config,
                {'use_sim_time': sim},
            ],
            output='screen',
            emulate_tty=True,
            condition=UnlessCondition(sim),
        ),
    ])
