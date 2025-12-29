# athena_planner/launch/nav2.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetRemap
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # --- Packages / paths ---
    athena_map_share = get_package_share_directory('athena_map')
    dem_launch = os.path.join(athena_map_share, 'launch', 'dem_costmap.launch.py')

    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    nav2_nav = os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')

    default_params = PathJoinSubstitution([
        FindPackageShare('athena_planner'), 'config', 'nav2_params.yaml'
    ])
    params_file = LaunchConfiguration('params_file')

    # --- Frame names (override if your robot uses different ones) ---
    map_frame  = LaunchConfiguration('map_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')

    # --- Static TFs (identity transforms for bring-up/testing) ---
    # map -> odom
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        # args: x y z roll pitch yaw parent child
        arguments=['400', '400', '0', '0', '0', '0', map_frame, odom_frame],
        output='screen',
    )

    twist_stamper_node = Node(
        package='twist_stamper',
        executable='twist_stamper',
        name='cmd_vel_stamper',
        remappings=[
            ('cmd_vel_in',  '/cmd_vel_nav2'),                      
            ('cmd_vel_out', '/ackermann_steering_controller/reference'),
        ],
    )
    
    return LaunchDescription([
        # Params
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Full path to the Nav2 params YAML'
        ),
        DeclareLaunchArgument('map_frame',  default_value='map'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),

        SetRemap(src='cmd_vel', dst='/cmd_vel_nav2'),

        # Static TFs (identity by default)
        static_map_to_odom,
        twist_stamper_node,

        IncludeLaunchDescription(PythonLaunchDescriptionSource(dem_launch)),

        # Nav2 core stack (planner/controller/smoother/behavior/BT nav + lifecycle)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_nav),
            launch_arguments={
                'params_file': params_file,
                'use_sim_time': 'true',
                'autostart': 'true'
            }.items()
        ),
    ])
