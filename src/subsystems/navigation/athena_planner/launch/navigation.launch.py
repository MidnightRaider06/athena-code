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
    
    # TODO: Re-enable the map functionality once the localizer works. Also needs corresponding changes in the Nav2 config file.
    #athena_map_share = get_package_share_directory('athena_map')
    #dem_launch = os.path.join(athena_map_share, 'launch', 'dem_costmap.launch.py')

    athena_planner_share = get_package_share_directory('athena_planner')
    nav2_nav = os.path.join(athena_planner_share, 'launch', 'nav2_nodes.launch.py')
    
    localizer_share = get_package_share_directory('localizer')
    localizer_launch_file = os.path.join(localizer_share, 'launch', 'localizer.launch.py')
    
    default_params = PathJoinSubstitution([
        FindPackageShare('athena_planner'), 'config', 'nav2_params.yaml'
    ])
    params_file = LaunchConfiguration('params_file')

    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    twist_stamper_node = Node(
        package='twist_stamper',
        executable='twist_stamper',
        name='cmd_vel_stamper',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('cmd_vel_in', '/cmd_vel_nav'),
            ('cmd_vel_out', '/ackermann_steering_controller/reference'),
        ],
    )
    
    localizer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localizer_launch_file),
        launch_arguments={
            'use_sim_time': 'true',
        }.items()
    )

    

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Full path to the Nav2 params YAML'
        ),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('use_respawn', default_value='False',
            description='Whether to respawn if a node crashes'),
        DeclareLaunchArgument('log_level', default_value='info',
            description='Log level for nav2 nodes'),

        twist_stamper_node,
        localizer_launch,

       #IncludeLaunchDescription(PythonLaunchDescriptionSource(dem_launch)),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_nav),
            launch_arguments={
                'params_file': params_file,
                'use_sim_time': 'true',
                'autostart': 'true',
                'use_respawn': use_respawn,
                'log_level': log_level
            }.items()
        ),
    ])