from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    athena_map_share = get_package_share_directory('athena_map')
    dem_launch_file = os.path.join(athena_map_share, 'launch', 'dem_costmap.launch.py')

    
    athena_planner_share = get_package_share_directory('athena_planner')
    nav2_nav = os.path.join(athena_planner_share, 'launch', 'nav2_nodes.launch.py')
    
    localizer_share = get_package_share_directory('localizer')
    localizer_launch_file = os.path.join(localizer_share, 'launch', 'localizer.launch.py')

    gps_goal_share = get_package_share_directory('gps_goal')
    gps_goal_launch_file = os.path.join(gps_goal_share, 'launch', 'gps_goal_server.launch.py')

    sensors_share = get_package_share_directory('athena_sensors')
    sensors_launch_file = os.path.join(sensors_share, 'launch', 'sensors.launch.py')

    default_params = PathJoinSubstitution([
        FindPackageShare('athena_planner'), 'config', 'nav2_params.yaml'
    ])

    sim = LaunchConfiguration('sim')
    params_file = LaunchConfiguration('params_file')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    twist_stamper_node = Node(
        package='twist_stamper',
        executable='twist_stamper',
        name='cmd_vel_stamper',
        parameters=[{'use_sim_time': sim}],
        remappings=[
            ('cmd_vel_in', '/cmd_vel_nav'),
            ('cmd_vel_out', '/ackermann_steering_controller/reference'),
        ],
    )


    dem_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(dem_launch_file),
        condition=IfCondition(LaunchConfiguration('use_dem'))
    )
    localizer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localizer_launch_file),
        launch_arguments={'sim': sim}.items(),
    )

    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sensors_launch_file),
        launch_arguments={'sim': sim}.items(),
    )

    gps_goal_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gps_goal_launch_file),
        launch_arguments={'use_sim_time': sim}.items(),
    )

    point_cloud_filterer_sim = Node(
        package='point_cloud_filterer',
        executable='point_cloud_filtered',
        name='point_cloud_filterer',
        parameters=[{
            'use_sim_time': sim,
            'input_topic': '/zed/zed_node/point_cloud/cloud_registered',
            'output_topic': '/zed/zed_node/point_cloud/cloud_registered_corrected',
            'frame_override': 'zed_left_camera_frame_optical'
        }],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(sim),
    )

    point_cloud_relay = Node(
        package='topic_tools',
        executable='relay',
        name='point_cloud_relay',
        parameters=[{'use_sim_time': sim}],
        arguments=[
            '/zed/zed_node/point_cloud/cloud_registered',
            '/zed/zed_node/point_cloud/cloud_registered_corrected',
        ],
        output='screen',
        condition=UnlessCondition(sim),
    )
    

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim',
            default_value='false',
            choices=['true', 'false'],
            description='Set true when running in Gazebo simulation (enables sim time and sensor bridges)',
        ),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Full path to the Nav2 params YAML',
        ),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument(
            'use_respawn', default_value='False',
            description='Whether to respawn if a node crashes',
        ),
        DeclareLaunchArgument('log_level', default_value='info',
            description='Log level for nav2 nodes'),
        DeclareLaunchArgument('use_dem', default_value='false',
            choices=['true', 'false'],
            description='Enable DEM costmap layer'),

        twist_stamper_node,
        dem_launch,
        localizer_launch,
        sensors_launch,
        point_cloud_filterer_sim,
        point_cloud_relay,
        gps_goal_launch,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_nav),
            launch_arguments={
                'params_file': params_file,
                'use_sim_time': sim,
                'autostart': 'true',
                'use_respawn': use_respawn,
                'log_level': log_level,
            }.items(),
        ),
    ])
