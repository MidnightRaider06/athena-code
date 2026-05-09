from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    sim               = LaunchConfiguration('sim')
    use_zed_localizer = LaunchConfiguration('use_zed_localizer')
    enable_gnss       = LaunchConfiguration('enable_gnss')
    params_file       = LaunchConfiguration('params_file')
    use_dem           = LaunchConfiguration('use_dem')
    use_respawn       = LaunchConfiguration('use_respawn')
    log_level         = LaunchConfiguration('log_level')
    use_config        = LaunchConfiguration('use_config')

    use_localizer = PythonExpression(
        ["'false' if '", use_zed_localizer, "' == 'true' else 'true'"]
    )
    
    

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('description'), 'urdf', 'athena_drive.urdf.xacro'
        ]),
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': sim,
        }],
    )
    
    config_gui = Node(
        package='nav2_config',
        executable='gui',  
        name='nav2_config_gui',
        output='screen',
        condition=IfCondition(use_config),
    )

    navigation_launch_file = os.path.join(
        get_package_share_directory('athena_planner'), 'launch', 'navigation.launch.py'
    )

    default_params = PathJoinSubstitution([
        FindPackageShare('athena_planner'), 'config', 'nav2_params.yaml'
    ])

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_file),
        launch_arguments={
            'sim':           sim,
            'params_file':   params_file,
            'use_dem':       use_dem,
            'use_respawn':   use_respawn,
            'log_level':     log_level,
            'use_localizer': use_localizer,
            'enable_gnss':   enable_gnss,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim',
            default_value='false',
            choices=['true', 'false'],
            description='Set true when running in Gazebo simulation',
        ),
        DeclareLaunchArgument(
            'use_zed_localizer',
            default_value='false',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'enable_gnss',
            default_value='false',
            choices=['true', 'false'],
            description='Enable GNSS fusion inside the ZED camera',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to the Nav2 params YAML',
        ),
        DeclareLaunchArgument(
            'use_dem',
            default_value='false',
            choices=['true', 'false'],
            description='Enable DEM costmap layer',
        ),
        DeclareLaunchArgument(
            'use_respawn',
            default_value='false',
            description='Whether to respawn if a node crashes',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log level for nav2 nodes',
        ),
        DeclareLaunchArgument(
            'use_config',
            default_value='false',
            choices=['true', 'false'],
            description='Launch the Nav2 config GUI',
        ),
    
        robot_state_publisher,
        navigation_launch,
        config_gui,
    ])
