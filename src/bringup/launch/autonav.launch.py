from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    default_params = PathJoinSubstitution([
        FindPackageShare('athena_planner'), 'config', 'nav2_params.yaml'
    ])
    minimal_params = PathJoinSubstitution([
        FindPackageShare('athena_planner'), 'config', 'nav2_params_minimal.yaml'
    ])
    selected_params = PythonExpression([
        "'", minimal_params, "' if '", LaunchConfiguration('use_minimal'), "' == 'true' else '", default_params, "'"
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='standalone',
            choices=['standalone', 'jetson', 'base_station'],
            description=(
                "'jetson': hardware + nav nodes on the rover. "
                "'base_station': operator teleop nodes. "
                "'standalone': all nodes on one machine."
            ),
        ),
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            choices=['true', 'false'],
            description='Enable sim time and RViz2.',
        ),
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='false',
            choices=['true', 'false'],
            description='Start drive with mock hardware mirroring commands to states.',
        ),
        DeclareLaunchArgument(
            'mock_sensor_commands',
            default_value='false',
            choices=['true', 'false'],
            description='Enable mock command interfaces for sensors. Only used when use_mock_hardware is true.',
        ),
        DeclareLaunchArgument(
            'robot_controller',
            default_value='rear_ackermann_controller',
            choices=['front_ackermann_controller', 'ackermann_steering_controller', 'rear_ackermann_controller'],
            description='Drive controller to start.',
        ),
        DeclareLaunchArgument(
            'use_zed_localizer',
            default_value='true',
            choices=['true', 'false'],
            description='Use ZED spatial localization instead of the EKF localizer.',
        ),
        DeclareLaunchArgument(
            'enable_gnss',
            default_value='true',
            choices=['true', 'false'],
            description='Enable GNSS fusion inside the ZED camera.',
        ),
        DeclareLaunchArgument(
            'use_minimal',
            default_value='false',
            choices=['true', 'false'],
            description='Use minimal Nav2 config (empty costmaps, basic BT) for ackermann testing.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=selected_params,
            description='Full path to the Nav2 params YAML. Defaults to minimal or full config based on use_minimal.',
        ),
        DeclareLaunchArgument(
            'use_dem',
            default_value='false',
            choices=['true', 'false'],
            description='Enable DEM costmap layer.',
        ),
        DeclareLaunchArgument(
            'use_respawn',
            default_value='false',
            choices=['true', 'false'],
            description='Respawn nav2 nodes if they crash.',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log level for nav2 nodes.',
        ),
        DeclareLaunchArgument(
            'use_config',
            default_value='false',
            choices=['true', 'false'],
            description='Launch the Nav2 config GUI.',
        ),
        DeclareLaunchArgument(
            'can_interface',
            default_value='can1',
            description='CAN interface to use for hardware interfaces.',
        ),
        OpaqueFunction(function=launch_setup),
    ])


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context)
    use_sim = LaunchConfiguration('use_sim').perform(context)
    use_mock_hardware = LaunchConfiguration('use_mock_hardware').perform(context)
    mock_sensor_commands = LaunchConfiguration('mock_sensor_commands').perform(context)
    robot_controller = LaunchConfiguration('robot_controller').perform(context)
    use_zed_localizer = LaunchConfiguration('use_zed_localizer').perform(context)
    enable_gnss = LaunchConfiguration('enable_gnss').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    use_dem = LaunchConfiguration('use_dem').perform(context)
    use_respawn = LaunchConfiguration('use_respawn').perform(context)
    log_level = LaunchConfiguration('log_level').perform(context)
    use_config = LaunchConfiguration('use_config').perform(context)
    can_interface = LaunchConfiguration('can_interface').perform(context)

    drive_bringup = get_package_share_directory('drive_bringup')
    nav_bringup = get_package_share_directory('nav_bringup')
    mag_heading = get_package_share_directory('mag_heading')

    drive = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(drive_bringup, 'launch', 'athena_drive.launch.py')
        ),
        launch_arguments={
            'mode': mode,
            'use_sim': use_sim,
            'use_mock_hardware': use_mock_hardware,
            'mock_sensor_commands': mock_sensor_commands,
            'robot_controller': robot_controller,
            'can_interface': can_interface,
        }.items(),
    )

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_bringup, 'launch', 'nav_bringup.launch.py')
        ),
        launch_arguments={
            'sim': use_sim,
            'use_zed_localizer': use_zed_localizer,
            'enable_gnss': enable_gnss,
            'params_file': params_file,
            'use_dem': use_dem,
            'use_respawn': use_respawn,
            'log_level': log_level,
            'use_config': use_config,
            'can_interface': can_interface,
        }.items(),
    )

    heading = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mag_heading, 'launch', 'mag_heading.launch.py')
        ),
    )

    if mode == 'base_station':
        return [drive]
    else:
        return [drive, nav, heading]
