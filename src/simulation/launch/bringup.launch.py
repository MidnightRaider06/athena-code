from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

ARGUMENTS = [
    DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace'
    ),
    DeclareLaunchArgument(
        'rviz',
        default_value='false',
        choices=['true', 'false'],
        description='Start RViz for visualization'
    ),
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        choices=['true', 'false'],
        description='Use simulation time from Gazebo'
    ),
    DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='Gazebo world file to load'
    ),
    DeclareLaunchArgument(
        'world_name',
        default_value='default',  
        description='Name of the world inside Gazebo'
    ),
]

def generate_launch_description():
    pkg_sim = get_package_share_directory('simulation')

    gazebo_launch = PathJoinSubstitution(
        [pkg_sim, 'launch', 'gz_sim.launch.py'])
    robot_spawn_launch = PathJoinSubstitution(
        [pkg_sim, 'launch', 'spawn.launch.py'])
    bridge_launch = PathJoinSubstitution(
        [pkg_sim, 'launch', 'bridge.launch.py'])
    control_launch = PathJoinSubstitution(
        [pkg_sim, 'launch', 'control.launch.py'])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch]),
        launch_arguments=[
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('world', LaunchConfiguration('world')),
        ]
    )

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_spawn_launch]),
        launch_arguments=[
            ('namespace', LaunchConfiguration('namespace')),
            ('rviz', LaunchConfiguration('rviz')),
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('world_name', LaunchConfiguration('world_name'))
        ]
    )

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bridge_launch])
    )

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([control_launch])
    )

    model_name = "rover"   # <-- change to your actual model name

    follow_cmd = ExecuteProcess(
        cmd=[
            'bash', '-c',
            f'ign service -s /gui/follow '
            f'--reqtype ignition.msgs.StringMsg '
            f'--reptype ignition.msgs.Boolean '
            f'--timeout 2000 '
            f'--req \'data: "{model_name}"\''
        ],
        output='screen'
    )

    offset_cmd = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'ign service -s /gui/follow/offset '
            '--reqtype ignition.msgs.Vector3d '
            '--reptype ignition.msgs.Boolean '
            '--timeout 2000 '
            '--req \'x: 2.0 y: 0.0 z: 0.5\''
        ],
        output='screen'
    )

    # Run the follow+offset commands a few seconds after Gazebo starts
    # (so the GUI + world + model are definitely available)
    follow_after_delay = TimerAction(
        period=5.0,          
        actions=[follow_cmd]
    )
    
    offset_after_delay = TimerAction(
        period=10.0,           
        actions=[offset_cmd]
    )


    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gazebo)
    ld.add_action(robot_spawn)
    ld.add_action(bridge)
    ld.add_action(control)
    ld.add_action(follow_after_delay)
    ld.add_action(offset_after_delay)
    return ld