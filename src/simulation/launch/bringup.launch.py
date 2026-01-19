from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

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
    DeclareLaunchArgument(
        'publish_ground_truth_tf',
        default_value='false',
        choices=['true', 'false'],
        description='Publish ground truth odom -> base_link transform'
    ),
    DeclareLaunchArgument(
    	'rqt', 
        default_value='false',
        choices=['true', 'false'],
        description='Open RQt.'
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
    ground_truth_tf_launch = PathJoinSubstitution(
        [pkg_sim, 'launch', 'ground_truth_tf.launch.py'])

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

    ground_truth_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ground_truth_tf_launch]),
        condition=IfCondition(LaunchConfiguration('publish_ground_truth_tf'))
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gazebo)
    ld.add_action(robot_spawn)
    ld.add_action(bridge)
    ld.add_action(control)
    ld.add_action(ground_truth_tf)
    return ld
