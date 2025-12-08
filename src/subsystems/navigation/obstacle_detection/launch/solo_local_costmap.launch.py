from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ns = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav_params = LaunchConfiguration('nav_params')
    filter_params = LaunchConfiguration('filter_params')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='', description='Robot namespace'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'nav_params',
            default_value=os.path.join(
                get_package_share_directory('obstacle_detection'), 'config', 'solo_local_costmap.yaml')),
        DeclareLaunchArgument(
            'filter_params',
            default_value=os.path.join(
                get_package_share_directory('obstacle_detection'), 'cfg', 'obstacle_filter.yaml')),

        # Obstacle filter node (subscribes ZED cloud, publishes /obstacles/points)
        Node(
            package='obstacle_detection',
            executable='obstacle_filter_node',
            namespace=ns,
            name='obstacle_filter',
            parameters=[filter_params, {'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # Standalone Nav2 costmap (no planners/controllers)
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            namespace='local_costmap',          # keep topics under /local_costmap
            name='local_costmap',
            parameters=[nav_params, {'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # Lifecycle manager for just this one node
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            namespace='local_costmap',
            name='lifecycle_manager_local_costmap',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['local_costmap']
            }],
            output='screen'
        ),
    ])
