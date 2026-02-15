from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument(
        'sim',
        default_value='true',
        choices=['true', 'false'],
        description='Use simulation topics'
    ),
    DeclareLaunchArgument(
        'marker_size',
        default_value='0.2',
        description='Size of ArUco marker in meters'
    ),
]

def generate_launch_description():
    aruco_node = Node(
        package='aruco_bt',
        executable='aruco_node',
        name='aruco_node',
        parameters=[
            {'sim': LaunchConfiguration('sim')},
            {'marker_size': LaunchConfiguration('marker_size')}
        ],
        output='screen'
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(aruco_node)
    
    return ld