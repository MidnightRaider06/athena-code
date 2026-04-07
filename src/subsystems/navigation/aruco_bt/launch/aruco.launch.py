from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument(
        'marker_size',
        default_value='0.2',
        description='Size of ArUco marker in meters'
    ),
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    ),
    DeclareLaunchArgument(
        'image_topic',
        default_value='/zed/zed_node/left/image_rect_color',
        description='Image topic to subscribe to for ArUco detection'
    ),
    DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/zed/zed_node/left/camera_info',
        description='Camera info topic corresponding to the image topic'
    ),
]

def generate_launch_description():
    aruco_node = Node(
        package='aruco_bt',
        executable='aruco_node',
        name='aruco_node',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'marker_size': LaunchConfiguration('marker_size')},
            {'image_topic': LaunchConfiguration('image_topic')},
            {'camera_info_topic': LaunchConfiguration('camera_info_topic')},
        ],
        output='screen'
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(aruco_node)
    
    return ld