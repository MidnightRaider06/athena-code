from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yolo_ros_bt',
            executable='yolo_ros_node',
            name='yolo_node',
            output='screen',
            parameters=[{
                'image_topic': '/zed/zed_node/rgb/color/rect/image',
                'target_classes': ['Bottle', 'Mallet', 'Rock-Pick-Hammer'],
                'conf_thres': 0.5,
            }]
        )
    ])