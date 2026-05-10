from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="led_indicator",
            executable="led_indicator_node",
            name="led_indicator",
            output="screen",
            parameters=[{
                "serial_port": "auto",
                "baud_rate": 115200,
            }],
        )
    ])
