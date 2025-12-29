from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    
    ackermann_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_steering_controller'],
        output='screen',
        remappings=[
        ("/ackermann_steering_controller/tf_odometry", "/tf"),
        ]
    )
    
    delayed_ackermann_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[ackermann_controller_spawner],
        )
    )

    ld = LaunchDescription()
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(delayed_ackermann_controller_spawner)
    return ld