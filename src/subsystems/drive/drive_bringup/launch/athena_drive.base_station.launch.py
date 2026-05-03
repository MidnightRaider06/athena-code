# Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # -- Declare arguments --
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="drive_bringup",
            description='Package with the controller\'s configuration in "config" folder.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "joystick_config",
            default_value="joystick.yaml",
            description="YAML file with the joystick configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "teleop_twist_config",
            default_value="teleop_twist.yaml",
            description="YAML file with the teleop_twist_node configuration.",
        )
    )

    # -- Initialize Arguments --
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    joystick_config = LaunchConfiguration("joystick_config")
    teleop_twist_config = LaunchConfiguration("teleop_twist_config")

    # -- Building Path Files --
    joystick_config = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", joystick_config]
    )
    teleop_twist_config = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", teleop_twist_config]
    )

    # -- Node Definitions -- 
    joystick_publisher = Node(
        package='teleop',
        executable='joystick',
        name='joystick',
        output='screen',
        parameters=[joystick_config],
        remappings=[
            ('controller_input', 'joy'),
            ('/controller_input', '/joy'),
        ],
    )

    teleop_twist_joy = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        output='screen',
        parameters=[teleop_twist_config],
        remappings=[
            ('/cmd_vel', '/rear_ackermann_controller/reference'),
        ],
    )

    return LaunchDescription(
        declared_arguments + 
        [
            joystick_publisher,
            teleop_twist_joy,
        ]
    )