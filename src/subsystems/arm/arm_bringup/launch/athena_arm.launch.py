# Copyright 2023 ros2_control Development Team
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

from launch import LaunchDescription, LaunchContext
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # -- Declare arguments --
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="arm_bringup",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="athena_arm_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="description",
            description="Description package with robot URDF/xacro files. Usually the argument \
            is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="athena_arm.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_file",
            default_value="rviz_config.rviz",
            description="Rviz config file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_package",
            default_value="arm_moveit",
            description="MoveIt package containing all the configurations and necessary files for \
            integrating MoveIt.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
            multi-robot setup. If changed than also joint names in the controllers' configuration \
            have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mock_sensor_commands",
            default_value="false",
            description="Enable mock command interfaces for sensors used for simple simulations. \
            Used only if 'use_mock_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="manual_arm_joint_by_joint_controller",
            choices=["manual_arm_joint_by_joint_controller"],
            description="Robot controller to start.",
        )
    )

    # -- Initialize Arguments --
    use_sim = LaunchConfiguration("use_sim")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    # joystick_config = LaunchConfiguration("joystick_config")
    # teleop_twist_config = LaunchConfiguration("teleop_twist_config")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    rviz_file = LaunchConfiguration("rviz_file")
    moveit_package = LaunchConfiguration("moveit_package")
    prefix = LaunchConfiguration("prefix")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    robot_controller = LaunchConfiguration("robot_controller")
    
    # -- Building Path Files --
    # Get URDF via xacro.
    # This is creating a terminal command that essentially expands all macros in this file
    # and creates the FULL URDF
    robot_description_path = PathJoinSubstitution(
        [FindPackageShare("description"), "urdf", "athena_arm.urdf.xacro"]
    )
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )
    joint_state_yaml = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", "initial_joint_states.yaml"]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", rviz_file]
    )
    joystick_config_file = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), 'config', 'joystick.yaml']
    )

    # MoveIt Config Setup (TODO: Currently not using Launch Configuration for description and these configs because moveit
    # config builder happens at launch time. Is there a way to keep these Launch configs when building file path?)
    robot_semantic_path = PathJoinSubstitution(
        [FindPackageShare("arm_moveit"), "srdf", "athena_arm.srdf"]
    )
    robot_kinematics_path = PathJoinSubstitution(
        [FindPackageShare("arm_moveit"), "config", "kinematics.yaml"]
    )
    moveit_controllers_config_path = PathJoinSubstitution(
        [FindPackageShare("arm_moveit"), "config", "moveit_controllers.yaml"]
    )

    # -- Additional Configuration Setup --
    # Robot Description Setup
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            robot_description_path,
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
            "mock_sensor_commands:=",
            mock_sensor_commands,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Load the robot configuration
    moveit_config = (
        MoveItConfigsBuilder("athena_arm", package_name="arm_moveit")
        .robot_description(file_path=robot_description_path.perform(LaunchContext()))
        .robot_description_semantic(file_path=robot_semantic_path.perform(LaunchContext()))
        .robot_description_kinematics(file_path=robot_kinematics_path.perform(LaunchContext()))
        .trajectory_execution(file_path=moveit_controllers_config_path.perform(LaunchContext()))
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"],
            default_planning_pipeline="ompl",
        )
        .to_moveit_configs()
    )

    # -- Node Definitions -- 
    joystick_publisher = Node(
        package='teleop',
        executable='joystick',
        name='joystick',
        output='screen',
        parameters = [joystick_config_file],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both", # screen and log
        parameters=[robot_controllers],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        # prefix=['xterm -e gdb -ex run --args']
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(use_sim),
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[joint_state_yaml],
        output='screen'
    )

    joint_state_publisher_gui_node = Node( 
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    ) # This is doing my transformations ONLY when there are no previous joint states.

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    motor_status_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["motor_status_broadcaster", "-c", "/controller_manager"],
    )

    robot_controller_names = [robot_controller]
    robot_controller_spawners = []
    for controller in robot_controller_names:
        robot_controller_spawners += [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "-c", "/controller_manager"],
            )
        ]

    inactive_robot_controller_names = ["manual_arm_cylindrical_controller", "joint_trajectory_controller", "arm_velocity_controller"]
    inactive_robot_controller_spawners = []
    for controller in inactive_robot_controller_names:
        inactive_robot_controller_spawners += [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "-c", "/controller_manager", "--inactive"],
            )
        ]

    # Handle switching between controllers
    controller_switcher_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=inactive_robot_controller_spawners[-1],
            on_exit=[TimerAction(
                period=3.0,
                actions=[Node(
                    package="arm_bringup",
                    executable="controller_switcher.py",
                    name="controller_switcher",
                    output="screen"
                )]
            )],
        )
    )

    # Delay loading and activation of `joint_state_broadcaster` after start of ros2_control_node
    delay_joint_state_broadcaster_spawner_after_ros2_control_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=3.0,
                    actions=[joint_state_broadcaster_spawner],
                ),
            ],
        )
    )

    # Delay motor_status_broadcaster after joint_state_broadcaster
    delay_motor_status_broadcaster_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[motor_status_broadcaster_spawner],
        )
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay loading and activation of robot_controller_names after `joint_state_broadcaster`
    delay_robot_controller_spawners_after_joint_state_broadcaster_spawner = []
    for i, controller in enumerate(robot_controller_spawners):
        delay_robot_controller_spawners_after_joint_state_broadcaster_spawner += [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=(
                        robot_controller_spawners[i - 1]
                        if i > 0
                        else joint_state_broadcaster_spawner
                    ),
                    on_exit=[controller],
                )
            )
        ]

    # Delay start of inactive_robot_controller_names after other controllers
    delay_inactive_robot_controller_spawners_after_joint_state_broadcaster_spawner = []
    for i, controller in enumerate(inactive_robot_controller_spawners):
        delay_inactive_robot_controller_spawners_after_joint_state_broadcaster_spawner += [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=(
                        inactive_robot_controller_spawners[i - 1]
                        if i > 0
                        else robot_controller_spawners[-1]
                    ),
                    on_exit=[controller],
                )
            )
        ]

    umdloop_can_node = Node(
            package='umdloop_can',
            executable='can_node',
            name='can_node',
            output='log',
            arguments=['--ros-args', '--log-level', 'fatal'] # prevents can node from outputting to terminal

    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()]
    )

    hello_moveit_node = Node(
        package="arm_moveit",
        executable="hello_moveit",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription(
        declared_arguments +
        [
            # umdloop_can_node,
            control_node,
            joystick_publisher,
            # joint_state_publisher, # sends 0s to /joint_states
            # joint_state_publisher_gui_node, # sends gui values to /joint_states
            robot_state_pub_node, # handles tf transforms, uses urdf on startup, then subscribers to /joint_states to update
            # move_group_node,
            # hello_moveit_node,
            delay_joint_state_broadcaster_spawner_after_ros2_control_node, # reads from hardware and sends values to /joint_states
            delay_motor_status_broadcaster_after_joint_state_broadcaster,
            delay_rviz_after_joint_state_broadcaster_spawner,
            controller_switcher_node,
        ]
        + delay_robot_controller_spawners_after_joint_state_broadcaster_spawner
        + delay_inactive_robot_controller_spawners_after_joint_state_broadcaster_spawner
    )