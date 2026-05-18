from launch import LaunchDescription, LaunchContext
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="arm_bringup",
            description='Package with the controller\'s configuration in "config" folder.',
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
            description="Description package with robot URDF/xacro files.",
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
            "prefix",
            default_value='""',
            description="Prefix of the joint names.",
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
            description="Enable mock command interfaces for sensors.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_3dof",
            default_value="false",
            description="Enable the joints required for the 3 Degree of Freedom Wrist",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "deactivate_talon",
            default_value="false",
            description="Deactivate the talon joints in the URDF when using mock hardware to prevent excessive CAN flow.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "can_interface",
            default_value="can1",
            description="CAN interface to use for hardware interfaces.",
        )
    )
    # -- Initialize Arguments --
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    prefix = LaunchConfiguration("prefix")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    use_3dof = LaunchConfiguration("use_3dof")
    deactivate_talon = LaunchConfiguration("deactivate_talon")
    can_interface = LaunchConfiguration("can_interface")
    
    # -- Building Path Files --
    robot_description_path = PathJoinSubstitution(
        [FindPackageShare("description"), "urdf", "athena_arm.urdf.xacro"]
    )
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )

    controller_switcher_config = PathJoinSubstitution(
        [FindPackageShare("bringup"), "config", "controller_switcher.yaml"]
    )

    # -- Additional Configuration Setup --
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
            "use_3dof:=",
            use_3dof,
            " ",
            "deactivate_talon:=",
            deactivate_talon,
            " ",
            "can_interface:=",
            can_interface,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # -- Node Definitions -- 
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_controllers],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    wrist_controller = PythonExpression([
        '"manual_3dof_wrist_joint_by_joint_controller" if "',
        use_3dof,
        '" == "true" else "manual_2dof_wrist_joint_by_joint_controller"'
    ])

    robot_controller_names = [
        "manual_arm_joint_by_joint_controller",
        wrist_controller,
        "manual_end_effector_gripper_claw_controller",
    ]
    robot_controller_spawners = []
    for controller in robot_controller_names:
        robot_controller_spawners += [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "-c", "/controller_manager"],
            )
        ]

    joint_trajectory_controller = PythonExpression([
        '"threedof_joint_trajectory_controller" if "',
        use_3dof,
        '" == "true" else "twodof_joint_trajectory_controller"'
    ])

    inactive_controller = PythonExpression([
        '"manual_2dof_wrist_joint_by_joint_controller" if "',
        use_3dof,
        '" == "true" else "manual_3dof_wrist_joint_by_joint_controller"',
    ])
    inactive_robot_controller_names = [
        inactive_controller,
        "manual_arm_cylindrical_controller",
        joint_trajectory_controller,
        "arm_velocity_controller",
    ]
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
                    package="bringup",
                    executable="controller_switcher.py",
                    name="controller_switcher",
                    output="screen",
                    parameters=[controller_switcher_config, {"subsystem": "arm"}]
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

    return LaunchDescription(
        declared_arguments +
        [
            control_node,
            robot_state_pub_node,
            delay_joint_state_broadcaster_spawner_after_ros2_control_node,
            controller_switcher_node,
        ]
        + delay_robot_controller_spawners_after_joint_state_broadcaster_spawner
        + delay_inactive_robot_controller_spawners_after_joint_state_broadcaster_spawner
    )
