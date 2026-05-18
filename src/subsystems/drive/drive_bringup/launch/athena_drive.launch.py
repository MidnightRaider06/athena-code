from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "mode",
            default_value="standalone",
            choices=["standalone", "jetson", "base_station"],
            description=(
                "Deployment mode. "
                "'standalone' (default) starts all nodes on a single machine. "
                "'jetson' starts only the control/hardware nodes (run on the rover). "
                "'base_station' starts only the teleop nodes (joystick + teleop_twist_joy)."
            ),
        )
    )
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
            default_value="drive_bringup",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
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
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="athena_drive_controllers.yaml",
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
            default_value="athena_drive.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_file",
            default_value="athena_drive.rviz",
            description="Rviz config file.",
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
            default_value="rear_ackermann_controller",
            choices=["front_ackermann_controller", "ackermann_steering_controller", "rear_ackermann_controller"],
            description="Robot controller to start.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "deactivate_odrive",
            default_value="false",
            description="Deactivate the ODrive joints in the URDF when using mock hardware to prevent excessive CAN flow.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "can_interface",
            default_value="can0",
            description="CAN interface to use for hardware interfaces.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    use_sim = LaunchConfiguration("use_sim")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    joystick_config_name = LaunchConfiguration("joystick_config")
    teleop_twist_config_name = LaunchConfiguration("teleop_twist_config")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    rviz_file = LaunchConfiguration("rviz_file")
    prefix = LaunchConfiguration("prefix")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    robot_controller = LaunchConfiguration("robot_controller")
    deactivate_odrive = LaunchConfiguration("deactivate_odrive")
    can_interface = LaunchConfiguration("can_interface")

    robot_description_path = PathJoinSubstitution(
        [FindPackageShare(description_package), "urdf", description_file]
    )
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )
    joystick_config = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", joystick_config_name]
    )
    teleop_twist_config = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", teleop_twist_config_name]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", rviz_file]
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
            "deactivate_odrive:=",
            deactivate_odrive,
            " ",
            "can_interface:=",
            can_interface,
        ]
    )

    robot_description = {"robot_description": robot_description_content}

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

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_controllers],
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/front_ackermann_controller/tf_odometry", "/tf"),
            ("/ackermann_steering_controller/reference", "/cmd_vel"),
        ],
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
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    motor_status_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["motor_status_controller", "-c", "/controller_manager"],
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

    gpio_controller_names = ["drive_led_gpio_controller", "power_module_gpio_controller"]
    gpio_controller_spawners = []
    for controller in gpio_controller_names:
        gpio_controller_spawners += [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "-c", "/controller_manager"],
            )
        ]

    inactive_robot_controller_names = ["ackermann_steering_controller", "drive_velocity_controller", "drive_position_controller"]
    inactive_robot_controller_spawners = []
    for controller in inactive_robot_controller_names:
        inactive_robot_controller_spawners += [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "-c", "/controller_manager", "--inactive"],
            )
        ]

    delay_gpio_controller_spawners_after_joint_state_broadcaster_spawner = []
    for i, controller in enumerate(gpio_controller_spawners):
        delay_gpio_controller_spawners_after_joint_state_broadcaster_spawner += [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=(
                        gpio_controller_spawners[i - 1]
                        if i > 0
                        else joint_state_broadcaster_spawner
                    ),
                    on_exit=[controller],
                )
            )
        ]

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
                    parameters=[controller_switcher_config]
                )]
            )],
        )
    )

    delay_joint_state_broadcaster_spawner_after_ros2_control_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[
                TimerAction(
                    period=5.0,
                    actions=[joint_state_broadcaster_spawner],
                ),
            ],
        )
    )

    delay_motor_status_controller_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[motor_status_controller_spawner],
        )
    )

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

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

    jetson_actions = [
        control_node,
        robot_state_pub_node,
        delay_joint_state_broadcaster_spawner_after_ros2_control_node,
        delay_motor_status_controller_after_joint_state_broadcaster,
        delay_rviz_after_joint_state_broadcaster_spawner,
        controller_switcher_node,
    ] + delay_robot_controller_spawners_after_joint_state_broadcaster_spawner \
      + delay_inactive_robot_controller_spawners_after_joint_state_broadcaster_spawner \
      + delay_gpio_controller_spawners_after_joint_state_broadcaster_spawner

    base_station_actions = [
        joystick_publisher,
        teleop_twist_joy,
    ]

    if mode == "jetson":
        return jetson_actions
    elif mode == "base_station":
        return base_station_actions
    else:
        return jetson_actions + base_station_actions
