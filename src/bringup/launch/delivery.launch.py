import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


# Drive owns the default /controller_manager. Arm runs in /arm to avoid the collision.
_ARM_NS = "arm"
_ARM_CM = f"/{_ARM_NS}/controller_manager"


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='standalone',
            choices=['standalone', 'jetson', 'base_station'],
            description=(
                "'jetson': hardware nodes on the rover. "
                "'base_station': operator teleop nodes. "
                "'standalone': all nodes on one machine."
            ),
        ),
        DeclareLaunchArgument('use_sim', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('use_mock_hardware', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('mock_sensor_commands', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument(
            'robot_controller',
            default_value='rear_ackermann_controller',
            choices=['front_ackermann_controller', 'ackermann_steering_controller', 'rear_ackermann_controller'],
        ),
        DeclareLaunchArgument('use_3dof', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('deactivate_talon', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('can_interface', default_value='can1'),
        OpaqueFunction(function=launch_setup),
    ])


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context)
    use_sim = LaunchConfiguration('use_sim').perform(context)
    use_mock_hardware = LaunchConfiguration('use_mock_hardware').perform(context)
    mock_sensor_commands = LaunchConfiguration('mock_sensor_commands').perform(context)
    robot_controller = LaunchConfiguration('robot_controller').perform(context)
    use_3dof = LaunchConfiguration('use_3dof').perform(context)
    deactivate_talon = LaunchConfiguration('deactivate_talon').perform(context)
    can_interface = LaunchConfiguration('can_interface').perform(context)

    drive_share = get_package_share_directory('drive_bringup')
    arm_bringup_share = get_package_share_directory('arm_bringup')
    description_share = get_package_share_directory('description')
    athena_gps_share = get_package_share_directory('athena_gps')
    mag_heading_share = get_package_share_directory('mag_heading')

    # ------------------------------------------------------------------
    # Drive: unmodified subsystem launch, root namespace.
    # ------------------------------------------------------------------
    drive = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(drive_share, 'launch', 'athena_drive.launch.py')),
        launch_arguments={
            'mode': mode,
            'use_sim': use_sim,
            'use_mock_hardware': use_mock_hardware,
            'mock_sensor_commands': mock_sensor_commands,
            'robot_controller': robot_controller,
            'can_interface': can_interface,
        }.items(),
    )

    # ------------------------------------------------------------------
    # GPS + heading: unmodified, root namespace.
    # ------------------------------------------------------------------
    gps = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(athena_gps_share, 'launch', 'gps_launch.py')),
        launch_arguments={'sim': use_sim}.items(),
    )
    heading = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(mag_heading_share, 'launch', 'mag_heading.launch.py')),
    )

    # ------------------------------------------------------------------
    # Arm: inlined, namespaced under /arm so its CM/topics don't clash with drive.
    # The arm subsystem files (URDF, controllers YAML, joystick YAML) are reused as-is.
    # ------------------------------------------------------------------
    arm_urdf_xacro = os.path.join(description_share, 'urdf', 'athena_arm.urdf.xacro')
    arm_controllers_yaml = os.path.join(arm_bringup_share, 'config', 'athena_arm_controllers.yaml')
    arm_joystick_yaml = os.path.join(arm_bringup_share, 'config', 'joystick.yaml')

    arm_urdf = Command([
        FindExecutable(name='xacro'), ' ',
        arm_urdf_xacro,
        ' prefix:=""',
        ' use_mock_hardware:=', use_mock_hardware,
        ' mock_sensor_commands:=', mock_sensor_commands,
        ' use_3dof:=', use_3dof,
        ' deactivate_talon:=', deactivate_talon, ' ',
        ' can_interface:=', can_interface, ' ',
    ])

    use_3dof_bool = use_3dof.lower() == 'true'
    wrist_active = 'manual_3dof_wrist_joint_by_joint_controller' if use_3dof_bool else 'manual_2dof_wrist_joint_by_joint_controller'
    wrist_inactive = 'manual_2dof_wrist_joint_by_joint_controller' if use_3dof_bool else 'manual_3dof_wrist_joint_by_joint_controller'
    jt_controller = 'threedof_joint_trajectory_controller' if use_3dof_bool else 'twodof_joint_trajectory_controller'

    active_controllers = [
        'manual_arm_joint_by_joint_controller',
        wrist_active,
        'manual_end_effector_gripper_claw_controller',
    ]
    inactive_controllers = [
        wrist_inactive,
        'manual_arm_cylindrical_controller',
        jt_controller,
        'arm_velocity_controller',
    ]

    arm_cm = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace=_ARM_NS,
        output='both',
        parameters=[arm_controllers_yaml],
        remappings=[('~/robot_description', 'robot_description')],
    )

    arm_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=_ARM_NS,
        output='both',
        parameters=[{'robot_description': arm_urdf}],
    )

    def spawner(name, inactive=False):
        args = [name, '-c', _ARM_CM, '--param-file', arm_controllers_yaml]
        if inactive:
            args.append('--inactive')
        return Node(
            package='controller_manager',
            executable='spawner',
            namespace=_ARM_NS,
            arguments=args,
        )

    jsb = spawner('joint_state_broadcaster')
    active_spawners = [spawner(c) for c in active_controllers]
    inactive_spawners = [spawner(c, inactive=True) for c in inactive_controllers]

    # Sequence: CM up → JSB → active spawners (chained) → inactive spawners (chained).
    delay_jsb = RegisterEventHandler(OnProcessStart(
        target_action=arm_cm,
        on_start=[TimerAction(period=3.0, actions=[jsb])],
    ))

    chained = []
    prev = jsb
    for sp in active_spawners + inactive_spawners:
        chained.append(RegisterEventHandler(OnProcessExit(target_action=prev, on_exit=[sp])))
        prev = sp

    arm_hardware = GroupAction([arm_cm, arm_rsp, delay_jsb, *chained])

    arm_joystick = Node(
        package='teleop',
        executable='joystick',
        namespace=_ARM_NS,
        name='joystick',
        output='screen',
        parameters=[arm_joystick_yaml],
    )

    if mode == 'jetson':
        return [drive, arm_hardware, gps, heading]
    elif mode == 'base_station':
        return [drive, arm_joystick]
    else:  # standalone
        return [drive, arm_hardware, arm_joystick, gps, heading]
