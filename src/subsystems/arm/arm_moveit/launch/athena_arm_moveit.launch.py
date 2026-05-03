from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder



def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_3dof",
            default_value="false",
            description="Enable the joints required for the 3 Degree of Freedom Wrist",
        )
    )

    use_3dof = LaunchConfiguration("use_3dof")

    joint_state_yaml = PathJoinSubstitution(
        [
            FindPackageShare("arm_bringup"),
            "config",
            "initial_joint_states.yaml",
        ]
    )
        
    robot_description_path = PathJoinSubstitution([FindPackageShare("description"),
                                                   "urdf",
                                                   "athena_arm.urdf.xacro"])
    robot_semantic_path = PathJoinSubstitution([FindPackageShare("arm_moveit"),
                                                "srdf",
                                                "athena_arm.srdf"])
    robot_kinematics_path = PathJoinSubstitution([FindPackageShare("arm_moveit"),
                                                  "config",
                                                  "kinematics.yaml"])
    moveit_controllers_config_file = PythonExpression([
        '"moveit_controllers_3dof.yaml" if "',
        use_3dof,
        '" == "true" else "moveit_controllers_2dof.yaml"'
    ])
    moveit_controllers_config_path = PathJoinSubstitution([FindPackageShare("arm_moveit"),
                                                           "config",
                                                           moveit_controllers_config_file])
    
    # Eventually want to use this, for now the package will be independent
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("description"), 
            "rviz", 
            "rviz_config.rviz"
        ]
    )

    # rviz_config_file = PathJoinSubstitution(
    #     [
    #         FindPackageShare("arm_moveit"), 
    #         "rviz", 
    #         "moveit.rviz"
    #     ]
    # )
    

    # Load the robot configuration
    moveit_config = (
        MoveItConfigsBuilder(
            "athena_arm", package_name="arm_moveit"
        )
        .robot_description(robot_description_path.perform(LaunchContext()))
        .robot_description_semantic("srdf/athena_arm.srdf")
        .trajectory_execution(file_path=moveit_controllers_config_path.perform(LaunchContext()))
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
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

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    hello_moveit_node = Node(
        package="arm_moveit",
        executable="hello_moveit",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    nodes = [
                # joint_state_publisher,
                joint_state_publisher_gui_node,
                robot_state_publisher,
                rviz_node,
                run_move_group_node,
                hello_moveit_node
            ]
    return LaunchDescription(declared_arguments + nodes)