import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import conditions
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue

ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='false',
                          choices=['true', 'false'],
                          description='Start rviz.'),
    DeclareLaunchArgument('rqt', default_value='false',
                          description='Open RQt.'),
    DeclareLaunchArgument('image_topic', default_value='/zed/zed_node/left/image_rect_color',
                          description='Topic to start viewing in RQt.'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
    DeclareLaunchArgument('world_name', default_value='',
                          description='World name'),
    
]

def generate_launch_description():
    pkg_description = get_package_share_directory('description')
    pkg_sim = get_package_share_directory('simulation')

    urdf_file = os.path.join(pkg_description, 'urdf', 'athena_drive.urdf.xacro')
    controllers_file = os.path.join(pkg_description, 'config', 'athena_drive_sim_controllers.yaml')
    rviz_config_file = os.path.join(pkg_sim, 'rviz', 'sim.rviz')
    
    namespace = LaunchConfiguration('namespace')
    robot_name = 'athena'

    robot_description_content = Command([
        'xacro ', urdf_file,
        ' use_mock_hardware:=true',
        ' sim_gazebo:=true',
        f' simulation_controllers:={controllers_file}'
    ])

    spawn_robot_group_action = GroupAction([
        PushRosNamespace(namespace),
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    robot_description_content,
                    value_type=str
                ),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', robot_name,
                       '-world', LaunchConfiguration('world_name'),
                       '-x', '0.0',
                       '-y', '0.0',
                       '-z', '3.0',
                       '-Y', '0.0',
                       '-topic', 'robot_description'],
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            condition=conditions.IfCondition(LaunchConfiguration('rviz'))
        ),
        
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='rqt',
            arguments=[LaunchConfiguration('image_topic')],
            condition=conditions.IfCondition(LaunchConfiguration('rqt'))
        ),
    ])

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(spawn_robot_group_action)
    return ld
