from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter




def generate_launch_description():
   use_sim_time = LaunchConfiguration('use_sim_time')
   autostart = LaunchConfiguration('autostart')
   params_file = LaunchConfiguration('params_file')
   use_respawn = LaunchConfiguration('use_respawn')
   log_level = LaunchConfiguration('log_level')


   default_bt_xml_path = PathJoinSubstitution([
       FindPackageShare('athena_planner'),
       'behavior_trees',
       'navigate_w_replanning_time.xml'
   ])


   lifecycle_nodes = [
       'controller_server',
       'smoother_server',
       'planner_server',
       'behavior_server',
       'bt_navigator',
       'waypoint_follower',
       'velocity_smoother',
   ]


   remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]


   declare_use_sim_time_cmd = DeclareLaunchArgument(
       'use_sim_time',
       default_value='false',
       description='Use simulation clock if true',
   )


   declare_params_file_cmd = DeclareLaunchArgument(
       'params_file',
       description='Full path to the ROS2 parameters file',
   )


   declare_autostart_cmd = DeclareLaunchArgument(
       'autostart',
       default_value='true',
       description='Automatically startup the nav2 stack',
   )


   declare_use_respawn_cmd = DeclareLaunchArgument(
       'use_respawn',
       default_value='False',
       description='Whether to respawn if a node crashes',
   )


   declare_log_level_cmd = DeclareLaunchArgument(
       'log_level', default_value='info', description='log level'
   )


   ld = LaunchDescription()


   ld.add_action(declare_use_sim_time_cmd)
   ld.add_action(declare_params_file_cmd)
   ld.add_action(declare_autostart_cmd)
   ld.add_action(declare_use_respawn_cmd)
   ld.add_action(declare_log_level_cmd)


   ld.add_action(SetParameter('use_sim_time', use_sim_time))


   ld.add_action(
       Node(
               package='nav2_controller',
               executable='controller_server',
               output='screen',
               respawn=use_respawn,
               respawn_delay=2.0,
               parameters=[params_file],
               arguments=['--ros-args', '--log-level', log_level],
               remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
       )
   )


   ld.add_action(
       Node(
               package='nav2_smoother',
               executable='smoother_server',
               name='smoother_server',
               output='screen',
               respawn=use_respawn,
               respawn_delay=2.0,
               parameters=[params_file],
               arguments=['--ros-args', '--log-level', log_level],
               remappings=remappings,
       )
   )


   ld.add_action(
       Node(
               package='nav2_planner',
               executable='planner_server',
               name='planner_server',
               output='screen',
               respawn=use_respawn,
               respawn_delay=2.0,
               parameters=[params_file],
               arguments=['--ros-args', '--log-level', log_level],
               remappings=remappings,
       )
   )


   ld.add_action(
       Node(
               package='nav2_behaviors',
               executable='behavior_server',
               name='behavior_server',
               output='screen',
               respawn=use_respawn,
               respawn_delay=2.0,
               parameters=[params_file],
               arguments=['--ros-args', '--log-level', log_level],
               remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
       )
   )


   ld.add_action(
       Node(
               package='nav2_bt_navigator',
               executable='bt_navigator',
               name='bt_navigator',
               output='screen',
               respawn=use_respawn,
               respawn_delay=2.0,
               parameters=[
                   params_file,
                   {'default_nav_to_pose_bt_xml': default_bt_xml_path}
               ],
               arguments=['--ros-args', '--log-level', log_level],
               remappings=remappings,
       )
   )


   ld.add_action(
       Node(
               package='nav2_waypoint_follower',
               executable='waypoint_follower',
               name='waypoint_follower',
               output='screen',
               respawn=use_respawn,
               respawn_delay=2.0,
               parameters=[params_file],
               arguments=['--ros-args', '--log-level', log_level],
               remappings=remappings,
       )
   )


   ld.add_action(
       Node(
               package='nav2_velocity_smoother',
               executable='velocity_smoother',
               name='velocity_smoother',
               output='screen',
               respawn=use_respawn,
               respawn_delay=2.0,
               parameters=[params_file],
               arguments=['--ros-args', '--log-level', log_level],
               remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
       )
   )


   ld.add_action(
       Node(
               package='nav2_lifecycle_manager',
               executable='lifecycle_manager',
               name='lifecycle_manager_navigation',
               output='screen',
               arguments=['--ros-args', '--log-level', log_level],
               parameters=[{'autostart': autostart}, {'node_names': lifecycle_nodes}],
       )
   )


   return ld




