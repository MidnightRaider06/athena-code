from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='zed_point_cloud_bridge',
            output='screen',
            arguments=[
                '/zed/zed_node/point_cloud/cloud_registered/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            ],
            remappings=[
                ('/zed/zed_node/point_cloud/cloud_registered/points',
                 '/zed/zed_node/point_cloud/cloud_registered'),
            ],
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='zed_rgb_bridge',
            output='screen',
            arguments=[
                '/zed/zed_node/left/image_rect_color@sensor_msgs/msg/Image@gz.msgs.Image',
                '/zed/zed_node/left/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
            ],
        ),
        
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            output='screen',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ]
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ground_truth_odom_bridge',
            output='screen',
            arguments=[
                '/odom/ground_truth@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            ]
        ),
    ])
