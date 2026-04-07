import numpy as np
import cv2
import cv2.aruco

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs  # noqa: F401 — registers PoseStamped transform support


class ArUcoNode(Node):
    def __init__(self):
        super().__init__('aruco_node')

        self.declare_parameter('marker_size', 0.2)
        self.declare_parameter('image_topic', '/zed/zed_node/left/image_rect_color')
        self.declare_parameter('camera_info_topic', '/zed/zed_node/left/camera_info')

        sim = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        self.marker_size_ = self.get_parameter('marker_size').get_parameter_value().double_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value

        if sim:
            self.marker_size_ = 0.50

        self.get_logger().info(f'sim={"true" if sim else "false"}')
        self.get_logger().info(f'marker_size={self.marker_size_:.3f} meters')
        self.get_logger().info(f'Subscribing to image feed: {image_topic}')
        self.get_logger().info(f'Subscribing to camera info: {camera_info_topic}')

        self.aruco_dict_ = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        try:
            self.aruco_params_ = cv2.aruco.DetectorParameters_create() # OpenCV 4.5.x
        except AttributeError:
            self.aruco_params_ = cv2.aruco.DetectorParameters() # OpenCV 4.7.x and later

        self.tf_buffer_ = tf2_ros.Buffer()
        self.tf_listener_ = tf2_ros.TransformListener(self.tf_buffer_, self)

        self.bridge_ = CvBridge()

        self.image_sub_ = self.create_subscription(
            Image, image_topic, self.image_callback, 1)

        self.camera_info_sub_ = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, 10)

        self.image_pub_ = self.create_publisher(Image, '/aruco_annotated_img', 10)
        self.pose_pub_ = self.create_publisher(PoseStamped, '/aruco_pose', 10)

        self.marker_id_ = -1
        self.corners_ = []
        self.all_corners_ = []
        self.camera_info_received_ = False
        self.camera_matrix_ = None
        self.dist_coeffs_ = None
        self.rvec_ = None
        self.tvec_ = None

    def camera_info_callback(self, msg):
        if not self.camera_info_received_:
            self.camera_matrix_ = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs_ = np.array(msg.d, dtype=np.float64).reshape(-1, 1)
            self.camera_info_received_ = True
            self.get_logger().info('Camera info received')
            self.destroy_subscription(self.camera_info_sub_)
            self.camera_info_sub_ = None

    def image_callback(self, msg):
        try:
            cv_ptr = self.bridge_.imgmsg_to_cv2(msg, 'mono8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge exception: {e}')
            return

        frame_color = cv2.cvtColor(cv_ptr, cv2.COLOR_GRAY2BGR)

        self.detect_aruco_markers(cv_ptr)

        if self.marker_id_ >= 0 and len(self.corners_) > 0:
            for i in range(4):
                start_point = (int(self.corners_[i][0]), int(self.corners_[i][1]))
                end_point = (int(self.corners_[(i + 1) % 4][0]),
                             int(self.corners_[(i + 1) % 4][1]))
                cv2.line(frame_color, start_point, end_point, (0, 255, 0), 2)

            text_pos = (int(self.corners_[0][0]), int(self.corners_[0][1]) - 20)
            label = f'id: {self.marker_id_}'
            cv2.putText(frame_color, label, text_pos,
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 0, 255), 2, cv2.LINE_AA)

            if self.camera_info_received_:
                self.estimate_and_publish_pose(msg)

                if self.rvec_ is not None and self.tvec_ is not None:
                    try:
                        cv2.aruco.drawAxis(frame_color, self.camera_matrix_,
                                           self.dist_coeffs_, self.rvec_, self.tvec_,
                                           self.marker_size_ * 0.5)
                    except AttributeError:
                        cv2.drawFrameAxes(frame_color, self.camera_matrix_,
                                          self.dist_coeffs_, self.rvec_, self.tvec_,
                                          self.marker_size_ * 0.5)

        annotated_msg = self.bridge_.cv2_to_imgmsg(frame_color, 'bgr8')
        annotated_msg.header = msg.header
        self.image_pub_.publish(annotated_msg)

    def detect_aruco_markers(self, frame):
        corners, ids, _ = cv2.aruco.detectMarkers(
            frame, self.aruco_dict_, parameters=self.aruco_params_)

        if ids is not None and len(ids) > 0:
            self.marker_id_ = int(ids[0][0])
            self.all_corners_ = [corners[0]]
            self.corners_ = [pt for pt in corners[0][0]]
        else:
            self.marker_id_ = -1
            self.corners_ = []
            self.all_corners_ = []

    def estimate_and_publish_pose(self, msg):
        if len(self.all_corners_) == 0:
            return

        if self.marker_size_ <= 0:
            self.get_logger().warn(f'Invalid marker_size_: {self.marker_size_:.3f}')
            return

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            self.all_corners_, self.marker_size_,
            self.camera_matrix_, self.dist_coeffs_)

        self.rvec_ = rvecs[0]
        self.tvec_ = tvecs[0]

        pose_camera = PoseStamped()
        pose_camera.header.stamp = msg.header.stamp
        pose_camera.header.frame_id = msg.header.frame_id

        pose_camera.pose.position.x = float(self.tvec_[0][0])
        pose_camera.pose.position.y = float(self.tvec_[0][1])
        pose_camera.pose.position.z = float(self.tvec_[0][2])

        pose_camera.pose.orientation.x = 0.0
        pose_camera.pose.orientation.y = 0.0
        pose_camera.pose.orientation.z = 0.0
        pose_camera.pose.orientation.w = 1.0

        try:
            pose_camera.header.stamp = Time().to_msg()
            pose_map = self.tf_buffer_.transform(pose_camera, 'map', timeout=Duration(seconds=0.1))
            pose_map.header.stamp = self.get_clock().now().to_msg()

            tf_map_base = self.tf_buffer_.lookup_transform(
                'map', 'base_footprint', Time(),
                timeout=Duration(seconds=0.1))

            pose_map.pose.orientation.x = tf_map_base.transform.rotation.x
            pose_map.pose.orientation.y = tf_map_base.transform.rotation.y
            pose_map.pose.orientation.z = tf_map_base.transform.rotation.z
            pose_map.pose.orientation.w = tf_map_base.transform.rotation.w

            self.pose_pub_.publish(pose_map)
        except Exception as ex:
            self.get_logger().warn(f'Could not transform to map frame: {ex}')


def main(args=None):
    rclpy.init(args=args)
    node = ArUcoNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()