import numpy as np
import cv2
import cv2.aruco

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import Detection2D, BoundingBox2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs


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

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector_ = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

        self.tf_buffer_ = tf2_ros.Buffer()
        self.tf_listener_ = tf2_ros.TransformListener(self.tf_buffer_, self)

        self.bridge_ = CvBridge()

        self.image_sub_ = self.create_subscription(
            Image, image_topic, self.image_callback, 1)

        self.camera_info_sub_ = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, 10)

        self.detection_pub_ = self.create_publisher(Detection2D, '/aruco_detection', 10)

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

        self.detect_aruco_markers(cv_ptr)

        if self.marker_id_ >= 0 and len(self.corners_) > 0 and self.camera_info_received_:
            pose = self.estimate_pose(msg)
            if pose is not None:
                xs = [pt[0] for pt in self.corners_]
                ys = [pt[1] for pt in self.corners_]

                detection = Detection2D()
                detection.header.stamp = self.get_clock().now().to_msg()
                detection.header.frame_id = 'map'
                detection.bbox.center.position.x = float(sum(xs) / 4)
                detection.bbox.center.position.y = float(sum(ys) / 4)
                detection.bbox.size_x = float(max(xs) - min(xs))
                detection.bbox.size_y = float(max(ys) - min(ys))

                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(self.marker_id_)
                hypothesis.hypothesis.score = 1.0
                hypothesis.pose.pose = pose
                detection.results.append(hypothesis)

                self.detection_pub_.publish(detection)

    def detect_aruco_markers(self, frame):
        corners, ids, _ = self.aruco_detector_.detectMarkers(frame)

        if ids is not None and len(ids) > 0:
            self.marker_id_ = int(ids[0][0])
            self.all_corners_ = [corners[0]]
            self.corners_ = [pt for pt in corners[0][0]]
        else:
            self.marker_id_ = -1
            self.corners_ = []
            self.all_corners_ = []

    def estimate_pose(self, msg):
        if len(self.all_corners_) == 0:
            return None

        if self.marker_size_ <= 0:
            self.get_logger().warn(f'Invalid marker_size_: {self.marker_size_:.3f}')
            return None

        half = self.marker_size_ / 2.0
        obj_points = np.array([[-half,  half, 0],[ half,  half, 0],[ half, -half, 0],[-half, -half, 0]], dtype=np.float32)

        img_points = self.all_corners_[0].reshape(4, 2)
        _, self.rvec_, self.tvec_ = cv2.solvePnP(
            obj_points, img_points,
            self.camera_matrix_, self.dist_coeffs_,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)

        pose_camera = PoseStamped()
        pose_camera.header.frame_id = msg.header.frame_id

        pose_camera.pose.position.x = float(self.tvec_[0][0])
        pose_camera.pose.position.y = float(self.tvec_[1][0])
        pose_camera.pose.position.z = float(self.tvec_[2][0])

        pose_camera.pose.orientation.x = 0.0
        pose_camera.pose.orientation.y = 0.0
        pose_camera.pose.orientation.z = 0.0
        pose_camera.pose.orientation.w = 1.0

        try:
            pose_camera.header.stamp = Time().to_msg()
            pose_map = self.tf_buffer_.transform(pose_camera, 'map', timeout=Duration(seconds=0.1))

            tf_map_base = self.tf_buffer_.lookup_transform(
                'map', 'base_footprint', Time(),
                timeout=Duration(seconds=0.1))

            pose_map.pose.orientation.x = tf_map_base.transform.rotation.x
            pose_map.pose.orientation.y = tf_map_base.transform.rotation.y
            pose_map.pose.orientation.z = tf_map_base.transform.rotation.z
            pose_map.pose.orientation.w = tf_map_base.transform.rotation.w

            return pose_map.pose
        except Exception as ex:
            self.get_logger().warn(f'Could not transform to map frame: {ex}')
            return None


def main(args=None):
    rclpy.init(args=args)
    node = ArUcoNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()