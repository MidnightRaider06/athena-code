import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from vision_msgs.msg import Detection2D


class ZedArUcoNode(Node):
    def __init__(self):
        super().__init__('zed_aruco_node')

        self.declare_parameter("sim", False)
        self.sim = self.get_parameter("sim").get_parameter_value().bool_value

        if self.sim:
            image_topic = '/camera'
            depth_topic = '/depth_camera'
        else:
            image_topic = '/zed/zed_node/left_gray/image_rect_gray'
            depth_topic = '/zed/zed_node/depth/depth_registered'

        self.get_logger().info(f"sim={self.sim}")
        self.get_logger().info(f"Subscribing to image feed: {image_topic}")
        self.get_logger().info(f"Subscribing to depth image feed: {depth_topic}")

        # Initialize CvBridge to convert ROS images to OpenCV
        self.bridge = CvBridge()

        # Subscribe to the ZED camera image and depth topics
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, depth_topic, self.depth_callback, 10
        )

        self.image_pub = self.create_publisher(Image, 'aruco_annotated_img', 10)
        self.tag_pub = self.create_publisher(Detection2D, 'aruco_loc', 10)
        self.depth_pub = self.create_publisher(String, 'aruco_depth', 10)

        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

        try:
            self.aruco_params = cv2.aruco.DetectorParameters()
        except AttributeError:
            self.aruco_params = cv2.aruco.DetectorParameters_create()

        self.corners = None
        self.marker_id = None
        self.latest_depth_map = None

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "mono8")
        frame_color = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        # Process image (ArUco detection, etc.)
        self.detect_aruco_markers(frame)

        # If we have a valid marker and a valid depth map, publish the information
        if self.marker_id is not None and self.corners is not None:

            for i in range(4):
                start_point = tuple(map(int, self.corners[i]))
                end_point = tuple(map(int, self.corners[(i + 1) % 4]))  # Connect to next corner
                cv2.line(frame_color, start_point, end_point, (0, 255, 0), 2)  # Draw green lines
            
            if self.corners is not None:
                text_pos = tuple(map(int, self.corners[0]))

                cv2.putText(
                    frame_color,
                    f"id: {self.marker_id}",
                    (text_pos[0], text_pos[1] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA
                )

            if not self.sim:
                if self.latest_depth_map is not None:
                    depth_values = [
                        self.latest_depth_map[int(self.corners[0][1]), int(self.corners[0][0])],  # Top-left
                        self.latest_depth_map[int(self.corners[1][1]), int(self.corners[1][0])],  # Top-right
                        self.latest_depth_map[int(self.corners[2][1]), int(self.corners[2][0])],  # Bottom-right
                        self.latest_depth_map[int(self.corners[3][1]), int(self.corners[3][0])]   # Bottom-left
                    ]

                    depth_avg = sum(depth_values) / len(depth_values)
                    depth_message = String()
                    depth_message.data = str(depth_avg)
                    self.depth_pub.publish(depth_message)
                else:
                    self.get_logger().warn("Depth map not received yet, skipping depth calc")

            # Publish tag bounding box as vision_msgs/Detection2D
            message = Detection2D()
            message.header = msg.header  # keep same timestamp/frame as the image

            # Extract x and y coordinates from all four corners
            x_values = [float(pt[0]) for pt in self.corners]
            y_values = [float(pt[1]) for pt in self.corners]

            x_min = min(x_values)
            y_min = min(y_values)
            x_max = max(x_values)
            y_max = max(y_values)

            # Detection2D bbox is defined by center pose (Pose2D) + size_x/size_y in pixels
            message.bbox.center.position.x = (x_min + x_max) * 0.5
            message.bbox.center.position.x = (y_min + y_max) * 0.5
            message.bbox.center.theta = 0.0  # axis-aligned bbox
            message.bbox.size_x = (x_max - x_min)
            message.bbox.size_y = (y_max - y_min)

            # message.results can be left empty (CorrectionNode only needs bbox)
            self.tag_pub.publish(message)


        annotated_msg = self.bridge.cv2_to_imgmsg(frame_color, encoding="bgr8")
        annotated_msg.header = msg.header
        self.image_pub.publish(annotated_msg)


    def depth_callback(self, msg):
        # Convert ROS Image to depth map
        self.latest_depth_map = self.bridge.imgmsg_to_cv2(msg, "32FC1")

    def detect_aruco_markers(self, frame):
        corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
                for i in range(len(ids)):
                    self.marker_id = ids[i][0]
                    self.corners = corners[i][0]
        else:
            self.marker_id = None
            self.corners = None

def main(args=None):
    rclpy.init(args=args)
    node = ZedArUcoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()