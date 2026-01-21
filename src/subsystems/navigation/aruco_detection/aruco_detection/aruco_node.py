import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from msgs.msg import BB

class ZedArUcoNode(Node):
    def __init__(self):
        super().__init__('zed_aruco_node')

        # Initialize CvBridge to convert ROS images to OpenCV
        self.bridge = CvBridge()

        # Subscribe to the ZED camera image and depth topics
        self.image_sub = self.create_subscription(
            Image, '/zed/zed_node/left_gray/image_rect_gray', self.image_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, '/zed/zed_node/depth/depth_registered', self.depth_callback, 10
        )

        self.image_pub = self.create_publisher(Image, 'annotated_img', 10)
        self.tag_pub = self.create_publisher(BB, 'aruco_loc', 10)

        self.depth_pub = self.create_publisher(String, 'depth', 10)

        #Load the ArUco dictionary
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.corners = None
        self.marker_id = None
        self.latest_depth_map = None

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV Image
        frame = self.bridge.imgmsg_to_cv2(msg, "mono8")
        frame_color = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        # Process image (ArUco detection, etc.)
        self.detect_aruco_markers(frame)

        # If we have a valid marker and a valid depth map, publish the information
        if self.marker_id is not None and self.latest_depth_map is not None:

            for i in range(4):
                start_point = tuple(map(int, self.corners[i]))
                end_point = tuple(map(int, self.corners[(i + 1) % 4]))  # Connect to next corner
                cv2.line(frame_color, start_point, end_point, (0, 255, 0), 2)  # Draw green lines


            depth_values = [
                self.latest_depth_map[int(self.corners[0][1]), int(self.corners[0][0])],  # Top-left
                self.latest_depth_map[int(self.corners[1][1]), int(self.corners[1][0])],  # Top-right
                self.latest_depth_map[int(self.corners[2][1]), int(self.corners[2][0])],  # Bottom-right
                self.latest_depth_map[int(self.corners[3][1]), int(self.corners[3][0])]   # Bottom-left
            ]

            depth_avg = sum(depth_values) / len(depth_values)  # Compute the average depth

            # Publish tag bounding box
            message = BB()
            message.img_width = len(frame[0])
            message.img_height = len(frame)

            # Extract x and y coordinates from all four corners
            x_values = [int(pt[0]) for pt in self.corners]
            y_values = [int(pt[1]) for pt in self.corners]

            message.bb_top_left_x = min(x_values)
            message.bb_top_left_y = min(y_values)
            message.bb_bottom_right_x = max(x_values)
            message.bb_bottom_right_y = max(y_values)
            
            depth_message = String()
            depth_message.data = str(depth_avg)

            self.depth_pub.publish(depth_message)
            self.tag_pub.publish(message)

        annotated_msg = self.bridge.cv2_to_imgmsg(frame_color, encoding="bgr8")
        self.image_pub.publish(annotated_msg)

    def depth_callback(self, msg):
        # Convert ROS Image to depth map
        self.latest_depth_map = self.bridge.imgmsg_to_cv2(msg, "32FC1")

    def detect_aruco_markers(self, frame):
        
        corners, ids, _  = self.aruco_detector.detectMarkers(frame)
        if ids is not None:
                for i in range(len(ids)):
                    self.marker_id = ids[i][0]
                    self.corners = corners[i][0]
        else:
            self.marker_id = None

def main(args=None):
    rclpy.init(args=args)
    node = ZedArUcoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()