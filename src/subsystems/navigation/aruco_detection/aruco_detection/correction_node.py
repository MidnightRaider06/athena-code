import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from vision_msgs.msg import Detection2D
from sensor_msgs.msg import Image

class CorrectionNode(Node):
    def __init__(self):
        super().__init__('correction_node')

        self.declare_parameter("sim", False)
        self.sim = self.get_parameter("sim").get_parameter_value().bool_value

        if self.sim:
            twist_topic = '/ackermann_steering_controller/reference'
            image_topic = '/camera'
        else:
            twist_topic = '/ackermann_steering_controller/reference' # TO-DO: add actual topic name
            image_topic = '/zed/zed_node/left_gray/image_rect_gray'

        self.get_logger().info(f"sim={self.sim}")
        self.get_logger().info(f"Publishing twist messages to topic: {twist_topic}")

        self.sub = self.create_subscription(Detection2D, 'aruco_loc', self.correction_callback, 10)
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.heading_pub = self.create_publisher(TwistStamped, twist_topic, 10)

        # PD coefficients, need tuning
        self.kProp = 0.01 
        self.kDer = 0.005 

        self.img_w = None
        self.img_h = None

        # Error Tracking
        self.currError = None
        self.prevError = None

    def image_callback(self, msg: Image):
        self.img_w = msg.width
        self.img_h = msg.height


    def correction_callback(self, msg: Detection2D):
        # stop if tag fills >7% of frame
        if self.img_w is None or self.img_h is None:
            #self.get_logger().warn("No image size received yet; skipping area-based stop check")
            return
        else:
            bbox_area_ratio = (msg.bbox.size_x * msg.bbox.size_y) / (float(self.img_w) * float(self.img_h))
            if bbox_area_ratio > 0.07:
                stop_msg = TwistStamped()
                stop_msg.header.stamp = self.get_clock().now().to_msg()
                stop_msg.twist.linear.x = 0.0
                stop_msg.twist.angular.z = 0.0
                self.heading_pub.publish(stop_msg)
                return

        if self.img_w is None:
            return  # no frame width yet
        error = msg.bbox.center.position.x - (float(self.img_w) * 0.5)


        # Initialize error values if first time
        if self.currError is None:
            self.currError = error
            self.prevError = error  # Avoid large derivative spikes
            return  # Skip first iteration

        self.prevError = self.currError
        self.currError = error

        # Compute PD output
        output = (self.kProp * self.currError) + (self.kDer * (self.currError - self.prevError))

        # Limit output to a reasonable angular speed
        max_angular_speed = 1.5
        output = max(-max_angular_speed, min(output, max_angular_speed))

        # Create Twist message
        correction_message = TwistStamped()
        correction_message.header.stamp = self.get_clock().now().to_msg()
        correction_message.twist.angular.z = output  # Rotate based on PD output

        correction_message.twist.linear.x = 0.5
        
        # Move forward if nearly aligned
        if abs(error) < 15:
            correction_message.twist.angular.z = 0.0
            correction_message.twist.linear.x = 1.0

        # Publish correction
        self.heading_pub.publish(correction_message)

def main(args=None):
    rclpy.init(args=args)
    node = CorrectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()