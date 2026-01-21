import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from msgs.msg import BB

class CorrectionNode(Node):
    def __init__(self):
        super().__init__('correction_node')

        self.sub = self.create_subscription(BB, 'aruco_loc', self.correction_callback, 10)
        self.heading_pub = self.create_publisher(Twist, 'heading', 10)

        # PD coefficients, need tuning
        self.kProp = 0.01 
        self.kDer = 0.005 

        # Error Tracking
        self.currError = None
        self.prevError = None

    def correction_callback(self, msg):
        error = (msg.bb_bottom_right_x + msg.bb_top_left_x - msg.img_width) / 2

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
        max_angular_speed = 1.5  # Adjust later if needed
        output = max(-max_angular_speed, min(output, max_angular_speed))

        # Create Twist message
        correction_message = Twist()
        correction_message.angular.z = output  # Rotate based on PD output
        
        # Stop if within stop_distance, otherwise move forward if nearly aligned
        if abs(error) < 10:
            correction_message.angular.z = 0.0

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