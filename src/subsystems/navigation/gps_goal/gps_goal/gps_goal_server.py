#!/usr/bin/env python3
import math

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from action_msgs.msg import GoalStatus

import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from msgs.action import NavigateToGPS
from msgs.msg import Heading
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Quaternion
from sensor_msgs.msg import NavSatFix


class GPSGoalServer(Node):
    def __init__(self):
        super().__init__('gps_goal_server')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('default_position_tolerance', 2.0)
        self.declare_parameter('heading_topic', 'heading')
        self.declare_parameter('gps_topic', 'gps/fix')

        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.default_tolerance = self.get_parameter('default_position_tolerance').value
        heading_topic = self.get_parameter('heading_topic').value
        gps_topic = self.get_parameter('gps_topic').value

        # ENU origin — set from first GPS fix received, not from parameters
        self.origin_lat = None
        self.origin_lon = None
        self.origin_alt = None

        self.current_heading_rad = None  # ENU radians: 0=East, CCW positive

        self.get_logger().info(
            f'GPS Goal Server starting — waiting for first fix on "{gps_topic}" to set ENU origin'
        )
        self.get_logger().info(
            f'map_frame="{self.map_frame}", robot_frame="{self.robot_frame}", '
            f'heading_topic="{heading_topic}"'
        )

        self.callback_group = ReentrantCallbackGroup()

        # TF2 buffer and listener to get robot pose in map frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribe to GPS fix — first message sets the ENU origin
        self.gps_sub = self.create_subscription(
            NavSatFix,
            gps_topic,
            self._gps_callback,
            10
        )
        self.get_logger().info(f'Subscribed to GPS topic: "{gps_topic}"')

        # Subscribe to heading published by mag_heading_node
        self.heading_sub = self.create_subscription(
            Heading,
            heading_topic,
            self._heading_callback,
            10
        )
        self.get_logger().info(f'Subscribed to heading topic: "{heading_topic}"')

        self._action_server = ActionServer(
            self,
            NavigateToGPS,
            'navigate_to_gps',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )

        self._nav2_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.callback_group
        )

        self.get_logger().info('GPS Goal Action Server ready')

    def _gps_callback(self, msg: NavSatFix):
        if self.origin_lat is not None:
            return  # origin already set, ignore subsequent messages
        self.origin_lat = msg.latitude
        self.origin_lon = msg.longitude
        self.origin_alt = msg.altitude if not math.isnan(msg.altitude) else 0.0
        self.get_logger().info(
            f'ENU origin set from first GPS fix: '
            f'lat={self.origin_lat:.6f}, lon={self.origin_lon:.6f}, alt={self.origin_alt:.2f} m'
        )

    def _heading_callback(self, msg: Heading):
        self.current_heading_rad = msg.heading
        self.get_logger().debug(
            f'Heading update: {math.degrees(self.current_heading_rad):.2f} deg ENU '
            f'(compass bearing: {msg.compass_bearing:.2f} deg)'
        )

    def goal_callback(self, goal_request):
        self.get_logger().info(
            f'Received GPS goal request: lat={goal_request.latitude:.6f}, '
            f'lon={goal_request.longitude:.6f}, tolerance={goal_request.position_tolerance:.2f}'
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def _get_map_to_enu_rotation(self) -> float | None:
        """Compute the rotation angle α (radians) of the map frame's +X axis in ENU.

        Derives the offset dynamically:
          - yaw_enu  = msg.heading (ENU radians, 0=East, CCW positive)
          - yaw_map  = robot yaw in map frame from TF
          - α        = yaw_enu - yaw_map

        A goal at (east, north) in ENU becomes:
          x_map = east*cos(α) + north*sin(α)
          y_map = -east*sin(α) + north*cos(α)
        """
        if self.current_heading_rad is None:
            self.get_logger().warn(
                'No heading data received yet — cannot compute map<->ENU rotation'
            )
            return None

        yaw_enu_rad = self.current_heading_rad
        self.get_logger().info(
            f'ENU yaw from heading: {math.degrees(yaw_enu_rad):.2f} deg'
        )

        # Look up robot's yaw in map frame
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f'TF lookup failed ("{self.map_frame}" → "{self.robot_frame}"): {e}'
            )
            return None

        q = transform.transform.rotation
        yaw_map_rad = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self.get_logger().info(
            f'Robot yaw in map frame: {math.degrees(yaw_map_rad):.2f} deg  '
            f'(q=[{q.x:.3f}, {q.y:.3f}, {q.z:.3f}, {q.w:.3f}])'
        )

        alpha = yaw_enu_rad - yaw_map_rad
        self.get_logger().info(
            f'Map→ENU rotation α = yaw_enu - yaw_map = '
            f'{math.degrees(yaw_enu_rad):.2f} - {math.degrees(yaw_map_rad):.2f} = {math.degrees(alpha):.2f} deg'
        )
        return alpha

    def _rotate_enu_to_map(self, east: float, north: float, alpha: float) -> tuple:
        """Rotate an ENU (east, north) position into map frame coordinates.

        Args:
            east:  east displacement from ENU origin (metres)
            north: north displacement from ENU origin (metres)
            alpha: map +X axis angle in ENU (radians), i.e. yaw_enu - yaw_map
        """
        cos_a = math.cos(alpha)
        sin_a = math.sin(alpha)
        x_map = east * cos_a + north * sin_a
        y_map = -east * sin_a + north * cos_a
        self.get_logger().info(
            f'ENU ({east:.3f}, {north:.3f}) → map ({x_map:.3f}, {y_map:.3f}) '
            f'with α={math.degrees(alpha):.2f} deg'
        )
        return x_map, y_map

    async def execute_callback(self, goal_handle: ServerGoalHandle):
        """Execute the GPS navigation goal."""
        self.get_logger().info('Executing GPS navigation goal...')

        lat = goal_handle.request.latitude
        lon = goal_handle.request.longitude
        tolerance = goal_handle.request.position_tolerance

        if tolerance <= 0.0:
            tolerance = self.default_tolerance

        if self.origin_lat is None:
            self.get_logger().error('ENU origin not set yet — no GPS fix received on gps/fix topic')
            goal_handle.abort()
            result = NavigateToGPS.Result()
            result.success = False
            result.message = 'ENU origin not set: no GPS fix received yet'
            return result

        self.get_logger().info(
            f'Target: lat={lat:.6f}, lon={lon:.6f}, tolerance={tolerance:.2f} m'
        )
        self.get_logger().info(
            f'ENU origin: lat={self.origin_lat:.6f}, lon={self.origin_lon:.6f}, alt={self.origin_alt:.2f} m'
        )

        try:
            east, north, up = self.gps_to_enu(lat, lon, self.origin_alt)
            self.get_logger().info(
                f'Raw ENU: east={east:.3f} m, north={north:.3f} m, up={up:.3f} m'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to convert GPS to ENU: {e}')
            goal_handle.abort()
            result = NavigateToGPS.Result()
            result.success = False
            result.message = f'GPS conversion failed: {e}'
            return result

        # Rotate ENU goal into map frame using heading + TF
        alpha = self._get_map_to_enu_rotation()
        if alpha is not None:
            x, y = self._rotate_enu_to_map(east, north, alpha)
        else:
            self.get_logger().warn(
                'Could not compute map frame rotation — falling back to raw ENU. '
                'Ensure heading topic is publishing and TF is available.'
            )
            x, y = east, north
            self.get_logger().info(f'Fallback (raw ENU): x={x:.3f} m, y={y:.3f} m')

        nav2_goal = NavigateToPose.Goal()
        nav2_goal.pose = PoseStamped()
        nav2_goal.pose.header.frame_id = self.map_frame
        nav2_goal.pose.header.stamp = self.get_clock().now().to_msg()
        nav2_goal.pose.pose.position.x = x
        nav2_goal.pose.pose.position.y = y
        nav2_goal.pose.pose.position.z = 0.0
        nav2_goal.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.get_logger().info(
            f'Sending to Nav2: x={x:.3f}, y={y:.3f} in frame "{self.map_frame}"'
        )

        if not self._nav2_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available')
            goal_handle.abort()
            result = NavigateToGPS.Result()
            result.success = False
            result.message = 'Nav2 action server not available'
            return result

        nav2_goal_future = self._nav2_client.send_goal_async(
            nav2_goal,
            feedback_callback=lambda feedback: self._nav2_feedback_callback(feedback, goal_handle)
        )

        nav2_goal_handle = await nav2_goal_future

        if not nav2_goal_handle.accepted:
            self.get_logger().error('Nav2 rejected the goal')
            goal_handle.abort()
            result = NavigateToGPS.Result()
            result.success = False
            result.message = 'Nav2 rejected the goal'
            return result

        self.get_logger().info('Nav2 accepted the goal, navigating...')

        nav2_result_future = nav2_goal_handle.get_result_async()
        nav2_result = await nav2_result_future

        if nav2_result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded!')
            goal_handle.succeed()
            result = NavigateToGPS.Result()
            result.success = True
            result.message = 'Successfully reached GPS goal'
        elif nav2_result.status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn('Navigation aborted')
            goal_handle.abort()
            result = NavigateToGPS.Result()
            result.success = False
            result.message = 'Navigation aborted by Nav2'
        elif nav2_result.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Navigation canceled')
            goal_handle.canceled()
            result = NavigateToGPS.Result()
            result.success = False
            result.message = 'Navigation canceled'
        else:
            self.get_logger().error(f'Navigation failed with status: {nav2_result.status}')
            goal_handle.abort()
            result = NavigateToGPS.Result()
            result.success = False
            result.message = f'Navigation failed with status {nav2_result.status}'

        return result

    def _nav2_feedback_callback(self, nav2_feedback, gps_goal_handle):
        feedback = NavigateToGPS.Feedback()
        feedback.distance_remaining = nav2_feedback.feedback.distance_remaining
        feedback.estimated_time_remaining = nav2_feedback.feedback.estimated_time_remaining
        feedback.current_pose = nav2_feedback.feedback.current_pose
        gps_goal_handle.publish_feedback(feedback)

    def gps_to_enu(self, lat: float, lon: float, alt: float) -> tuple:
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        origin_lat_rad = math.radians(self.origin_lat)
        origin_lon_rad = math.radians(self.origin_lon)

        R = 6378137.0

        lat_center = (lat_rad + origin_lat_rad) / 2.0

        east = R * (lon_rad - origin_lon_rad) * math.cos(lat_center)
        north = R * (lat_rad - origin_lat_rad)
        up = alt - self.origin_alt

        return (east, north, up)


def main(args=None):
    rclpy.init(args=args)

    gps_goal_server = GPSGoalServer()

    try:
        rclpy.spin(gps_goal_server)
    except KeyboardInterrupt:
        pass
    finally:
        gps_goal_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
