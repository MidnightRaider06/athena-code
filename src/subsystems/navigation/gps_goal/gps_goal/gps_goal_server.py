#!/usr/bin/env python3
import math

import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from action_msgs.msg import GoalStatus

from msgs.action import NavigateToGPS
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Quaternion


class GPSGoalServer(Node):
    def __init__(self):
        super().__init__('gps_goal_server')

        self.declare_parameter('origin_lat', 38.42391162772634)
        self.declare_parameter('origin_lon', -110.78490558433397)
        self.declare_parameter('origin_alt', 0.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('default_position_tolerance', 2.0)

        self.origin_lat = self.get_parameter('origin_lat').value
        self.origin_lon = self.get_parameter('origin_lon').value
        self.origin_alt = self.get_parameter('origin_alt').value
        self.map_frame = self.get_parameter('map_frame').value
        self.default_tolerance = self.get_parameter('default_position_tolerance').value

        self.get_logger().info(
            f'GPS Goal Server initialized with ENU origin: '
            f'({self.origin_lat:.6f}, {self.origin_lon:.6f}, {self.origin_alt:.2f})'
        )

        self.callback_group = ReentrantCallbackGroup()

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

    def goal_callback(self, goal_request):
        self.get_logger().info(
            f'Received GPS goal request: lat={goal_request.latitude:.6f}, '
            f'lon={goal_request.longitude:.6f}, tolerance={goal_request.position_tolerance:.2f}'
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle: ServerGoalHandle):
        """Execute the GPS navigation goal."""
        self.get_logger().info('Executing GPS navigation goal...')

        lat = goal_handle.request.latitude
        lon = goal_handle.request.longitude
        tolerance = goal_handle.request.position_tolerance

        if tolerance <= 0.0:
            tolerance = self.default_tolerance

        try:
            x, y, z = self.gps_to_enu(lat, lon, self.origin_alt)
            self.get_logger().info(
                f'Converted GPS ({lat:.6f}, {lon:.6f}) to ENU: ({x:.2f}, {y:.2f}, {z:.2f})'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to convert GPS to ENU: {e}')
            goal_handle.abort()
            result = NavigateToGPS.Result()
            result.success = False
            result.message = f'GPS conversion failed: {e}'
            return result

        nav2_goal = NavigateToPose.Goal()
        nav2_goal.pose = PoseStamped()
        nav2_goal.pose.header.frame_id = self.map_frame
        nav2_goal.pose.header.stamp = self.get_clock().now().to_msg()
        nav2_goal.pose.pose.position.x = x
        nav2_goal.pose.pose.position.y = y
        nav2_goal.pose.pose.position.z = 0.0

        nav2_goal.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        if not self._nav2_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available')
            goal_handle.abort()
            result = NavigateToGPS.Result()
            result.success = False
            result.message = 'Nav2 action server not available'
            return result

        self.get_logger().info(
            f'Sending goal to Nav2: x={x:.2f}, y={y:.2f} in frame {self.map_frame}'
        )

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
