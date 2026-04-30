#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from action_msgs.msg import GoalStatus, GoalStatusArray
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion

from nav2_msgs.action import NavigateToPose

from msgs.msg import WaypointEntry, WaypointManagerState
from msgs.srv import ListWaypoints, GetWaypoint
from msgs.action import NavigateToGPS, NavigateToWaypoint


class WaypointManagerNode(Node):
    def __init__(self):
        super().__init__('waypoint_manager')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')
        self._map_frame = self.get_parameter('map_frame').value
        self._robot_frame = self.get_parameter('robot_frame').value

        # In-memory store: only locations where Nav2 reported STATUS_SUCCEEDED
        self._waypoints: dict[str, dict] = {}
        self._wp_counter = 0

        # Track nav2 goal statuses to detect SUCCEEDED transitions
        self._seen_goal_statuses: dict[bytes, int] = {}

        self._nav_status = 'idle'
        self._active_nav_label = ''

        self._cb_group = ReentrantCallbackGroup()

        # TF listener for pose lookup on nav success
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # /nav_mode publisher — QoS must match NavSelector (transient_local, reliable, keep_last(1))
        nav_mode_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._nav_mode_pub = self.create_publisher(String, '/nav_mode', nav_mode_qos)

        # State broadcast at 2 Hz
        self._state_pub = self.create_publisher(
            WaypointManagerState, '/waypoint_manager/state', 10
        )
        self.create_timer(0.5, self._publish_state)

        # Watch Nav2 goal statuses — store waypoint whenever navigate_to_pose succeeds
        self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self._nav2_status_cb,
            10,
        )

        # Query services
        self.create_service(ListWaypoints, '~/list_waypoints', self._list_waypoints_cb)
        self.create_service(GetWaypoint, '~/get_waypoint', self._get_waypoint_cb)

        # NavigateToWaypoint action server — command interface only, does not store waypoints
        self._nav_action_server = ActionServer(
            self,
            NavigateToWaypoint,
            '~/navigate_to_waypoint',
            execute_callback=self._navigate_execute_cb,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cb_group,
        )

        # Nav2 NavigateToPose action client (map-frame goals)
        self._nav2_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self._cb_group,
        )

        # GPS action client (lat/lon goals — requires gps_goal server to be running)
        self._gps_client = ActionClient(
            self,
            NavigateToGPS,
            'navigate_to_gps',
            callback_group=self._cb_group,
        )

        self.get_logger().info('WaypointManager ready')

    # ── Nav2 status watcher ───────────────────────────────────────────────────

    def _nav2_status_cb(self, msg: GoalStatusArray):
        for entry in msg.status_list:
            uid = bytes(entry.goal_info.goal_id.uuid)
            prev = self._seen_goal_statuses.get(uid)
            curr = entry.status

            if prev != GoalStatus.STATUS_SUCCEEDED and curr == GoalStatus.STATUS_SUCCEEDED:
                self._store_current_pose()

            self._seen_goal_statuses[uid] = curr

    def _store_current_pose(self):
        try:
            t = self._tf_buffer.lookup_transform(
                self._map_frame, self._robot_frame, rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed on nav success, waypoint not stored: {e}')
            return

        x = t.transform.translation.x
        y = t.transform.translation.y
        q = t.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        self._wp_counter += 1
        wp_id = f'wp_{self._wp_counter}'
        now = self.get_clock().now().to_msg()
        self._waypoints[wp_id] = {
            'id': wp_id,
            'x': x,
            'y': y,
            'yaw': yaw,
            'description': '',
            'timestamp': {'sec': now.sec, 'nanosec': now.nanosec},
        }
        self.get_logger().info(
            f'Nav2 goal succeeded — stored "{wp_id}" at ({x:.2f}, {y:.2f})'
        )

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _dict_to_msg(self, d: dict) -> WaypointEntry:
        msg = WaypointEntry()
        msg.id = d['id']
        msg.x = float(d['x'])
        msg.y = float(d['y'])
        msg.yaw = float(d['yaw'])
        msg.description = d.get('description', '')
        msg.visited = True
        ts = d.get('timestamp', {})
        msg.timestamp.sec = int(ts.get('sec', 0))
        msg.timestamp.nanosec = int(ts.get('nanosec', 0))
        return msg

    # ── State broadcast ───────────────────────────────────────────────────────

    def _publish_state(self):
        msg = WaypointManagerState()
        msg.active_waypoint_id = self._active_nav_label
        msg.nav_status = self._nav_status
        msg.waypoints = [self._dict_to_msg(d) for d in self._waypoints.values()]
        msg.last_update = self.get_clock().now().to_msg()
        self._state_pub.publish(msg)

    # ── Service callbacks ─────────────────────────────────────────────────────

    def _list_waypoints_cb(self, request, response):
        response.waypoints = [self._dict_to_msg(d) for d in self._waypoints.values()]
        return response

    def _get_waypoint_cb(self, request, response):
        if request.id not in self._waypoints:
            response.found = False
            return response
        response.found = True
        response.waypoint = self._dict_to_msg(self._waypoints[request.id])
        return response

    # ── Action callbacks ──────────────────────────────────────────────────────

    def _goal_callback(self, goal_request):
        if self._nav_status == 'navigating':
            self.get_logger().warn('Goal rejected: navigation already in progress')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel requested for active navigation')
        return CancelResponse.ACCEPT

    async def _navigate_execute_cb(self, goal_handle: ServerGoalHandle):
        req = goal_handle.request
        label = req.name or 'unnamed'

        self._active_nav_label = label
        self._nav_status = 'navigating'

        self._nav_mode_pub.publish(String(data='GNSS'))

        result = NavigateToWaypoint.Result()

        def _feedback_cb(fb):
            feedback = NavigateToWaypoint.Feedback()
            feedback.status = 'navigating'
            feedback.distance_remaining = fb.feedback.distance_remaining
            goal_handle.publish_feedback(feedback)

        if req.use_gps:
            self.get_logger().info(
                f'Navigating to "{label}" via GPS '
                f'(lat={req.latitude:.6f}, lon={req.longitude:.6f})'
            )
            if not self._gps_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('navigate_to_gps server not available')
                self._nav_status = 'failed'
                self._active_nav_label = ''
                goal_handle.abort()
                result.success = False
                result.message = 'navigate_to_gps server not available'
                return result

            gps_goal = NavigateToGPS.Goal()
            gps_goal.latitude = req.latitude
            gps_goal.longitude = req.longitude
            gps_goal.position_tolerance = req.position_tolerance

            send_future = self._gps_client.send_goal_async(gps_goal, feedback_callback=_feedback_cb)
        else:
            self.get_logger().info(
                f'Navigating to "{label}" via map frame '
                f'({req.x:.2f}, {req.y:.2f}), yaw={req.yaw:.2f}'
            )
            if not self._nav2_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('navigate_to_pose server not available')
                self._nav_status = 'failed'
                self._active_nav_label = ''
                goal_handle.abort()
                result.success = False
                result.message = 'navigate_to_pose server not available'
                return result

            pose_goal = NavigateToPose.Goal()
            pose_goal.pose = PoseStamped()
            pose_goal.pose.header.frame_id = self._map_frame
            pose_goal.pose.header.stamp = self.get_clock().now().to_msg()
            pose_goal.pose.pose.position.x = req.x
            pose_goal.pose.pose.position.y = req.y
            pose_goal.pose.pose.position.z = 0.0
            pose_goal.pose.pose.orientation = Quaternion(
                x=0.0,
                y=0.0,
                z=math.sin(req.yaw / 2.0),
                w=math.cos(req.yaw / 2.0),
            )

            send_future = self._nav2_client.send_goal_async(pose_goal, feedback_callback=_feedback_cb)

        nav_goal_handle = await send_future

        if not nav_goal_handle.accepted:
            self.get_logger().error('Goal rejected by navigation server')
            self._nav_status = 'failed'
            self._active_nav_label = ''
            goal_handle.abort()
            result.success = False
            result.message = 'Goal rejected by navigation server'
            return result

        nav_result = await nav_goal_handle.get_result_async()

        if goal_handle.is_cancel_requested:
            self._nav_status = 'idle'
            self._active_nav_label = ''
            goal_handle.canceled()
            result.success = False
            result.message = 'Canceled'
            return result

        if nav_result.status == GoalStatus.STATUS_SUCCEEDED:
            self._nav_status = 'success'
            goal_handle.succeed()
            result.success = True
            result.message = f'Reached "{label}"'
        elif nav_result.status == GoalStatus.STATUS_CANCELED:
            self._nav_status = 'idle'
            goal_handle.canceled()
            result.success = False
            result.message = 'Navigation canceled'
        else:
            self._nav_status = 'failed'
            self.get_logger().warn(
                f'Navigation to "{label}" failed (status={nav_result.status})'
            )
            goal_handle.abort()
            result.success = False
            result.message = f'Navigation failed (status={nav_result.status})'

        self._active_nav_label = ''
        return result


def main(args=None):
    rclpy.init(args=args)
    node = WaypointManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
