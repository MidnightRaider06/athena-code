#!/usr/bin/env python3

import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from action_msgs.msg import GoalStatus, GoalStatusArray

from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

from msgs.msg import WaypointEntry, WaypointManagerState
from msgs.srv import AddWaypoint, ClearWaypoints
from msgs.action import NavigateToGPS, NavigateToWaypoint


class WaypointManagerNode(Node):
    def __init__(self):
        super().__init__('waypoint_manager')

        # In-memory store: uint32 id → {id, latitude, longitude, timestamp}
        self._waypoints: dict[int, dict] = {}
        self._wp_counter = 0

        self._latest_gps: NavSatFix | None = None
        self._seen_goal_statuses: dict[bytes, int] = {}

        self._nav_status = WaypointManagerState.STATUS_IDLE
        self._active_waypoint_id = 0  # 0 = none
        self._current_nav_goal_handle = None

        self._cb_group = ReentrantCallbackGroup()

        self.declare_parameter('gps_topic', '/gps/fix')
        self.declare_parameter('nav_mode_topic', '/nav_mode')
        self.declare_parameter('nav2_status_topic', '/navigate_to_pose/_action/status')
        self.declare_parameter('navigate_to_gps_action', 'navigate_to_gps')
        self.declare_parameter('waypoints_file', os.path.expanduser('~/.ros/waypoints.yaml'))
        self.declare_parameter('load_from_file', True)

        gps_topic = self.get_parameter('gps_topic').value
        nav_mode_topic = self.get_parameter('nav_mode_topic').value
        nav2_status_topic = self.get_parameter('nav2_status_topic').value
        navigate_to_gps_action = self.get_parameter('navigate_to_gps_action').value
        self._waypoints_file = os.path.expanduser(
            self.get_parameter('waypoints_file').value
        )

        if self.get_parameter('load_from_file').value:
            self._load_waypoints()

        # QoS must match NavSelector (transient_local, reliable, depth=1)
        nav_mode_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._nav_mode_pub = self.create_publisher(String, nav_mode_topic, nav_mode_qos)

        self._state_pub = self.create_publisher(
            WaypointManagerState, '/waypoint_manager/state', 10
        )
        self.create_timer(0.5, self._publish_state)

        self.create_subscription(NavSatFix, gps_topic, self._gps_cb, 10)
        self.get_logger().info(f'Subscribed to {gps_topic}')

        self.create_subscription(
            GoalStatusArray,
            nav2_status_topic,
            self._nav2_status_cb,
            10,
        )
        self.get_logger().info(f'Subscribed to {nav2_status_topic}')

        self.create_service(AddWaypoint, '~/add_waypoint', self._add_waypoint_cb)
        self.create_service(ClearWaypoints, '~/clear_waypoints', self._clear_waypoints_cb)

        self._nav_action_server = ActionServer(
            self,
            NavigateToWaypoint,
            '~/navigate_to_waypoint',
            execute_callback=self._navigate_execute_cb,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cb_group,
        )

        self._gps_client = ActionClient(
            self,
            NavigateToGPS,
            navigate_to_gps_action,
            callback_group=self._cb_group,
        )

        self.get_logger().info('WaypointManager ready')

    # ── File persistence ──────────────────────────────────────────────────────

    def _load_waypoints(self):
        if not os.path.exists(self._waypoints_file):
            self.get_logger().info(f'No waypoints file at {self._waypoints_file}, starting empty')
            return
        try:
            with open(self._waypoints_file, 'r') as f:
                data = yaml.safe_load(f)
            if not data:
                return
            self._wp_counter = int(data.get('counter', 0))
            for wp_id, d in data.get('waypoints', {}).items():
                wp_id = int(wp_id)
                self._waypoints[wp_id] = {
                    'id': wp_id,
                    'latitude': float(d['latitude']),
                    'longitude': float(d['longitude']),
                    'timestamp': d.get('timestamp', {'sec': 0, 'nanosec': 0}),
                }
            self.get_logger().info(
                f'Loaded {len(self._waypoints)} waypoint(s) from {self._waypoints_file}'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints file: {e}')

    def _save_waypoints(self):
        data = {
            'counter': self._wp_counter,
            'waypoints': {
                wp_id: {
                    'latitude': d['latitude'],
                    'longitude': d['longitude'],
                    'timestamp': d['timestamp'],
                }
                for wp_id, d in self._waypoints.items()
            },
        }
        tmp = self._waypoints_file + '.tmp'
        try:
            os.makedirs(os.path.dirname(self._waypoints_file), exist_ok=True)
            with open(tmp, 'w') as f:
                yaml.dump(data, f)
            os.replace(tmp, self._waypoints_file)
        except Exception as e:
            self.get_logger().error(f'Failed to save waypoints file: {e}')

    # ── GPS fix cache ─────────────────────────────────────────────────────────

    def _gps_cb(self, msg: NavSatFix):
        if self._latest_gps is None:
            self.get_logger().info(
                f'First GPS fix received: ({msg.latitude:.6f}, {msg.longitude:.6f}), '
                f'status={msg.status.status}'
            )
        self._latest_gps = msg

    # ── Nav2 status watcher ───────────────────────────────────────────────────

    def _nav2_status_cb(self, msg: GoalStatusArray):
        for entry in msg.status_list:
            uid = bytes(entry.goal_info.goal_id.uuid)
            prev = self._seen_goal_statuses.get(uid)
            curr = entry.status
            if prev != GoalStatus.STATUS_SUCCEEDED and curr == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Nav2 goal succeeded — storing GPS waypoint')
                self._store_current_gps()
            self._seen_goal_statuses[uid] = curr

    def _store_current_gps(self):
        if self._latest_gps is None:
            self.get_logger().warn('No GPS fix available; waypoint not stored')
            return
        gps = self._latest_gps
        self._wp_counter += 1
        wp_id = self._wp_counter
        now = self.get_clock().now().to_msg()
        self._waypoints[wp_id] = {
            'id': wp_id,
            'latitude': gps.latitude,
            'longitude': gps.longitude,
            'timestamp': {'sec': now.sec, 'nanosec': now.nanosec},
        }
        self.get_logger().info(
            f'Nav2 goal succeeded — stored waypoint {wp_id} at '
            f'({gps.latitude:.6f}, {gps.longitude:.6f})'
        )
        self._save_waypoints()

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _dict_to_msg(self, d: dict) -> WaypointEntry:
        msg = WaypointEntry()
        msg.id = d['id']
        msg.latitude = float(d['latitude'])
        msg.longitude = float(d['longitude'])
        ts = d.get('timestamp', {})
        msg.timestamp.sec = int(ts.get('sec', 0))
        msg.timestamp.nanosec = int(ts.get('nanosec', 0))
        return msg

    # ── State broadcast ───────────────────────────────────────────────────────

    def _publish_state(self):
        msg = WaypointManagerState()
        msg.active_waypoint_id = self._active_waypoint_id
        msg.nav_status = self._nav_status
        msg.waypoints = [self._dict_to_msg(d) for d in self._waypoints.values()]
        msg.last_update = self.get_clock().now().to_msg()
        self._state_pub.publish(msg)

    # ── Service callbacks ──────────────────────────────────────────────────────

    def _add_waypoint_cb(self, request, response):
        now = self.get_clock().now().to_msg()
        self._wp_counter += 1
        wp_id = self._wp_counter
        self._waypoints[wp_id] = {
            'id': wp_id,
            'latitude': request.latitude,
            'longitude': request.longitude,
            'timestamp': {'sec': now.sec, 'nanosec': now.nanosec},
        }
        self._save_waypoints()
        self.get_logger().info(
            f'Manually added waypoint {wp_id} at '
            f'({request.latitude:.6f}, {request.longitude:.6f})'
        )
        response.success = True
        response.message = f'Added waypoint {wp_id}'
        response.assigned_id = wp_id
        return response

    def _clear_waypoints_cb(self, request, response):
        count = len(self._waypoints)
        self._waypoints.clear()
        self._wp_counter = 0
        self._save_waypoints()
        response.success = True
        response.message = f'Cleared {count} waypoint(s)'
        self.get_logger().info(f'Cleared {count} waypoint(s)')
        return response

    # ── Action callbacks ──────────────────────────────────────────────────────

    def _goal_callback(self, goal_request):
        if self._nav_status == WaypointManagerState.STATUS_NAVIGATING:
            self.get_logger().warn('Goal rejected: navigation already in progress')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel requested — forwarding to navigate_to_gps')
        if self._current_nav_goal_handle is not None:
            self._current_nav_goal_handle.cancel_goal_async()
        return CancelResponse.ACCEPT

    async def _navigate_execute_cb(self, goal_handle: ServerGoalHandle):
        wp_id = goal_handle.request.waypoint_id
        result = NavigateToWaypoint.Result()

        if wp_id not in self._waypoints:
            self.get_logger().error(f'Waypoint {wp_id} not found')
            goal_handle.abort()
            result.success = False
            result.message = f'Waypoint {wp_id} not found'
            return result

        wp = self._waypoints[wp_id]
        self._active_waypoint_id = wp_id
        self._nav_status = WaypointManagerState.STATUS_NAVIGATING
        self._nav_mode_pub.publish(String(data='GNSS'))

        self.get_logger().info(
            f'Navigating to waypoint {wp_id} '
            f'(lat={wp["latitude"]:.6f}, lon={wp["longitude"]:.6f})'
        )

        def _feedback_cb(fb):
            feedback = NavigateToWaypoint.Feedback()
            feedback.status = 'navigating'
            feedback.distance_remaining = fb.feedback.distance_remaining
            goal_handle.publish_feedback(feedback)

        if not self._gps_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('navigate_to_gps server not available')
            self._nav_status = WaypointManagerState.STATUS_FAILED
            self._active_waypoint_id = 0
            goal_handle.abort()
            result.success = False
            result.message = 'navigate_to_gps server not available'
            return result

        gps_goal = NavigateToGPS.Goal()
        gps_goal.latitude = wp['latitude']
        gps_goal.longitude = wp['longitude']
        gps_goal.position_tolerance = 0.0  # use gps_goal server default

        send_future = self._gps_client.send_goal_async(gps_goal, feedback_callback=_feedback_cb)
        nav_goal_handle = await send_future

        if not nav_goal_handle.accepted:
            self.get_logger().error('Goal rejected by navigate_to_gps server')
            self._nav_status = WaypointManagerState.STATUS_FAILED
            self._active_waypoint_id = 0
            goal_handle.abort()
            result.success = False
            result.message = 'Goal rejected by navigate_to_gps server'
            return result

        self._current_nav_goal_handle = nav_goal_handle
        try:
            nav_result = await nav_goal_handle.get_result_async()
        finally:
            self._current_nav_goal_handle = None

        if goal_handle.is_cancel_requested:
            self._nav_status = WaypointManagerState.STATUS_CANCELED
            self._active_waypoint_id = 0
            goal_handle.canceled()
            result.success = False
            result.message = 'Canceled'
            return result

        if nav_result.status == GoalStatus.STATUS_SUCCEEDED:
            self._nav_status = WaypointManagerState.STATUS_SUCCEEDED
            goal_handle.succeed()
            result.success = True
            result.message = f'Reached waypoint {wp_id}'
        elif nav_result.status == GoalStatus.STATUS_CANCELED:
            self._nav_status = WaypointManagerState.STATUS_CANCELED
            goal_handle.canceled()
            result.success = False
            result.message = 'Navigation canceled'
        else:
            self._nav_status = WaypointManagerState.STATUS_FAILED
            self.get_logger().warn(
                f'Navigation to waypoint {wp_id} failed (status={nav_result.status})'
            )
            goal_handle.abort()
            result.success = False
            result.message = f'Navigation failed (status={nav_result.status})'

        self._active_waypoint_id = 0
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
