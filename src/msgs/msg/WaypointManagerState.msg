uint8 STATUS_IDLE = 0
uint8 STATUS_NAVIGATING = 1
uint8 STATUS_SUCCEEDED = 2
uint8 STATUS_CANCELED = 3
uint8 STATUS_FAILED = 4

uint32 active_waypoint_id
uint8 nav_status
WaypointEntry[] waypoints
builtin_interfaces/Time last_update
