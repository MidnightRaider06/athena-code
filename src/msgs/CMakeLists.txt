cmake_minimum_required(VERSION 3.8)
project(msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CANA.msg"
  "msg/CANB.msg"
  "msg/Currentdraw.msg"
  "msg/JointStatus.msg"
  "msg/SystemInfo.msg"
  "msg/Heading.msg"
  "msg/WaypointEntry.msg"
  "msg/WaypointManagerState.msg"
  "srv/SetController.srv"
  "srv/AddWaypoint.srv"
  "srv/ClearWaypoints.srv"
  "msg/LedStatus.msg"
  "srv/StatusReq.srv"
  "srv/MaintenanceReq.srv"
  "action/NavigateToGPS.action"
  "action/NavigateToWaypoint.action"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs builtin_interfaces action_msgs
)

ament_package()
