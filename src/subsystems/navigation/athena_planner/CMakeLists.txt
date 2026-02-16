cmake_minimum_required(VERSION 3.8)
project(athena_planner)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_util REQUIRED)  
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(bt_nodes SHARED
  plugins/nav_selector_node.cpp
  plugins/aruco_detected_node.cpp
  plugins/get_aruco_pose_node.cpp
  plugins/spiral_coverage_action_node.cpp
)

target_compile_definitions(bt_nodes PRIVATE BT_PLUGIN_EXPORT)

target_include_directories(bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/plugins>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(bt_nodes
  rclcpp
  std_msgs
  behaviortree_cpp_v3
  nav2_util  
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS bt_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

ament_export_libraries(bt_nodes)
ament_export_dependencies(rclcpp std_msgs behaviortree_cpp_v3 nav2_util nav_msgs tf2 tf2_geometry_msgs)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
