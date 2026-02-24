cmake_minimum_required(VERSION 3.8)
project(aruco_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Add executable
add_executable(aruco_node src/aruco_node.cpp)

# Include directories
target_include_directories(aruco_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Link dependencies
ament_target_dependencies(aruco_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  OpenCV
  tf2_ros
  tf2_geometry_msgs)

target_compile_features(aruco_node PUBLIC c_std_99 cxx_std_17)

# Install targets
install(TARGETS aruco_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
