cmake_minimum_required(VERSION 3.8)
project(simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)

# --- Install data directories so package:// URIs work ---
# Guard each one so CMake doesn't error if a folder is missing.
foreach(dir urdf meshes config launch worlds textures models)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${dir})
    install(DIRECTORY ${dir}/
      DESTINATION share/${PROJECT_NAME}/${dir}
    )
  endif()
endforeach()

# Install Python scripts
install(PROGRAMS
  scripts/ground_truth_tf_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp 
  std_msgs 
  xacro 
  robot_state_publisher 
  ros2_control 
  ros2_controllers
  ros_gz_sim
  ros_gz_bridge
)

# Tests (none right now, keep lint off unless you want it)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()