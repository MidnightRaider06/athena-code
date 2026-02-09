cmake_minimum_required(VERSION 3.8)
project(killswitch_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  umdloop_can_library
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(ament_cmake_auto REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

ament_auto_find_build_dependencies()

include_directories(include)

add_library(
  killswitch_ros2_control
  SHARED
  src/killswitch_hardware_interface.cpp
)

target_compile_features(killswitch_ros2_control PUBLIC cxx_std_20)
target_include_directories(killswitch_ros2_control PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/killswitch_ros2_control>
)

# Link CAN library
target_link_libraries(killswitch_ros2_control PUBLIC umdloop_can_library::umdloop_can_library)

ament_target_dependencies(
  killswitch_ros2_control PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface killswitch_hardware_interface.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/killswitch_ros2_control
)

install(TARGETS killswitch_ros2_control
  EXPORT export_killswitch_ros2_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_killswitch_ros2_control)
ament_export_include_directories(include)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
