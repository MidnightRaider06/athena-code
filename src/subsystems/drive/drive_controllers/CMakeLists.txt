cmake_minimum_required(VERSION 3.8)
project(drive_controllers)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  controller_interface
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
  tf2_msgs
)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(umdloop_can_library REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(ros2_control_test_assets REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Add swerve_drive_controller library related compile commands
generate_parameter_library(swerve_drive_controller_parameters
  config/swerve_drive_controller.yaml
  include/athena_drive_controllers/validate_drive_controller_parameters.hpp
)

generate_parameter_library(double_ackermann_controller_parameters
  config/double_ackermann_controller.yaml
  include/athena_drive_controllers/validate_drive_controller_parameters.hpp
)

generate_parameter_library(crab_steering_controller_parameters
  config/crab_steering_controller.yaml
  include/athena_drive_controllers/validate_drive_controller_parameters.hpp
)

generate_parameter_library(front_ackermann_controller_parameters
  config/front_ackermann_controller.yaml
  include/athena_drive_controllers/validate_drive_controller_parameters.hpp
)

generate_parameter_library(rear_ackermann_controller_parameters
  config/rear_ackermann_controller.yaml
)

add_library(
  drive_controllers
  SHARED
  src/swerve_drive_controller.cpp
  src/double_ackermann_controller.cpp
  src/front_ackermann_controller.cpp
  src/rear_ackermann_controller.cpp
)
target_include_directories(drive_controllers PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(drive_controllers swerve_drive_controller_parameters double_ackermann_controller_parameters crab_steering_controller_parameters front_ackermann_controller_parameters rear_ackermann_controller_parameters umdloop_can_library::umdloop_can_library)
ament_target_dependencies(drive_controllers ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(drive_controllers PRIVATE "SWERVE_DRIVE_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface athena_drive_controllers.xml)

install(
  TARGETS
  drive_controllers
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# if(BUILD_TESTING)

#   ament_add_gmock(test_load_swerve_drive_controller test/test_load_swerve_drive_controller.cpp)
#   target_include_directories(test_load_swerve_drive_controller PRIVATE include)
#   ament_target_dependencies(
#     test_load_swerve_drive_controller
#     controller_manager
#     hardware_interface
#     ros2_control_test_assets
#   )

#   add_rostest_with_parameters_gmock(test_swerve_drive_controller test/test_swerve_drive_controller.cpp ${CMAKE_CURRENT_SOURCE_DIR}/test/swerve_drive_controller_params.yaml)
#   target_include_directories(test_swerve_drive_controller PRIVATE include)
#   target_link_libraries(test_swerve_drive_controller swerve_drive_controller)
#   ament_target_dependencies(
#     test_swerve_drive_controller
#     controller_interface
#     hardware_interface
#   )

#   add_rostest_with_parameters_gmock(test_swerve_drive_controller_preceeding test/test_swerve_drive_controller_preceeding.cpp ${CMAKE_CURRENT_SOURCE_DIR}/test/swerve_drive_controller_preceeding_params.yaml)
#   target_include_directories(test_swerve_drive_controller_preceeding PRIVATE include)
#   target_link_libraries(test_swerve_drive_controller_preceeding swerve_drive_controller)
#   ament_target_dependencies(
#     test_swerve_drive_controller_preceeding
#     controller_interface
#     hardware_interface
#   )
# endif()

ament_export_include_directories(
  include
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_export_libraries(
  drive_controllers
)

ament_package()
