cmake_minimum_required(VERSION 3.8)
project(arm_controllers)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(ros2_control_test_assets REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Add arm_controllers library related compile commands
generate_parameter_library(manual_arm_cylindrical_controller_parameters
  config/manual_arm_cylindrical_controller.yaml
  include/athena_arm_controllers/validate_athena_arm_controller_parameters.hpp
)

generate_parameter_library(manual_2dof_wrist_joint_by_joint_controller_parameters
  config/manual_2dof_wrist_joint_by_joint_controller.yaml
  include/athena_arm_controllers/validate_athena_arm_controller_parameters.hpp
)

generate_parameter_library(manual_3dof_wrist_joint_by_joint_controller_parameters
  config/manual_3dof_wrist_joint_by_joint_controller.yaml
  include/athena_arm_controllers/validate_athena_arm_controller_parameters.hpp
)

generate_parameter_library(manual_arm_joint_by_joint_controller_parameters
  config/manual_arm_joint_by_joint_controller.yaml
  include/athena_arm_controllers/validate_athena_arm_controller_parameters.hpp
)

generate_parameter_library(manual_end_effector_gripper_claw_controller_parameters
  config/manual_end_effector_gripper_claw_controller.yaml
  include/athena_arm_controllers/validate_athena_arm_controller_parameters.hpp
)

generate_parameter_library(manual_3dof_wrist_orientation_controller_parameters
  config/manual_3dof_wrist_orientation_controller.yaml
  include/athena_arm_controllers/validate_athena_arm_controller_parameters.hpp
)

add_library(
  arm_controllers
  SHARED
  src/manual_arm_cylindrical_controller.cpp
  src/manual_2dof_wrist_joint_by_joint_controller.cpp
  src/manual_3dof_wrist_joint_by_joint_controller.cpp
  src/manual_arm_joint_by_joint_controller.cpp
  src/manual_end_effector_gripper_claw_controller.cpp
  src/manual_3dof_wrist_orientation_controller.cpp
  src/velocity_calculations.cpp
)
target_include_directories(arm_controllers PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(arm_controllers manual_arm_cylindrical_controller_parameters)
target_link_libraries(arm_controllers manual_2dof_wrist_joint_by_joint_controller_parameters)
target_link_libraries(arm_controllers manual_3dof_wrist_joint_by_joint_controller_parameters)
target_link_libraries(arm_controllers manual_arm_joint_by_joint_controller_parameters)
target_link_libraries(arm_controllers manual_end_effector_gripper_claw_controller_parameters)
target_link_libraries(arm_controllers manual_3dof_wrist_orientation_controller_parameters)
ament_target_dependencies(arm_controllers ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(arm_controllers PRIVATE "manual_arm_cylindrical_controller_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface athena_arm_controllers.xml)

install(
  TARGETS
  arm_controllers
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# if(BUILD_TESTING)

#   ament_add_gmock(test_load_manual_arm_cylindrical_controller test/test_load_manual_arm_cylindrical_controller.cpp)
#   target_include_directories(test_load_manual_arm_cylindrical_controller PRIVATE include)
#   ament_target_dependencies(
#     test_load_manual_arm_cylindrical_controller
#     controller_manager
#     hardware_interface
#     ros2_control_test_assets
#   )

#   add_rostest_with_parameters_gmock(test_manual_arm_cylindrical_controller test/test_manual_arm_cylindrical_controller.cpp ${CMAKE_CURRENT_SOURCE_DIR}/test/manual_arm_cylindrical_controller_params.yaml)
#   target_include_directories(test_manual_arm_cylindrical_controller PRIVATE include)
#   target_link_libraries(test_manual_arm_cylindrical_controller manual_arm_cylindrical_controller)
#   ament_target_dependencies(
#     test_manual_arm_cylindrical_controller
#     controller_interface
#     hardware_interface
#   )

#   add_rostest_with_parameters_gmock(test_manual_arm_cylindrical_controller_preceeding test/test_manual_arm_cylindrical_controller_preceeding.cpp ${CMAKE_CURRENT_SOURCE_DIR}/test/manual_arm_cylindrical_controller_preceeding_params.yaml)
#   target_include_directories(test_manual_arm_cylindrical_controller_preceeding PRIVATE include)
#   target_link_libraries(test_manual_arm_cylindrical_controller_preceeding manual_arm_cylindrical_controller)
#   ament_target_dependencies(
#     test_manual_arm_cylindrical_controller_preceeding
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
  arm_controllers
)

ament_package()
