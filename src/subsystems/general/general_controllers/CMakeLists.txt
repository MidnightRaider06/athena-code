cmake_minimum_required(VERSION 3.8)
project(general_controllers)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  diagnostic_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(motor_status_broadcaster_parameters
  config/motor_status_broadcaster.yaml
)

add_library(
  general_controllers
  SHARED
  src/motor_status_broadcaster.cpp
)
target_include_directories(general_controllers PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(general_controllers motor_status_broadcaster_parameters)
ament_target_dependencies(general_controllers ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(general_controllers PRIVATE "GENERAL_CONTROLLERS_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface general_controllers.xml)

install(
  TARGETS
  general_controllers
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_export_libraries(
  general_controllers
)

ament_package()
