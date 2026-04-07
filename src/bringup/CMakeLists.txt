cmake_minimum_required(VERSION 3.8)
project(bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install directories only if they exist
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)
endif()

# Install controller_switcher script from bringup package if present
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/controller_switcher.py OR ${CMAKE_CURRENT_SOURCE_DIR}/scripts/cmd_vel_to_rmd.py)
  install(PROGRAMS
    scripts/controller_switcher.py
    scripts/cmd_vel_to_rmd.py
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

ament_package()
