cmake_minimum_required(VERSION 3.5)
project(athena_gps)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(MAVSDK REQUIRED)
add_executable(athena_gps src/PixhawkNode.cpp)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_target_dependencies(athena_gps
        rclcpp
        std_msgs
        sensor_msgs
)
target_link_libraries(athena_gps
        MAVSDK::mavsdk
)
install(TARGETS
        athena_gps
        DESTINATION lib/${PROJECT_NAME})

ament_package()