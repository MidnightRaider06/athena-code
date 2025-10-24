cmake_minimum_required(VERSION 3.8)
project(umdloop_can_library VERSION 1.0.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# -------------------- Dependencies --------------------
include(FetchContent)

# Fetch SerialCAN from GitHub
FetchContent_Declare(
        SerialCAN
        GIT_REPOSITORY https://github.com/mac-can/SerialCAN.git
        GIT_TAG main
        GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(SerialCAN)

# -------------------- Build SerialCAN --------------------
add_custom_target(umdloop_can_library_build_serialcan
        # First generate Version.h and build_no.h
        COMMAND bash ./build_no.sh
        # Then build the libraries with proper CFLAGS
        COMMAND ${CMAKE_COMMAND} -E env "CFLAGS=-fPIC" "CXXFLAGS=-fPIC" 
                ${CMAKE_MAKE_PROGRAM} -C Libraries/SerialCAN all
        COMMAND ${CMAKE_COMMAND} -E env "CFLAGS=-fPIC" "CXXFLAGS=-fPIC"
                ${CMAKE_MAKE_PROGRAM} -C Libraries/CANAPI all
        WORKING_DIRECTORY ${serialcan_SOURCE_DIR}
        COMMENT "Building SerialCAN libraries with -fPIC"
)

# -------------------- SerialCAN static libraries --------------------
set(SERIALCAN_LIB ${serialcan_SOURCE_DIR}/Libraries/SerialCAN/libserialcan.a)
set(UVCANSLC_LIB ${serialcan_SOURCE_DIR}/Libraries/CANAPI/libuvcanslc.a)

# We'll link the static libraries directly instead of using imported targets

# Create interface library to handle SerialCAN includes
add_library(SerialCAN_Headers INTERFACE)
target_include_directories(SerialCAN_Headers INTERFACE
        $<BUILD_INTERFACE:${serialcan_SOURCE_DIR}/Includes>
)
add_dependencies(SerialCAN_Headers umdloop_can_library_build_serialcan)

# -------------------- Library sources --------------------
set(UMDLOOP_CAN_LIBRARY_SOURCES
        src/SerialCanBus.cpp
)

set(UMDLOOP_CAN_LIBRARY_HEADERS
        include/umdloop_can_library/CanFrame.hpp
        include/umdloop_can_library/ICanBus.hpp
        include/umdloop_can_library/SerialCanBus.hpp
)

# Add SocketCAN support only on Linux
if(UNIX AND NOT APPLE)
    list(APPEND UMDLOOP_CAN_LIBRARY_SOURCES src/SocketCanBus.cpp)
    list(APPEND UMDLOOP_CAN_LIBRARY_HEADERS include/umdloop_can_library/SocketCanBus.hpp)
    message(STATUS "SocketCAN support enabled")
else()
    message(STATUS "SocketCAN support disabled (Linux only)")
endif()

# -------------------- Build umdloop_can_library --------------------
add_library(umdloop_can_library STATIC ${UMDLOOP_CAN_LIBRARY_SOURCES})
add_library(umdloop_can_library::umdloop_can_library ALIAS umdloop_can_library)

# Enable position independent code for static library
set_target_properties(umdloop_can_library PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(umdloop_can_library
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Link SerialCAN static libraries directly
target_link_libraries(umdloop_can_library
        PRIVATE ${SERIALCAN_LIB} ${UVCANSLC_LIB}
        PUBLIC SerialCAN_Headers
)

# Ensure SerialCAN builds first
add_dependencies(umdloop_can_library umdloop_can_library_build_serialcan)

set_target_properties(umdloop_can_library PROPERTIES
        VERSION ${PROJECT_VERSION}
        SOVERSION 1
)

# -------------------- Installation --------------------
include(GNUInstallDirs)

# Library and headers
install(TARGETS umdloop_can_library SerialCAN_Headers
        EXPORT umdloop_can_libraryTargets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Optional: install SerialCAN static libraries for reuse
install(FILES
        ${SERIALCAN_LIB}
        ${UVCANSLC_LIB}
        DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# Headers
install(DIRECTORY include/umdloop_can_library
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY ${serialcan_SOURCE_DIR}/Includes/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
        FILES_MATCHING PATTERN "*.h"
)

# Export CMake targets
install(EXPORT umdloop_can_libraryTargets
        FILE umdloop_can_libraryTargets.cmake
        NAMESPACE umdloop_can_library::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/umdloop_can_library
)

# Export include directories for ament
ament_export_include_directories(include)
ament_export_targets(umdloop_can_libraryTargets)

# Generate config files for find_package
include(CMakePackageConfigHelpers)

configure_package_config_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/umdloop_can_libraryConfig.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/umdloop_can_libraryConfig.cmake"
        INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/umdloop_can_library
)

write_basic_package_version_file(
        "${CMAKE_CURRENT_BINARY_DIR}/umdloop_can_libraryConfigVersion.cmake"
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY AnyNewerVersion
)

install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/umdloop_can_libraryConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/umdloop_can_libraryConfigVersion.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/umdloop_can_library
)

ament_package()
