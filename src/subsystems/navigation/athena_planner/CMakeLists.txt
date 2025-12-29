cmake_minimum_required(VERSION 3.8)
project(athena_planner)

# Warnings
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Buildtool dep only (no code targets here)
find_package(ament_cmake REQUIRED)

# ---- Install runtime assets ----
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# (Optional) install behavior trees if the folder exists
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees)
  install(DIRECTORY behavior_trees/
    DESTINATION share/${PROJECT_NAME}/behavior_trees
  )
endif()

# ---- Lint / tests (still fine even with no targets) ----
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Mark as found to avoid needing templates
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
