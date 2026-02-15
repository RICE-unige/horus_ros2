cmake_minimum_required(VERSION 3.8)
project(horus_backend)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(horus_interfaces REQUIRED)
find_package(nav2_msgs QUIET)
find_package(rclcpp_action QUIET)

set(HORUS_BACKEND_HAS_NAV2 FALSE)
if(nav2_msgs_FOUND AND rclcpp_action_FOUND)
  set(HORUS_BACKEND_HAS_NAV2 TRUE)
  message(STATUS "horus_backend: Nav2 adapter enabled")
else()
  message(STATUS "horus_backend: Nav2 adapter disabled (nav2_msgs or rclcpp_action not found)")
endif()

# Include directories
include_directories(include)

# Create library for backend components
add_library(${PROJECT_NAME}_lib
  src/backend_node.cpp
  src/tcp_server.cpp
  src/plugin_manager.cpp
)

if(HORUS_BACKEND_HAS_NAV2)
  target_sources(${PROJECT_NAME}_lib PRIVATE src/nav2_action_adapter.cpp)
  target_compile_definitions(${PROJECT_NAME}_lib PRIVATE HORUS_BACKEND_HAS_NAV2=1)
endif()

# Dependencies for library
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  horus_interfaces
)

if(HORUS_BACKEND_HAS_NAV2)
  ament_target_dependencies(${PROJECT_NAME}_lib
    nav2_msgs
    rclcpp_action
  )
endif()

# Main executable
add_executable(horus_backend_node
  src/main.cpp
)

# Link library to executable
target_link_libraries(horus_backend_node ${PROJECT_NAME}_lib)

# Dependencies for executable
ament_target_dependencies(horus_backend_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  horus_interfaces
)

if(HORUS_BACKEND_HAS_NAV2)
  target_compile_definitions(horus_backend_node PRIVATE HORUS_BACKEND_HAS_NAV2=1)
  ament_target_dependencies(horus_backend_node
    nav2_msgs
    rclcpp_action
  )
endif()

# Install executable
install(TARGETS horus_backend_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # ── disable the auto‐copyright test ──
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # TODO: Add tests later
  # find_package(ament_cmake_gtest REQUIRED)
  # ament_add_gtest(test_backend test/test_backend.cpp)
  # target_link_libraries(test_backend ${PROJECT_NAME}_lib)
endif()

ament_package()
