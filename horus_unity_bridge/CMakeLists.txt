cmake_minimum_required(VERSION 3.8)
project(horus_unity_bridge VERSION 0.1.0)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3 -march=native)
endif()

# C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(PkgConfig QUIET)

# Find nlohmann_json (header-only library)
find_package(nlohmann_json 3.2.0 QUIET)
if(NOT nlohmann_json_FOUND)
  # If not found via find_package, try to use system install
  find_path(NLOHMANN_JSON_INCLUDE_DIR nlohmann/json.hpp
    HINTS /usr/include /usr/local/include)
  if(NLOHMANN_JSON_INCLUDE_DIR)
    message(STATUS "Found nlohmann/json: ${NLOHMANN_JSON_INCLUDE_DIR}")
  else()
    message(FATAL_ERROR "nlohmann_json not found. Install with: sudo apt-get install nlohmann-json3-dev")
  endif()
endif()

option(ENABLE_WEBRTC "Enable WebRTC camera streaming support" ON)
if(ENABLE_WEBRTC)
  if(NOT PkgConfig_FOUND)
    message(WARNING "PkgConfig not found. Disabling WebRTC support.")
    set(ENABLE_WEBRTC OFF)
  else()
    pkg_check_modules(GST QUIET gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0)
    if(NOT GST_FOUND)
      message(WARNING "GStreamer development packages not found. Disabling WebRTC support.")
      set(ENABLE_WEBRTC OFF)
    else()
      find_package(OpenCV QUIET COMPONENTS core imgcodecs imgproc)
      if(NOT OpenCV_FOUND)
        message(WARNING "OpenCV not found. Disabling WebRTC support.")
        set(ENABLE_WEBRTC OFF)
      else()
        include(FetchContent)
        set(HORUS_BRIDGE_BUILD_TESTING ${BUILD_TESTING})
        set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
        FetchContent_Declare(
          libdatachannel
          GIT_REPOSITORY https://github.com/paullouisageneau/libdatachannel.git
          GIT_TAG v0.20.1
          GIT_SHALLOW TRUE
        )
        set(NO_EXAMPLES ON CACHE BOOL "" FORCE)
        set(NO_TESTS ON CACHE BOOL "" FORCE)
        set(TEST_APPS OFF CACHE BOOL "" FORCE)
        FetchContent_MakeAvailable(libdatachannel)
        set(BUILD_TESTING ${HORUS_BRIDGE_BUILD_TESTING} CACHE BOOL "" FORCE)
        if(BUILD_TESTING AND DEFINED libdatachannel_BINARY_DIR)
          file(WRITE
            "${libdatachannel_BINARY_DIR}/CTestTestfile.cmake"
            "# Vendored dependency tests are intentionally excluded from the horus_unity_bridge package CTest surface.\n")
        endif()
      endif()
    endif()
  endif()
endif()

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

if(ENABLE_WEBRTC)
  include_directories(${GST_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
  add_compile_definitions(ENABLE_WEBRTC)
endif()

# Source files
set(BRIDGE_CORE_SOURCES
  src/unity_bridge_node.cpp
  src/protocol_handler.cpp
  src/connection_manager.cpp
  src/message_router.cpp
  src/control_lease_manager.cpp
  src/topic_manager.cpp
  src/service_manager.cpp
)

if(ENABLE_WEBRTC)
  list(APPEND BRIDGE_CORE_SOURCES src/webrtc_manager.cpp)
endif()

# Unity bridge core library
add_library(horus_unity_bridge_core ${BRIDGE_CORE_SOURCES})

ament_target_dependencies(horus_unity_bridge_core
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_msgs
  rosidl_default_runtime
  rosidl_typesupport_cpp
  rosidl_typesupport_introspection_cpp
  std_srvs
  example_interfaces
)

# Link pthread for threading support
target_link_libraries(horus_unity_bridge_core pthread)

if(ENABLE_WEBRTC)
  target_link_libraries(horus_unity_bridge_core
    LibDataChannel::LibDataChannel
    ${GST_LIBRARIES}
    ${OpenCV_LIBRARIES}
  )
endif()

if(nlohmann_json_FOUND)
  target_link_libraries(horus_unity_bridge_core nlohmann_json::nlohmann_json)
endif()

target_include_directories(horus_unity_bridge_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Unity bridge executable
add_executable(horus_unity_bridge_node src/main.cpp)
target_link_libraries(horus_unity_bridge_node horus_unity_bridge_core)

# Install targets
install(TARGETS horus_unity_bridge_node horus_unity_bridge_core
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(horus_unity_bridge_tests
    test/test_unity_bridge.cpp
  )
  if(TARGET horus_unity_bridge_tests)
    target_link_libraries(horus_unity_bridge_tests horus_unity_bridge_core)
    ament_target_dependencies(horus_unity_bridge_tests
      rclcpp
      std_msgs
      sensor_msgs
      geometry_msgs
      nav_msgs
      tf2_msgs
      rosidl_default_runtime
      rosidl_typesupport_cpp
      rosidl_typesupport_introspection_cpp
      std_srvs
      example_interfaces
    )
  endif()
endif()

ament_package()
