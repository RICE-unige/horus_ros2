cmake_minimum_required(VERSION 3.8)
project(horus_unity_bridge VERSION 0.1.0)

# C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(HORUS_UNITY_BRIDGE_COMPILE_OPTIONS)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  list(APPEND HORUS_UNITY_BRIDGE_COMPILE_OPTIONS
    -Wall
    -Wextra
    -Wpedantic
    $<$<CONFIG:Release>:-O3>
  )
endif()

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

option(ENABLE_WEBRTC "Enable WebRTC camera streaming support" OFF)
option(
  HORUS_ALLOW_LIBDATACHANNEL_FETCH
  "Allow CMake to fetch the pinned libdatachannel source tree when WebRTC is enabled and no system package is available"
  OFF)
set(HORUS_LIBDATACHANNEL_TAG "a16be55e2f0630112b7d1e2d9ad2e30d4f21756b" CACHE STRING "Pinned libdatachannel annotated tag object")
set(
  HORUS_LIBDATACHANNEL_COMMIT
  "7841d9f34cf9bd735958ae203a2536c14240c8a5"
  CACHE STRING
  "Expected libdatachannel commit resolved by HORUS_LIBDATACHANNEL_TAG")
set(HORUS_LIBDATACHANNEL_TARGET "")
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
        find_package(LibDataChannel CONFIG QUIET)
        if(TARGET LibDataChannel::LibDataChannelStatic)
          set(HORUS_LIBDATACHANNEL_TARGET LibDataChannel::LibDataChannelStatic)
        elseif(TARGET LibDataChannel::LibDataChannel)
          set(HORUS_LIBDATACHANNEL_TARGET LibDataChannel::LibDataChannel)
        elseif(HORUS_ALLOW_LIBDATACHANNEL_FETCH)
          include(FetchContent)
          if(NOT DEFINED BUILD_TESTING)
            set(BUILD_TESTING ON CACHE BOOL "Build package tests" FORCE)
          endif()
          set(HORUS_BRIDGE_BUILD_TESTING ${BUILD_TESTING})
          set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
          set(PLOG_INSTALL OFF CACHE BOOL "" FORCE)
          set(JSON_Install OFF CACHE BOOL "" FORCE)
          fetchcontent_declare(
            libdatachannel
            GIT_REPOSITORY https://github.com/paullouisageneau/libdatachannel.git
            GIT_TAG ${HORUS_LIBDATACHANNEL_TAG}
            GIT_SHALLOW TRUE
            GIT_SUBMODULES_RECURSE TRUE
          )
          set(NO_EXAMPLES ON CACHE BOOL "" FORCE)
          set(NO_TESTS ON CACHE BOOL "" FORCE)
          set(TEST_APPS OFF CACHE BOOL "" FORCE)
          fetchcontent_getproperties(libdatachannel)
          if(NOT libdatachannel_POPULATED)
            fetchcontent_populate(libdatachannel)
            execute_process(
              COMMAND git rev-parse HEAD
              WORKING_DIRECTORY ${libdatachannel_SOURCE_DIR}
              OUTPUT_VARIABLE HORUS_LIBDATACHANNEL_ACTUAL_COMMIT
              OUTPUT_STRIP_TRAILING_WHITESPACE
              RESULT_VARIABLE HORUS_LIBDATACHANNEL_COMMIT_RESULT)
            if(NOT HORUS_LIBDATACHANNEL_COMMIT_RESULT EQUAL 0 OR
                NOT HORUS_LIBDATACHANNEL_ACTUAL_COMMIT STREQUAL HORUS_LIBDATACHANNEL_COMMIT)
              message(FATAL_ERROR
                "Fetched libdatachannel commit ${HORUS_LIBDATACHANNEL_ACTUAL_COMMIT} "
                "does not match expected ${HORUS_LIBDATACHANNEL_COMMIT}")
            endif()
            set(HORUS_LIBDATACHANNEL_SUBMODULE_LOCKS
              "deps/json|9cca280a4d0ccf0c08f47a99aa71d1b0e52f8d03"
              "deps/libjuice|0b6f958baba55e1a4eb31ec2137f62b2e07382ae"
              "deps/libsrtp|a566a9cfcd619e8327784aa7cff4a1276dc1e895"
              "deps/plog|e21baecd4753f14da64ede979c5a19302618b752"
              "deps/usrsctp|ebb18adac6501bad4501b1f6dccb67a1c85cc299")
            foreach(lock_entry IN LISTS HORUS_LIBDATACHANNEL_SUBMODULE_LOCKS)
              string(REPLACE "|" ";" lock_parts "${lock_entry}")
              list(GET lock_parts 0 submodule_path)
              list(GET lock_parts 1 expected_sha)
              execute_process(
                COMMAND git rev-parse HEAD
                WORKING_DIRECTORY ${libdatachannel_SOURCE_DIR}/${submodule_path}
                OUTPUT_VARIABLE actual_sha
                OUTPUT_STRIP_TRAILING_WHITESPACE
                RESULT_VARIABLE submodule_result)
              if(NOT submodule_result EQUAL 0 OR NOT actual_sha STREQUAL expected_sha)
                message(FATAL_ERROR "Fetched libdatachannel submodule ${submodule_path} at ${actual_sha}, expected ${expected_sha}")
              endif()
            endforeach()
            # ament's symlink-install mode wraps install() globally. Keep the
            # vendored dependency build-only so third-party install rules do not
            # enter this ROS package's install script.
            function(install)
            endfunction()
            add_subdirectory(
              ${libdatachannel_SOURCE_DIR}
              ${libdatachannel_BINARY_DIR}
              EXCLUDE_FROM_ALL
            )
            if(AMENT_CMAKE_SYMLINK_INSTALL)
              include("${ament_cmake_core_DIR}/symlink_install/install.cmake")
            endif()
          endif()
          set(BUILD_TESTING ${HORUS_BRIDGE_BUILD_TESTING} CACHE BOOL "Build package tests" FORCE)
          if(BUILD_TESTING AND DEFINED libdatachannel_BINARY_DIR)
            file(WRITE
              "${libdatachannel_BINARY_DIR}/CTestTestfile.cmake"
              "# Vendored dependency tests are intentionally excluded from the horus_unity_bridge package CTest surface.\n")
          endif()
          set(HORUS_LIBDATACHANNEL_TARGET LibDataChannel::LibDataChannelStatic)
        else()
          message(FATAL_ERROR
            "ENABLE_WEBRTC=ON requires an installed LibDataChannel CMake package. "
            "To use the pinned network fallback, add -DHORUS_ALLOW_LIBDATACHANNEL_FETCH=ON.")
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
  src/horuslink_channel_table.cpp
  src/horuslink_control_messages.cpp
  src/horuslink_protocol.cpp
  src/horuslink_session.cpp
  src/protocol_handler.cpp
  src/bridge_metrics.cpp
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
target_compile_options(horus_unity_bridge_core PRIVATE ${HORUS_UNITY_BRIDGE_COMPILE_OPTIONS})

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
    ${HORUS_LIBDATACHANNEL_TARGET}
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
target_compile_options(horus_unity_bridge_node PRIVATE ${HORUS_UNITY_BRIDGE_COMPILE_OPTIONS})
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
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)

  ament_copyright()
  ament_cpplint(
    FILTERS
      -build/include_order
      -build/include_what_you_use
      -whitespace/comments
      -whitespace/line_length
  )
  ament_flake8()
  ament_lint_cmake()
  ament_uncrustify()

  ament_add_gtest(horus_unity_bridge_tests
    test/test_horuslink_channel_table.cpp
    test/test_horuslink_control_messages.cpp
    test/test_horuslink_protocol.cpp
    test/test_horuslink_session.cpp
    test/test_unity_bridge.cpp
  )
  if(TARGET horus_unity_bridge_tests)
    target_compile_options(horus_unity_bridge_tests PRIVATE ${HORUS_UNITY_BRIDGE_COMPILE_OPTIONS})
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
