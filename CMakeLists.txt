# Copyright (c) 2025-present Polymath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

cmake_minimum_required(VERSION 3.14)
project(livekit_ros2_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=switch)
  add_link_options("-Wl,--no-undefined")
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(nlohmann_json REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(PkgConfig REQUIRED)
find_library(SPDLOG_LIBRARY NAMES spdlog REQUIRED)

include(cmake/livekit_sdk.cmake)
livekit_ros2_bridge_configure_livekit_sdk()

generate_parameter_library(livekit_ros2_bridge_parameters
  parameters.yaml
)

set(livekit_ros2_bridge_public_include_dir "${CMAKE_CURRENT_SOURCE_DIR}/include")
set(livekit_ros2_bridge_private_include_dir "${CMAKE_CURRENT_SOURCE_DIR}/src")

pkg_search_module(gstreamer REQUIRED IMPORTED_TARGET gstreamer-1.0)
pkg_search_module(gstreamer-app REQUIRED IMPORTED_TARGET gstreamer-app-1.0)
pkg_search_module(gstreamer-video REQUIRED IMPORTED_TARGET gstreamer-video-1.0)

add_library(livekit_ros2_bridge_ros_interfaces STATIC
  src/ros_interfaces/definition_lookup.cpp
  src/ros_interfaces/message_type_support.cpp
  src/ros_interfaces/service_type_support.cpp
)
set_target_properties(livekit_ros2_bridge_ros_interfaces PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(livekit_ros2_bridge_ros_interfaces PUBLIC
  "${livekit_ros2_bridge_private_include_dir}"
)
target_link_libraries(livekit_ros2_bridge_ros_interfaces PUBLIC
  rclcpp::rclcpp
)
target_link_libraries(livekit_ros2_bridge_ros_interfaces PRIVATE
  ament_index_cpp::ament_index_cpp
)

add_library(livekit_ros2_bridge_video STATIC
  src/video/gstreamer_pipeline.cpp
  src/video/gstreamer_stream.cpp
  src/video/ros_stream.cpp
  src/video/pipeline_failure_handler.cpp
  src/video/stream_config.cpp
  src/video/stream_spec.cpp
  src/video/track_publisher.cpp
)
set_target_properties(livekit_ros2_bridge_video PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(livekit_ros2_bridge_video PUBLIC
  "${livekit_ros2_bridge_private_include_dir}"
)
target_link_libraries(livekit_ros2_bridge_video PUBLIC
  magic_enum::magic_enum
  rclcpp::rclcpp
  LiveKit::livekit
  livekit_ros2_bridge_parameters
  ${sensor_msgs_TARGETS}
)
target_link_libraries(livekit_ros2_bridge_video PRIVATE
  PkgConfig::gstreamer
  PkgConfig::gstreamer-app
  PkgConfig::gstreamer-video
)

add_library(${PROJECT_NAME} SHARED
  src/access_policy.cpp
  src/connection_watchdog.cpp
  src/data_track_publisher.cpp
  src/node.cpp
  src/protocol/cdr.cpp
  src/protocol/detail/base64.cpp
  src/protocol/interfaces_json.cpp
  src/protocol/resources_json.cpp
  src/protocol/services_json.cpp
  src/protocol/subscriptions_json.cpp
  src/protocol/topic_publish_json.cpp
  src/room_connection.cpp
  src/ros_executor_queue.cpp
  src/ros_service_caller.cpp
  src/ros_topic_publisher.cpp
  src/rpc_router.cpp
  src/runtime_config.cpp
  src/runtime.cpp
  src/subscription_lease_manager.cpp
  src/subscription_qos.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE
  "${livekit_ros2_bridge_private_include_dir}"
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  magic_enum::magic_enum
  rclcpp::rclcpp
  rclcpp_components::component
)
target_link_libraries(${PROJECT_NAME} PRIVATE
  # LiveKit 0.3.4 keeps spdlog private in CMake but still exports spdlog
  # symbols from liblivekit.so. Ensure ROS binds its logging plugin to the
  # system spdlog before liblivekit enters the process.
  -Wl,--no-as-needed
  "${SPDLOG_LIBRARY}"
  -Wl,--as-needed
  livekit_ros2_bridge_parameters
  livekit_ros2_bridge_ros_interfaces
  livekit_ros2_bridge_video
  LiveKit::livekit
  nlohmann_json::nlohmann_json
  OpenSSL::Crypto
  rmw::rmw
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "livekit_ros2_bridge::Node"
  EXECUTABLE ${PROJECT_NAME}_node
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${livekit_ros2_bridge_public_include_dir}>"
  "$<INSTALL_INTERFACE:include>"
)
install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME})

if(BUILD_TESTING)
  ament_auto_find_test_dependencies()

  function(livekit_ros2_bridge_add_test test_name test_source)
    ament_add_gtest(${test_name} ${test_source})
    target_include_directories(${test_name} PRIVATE
      "${livekit_ros2_bridge_private_include_dir}"
    )
    target_link_libraries(${test_name}
      ${PROJECT_NAME}
      -Wl,--no-as-needed
      "${SPDLOG_LIBRARY}"
      -Wl,--as-needed
      LiveKit::livekit
      rclcpp::rclcpp
      livekit_ros2_bridge_parameters
      ${sensor_msgs_TARGETS}
      ${ARGN}
    )
  endfunction()

  livekit_ros2_bridge_add_test(test_access_policy test/cpp/test_access_policy.cpp)
  livekit_ros2_bridge_add_test(test_base64 test/cpp/test_base64.cpp)
  livekit_ros2_bridge_add_test(test_callback_gate test/cpp/test_callback_gate.cpp)
  livekit_ros2_bridge_add_test(test_connection_watchdog test/cpp/test_connection_watchdog.cpp)
  livekit_ros2_bridge_add_test(test_data_track_publisher test/cpp/test_data_track_publisher.cpp)
  livekit_ros2_bridge_add_test(
    test_subscription_lease_manager
    test/cpp/test_subscription_lease_manager.cpp
    ${test_msgs_TARGETS}
  )
  livekit_ros2_bridge_add_test(test_ros_interface_definition_lookup test/cpp/test_ros_interface_definition_lookup.cpp)
  livekit_ros2_bridge_add_test(
    test_payload_helpers
    test/cpp/test_payload_helpers.cpp
    nlohmann_json::nlohmann_json
  )
  livekit_ros2_bridge_add_test(test_event_throttle test/cpp/test_event_throttle.cpp)
  livekit_ros2_bridge_add_test(
    test_gstreamer_resources
    test/cpp/test_gstreamer_resources.cpp
    PkgConfig::gstreamer
    PkgConfig::gstreamer-video
  )
  livekit_ros2_bridge_add_test(
    test_interface_payloads
    test/cpp/test_interface_payloads.cpp
    nlohmann_json::nlohmann_json
    ${std_msgs_TARGETS}
  )
  livekit_ros2_bridge_add_test(test_ros_graph_lookup test/cpp/test_ros_graph_lookup.cpp)
  livekit_ros2_bridge_add_test(
    test_ros_service_type_support
    test/cpp/test_ros_service_type_support.cpp
    ${std_srvs_TARGETS}
  )
  livekit_ros2_bridge_add_test(test_log_event test/cpp/test_log_event.cpp)
  livekit_ros2_bridge_add_test(test_node test/cpp/test_node.cpp ${std_srvs_TARGETS})
  livekit_ros2_bridge_add_test(
    test_resource_list_payloads
    test/cpp/test_resource_list_payloads.cpp
    nlohmann_json::nlohmann_json
  )
  livekit_ros2_bridge_add_test(test_ros_executor_queue test/cpp/test_ros_executor_queue.cpp)
  livekit_ros2_bridge_add_test(test_ros_resource_name_utils test/cpp/test_ros_resource_name_utils.cpp)
  livekit_ros2_bridge_add_test(test_ros_service_caller test/cpp/test_ros_service_caller.cpp ${std_srvs_TARGETS})
  livekit_ros2_bridge_add_test(test_rpc_router test/cpp/test_rpc_router.cpp ${std_srvs_TARGETS})
  livekit_ros2_bridge_add_test(test_runtime test/cpp/test_runtime.cpp ${std_srvs_TARGETS})
  livekit_ros2_bridge_add_test(test_runtime_config test/cpp/test_runtime_config.cpp)
  livekit_ros2_bridge_add_test(test_scope_exit test/cpp/test_scope_exit.cpp)
  livekit_ros2_bridge_add_test(
    test_service_call_payloads
    test/cpp/test_service_call_payloads.cpp
    nlohmann_json::nlohmann_json
  )
  livekit_ros2_bridge_add_test(
    test_subscription_payloads
    test/cpp/test_subscription_payloads.cpp
    nlohmann_json::nlohmann_json
  )
  livekit_ros2_bridge_add_test(test_subscription_qos test/cpp/test_subscription_qos.cpp)
  livekit_ros2_bridge_add_test(
    test_topic_publish_request
    test/cpp/test_topic_publish_request.cpp
    nlohmann_json::nlohmann_json
  )
  livekit_ros2_bridge_add_test(
    test_ros_topic_publisher
    test/cpp/test_ros_topic_publisher.cpp
    ${std_msgs_TARGETS}
    ${test_msgs_TARGETS}
  )
  livekit_ros2_bridge_add_test(test_video_stream_spec test/cpp/test_video_stream_spec.cpp)
  livekit_ros2_bridge_add_test(
    test_video_streams
    test/cpp/test_video_streams.cpp
    PkgConfig::gstreamer
    PkgConfig::gstreamer-app
    PkgConfig::gstreamer-video
  )
  livekit_ros2_bridge_add_test(test_video_track_publisher test/cpp/test_video_track_publisher.cpp)
endif()

ament_package()
