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

add_library(${PROJECT_NAME} SHARED
  src/access_policy.cpp
  src/data_track_publisher.cpp
  src/control_packet_router.cpp
  src/ros_image_format_mapping.cpp
  src/interface_definition_lookup.cpp
  src/node.cpp
  src/payloads/cdr_payload.cpp
  src/payloads/interface_payloads.cpp
  src/payloads/resource_list_payloads.cpp
  src/payloads/service_call_payloads.cpp
  src/payloads/stream_control_payloads.cpp
  src/room_session.cpp
  src/ros_executor_queue.cpp
  src/ros_service_caller.cpp
  src/rpc_router.cpp
  src/runtime_config.cpp
  src/runtime.cpp
  src/subscription_qos.cpp
  src/subscription_heartbeat_processor.cpp
  src/subscription_registry.cpp
  src/topic_publish_command.cpp
  src/topic_publisher.cpp
  src/utils/base64.cpp
  src/video_config.cpp
  src/video_ingestor.cpp
  src/video_stream_manager.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE
  "${livekit_ros2_bridge_private_include_dir}"
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
)
target_link_libraries(${PROJECT_NAME} PRIVATE
  ${sensor_msgs_TARGETS}
  ament_index_cpp::ament_index_cpp
  livekit_ros2_bridge_parameters
  LiveKit::livekit
  nlohmann_json::nlohmann_json
  OpenSSL::Crypto
  PkgConfig::gstreamer
  PkgConfig::gstreamer-app
  PkgConfig::gstreamer-video
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

  ament_add_gtest(test_access_policy test/cpp/test_access_policy.cpp)
  target_link_libraries(test_access_policy ${PROJECT_NAME})

  ament_add_gtest(test_base64 test/cpp/test_base64.cpp)
  target_link_libraries(test_base64 ${PROJECT_NAME})

  ament_add_gtest(test_bounded_lru_cache test/cpp/test_bounded_lru_cache.cpp)
  target_link_libraries(test_bounded_lru_cache ${PROJECT_NAME})

  ament_add_gtest(test_data_track_publisher test/cpp/test_data_track_publisher.cpp)
  target_link_libraries(test_data_track_publisher ${PROJECT_NAME})

  ament_add_gtest(test_control_packet_router test/cpp/test_control_packet_router.cpp)
  target_link_libraries(test_control_packet_router ${PROJECT_NAME})

  ament_add_gtest(test_subscription_heartbeat_processor test/cpp/test_subscription_heartbeat_processor.cpp)
  target_link_libraries(test_subscription_heartbeat_processor ${PROJECT_NAME} ${sensor_msgs_TARGETS})

  ament_add_gtest(test_interface_definition_lookup test/cpp/test_interface_definition_lookup.cpp)
  target_link_libraries(test_interface_definition_lookup ${PROJECT_NAME})

  ament_add_gtest(test_event_throttle test/cpp/test_event_throttle.cpp)
  target_include_directories(test_event_throttle PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(test_event_throttle ${PROJECT_NAME})

  ament_add_gtest(test_gstreamer_raii test/cpp/test_gstreamer_raii.cpp)
  target_link_libraries(test_gstreamer_raii ${PROJECT_NAME} PkgConfig::gstreamer)

  ament_add_gtest(test_interface_payloads test/cpp/test_interface_payloads.cpp)
  target_link_libraries(test_interface_payloads ${PROJECT_NAME} nlohmann_json::nlohmann_json)

  ament_add_gtest(test_interface_type_utils test/cpp/test_interface_type_utils.cpp)
  target_link_libraries(test_interface_type_utils ${PROJECT_NAME})

  ament_add_gtest(test_log_event test/cpp/test_log_event.cpp)
  target_include_directories(test_log_event PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_link_libraries(test_log_event ${PROJECT_NAME})

  ament_add_gtest(test_node test/cpp/test_node.cpp)
  target_link_libraries(test_node ${PROJECT_NAME} ${std_srvs_TARGETS})

  ament_add_gtest(test_quiesce_guard test/cpp/test_quiesce_guard.cpp)
  target_link_libraries(test_quiesce_guard ${PROJECT_NAME})

  ament_add_gtest(test_reentrant_quiesce_guard test/cpp/test_reentrant_quiesce_guard.cpp)
  target_link_libraries(test_reentrant_quiesce_guard ${PROJECT_NAME})

  ament_add_gtest(test_resource_list_payloads test/cpp/test_resource_list_payloads.cpp)
  target_link_libraries(test_resource_list_payloads ${PROJECT_NAME} nlohmann_json::nlohmann_json)

  ament_add_gtest(test_ros_executor_queue test/cpp/test_ros_executor_queue.cpp)
  target_link_libraries(test_ros_executor_queue ${PROJECT_NAME})

  ament_add_gtest(test_ros_resource_name_utils test/cpp/test_ros_resource_name_utils.cpp)
  target_link_libraries(test_ros_resource_name_utils ${PROJECT_NAME})

  ament_add_gtest(test_ros_service_caller test/cpp/test_ros_service_caller.cpp)
  target_link_libraries(test_ros_service_caller ${PROJECT_NAME} ${std_srvs_TARGETS})

  ament_add_gtest(test_rpc_router test/cpp/test_rpc_router.cpp)
  target_link_libraries(test_rpc_router ${PROJECT_NAME} ${std_srvs_TARGETS} ${sensor_msgs_TARGETS})

  ament_add_gtest(test_runtime test/cpp/test_runtime.cpp)
  target_link_libraries(test_runtime ${PROJECT_NAME} ${std_srvs_TARGETS} ${sensor_msgs_TARGETS})

  ament_add_gtest(test_runtime_config test/cpp/test_runtime_config.cpp)
  target_link_libraries(test_runtime_config ${PROJECT_NAME})

  ament_add_gtest(test_scope_exit test/cpp/test_scope_exit.cpp)
  target_link_libraries(test_scope_exit ${PROJECT_NAME})

  ament_add_gtest(test_service_call_payloads test/cpp/test_service_call_payloads.cpp)
  target_link_libraries(test_service_call_payloads ${PROJECT_NAME} nlohmann_json::nlohmann_json)

  ament_add_gtest(test_stream_control_payloads test/cpp/test_stream_control_payloads.cpp)
  target_link_libraries(test_stream_control_payloads ${PROJECT_NAME} nlohmann_json::nlohmann_json)

  ament_add_gtest(test_subscription_qos test/cpp/test_subscription_qos.cpp)
  target_link_libraries(test_subscription_qos ${PROJECT_NAME})

  ament_add_gtest(test_subscription_registry test/cpp/test_subscription_registry.cpp)
  target_link_libraries(test_subscription_registry ${PROJECT_NAME} ${test_msgs_TARGETS} ${sensor_msgs_TARGETS})

  ament_add_gtest(test_topic_publish_command test/cpp/test_topic_publish_command.cpp)
  target_link_libraries(test_topic_publish_command ${PROJECT_NAME} nlohmann_json::nlohmann_json)

  ament_add_gtest(test_topic_publisher test/cpp/test_topic_publisher.cpp)
  target_link_libraries(test_topic_publisher ${PROJECT_NAME} ${std_msgs_TARGETS} ${test_msgs_TARGETS})

  ament_add_gtest(test_video_config test/cpp/test_video_config.cpp)
  target_link_libraries(test_video_config ${PROJECT_NAME})

  ament_add_gtest(test_video_stream_manager test/cpp/test_video_stream_manager.cpp)
  target_link_libraries(test_video_stream_manager ${PROJECT_NAME} ${sensor_msgs_TARGETS})

  set(livekit_ros2_bridge_test_targets
    test_access_policy
    test_base64
    test_bounded_lru_cache
    test_data_track_publisher
    test_control_packet_router
    test_event_throttle
    test_gstreamer_raii
    test_interface_definition_lookup
    test_interface_payloads
    test_interface_type_utils
    test_log_event
    test_node
    test_quiesce_guard
    test_reentrant_quiesce_guard
    test_resource_list_payloads
    test_ros_executor_queue
    test_ros_resource_name_utils
    test_ros_service_caller
    test_rpc_router
    test_runtime
    test_runtime_config
    test_scope_exit
    test_service_call_payloads
    test_stream_control_payloads
    test_subscription_qos
    test_subscription_heartbeat_processor
    test_subscription_registry
    test_topic_publish_command
    test_topic_publisher
    test_video_config
    test_video_stream_manager
  )
  foreach(test_target IN LISTS livekit_ros2_bridge_test_targets)
    target_include_directories(${test_target} PRIVATE
      "${livekit_ros2_bridge_private_include_dir}"
    )
    target_link_libraries(${test_target}
      rclcpp::rclcpp
      livekit_ros2_bridge_parameters
      ${sensor_msgs_TARGETS}
    )
  endforeach()
endif()

ament_package()
