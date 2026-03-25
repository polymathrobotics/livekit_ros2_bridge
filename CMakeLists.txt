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

# Fetch pre-built LiveKit C++ SDK (distro-specific for glibc compat).
# Update version and both hashes together.
set(LIVEKIT_SDK_VERSION "0.3.1")
set(LIVEKIT_SDK_SHA256_JAMMY "2c5909f6d9eb3bf86da46336713602a10e404f3d75b196ddefc3a596c64a2953")
set(LIVEKIT_SDK_SHA256_NOBLE "2f100aa891dbae7db58a64dec30268e917b2e3dba8ff43004a5d431e37c2ac34")

include(FetchContent)
if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()

if("$ENV{ROS_DISTRO}" STREQUAL "humble")
  set(_sdk_distro "jammy")
  set(_sdk_sha256 "${LIVEKIT_SDK_SHA256_JAMMY}")
else()
  set(_sdk_distro "noble")
  set(_sdk_sha256 "${LIVEKIT_SDK_SHA256_NOBLE}")
endif()

set(_sdk_base_url
    "https://github.com/jon-mcmillan/livekit-client-sdk-cpp/releases/download/v${LIVEKIT_SDK_VERSION}")
fetchcontent_declare(livekit_sdk
  URL "${_sdk_base_url}/livekit-sdk-linux-x64-${_sdk_distro}-${LIVEKIT_SDK_VERSION}.tar.gz"
  URL_HASH SHA256=${_sdk_sha256}
)
fetchcontent_populate(livekit_sdk)
list(APPEND CMAKE_PREFIX_PATH "${livekit_sdk_SOURCE_DIR}")
find_package(LiveKit REQUIRED)
file(GLOB _sdk_libs "${livekit_sdk_SOURCE_DIR}/lib/*.so*")
install(FILES ${_sdk_libs} DESTINATION lib)

generate_parameter_library(livekit_ros2_bridge_parameters
  src/parameters.yaml
)

generate_parameter_module(parameters
  livekit_ros2_bridge/parameters.yaml
  livekit_ros2_bridge.core.validators
)

# Install Python modules and entry points
ament_python_install_package(${PROJECT_NAME}
  SCRIPTS_DESTINATION lib/${PROJECT_NAME}
)

add_library(${PROJECT_NAME} SHARED
  src/node.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  livekit_ros2_bridge_parameters
  LiveKit::livekit
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "livekit_ros2_bridge::Node"
  EXECUTABLE ${PROJECT_NAME}_node
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME})

if(BUILD_TESTING)
  ament_auto_find_test_dependencies()

  ament_add_gtest(test_cpp_smoke test/cpp/test_cpp_smoke.cpp)
  target_link_libraries(test_cpp_smoke ${PROJECT_NAME})
  ament_add_pytest_test(pytest ${CMAKE_CURRENT_SOURCE_DIR}/test)
endif()

ament_package()
