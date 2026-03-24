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
