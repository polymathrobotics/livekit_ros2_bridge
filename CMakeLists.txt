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

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_target_dependencies REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(rclcpp REQUIRED)

generate_parameter_module(parameters
  livekit_ros2_bridge/parameters.yaml
  livekit_ros2_bridge.core.validators
)

# Install Python modules and entry points
ament_python_install_package(${PROJECT_NAME}
  SCRIPTS_DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

add_library(${PROJECT_NAME}_node
  src/node.cpp
)
target_include_directories(${PROJECT_NAME}_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  target_compile_options(${PROJECT_NAME}_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(${PROJECT_NAME}
  src/main.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_node
)
ament_target_dependencies(${PROJECT_NAME} rclcpp)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_cpp_smoke test/cpp/test_cpp_smoke.cpp)
  target_link_libraries(test_cpp_smoke ${PROJECT_NAME}_node)
  ament_target_dependencies(test_cpp_smoke rclcpp)
  ament_add_pytest_test(pytest ${CMAKE_CURRENT_SOURCE_DIR}/test)
endif()

ament_package()
