// Copyright (c) 2025-present Polymath Robotics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "livekit_ros2_bridge/node.hpp"

namespace livekit_ros2_bridge {

class CppSmokeTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(CppSmokeTest, ConstructsNode) {
  rclcpp::NodeOptions options;
  options.append_parameter_override("livekit.url", "ws://localhost:7880");
  options.append_parameter_override("livekit.token", "test_token");
  const auto node = std::make_shared<Node>(options);

  EXPECT_STREQ(node->get_name(), "livekit_ros2_bridge");
}

TEST_F(CppSmokeTest, ThrowsIfParametersEmpty) {
  // No overrides — both params default to "", failing not_empty<> validation.
  EXPECT_THROW(std::make_shared<Node>(rclcpp::NodeOptions()), std::exception);
}

}  // namespace livekit_ros2_bridge
