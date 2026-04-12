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

#include "gtest/gtest.h"
#include "livekit_ros2_bridge/node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace livekit_ros2_bridge
{

class NodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(NodeTest, ConstructsWithDefaultRoomSession)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("livekit.url", "not-a-url");
  options.append_parameter_override("livekit.room", "robot-room");
  options.append_parameter_override("livekit.token", "test-token");

  const auto node = std::make_shared<Node>(options);

  ASSERT_NE(node, nullptr);
  EXPECT_STREQ(node->get_name(), "livekit_ros2_bridge");
}

}  // namespace livekit_ros2_bridge
