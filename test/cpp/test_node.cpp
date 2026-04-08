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
#include <string>

#include "fake_room_session.hpp"
#include "gtest/gtest.h"
#include "livekit_ros2_bridge/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "room_session.hpp"

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

TEST_F(NodeTest, ConstructsWithInjectedSessionAndOwnsRuntimeLifecycle)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("livekit.url", "ws://test:7880");
  options.append_parameter_override("livekit.room", "robot-room");
  options.append_parameter_override("livekit.token", "test-token");

  auto session = std::make_unique<FakeRoomSession>();
  auto state = session->state;

  {
    const auto node = std::make_shared<Node>(options, std::move(session));

    ASSERT_NE(node, nullptr);
    EXPECT_STREQ(node->get_name(), "livekit_ros2_bridge");
    EXPECT_TRUE(state->started);
    EXPECT_FALSE(state->stopped);
  }

  EXPECT_TRUE(state->stopped);
}

}  // namespace livekit_ros2_bridge
