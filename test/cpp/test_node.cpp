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

#include <stdexcept>
#include <string>

#include "gtest/gtest.h"
#include "livekit_ros2_bridge/node.hpp"
#include "ros_test_support.hpp"

namespace livekit_ros2_bridge
{
class NodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    static test_support::ScopedRclcppInit init;
  }
};

TEST_F(NodeTest, ConstructsWithRequiredConnectionParameters)
{
  rclcpp::NodeOptions options;
  // This test only exercises presence of the required startup parameters. A non-empty placeholder
  // URL is enough because Node construction does not synchronously reject malformed values here.
  options.append_parameter_override("livekit.url", "not-a-url");
  options.append_parameter_override("livekit.room", "robot-room");
  options.append_parameter_override("livekit.token", "test-token");

  EXPECT_NO_THROW((void)std::make_shared<Node>(options));
}

TEST_F(NodeTest, FailsWhenUrlMissing)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("livekit.room", "robot-room");
  options.append_parameter_override("livekit.token", "test-token");

  std::string message;
  try {
    (void)std::make_shared<Node>(options);
    ADD_FAILURE() << "Expected Node construction to fail";
  } catch (const std::exception & error) {
    message = error.what();
  }

  EXPECT_NE(message.find("livekit.url"), std::string::npos) << message;
  EXPECT_TRUE(message.find("empty") != std::string::npos || message.find("required") != std::string::npos) << message;
}

}  // namespace livekit_ros2_bridge
