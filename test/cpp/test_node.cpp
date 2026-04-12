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
namespace
{

rclcpp::NodeOptions makeConnectionOptions(
  const char * url = "not-a-url", const char * room = "robot-room", const char * token = nullptr)
{
  rclcpp::NodeOptions options;
  if (url != nullptr) {
    options.append_parameter_override("livekit.url", url);
  }
  if (room != nullptr) {
    options.append_parameter_override("livekit.room", room);
  }
  if (token != nullptr) {
    options.append_parameter_override("livekit.token", token);
  }
  return options;
}

std::string captureNodeConstructionError(const rclcpp::NodeOptions & options)
{
  try {
    (void)std::make_shared<Node>(options);
    ADD_FAILURE() << "Expected Node construction to fail";
  } catch (const std::exception & error) {
    return error.what();
  }

  return {};
}

}  // namespace

class NodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    static test_support::ScopedRclcppInit init;
  }
};

TEST_F(NodeTest, ConstructsAndDestroysWithRequiredConnectionParameters)
{
  EXPECT_NO_THROW((void)std::make_shared<Node>(makeConnectionOptions("not-a-url", "robot-room", "test-token")));
}

// TODO(jon): Keep detailed per-parameter startup validation in test_runtime_config.cpp. Add a
// RoomConnection injection seam here if Node needs direct coverage for runtime-initialization failures.
TEST_F(NodeTest, InvalidStartupConfigurationFailsConstruction)
{
  const std::string missing_url_error =
    captureNodeConstructionError(makeConnectionOptions(nullptr, "robot-room", "test-token"));
  EXPECT_NE(missing_url_error.find("livekit.url"), std::string::npos) << missing_url_error;
  EXPECT_TRUE(
    missing_url_error.find("empty") != std::string::npos || missing_url_error.find("required") != std::string::npos)
    << missing_url_error;
}

}  // namespace livekit_ros2_bridge
