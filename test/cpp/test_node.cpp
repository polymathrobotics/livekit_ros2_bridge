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

#include <exception>
#include <memory>

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
  // This test only exercises presence of the required startup parameters. Use a syntactically
  // valid loopback URL so the asynchronous LiveKit connection path does not fail URL parsing
  // during teardown on distros where the SDK reports that failure from another thread.
  options.append_parameter_override("livekit.url", "ws://127.0.0.1:9");
  options.append_parameter_override("livekit.token", "test-token");

  EXPECT_NO_THROW((void)std::make_shared<Node>(options));
}

TEST_F(NodeTest, FailsWhenUrlMissing)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("livekit.token", "test-token");

  // generate_parameter_library owns the required-parameter validation details; this
  // node-boundary test only verifies that construction surfaces the failure.
  EXPECT_THROW((void)std::make_shared<Node>(options), std::exception);
}

}  // namespace livekit_ros2_bridge
