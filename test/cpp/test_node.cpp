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
#include <vector>

#include "fake_livekit_session.hpp"
#include "gtest/gtest.h"
#include "livekit_ros2_bridge/node.hpp"
#include "livekit_ros2_bridge/protocol.hpp"
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

TEST_F(NodeTest, RegistersRpcPlaceholdersOnConnect)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("livekit.url", "ws://test:7880");
  options.append_parameter_override("livekit.token", "test_token");

  auto session = std::make_unique<FakeLiveKitSession>();
  auto state = session->state;
  const auto node = std::make_shared<Node>(options, std::move(session));
  const std::vector<std::string> expected_methods{
    protocol::kRpcTopicSubscribe, protocol::kRpcTopicUnsubscribe, protocol::kRpcServiceCall};

  ASSERT_NE(node, nullptr);
  EXPECT_EQ(state->registered_methods, expected_methods);
}

TEST_F(NodeTest, UnregistersRpcMethodsBeforeDisconnect)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("livekit.url", "ws://test:7880");
  options.append_parameter_override("livekit.token", "test_token");

  auto session = std::make_unique<FakeLiveKitSession>();
  auto state = session->state;
  {
    const auto node = std::make_shared<Node>(options, std::move(session));
    ASSERT_NE(node, nullptr);
  }

  ASSERT_EQ(state->events.size(), 4U);
  EXPECT_EQ(state->events[0], "unregister:" + std::string(protocol::kRpcTopicSubscribe));
  EXPECT_EQ(state->events[1], "unregister:" + std::string(protocol::kRpcTopicUnsubscribe));
  EXPECT_EQ(state->events[2], "unregister:" + std::string(protocol::kRpcServiceCall));
  EXPECT_EQ(state->events[3], "disconnect");
  EXPECT_EQ(
    state->unregistered_methods,
    (std::vector<std::string>{
      protocol::kRpcTopicSubscribe, protocol::kRpcTopicUnsubscribe, protocol::kRpcServiceCall}));
}

TEST_F(NodeTest, SubscribeRpcReturnsForbiddenWhenTopicIsDenied)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("livekit.url", "ws://test:7880");
  options.append_parameter_override("livekit.token", "test_token");
  options.append_parameter_override("access.rules.subscribe.allow", std::vector<std::string>{"/allowed"});

  auto session = std::make_unique<FakeLiveKitSession>();
  auto state = session->state;
  const auto node = std::make_shared<Node>(options, std::move(session));
  ASSERT_NE(node, nullptr);

  const auto handler_it = state->handlers.find(protocol::kRpcTopicSubscribe);
  ASSERT_NE(handler_it, state->handlers.end());

  try {
    handler_it->second(
      RpcInvocation{
        "request-1",
        "participant-1",
        R"({"topic":"/blocked","preferred_interval_ms":100})",
        0.0,
      });
    FAIL() << "Expected BridgeRpcError";
  } catch (const BridgeRpcError & exc) {
    EXPECT_EQ(exc.code(), protocol::kRpcErrorForbidden);
    EXPECT_STREQ(exc.what(), "ROS topic '/blocked' not permitted.");
  }
}

}  // namespace livekit_ros2_bridge
