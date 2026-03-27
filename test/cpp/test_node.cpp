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
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "livekit_ros2_bridge/livekit_session.hpp"
#include "livekit_ros2_bridge/node.hpp"
#include "livekit_ros2_bridge/protocol.hpp"
#include "rclcpp/rclcpp.hpp"

namespace livekit_ros2_bridge
{

namespace
{

class FakeLiveKitSession final : public LiveKitSession
{
public:
  bool connect(const std::string &, const std::string &) override
  {
    return true;
  }

  bool registerRpcMethod(const std::string & method_name, RpcHandler handler) override
  {
    (void)handler;
    registered_methods.push_back(method_name);
    return true;
  }

  bool unregisterRpcMethod(const std::string & method_name) override
  {
    (void)method_name;
    return true;
  }

  void disconnect() override
  {}

  std::vector<std::string> registered_methods;
};

}  // namespace

TEST(NodeTest, RegistersRpcPlaceholdersOnConnect)
{
  rclcpp::init(0, nullptr);

  rclcpp::NodeOptions options;
  options.append_parameter_override("livekit.url", "ws://test:7880");
  options.append_parameter_override("livekit.token", "test_token");

  auto session = std::make_unique<FakeLiveKitSession>();
  auto & fake = *session;
  const auto node = std::make_shared<Node>(options, std::move(session));
  const std::vector<std::string> expected_methods{
    protocol::kRpcTopicSubscribe, protocol::kRpcTopicUnsubscribe, protocol::kRpcServiceCall};

  ASSERT_NE(node, nullptr);
  EXPECT_EQ(fake.registered_methods, expected_methods);

  rclcpp::shutdown();
}

}  // namespace livekit_ros2_bridge
