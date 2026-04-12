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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "access_policy.hpp"
#include "fake_room_connection.hpp"
#include "gtest/gtest.h"
#include "nlohmann/json.hpp"
#include "payloads/cdr_payload.hpp"
#include "protocol.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "ros_executor_queue.hpp"
#include "ros_service_caller.hpp"
#include "rpc_router.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace livekit_ros2_bridge
{

namespace
{

constexpr auto kSpinPollInterval = std::chrono::milliseconds(5);
constexpr auto kSpinTimeout = std::chrono::seconds(5);

template <typename MessageT>
std::vector<std::uint8_t> serializeMessage(const MessageT & message)
{
  rclcpp::Serialization<MessageT> serialization;
  rclcpp::SerializedMessage serialized;
  serialization.serialize_message(&message, &serialized);
  const auto & rcl_msg = serialized.get_rcl_serialized_message();
  return std::vector<std::uint8_t>(rcl_msg.buffer, rcl_msg.buffer + rcl_msg.buffer_length);
}

template <typename MessageT>
MessageT deserializeMessage(const std::vector<std::uint8_t> & payload)
{
  rclcpp::SerializedMessage serialized(payload.size());
  auto & rcl_msg = serialized.get_rcl_serialized_message();
  std::copy(payload.begin(), payload.end(), rcl_msg.buffer);
  rcl_msg.buffer_length = payload.size();

  rclcpp::Serialization<MessageT> serialization;
  MessageT message;
  serialization.deserialize_message(&serialized, &message);
  return message;
}

bool spinUntil(
  rclcpp::executors::SingleThreadedExecutor & executor,
  const std::function<bool()> & predicate,
  std::chrono::milliseconds timeout = kSpinTimeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(kSpinPollInterval);
  }
  return predicate();
}

template <typename FutureT>
bool waitForFutureReady(
  rclcpp::executors::SingleThreadedExecutor & executor,
  FutureT & future,
  std::chrono::milliseconds timeout = kSpinTimeout)
{
  return spinUntil(
    executor,
    [&future]() { return future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready; },
    timeout);
}

void expectRpcHandlerError(
  const std::function<void()> & action,
  std::uint32_t expected_code,
  const std::optional<std::string> & expected_message = std::nullopt)
{
  try {
    action();
    FAIL() << "Expected RpcHandlerError";
  } catch (const RpcHandlerError & exc) {
    EXPECT_EQ(exc.code(), expected_code);
    if (expected_message.has_value()) {
      EXPECT_STREQ(exc.what(), expected_message->c_str());
    }
  }
}

std::string nextNodeName(const std::string & prefix)
{
  static std::atomic<int> counter{0};
  return prefix + "_" + std::to_string(counter.fetch_add(1));
}

std::string makeServiceCallRequestPayload(
  const std::string & service,
  const std::string & interface_type,
  const std::vector<std::uint8_t> & request_payload,
  std::optional<int> timeout_ms = std::nullopt)
{
  auto body = nlohmann::json{
    {"service", service},
    {"interface_type", interface_type},
    {"request", serializeCdrPayload(request_payload)},
  };
  if (timeout_ms.has_value()) {
    body["timeout_ms"] = *timeout_ms;
  }
  return body.dump();
}

AccessPolicy makeSubscribePolicy(std::vector<std::string> allow = {}, std::vector<std::string> deny = {})
{
  AccessPolicyConfig config;
  config.subscribe.allow = std::move(allow);
  config.subscribe.deny = std::move(deny);
  return AccessPolicy(config);
}

AccessPolicy makeServicePolicy(std::vector<std::string> allow = {}, std::vector<std::string> deny = {})
{
  AccessPolicyConfig config;
  config.service.allow = std::move(allow);
  config.service.deny = std::move(deny);
  return AccessPolicy(config);
}

class RpcRouterHarness
{
public:
  explicit RpcRouterHarness(const AccessPolicy & access_policy = AccessPolicy())
  : node(std::make_shared<rclcpp::Node>(nextNodeName("rpc_router_test_node")))
  , ros_executor_queue(*node)
  , ros_service_caller(*node)
  , rpc_router(*node, access_policy, ros_executor_queue, ros_service_caller)
  {
    rpc_router.registerRpcMethods(session);
  }

  ~RpcRouterHarness()
  {
    rpc_router.unregisterRpcMethods(session);
    ros_service_caller.shutdown();
    ros_executor_queue.shutdown();
  }

  std::optional<std::string> invokeRpc(const std::string & method_name, const RpcInvocation & invocation)
  {
    const auto handler_it = session.state->rpc_handlers.find(method_name);
    if (handler_it == session.state->rpc_handlers.end()) {
      throw std::runtime_error("RPC method not registered in harness: " + method_name);
    }
    return handler_it->second(invocation);
  }

  std::shared_ptr<rclcpp::Node> node;
  RosExecutorQueue ros_executor_queue;
  RosServiceCaller ros_service_caller;
  FakeRoomConnection session;
  RpcRouter rpc_router;
};

class RpcRouterTest : public ::testing::Test
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

TEST_F(RpcRouterTest, RegisteredRpcHandlersRequireCallerIdentityBeforeParsing)
{
  RpcRouterHarness harness;
  const std::vector<std::string> methods{
    protocol::kRpcServiceCall,
    protocol::kRpcInterfacesGet,
    protocol::kRpcServicesList,
    protocol::kRpcTopicsList,
  };

  for (const auto & method : methods) {
    expectRpcHandlerError(
      [&]() {
        harness.invokeRpc(
          method,
          RpcInvocation{
            "",
            R"({not-json})",
          });
      },
      protocol::kRpcErrorUnauthorized);
  }
}

TEST_F(RpcRouterTest, ServiceCallRpcMapsInvalidPayloadToInvalidRequest)
{
  RpcRouterHarness harness(makeServicePolicy({"*"}));

  expectRpcHandlerError(
    [&]() {
      harness.invokeRpc(
        protocol::kRpcServiceCall,
        RpcInvocation{
          "participant-1",
          R"({"service":"/rpc_router/set_bool"})",
        });
    },
    protocol::kRpcErrorInvalidRequest);
}

TEST_F(RpcRouterTest, ServiceCallRpcReturnsForbiddenWhenServiceIsDenied)
{
  RpcRouterHarness harness(makeServicePolicy({"/allowed_service"}));
  std_srvs::srv::SetBool::Request request_message;

  expectRpcHandlerError(
    [&]() {
      harness.invokeRpc(
        protocol::kRpcServiceCall,
        RpcInvocation{
          "participant-1",
          makeServiceCallRequestPayload("/denied_service", "std_srvs/srv/SetBool", serializeMessage(request_message)),
        });
    },
    protocol::kRpcErrorForbidden,
    "ROS service '/denied_service' not permitted.");
}

TEST_F(RpcRouterTest, InterfacesGetRpcMapsUnknownTypeToInternalError)
{
  RpcRouterHarness harness;

  expectRpcHandlerError(
    [&]() {
      harness.invokeRpc(
        protocol::kRpcInterfacesGet,
        RpcInvocation{
          "participant-1",
          R"({"interface_types":["nonexistent_pkg/msg/Foo"]})",
        });
    },
    protocol::kRpcErrorInternal);
}

TEST_F(RpcRouterTest, InterfacesGetRpcReturnsDefinitionForKnownType)
{
  RpcRouterHarness harness;

  const auto response = harness.invokeRpc(
    protocol::kRpcInterfacesGet,
    RpcInvocation{
      "participant-1",
      R"({"interface_types":["std_msgs/msg/String"]})",
    });

  ASSERT_TRUE(response.has_value());
  const auto body = nlohmann::json::parse(*response);
  ASSERT_EQ(body["interfaces"].size(), 1U);
  const auto & interface_definition = body["interfaces"][0];
  EXPECT_EQ(interface_definition["interface_type"].get<std::string>(), "std_msgs/msg/String");
  EXPECT_EQ(interface_definition["format"].get<std::string>(), "ros2msg");
  EXPECT_FALSE(interface_definition["definition"].get<std::string>().empty());
}

TEST_F(RpcRouterTest, ServiceCallRpcDispatchesAndReturnsResponse)
{
  RpcRouterHarness harness(makeServicePolicy({"*"}));
  auto server_node = std::make_shared<rclcpp::Node>(nextNodeName("rpc_router_service_server"));
  auto service = server_node->create_service<std_srvs::srv::SetBool>(
    "/rpc_router/set_bool",
    [](const std_srvs::srv::SetBool::Request::SharedPtr request, std_srvs::srv::SetBool::Response::SharedPtr response) {
      response->success = request->data;
      response->message = request->data ? "enabled" : "disabled";
    });
  (void)service;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node);
  executor.add_node(server_node);

  ASSERT_TRUE(spinUntil(
    executor, [&]() { return harness.node->get_service_names_and_types().count("/rpc_router/set_bool") > 0U; }));

  auto handler_future = std::async(std::launch::async, [&]() {
    std_srvs::srv::SetBool::Request request_message;
    request_message.data = true;
    return harness.invokeRpc(
      protocol::kRpcServiceCall,
      RpcInvocation{
        "participant-1",
        makeServiceCallRequestPayload(
          "/rpc_router/set_bool", "std_srvs/srv/SetBool", serializeMessage(request_message)),
      });
  });

  ASSERT_TRUE(waitForFutureReady(executor, handler_future));

  const auto rpc_response = handler_future.get();
  ASSERT_TRUE(rpc_response.has_value());

  const auto body = nlohmann::json::parse(*rpc_response);
  EXPECT_TRUE(body["ok"].get<bool>());
  EXPECT_EQ(body["service"]["name"].get<std::string>(), "/rpc_router/set_bool");
  EXPECT_EQ(body["service"]["interface_type"].get<std::string>(), "std_srvs/srv/SetBool");
  EXPECT_TRUE(body.contains("response"));

  const auto response_payload = parseCdrPayload(body, "response");
  const auto response_message = deserializeMessage<std_srvs::srv::SetBool::Response>(response_payload);
  EXPECT_TRUE(response_message.success);
  EXPECT_EQ(response_message.message, "enabled");
}

TEST_F(RpcRouterTest, ServiceCallRpcReturnsInternalErrorWhenServiceCallTimesOut)
{
  RpcRouterHarness harness(makeServicePolicy({"*"}));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node);

  auto handler_future = std::async(std::launch::async, [&]() {
    std_srvs::srv::SetBool::Request request_message;
    request_message.data = true;
    return harness.invokeRpc(
      protocol::kRpcServiceCall,
      RpcInvocation{
        "participant-1",
        makeServiceCallRequestPayload(
          "/no_such_service", "std_srvs/srv/SetBool", serializeMessage(request_message), 200),
      });
  });

  ASSERT_TRUE(waitForFutureReady(executor, handler_future));

  expectRpcHandlerError([&]() { handler_future.get(); }, protocol::kRpcErrorInternal, "Service call timed out.");
}

TEST_F(RpcRouterTest, ServiceListRpcFiltersAllowedServicesOnRosExecutorThread)
{
  RpcRouterHarness harness(makeServicePolicy({"/rpc_router/allowed_service"}));
  auto allowed_service = harness.node->create_service<std_srvs::srv::SetBool>(
    "/rpc_router/allowed_service",
    [](const std_srvs::srv::SetBool::Request::SharedPtr, std_srvs::srv::SetBool::Response::SharedPtr response) {
      response->success = true;
    });
  auto blocked_service = harness.node->create_service<std_srvs::srv::SetBool>(
    "/rpc_router/blocked_service",
    [](const std_srvs::srv::SetBool::Request::SharedPtr, std_srvs::srv::SetBool::Response::SharedPtr response) {
      response->success = false;
    });
  (void)allowed_service;
  (void)blocked_service;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node);

  ASSERT_TRUE(spinUntil(executor, [&]() {
    const auto services = harness.node->get_service_names_and_types();
    return services.count("/rpc_router/allowed_service") > 0U && services.count("/rpc_router/blocked_service") > 0U;
  }));

  auto handler_future = std::async(std::launch::async, [&]() {
    return harness.invokeRpc(
      protocol::kRpcServicesList,
      RpcInvocation{
        "participant-1",
        R"({"query":"rpc_router"})",
      });
  });

  ASSERT_TRUE(waitForFutureReady(executor, handler_future));

  const auto response = handler_future.get();
  ASSERT_TRUE(response.has_value());
  const auto body = nlohmann::json::parse(*response);
  ASSERT_EQ(body["services"].size(), 1U);
  EXPECT_EQ(body["services"][0]["name"].get<std::string>(), "/rpc_router/allowed_service");
  EXPECT_EQ(body["services"][0]["interface_type"].get<std::string>(), "std_srvs/srv/SetBool");
}

TEST_F(RpcRouterTest, TopicListRpcFiltersAllowedTopicsOnRosExecutorThread)
{
  RpcRouterHarness harness(makeSubscribePolicy({"/rpc_router/allowed_topic"}));
  auto allowed_topic =
    harness.node->create_publisher<sensor_msgs::msg::BatteryState>("/rpc_router/allowed_topic", rclcpp::QoS(10));
  auto blocked_topic =
    harness.node->create_publisher<sensor_msgs::msg::BatteryState>("/rpc_router/blocked_topic", rclcpp::QoS(10));
  (void)allowed_topic;
  (void)blocked_topic;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node);

  ASSERT_TRUE(spinUntil(executor, [&]() {
    const auto topics = harness.node->get_topic_names_and_types();
    return topics.count("/rpc_router/allowed_topic") > 0U && topics.count("/rpc_router/blocked_topic") > 0U;
  }));

  auto handler_future = std::async(std::launch::async, [&]() {
    return harness.invokeRpc(
      protocol::kRpcTopicsList,
      RpcInvocation{
        "participant-1",
        R"({"query":"rpc_router"})",
      });
  });

  ASSERT_TRUE(waitForFutureReady(executor, handler_future));

  const auto response = handler_future.get();
  ASSERT_TRUE(response.has_value());
  const auto body = nlohmann::json::parse(*response);
  ASSERT_EQ(body["topics"].size(), 1U);
  EXPECT_EQ(body["topics"][0]["name"].get<std::string>(), "/rpc_router/allowed_topic");
  EXPECT_EQ(body["topics"][0]["interface_type"].get<std::string>(), "sensor_msgs/msg/BatteryState");
}

}  // namespace

}  // namespace livekit_ros2_bridge
