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
#include "protocol/cdr.hpp"
#include "protocol/constants.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/serialization.hpp"
#include "ros_executor_queue.hpp"
#include "ros_service_caller.hpp"
#include "ros_test_support.hpp"
#include "rpc_router.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace livekit_ros2_bridge
{

namespace
{

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

class ScopedExecutorThread
{
public:
  explicit ScopedExecutorThread(rclcpp::executors::SingleThreadedExecutor & executor)
  : executor_(executor)
  , thread_([this]() { executor_.spin(); })
  {}

  ~ScopedExecutorThread()
  {
    executor_.cancel();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

private:
  rclcpp::executors::SingleThreadedExecutor & executor_;
  std::thread thread_;
};

std::string nextNodeName(const std::string & prefix)
{
  static std::atomic<int> counter{0};
  return prefix + "_" + std::to_string(counter.fetch_add(1));
}

std::string makeServiceCallRequestPayload(
  const std::string & service,
  const std::string & interface_type,
  const std::vector<std::uint8_t> & payload,
  std::optional<int> timeout_ms = std::nullopt)
{
  auto body = nlohmann::json{
    {"service", service},
    {"interface_type", interface_type},
    {"request", protocol::cdr::serialize(payload)},
  };
  if (timeout_ms.has_value()) {
    body["timeout_ms"] = *timeout_ms;
  }
  return body.dump();
}

std::string makeSetBoolRequestPayload(
  const std::string & service, bool data = false, std::optional<int> timeout_ms = std::nullopt)
{
  std_srvs::srv::SetBool::Request request_message;
  request_message.data = data;
  return makeServiceCallRequestPayload(service, "std_srvs/srv/SetBool", serializeMessage(request_message), timeout_ms);
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
  explicit RpcRouterHarness(const AccessPolicy & policy = AccessPolicy())
  : node(std::make_shared<rclcpp::Node>(nextNodeName("rpc_router_test_node")))
  , queue(node->get_node_base_interface(), node->get_node_waitables_interface(), node->get_clock())
  , caller(node->get_node_base_interface(), node->get_node_graph_interface(), node->get_node_waitables_interface())
  , router(node->get_node_graph_interface(), policy, queue, caller)
  {
    router.registerRpcs(connection);
  }

  std::optional<std::string> invokeRpc(const std::string & method, const RpcInvocation & invocation)
  {
    const auto handler_it = connection.state->rpc_handlers.find(method);
    if (handler_it == connection.state->rpc_handlers.end()) {
      throw std::runtime_error("RPC method not registered in harness: " + method);
    }
    return handler_it->second(invocation);
  }

  std::shared_ptr<rclcpp::Node> node;
  RosExecutorQueue queue;
  RosServiceCaller caller;
  FakeRoomConnection connection;
  RpcRouter router;
};

TEST(RpcRouterTest, RegisteredRpcHandlersRequireCallerIdentityBeforeParsing)
{
  test_support::ScopedRclcppInit init;
  RpcRouterHarness harness;

  const auto expectUnauthorized = [&](const std::string & method) {
    expectRpcHandlerError(
      [&]() {
        harness.invokeRpc(
          method,
          RpcInvocation{
            "",
            R"({not-json})",
          });
      },
      protocol::kUnauthorizedRpcError);
  };

  expectUnauthorized(protocol::kCallServiceRpc);
  expectUnauthorized(protocol::kShowInterfaceRpc);
  expectUnauthorized(protocol::kListServicesRpc);
  expectUnauthorized(protocol::kListTopicsRpc);
}

TEST(RpcRouterTest, ServiceCallRpcMapsInvalidPayloadToInvalidRequest)
{
  test_support::ScopedRclcppInit init;
  RpcRouterHarness harness(makeServicePolicy({"*"}));

  expectRpcHandlerError(
    [&]() {
      harness.invokeRpc(
        protocol::kCallServiceRpc,
        RpcInvocation{
          "participant-1",
          R"({"service":"/rpc_router/set_bool"})",
        });
    },
    protocol::kInvalidRequestRpcError);
}

TEST(RpcRouterTest, ServiceCallRpcReturnsForbiddenWhenServiceIsDenied)
{
  test_support::ScopedRclcppInit init;
  RpcRouterHarness harness(makeServicePolicy({"/allowed_service"}));

  expectRpcHandlerError(
    [&]() {
      harness.invokeRpc(
        protocol::kCallServiceRpc,
        RpcInvocation{
          "participant-1",
          makeSetBoolRequestPayload("/denied_service"),
        });
    },
    protocol::kForbiddenRpcError,
    "ROS service '/denied_service' not permitted.");
}

TEST(RpcRouterTest, InterfacesGetRpcMapsUnknownTypeToInternalError)
{
  test_support::ScopedRclcppInit init;
  RpcRouterHarness harness;

  expectRpcHandlerError(
    [&]() {
      harness.invokeRpc(
        protocol::kShowInterfaceRpc,
        RpcInvocation{
          "participant-1",
          R"({"interface_types":["nonexistent_pkg/msg/Foo"]})",
        });
    },
    protocol::kInternalRpcError);
}

TEST(RpcRouterTest, InterfacesGetRpcReturnsDefinitionForKnownTypeAndDeduplicatesRepeatedRequests)
{
  test_support::ScopedRclcppInit init;
  RpcRouterHarness harness;

  const auto response = harness.invokeRpc(
    protocol::kShowInterfaceRpc,
    RpcInvocation{
      "participant-1",
      R"({"interface_types":["std_msgs/msg/String","  std_msgs/msg/String  "]})",
    });

  ASSERT_TRUE(response.has_value());
  const auto body = nlohmann::json::parse(*response);
  ASSERT_EQ(body["interfaces"].size(), 1U);
  const auto & interface_definition = body["interfaces"][0];
  EXPECT_EQ(interface_definition["interface_type"].get<std::string>(), "std_msgs/msg/String");
  EXPECT_EQ(interface_definition["format"].get<std::string>(), "ros2msg");
  EXPECT_FALSE(interface_definition["definition"].get<std::string>().empty());
}

TEST(RpcRouterTest, ResourceListRpcsMapNonPositiveLimitToInvalidRequest)
{
  test_support::ScopedRclcppInit init;
  RpcRouterHarness harness;

  const auto expectInvalidLimit = [&](const std::string & method) {
    expectRpcHandlerError(
      [&]() {
        harness.invokeRpc(
          method,
          RpcInvocation{
            "participant-1",
            R"({"limit":0})",
          });
      },
      protocol::kInvalidRequestRpcError,
      "limit must be a positive integer");
  };

  expectInvalidLimit(protocol::kListServicesRpc);
  expectInvalidLimit(protocol::kListTopicsRpc);
}

TEST(RpcRouterTest, RegisterRpcsIsBestEffortAndUnregistersAllEntrypoints)
{
  test_support::ScopedRclcppInit init;
  auto node = std::make_shared<rclcpp::Node>(nextNodeName("rpc_router_registration_node"));
  RosExecutorQueue queue(node->get_node_base_interface(), node->get_node_waitables_interface(), node->get_clock());
  RosServiceCaller caller(
    node->get_node_base_interface(), node->get_node_graph_interface(), node->get_node_waitables_interface());
  FakeRoomConnection connection;
  connection.state->rejected_rpc_methods = {protocol::kListServicesRpc};

  const std::vector<std::string> expected_methods = {
    protocol::kCallServiceRpc,
    protocol::kShowInterfaceRpc,
    protocol::kListServicesRpc,
    protocol::kListTopicsRpc,
  };

  {
    RpcRouter router(node->get_node_graph_interface(), AccessPolicy(), queue, caller);

    EXPECT_FALSE(router.registerRpcs(connection));
    EXPECT_EQ(connection.state->registered_rpc_methods, expected_methods);
    EXPECT_EQ(connection.state->rpc_handlers.count(protocol::kCallServiceRpc), 1U);
    EXPECT_EQ(connection.state->rpc_handlers.count(protocol::kShowInterfaceRpc), 1U);
    EXPECT_EQ(connection.state->rpc_handlers.count(protocol::kListServicesRpc), 0U);
    EXPECT_EQ(connection.state->rpc_handlers.count(protocol::kListTopicsRpc), 1U);
  }

  EXPECT_TRUE(connection.state->rpc_handlers.empty());
  EXPECT_EQ(connection.state->unregistered_rpc_methods, expected_methods);

  caller.shutdown();
  queue.shutdown();
}

TEST(RpcRouterTest, LegacyRpcNamesAreNotRegistered)
{
  test_support::ScopedRclcppInit init;
  RpcRouterHarness harness;

  EXPECT_EQ(harness.connection.state->rpc_handlers.count("ros.services.call"), 0U);
  EXPECT_EQ(harness.connection.state->rpc_handlers.count("ros.interfaces.get"), 0U);
  EXPECT_EQ(harness.connection.state->rpc_handlers.count("ros.services.list"), 0U);
  EXPECT_EQ(harness.connection.state->rpc_handlers.count("ros.topics.list"), 0U);
}

TEST(RpcRouterTest, ServiceCallRpcDispatchesAndReturnsResponse)
{
  test_support::ScopedRclcppInit init;
  RpcRouterHarness harness(makeServicePolicy({"*"}));
  auto server_node = std::make_shared<rclcpp::Node>(nextNodeName("rpc_router_service_server"));
  [[maybe_unused]] const auto service = server_node->create_service<std_srvs::srv::SetBool>(
    "/rpc_router/set_bool",
    [](const std_srvs::srv::SetBool::Request::SharedPtr request, std_srvs::srv::SetBool::Response::SharedPtr response) {
      response->success = request->data;
      response->message = request->data ? "enabled" : "disabled";
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node);
  executor.add_node(server_node);

  ASSERT_TRUE(test_support::spinUntil(executor, [&]() {
    return harness.node->get_service_names_and_types().count("/rpc_router/set_bool") > 0U;
  }));

  ScopedExecutorThread executor_thread(executor);
  const auto rpc_response = harness.invokeRpc(
    protocol::kCallServiceRpc,
    RpcInvocation{
      "participant-1",
      makeSetBoolRequestPayload("/rpc_router/set_bool", true),
    });
  ASSERT_TRUE(rpc_response.has_value());

  const auto body = nlohmann::json::parse(*rpc_response);
  EXPECT_EQ(body["service"].get<std::string>(), "/rpc_router/set_bool");
  EXPECT_EQ(body["interface_type"].get<std::string>(), "std_srvs/srv/SetBool");
  EXPECT_FALSE(body.contains("ok"));
  EXPECT_FALSE(body.contains("elapsed_ms"));

  const auto payload = protocol::cdr::parse(body, protocol::cdr::Field::Response);
  const auto response_message = deserializeMessage<std_srvs::srv::SetBool::Response>(payload);
  EXPECT_TRUE(response_message.success);
  EXPECT_EQ(response_message.message, "enabled");
}

TEST(RpcRouterTest, ServiceCallRpcReturnsInternalErrorWhenServiceCallTimesOut)
{
  test_support::ScopedRclcppInit init;
  RpcRouterHarness harness(makeServicePolicy({"*"}));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node);

  ScopedExecutorThread executor_thread(executor);
  expectRpcHandlerError(
    [&]() {
      harness.invokeRpc(
        protocol::kCallServiceRpc,
        RpcInvocation{
          "participant-1",
          makeSetBoolRequestPayload("/no_such_service", true, 200),
        });
    },
    protocol::kInternalRpcError,
    "Service call timed out.");
}

TEST(RpcRouterTest, ServicesListRpcFiltersAllowedResourcesOnRosExecutorThread)
{
  test_support::ScopedRclcppInit init;
  RpcRouterHarness harness(makeServicePolicy({"/rpc_router/allowed_service"}));
  [[maybe_unused]] const auto allowed_service = harness.node->create_service<std_srvs::srv::SetBool>(
    "/rpc_router/allowed_service",
    [](const std_srvs::srv::SetBool::Request::SharedPtr, std_srvs::srv::SetBool::Response::SharedPtr response) {
      response->success = true;
    });
  [[maybe_unused]] const auto blocked_service = harness.node->create_service<std_srvs::srv::SetBool>(
    "/rpc_router/blocked_service",
    [](const std_srvs::srv::SetBool::Request::SharedPtr, std_srvs::srv::SetBool::Response::SharedPtr response) {
      response->success = false;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node);

  ASSERT_TRUE(test_support::spinUntil(executor, [&]() {
    const auto services = harness.node->get_service_names_and_types();
    return services.count("/rpc_router/allowed_service") > 0U && services.count("/rpc_router/blocked_service") > 0U;
  }));

  ScopedExecutorThread executor_thread(executor);
  const auto response = harness.invokeRpc(
    protocol::kListServicesRpc,
    RpcInvocation{
      "participant-1",
      R"({"query":"rpc_router"})",
    });
  ASSERT_TRUE(response.has_value());
  const auto body = nlohmann::json::parse(*response);
  ASSERT_EQ(body["services"].size(), 1U);
  EXPECT_EQ(body["services"][0]["service"].get<std::string>(), "/rpc_router/allowed_service");
  EXPECT_EQ(body["services"][0]["interface_type"].get<std::string>(), "std_srvs/srv/SetBool");
}

TEST(RpcRouterTest, TopicsListRpcMatchesInterfaceTypeQueryAndAppliesLimitAfterPolicyFiltering)
{
  test_support::ScopedRclcppInit init;
  RpcRouterHarness harness(makeSubscribePolicy({"/rpc_router/visible_topic"}));
  [[maybe_unused]] const auto blocked_topic =
    harness.node->create_publisher<sensor_msgs::msg::BatteryState>("/rpc_router/a_blocked_topic", rclcpp::QoS(10));
  [[maybe_unused]] const auto visible_topic =
    harness.node->create_publisher<sensor_msgs::msg::BatteryState>("/rpc_router/visible_topic", rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node);

  ASSERT_TRUE(test_support::spinUntil(executor, [&]() {
    const auto topics = harness.node->get_topic_names_and_types();
    return topics.count("/rpc_router/a_blocked_topic") > 0U && topics.count("/rpc_router/visible_topic") > 0U;
  }));

  ScopedExecutorThread executor_thread(executor);
  const auto response = harness.invokeRpc(
    protocol::kListTopicsRpc,
    RpcInvocation{
      "participant-1",
      R"({"query":"BatteryState","limit":1})",
    });
  ASSERT_TRUE(response.has_value());
  const auto body = nlohmann::json::parse(*response);
  ASSERT_EQ(body["topics"].size(), 1U);
  EXPECT_EQ(body["topics"][0]["topic"].get<std::string>(), "/rpc_router/visible_topic");
  EXPECT_EQ(body["topics"][0]["interface_type"].get<std::string>(), "sensor_msgs/msg/BatteryState");
}

}  // namespace

}  // namespace livekit_ros2_bridge
