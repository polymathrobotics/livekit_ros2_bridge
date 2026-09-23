// Copyright 2025 Polymath Robotics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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

#include "core/access_policy.hpp"
#include "core/ros_executor_queue.hpp"
#include "core/ros_service_caller.hpp"
#include "core/rpc_router.hpp"
#include "core/subscription_lease_manager.hpp"
#include "gtest/gtest.h"
#include "livekit/local_participant.h"
#include "livekit/rpc_error.h"
#include "nlohmann/json.hpp"
#include "protocol/cdr.hpp"
#include "protocol/constants.hpp"
#include "protocol/detail/base64.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/serialization.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "support/fake_room_connection.hpp"
#include "support/ros_test_support.hpp"
#include "utils/serialized_message.hpp"

namespace livekit_ros2_bridge
{

namespace
{

template <typename MessageT>
rclcpp::SerializedMessage serializeMessage(const MessageT & message)
{
  rclcpp::Serialization<MessageT> serialization;
  rclcpp::SerializedMessage serialized;
  serialization.serialize_message(&message, &serialized);
  return serialized;
}

std::vector<std::uint8_t> serializedMessageBytes(const rclcpp::SerializedMessage & message)
{
  const auto & raw = message.get_rcl_serialized_message();
  return {raw.buffer, raw.buffer + raw.buffer_length};
}

template <typename MessageT>
MessageT deserializeMessage(const rclcpp::SerializedMessage & payload)
{
  rclcpp::Serialization<MessageT> serialization;
  MessageT message;
  serialization.deserialize_message(&payload, &message);
  return message;
}

void expectRpcError(
  const std::function<void()> & action,
  std::uint32_t expected_code,
  const std::optional<std::string> & expected_message = std::nullopt)
{
  try {
    action();
    FAIL() << "Expected livekit::RpcError";
  } catch (const livekit::RpcError & exc) {
    EXPECT_EQ(exc.code(), expected_code);
    if (expected_message.has_value()) {
      EXPECT_EQ(exc.message(), *expected_message);
    }
  }
}

livekit::RpcInvocationData makeRpcInvocation(
  std::string caller_identity, std::string payload, std::string request_id = {})
{
  livekit::RpcInvocationData invocation;
  invocation.caller_identity = std::move(caller_identity);
  invocation.payload = std::move(payload);
  invocation.request_id = std::move(request_id);
  invocation.response_timeout_sec = 0.0;
  return invocation;
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
  const rclcpp::SerializedMessage & payload,
  std::optional<int> timeout_ms = std::nullopt)
{
  auto body = nlohmann::json{
    {"service", service},
    {"interface_type", interface_type},
    {"request", protocol::cdr::serialize(serializedMessageBytes(payload))},
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

sensor_msgs::msg::BatteryState makeBatteryState()
{
  sensor_msgs::msg::BatteryState message;
  message.voltage = 48.5F;
  message.percentage = 0.9F;
  return message;
}

std::vector<std::uint8_t> makeSubscribeHeartbeat(const std::string & topic)
{
  const std::string body = std::string(R"({"subscriptions":[{"kind":"topic","name":")") + topic + R"("}]})";
  return std::vector<std::uint8_t>(body.begin(), body.end());
}

std::string makeEchoOnceRequest(const std::string & kind, const std::string & name)
{
  return nlohmann::json{{"kind", kind}, {"name", name}}.dump();
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
  , queue(RosExecutorQueue::NodeInterfaces(*node), node->get_clock())
  , caller(node->get_node_base_interface(), node->get_node_graph_interface(), node->get_node_waitables_interface())
  // The lease manager owns the per-topic cache; it allows all subscribes so a test can populate the
  // cache regardless of the router's (separately supplied) access policy.
  , lease_manager(
      node->get_node_parameters_interface(),
      node->get_node_topics_interface(),
      node->get_node_graph_interface(),
      node->get_clock(),
      connection,
      makeSubscribePolicy({"*"}))
  , router(node->get_node_graph_interface(), policy, queue, caller, lease_manager)
  {
    router.registerRpcs(connection);
  }

  std::optional<std::string> invokeRpc(const std::string & method, const livekit::RpcInvocationData & invocation)
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
  SubscriptionLeaseManager lease_manager;
  RpcRouter router;
};

class RpcRouterTest : public test_support::RclcppTestSuite
{};

TEST_F(RpcRouterTest, RegisteredRpcHandlersRequireCallerIdentityBeforeParsing)
{
  RpcRouterHarness harness;

  // Split rule: ROS-proxy RPCs (ros2.*) require caller identity; bridge-introspection RPCs
  // (lkros.capability) do not, so they are deliberately excluded from this coverage.
  const auto expectUnauthorized = [&](const std::string & method) {
    expectRpcError(
      [&]() { harness.invokeRpc(method, makeRpcInvocation("", R"({not-json})")); }, protocol::kUnauthorizedRpcCode);
  };

  expectUnauthorized(protocol::kCallServiceMethod);
  expectUnauthorized(protocol::kShowInterfaceMethod);
  expectUnauthorized(protocol::kListServicesMethod);
  expectUnauthorized(protocol::kListTopicsMethod);
  expectUnauthorized(protocol::kTopicEchoOnceMethod);
}

void expectCapabilityBody(const std::optional<std::string> & response)
{
  ASSERT_TRUE(response.has_value());
  const auto parsed = nlohmann::json::parse(*response);
  const nlohmann::json expected = {
    {"features", nlohmann::json::object()},
    {"v", protocol::kProtocolVersion},
  };
  ASSERT_EQ(parsed, expected);
}

TEST_F(RpcRouterTest, CapabilityRpcReturnsExactBodyWithoutIdentityOrPayloadValidation)
{
  RpcRouterHarness harness;

  const auto response = harness.invokeRpc(protocol::kCapabilityMethod, makeRpcInvocation("", R"({not-json})"));
  expectCapabilityBody(response);
}

TEST_F(RpcRouterTest, CapabilityRpcIgnoresJunkPayloadAndAnswersUnderDefaultDenyPolicy)
{
  RpcRouterHarness harness(makeServicePolicy({}, {"*"}));

  const auto response = harness.invokeRpc(protocol::kCapabilityMethod, makeRpcInvocation("participant-1", R"("junk")"));
  expectCapabilityBody(response);
  EXPECT_TRUE(harness.connection.state->sent_byte_streams.empty());
  EXPECT_TRUE(harness.connection.state->published_data_calls.empty());
}

TEST_F(RpcRouterTest, ServiceCallRpcMapsInvalidPayloadToInvalidRequest)
{
  RpcRouterHarness harness(makeServicePolicy({"*"}));

  expectRpcError(
    [&]() {
      harness.invokeRpc(
        protocol::kCallServiceMethod, makeRpcInvocation("participant-1", R"({"service":"/rpc_router/set_bool"})"));
    },
    protocol::kInvalidRequestRpcCode);
}

TEST_F(RpcRouterTest, ServiceCallRpcReturnsForbiddenWhenServiceIsDenied)
{
  RpcRouterHarness harness(makeServicePolicy({"/allowed_service"}));

  expectRpcError(
    [&]() {
      harness.invokeRpc(
        protocol::kCallServiceMethod, makeRpcInvocation("participant-1", makeSetBoolRequestPayload("/denied_service")));
    },
    protocol::kForbiddenRpcCode,
    "ROS service '/denied_service' not permitted.");
}

TEST_F(RpcRouterTest, InterfaceShowRpcMapsUnknownTypeToInternalError)
{
  RpcRouterHarness harness;

  expectRpcError(
    [&]() {
      harness.invokeRpc(
        protocol::kShowInterfaceMethod,
        makeRpcInvocation("participant-1", R"({"interface_types":["nonexistent_pkg/msg/Foo"]})"));
    },
    protocol::kInternalRpcCode);
}

TEST_F(RpcRouterTest, InterfaceShowRpcReturnsDefinitionForKnownTypeAndDeduplicatesRepeatedRequests)
{
  RpcRouterHarness harness;

  const auto response = harness.invokeRpc(
    protocol::kShowInterfaceMethod,
    makeRpcInvocation("participant-1", R"({"interface_types":["std_msgs/msg/String","  std_msgs/msg/String  "]})"));

  ASSERT_TRUE(response.has_value());
  const auto body = nlohmann::json::parse(*response);
  ASSERT_EQ(body["interfaces"].size(), 1U);
  const auto & interface_definition = body["interfaces"][0];
  EXPECT_EQ(interface_definition["interface_type"].get<std::string>(), "std_msgs/msg/String");
  EXPECT_EQ(interface_definition["format"].get<std::string>(), "ros2msg");
  EXPECT_FALSE(interface_definition["definition"].get<std::string>().empty());
}

TEST_F(RpcRouterTest, ResourceListRpcsMapNonPositiveLimitToInvalidRequest)
{
  RpcRouterHarness harness;

  const auto expectInvalidLimit = [&](const std::string & method) {
    expectRpcError(
      [&]() { harness.invokeRpc(method, makeRpcInvocation("participant-1", R"({"limit":0})")); },
      protocol::kInvalidRequestRpcCode,
      "limit must be a positive integer");
  };

  expectInvalidLimit(protocol::kListServicesMethod);
  expectInvalidLimit(protocol::kListTopicsMethod);
}

TEST_F(RpcRouterTest, RegisterRpcsIsBestEffortAndUnregistersAllEntrypoints)
{
  auto node = std::make_shared<rclcpp::Node>(nextNodeName("rpc_router_registration_node"));
  RosExecutorQueue queue(RosExecutorQueue::NodeInterfaces(*node), node->get_clock());
  RosServiceCaller caller(
    node->get_node_base_interface(), node->get_node_graph_interface(), node->get_node_waitables_interface());
  FakeRoomConnection connection;
  connection.state->rejected_rpc_methods = {protocol::kListServicesMethod};
  SubscriptionLeaseManager lease_manager(
    node->get_node_parameters_interface(),
    node->get_node_topics_interface(),
    node->get_node_graph_interface(),
    node->get_clock(),
    connection,
    makeSubscribePolicy({"*"}));

  const std::vector<std::string> expected_methods = {
    protocol::kCallServiceMethod,
    protocol::kShowInterfaceMethod,
    protocol::kListServicesMethod,
    protocol::kListTopicsMethod,
    protocol::kTopicEchoOnceMethod,
    protocol::kCapabilityMethod,
  };

  {
    RpcRouter router(node->get_node_graph_interface(), AccessPolicy(), queue, caller, lease_manager);

    EXPECT_FALSE(router.registerRpcs(connection));
    EXPECT_EQ(connection.state->registered_rpc_methods, expected_methods);
    EXPECT_EQ(connection.state->rpc_handlers.count(protocol::kCallServiceMethod), 1U);
    EXPECT_EQ(connection.state->rpc_handlers.count(protocol::kShowInterfaceMethod), 1U);
    EXPECT_EQ(connection.state->rpc_handlers.count(protocol::kListServicesMethod), 0U);
    EXPECT_EQ(connection.state->rpc_handlers.count(protocol::kListTopicsMethod), 1U);
    EXPECT_EQ(connection.state->rpc_handlers.count(protocol::kTopicEchoOnceMethod), 1U);
    EXPECT_EQ(connection.state->rpc_handlers.count(protocol::kCapabilityMethod), 1U);
  }

  EXPECT_TRUE(connection.state->rpc_handlers.empty());
  EXPECT_EQ(connection.state->unregistered_rpc_methods, expected_methods);

  caller.shutdown();
  queue.shutdown();
}

TEST_F(RpcRouterTest, ServiceCallRpcDispatchesAndReturnsResponse)
{
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

  ASSERT_TRUE(test_support::spinUntil(
    executor, [&]() { return harness.node->get_service_names_and_types().count("/rpc_router/set_bool") > 0U; }));

  ScopedExecutorThread executor_thread(executor);
  const auto rpc_response = harness.invokeRpc(
    protocol::kCallServiceMethod,
    makeRpcInvocation("participant-1", makeSetBoolRequestPayload("/rpc_router/set_bool", true)));
  ASSERT_TRUE(rpc_response.has_value());

  const auto body = nlohmann::json::parse(*rpc_response);
  EXPECT_EQ(body["service"].get<std::string>(), "/rpc_router/set_bool");
  EXPECT_EQ(body["interface_type"].get<std::string>(), "std_srvs/srv/SetBool");
  EXPECT_FALSE(body.contains("ok"));
  EXPECT_FALSE(body.contains("elapsed_ms"));

  ASSERT_TRUE(body["response"].is_object());
  EXPECT_EQ(body["response"]["content_type"].get<std::string>(), protocol::kCdrContentType);
  const auto decoded = protocol::detail::base64::decode(body["response"]["payload_base64"].get<std::string>());
  ASSERT_EQ(decoded.status, protocol::detail::base64::Status::Ok);
  const auto payload = makeSerializedMessage(decoded.bytes);
  const auto response_message = deserializeMessage<std_srvs::srv::SetBool::Response>(payload);
  EXPECT_TRUE(response_message.success);
  EXPECT_EQ(response_message.message, "enabled");
}

TEST_F(RpcRouterTest, ServiceCallRpcReturnsInternalErrorWhenServiceCallTimesOut)
{
  RpcRouterHarness harness(makeServicePolicy({"*"}));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node);

  ScopedExecutorThread executor_thread(executor);
  expectRpcError(
    [&]() {
      harness.invokeRpc(
        protocol::kCallServiceMethod,
        makeRpcInvocation("participant-1", makeSetBoolRequestPayload("/no_such_service", true, 200)));
    },
    protocol::kInternalRpcCode,
    "Service call timed out.");
}

TEST_F(RpcRouterTest, ServicesListRpcFiltersAllowedResourcesOnRosExecutorThread)
{
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
  const auto response =
    harness.invokeRpc(protocol::kListServicesMethod, makeRpcInvocation("participant-1", R"({"query":"rpc_router"})"));
  ASSERT_TRUE(response.has_value());
  const auto body = nlohmann::json::parse(*response);
  ASSERT_EQ(body["services"].size(), 1U);
  EXPECT_EQ(body["services"][0]["service"].get<std::string>(), "/rpc_router/allowed_service");
  EXPECT_EQ(body["services"][0]["interface_type"].get<std::string>(), "std_srvs/srv/SetBool");
}

TEST_F(RpcRouterTest, TopicsListRpcMatchesInterfaceTypeQueryAndAppliesLimitAfterPolicyFiltering)
{
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
    protocol::kListTopicsMethod, makeRpcInvocation("participant-1", R"({"query":"BatteryState","limit":1})"));
  ASSERT_TRUE(response.has_value());
  const auto body = nlohmann::json::parse(*response);
  ASSERT_EQ(body["topics"].size(), 1U);
  EXPECT_EQ(body["topics"][0]["topic"].get<std::string>(), "/rpc_router/visible_topic");
  EXPECT_EQ(body["topics"][0]["interface_type"].get<std::string>(), "sensor_msgs/msg/BatteryState");
}

TEST_F(RpcRouterTest, TopicEchoOnceRpcSendsStreamToCallerForCachedTransientLocalTopic)
{
  RpcRouterHarness harness(makeSubscribePolicy({"*"}));
  const std::string topic = "/rpc_router/echo_once_transient_local";

  rclcpp::QoS transient_local_qos(1);
  transient_local_qos.transient_local();
  auto publisher = harness.node->create_publisher<sensor_msgs::msg::BatteryState>(topic, transient_local_qos);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node);
  ASSERT_TRUE(
    test_support::spinUntil(executor, [&]() { return harness.node->get_topic_names_and_types().count(topic) > 0U; }));

  // Subscribing caches the transient_local sample; publish until the bridge has received and cached it.
  harness.lease_manager.handleHeartbeatPayload("participant-1", makeSubscribeHeartbeat(topic));
  const auto message = makeBatteryState();
  ASSERT_TRUE(test_support::spinUntil(executor, [&]() {
    publisher->publish(message);
    return !harness.connection.state->pushed_data_track_frames.empty();
  }));

  ScopedExecutorThread executor_thread(executor);
  const auto response = harness.invokeRpc(
    protocol::kTopicEchoOnceMethod, makeRpcInvocation("participant-1", makeEchoOnceRequest("topic", topic)));
  ASSERT_TRUE(response.has_value());
  EXPECT_EQ(nlohmann::json::parse(*response)["result"].get<std::string>(), "sent");

  ASSERT_EQ(harness.connection.state->sent_byte_streams.size(), 1U);
  const auto & stream = harness.connection.state->sent_byte_streams[0];
  EXPECT_EQ(stream.topic, protocol::kEchoOnceTopic);
  EXPECT_EQ(stream.name, topic);
  EXPECT_EQ(stream.content_type, protocol::kCdrContentType);
  EXPECT_EQ(stream.destination_identity, "participant-1");

  const auto decoded = deserializeMessage<sensor_msgs::msg::BatteryState>(makeSerializedMessage(stream.payload));
  EXPECT_FLOAT_EQ(decoded.voltage, message.voltage);
}

TEST_F(RpcRouterTest, TopicEchoOnceRpcReturnsNoneWhenNothingCached)
{
  RpcRouterHarness harness(makeSubscribePolicy({"*"}));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node);

  ScopedExecutorThread executor_thread(executor);
  const auto response = harness.invokeRpc(
    protocol::kTopicEchoOnceMethod,
    makeRpcInvocation("participant-1", makeEchoOnceRequest("topic", "/rpc_router/echo_once_uncached")));
  ASSERT_TRUE(response.has_value());
  EXPECT_EQ(nlohmann::json::parse(*response)["result"].get<std::string>(), "none");
  EXPECT_TRUE(harness.connection.state->sent_byte_streams.empty());
}

TEST_F(RpcRouterTest, TopicEchoOnceRpcReturnsForbiddenForDeniedTopicAndSendsNoStream)
{
  RpcRouterHarness harness(makeSubscribePolicy({"*"}, {"/rpc_router/echo_once_denied"}));

  expectRpcError(
    [&]() {
      harness.invokeRpc(
        protocol::kTopicEchoOnceMethod,
        makeRpcInvocation("participant-1", makeEchoOnceRequest("topic", "/rpc_router/echo_once_denied")));
    },
    protocol::kForbiddenRpcCode,
    "ROS topic '/rpc_router/echo_once_denied' not permitted.");
  EXPECT_TRUE(harness.connection.state->sent_byte_streams.empty());
}

TEST_F(RpcRouterTest, TopicEchoOnceRpcRejectsUnsupportedKindAsInvalidRequest)
{
  RpcRouterHarness harness(makeSubscribePolicy({"*"}));

  expectRpcError(
    [&]() {
      harness.invokeRpc(
        protocol::kTopicEchoOnceMethod,
        makeRpcInvocation("participant-1", makeEchoOnceRequest("other_video", "/rpc_router/echo_once_topic")));
    },
    protocol::kInvalidRequestRpcCode);
  EXPECT_TRUE(harness.connection.state->sent_byte_streams.empty());
}

}  // namespace

}  // namespace livekit_ros2_bridge
