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
#include <chrono>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "payloads/service_call_payloads.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#define private public
#include "ros_service_caller.hpp"
#undef private
#include "std_srvs/srv/set_bool.hpp"

namespace livekit_ros2_bridge
{

namespace
{

constexpr int kDefaultTimeoutMs = 2000;
constexpr int kMaxInflightPerRequester = 4;
constexpr auto kSpinPollInterval = std::chrono::milliseconds(10);
constexpr auto kSpinTimeout = std::chrono::seconds(5);
constexpr auto kDefaultTimeoutSlack = std::chrono::milliseconds(250);
constexpr auto kDefaultTimeoutUpperBoundSlack = std::chrono::milliseconds(2000);
constexpr auto kExplicitTimeoutLowerBound = std::chrono::milliseconds(150);
constexpr auto kExplicitTimeoutUpperBound = std::chrono::milliseconds(2000);
constexpr auto kShutdownCoordinationTimeout = std::chrono::seconds(2);
constexpr auto kShutdownBlockedWindow = std::chrono::milliseconds(50);
constexpr int kStandardRequestTimeoutMs = 5000;
constexpr int kResponseSettleTimeoutMs = 2000;

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
std::string expectRuntimeErrorMessage(FutureT & future)
{
  try {
    (void)future.get();
    ADD_FAILURE() << "Expected std::runtime_error";
  } catch (const std::runtime_error & exc) {
    return exc.what();
  } catch (...) {
    ADD_FAILURE() << "Expected std::runtime_error";
  }
  return "";
}

template <typename FutureT>
std::string expectInvalidArgumentMessage(FutureT & future)
{
  try {
    (void)future.get();
    ADD_FAILURE() << "Expected std::invalid_argument";
  } catch (const std::invalid_argument & exc) {
    return exc.what();
  } catch (...) {
    ADD_FAILURE() << "Expected std::invalid_argument";
  }
  return "";
}

template <typename FutureT>
void expectFuturePending(FutureT & future)
{
  EXPECT_EQ(future.wait_for(std::chrono::milliseconds(0)), std::future_status::timeout);
}

bool waitForService(
  rclcpp::executors::SingleThreadedExecutor & executor,
  rclcpp::Node & node,
  const std::string & service_name,
  std::chrono::milliseconds timeout = kSpinTimeout)
{
  return spinUntil(
    executor,
    [&]() {
      const auto services = node.get_service_names_and_types();
      return services.find(service_name) != services.end();
    },
    timeout);
}

ServiceCallRequest makeSetBoolRequest(const std::string & service, int timeout_ms, bool data = true)
{
  ServiceCallRequest request;
  request.service = service;
  request.interface_type = "std_srvs/srv/SetBool";
  std_srvs::srv::SetBool::Request ros_request;
  ros_request.data = data;
  request.request_payload = serializeMessage(ros_request);
  request.timeout_ms = timeout_ms;
  return request;
}

class RosServiceCallerTest : public ::testing::Test
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

TEST_F(RosServiceCallerTest, CallsServiceAndReturnsResponse)
{
  auto caller_node = std::make_shared<rclcpp::Node>("ros_service_caller_node");
  auto server_node = std::make_shared<rclcpp::Node>("service_server_node");

  auto service = server_node->create_service<std_srvs::srv::SetBool>(
    "/test_set_bool",
    [](const std_srvs::srv::SetBool::Request::SharedPtr request, std_srvs::srv::SetBool::Response::SharedPtr response) {
      response->success = request->data;
      response->message = request->data ? "enabled" : "disabled";
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(caller_node);
  executor.add_node(server_node);

  RosServiceCaller caller(*caller_node);

  auto future = caller.call("requester-1", makeSetBoolRequest("/test_set_bool", kResponseSettleTimeoutMs));

  ASSERT_TRUE(
    spinUntil(executor, [&]() { return future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready; }));

  const RosServiceCaller::ServiceCallResponse service_call_response = future.get();
  EXPECT_EQ(service_call_response.service, "/test_set_bool");
  EXPECT_EQ(service_call_response.interface_type, "std_srvs/srv/SetBool");
  const auto response = deserializeMessage<std_srvs::srv::SetBool::Response>(service_call_response.response);
  EXPECT_TRUE(response.success);
  EXPECT_EQ(response.message, "enabled");

  caller.shutdown();
}

TEST_F(RosServiceCallerTest, MatchesConcurrentResponsesByClientAndSequence)
{
  auto caller_node = std::make_shared<rclcpp::Node>("ros_service_caller_multi_client_node");
  auto server_node = std::make_shared<rclcpp::Node>("service_server_multi_client_node");

  auto alpha_service = server_node->create_service<std_srvs::srv::SetBool>(
    "/test_set_bool_alpha",
    [](const std_srvs::srv::SetBool::Request::SharedPtr, std_srvs::srv::SetBool::Response::SharedPtr response) {
      response->success = true;
      response->message = "alpha";
    });
  auto beta_service = server_node->create_service<std_srvs::srv::SetBool>(
    "/test_set_bool_beta",
    [](const std_srvs::srv::SetBool::Request::SharedPtr, std_srvs::srv::SetBool::Response::SharedPtr response) {
      response->success = false;
      response->message = "beta";
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(caller_node);
  executor.add_node(server_node);

  ASSERT_TRUE(waitForService(executor, *caller_node, "/test_set_bool_alpha"));
  ASSERT_TRUE(waitForService(executor, *caller_node, "/test_set_bool_beta"));

  RosServiceCaller caller(*caller_node);

  auto alpha_future = caller.call("requester-alpha", makeSetBoolRequest("/test_set_bool_alpha", 1000));
  auto beta_future = caller.call("requester-beta", makeSetBoolRequest("/test_set_bool_beta", 1000, false));

  ASSERT_TRUE(spinUntil(executor, [&]() {
    return alpha_future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready &&
           beta_future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready;
  }));

  const auto alpha_result = alpha_future.get();
  EXPECT_EQ(alpha_result.service, "/test_set_bool_alpha");
  EXPECT_EQ(alpha_result.interface_type, "std_srvs/srv/SetBool");
  const auto alpha_response = deserializeMessage<std_srvs::srv::SetBool::Response>(alpha_result.response);
  EXPECT_TRUE(alpha_response.success);
  EXPECT_EQ(alpha_response.message, "alpha");

  const auto beta_result = beta_future.get();
  EXPECT_EQ(beta_result.service, "/test_set_bool_beta");
  EXPECT_EQ(beta_result.interface_type, "std_srvs/srv/SetBool");
  const auto beta_response = deserializeMessage<std_srvs::srv::SetBool::Response>(beta_result.response);
  EXPECT_FALSE(beta_response.success);
  EXPECT_EQ(beta_response.message, "beta");

  caller.shutdown();
}

TEST_F(RosServiceCallerTest, UsesDefaultTimeoutWhenTimeoutNotProvided)
{
  auto caller_node = std::make_shared<rclcpp::Node>("ros_service_caller_default_timeout_node");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(caller_node);

  RosServiceCaller caller(*caller_node);

  const auto start = std::chrono::steady_clock::now();
  auto future = caller.call("requester-1", makeSetBoolRequest("/nonexistent_service", 0));

  ASSERT_TRUE(spinUntil(
    executor,
    [&]() { return future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready; },
    std::chrono::seconds(4)));

  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start);
  EXPECT_GE(elapsed, std::chrono::milliseconds(kDefaultTimeoutMs) - kDefaultTimeoutSlack);
  EXPECT_LT(elapsed, std::chrono::milliseconds(kDefaultTimeoutMs) + kDefaultTimeoutUpperBoundSlack);
  EXPECT_EQ(expectRuntimeErrorMessage(future), "Service call timed out.");

  caller.shutdown();
}

TEST_F(RosServiceCallerTest, UsesExplicitTimeoutWhenServiceUnavailable)
{
  auto caller_node = std::make_shared<rclcpp::Node>("ros_service_caller_timeout_node");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(caller_node);

  RosServiceCaller caller(*caller_node);

  const auto start = std::chrono::steady_clock::now();
  auto future = caller.call("requester-1", makeSetBoolRequest("/nonexistent_service", 200));

  ASSERT_TRUE(spinUntil(
    executor,
    [&]() { return future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready; },
    std::chrono::seconds(3)));

  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start);
  EXPECT_GE(elapsed, kExplicitTimeoutLowerBound);
  EXPECT_LT(elapsed, kExplicitTimeoutUpperBound);
  EXPECT_EQ(expectRuntimeErrorMessage(future), "Service call timed out.");

  caller.shutdown();
}

TEST_F(RosServiceCallerTest, EnforcesRequesterIdentityInflightLimit)
{
  auto caller_node = std::make_shared<rclcpp::Node>("ros_service_caller_inflight_node");

  RosServiceCaller caller(*caller_node);

  for (int i = 0; i < kMaxInflightPerRequester; ++i) {
    (void)caller.call("requester-1", makeSetBoolRequest("/blocked_service", kStandardRequestTimeoutMs));
  }

  auto overflow_future = caller.call("requester-1", makeSetBoolRequest("/blocked_service", kStandardRequestTimeoutMs));

  EXPECT_EQ(expectRuntimeErrorMessage(overflow_future), "Requester identity service call limit reached.");

  caller.shutdown();
}

TEST_F(RosServiceCallerTest, ReleasesRequesterIdentityInflightQuotaWhenRequestBuildFails)
{
  auto caller_node = std::make_shared<rclcpp::Node>("ros_service_caller_build_failure_node");

  RosServiceCaller caller(*caller_node);

  const auto request = makeSetBoolRequest("/blocked_service", kStandardRequestTimeoutMs);

  for (int i = 0; i < kMaxInflightPerRequester - 1; ++i) {
    (void)caller.call("requester-1", request);
  }

  ServiceCallRequest malformed_request = request;
  malformed_request.request_payload.clear();
  auto malformed_future = caller.call("requester-1", malformed_request);
  const std::string malformed_error = expectRuntimeErrorMessage(malformed_future);
  EXPECT_NE(malformed_error.find("Failed to build service request:"), std::string::npos);

  auto recovered_future = caller.call("requester-1", request);
  expectFuturePending(recovered_future);

  auto overflow_future = caller.call("requester-1", request);
  EXPECT_EQ(expectRuntimeErrorMessage(overflow_future), "Requester identity service call limit reached.");

  caller.shutdown();
}

TEST_F(RosServiceCallerTest, ReleasesRequesterIdentityInflightQuotaWhenCallSettlesByResponse)
{
  auto caller_node = std::make_shared<rclcpp::Node>("ros_service_caller_response_release_node");
  auto server_node = std::make_shared<rclcpp::Node>("service_server_response_release_node");

  auto service = server_node->create_service<std_srvs::srv::SetBool>(
    "/release_response",
    [](const std_srvs::srv::SetBool::Request::SharedPtr, std_srvs::srv::SetBool::Response::SharedPtr response) {
      response->success = true;
      response->message = "released";
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(caller_node);
  executor.add_node(server_node);

  ASSERT_TRUE(waitForService(executor, *caller_node, "/release_response"));

  RosServiceCaller caller(*caller_node);

  const auto holding_request = makeSetBoolRequest("/blocked_response_release", kStandardRequestTimeoutMs);
  for (int i = 0; i < kMaxInflightPerRequester - 1; ++i) {
    (void)caller.call("requester-1", holding_request);
  }

  auto settled_future = caller.call("requester-1", makeSetBoolRequest("/release_response", kResponseSettleTimeoutMs));

  ASSERT_TRUE(spinUntil(
    executor, [&]() { return settled_future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready; }));

  const RosServiceCaller::ServiceCallResponse settled_response_payload = settled_future.get();
  const auto settled_response = deserializeMessage<std_srvs::srv::SetBool::Response>(settled_response_payload.response);
  EXPECT_TRUE(settled_response.success);
  EXPECT_EQ(settled_response.message, "released");

  auto recovered_future = caller.call("requester-1", holding_request);
  expectFuturePending(recovered_future);

  auto overflow_future = caller.call("requester-1", holding_request);
  EXPECT_EQ(expectRuntimeErrorMessage(overflow_future), "Requester identity service call limit reached.");

  caller.shutdown();
}

TEST_F(RosServiceCallerTest, ReleasesRequesterIdentityInflightQuotaWhenCallTimesOut)
{
  auto caller_node = std::make_shared<rclcpp::Node>("ros_service_caller_timeout_release_node");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(caller_node);

  RosServiceCaller caller(*caller_node);

  const auto holding_request = makeSetBoolRequest("/blocked_timeout_release", kStandardRequestTimeoutMs);
  for (int i = 0; i < kMaxInflightPerRequester - 1; ++i) {
    (void)caller.call("requester-1", holding_request);
  }

  auto settled_future = caller.call("requester-1", makeSetBoolRequest("/timed_out_release", 200));

  ASSERT_TRUE(spinUntil(
    executor,
    [&]() { return settled_future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready; },
    std::chrono::seconds(3)));

  EXPECT_EQ(expectRuntimeErrorMessage(settled_future), "Service call timed out.");

  auto recovered_future = caller.call("requester-1", holding_request);
  expectFuturePending(recovered_future);

  auto overflow_future = caller.call("requester-1", holding_request);
  EXPECT_EQ(expectRuntimeErrorMessage(overflow_future), "Requester identity service call limit reached.");

  caller.shutdown();
}

TEST_F(RosServiceCallerTest, ReleasesRequesterIdentityInflightQuotaWhenRequesterDisconnects)
{
  auto caller_node = std::make_shared<rclcpp::Node>("ros_service_caller_disconnect_release_node");

  RosServiceCaller caller(*caller_node);

  const auto request = makeSetBoolRequest("/disconnect_release", kStandardRequestTimeoutMs);
  std::vector<std::future<RosServiceCaller::ServiceCallResponse>> pending_futures;
  for (int i = 0; i < kMaxInflightPerRequester; ++i) {
    pending_futures.push_back(caller.call("requester-1", request));
  }

  caller.cancelCallsForRequester("requester-1");

  for (auto & pending_future : pending_futures) {
    EXPECT_EQ(expectRuntimeErrorMessage(pending_future), "Requester identity disconnected.");
  }

  for (int i = 0; i < kMaxInflightPerRequester; ++i) {
    auto recovered_future = caller.call("requester-1", request);
    expectFuturePending(recovered_future);
  }

  auto overflow_future = caller.call("requester-1", request);
  EXPECT_EQ(expectRuntimeErrorMessage(overflow_future), "Requester identity service call limit reached.");

  caller.shutdown();
}

TEST_F(RosServiceCallerTest, SessionResetCompletesPendingCallsAndReleasesRequesterIdentityInflightQuota)
{
  auto caller_node = std::make_shared<rclcpp::Node>("ros_service_caller_reset_release_node");

  RosServiceCaller caller(*caller_node);

  const auto request = makeSetBoolRequest("/session_reset_release", kStandardRequestTimeoutMs);
  std::vector<std::future<RosServiceCaller::ServiceCallResponse>> pending_futures;
  for (int i = 0; i < kMaxInflightPerRequester; ++i) {
    pending_futures.push_back(caller.call("requester-1", request));
  }

  for (auto & pending_future : pending_futures) {
    expectFuturePending(pending_future);
  }

  caller.resetSessionState();

  for (auto & pending_future : pending_futures) {
    ASSERT_EQ(pending_future.wait_for(std::chrono::milliseconds(0)), std::future_status::ready);
    EXPECT_EQ(expectRuntimeErrorMessage(pending_future), "LiveKit session reset.");
  }

  for (int i = 0; i < kMaxInflightPerRequester; ++i) {
    auto recovered_future = caller.call("requester-1", request);
    expectFuturePending(recovered_future);
  }

  auto overflow_future = caller.call("requester-1", request);
  EXPECT_EQ(expectRuntimeErrorMessage(overflow_future), "Requester identity service call limit reached.");

  caller.shutdown();
}

TEST_F(RosServiceCallerTest, RejectsEmptyRequesterIdentityBeforeConsumingRequesterIdentityInflightQuota)
{
  auto caller_node = std::make_shared<rclcpp::Node>("ros_service_caller_empty_requester_node");

  RosServiceCaller caller(*caller_node);

  const auto request = makeSetBoolRequest("/blocked_service", kStandardRequestTimeoutMs);

  auto anonymous_future = caller.call("", request);
  ASSERT_EQ(anonymous_future.wait_for(std::chrono::milliseconds(0)), std::future_status::ready);
  EXPECT_EQ(expectInvalidArgumentMessage(anonymous_future), "requester_identity is required");

  for (int i = 0; i < kMaxInflightPerRequester; ++i) {
    (void)caller.call("requester-1", request);
  }

  auto overflow_future = caller.call("requester-1", request);
  EXPECT_EQ(expectRuntimeErrorMessage(overflow_future), "Requester identity service call limit reached.");

  caller.shutdown();
}

TEST_F(RosServiceCallerTest, ResolvesServiceTypeFromGraph)
{
  auto caller_node = std::make_shared<rclcpp::Node>("ros_service_caller_resolve_node");
  auto server_node = std::make_shared<rclcpp::Node>("service_server_resolve_node");

  auto service = server_node->create_service<std_srvs::srv::SetBool>(
    "/resolve_test",
    [](const std_srvs::srv::SetBool::Request::SharedPtr, std_srvs::srv::SetBool::Response::SharedPtr response) {
      response->success = true;
      response->message = "resolved";
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(caller_node);
  executor.add_node(server_node);

  ASSERT_TRUE(waitForService(executor, *caller_node, "/resolve_test"));

  RosServiceCaller caller(*caller_node);

  auto future = caller.call("requester-1", makeSetBoolRequest("/resolve_test", kResponseSettleTimeoutMs, false));

  ASSERT_TRUE(
    spinUntil(executor, [&]() { return future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready; }));

  const RosServiceCaller::ServiceCallResponse service_call_response = future.get();
  EXPECT_EQ(service_call_response.interface_type, "std_srvs/srv/SetBool");
  const auto response = deserializeMessage<std_srvs::srv::SetBool::Response>(service_call_response.response);
  EXPECT_TRUE(response.success);

  caller.shutdown();
}

TEST_F(RosServiceCallerTest, RejectsUnresolvableServiceType)
{
  auto caller_node = std::make_shared<rclcpp::Node>("ros_service_caller_unresolvable_node");

  RosServiceCaller caller(*caller_node);

  ServiceCallRequest request;
  request.service = "/no_such_service";
  std_srvs::srv::SetBool::Request ros_request;
  ros_request.data = false;
  request.request_payload = serializeMessage(ros_request);
  request.timeout_ms = 100;

  auto future = caller.call("requester-1", request);

  EXPECT_EQ(expectInvalidArgumentMessage(future), "No ROS types found for service '/no_such_service'.");

  caller.shutdown();
}

TEST_F(RosServiceCallerTest, CachesInvalidRequestedServiceTypeFailures)
{
  auto caller_node = std::make_shared<rclcpp::Node>("ros_service_caller_invalid_type_cache_node");

  RosServiceCaller caller(*caller_node);

  int resolution_attempts = 0;
  caller.setTypeSupportResolveHookForTest([&resolution_attempts](const std::string & service_type) {
    ++resolution_attempts;
    EXPECT_EQ(service_type, "nonexistent_pkg/srv/Foo");
  });

  ServiceCallRequest request;
  request.service = "/no_such_service";
  request.interface_type = "nonexistent_pkg/srv/Foo";
  std_srvs::srv::SetBool::Request ros_request;
  ros_request.data = false;
  request.request_payload = serializeMessage(ros_request);
  request.timeout_ms = 100;

  auto first_future = caller.call("requester-1", request);
  const std::string first_error = expectRuntimeErrorMessage(first_future);
  EXPECT_NE(first_error.find("Failed creating service client:"), std::string::npos);

  auto second_future = caller.call("requester-1", request);
  const std::string second_error = expectRuntimeErrorMessage(second_future);

  EXPECT_EQ(resolution_attempts, 1);
  EXPECT_EQ(second_error, first_error);

  caller.shutdown();
}

TEST_F(RosServiceCallerTest, SessionResetClearsResolvedServiceSupportCaches)
{
  auto caller_node = std::make_shared<rclcpp::Node>("ros_service_caller_session_reset_cache_node");
  auto server_node = std::make_shared<rclcpp::Node>("service_server_session_reset_cache_node");

  auto service = server_node->create_service<std_srvs::srv::SetBool>(
    "/session_reset_cache_test",
    [](const std_srvs::srv::SetBool::Request::SharedPtr request, std_srvs::srv::SetBool::Response::SharedPtr response) {
      response->success = request->data;
      response->message = request->data ? "enabled" : "disabled";
    });
  (void)service;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(caller_node);
  executor.add_node(server_node);

  ASSERT_TRUE(waitForService(executor, *caller_node, "/session_reset_cache_test"));

  RosServiceCaller caller(*caller_node);

  int resolution_attempts = 0;
  caller.setTypeSupportResolveHookForTest([&resolution_attempts](const std::string & service_type) {
    ++resolution_attempts;
    EXPECT_EQ(service_type, "std_srvs/srv/SetBool");
  });

  auto first_future =
    caller.call("requester-1", makeSetBoolRequest("/session_reset_cache_test", kResponseSettleTimeoutMs));
  ASSERT_TRUE(spinUntil(
    executor, [&]() { return first_future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready; }));
  EXPECT_TRUE(deserializeMessage<std_srvs::srv::SetBool::Response>(first_future.get().response).success);
  EXPECT_EQ(resolution_attempts, 1);

  auto second_future =
    caller.call("requester-1", makeSetBoolRequest("/session_reset_cache_test", kResponseSettleTimeoutMs, false));
  ASSERT_TRUE(spinUntil(
    executor, [&]() { return second_future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready; }));
  EXPECT_FALSE(deserializeMessage<std_srvs::srv::SetBool::Response>(second_future.get().response).success);
  EXPECT_EQ(resolution_attempts, 1);

  caller.resetSessionState();

  auto third_future =
    caller.call("requester-1", makeSetBoolRequest("/session_reset_cache_test", kResponseSettleTimeoutMs));
  ASSERT_TRUE(spinUntil(
    executor, [&]() { return third_future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready; }));
  EXPECT_TRUE(deserializeMessage<std_srvs::srv::SetBool::Response>(third_future.get().response).success);
  EXPECT_EQ(resolution_attempts, 2);

  caller.shutdown();
}

TEST_F(RosServiceCallerTest, ShutdownWaitsForActivePollTimerCallback)
{
  auto caller_node = std::make_shared<rclcpp::Node>("ros_service_caller_shutdown_quiesce_node");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(caller_node);

  RosServiceCaller caller(*caller_node);

  auto poll_entered = std::make_shared<std::promise<void>>();
  auto poll_entered_future = poll_entered->get_future();
  auto release_poll = std::make_shared<std::promise<void>>();
  auto release_poll_future = release_poll->get_future().share();
  auto poll_exited = std::make_shared<std::promise<void>>();
  auto poll_exited_future = poll_exited->get_future();

  caller.setPollCallbackHooksForTest(
    [poll_entered, release_poll_future]() {
      poll_entered->set_value();
      release_poll_future.wait();
    },
    [poll_exited]() { poll_exited->set_value(); });

  std::thread spin_thread([&executor]() { executor.spin(); });

  bool release_sent = false;
  const auto release_poll_callback = [&]() {
    if (release_sent) {
      return;
    }
    release_sent = true;
    release_poll->set_value();
  };

  auto pending_future = caller.call("requester-1", makeSetBoolRequest("/shutdown_quiesce", kStandardRequestTimeoutMs));

  const auto poll_started = poll_entered_future.wait_for(kShutdownCoordinationTimeout);
  EXPECT_EQ(poll_started, std::future_status::ready);

  auto shutdown_started = std::make_shared<std::promise<void>>();
  auto shutdown_started_future = shutdown_started->get_future();
  auto shutdown_future = std::async(std::launch::async, [&caller, shutdown_started]() {
    shutdown_started->set_value();
    caller.shutdown();
  });

  const auto shutdown_started_status = shutdown_started_future.wait_for(kShutdownCoordinationTimeout);
  EXPECT_EQ(shutdown_started_status, std::future_status::ready);

  if (poll_started == std::future_status::ready && shutdown_started_status == std::future_status::ready) {
    EXPECT_EQ(shutdown_future.wait_for(kShutdownBlockedWindow), std::future_status::timeout);
    EXPECT_EQ(pending_future.wait_for(kShutdownBlockedWindow), std::future_status::timeout);
  }

  release_poll_callback();

  EXPECT_EQ(poll_exited_future.wait_for(kShutdownCoordinationTimeout), std::future_status::ready);
  EXPECT_EQ(shutdown_future.wait_for(kShutdownCoordinationTimeout), std::future_status::ready);
  EXPECT_EQ(expectRuntimeErrorMessage(pending_future), "Service caller shut down.");

  executor.cancel();
  spin_thread.join();
}

TEST_F(RosServiceCallerTest, ShutdownFromActivePollTimerCallbackDoesNotDeadlock)
{
  auto caller_node = std::make_shared<rclcpp::Node>("ros_service_caller_reentrant_shutdown_node");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(caller_node);

  RosServiceCaller caller(*caller_node);

  auto shutdown_completed = std::make_shared<std::promise<void>>();
  auto shutdown_completed_future = shutdown_completed->get_future();
  auto poll_exited = std::make_shared<std::promise<void>>();
  auto poll_exited_future = poll_exited->get_future();

  caller.setPollCallbackHooksForTest(
    [&caller, shutdown_completed]() {
      caller.shutdown();
      shutdown_completed->set_value();
    },
    [poll_exited]() { poll_exited->set_value(); });

  std::thread spin_thread([&executor]() { executor.spin(); });

  auto pending_future =
    caller.call("requester-1", makeSetBoolRequest("/reentrant_shutdown", kStandardRequestTimeoutMs));

  EXPECT_EQ(shutdown_completed_future.wait_for(kShutdownCoordinationTimeout), std::future_status::ready);
  EXPECT_EQ(poll_exited_future.wait_for(kShutdownCoordinationTimeout), std::future_status::ready);
  EXPECT_EQ(pending_future.wait_for(kShutdownCoordinationTimeout), std::future_status::ready);
  EXPECT_EQ(expectRuntimeErrorMessage(pending_future), "Service caller shut down.");

  executor.cancel();
  spin_thread.join();
}

TEST_F(RosServiceCallerTest, RejectsCallAfterShutdown)
{
  // Cover the early-return path in RosServiceCaller::call when shutdown_flag
  // is already true: the returned future must already carry the shutdown failure.
  auto caller_node = std::make_shared<rclcpp::Node>("ros_service_caller_post_shutdown_node");

  RosServiceCaller caller(*caller_node);

  caller.shutdown();

  ServiceCallRequest request;
  request.service = "/any_service";
  request.interface_type = "std_srvs/srv/SetBool";
  std_srvs::srv::SetBool::Request ros_request;
  ros_request.data = true;
  request.request_payload = serializeMessage(ros_request);
  request.timeout_ms = 100;

  auto future = caller.call("requester-1", request);

  EXPECT_EQ(expectRuntimeErrorMessage(future), "Service caller is shut down.");
}

}  // namespace

}  // namespace livekit_ros2_bridge
