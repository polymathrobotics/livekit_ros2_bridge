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
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "payloads/service_call_payloads.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/serialization.hpp"
#include "ros_test_support.hpp"

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

template <typename FutureT>
bool waitForFutureReady(
  rclcpp::executors::SingleThreadedExecutor & executor,
  FutureT & future,
  std::chrono::milliseconds timeout = kSpinTimeout)
{
  return test_support::spinUntil(
    executor, [&]() { return future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready; }, timeout);
}

void expectReleasedInflightSlot(
  RosServiceCaller & caller, const std::string & requester, const ServiceCallRequest & request)
{
  auto recovered_future = caller.call(requester, request);
  expectFuturePending(recovered_future);

  auto overflow_future = caller.call(requester, request);
  EXPECT_EQ(expectRuntimeErrorMessage(overflow_future), "Requester identity service call limit reached.");
}

bool waitForService(
  rclcpp::executors::SingleThreadedExecutor & executor,
  rclcpp::Node & node,
  const std::string & service_name,
  std::chrono::milliseconds timeout = kSpinTimeout)
{
  return test_support::spinUntil(
    executor,
    [&]() {
      const auto services = node.get_service_names_and_types();
      return services.find(service_name) != services.end();
    },
    timeout);
}

ServiceCallRequest makeSetBoolRequest(
  const std::string & service, int timeout_ms, std::optional<std::string> interface_type, bool data = true)
{
  ServiceCallRequest request;
  request.service = service;
  request.interface_type = interface_type.value_or("");
  std_srvs::srv::SetBool::Request ros_request;
  ros_request.data = data;
  request.request_payload = serializeMessage(ros_request);
  request.timeout_ms = timeout_ms;
  return request;
}

ServiceCallRequest makeSetBoolRequest(const std::string & service, int timeout_ms, bool data = true)
{
  return makeSetBoolRequest(service, timeout_ms, std::optional<std::string>{"std_srvs/srv/SetBool"}, data);
}

void saturateInflightQuota(
  RosServiceCaller & caller,
  const std::string & requester,
  const ServiceCallRequest & request,
  int count = kMaxInflightPerRequester)
{
  for (int i = 0; i < count; ++i) {
    (void)caller.call(requester, request);
  }
}

class RosServiceCallerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    static test_support::ScopedRclcppInit rclcpp_init;
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

  ASSERT_TRUE(waitForFutureReady(executor, future));

  const RosServiceCaller::ServiceCallResponse result = future.get();
  EXPECT_EQ(result.service, "/test_set_bool");
  const auto response = deserializeMessage<std_srvs::srv::SetBool::Response>(result.response);
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

  ASSERT_TRUE(waitForFutureReady(executor, alpha_future));
  ASSERT_TRUE(waitForFutureReady(executor, beta_future));

  const auto alpha_result = alpha_future.get();
  EXPECT_EQ(alpha_result.service, "/test_set_bool_alpha");
  const auto alpha_response = deserializeMessage<std_srvs::srv::SetBool::Response>(alpha_result.response);
  EXPECT_TRUE(alpha_response.success);
  EXPECT_EQ(alpha_response.message, "alpha");

  const auto beta_result = beta_future.get();
  EXPECT_EQ(beta_result.service, "/test_set_bool_beta");
  const auto beta_response = deserializeMessage<std_srvs::srv::SetBool::Response>(beta_result.response);
  EXPECT_FALSE(beta_response.success);
  EXPECT_EQ(beta_response.message, "beta");

  caller.shutdown();
}

TEST_F(RosServiceCallerTest, UsesDefaultAndExplicitTimeoutsWhenServiceUnavailable)
{
  auto caller_node = std::make_shared<rclcpp::Node>("ros_service_caller_timeout_node");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(caller_node);

  RosServiceCaller caller(*caller_node);

  {
    const auto start = std::chrono::steady_clock::now();
    auto future = caller.call("requester-1", makeSetBoolRequest("/nonexistent_service", 0));

    ASSERT_TRUE(waitForFutureReady(executor, future, std::chrono::seconds(4)));

    const auto elapsed =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start);
    EXPECT_GE(elapsed, std::chrono::milliseconds(kDefaultTimeoutMs) - kDefaultTimeoutSlack);
    EXPECT_LT(elapsed, std::chrono::milliseconds(kDefaultTimeoutMs) + kDefaultTimeoutUpperBoundSlack);
    EXPECT_EQ(expectRuntimeErrorMessage(future), "Service call timed out.");
  }

  {
    const auto start = std::chrono::steady_clock::now();
    auto future = caller.call("requester-1", makeSetBoolRequest("/nonexistent_service", 200));

    ASSERT_TRUE(waitForFutureReady(executor, future, std::chrono::seconds(3)));

    const auto elapsed =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start);
    EXPECT_GE(elapsed, kExplicitTimeoutLowerBound);
    EXPECT_LT(elapsed, kExplicitTimeoutUpperBound);
    EXPECT_EQ(expectRuntimeErrorMessage(future), "Service call timed out.");
  }

  caller.shutdown();
}

TEST_F(RosServiceCallerTest, EnforcesRequesterIdentityInflightLimit)
{
  auto caller_node = std::make_shared<rclcpp::Node>("ros_service_caller_inflight_node");

  RosServiceCaller caller(*caller_node);
  saturateInflightQuota(caller, "requester-1", makeSetBoolRequest("/blocked_service", kStandardRequestTimeoutMs));

  auto overflow_future = caller.call("requester-1", makeSetBoolRequest("/blocked_service", kStandardRequestTimeoutMs));

  EXPECT_EQ(expectRuntimeErrorMessage(overflow_future), "Requester identity service call limit reached.");

  caller.shutdown();
}

TEST_F(RosServiceCallerTest, ReleasesRequesterIdentityInflightQuotaWhenRequestBuildFails)
{
  auto caller_node = std::make_shared<rclcpp::Node>("ros_service_caller_build_failure_node");

  RosServiceCaller caller(*caller_node);

  const auto request = makeSetBoolRequest("/blocked_service", kStandardRequestTimeoutMs);
  saturateInflightQuota(caller, "requester-1", request, kMaxInflightPerRequester - 1);

  ServiceCallRequest malformed_request = request;
  malformed_request.request_payload.clear();
  auto malformed_future = caller.call("requester-1", malformed_request);
  const std::string malformed_error = expectRuntimeErrorMessage(malformed_future);
  EXPECT_NE(malformed_error.find("Failed to build service request:"), std::string::npos);

  expectReleasedInflightSlot(caller, "requester-1", request);

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
  saturateInflightQuota(caller, "requester-1", holding_request, kMaxInflightPerRequester - 1);

  auto settled_future = caller.call("requester-1", makeSetBoolRequest("/release_response", kResponseSettleTimeoutMs));

  ASSERT_TRUE(waitForFutureReady(executor, settled_future));

  const RosServiceCaller::ServiceCallResponse result = settled_future.get();
  const auto settled_response = deserializeMessage<std_srvs::srv::SetBool::Response>(result.response);
  EXPECT_TRUE(settled_response.success);
  EXPECT_EQ(settled_response.message, "released");

  expectReleasedInflightSlot(caller, "requester-1", holding_request);

  caller.shutdown();
}

TEST_F(RosServiceCallerTest, ReleasesRequesterIdentityInflightQuotaWhenCallTimesOut)
{
  auto caller_node = std::make_shared<rclcpp::Node>("ros_service_caller_timeout_release_node");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(caller_node);

  RosServiceCaller caller(*caller_node);

  const auto holding_request = makeSetBoolRequest("/blocked_timeout_release", kStandardRequestTimeoutMs);
  saturateInflightQuota(caller, "requester-1", holding_request, kMaxInflightPerRequester - 1);

  auto settled_future = caller.call("requester-1", makeSetBoolRequest("/timed_out_release", 200));

  ASSERT_TRUE(waitForFutureReady(executor, settled_future, std::chrono::seconds(3)));

  EXPECT_EQ(expectRuntimeErrorMessage(settled_future), "Service call timed out.");

  expectReleasedInflightSlot(caller, "requester-1", holding_request);

  caller.shutdown();
}

TEST_F(RosServiceCallerTest, DropsLateTimedOutResponseBeforeSettlingLaterCallOnSameService)
{
  auto caller_node = std::make_shared<rclcpp::Node>("ros_service_caller_late_timeout_drop_node");
  auto server_node = std::make_shared<rclcpp::Node>("service_server_late_timeout_drop_node");

  auto first_request_started = std::make_shared<std::promise<void>>();
  auto first_request_started_future = first_request_started->get_future();
  auto release_first_response = std::make_shared<std::promise<void>>();
  auto release_first_response_future = release_first_response->get_future().share();
  auto second_request_started = std::make_shared<std::promise<void>>();
  auto second_request_started_future = second_request_started->get_future();
  auto release_second_response = std::make_shared<std::promise<void>>();
  auto release_second_response_future = release_second_response->get_future().share();
  auto invocation_count = std::make_shared<std::atomic<int>>(0);

  auto service = server_node->create_service<std_srvs::srv::SetBool>(
    "/late_timeout_drop",
    [first_request_started,
     release_first_response_future,
     second_request_started,
     release_second_response_future,
     invocation_count](
      const std_srvs::srv::SetBool::Request::SharedPtr request, std_srvs::srv::SetBool::Response::SharedPtr response) {
      const int invocation_index = invocation_count->fetch_add(1, std::memory_order_relaxed) + 1;
      if (invocation_index == 1) {
        first_request_started->set_value();
        release_first_response_future.wait();
      } else if (invocation_index == 2) {
        second_request_started->set_value();
        release_second_response_future.wait();
      }

      response->success = request->data;
      response->message = "completed";
    });
  (void)service;

  rclcpp::executors::SingleThreadedExecutor caller_executor;
  caller_executor.add_node(caller_node);

  rclcpp::executors::SingleThreadedExecutor server_executor;
  server_executor.add_node(server_node);

  ASSERT_TRUE(waitForService(caller_executor, *caller_node, "/late_timeout_drop"));

  RosServiceCaller caller(*caller_node);

  std::thread caller_spin_thread([&caller_executor]() { caller_executor.spin(); });
  std::thread server_spin_thread([&server_executor]() { server_executor.spin(); });

  bool first_response_released = false;
  bool second_response_released = false;
  const auto release_first_callback = [&]() {
    if (first_response_released) {
      return;
    }
    first_response_released = true;
    release_first_response->set_value();
  };
  const auto release_second_callback = [&]() {
    if (second_response_released) {
      return;
    }
    second_response_released = true;
    release_second_response->set_value();
  };

  auto first_future = caller.call("requester-1", makeSetBoolRequest("/late_timeout_drop", 100));

  ASSERT_EQ(first_request_started_future.wait_for(kShutdownCoordinationTimeout), std::future_status::ready);
  ASSERT_TRUE(
    test_support::waitUntil(
      [&]() { return first_future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready; },
      std::chrono::seconds(3)));
  EXPECT_EQ(expectRuntimeErrorMessage(first_future), "Service call timed out.");

  auto second_future =
    caller.call("requester-1", makeSetBoolRequest("/late_timeout_drop", kResponseSettleTimeoutMs, false));
  expectFuturePending(second_future);

  release_first_callback();

  ASSERT_EQ(second_request_started_future.wait_for(kShutdownCoordinationTimeout), std::future_status::ready);
  EXPECT_EQ(second_future.wait_for(std::chrono::milliseconds(200)), std::future_status::timeout);

  release_second_callback();

  ASSERT_TRUE(
    test_support::waitUntil(
      [&]() { return second_future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready; },
      std::chrono::seconds(3)));
  const auto second_response_payload = second_future.get();
  EXPECT_EQ(second_response_payload.service, "/late_timeout_drop");
  EXPECT_EQ(second_response_payload.interface_type, "std_srvs/srv/SetBool");
  const auto second_response = deserializeMessage<std_srvs::srv::SetBool::Response>(second_response_payload.response);
  EXPECT_FALSE(second_response.success);
  EXPECT_EQ(second_response.message, "completed");

  release_first_callback();
  release_second_callback();
  caller.shutdown();
  caller_executor.cancel();
  server_executor.cancel();
  caller_spin_thread.join();
  server_spin_thread.join();
}

TEST_F(RosServiceCallerTest, CancelCallsForRequesterOnlySettlesMatchingRequesterCalls)
{
  auto caller_node = std::make_shared<rclcpp::Node>("ros_service_caller_scoped_disconnect_node");

  RosServiceCaller caller(*caller_node);

  const auto request = makeSetBoolRequest("/scoped_disconnect_release", kStandardRequestTimeoutMs);

  std::vector<std::future<RosServiceCaller::ServiceCallResponse>> requester_one_futures;
  for (int i = 0; i < kMaxInflightPerRequester; ++i) {
    requester_one_futures.push_back(caller.call("requester-1", request));
  }
  auto requester_two_future = caller.call("requester-2", request);

  caller.cancelCallsForRequester("requester-1");

  for (auto & requester_one_future : requester_one_futures) {
    EXPECT_EQ(expectRuntimeErrorMessage(requester_one_future), "Requester identity disconnected.");
  }
  expectFuturePending(requester_two_future);

  saturateInflightQuota(caller, "requester-1", request);
  auto requester_one_overflow_future = caller.call("requester-1", request);
  EXPECT_EQ(expectRuntimeErrorMessage(requester_one_overflow_future), "Requester identity service call limit reached.");

  saturateInflightQuota(caller, "requester-2", request, kMaxInflightPerRequester - 1);
  auto requester_two_overflow_future = caller.call("requester-2", request);
  EXPECT_EQ(expectRuntimeErrorMessage(requester_two_overflow_future), "Requester identity service call limit reached.");

  caller.shutdown();
}

TEST_F(RosServiceCallerTest, SessionResetCompletesInflightCallsAndReleasesRequesterIdentityInflightQuota)
{
  auto caller_node = std::make_shared<rclcpp::Node>("ros_service_caller_reset_release_node");

  RosServiceCaller caller(*caller_node);

  const auto request = makeSetBoolRequest("/session_reset_release", kStandardRequestTimeoutMs);
  std::vector<std::future<RosServiceCaller::ServiceCallResponse>> inflight_futures;
  for (int i = 0; i < kMaxInflightPerRequester; ++i) {
    inflight_futures.push_back(caller.call("requester-1", request));
  }

  for (auto & inflight_future : inflight_futures) {
    expectFuturePending(inflight_future);
  }

  caller.resetSessionState();

  for (auto & inflight_future : inflight_futures) {
    EXPECT_EQ(expectRuntimeErrorMessage(inflight_future), "LiveKit session reset.");
  }

  saturateInflightQuota(caller, "requester-1", request);
  auto overflow_future = caller.call("requester-1", request);
  EXPECT_EQ(expectRuntimeErrorMessage(overflow_future), "Requester identity service call limit reached.");

  caller.shutdown();
}

TEST_F(RosServiceCallerTest, RejectsEmptyRequesterIdentity)
{
  auto caller_node = std::make_shared<rclcpp::Node>("ros_service_caller_empty_requester_node");

  RosServiceCaller caller(*caller_node);

  auto anonymous_future = caller.call("", makeSetBoolRequest("/blocked_service", kStandardRequestTimeoutMs));
  ASSERT_EQ(anonymous_future.wait_for(std::chrono::milliseconds(0)), std::future_status::ready);
  EXPECT_EQ(expectInvalidArgumentMessage(anonymous_future), "requester_identity is required");

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

  auto future =
    caller.call("requester-1", makeSetBoolRequest("/resolve_test", kResponseSettleTimeoutMs, std::nullopt, false));

  ASSERT_TRUE(waitForFutureReady(executor, future));

  const RosServiceCaller::ServiceCallResponse result = future.get();
  EXPECT_EQ(result.interface_type, "std_srvs/srv/SetBool");
  const auto response = deserializeMessage<std_srvs::srv::SetBool::Response>(result.response);
  EXPECT_TRUE(response.success);

  caller.shutdown();
}

TEST_F(RosServiceCallerTest, RejectsUnresolvableServiceType)
{
  auto caller_node = std::make_shared<rclcpp::Node>("ros_service_caller_unresolvable_node");

  RosServiceCaller caller(*caller_node);

  ServiceCallRequest request = makeSetBoolRequest("/no_such_service", 100, std::nullopt, false);

  auto future = caller.call("requester-1", request);

  EXPECT_EQ(expectInvalidArgumentMessage(future), "No ROS types found for service '/no_such_service'.");

  caller.shutdown();
}

TEST_F(RosServiceCallerTest, CachesInvalidRequestedServiceTypeFailures)
{
  auto caller_node = std::make_shared<rclcpp::Node>("ros_service_caller_invalid_type_cache_node");

  RosServiceCaller caller(*caller_node);

  int type_support_load_attempts = 0;
  caller.setTypeSupportLoadCallbackForTest([&type_support_load_attempts](const std::string & interface_type) {
    ++type_support_load_attempts;
    EXPECT_EQ(interface_type, "nonexistent_pkg/srv/Foo");
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

  EXPECT_EQ(type_support_load_attempts, 1);
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

  int type_support_load_attempts = 0;
  caller.setTypeSupportLoadCallbackForTest([&type_support_load_attempts](const std::string & interface_type) {
    ++type_support_load_attempts;
    EXPECT_EQ(interface_type, "std_srvs/srv/SetBool");
  });

  auto first_future =
    caller.call("requester-1", makeSetBoolRequest("/session_reset_cache_test", kResponseSettleTimeoutMs));
  ASSERT_TRUE(waitForFutureReady(executor, first_future));
  EXPECT_TRUE(deserializeMessage<std_srvs::srv::SetBool::Response>(first_future.get().response).success);
  EXPECT_EQ(type_support_load_attempts, 1);

  auto second_future =
    caller.call("requester-1", makeSetBoolRequest("/session_reset_cache_test", kResponseSettleTimeoutMs, false));
  ASSERT_TRUE(waitForFutureReady(executor, second_future));
  EXPECT_FALSE(deserializeMessage<std_srvs::srv::SetBool::Response>(second_future.get().response).success);
  EXPECT_EQ(type_support_load_attempts, 1);

  caller.resetSessionState();

  auto third_future =
    caller.call("requester-1", makeSetBoolRequest("/session_reset_cache_test", kResponseSettleTimeoutMs));
  ASSERT_TRUE(waitForFutureReady(executor, third_future));
  EXPECT_TRUE(deserializeMessage<std_srvs::srv::SetBool::Response>(third_future.get().response).success);
  EXPECT_EQ(type_support_load_attempts, 2);

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

  caller.setPollCallbacksForTest(
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

  auto inflight_future = caller.call("requester-1", makeSetBoolRequest("/shutdown_quiesce", kStandardRequestTimeoutMs));

  ASSERT_EQ(poll_entered_future.wait_for(kShutdownCoordinationTimeout), std::future_status::ready);

  auto shutdown_started = std::make_shared<std::promise<void>>();
  auto shutdown_started_future = shutdown_started->get_future();
  auto shutdown_future = std::async(std::launch::async, [&caller, shutdown_started]() {
    shutdown_started->set_value();
    caller.shutdown();
  });

  ASSERT_EQ(shutdown_started_future.wait_for(kShutdownCoordinationTimeout), std::future_status::ready);
  EXPECT_EQ(shutdown_future.wait_for(kShutdownBlockedWindow), std::future_status::timeout);
  EXPECT_EQ(inflight_future.wait_for(kShutdownBlockedWindow), std::future_status::timeout);

  release_poll_callback();

  EXPECT_EQ(poll_exited_future.wait_for(kShutdownCoordinationTimeout), std::future_status::ready);
  EXPECT_EQ(shutdown_future.wait_for(kShutdownCoordinationTimeout), std::future_status::ready);
  EXPECT_EQ(expectRuntimeErrorMessage(inflight_future), "Service caller shut down.");

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

  caller.setPollCallbacksForTest(
    [&caller, shutdown_completed]() {
      caller.shutdown();
      shutdown_completed->set_value();
    },
    [poll_exited]() { poll_exited->set_value(); });

  std::thread spin_thread([&executor]() { executor.spin(); });

  auto inflight_future =
    caller.call("requester-1", makeSetBoolRequest("/reentrant_shutdown", kStandardRequestTimeoutMs));

  EXPECT_EQ(shutdown_completed_future.wait_for(kShutdownCoordinationTimeout), std::future_status::ready);
  EXPECT_EQ(poll_exited_future.wait_for(kShutdownCoordinationTimeout), std::future_status::ready);
  EXPECT_EQ(inflight_future.wait_for(kShutdownCoordinationTimeout), std::future_status::ready);
  EXPECT_EQ(expectRuntimeErrorMessage(inflight_future), "Service caller shut down.");

  executor.cancel();
  spin_thread.join();
}

TEST_F(RosServiceCallerTest, RejectsCallAfterShutdown)
{
  auto caller_node = std::make_shared<rclcpp::Node>("ros_service_caller_post_shutdown_node");

  RosServiceCaller caller(*caller_node);

  caller.shutdown();

  auto future = caller.call("requester-1", makeSetBoolRequest("/any_service", 100));

  EXPECT_EQ(expectRuntimeErrorMessage(future), "Service caller is shut down.");
}

}  // namespace

}  // namespace livekit_ros2_bridge
