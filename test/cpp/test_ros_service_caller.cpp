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

#include <atomic>
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "protocol/services.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/serialization.hpp"
#include "ros_service_caller.hpp"
#include "ros_test_support.hpp"
#include "rosidl_runtime_cpp/traits.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "utils/serialized_message.hpp"

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
constexpr int kStandardRequestTimeoutMs = 5000;
constexpr int kResponseSettleTimeoutMs = 2000;

const char * setBoolServiceType()
{
  return rosidl_generator_traits::name<std_srvs::srv::SetBool>();
}

template <typename MessageT>
rclcpp::SerializedMessage serializeMessage(const MessageT & message)
{
  rclcpp::Serialization<MessageT> serialization;
  rclcpp::SerializedMessage serialized;
  serialization.serialize_message(&message, &serialized);
  return serialized;
}

template <typename MessageT>
MessageT deserializeMessage(const std::vector<std::uint8_t> & payload)
{
  auto serialized = makeSerializedMessage(payload);
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

RosServiceCaller makeServiceCaller(const std::shared_ptr<rclcpp::Node> & node)
{
  return RosServiceCaller(
    node->get_node_base_interface(), node->get_node_graph_interface(), node->get_node_waitables_interface());
}

ServiceCallRequest makeSetBoolRequest(
  const std::string & service, int timeout_ms, std::optional<std::string> interface_type, bool data = true)
{
  ServiceCallRequest request;
  request.name = service;
  request.interface_type = interface_type.value_or("");
  std_srvs::srv::SetBool::Request ros_request;
  ros_request.data = data;
  request.payload = serializeMessage(ros_request);
  request.timeout = std::chrono::milliseconds(timeout_ms);
  return request;
}

ServiceCallRequest makeSetBoolRequest(const std::string & service, int timeout_ms, bool data = true)
{
  return makeSetBoolRequest(service, timeout_ms, std::optional<std::string>{setBoolServiceType()}, data);
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

  auto caller = makeServiceCaller(caller_node);

  auto future = caller.call("requester-1", makeSetBoolRequest("/test_set_bool", kResponseSettleTimeoutMs));

  ASSERT_TRUE(waitForFutureReady(executor, future));

  const RosServiceCaller::Response result = future.get();
  const auto response = deserializeMessage<std_srvs::srv::SetBool::Response>(result.payload);
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

  auto caller = makeServiceCaller(caller_node);

  auto alpha_future = caller.call("requester-alpha", makeSetBoolRequest("/test_set_bool_alpha", 1000));
  auto beta_future = caller.call("requester-beta", makeSetBoolRequest("/test_set_bool_beta", 1000, false));

  ASSERT_TRUE(waitForFutureReady(executor, alpha_future));
  ASSERT_TRUE(waitForFutureReady(executor, beta_future));

  const auto alpha_result = alpha_future.get();
  EXPECT_EQ(alpha_result.name, "/test_set_bool_alpha");
  const auto alpha_response = deserializeMessage<std_srvs::srv::SetBool::Response>(alpha_result.payload);
  EXPECT_TRUE(alpha_response.success);
  EXPECT_EQ(alpha_response.message, "alpha");

  const auto beta_result = beta_future.get();
  EXPECT_EQ(beta_result.name, "/test_set_bool_beta");
  const auto beta_response = deserializeMessage<std_srvs::srv::SetBool::Response>(beta_result.payload);
  EXPECT_FALSE(beta_response.success);
  EXPECT_EQ(beta_response.message, "beta");

  caller.shutdown();
}

TEST_F(RosServiceCallerTest, UsesDefaultAndExplicitTimeoutsWhenServiceUnavailable)
{
  auto caller_node = std::make_shared<rclcpp::Node>("ros_service_caller_timeout_node");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(caller_node);

  auto caller = makeServiceCaller(caller_node);

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

TEST_F(RosServiceCallerTest, ReleasesRequesterIdentityInflightQuotaWhenRequestBuildFails)
{
  auto caller_node = std::make_shared<rclcpp::Node>("ros_service_caller_build_failure_node");

  auto caller = makeServiceCaller(caller_node);

  const auto request = makeSetBoolRequest("/blocked_service", kStandardRequestTimeoutMs);
  saturateInflightQuota(caller, "requester-1", request, kMaxInflightPerRequester - 1);

  ServiceCallRequest malformed_request = request;
  malformed_request.payload = rclcpp::SerializedMessage();
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

  auto caller = makeServiceCaller(caller_node);

  const auto holding_request = makeSetBoolRequest("/blocked_response_release", kStandardRequestTimeoutMs);
  saturateInflightQuota(caller, "requester-1", holding_request, kMaxInflightPerRequester - 1);

  auto settled_future = caller.call("requester-1", makeSetBoolRequest("/release_response", kResponseSettleTimeoutMs));

  ASSERT_TRUE(waitForFutureReady(executor, settled_future));
  (void)settled_future.get();

  expectReleasedInflightSlot(caller, "requester-1", holding_request);

  caller.shutdown();
}

TEST_F(RosServiceCallerTest, ReleasesRequesterIdentityInflightQuotaWhenCallTimesOut)
{
  auto caller_node = std::make_shared<rclcpp::Node>("ros_service_caller_timeout_release_node");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(caller_node);

  auto caller = makeServiceCaller(caller_node);

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
  auto first_request_observed = std::make_shared<std::atomic<bool>>(false);
  auto second_request_observed = std::make_shared<std::atomic<bool>>(false);

  auto service = server_node->create_service<std_srvs::srv::SetBool>(
    "/late_timeout_drop",
    [first_request_started,
     release_first_response_future,
     second_request_started,
     release_second_response_future,
     first_request_observed,
     second_request_observed](
      const std_srvs::srv::SetBool::Request::SharedPtr request, std_srvs::srv::SetBool::Response::SharedPtr response) {
      if (request->data) {
        bool expected = false;
        if (first_request_observed->compare_exchange_strong(expected, true, std::memory_order_relaxed)) {
          first_request_started->set_value();
        }
        release_first_response_future.wait();
      } else {
        bool expected = false;
        if (second_request_observed->compare_exchange_strong(expected, true, std::memory_order_relaxed)) {
          second_request_started->set_value();
        }
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

  auto caller = makeServiceCaller(caller_node);

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

  release_first_callback();

  ASSERT_EQ(second_request_started_future.wait_for(kShutdownCoordinationTimeout), std::future_status::ready);
  EXPECT_EQ(second_future.wait_for(std::chrono::milliseconds(200)), std::future_status::timeout);

  release_second_callback();

  ASSERT_TRUE(
    test_support::waitUntil(
      [&]() { return second_future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready; },
      std::chrono::seconds(3)));
  const auto second_result = second_future.get();
  const auto second_response = deserializeMessage<std_srvs::srv::SetBool::Response>(second_result.payload);
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

TEST_F(RosServiceCallerTest, CancelForRequesterOnlySettlesMatchingCalls)
{
  auto caller_node = std::make_shared<rclcpp::Node>("ros_service_caller_scoped_disconnect_node");

  auto caller = makeServiceCaller(caller_node);

  const auto request = makeSetBoolRequest("/scoped_disconnect_release", kStandardRequestTimeoutMs);

  std::vector<std::future<RosServiceCaller::Response>> requester_one_futures;
  for (int i = 0; i < kMaxInflightPerRequester; ++i) {
    requester_one_futures.push_back(caller.call("requester-1", request));
  }
  auto requester_two_future = caller.call("requester-2", request);

  caller.cancelForRequester("requester-1");

  for (auto & requester_one_future : requester_one_futures) {
    EXPECT_EQ(expectRuntimeErrorMessage(requester_one_future), "Requester identity disconnected.");
  }

  saturateInflightQuota(caller, "requester-1", request);
  auto requester_one_overflow_future = caller.call("requester-1", request);
  EXPECT_EQ(expectRuntimeErrorMessage(requester_one_overflow_future), "Requester identity service call limit reached.");

  saturateInflightQuota(caller, "requester-2", request, kMaxInflightPerRequester - 1);
  auto requester_two_overflow_future = caller.call("requester-2", request);
  EXPECT_EQ(expectRuntimeErrorMessage(requester_two_overflow_future), "Requester identity service call limit reached.");

  caller.shutdown();
}

TEST_F(RosServiceCallerTest, RejectsEmptyRequesterIdentity)
{
  auto caller_node = std::make_shared<rclcpp::Node>("ros_service_caller_empty_requester_node");

  auto caller = makeServiceCaller(caller_node);

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

  auto caller = makeServiceCaller(caller_node);

  auto future =
    caller.call("requester-1", makeSetBoolRequest("/resolve_test", kResponseSettleTimeoutMs, std::nullopt, false));

  ASSERT_TRUE(waitForFutureReady(executor, future));

  const RosServiceCaller::Response result = future.get();
  EXPECT_EQ(result.interface_type, setBoolServiceType());
  const auto response = deserializeMessage<std_srvs::srv::SetBool::Response>(result.payload);
  EXPECT_TRUE(response.success);

  caller.shutdown();
}

TEST_F(RosServiceCallerTest, RejectsUnresolvableServiceType)
{
  auto caller_node = std::make_shared<rclcpp::Node>("ros_service_caller_unresolvable_node");

  auto caller = makeServiceCaller(caller_node);

  ServiceCallRequest request = makeSetBoolRequest("/no_such_service", 100, std::nullopt, false);

  auto future = caller.call("requester-1", request);

  EXPECT_EQ(expectInvalidArgumentMessage(future), "No ROS types found for service '/no_such_service'.");

  caller.shutdown();
}

TEST_F(RosServiceCallerTest, RejectsCallAfterShutdown)
{
  auto caller_node = std::make_shared<rclcpp::Node>("ros_service_caller_post_shutdown_node");

  auto caller = makeServiceCaller(caller_node);

  caller.shutdown();

  auto future = caller.call("requester-1", makeSetBoolRequest("/any_service", 100));

  EXPECT_EQ(expectRuntimeErrorMessage(future), "Service caller is shut down.");
}

}  // namespace

}  // namespace livekit_ros2_bridge
