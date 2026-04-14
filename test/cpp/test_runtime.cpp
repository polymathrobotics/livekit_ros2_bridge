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
#include <cstdlib>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "fake_room_connection.hpp"
#include "gtest/gtest.h"
#include "nlohmann/json.hpp"
#include "rclcpp/serialization.hpp"
#include "ros_test_support.hpp"
#include "runtime.hpp"
#include "runtime_config.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "wire/cdr.hpp"
#include "wire/protocol.hpp"

namespace livekit_ros2_bridge
{

namespace
{
using test_support::ScopedRclcppInit;
using test_support::spinUntil;
using test_support::waitForTopicType;
using test_support::waitUntil;

constexpr auto kHealthyConnectionObservationWindow = std::chrono::milliseconds(1200);
constexpr auto kWatchdogObservationWindow = std::chrono::seconds(2);
constexpr auto kRuntimeTestPollInterval = std::chrono::milliseconds(20);
constexpr int kRuntimeScenarioCompleted = EXIT_SUCCESS;
constexpr int kRuntimeScenarioTimedOutWithoutWatchdog = 64;

std::string nextNodeName(const std::string & prefix)
{
  static std::atomic<size_t> next_suffix{0};
  return prefix + "_" + std::to_string(next_suffix.fetch_add(1));
}

const std::vector<std::string> & expectedRpcMethods()
{
  static const std::vector<std::string> methods{
    wire::protocol::kRpcServiceCall,
    wire::protocol::kRpcInterfacesGet,
    wire::protocol::kRpcServicesList,
    wire::protocol::kRpcTopicsList,
  };
  return methods;
}

std::vector<std::string> expectedShutdownEventLog()
{
  std::vector<std::string> event_log;
  event_log.reserve(expectedRpcMethods().size() + 1U);
  for (const auto & method : expectedRpcMethods()) {
    event_log.push_back("unregister:" + method);
  }
  event_log.emplace_back("stop");
  return event_log;
}

rclcpp::NodeOptions makeBaseOptions()
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("livekit.url", "ws://test:7880");
  options.append_parameter_override("livekit.room", "robot-room");
  return options;
}

rclcpp::NodeOptions makeStaticTokenOptions()
{
  auto options = makeBaseOptions();
  options.append_parameter_override("livekit.token", "test_token");
  return options;
}

template <typename MessageT>
std::vector<std::uint8_t> serializeMessage(const MessageT & message)
{
  rclcpp::Serialization<MessageT> serialization;
  rclcpp::SerializedMessage serialized;
  serialization.serialize_message(&message, &serialized);
  const auto & rcl_msg = serialized.get_rcl_serialized_message();
  return std::vector<std::uint8_t>(rcl_msg.buffer, rcl_msg.buffer + rcl_msg.buffer_length);
}

nlohmann::json extractSinglePublishedStatusEnvelope(
  const FakeRoomConnectionState & state, const std::string & requester_identity)
{
  if (state.published_outgoing_packets.size() != 1U) {
    ADD_FAILURE() << "Expected one published status response, got " << state.published_outgoing_packets.size();
    return nlohmann::json::object();
  }

  const auto & packet = state.published_outgoing_packets.front();
  EXPECT_EQ(packet.topic, wire::protocol::kControlSubscriptionsStatus);
  EXPECT_EQ(packet.recipient_identities, (std::vector<std::string>{requester_identity}));
  return nlohmann::json::parse(packet.payload.begin(), packet.payload.end());
}

void spinExecutorFor(rclcpp::executors::SingleThreadedExecutor & executor, std::chrono::milliseconds duration)
{
  const auto deadline = std::chrono::steady_clock::now() + duration;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(kRuntimeTestPollInterval);
  }
}

struct RuntimeHarness
{
  std::shared_ptr<rclcpp::Node> node;
  FakeRoomConnection * fake_room_connection = nullptr;
  std::shared_ptr<FakeRoomConnectionState> state;
  std::unique_ptr<Runtime> runtime;
};

template <typename ConfigureConnectionT>
RuntimeHarness makeRuntimeHarness(const rclcpp::NodeOptions & options, ConfigureConnectionT configure_room_connection)
{
  RuntimeHarness harness;
  harness.node = std::make_shared<rclcpp::Node>(nextNodeName("runtime_test_node"), options);

  auto room_connection = std::make_unique<FakeRoomConnection>();
  harness.fake_room_connection = room_connection.get();
  harness.state = room_connection->state;
  configure_room_connection(*room_connection);

  RuntimeConfig config = loadRuntimeConfig(harness.node->get_node_parameters_interface());
  harness.runtime = std::make_unique<Runtime>(*harness.node, std::move(room_connection), std::move(config));
  return harness;
}

RuntimeHarness makeRuntimeHarness(const rclcpp::NodeOptions & options)
{
  return makeRuntimeHarness(options, [](FakeRoomConnection &) {});
}

[[noreturn]] void runRuntimeScenario(
  const rclcpp::NodeOptions & options,
  const std::function<void(RuntimeHarness &)> & configure_runtime,
  std::chrono::milliseconds observation_window,
  int exit_code_after_observation)
{
  ScopedRclcppInit rclcpp_init;
  auto harness = makeRuntimeHarness(options);
  configure_runtime(harness);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node);
  spinExecutorFor(executor, observation_window);
  std::_Exit(exit_code_after_observation);
}

}  // namespace

class RuntimeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    static ScopedRclcppInit rclcpp_init;
  }
};

TEST_F(RuntimeTest, RegistersRpcMethodsDuringStartup)
{
  auto harness = makeRuntimeHarness(makeStaticTokenOptions());

  EXPECT_TRUE(harness.state->started);
  EXPECT_EQ(harness.state->registered_rpc_methods, expectedRpcMethods());
  EXPECT_EQ(harness.state->rpc_handlers.size(), expectedRpcMethods().size());
  EXPECT_TRUE(static_cast<bool>(harness.state->callbacks.on_connected));
  EXPECT_TRUE(static_cast<bool>(harness.state->callbacks.on_reconnect_requested));
  EXPECT_TRUE(static_cast<bool>(harness.state->callbacks.on_connection_reset));
  EXPECT_TRUE(static_cast<bool>(harness.state->callbacks.on_remote_participant_disconnected));
  EXPECT_TRUE(static_cast<bool>(harness.state->callbacks.on_incoming_packet_received));
}

TEST_F(RuntimeTest, WatchdogExitsWhenInitialConnectNeverSucceeds)
{
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";

  EXPECT_EXIT(
    {
      auto options = makeStaticTokenOptions();
      options.append_parameter_override("health.watchdog.recovery_timeout_seconds", 0.0);
      runRuntimeScenario(
        options, [](RuntimeHarness &) {}, kWatchdogObservationWindow, kRuntimeScenarioTimedOutWithoutWatchdog);
    },
    ::testing::ExitedWithCode(EXIT_FAILURE),
    ".*");
}

TEST_F(RuntimeTest, WatchdogDoesNotExitAfterInitialConnectSucceeds)
{
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";

  EXPECT_EXIT(
    {
      auto options = makeStaticTokenOptions();
      options.append_parameter_override("health.watchdog.recovery_timeout_seconds", 0.3);
      runRuntimeScenario(
        options,
        [](RuntimeHarness & harness) { harness.fake_room_connection->emitConnected(); },
        kHealthyConnectionObservationWindow,
        kRuntimeScenarioCompleted);
    },
    ::testing::ExitedWithCode(kRuntimeScenarioCompleted),
    ".*");
}

TEST_F(RuntimeTest, WatchdogExitsWhenRecoveryTimeoutExpires)
{
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";

  EXPECT_EXIT(
    {
      auto options = makeStaticTokenOptions();
      options.append_parameter_override("health.watchdog.recovery_timeout_seconds", 0.0);
      runRuntimeScenario(
        options,
        [](RuntimeHarness & harness) {
          harness.fake_room_connection->emitConnected();
          harness.fake_room_connection->emitReconnectRequested("room_disconnected");
        },
        kWatchdogObservationWindow,
        kRuntimeScenarioTimedOutWithoutWatchdog);
    },
    ::testing::ExitedWithCode(EXIT_FAILURE),
    ".*");
}

TEST_F(RuntimeTest, WatchdogClearsRecoveryTimeoutAfterRecovery)
{
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";

  EXPECT_EXIT(
    {
      auto options = makeStaticTokenOptions();
      options.append_parameter_override("health.watchdog.recovery_timeout_seconds", 0.3);
      runRuntimeScenario(
        options,
        [](RuntimeHarness & harness) {
          harness.fake_room_connection->emitConnected();
          harness.fake_room_connection->emitReconnectRequested("room_disconnected");
          harness.fake_room_connection->emitConnected();
        },
        kHealthyConnectionObservationWindow,
        kRuntimeScenarioCompleted);
    },
    ::testing::ExitedWithCode(kRuntimeScenarioCompleted),
    ".*");
}

TEST_F(RuntimeTest, WatchdogDisabledNeverExitsForDisconnectedConnection)
{
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";

  EXPECT_EXIT(
    {
      auto options = makeStaticTokenOptions();
      options.append_parameter_override("health.watchdog.enabled", false);
      options.append_parameter_override("health.watchdog.recovery_timeout_seconds", 0.0);
      runRuntimeScenario(
        options, [](RuntimeHarness &) {}, kHealthyConnectionObservationWindow, kRuntimeScenarioCompleted);
    },
    ::testing::ExitedWithCode(kRuntimeScenarioCompleted),
    ".*");
}

TEST_F(RuntimeTest, ShutdownPreventsPendingWatchdogExit)
{
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";

  EXPECT_EXIT(
    {
      auto options = makeStaticTokenOptions();
      options.append_parameter_override("health.watchdog.recovery_timeout_seconds", 0.0);
      runRuntimeScenario(
        options,
        [](RuntimeHarness & harness) { harness.runtime.reset(); },
        kHealthyConnectionObservationWindow,
        kRuntimeScenarioCompleted);
    },
    ::testing::ExitedWithCode(kRuntimeScenarioCompleted),
    ".*");
}

TEST_F(RuntimeTest, StartupFailsWhenRequiredRpcRegistrationFails)
{
  auto node = std::make_shared<rclcpp::Node>(nextNodeName("runtime_test_node"), makeStaticTokenOptions());
  RuntimeConfig config = loadRuntimeConfig(node->get_node_parameters_interface());
  auto room_connection = std::make_unique<FakeRoomConnection>();
  auto state = room_connection->state;
  state->rejected_rpc_methods = {wire::protocol::kRpcInterfacesGet};

  try {
    Runtime runtime(*node, std::move(room_connection), std::move(config));
    FAIL() << "Expected std::runtime_error";
  } catch (const std::runtime_error & exc) {
    EXPECT_STREQ(exc.what(), "Failed to register required RPC methods");
  }

  EXPECT_FALSE(state->started);
  EXPECT_TRUE(state->stopped);
  EXPECT_EQ(state->registered_rpc_methods, expectedRpcMethods());
  EXPECT_EQ(state->unregistered_rpc_methods, expectedRpcMethods());
  EXPECT_TRUE(state->rpc_handlers.empty());
}

TEST_F(RuntimeTest, DestructionRunsSingleOrderedTeardown)
{
  auto harness = makeRuntimeHarness(makeStaticTokenOptions());

  harness.runtime.reset();

  EXPECT_EQ(harness.state->event_log, expectedShutdownEventLog());
}

TEST_F(RuntimeTest, ExplicitShutdownIsIdempotentAcrossRepeatedCallsAndDestruction)
{
  const auto expected_event_log = expectedShutdownEventLog();
  auto harness = makeRuntimeHarness(makeStaticTokenOptions());

  harness.runtime->shutdown();

  EXPECT_EQ(harness.state->event_log, expected_event_log);

  harness.runtime->shutdown();
  harness.runtime.reset();

  EXPECT_EQ(harness.state->event_log, expected_event_log);
}

TEST_F(RuntimeTest, IncomingPacketPublishesAfterExecutorDispatch)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("access.rules.publish.allow", std::vector<std::string>{"/battery/cmd"});

  auto harness = makeRuntimeHarness(options);

  auto observer = std::make_shared<rclcpp::Node>(nextNodeName("runtime_publish_observer"));
  std::optional<sensor_msgs::msg::BatteryState> received_message;
  [[maybe_unused]] auto subscription = observer->create_subscription<sensor_msgs::msg::BatteryState>(
    "/battery/cmd", rclcpp::QoS(10), [&received_message](const sensor_msgs::msg::BatteryState & message) {
      received_message = message;
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node);
  executor.add_node(observer);
  ASSERT_TRUE(waitForTopicType(executor, harness.node, "/battery/cmd", "sensor_msgs/msg/BatteryState"));

  sensor_msgs::msg::BatteryState expected_message;
  expected_message.voltage = 48.5F;
  expected_message.percentage = 0.75F;

  const std::string payload =
    nlohmann::json{
      {"topic", "/battery/cmd"},
      {"interface_type", "sensor_msgs/msg/BatteryState"},
      {"message", wire::cdr::serialize(serializeMessage(expected_message))},
    }
      .dump();
  harness.fake_room_connection->emitIncomingPacket(
    IncomingPacket{
      std::vector<std::uint8_t>(payload.begin(), payload.end()),
      wire::protocol::kControlTopicPublish,
      "participant-1",
    });

  EXPECT_FALSE(received_message.has_value());
  ASSERT_TRUE(spinUntil(executor, [&received_message]() { return received_message.has_value(); }));
  EXPECT_NEAR(received_message->voltage, expected_message.voltage, 1e-6F);
  EXPECT_NEAR(received_message->percentage, expected_message.percentage, 1e-6F);
}

TEST_F(RuntimeTest, ParticipantRefreshRepublishesDataTrackOnNextHeartbeat)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("access.rules.subscribe.allow", std::vector<std::string>{"/battery"});

  auto harness = makeRuntimeHarness(options);

  auto observer = std::make_shared<rclcpp::Node>(nextNodeName("participant_refresh_observer"));
  [[maybe_unused]] auto publisher =
    observer->create_publisher<sensor_msgs::msg::BatteryState>("/battery", rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node);
  executor.add_node(observer);
  ASSERT_TRUE(waitForTopicType(executor, harness.node, "/battery", "sensor_msgs/msg/BatteryState"));

  const std::string heartbeat =
    R"({"subscriptions":[{"kind":"topic","name":"/battery","delivery_preferences":{"interval_ms":1000}}]})";
  harness.fake_room_connection->emitIncomingPacket(
    IncomingPacket{
      std::vector<std::uint8_t>(heartbeat.begin(), heartbeat.end()),
      wire::protocol::kControlSubscriptionsHeartbeat,
      "participant-1",
    });

  ASSERT_TRUE(spinUntil(executor, [&]() { return harness.state->published_data_track_names.size() == 1U; }));

  harness.fake_room_connection->emitParticipantDisconnected("participant-1");
  harness.fake_room_connection->emitIncomingPacket(
    IncomingPacket{
      std::vector<std::uint8_t>(heartbeat.begin(), heartbeat.end()),
      wire::protocol::kControlSubscriptionsHeartbeat,
      "participant-1",
    });

  ASSERT_TRUE(spinUntil(executor, [&]() { return harness.state->published_data_track_names.size() == 2U; }));
}

TEST_F(RuntimeTest, ConnectionResetClearsSubscriptionsAndAllowsHeartbeatToRecreateDataTrack)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("access.rules.subscribe.allow", std::vector<std::string>{"/battery"});

  auto harness = makeRuntimeHarness(options);

  auto observer = std::make_shared<rclcpp::Node>(nextNodeName("connection_reset_observer"));
  [[maybe_unused]] auto publisher =
    observer->create_publisher<sensor_msgs::msg::BatteryState>("/battery", rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node);
  executor.add_node(observer);
  ASSERT_TRUE(waitForTopicType(executor, harness.node, "/battery", "sensor_msgs/msg/BatteryState"));

  const std::string heartbeat =
    R"({"subscriptions":[{"kind":"topic","name":"/battery","delivery_preferences":{"interval_ms":1000}}]})";
  harness.fake_room_connection->emitIncomingPacket(
    IncomingPacket{
      std::vector<std::uint8_t>(heartbeat.begin(), heartbeat.end()),
      wire::protocol::kControlSubscriptionsHeartbeat,
      "participant-1",
    });

  ASSERT_TRUE(spinUntil(executor, [&]() { return harness.state->published_data_track_names.size() == 1U; }));

  harness.fake_room_connection->emitConnectionReset();

  ASSERT_TRUE(spinUntil(executor, [&]() { return harness.state->unpublished_data_track_names.size() == 1U; }));
  EXPECT_EQ(harness.state->unpublished_data_track_names.front(), harness.state->published_data_track_names.front());

  harness.fake_room_connection->emitIncomingPacket(
    IncomingPacket{
      std::vector<std::uint8_t>(heartbeat.begin(), heartbeat.end()),
      wire::protocol::kControlSubscriptionsHeartbeat,
      "participant-1",
    });

  ASSERT_TRUE(spinUntil(executor, [&]() { return harness.state->published_data_track_names.size() == 2U; }));
}

TEST_F(RuntimeTest, VideoHeartbeatPublishesTrackNameAndInProcessVideoTrack)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("access.rules.subscribe.allow", std::vector<std::string>{"/camera/front"});

  auto harness = makeRuntimeHarness(options);

  auto observer = std::make_shared<rclcpp::Node>(nextNodeName("runtime_video_watchdog_observer"));
  auto publisher = observer->create_publisher<sensor_msgs::msg::Image>("/camera/front", rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node);
  executor.add_node(observer);
  ASSERT_TRUE(waitForTopicType(executor, harness.node, "/camera/front", "sensor_msgs/msg/Image"));

  const std::string heartbeat = R"({"subscriptions":[{"kind":"topic","name":"/camera/front"}]})";
  harness.fake_room_connection->emitIncomingPacket(
    IncomingPacket{
      std::vector<std::uint8_t>(heartbeat.begin(), heartbeat.end()),
      wire::protocol::kControlSubscriptionsHeartbeat,
      "participant-1",
    });

  ASSERT_TRUE(spinUntil(executor, [&]() { return harness.state->published_outgoing_packets.size() == 1U; }));
  const auto status = extractSinglePublishedStatusEnvelope(*harness.state, "participant-1");
  ASSERT_TRUE(status.contains("subscriptions"));
  ASSERT_EQ(status["subscriptions"].size(), 1U);
  const auto & delivery = status["subscriptions"][0]["delivery"];
  EXPECT_EQ(delivery["kind"], "video");
  EXPECT_FALSE(delivery["track_name"].get<std::string>().empty());

  sensor_msgs::msg::Image image;
  image.header.stamp.sec = 1;
  image.width = 2;
  image.height = 2;
  image.step = 6;
  image.encoding = "rgb8";
  image.data = {
    255,
    0,
    0,
    0,
    255,
    0,
    0,
    0,
    255,
    255,
    255,
    255,
  };
  publisher->publish(image);

  ASSERT_TRUE(spinUntil(executor, [&]() { return harness.state->published_video_track_names.size() == 1U; }));
}

TEST_F(RuntimeTest, StopTimeCallbacksDoNotSubmitNewIngressAfterShutdownStarts)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("access.rules.subscribe.allow", std::vector<std::string>{"/battery"});

  const std::string heartbeat =
    R"({"subscriptions":[{"kind":"topic","name":"/battery","delivery_preferences":{"interval_ms":125}}]})";
  auto harness = makeRuntimeHarness(options, [&heartbeat](FakeRoomConnection & room_connection) {
    room_connection.state->stop_hook = [heartbeat](const RoomEventCallbacks & callbacks) {
      if (callbacks.on_connection_reset) {
        callbacks.on_connection_reset();
      }

      if (callbacks.on_remote_participant_disconnected) {
        callbacks.on_remote_participant_disconnected("participant-1");
      }

      if (callbacks.on_incoming_packet_received) {
        callbacks.on_incoming_packet_received(
          IncomingPacket{
            std::vector<std::uint8_t>(heartbeat.begin(), heartbeat.end()),
            wire::protocol::kControlSubscriptionsHeartbeat,
            "participant-1",
          });
      }
    };
  });

  auto observer = std::make_shared<rclcpp::Node>(nextNodeName("shutdown_stop_observer"));
  [[maybe_unused]] auto publisher =
    observer->create_publisher<sensor_msgs::msg::BatteryState>("/battery", rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node);
  executor.add_node(observer);
  ASSERT_TRUE(waitForTopicType(executor, harness.node, "/battery", "sensor_msgs/msg/BatteryState"));

  std::thread executor_thread([&executor]() { executor.spin(); });

  std::promise<void> destroy_finished_promise;
  auto destroy_finished = destroy_finished_promise.get_future();
  std::thread destroy_thread([&harness, &destroy_finished_promise]() {
    harness.runtime.reset();
    destroy_finished_promise.set_value();
  });

  EXPECT_EQ(destroy_finished.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  destroy_thread.join();
  executor.cancel();
  executor_thread.join();

  EXPECT_TRUE(harness.state->published_outgoing_packets.empty());
  EXPECT_TRUE(harness.state->published_data_track_names.empty());
}

TEST_F(RuntimeTest, ShutdownWaitsForRunningPublishTrackBeforeClearingSubscriptions)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("access.rules.subscribe.allow", std::vector<std::string>{"/battery"});

  std::promise<void> publish_started_promise;
  auto publish_started = publish_started_promise.get_future();
  std::promise<void> release_publish_promise;
  auto release_publish = release_publish_promise.get_future().share();
  std::atomic<bool> publish_started_once{false};
  auto harness = makeRuntimeHarness(
    options, [&publish_started_promise, &release_publish, &publish_started_once](FakeRoomConnection & room_connection) {
      room_connection.state->publish_data_track_handler =
        [&publish_started_promise, &release_publish, &publish_started_once](const std::string &) {
          if (!publish_started_once.exchange(true)) {
            publish_started_promise.set_value();
          }
          release_publish.wait();
          return std::shared_ptr<livekit::LocalDataTrack>{};
        };
    });

  auto observer = std::make_shared<rclcpp::Node>(nextNodeName("shutdown_publish_track_observer"));
  auto publisher = observer->create_publisher<sensor_msgs::msg::BatteryState>("/battery", rclcpp::QoS(10));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node);
  executor.add_node(observer);
  ASSERT_TRUE(waitForTopicType(executor, harness.node, "/battery", "sensor_msgs/msg/BatteryState"));

  std::thread executor_thread([&executor]() { executor.spin(); });

  const std::string heartbeat =
    R"({"subscriptions":[{"kind":"topic","name":"/battery","delivery_preferences":{"interval_ms":125}}]})";
  harness.state->callbacks.on_incoming_packet_received(
    IncomingPacket{
      std::vector<std::uint8_t>(heartbeat.begin(), heartbeat.end()),
      wire::protocol::kControlSubscriptionsHeartbeat,
      "participant-1",
    });

  const bool publish_started_ready = publish_started.wait_for(std::chrono::seconds(1)) == std::future_status::ready;
  EXPECT_TRUE(publish_started_ready);
  if (!publish_started_ready) {
    release_publish_promise.set_value();
    executor.cancel();
    executor_thread.join();
    return;
  }

  // Graph counts can transiently exceed one across distros during discovery and
  // teardown. The shutdown contract here is only that the subscription stays
  // present until the blocked publish finishes.
  const bool subscription_ready = waitUntil([&publisher]() { return publisher->get_subscription_count() >= 1U; });
  EXPECT_TRUE(subscription_ready);
  if (!subscription_ready) {
    release_publish_promise.set_value();
    executor.cancel();
    executor_thread.join();
    return;
  }

  std::promise<void> destroy_finished_promise;
  auto destroy_finished = destroy_finished_promise.get_future();
  std::thread destroy_thread([&harness, &destroy_finished_promise]() {
    harness.runtime.reset();
    destroy_finished_promise.set_value();
  });

  EXPECT_EQ(destroy_finished.wait_for(std::chrono::milliseconds(100)), std::future_status::timeout);
  EXPECT_GE(publisher->get_subscription_count(), 1U);

  release_publish_promise.set_value();

  EXPECT_EQ(destroy_finished.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  destroy_thread.join();
  executor.cancel();
  executor_thread.join();

  EXPECT_EQ(harness.state->published_data_track_names.size(), 1U);
}

}  // namespace livekit_ros2_bridge
