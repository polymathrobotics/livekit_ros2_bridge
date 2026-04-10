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
#include <filesystem>
#include <fstream>
#include <functional>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "fake_room_session.hpp"
#include "gtest/gtest.h"
#include "nlohmann/json.hpp"
#include "payloads/cdr_payload.hpp"
#include "protocol.hpp"
#include "rclcpp/serialization.hpp"
#include "ros_test_support.hpp"
#include "runtime.hpp"
#include "runtime_config.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace livekit_ros2_bridge
{

namespace
{
using test_support::ScopedRclcppInit;
using test_support::spinUntil;
using test_support::waitForTopicType;
using test_support::waitUntil;

constexpr auto kHealthyPublisherObservationWindow = std::chrono::milliseconds(1200);
constexpr auto kRuntimeTestPollInterval = std::chrono::milliseconds(20);

std::string nextNodeName(const std::string & prefix)
{
  static std::atomic<size_t> next_suffix{0};
  return prefix + "_" + std::to_string(next_suffix.fetch_add(1));
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
  const FakeRoomSessionState & state, const std::string & requester_identity)
{
  if (state.published_outgoing_control_packets.size() != 1U) {
    ADD_FAILURE() << "Expected one published status response, got " << state.published_outgoing_control_packets.size();
    return nlohmann::json::object();
  }

  const auto & packet = state.published_outgoing_control_packets.front();
  EXPECT_EQ(packet.control_topic, protocol::kControlSubscriptionsStatus);
  EXPECT_EQ(packet.recipient_identities, (std::vector<std::string>{requester_identity}));
  return nlohmann::json::parse(packet.payload.begin(), packet.payload.end());
}

std::size_t countFileLines(const std::filesystem::path & path)
{
  std::ifstream stream(path);
  std::size_t lines = 0U;
  std::string line;
  while (std::getline(stream, line)) {
    ++lines;
  }
  return lines;
}

class ScopedPathEnvPrepend
{
public:
  explicit ScopedPathEnvPrepend(const std::filesystem::path & prepend_path)
  : previous_path_(std::getenv("PATH") == nullptr ? "" : std::getenv("PATH"))
  {
    const std::string updated_path = prepend_path.string() + ":" + previous_path_;
    if (setenv("PATH", updated_path.c_str(), 1) != 0) {
      throw std::runtime_error("Failed to update PATH for runtime test.");
    }
  }

  ~ScopedPathEnvPrepend()
  {
    if (previous_path_.empty()) {
      unsetenv("PATH");
      return;
    }
    (void)setenv("PATH", previous_path_.c_str(), 1);
  }

private:
  std::string previous_path_;
};

struct RuntimeHarness
{
  std::shared_ptr<rclcpp::Node> node;
  FakeRoomSession * fake_session = nullptr;
  std::shared_ptr<FakeRoomSessionState> state;
  std::unique_ptr<Runtime> runtime;
};

template <typename ConfigureSessionT>
RuntimeHarness makeRuntimeHarness(const rclcpp::NodeOptions & options, ConfigureSessionT configure_session)
{
  RuntimeHarness harness;
  harness.node = std::make_shared<rclcpp::Node>(nextNodeName("runtime_test_node"), options);

  auto session = std::make_unique<FakeRoomSession>();
  harness.fake_session = session.get();
  harness.state = session->state;
  configure_session(*session);

  RuntimeConfig startup_config =
    loadRuntimeConfig(harness.node->get_node_parameters_interface(), harness.node->get_name());
  harness.runtime = std::make_unique<Runtime>(*harness.node, std::move(session), std::move(startup_config));
  return harness;
}

RuntimeHarness makeRuntimeHarness(const rclcpp::NodeOptions & options)
{
  return makeRuntimeHarness(options, [](FakeRoomSession &) {});
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

TEST_F(RuntimeTest, RegistersRpcMethodsOnConnect)
{
  auto harness = makeRuntimeHarness(makeStaticTokenOptions());
  const std::vector<std::string> expected_methods{
    protocol::kRpcServiceCall, protocol::kRpcInterfacesGet, protocol::kRpcServicesList, protocol::kRpcTopicsList};

  ASSERT_NE(harness.runtime, nullptr);
  EXPECT_TRUE(harness.state->started);
  EXPECT_EQ(harness.state->registered_rpc_methods, expected_methods);
  EXPECT_EQ(harness.state->rpc_handlers.size(), expected_methods.size());
  EXPECT_TRUE(static_cast<bool>(harness.state->callbacks.on_session_reset));
  EXPECT_TRUE(static_cast<bool>(harness.state->callbacks.on_participant_disconnected));
  EXPECT_TRUE(static_cast<bool>(harness.state->callbacks.on_incoming_control_packet_received));
}

TEST_F(RuntimeTest, StartupFailsWhenRequiredRpcRegistrationFails)
{
  auto node = std::make_shared<rclcpp::Node>(nextNodeName("runtime_test_node"), makeStaticTokenOptions());
  auto session = std::make_unique<FakeRoomSession>();
  auto state = session->state;
  state->rejected_rpc_methods = {protocol::kRpcInterfacesGet};
  RuntimeConfig startup_config = loadRuntimeConfig(node->get_node_parameters_interface(), node->get_name());

  try {
    Runtime runtime(*node, std::move(session), std::move(startup_config));
    FAIL() << "Expected std::runtime_error";
  } catch (const std::runtime_error & exc) {
    EXPECT_STREQ(exc.what(), "Failed to register required RPC methods");
  }

  const std::vector<std::string> expected_methods{
    protocol::kRpcServiceCall, protocol::kRpcInterfacesGet, protocol::kRpcServicesList, protocol::kRpcTopicsList};

  EXPECT_TRUE(state->started);
  EXPECT_TRUE(state->stopped);
  EXPECT_EQ(state->registered_rpc_methods, expected_methods);
  EXPECT_EQ(state->unregistered_rpc_methods, expected_methods);
  EXPECT_TRUE(state->rpc_handlers.empty());
}

TEST_F(RuntimeTest, UnregistersRpcMethodsBeforeStop)
{
  auto harness = makeRuntimeHarness(makeStaticTokenOptions());
  ASSERT_NE(harness.runtime, nullptr);

  harness.runtime.reset();

  ASSERT_EQ(harness.state->event_log.size(), 5U);
  EXPECT_EQ(harness.state->event_log[0], "unregister:" + std::string(protocol::kRpcServiceCall));
  EXPECT_EQ(harness.state->event_log[1], "unregister:" + std::string(protocol::kRpcInterfacesGet));
  EXPECT_EQ(harness.state->event_log[2], "unregister:" + std::string(protocol::kRpcServicesList));
  EXPECT_EQ(harness.state->event_log[3], "unregister:" + std::string(protocol::kRpcTopicsList));
  EXPECT_EQ(harness.state->event_log[4], "stop");
  EXPECT_TRUE(harness.state->stopped);
  EXPECT_EQ(
    harness.state->unregistered_rpc_methods,
    (std::vector<std::string>{
      protocol::kRpcServiceCall, protocol::kRpcInterfacesGet, protocol::kRpcServicesList, protocol::kRpcTopicsList}));
}

TEST_F(RuntimeTest, IncomingControlPacketPublishesAfterExecutorDispatch)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("access.rules.publish.allow", std::vector<std::string>{"/battery/cmd"});

  auto harness = makeRuntimeHarness(options);
  ASSERT_NE(harness.runtime, nullptr);

  auto observer = std::make_shared<rclcpp::Node>(nextNodeName("runtime_publish_observer"));
  std::optional<sensor_msgs::msg::BatteryState> received_message;
  auto subscription = observer->create_subscription<sensor_msgs::msg::BatteryState>(
    "/battery/cmd", rclcpp::QoS(10), [&received_message](const sensor_msgs::msg::BatteryState & message) {
      received_message = message;
    });
  ASSERT_NE(subscription, nullptr);

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
      {"message", serializeCdrPayload(serializeMessage(expected_message))},
    }
      .dump();
  harness.fake_session->emitIncomingControlPacket(
    IncomingControlPacket{
      std::vector<std::uint8_t>(payload.begin(), payload.end()),
      protocol::kControlTopicPublish,
      "participant-1",
    });

  EXPECT_FALSE(received_message.has_value());
  ASSERT_TRUE(spinUntil(executor, [&received_message]() { return received_message.has_value(); }));
  ASSERT_TRUE(received_message.has_value());
  EXPECT_NEAR(received_message->voltage, expected_message.voltage, 1e-6F);
  EXPECT_NEAR(received_message->percentage, expected_message.percentage, 1e-6F);
  EXPECT_EQ(observer->count_publishers("/battery/cmd"), 1U);
}

TEST_F(RuntimeTest, ParticipantRefreshReplaysPublishedCdrTrackOnNextHeartbeat)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("access.rules.subscribe.allow", std::vector<std::string>{"/battery"});

  auto harness = makeRuntimeHarness(options, [](FakeRoomSession & session) {
    session.state->publish_cdr_track_handler = [](const std::string &) {
      return std::shared_ptr<livekit::LocalDataTrack>{};
    };
  });
  ASSERT_NE(harness.runtime, nullptr);

  auto observer = std::make_shared<rclcpp::Node>(nextNodeName("participant_refresh_observer"));
  auto publisher = observer->create_publisher<sensor_msgs::msg::BatteryState>("/battery", rclcpp::QoS(10));
  ASSERT_NE(publisher, nullptr);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node);
  executor.add_node(observer);
  ASSERT_TRUE(waitForTopicType(executor, harness.node, "/battery", "sensor_msgs/msg/BatteryState"));

  const std::string heartbeat =
    R"({"subscriptions":[{"topic":"/battery","delivery_preferences":{"interval_ms":1000}}]})";
  harness.fake_session->emitIncomingControlPacket(
    IncomingControlPacket{
      std::vector<std::uint8_t>(heartbeat.begin(), heartbeat.end()),
      protocol::kControlSubscriptionsHeartbeat,
      "participant-1",
    });

  ASSERT_TRUE(spinUntil(executor, [&]() { return harness.state->published_cdr_track_names.size() == 1U; }));

  harness.fake_session->emitParticipantDisconnected("participant-1");
  harness.fake_session->emitIncomingControlPacket(
    IncomingControlPacket{
      std::vector<std::uint8_t>(heartbeat.begin(), heartbeat.end()),
      protocol::kControlSubscriptionsHeartbeat,
      "participant-1",
    });

  ASSERT_TRUE(spinUntil(executor, [&]() { return harness.state->published_cdr_track_names.size() == 2U; }));
  EXPECT_EQ(harness.state->published_cdr_track_names[0], harness.state->published_cdr_track_names[1]);
  EXPECT_NE(
    std::find(harness.state->event_log.begin(), harness.state->event_log.end(), "unpublish_cdr_track"),
    harness.state->event_log.end());
}

TEST_F(RuntimeTest, NewParticipantReplaysPublishedCdrTrackOnFirstHeartbeat)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("access.rules.subscribe.allow", std::vector<std::string>{"/battery"});

  auto harness = makeRuntimeHarness(options, [](FakeRoomSession & session) {
    session.state->publish_cdr_track_handler = [](const std::string &) {
      return std::shared_ptr<livekit::LocalDataTrack>{};
    };
  });
  ASSERT_NE(harness.runtime, nullptr);

  auto observer = std::make_shared<rclcpp::Node>(nextNodeName("new_participant_replay_observer"));
  auto publisher = observer->create_publisher<sensor_msgs::msg::BatteryState>("/battery", rclcpp::QoS(10));
  ASSERT_NE(publisher, nullptr);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node);
  executor.add_node(observer);
  ASSERT_TRUE(waitForTopicType(executor, harness.node, "/battery", "sensor_msgs/msg/BatteryState"));

  const std::string heartbeat =
    R"({"subscriptions":[{"topic":"/battery","delivery_preferences":{"interval_ms":1000}}]})";
  harness.fake_session->emitIncomingControlPacket(
    IncomingControlPacket{
      std::vector<std::uint8_t>(heartbeat.begin(), heartbeat.end()),
      protocol::kControlSubscriptionsHeartbeat,
      "participant-1",
    });

  ASSERT_TRUE(spinUntil(executor, [&]() { return harness.state->published_cdr_track_names.size() == 1U; }));

  harness.fake_session->emitIncomingControlPacket(
    IncomingControlPacket{
      std::vector<std::uint8_t>(heartbeat.begin(), heartbeat.end()),
      protocol::kControlSubscriptionsHeartbeat,
      "participant-2",
    });

  ASSERT_TRUE(spinUntil(executor, [&]() { return harness.state->published_cdr_track_names.size() == 2U; }));
  EXPECT_EQ(harness.state->published_cdr_track_names[0], harness.state->published_cdr_track_names[1]);
  EXPECT_NE(
    std::find(harness.state->event_log.begin(), harness.state->event_log.end(), "unpublish_cdr_track"),
    harness.state->event_log.end());
}

TEST_F(RuntimeTest, VideoWatchdogRestartsUnhealthyPublisherWithoutSessionReset)
{
  const auto temp_dir = std::filesystem::temp_directory_path() / nextNodeName("runtime_fake_gstreamer_publisher");
  std::filesystem::remove_all(temp_dir);
  ASSERT_TRUE(std::filesystem::create_directories(temp_dir));
  const auto invocation_log = temp_dir / "invocations.log";
  const auto fake_publisher = temp_dir / "gstreamer-publisher";

  {
    std::ofstream script(fake_publisher);
    ASSERT_TRUE(script.is_open());
    script << "#!/bin/sh\n";
    script << "echo invoked >> '" << invocation_log.string() << "'\n";
    script << "sleep 3600\n";
  }
  std::filesystem::permissions(
    fake_publisher,
    std::filesystem::perms::owner_exec | std::filesystem::perms::owner_read | std::filesystem::perms::owner_write,
    std::filesystem::perm_options::replace);
  ScopedPathEnvPrepend scoped_path_override(temp_dir);

  auto options = makeBaseOptions();
  options.append_parameter_override("livekit.api_key", "test-api-key");
  options.append_parameter_override("livekit.api_secret", "test-api-secret");
  options.append_parameter_override("access.rules.subscribe.allow", std::vector<std::string>{"/camera/front"});

  auto node = std::make_shared<rclcpp::Node>(nextNodeName("runtime_video_watchdog_node"), options);
  auto session = std::make_unique<FakeRoomSession>();
  auto * fake_session = session.get();
  auto state = session->state;

  RuntimeConfig startup_config = loadRuntimeConfig(node->get_node_parameters_interface(), node->get_name());
  ASSERT_TRUE(startup_config.video_sidecar_config.has_value());
  startup_config.video_sidecar_config->health_check_startup_grace = std::chrono::milliseconds(100);
  startup_config.video_sidecar_config->unhealthy_restart_threshold = 2U;
  auto runtime = std::make_unique<Runtime>(*node, std::move(session), std::move(startup_config));

  auto observer = std::make_shared<rclcpp::Node>(nextNodeName("runtime_video_watchdog_observer"));
  auto publisher = observer->create_publisher<sensor_msgs::msg::Image>("/camera/front", rclcpp::QoS(10));
  ASSERT_NE(publisher, nullptr);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(observer);
  ASSERT_TRUE(waitForTopicType(executor, node, "/camera/front", "sensor_msgs/msg/Image"));

  const std::string heartbeat = R"({"subscriptions":[{"topic":"/camera/front"}]})";
  fake_session->emitIncomingControlPacket(
    IncomingControlPacket{
      std::vector<std::uint8_t>(heartbeat.begin(), heartbeat.end()),
      protocol::kControlSubscriptionsHeartbeat,
      "participant-1",
    });

  ASSERT_TRUE(spinUntil(executor, [&]() { return state->published_outgoing_control_packets.size() == 1U; }));
  ASSERT_TRUE(waitUntil([&invocation_log]() { return countFileLines(invocation_log) == 1U; }));

  const auto status = extractSinglePublishedStatusEnvelope(*state, "participant-1");
  ASSERT_TRUE(status.contains("streams"));
  ASSERT_EQ(status["streams"].size(), 1U);
  const auto & delivery = status["streams"][0]["delivery"];
  const std::string publisher_identity = delivery["publisher_identity"].get<std::string>();
  fake_session->setVideoPublisherHealthy(publisher_identity, true);

  const auto stable_deadline = std::chrono::steady_clock::now() + kHealthyPublisherObservationWindow;
  while (std::chrono::steady_clock::now() < stable_deadline) {
    executor.spin_some();
    EXPECT_EQ(countFileLines(invocation_log), 1U);
    std::this_thread::sleep_for(kRuntimeTestPollInterval);
  }

  fake_session->setVideoPublisherHealthy(publisher_identity, false);
  ASSERT_TRUE(
    spinUntil(executor, [&invocation_log]() { return countFileLines(invocation_log) == 2U; }, std::chrono::seconds(5)));

  runtime.reset();
  std::filesystem::remove_all(temp_dir);
}

TEST_F(RuntimeTest, StopTimeCallbacksDoNotSubmitNewIngressAfterShutdownStarts)
{
  auto options = makeStaticTokenOptions();
  options.append_parameter_override("access.rules.subscribe.allow", std::vector<std::string>{"/battery"});

  const std::string heartbeat =
    R"({"subscriptions":[{"topic":"/battery","delivery_preferences":{"interval_ms":125}}]})";
  auto harness = makeRuntimeHarness(options, [&heartbeat](FakeRoomSession & session) {
    session.state->stop_hook = [heartbeat](const RoomSessionCallbacks & callbacks) {
      if (callbacks.on_session_reset) {
        callbacks.on_session_reset();
      }
      if (callbacks.on_participant_disconnected) {
        callbacks.on_participant_disconnected("participant-1");
      }
      if (callbacks.on_incoming_control_packet_received) {
        callbacks.on_incoming_control_packet_received(
          IncomingControlPacket{
            std::vector<std::uint8_t>(heartbeat.begin(), heartbeat.end()),
            protocol::kControlSubscriptionsHeartbeat,
            "participant-1",
          });
      }
    };
  });
  ASSERT_NE(harness.runtime, nullptr);

  auto observer = std::make_shared<rclcpp::Node>(nextNodeName("shutdown_stop_observer"));
  auto publisher = observer->create_publisher<sensor_msgs::msg::BatteryState>("/battery", rclcpp::QoS(10));
  ASSERT_NE(publisher, nullptr);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node);
  executor.add_node(observer);

  const bool topic_ready = waitForTopicType(executor, harness.node, "/battery", "sensor_msgs/msg/BatteryState");
  EXPECT_TRUE(topic_ready);
  if (!topic_ready) {
    return;
  }

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

  EXPECT_TRUE(harness.state->published_outgoing_control_packets.empty());
  EXPECT_TRUE(harness.state->published_cdr_track_names.empty());
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
    options, [&publish_started_promise, &release_publish, &publish_started_once](FakeRoomSession & session) {
      session.state->publish_cdr_track_handler =
        [&publish_started_promise, &release_publish, &publish_started_once](const std::string &) {
          if (!publish_started_once.exchange(true)) {
            publish_started_promise.set_value();
          }
          release_publish.wait();
          return std::shared_ptr<livekit::LocalDataTrack>{};
        };
    });
  ASSERT_NE(harness.runtime, nullptr);

  auto observer = std::make_shared<rclcpp::Node>(nextNodeName("shutdown_publish_track_observer"));
  auto publisher = observer->create_publisher<sensor_msgs::msg::BatteryState>("/battery", rclcpp::QoS(10));
  ASSERT_NE(publisher, nullptr);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(harness.node);
  executor.add_node(observer);

  const bool topic_ready = waitForTopicType(executor, harness.node, "/battery", "sensor_msgs/msg/BatteryState");
  EXPECT_TRUE(topic_ready);
  if (!topic_ready) {
    return;
  }

  std::thread executor_thread([&executor]() { executor.spin(); });

  const std::string heartbeat =
    R"({"subscriptions":[{"topic":"/battery","delivery_preferences":{"interval_ms":125}}]})";
  harness.state->callbacks.on_incoming_control_packet_received(
    IncomingControlPacket{
      std::vector<std::uint8_t>(heartbeat.begin(), heartbeat.end()),
      protocol::kControlSubscriptionsHeartbeat,
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

  const bool subscription_ready = waitUntil([&publisher]() { return publisher->get_subscription_count() == 1U; });
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
  EXPECT_EQ(publisher->get_subscription_count(), 1U);

  release_publish_promise.set_value();

  EXPECT_EQ(destroy_finished.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  destroy_thread.join();
  executor.cancel();
  executor_thread.join();

  EXPECT_EQ(harness.state->published_cdr_track_names.size(), 1U);
}

}  // namespace livekit_ros2_bridge
