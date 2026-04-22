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

#include "runtime.hpp"

#include <chrono>
#include <cstdlib>
#include <functional>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <utility>

#include "data_stream_registry.hpp"
#include "packet_router.hpp"
#include "rclcpp/create_timer.hpp"
#include "ros_executor_queue.hpp"
#include "ros_node_interfaces.hpp"
#include "ros_service_caller.hpp"
#include "ros_topic_publisher.hpp"
#include "rpc_router.hpp"
#include "subscription_lease_manager.hpp"
#include "utils/event_throttle.hpp"
#include "utils/log_event.hpp"
#include "utils/scope_exit.hpp"
#include "video_profiling.hpp"
#include "video_stream_registry.hpp"

namespace livekit_ros2_bridge
{

namespace
{

constexpr auto kLeaseGcInterval = std::chrono::seconds(1);
constexpr auto kWatchdogEvaluationInterval = std::chrono::milliseconds(250);
constexpr auto kShutdownExitDelay = std::chrono::milliseconds(100);

}  // namespace

class Runtime::Impl final
{
public:
  using SteadyClock = std::chrono::steady_clock;

  Impl(rclcpp::Node & node, std::unique_ptr<RoomConnection> connection, RuntimeConfig config)
  : interfaces_(makeRosNodeInterfaces(node))
  , config_(std::move(config))
  , room_connection_(std::move(connection))
  , ros_executor_queue_(interfaces_.executor())
  , ros_topic_publisher_(interfaces_.publisher(), config_.access_policy)
  , ros_service_caller_(interfaces_.service())
  , data_stream_registry_(interfaces_.subscription(), *room_connection_, &config_.subscription_qos)
  , profiling_registry_(
      config_.profiling.enabled
        ? std::optional<VideoProfilingRegistry>(std::in_place, interfaces_.logger, config_.profiling)
        : std::nullopt)
  , video_stream_registry_(
      interfaces_.subscription(),
      *room_connection_,
      &config_.subscription_qos,
      profiling_registry_ ? &*profiling_registry_ : nullptr,
      &config_.video_stream)
  , subscription_lease_manager_(
      interfaces_.graphOnly(), *room_connection_, config_.access_policy, data_stream_registry_, video_stream_registry_)
  , packet_router_(
      interfaces_.clock,
      [this](std::function<void()> work) { submitToExecutor(std::move(work)); },
      subscription_lease_manager_,
      ros_topic_publisher_)
  , rpc_router_(interfaces_.graph, config_.access_policy, ros_executor_queue_, ros_service_caller_)
  {
    LogEvent(interfaces_.logger, "runtime_startup_begin")
      .fieldOr("url", config_.livekit.url, "<unset>")
      .field("token_present", !config_.livekit.access_token.empty())
      .info();

    bool startup_complete = false;
    ScopeExit rollback([this, &startup_complete]() {
      if (!startup_complete) {
        teardown();
      }
    });

    if (config_.health.watchdog_enabled) {
      setWatchdogUnhealthy("startup_connect_pending");
      watchdog_timer_ = rclcpp::create_wall_timer(
        kWatchdogEvaluationInterval,
        [this]() { checkWatchdog(); },
        nullptr,
        interfaces_.base.get(),
        interfaces_.timers.get());
    }

    if (profiling_registry_) {
      profiling_registry_->logConfig();
      video_profile_summary_timer_ = rclcpp::create_wall_timer(
        profiling_registry_->config().summary_interval,
        [this]() { profiling_registry_->logSummaries(); },
        nullptr,
        interfaces_.base.get(),
        interfaces_.timers.get());
    }

    subscription_lease_gc_timer_ = rclcpp::create_wall_timer(
      kLeaseGcInterval,
      [this]() { submitToExecutor([this]() { subscription_lease_manager_.pruneExpiredLeases(); }); },
      nullptr,
      interfaces_.base.get(),
      interfaces_.timers.get());

    if (!rpc_router_.registerRpcs(*room_connection_)) {
      LogEvent(interfaces_.logger, "runtime_startup_failed")
        .fieldOr("url", config_.livekit.url, "<unset>")
        .field("token_present", !config_.livekit.access_token.empty())
        .field("reason", "required_rpc_registration_failed")
        .error();
      throw std::runtime_error("Failed to register required RPC methods");
    }

    RoomEventCallbacks callbacks;
    callbacks.on_connected = std::bind(&Impl::onRoomConnected, this);
    callbacks.on_incoming_packet_received = std::bind(&Impl::onRoomIncomingPacket, this, std::placeholders::_1);
    callbacks.on_remote_participant_disconnected =
      std::bind(&Impl::onRoomRemoteParticipantDisconnected, this, std::placeholders::_1);
    callbacks.on_reconnect_requested = std::bind(&Impl::onRoomReconnectRequested, this, std::placeholders::_1);
    callbacks.on_reconnecting = std::bind(&Impl::onRoomReconnecting, this, std::placeholders::_1);
    callbacks.on_reconnected = std::bind(&Impl::onRoomReconnected, this);
    callbacks.on_connection_reset = std::bind(&Impl::onRoomConnectionReset, this);

    room_connection_->start(config_.livekit, std::move(callbacks));
    startup_complete = true;
  }

  ~Impl()
  {
    teardown();
  }

  Impl(const Impl &) = delete;
  Impl & operator=(const Impl &) = delete;
  Impl(Impl &&) = delete;
  Impl & operator=(Impl &&) = delete;

private:
  void teardown()
  {
    if (shutting_down_.exchange(true)) {
      return;
    }

    LogEvent(interfaces_.logger, "runtime_shutdown_start").info();

    subscription_lease_gc_timer_.reset();
    watchdog_timer_.reset();
    video_profile_summary_timer_.reset();

    rpc_router_.unregisterRpcs(*room_connection_);
    room_connection_->stop();
    ros_executor_queue_.shutdown();
    subscription_lease_manager_.shutdown();
    data_stream_registry_.shutdown();
    video_stream_registry_.shutdown();
    ros_service_caller_.shutdown();
    ros_topic_publisher_.shutdown();

    if (profiling_registry_) {
      profiling_registry_->logSummaries();
      profiling_registry_->flushTrace();
    }

    LogEvent(interfaces_.logger, "runtime_shutdown_complete").info();
  }

  void onRoomConnected()
  {
    if (shutting_down_.load()) {
      return;
    }

    setWatchdogHealthy("room_connected");
    LogEvent(interfaces_.logger, "runtime_ready").info();
  }

  void onRoomIncomingPacket(const IncomingPacket & packet)
  {
    if (shutting_down_.load()) {
      LogEvent(interfaces_.logger, "packet_dropped")
        .field("reason", "shutdown")
        .field("topic", packet.topic)
        .fieldOr("requester_identity", packet.requester_identity)
        .warnThrottle(*interfaces_.clock, std::chrono::seconds(5));
      return;
    }

    packet_router_.handle(packet);
  }

  void onRoomRemoteParticipantDisconnected(std::string remote_participant_identity)
  {
    submitToExecutor([this, remote_participant_identity = std::move(remote_participant_identity)]() {
      subscription_lease_manager_.onRemoteParticipantDisconnected(remote_participant_identity);
      ros_service_caller_.cancelCallsForRequester(remote_participant_identity);
    });
  }

  void onRoomReconnectRequested(const std::string & reason)
  {
    if (shutting_down_.load()) {
      return;
    }

    const std::string_view disconnect_reason =
      reason.empty() ? std::string_view("connection_lost") : std::string_view(reason);
    setWatchdogUnhealthy(disconnect_reason);

    LogEvent log =
      LogEvent(interfaces_.logger, "runtime_disconnect_observed").field("disconnect_reason", disconnect_reason);

    if (!config_.health.watchdog_enabled) {
      log.info();
      return;
    }

    log.field("recovery_timeout_seconds", config_.health.watchdog_recovery_timeout.count() / 1000.0);
    log.warn();
  }

  void onRoomReconnecting(const std::string & reason)
  {
    onRoomReconnectRequested(reason);
  }

  void onRoomReconnected()
  {
    if (shutting_down_.load()) {
      return;
    }

    setWatchdogHealthy("room_reconnected");
    LogEvent(interfaces_.logger, "runtime_ready").info();
  }

  void onRoomConnectionReset()
  {
    submitToExecutor([this]() {
      subscription_lease_manager_.resetSessionState();
      ros_service_caller_.resetSessionState();
    });
  }

  void submitToExecutor(std::function<void()> work)
  {
    if (shutting_down_.load()) {
      if (const std::size_t count = executor_shutdown_enqueue_drop_.recordAndTakePendingCount(); count > 0U) {
        LogEvent(interfaces_.logger, "executor_work_dropped")
          .field("reason", "shutdown")
          .field("stage", "enqueue")
          .field("count", count)
          .warn();
      }
      return;
    }

    (void)ros_executor_queue_.submit([this, work = std::move(work)]() mutable {
      if (shutting_down_.load()) {
        if (const std::size_t count = executor_shutdown_execute_drop_.recordAndTakePendingCount(); count > 0U) {
          LogEvent(interfaces_.logger, "executor_work_dropped")
            .field("reason", "shutdown")
            .field("stage", "execute")
            .field("count", count)
            .warn();
        }
        return;
      }

      work();
    });
  }

  void setWatchdogHealthy(std::string_view reason)
  {
    if (!config_.health.watchdog_enabled) {
      return;
    }

    std::optional<double> unhealthy_duration_seconds;
    const auto now = SteadyClock::now();
    {
      std::lock_guard<std::mutex> lock(watchdog_mutex_);
      if (watchdog_unhealthy_since_.has_value()) {
        unhealthy_duration_seconds = std::chrono::duration<double>(now - *watchdog_unhealthy_since_).count();
      }
      watchdog_deadline_.reset();
      watchdog_unhealthy_since_.reset();
    }

    if (!unhealthy_duration_seconds.has_value()) {
      return;
    }

    LogEvent(interfaces_.logger, "runtime_watchdog_healthy")
      .fieldOr("reason", reason, "unknown")
      .field("unhealthy_duration_seconds", *unhealthy_duration_seconds)
      .info();
  }

  void setWatchdogUnhealthy(std::string_view reason)
  {
    if (!config_.health.watchdog_enabled) {
      return;
    }

    bool transitioned = false;
    const auto now = SteadyClock::now();
    {
      std::lock_guard<std::mutex> lock(watchdog_mutex_);
      watchdog_deadline_ = now + config_.health.watchdog_recovery_timeout;
      if (!watchdog_unhealthy_since_.has_value()) {
        watchdog_unhealthy_since_ = now;
        transitioned = true;
      }
    }

    if (!transitioned) {
      return;
    }

    LogEvent event = LogEvent(interfaces_.logger, "runtime_watchdog_unhealthy")
                       .fieldOr("reason", reason, "unknown")
                       .field("recovery_timeout_seconds", config_.health.watchdog_recovery_timeout.count() / 1000.0);
    if (reason == "startup_connect_pending") {
      event.info();
      return;
    }
    event.warn();
  }

  void checkWatchdog()
  {
    if (shutting_down_.load()) {
      return;
    }

    const auto now = SteadyClock::now();
    {
      std::lock_guard<std::mutex> lock(watchdog_mutex_);
      if (!watchdog_deadline_.has_value()) {
        return;
      }
      if (now < *watchdog_deadline_) {
        return;
      }
    }

    if (shutting_down_.exchange(true)) {
      return;
    }

    LogEvent(interfaces_.logger, "runtime_watchdog_triggered")
      .field("disconnect_reason", "recovery_timeout")
      .field("recovery_timeout_seconds", config_.health.watchdog_recovery_timeout.count() / 1000.0)
      .error();

    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }

    std::this_thread::sleep_for(kShutdownExitDelay);
    std::_Exit(EXIT_FAILURE);
  }

  RosNodeInterfaces interfaces_;
  RuntimeConfig config_;
  std::atomic<bool> shutting_down_{false};
  std::mutex watchdog_mutex_;
  std::optional<SteadyClock::time_point> watchdog_deadline_;
  std::optional<SteadyClock::time_point> watchdog_unhealthy_since_;
  std::unique_ptr<RoomConnection> room_connection_;
  RosExecutorQueue ros_executor_queue_;
  RosTopicPublisher ros_topic_publisher_;
  RosServiceCaller ros_service_caller_;
  DataStreamRegistry data_stream_registry_;
  std::optional<VideoProfilingRegistry> profiling_registry_;
  VideoStreamRegistry video_stream_registry_;
  SubscriptionLeaseManager subscription_lease_manager_;
  PacketRouter packet_router_;
  RpcRouter rpc_router_;
  rclcpp::TimerBase::SharedPtr subscription_lease_gc_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::TimerBase::SharedPtr video_profile_summary_timer_;
  EventThrottle executor_shutdown_enqueue_drop_{std::chrono::seconds(5)};
  EventThrottle executor_shutdown_execute_drop_{std::chrono::seconds(5)};
};

Runtime::Runtime(rclcpp::Node & node, std::unique_ptr<RoomConnection> connection, RuntimeConfig config)
{
  if (connection == nullptr) {
    throw std::runtime_error("Failed to create LiveKit room connection");
  }
  impl_ = std::make_unique<Impl>(node, std::move(connection), std::move(config));
}

Runtime::~Runtime() = default;

}  // namespace livekit_ros2_bridge
