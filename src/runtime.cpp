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

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <utility>

#include "packet_router.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/node_interfaces/node_timers_interface.hpp"
#include "ros_executor_queue.hpp"
#include "ros_service_caller.hpp"
#include "ros_topic_publisher.hpp"
#include "rpc_router.hpp"
#include "subscription_lease_manager.hpp"
#include "utils/log_event.hpp"

namespace livekit_ros2_bridge
{

namespace
{

constexpr auto kLeaseGcInterval = std::chrono::seconds(1);
constexpr auto kWatchdogEvaluationInterval = std::chrono::milliseconds(250);
constexpr auto kShutdownExitDelay = std::chrono::milliseconds(100);

class CallbackAdmission final
{
public:
  bool close()
  {
    bool expected = false;
    return closed_.compare_exchange_strong(expected, true);
  }

  bool isClosed() const
  {
    return closed_.load();
  }

private:
  std::atomic<bool> closed_{false};
};

class ShutdownLogScope final
{
public:
  explicit ShutdownLogScope(rclcpp::Logger logger)
  : logger_(std::move(logger))
  {}

  void begin()
  {
    if (started_) {
      return;
    }

    started_ = true;
    LogEvent(logger_, "runtime_shutdown_start").info();
  }

  ~ShutdownLogScope()
  {
    if (!started_) {
      return;
    }

    LogEvent(logger_, "runtime_shutdown_complete").info();
  }

private:
  rclcpp::Logger logger_;
  bool started_ = false;
};

class ScopedRoomConnection final
{
public:
  explicit ScopedRoomConnection(std::unique_ptr<RoomConnection> connection)
  : connection_(std::move(connection))
  {}

  ~ScopedRoomConnection()
  {
    if (connection_ != nullptr) {
      connection_->stop();
    }
  }

  RoomConnection & connection() const
  {
    return *connection_;
  }

private:
  std::unique_ptr<RoomConnection> connection_;
};

class ScopedRpcRegistration final
{
public:
  ScopedRpcRegistration() = default;

  ~ScopedRpcRegistration()
  {
    if (router_ == nullptr || connection_ == nullptr) {
      return;
    }

    router_->unregisterRpcs(*connection_);
  }

  void arm(RpcRouter & router, RoomConnection & connection)
  {
    router_ = &router;
    connection_ = &connection;
  }

private:
  RpcRouter * router_ = nullptr;
  RoomConnection * connection_ = nullptr;
};

}  // namespace

class Runtime::Impl final
{
public:
  using SteadyClock = std::chrono::steady_clock;

  Impl(rclcpp::Node & node, std::unique_ptr<RoomConnection> connection, RuntimeConfig config)
  : shutdown_log_scope_(node.get_logger())
  , base_(node.get_node_base_interface())
  , graph_(node.get_node_graph_interface())
  , timers_(node.get_node_timers_interface())
  , clock_(node.get_clock())
  , logger_(node.get_logger())
  , config_(std::move(config))
  , room_connection_(std::move(connection))
  , ros_executor_queue_(base_, node.get_node_waitables_interface(), clock_)
  , ros_topic_publisher_(node.get_node_topics_interface(), graph_, clock_, config_.access_policy)
  , ros_service_caller_(base_, graph_, node.get_node_waitables_interface())
  , subscription_lease_manager_(
      node.get_node_parameters_interface(),
      node.get_node_topics_interface(),
      graph_,
      clock_,
      room_connection_.connection(),
      config_.access_policy,
      &config_.subscription_qos,
      &config_.video_stream)
  , packet_router_(
      clock_,
      [this](std::function<void()> work) { submitToExecutor(std::move(work)); },
      subscription_lease_manager_,
      ros_topic_publisher_)
  , rpc_router_(graph_, config_.access_policy, ros_executor_queue_, ros_service_caller_)
  {
    LogEvent(logger_, "runtime_startup_begin")
      .fieldOr("url", config_.livekit.url, "<unset>")
      .field("token_present", !config_.livekit.access_token.empty())
      .info();

    if (config_.health.watchdog_enabled) {
      setWatchdogUnhealthy("startup_connect_pending");
      watchdog_timer_ = rclcpp::create_wall_timer(
        kWatchdogEvaluationInterval, [this]() { checkWatchdog(); }, nullptr, base_.get(), timers_.get());
    }

    subscription_lease_gc_timer_ = rclcpp::create_wall_timer(
      kLeaseGcInterval,
      [this]() { submitToExecutor([this]() { subscription_lease_manager_.pruneExpiredLeases(); }); },
      nullptr,
      base_.get(),
      timers_.get());

    const bool rpcs_registered = rpc_router_.registerRpcs(room_connection_.connection());
    rpc_registration_.arm(rpc_router_, room_connection_.connection());
    if (!rpcs_registered) {
      LogEvent(logger_, "runtime_startup_failed")
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

    room_connection_.connection().start(config_.livekit, std::move(callbacks));
  }

  ~Impl()
  {
    if (!callback_admission_.close()) {
      return;
    }

    shutdown_log_scope_.begin();

    subscription_lease_gc_timer_.reset();
    watchdog_timer_.reset();

    ros_executor_queue_.shutdown();
  }

  Impl(const Impl &) = delete;
  Impl & operator=(const Impl &) = delete;
  Impl(Impl &&) = delete;
  Impl & operator=(Impl &&) = delete;

private:
  void onRoomConnected()
  {
    if (callback_admission_.isClosed()) {
      return;
    }

    setWatchdogHealthy("room_connected");
    LogEvent(logger_, "runtime_ready").info();
  }

  void onRoomIncomingPacket(const IncomingPacket & packet)
  {
    if (callback_admission_.isClosed()) {
      LogEvent(logger_, "packet_dropped")
        .field("reason", "shutdown")
        .field("topic", packet.topic)
        .fieldOr("requester_identity", packet.requester_identity)
        .warnThrottle(*clock_, std::chrono::seconds(5));
      return;
    }

    packet_router_.handle(packet);
  }

  void onRoomRemoteParticipantDisconnected(std::string remote_participant_identity)
  {
    if (callback_admission_.isClosed()) {
      return;
    }

    submitToExecutor([this, remote_participant_identity = std::move(remote_participant_identity)]() {
      subscription_lease_manager_.onRemoteParticipantDisconnected(remote_participant_identity);
      ros_service_caller_.cancelForRequester(remote_participant_identity);
    });
  }

  void onRoomReconnectRequested(const std::string & reason)
  {
    if (callback_admission_.isClosed()) {
      return;
    }

    const std::string_view disconnect_reason =
      reason.empty() ? std::string_view("connection_lost") : std::string_view(reason);
    setWatchdogUnhealthy(disconnect_reason);

    LogEvent log = LogEvent(logger_, "runtime_disconnect_observed").field("disconnect_reason", disconnect_reason);

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
    if (callback_admission_.isClosed()) {
      return;
    }

    setWatchdogHealthy("room_reconnected");
    LogEvent(logger_, "runtime_ready").info();
  }

  void onRoomConnectionReset()
  {
    if (callback_admission_.isClosed()) {
      return;
    }

    submitToExecutor([this]() {
      subscription_lease_manager_.resetSessionState();
      ros_service_caller_.resetSessionState();
    });
  }

  void submitToExecutor(std::function<void()> work)
  {
    if (callback_admission_.isClosed()) {
      return;
    }

    (void)ros_executor_queue_.submit([work = std::move(work)]() mutable { work(); });
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

    LogEvent(logger_, "runtime_watchdog_healthy")
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

    LogEvent event = LogEvent(logger_, "runtime_watchdog_unhealthy")
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
    if (callback_admission_.isClosed()) {
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

    if (!callback_admission_.close()) {
      return;
    }

    LogEvent(logger_, "runtime_watchdog_triggered")
      .field("disconnect_reason", "recovery_timeout")
      .field("recovery_timeout_seconds", config_.health.watchdog_recovery_timeout.count() / 1000.0)
      .error();

    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }

    std::this_thread::sleep_for(kShutdownExitDelay);
    std::_Exit(EXIT_FAILURE);
  }

  ShutdownLogScope shutdown_log_scope_;
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base_;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph_;
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr timers_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;
  RuntimeConfig config_;
  CallbackAdmission callback_admission_;
  std::mutex watchdog_mutex_;
  std::optional<SteadyClock::time_point> watchdog_deadline_;
  std::optional<SteadyClock::time_point> watchdog_unhealthy_since_;
  ScopedRoomConnection room_connection_;
  RosExecutorQueue ros_executor_queue_;
  RosTopicPublisher ros_topic_publisher_;
  RosServiceCaller ros_service_caller_;
  SubscriptionLeaseManager subscription_lease_manager_;
  PacketRouter packet_router_;
  RpcRouter rpc_router_;
  ScopedRpcRegistration rpc_registration_;
  rclcpp::TimerBase::SharedPtr subscription_lease_gc_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
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
