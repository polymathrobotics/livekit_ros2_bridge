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
#include <stdexcept>
#include <utility>

#include "cdr_track_publisher.hpp"
#include "control_packet_router.hpp"
#include "ros_executor_queue.hpp"
#include "ros_service_caller.hpp"
#include "rpc_router.hpp"
#include "subscription_heartbeat_processor.hpp"
#include "subscription_registry.hpp"
#include "topic_publish_command.hpp"
#include "topic_publisher.hpp"
#include "video_sidecar_supervisor.hpp"

namespace livekit_ros2_bridge
{

namespace
{

constexpr auto kLeaseGcInterval = std::chrono::seconds(1);
constexpr auto kReconnectInitialBackoff = std::chrono::milliseconds(500);
constexpr auto kReconnectMaxBackoff = std::chrono::milliseconds(10000);

}  // namespace

Runtime::Runtime(rclcpp::Node & node, std::unique_ptr<RoomSession> session, RuntimeConfig runtime_config)
: node_(node)
, room_session_(std::move(session))
, video_config_(std::move(runtime_config.video_config))
{
  if (room_session_ == nullptr) {
    throw std::runtime_error("Failed to create LiveKit session");
  }

  RCLCPP_INFO(node_.get_logger(), "Parameters loaded");
  RCLCPP_INFO(
    node_.get_logger(),
    "LiveKit config loaded: url=%s room=%s identity=%s ttl_seconds=%ld refresh_margin_seconds=%ld",
    runtime_config.connect_config.url.c_str(),
    runtime_config.connect_config.room.c_str(),
    runtime_config.connect_config.identity.c_str(),
    static_cast<long>(runtime_config.loaded_params.livekit.token_ttl_seconds),
    static_cast<long>(runtime_config.loaded_params.livekit.token_refresh_margin_seconds));

  ros_executor_queue_ = std::make_unique<RosExecutorQueue>(node_);
  cdr_track_publisher_ = std::make_unique<CdrTrackPublisher>(*room_session_, node_.get_clock());
  ros_topic_publisher_ = std::make_unique<RosTopicPublisher>(
    node_, runtime_config.access_policy, runtime_config.loaded_params.publish.max_topics);

  if (runtime_config.video_sidecar_config.has_value()) {
    video_sidecar_supervisor_ = std::make_unique<VideoSidecarSupervisor>(
      *runtime_config.video_sidecar_config,
      buildGstreamerSidecarCommand,
      [this](const std::string & publisher_identity) {
        return room_session_ != nullptr && room_session_->isVideoPublisherHealthy(publisher_identity);
      });
  }

  subscription_registry_ = std::make_unique<SubscriptionRegistry>(
    node_,
    [this](const std::string & track_name, const std::uint8_t * data, std::size_t size) {
      cdr_track_publisher_->pushMessage(track_name, data, size);
    },
    [this](const std::string & track_name, std::size_t generation) {
      submitExecutorWork([this, track_name, generation]() {
        cdr_track_publisher_->publishTrack(track_name, generation, *subscription_registry_);
      });
    },
    [this](const std::string & track_name) { cdr_track_publisher_->unpublishTrack(track_name); },
    video_sidecar_supervisor_.get(),
    &video_config_);

  subscription_heartbeat_processor_ = std::make_unique<SubscriptionHeartbeatProcessor>(
    *subscription_registry_, *room_session_, runtime_config.access_policy, node_.get_clock());
  ros_service_caller_ = std::make_unique<RosServiceCaller>(node_);
  rpc_router_ =
    std::make_unique<RpcRouter>(node_, runtime_config.access_policy, *ros_executor_queue_, *ros_service_caller_);
  control_packet_router_ = std::make_unique<ControlPacketRouter>(
    node_.get_logger(),
    ControlPacketRouter::Handlers{
      [this](std::string requester_identity, SubscriptionHeartbeat heartbeat) {
        submitExecutorWork(
          [this, requester_identity = std::move(requester_identity), heartbeat = std::move(heartbeat)]() {
            subscription_heartbeat_processor_->process(requester_identity, heartbeat);
          });
      },
      [this](std::string requester_identity, TopicPublishCommand command) {
        submitExecutorWork([this, requester_identity = std::move(requester_identity), command = std::move(command)]() {
          ros_topic_publisher_->publish(requester_identity, command);
        });
      },
    });

  lease_gc_timer_ = node_.create_wall_timer(kLeaseGcInterval, [this]() {
    submitExecutorWork([this]() {
      subscription_heartbeat_processor_->sweepExpiredSessionLeases();
      subscription_registry_->sweepExpiredLeases();
    });
  });

  room_session_->start(
    runtime_config.connect_config,
    runtime_config.token_source,
    RoomSessionCallbacks{
      [this]() {
        submitExecutorWork([this]() {
          cdr_track_publisher_->unpublishAll();
          subscription_registry_->resetSessionState();
          ros_service_caller_->resetSessionState();
        });
      },
      [this](const std::string & requester_identity) {
        const std::size_t gen = subscription_registry_->registryGeneration();
        submitExecutorWork([this, requester_identity, gen]() {
          // Keep leases alive across a browser refresh, but remember that the requester will
          // need fresh CDR track publications because LiveKit binds those publications to the
          // old participant session that just disconnected.
          subscription_registry_->markRequesterForCdrReplay(requester_identity, gen);
          ros_service_caller_->cancelCallsForRequester(requester_identity);
        });
      },
      [this](const IncomingControlPacket & packet) { handleIncomingControlPacket(packet); },
    },
    kReconnectInitialBackoff,
    kReconnectMaxBackoff,
    std::chrono::seconds(runtime_config.loaded_params.livekit.token_refresh_margin_seconds));
  if (!rpc_router_->registerRpcMethods(*room_session_)) {
    RCLCPP_ERROR(node_.get_logger(), "event=runtime_startup_failed reason=required_rpc_registration_failed");
    shutdown();
    throw std::runtime_error("Failed to register required RPC methods");
  }
}

Runtime::~Runtime()
{
  shutdown();
}

void Runtime::shutdown()
{
  if (shutting_down_.exchange(true)) {
    return;
  }

  lease_gc_timer_.reset();

  if (rpc_router_ != nullptr && room_session_ != nullptr) {
    rpc_router_->unregisterRpcMethods(*room_session_);
  }
  // Stop the room session before shutting down the executor queue so SDK callbacks can no longer
  // enqueue fresh ROS work while already-running executor tasks finish.
  if (room_session_ != nullptr) {
    room_session_->stop();
  }
  if (ros_executor_queue_ != nullptr) {
    ros_executor_queue_->shutdown();
  }
  if (subscription_registry_ != nullptr) {
    subscription_registry_->shutdown();
  }
  if (cdr_track_publisher_ != nullptr) {
    cdr_track_publisher_->unpublishAll();
  }
  if (video_sidecar_supervisor_ != nullptr) {
    video_sidecar_supervisor_->shutdown();
  }
  if (ros_service_caller_ != nullptr) {
    ros_service_caller_->shutdown();
  }
  if (ros_topic_publisher_ != nullptr) {
    ros_topic_publisher_->shutdown();
  }
}

bool Runtime::isShuttingDown() const
{
  return shutting_down_.load();
}

void Runtime::submitExecutorWork(std::function<void()> fn)
{
  if (isShuttingDown() || ros_executor_queue_ == nullptr) {
    return;
  }
  (void)ros_executor_queue_->submit([this, fn = std::move(fn)]() mutable {
    // The queue can still be draining work that was accepted before shutdown flipped the flag.
    if (isShuttingDown()) {
      return;
    }
    fn();
  });
}

void Runtime::handleIncomingControlPacket(const IncomingControlPacket & packet) const
{
  if (isShuttingDown() || control_packet_router_ == nullptr) {
    return;
  }
  control_packet_router_->route(packet);
}

}  // namespace livekit_ros2_bridge
