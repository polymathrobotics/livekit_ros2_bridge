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

#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "data_track_publisher.hpp"

namespace rclcpp
{
class GenericSubscription;
class Node;
class SerializedMessage;
}  // namespace rclcpp

namespace livekit_ros2_bridge
{

struct SubscriptionQosConfig;
class QuiesceGate;
class RoomConnection;
class SubscriptionRegistry;

// todo: reword for clarity
// SubscriptionRegistry owns the shared lease state for a topic and creates one DataStreamInstance
// when that topic needs a data delivery runtime. Each DataStreamInstance owns the ROS
// subscription plus one DataTrackPublisher for the matching LiveKit data track.
class DataStreamInstance final : public std::enable_shared_from_this<DataStreamInstance>
{
public:
  using Clock = std::chrono::steady_clock;

  enum class State
  {
    kNone,
    kPending,
    kPublished,
    kFailed
  };

  DataStreamInstance(const DataStreamInstance &) = delete;
  DataStreamInstance & operator=(const DataStreamInstance &) = delete;
  DataStreamInstance(DataStreamInstance &&) = delete;
  DataStreamInstance & operator=(DataStreamInstance &&) = delete;
  ~DataStreamInstance();

  const std::string & trackName() const;
  int intervalMs() const;
  State state() const;

  void setIntervalMs(int interval_ms);
  // Starts one LiveKit publish attempt when this instance is idle or recovering from a failed
  // publish. The registry-supplied generation is echoed back through completion callbacks so
  // stale async results from an older lifetime can be ignored.
  void start(std::size_t generation);
  // Re-publishes the same deterministic track name without recreating the ROS subscription so a
  // rejoined participant session can observe the track again.
  void republish(std::size_t generation);
  // Accepts publish completion only for the currently pending generation. Delayed completions
  // from a prior publish, reset, or replacement instance are rejected as stale.
  // todo: rename for clarity
  bool completePublish(std::size_t generation);
  // Records a failed publish attempt while keeping the ROS subscription alive so a later lease
  // refresh can retry on the same topic runtime.
  void failPublish();
  void shutdown();

private:
  friend class SubscriptionRegistry;

  // todo: get this noisy stuff moved elsewhere
  // Owns the state-machine contract for one deterministic LiveKit data track name. The reserved
  // generation advances only when a new publish attempt starts, and completion can succeed only
  // while that exact generation is still pending.
  struct PublicationState
  {
    State current() const
    {
      return state;
    }

    bool canStart() const
    {
      return state == State::kNone || state == State::kFailed;
    }

    bool canRepublish() const
    {
      return state == State::kPublished;
    }

    bool isPublished() const
    {
      return state == State::kPublished;
    }

    void beginPublish(std::size_t generation)
    {
      reserved_generation = generation;
      state = State::kPending;
    }

    bool completePublish(std::size_t generation)
    {
      if (state != State::kPending) {
        return false;
      }

      if (reserved_generation != generation) {
        return false;
      }

      state = State::kPublished;
      return true;
    }

    void failPublish()
    {
      state = State::kFailed;
    }

    void reset()
    {
      state = State::kNone;
    }

    State state = State::kNone;
    std::size_t reserved_generation = 0U;
  };

  // todo: examples and get the noisy part moved elsewhere
  // Tracks the per-track interval suppression window. The window advances only when a
  // published track forwards a message, and it resets whenever the publication state is torn
  // down so the next successful publish can deliver immediately.
  struct SuppressionWindow
  {
    int intervalMs() const
    {
      return interval_ms;
    }

    void setIntervalMs(int interval_ms)
    {
      this->interval_ms = interval_ms;
    }

    void reset()
    {
      last_delivery_at.reset();
    }

    bool allow(Clock::time_point now)
    {
      if (interval_ms == 0) {
        return true;
      }

      if (!last_delivery_at) {
        last_delivery_at = now;
        return true;
      }

      const auto window = std::chrono::milliseconds(interval_ms);
      if (now - *last_delivery_at < window) {
        return false;
      }

      last_delivery_at = now;
      return true;
    }

    int interval_ms = 0;
    std::optional<Clock::time_point> last_delivery_at;
  };

  DataStreamInstance(
    std::string topic,
    std::string interface_type,
    rclcpp::Node & node,
    RoomConnection & room_connection,
    SubscriptionRegistry & registry,
    QuiesceGate & callback_gate,
    const SubscriptionQosConfig * qos_config);

  void subscribe();
  void forwardMessage(const rclcpp::SerializedMessage & message);

  rclcpp::Node & node_;

  std::string topic_;
  std::string interface_type_;
  std::string track_name_;

  std::shared_ptr<rclcpp::GenericSubscription> subscription_;
  DataTrackPublisher publisher_;
  // todo: rename subscription_registry_
  SubscriptionRegistry & registry_;

  const SubscriptionQosConfig * qos_config_;
  SuppressionWindow suppression_window_;
  PublicationState publication_;

  // Captured from SubscriptionRegistry's QuiesceGate when this instance is created. Every queued
  // ROS callback must present the same gate generation before touching this instance.
  std::size_t gate_generation_ = 0U;
  QuiesceGate & callback_gate_;
};

}  // namespace livekit_ros2_bridge
