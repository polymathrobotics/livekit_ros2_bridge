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

#include "data_stream_registry.hpp"

#include <stdexcept>
#include <utility>

#include "rclcpp/logging.hpp"
#include "utils/log_event.hpp"

namespace livekit_ros2_bridge
{

namespace
{

const auto kLogger = rclcpp::get_logger("livekit_ros2_bridge.data_stream_registry");

}  // namespace

DataStreamRegistry::DataStreamRegistry(
  SubscriptionNodeInterfaces interfaces, RoomConnection & room_connection, const SubscriptionQosConfig * qos_config)
: interfaces_(std::move(interfaces))
, room_connection_(room_connection)
, qos_config_(qos_config)
{}

DataStreamRegistry::DataStreamRegistry(
  rclcpp::Node & node, RoomConnection & room_connection, const SubscriptionQosConfig * qos_config)
: DataStreamRegistry(makeRosNodeInterfaces(node).subscription(), room_connection, qos_config)
{}

DataStreamRegistry::~DataStreamRegistry()
{
  shutdown();
}

void DataStreamRegistry::create(const std::string & topic, const std::string & interface_type)
{
  if (is_shutdown_.load()) {
    LogEvent(kLogger, "data_stream_registry_create_rejected")
      .field("resource", topic)
      .field("reason", "shutdown")
      .warn();
    throw std::runtime_error("Data stream registry is shut down.");
  }
  if (instances_.find(topic) != instances_.end()) {
    throw std::logic_error("data stream already exists for topic '" + topic + "'");
  }

  auto instance = std::shared_ptr<DataStreamInstance>(
    new DataStreamInstance(topic, interface_type, interfaces_, room_connection_, *this, callback_gate_, qos_config_));
  instance->subscribe();

  const std::string track_name = instance->trackName();
  if (findInstanceByTrackName(track_name) != nullptr) {
    throw std::logic_error("data stream already exists for track '" + track_name + "'");
  }

  instances_.emplace(topic, std::move(instance));
}

DataStreamInstance * DataStreamRegistry::find(const std::string & topic)
{
  const auto it = instances_.find(topic);
  return it == instances_.end() ? nullptr : it->second.get();
}

const DataStreamInstance * DataStreamRegistry::find(const std::string & topic) const
{
  const auto it = instances_.find(topic);
  return it == instances_.end() ? nullptr : it->second.get();
}

void DataStreamRegistry::setIntervalMs(const std::string & topic, int interval_ms)
{
  requireInstance(topic)->setIntervalMs(interval_ms);
}

void DataStreamRegistry::start(const std::string & topic)
{
  requireInstance(topic)->start(generation_.load());
}

void DataStreamRegistry::republish(const std::string & topic)
{
  requireInstance(topic)->republish(generation_.load());
}

void DataStreamRegistry::stop(const std::string & topic)
{
  const auto it = instances_.find(topic);
  if (it == instances_.end()) {
    return;
  }

  auto instance = std::move(it->second);
  instances_.erase(it);
  instance->shutdown();
  generation_.fetch_add(1);
}

bool DataStreamRegistry::onTrackPublished(const std::string & track_name, std::size_t generation)
{
  if (is_shutdown_.load()) {
    return false;
  }

  const auto instance = findInstanceByTrackName(track_name);
  if (instance == nullptr) {
    return false;
  }

  return instance->completePublish(generation);
}

void DataStreamRegistry::onTrackFailed(const std::string & track_name)
{
  if (is_shutdown_.load()) {
    return;
  }

  const auto instance = findInstanceByTrackName(track_name);
  if (instance == nullptr) {
    return;
  }

  instance->failPublish();
}

std::size_t DataStreamRegistry::generation() const
{
  return generation_.load();
}

void DataStreamRegistry::resetSessionState()
{
  if (is_shutdown_.load()) {
    return;
  }

  LogEvent(kLogger, "data_stream_registry_reset_begin").field("stream_count", instances_.size()).info();
  clearInstances(true);
}

void DataStreamRegistry::shutdown()
{
  if (is_shutdown_.exchange(true)) {
    return;
  }

  LogEvent(kLogger, "data_stream_registry_shutdown_begin").field("stream_count", instances_.size()).info();
  clearInstances(false);
}

std::shared_ptr<DataStreamInstance> DataStreamRegistry::requireInstance(const std::string & topic) const
{
  const auto it = instances_.find(topic);
  if (it == instances_.end()) {
    throw std::logic_error("unknown data stream topic '" + topic + "'");
  }

  return it->second;
}

std::shared_ptr<DataStreamInstance> DataStreamRegistry::findInstanceByTrackName(const std::string & track_name) const
{
  // Publish completion/failure callbacks are infrequent, so derive the matching instance from
  // the owned runtimes instead of mirroring a second track-name index.
  for (const auto & [topic, instance] : instances_) {
    (void)topic;
    if (instance->trackName() == track_name) {
      return instance;
    }
  }

  return nullptr;
}

void DataStreamRegistry::clearInstances(bool reopen_gate)
{
  const std::size_t callback_generation = callback_gate_.close();
  auto owned_instances = std::move(instances_);
  instances_.clear();

  for (auto & [topic, instance] : owned_instances) {
    (void)topic;
    instance->shutdown();
  }

  generation_.fetch_add(1);
  if (reopen_gate) {
    callback_gate_.open(callback_generation);
  }
}

}  // namespace livekit_ros2_bridge
