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
#include <cstdlib>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace livekit_ros2_bridge::test_support
{

constexpr auto kDefaultWaitTimeout = std::chrono::seconds(2);
constexpr auto kWaitPollInterval = std::chrono::milliseconds(20);

inline std::chrono::nanoseconds boundedWaitTime(const std::chrono::steady_clock::time_point & deadline)
{
  const auto now = std::chrono::steady_clock::now();
  if (now >= deadline) {
    return std::chrono::nanoseconds(0);
  }

  const auto remaining = std::chrono::duration_cast<std::chrono::nanoseconds>(deadline - now);
  const auto poll_interval = std::chrono::duration_cast<std::chrono::nanoseconds>(kWaitPollInterval);
  return remaining < poll_interval ? remaining : poll_interval;
}

class ScopedRclcppInit
{
public:
  ScopedRclcppInit()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  ~ScopedRclcppInit()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

class ScopedEnvironmentVariable
{
public:
  ScopedEnvironmentVariable(const char * name, std::optional<std::string> value)
  : name_(name)
  , previous_value_(read(name))
  {
    apply(value);
  }

  ~ScopedEnvironmentVariable()
  {
    try {
      apply(previous_value_);
    } catch (...) {}
  }

private:
  static std::optional<std::string> read(const char * name)
  {
    const char * value = std::getenv(name);
    if (value == nullptr) {
      return std::nullopt;
    }
    return std::string(value);
  }

  void apply(const std::optional<std::string> & value) const
  {
    if (value.has_value()) {
      if (::setenv(name_.c_str(), value->c_str(), 1) != 0) {
        throw std::runtime_error("setenv failed for " + name_);
      }
      return;
    }

    if (::unsetenv(name_.c_str()) != 0) {
      throw std::runtime_error("unsetenv failed for " + name_);
    }
  }

  std::string name_;
  std::optional<std::string> previous_value_;
};

inline bool spinUntil(
  rclcpp::executors::SingleThreadedExecutor & executor,
  const std::function<bool()> & predicate,
  std::chrono::milliseconds timeout = kDefaultWaitTimeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(kWaitPollInterval);
  }
  executor.spin_some();
  return predicate();
}

inline bool waitUntil(const std::function<bool()> & predicate, std::chrono::milliseconds timeout = kDefaultWaitTimeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(kWaitPollInterval);
  }
  return predicate();
}

inline bool topicHasSingleType(
  const std::shared_ptr<rclcpp::Node> & node, const std::string & topic, const std::string & expected_type)
{
  const auto topics = node->get_topic_names_and_types();
  const auto entry = topics.find(topic);
  return entry != topics.end() && entry->second.size() == 1U && entry->second.front() == expected_type;
}

inline bool waitForTopicType(
  rclcpp::executors::SingleThreadedExecutor & executor,
  const std::shared_ptr<rclcpp::Node> & node,
  const std::string & topic,
  const std::string & expected_type,
  std::chrono::milliseconds timeout = kDefaultWaitTimeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  const auto graph_event = node->get_graph_event();
  bool found = false;

  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    if (topicHasSingleType(node, topic, expected_type)) {
      found = true;
      break;
    }
    graph_event->check_and_clear();
    node->wait_for_graph_change(graph_event, boundedWaitTime(deadline));
  }

  found = found || topicHasSingleType(node, topic, expected_type);

  if (!found) {
    RCLCPP_WARN(
      node->get_logger(),
      "event=wait_for_topic_type_timeout topic=%s expected_type=%s timeout_ms=%lld",
      topic.c_str(),
      expected_type.c_str(),
      static_cast<long long>(timeout.count()));
  }

  return found;
}

}  // namespace livekit_ros2_bridge::test_support
