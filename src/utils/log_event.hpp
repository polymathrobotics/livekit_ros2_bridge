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
#include <exception>
#include <ostream>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>

#include "rclcpp/clock.hpp"
#include "rclcpp/logging.hpp"

namespace livekit_ros2_bridge
{

constexpr std::string_view kUnknownLogFieldValue = "<unknown>";
constexpr std::string_view kUnknownExceptionLogFieldValue = "unknown_exception";

// Builds event=<name> plus ordered key=value fields; keys and values must be log-parser-safe.
class LogEvent
{
public:
  explicit LogEvent(rclcpp::Logger logger, std::string_view event_name)
  : logger_(std::move(logger))
  {
    stream_ << std::boolalpha << "event=" << event_name;
  }

  LogEvent(const LogEvent &) = delete;
  LogEvent & operator=(const LogEvent &) = delete;
  LogEvent(LogEvent &&) = default;
  LogEvent & operator=(LogEvent &&) = default;

  template <typename T>
  LogEvent & field(std::string_view key, const T & value) &
  {
    appendField(key, value);
    return *this;
  }

  template <typename T>
  LogEvent && field(std::string_view key, const T & value) &&
  {
    appendField(key, value);
    return std::move(*this);
  }

  LogEvent & field(std::string_view key, const char * value) &
  {
    appendField(key, value);
    return *this;
  }

  LogEvent && field(std::string_view key, const char * value) &&
  {
    appendField(key, value);
    return std::move(*this);
  }

  template <typename T>
  LogEvent & fieldIf(bool condition, std::string_view key, const T & value) &
  {
    appendFieldIf(condition, key, value);
    return *this;
  }

  template <typename T>
  LogEvent && fieldIf(bool condition, std::string_view key, const T & value) &&
  {
    appendFieldIf(condition, key, value);
    return std::move(*this);
  }

  LogEvent & fieldIfNotEmpty(std::string_view key, const std::string & value) &
  {
    appendFieldIfNotEmpty(key, value);
    return *this;
  }

  LogEvent && fieldIfNotEmpty(std::string_view key, const std::string & value) &&
  {
    appendFieldIfNotEmpty(key, value);
    return std::move(*this);
  }

  LogEvent & fieldIfNotEmpty(std::string_view key, std::string_view value) &
  {
    appendFieldIfNotEmpty(key, value);
    return *this;
  }

  LogEvent && fieldIfNotEmpty(std::string_view key, std::string_view value) &&
  {
    appendFieldIfNotEmpty(key, value);
    return std::move(*this);
  }

  LogEvent & fieldIfNotEmpty(std::string_view key, const char * value) &
  {
    appendFieldIfNotEmpty(key, value);
    return *this;
  }

  LogEvent && fieldIfNotEmpty(std::string_view key, const char * value) &&
  {
    appendFieldIfNotEmpty(key, value);
    return std::move(*this);
  }

  LogEvent & fieldOr(
    std::string_view key, const std::string & value, std::string_view fallback = kUnknownLogFieldValue) &
  {
    appendFieldOr(key, value, fallback);
    return *this;
  }

  LogEvent && fieldOr(
    std::string_view key, const std::string & value, std::string_view fallback = kUnknownLogFieldValue) &&
  {
    appendFieldOr(key, value, fallback);
    return std::move(*this);
  }

  LogEvent & fieldOr(std::string_view key, std::string_view value, std::string_view fallback = kUnknownLogFieldValue) &
  {
    appendFieldOr(key, value, fallback);
    return *this;
  }

  LogEvent && fieldOr(
    std::string_view key, std::string_view value, std::string_view fallback = kUnknownLogFieldValue) &&
  {
    appendFieldOr(key, value, fallback);
    return std::move(*this);
  }

  LogEvent & fieldOr(std::string_view key, const char * value, std::string_view fallback = kUnknownLogFieldValue) &
  {
    appendFieldOr(key, value, fallback);
    return *this;
  }

  LogEvent && fieldOr(std::string_view key, const char * value, std::string_view fallback = kUnknownLogFieldValue) &&
  {
    appendFieldOr(key, value, fallback);
    return std::move(*this);
  }

  LogEvent & fieldException(std::string_view key, std::exception_ptr exception) &
  {
    appendFieldException(key, std::move(exception));
    return *this;
  }

  LogEvent && fieldException(std::string_view key, std::exception_ptr exception) &&
  {
    appendFieldException(key, std::move(exception));
    return std::move(*this);
  }

  std::string str() const
  {
    return stream_.str();
  }

  void debug() const
  {
    RCLCPP_DEBUG_STREAM(logger_, str());
  }

  void info() const
  {
    RCLCPP_INFO_STREAM(logger_, str());
  }

  void warn() const
  {
    RCLCPP_WARN_STREAM(logger_, str());
  }

  void error() const
  {
    RCLCPP_ERROR_STREAM(logger_, str());
  }

  template <typename Rep, typename Period>
  void warnThrottle(rclcpp::Clock & clock, const std::chrono::duration<Rep, Period> & interval) const
  {
    // ROS throttle macros take integer millisecond periods.
    const auto throttle_interval_ms = std::chrono::duration_cast<std::chrono::milliseconds>(interval).count();
    RCLCPP_WARN_STREAM_THROTTLE(logger_, clock, throttle_interval_ms, str());
  }

private:
  rclcpp::Logger logger_;
  std::ostringstream stream_;

  template <typename T>
  void appendField(std::string_view key, const T & value)
  {
    stream_ << " " << key << "=" << value;
  }

  void appendField(std::string_view key, const char * value)
  {
    stream_ << " " << key << "=" << (value == nullptr ? "<null>" : value);
  }

  template <typename T>
  void appendFieldIf(bool condition, std::string_view key, const T & value)
  {
    if (condition) {
      appendField(key, value);
    }
  }

  void appendFieldIfNotEmpty(std::string_view key, const std::string & value)
  {
    if (!value.empty()) {
      appendField(key, value);
    }
  }

  void appendFieldIfNotEmpty(std::string_view key, std::string_view value)
  {
    if (!value.empty()) {
      appendField(key, value);
    }
  }

  void appendFieldIfNotEmpty(std::string_view key, const char * value)
  {
    if (value != nullptr && value[0] != '\0') {
      appendField(key, value);
    }
  }

  void appendFieldOr(std::string_view key, const std::string & value, std::string_view fallback)
  {
    stream_ << " " << key << "=" << (value.empty() ? fallback : std::string_view(value));
  }

  void appendFieldOr(std::string_view key, std::string_view value, std::string_view fallback)
  {
    stream_ << " " << key << "=" << (value.empty() ? fallback : value);
  }

  void appendFieldOr(std::string_view key, const char * value, std::string_view fallback)
  {
    stream_ << " " << key << "=";
    if (value == nullptr || value[0] == '\0') {
      stream_ << fallback;
    } else {
      stream_ << value;
    }
  }

  void appendFieldException(std::string_view key, std::exception_ptr exception)
  {
    try {
      std::rethrow_exception(exception);
    } catch (const std::exception & exc) {
      appendField(key, exc.what());
    } catch (...) {
      appendField(key, kUnknownExceptionLogFieldValue);
    }
  }
};

}  // namespace livekit_ros2_bridge
