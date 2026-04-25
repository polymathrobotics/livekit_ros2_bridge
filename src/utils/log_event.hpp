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

constexpr std::string_view kUnknownFieldValue = "<unknown>";
constexpr std::string_view kUnknownExceptionValue = "unknown_exception";

// Builds event=<name> plus ordered key=value fields; keys and values must be log-parser-safe.
class LogEvent
{
public:
  explicit LogEvent(rclcpp::Logger logger, std::string_view name)
  : logger_(std::move(logger))
  {
    message_ << std::boolalpha << "event=" << name;
  }

  LogEvent(const LogEvent &) = delete;
  LogEvent & operator=(const LogEvent &) = delete;
  LogEvent(LogEvent &&) = default;
  LogEvent & operator=(LogEvent &&) = default;

  template <typename T>
  LogEvent & field(std::string_view key, const T & value) &
  {
    message_ << " " << key << "=" << value;
    return *this;
  }

  template <typename T>
  LogEvent && field(std::string_view key, const T & value) &&
  {
    static_cast<LogEvent &>(*this).field(key, value);
    return std::move(*this);
  }

  LogEvent & field(std::string_view key, const char * value) &
  {
    message_ << " " << key << "=" << (value == nullptr ? "<null>" : value);
    return *this;
  }

  LogEvent && field(std::string_view key, const char * value) &&
  {
    static_cast<LogEvent &>(*this).field(key, value);
    return std::move(*this);
  }

  template <typename T>
  LogEvent & fieldIf(bool include, std::string_view key, const T & value) &
  {
    if (include) {
      field(key, value);
    }
    return *this;
  }

  template <typename T>
  LogEvent && fieldIf(bool include, std::string_view key, const T & value) &&
  {
    static_cast<LogEvent &>(*this).fieldIf(include, key, value);
    return std::move(*this);
  }

  LogEvent & fieldIfNotEmpty(std::string_view key, const std::string & value) &
  {
    if (!value.empty()) {
      field(key, value);
    }
    return *this;
  }

  LogEvent && fieldIfNotEmpty(std::string_view key, const std::string & value) &&
  {
    static_cast<LogEvent &>(*this).fieldIfNotEmpty(key, value);
    return std::move(*this);
  }

  LogEvent & fieldIfNotEmpty(std::string_view key, std::string_view value) &
  {
    if (!value.empty()) {
      field(key, value);
    }
    return *this;
  }

  LogEvent && fieldIfNotEmpty(std::string_view key, std::string_view value) &&
  {
    static_cast<LogEvent &>(*this).fieldIfNotEmpty(key, value);
    return std::move(*this);
  }

  LogEvent & fieldIfNotEmpty(std::string_view key, const char * value) &
  {
    if (value != nullptr && value[0] != '\0') {
      field(key, value);
    }
    return *this;
  }

  LogEvent && fieldIfNotEmpty(std::string_view key, const char * value) &&
  {
    static_cast<LogEvent &>(*this).fieldIfNotEmpty(key, value);
    return std::move(*this);
  }

  LogEvent & fieldOr(std::string_view key, const std::string & value, std::string_view fallback = kUnknownFieldValue) &
  {
    message_ << " " << key << "=" << (value.empty() ? fallback : std::string_view(value));
    return *this;
  }

  LogEvent && fieldOr(
    std::string_view key, const std::string & value, std::string_view fallback = kUnknownFieldValue) &&
  {
    static_cast<LogEvent &>(*this).fieldOr(key, value, fallback);
    return std::move(*this);
  }

  LogEvent & fieldOr(std::string_view key, std::string_view value, std::string_view fallback = kUnknownFieldValue) &
  {
    message_ << " " << key << "=" << (value.empty() ? fallback : value);
    return *this;
  }

  LogEvent && fieldOr(std::string_view key, std::string_view value, std::string_view fallback = kUnknownFieldValue) &&
  {
    static_cast<LogEvent &>(*this).fieldOr(key, value, fallback);
    return std::move(*this);
  }

  LogEvent & fieldOr(std::string_view key, const char * value, std::string_view fallback = kUnknownFieldValue) &
  {
    message_ << " " << key << "=";
    if (value == nullptr || value[0] == '\0') {
      message_ << fallback;
    } else {
      message_ << value;
    }
    return *this;
  }

  LogEvent && fieldOr(std::string_view key, const char * value, std::string_view fallback = kUnknownFieldValue) &&
  {
    static_cast<LogEvent &>(*this).fieldOr(key, value, fallback);
    return std::move(*this);
  }

  LogEvent & fieldException(std::string_view key, std::exception_ptr error) &
  {
    try {
      std::rethrow_exception(error);
    } catch (const std::exception & exc) {
      field(key, exc.what());
    } catch (...) {
      field(key, kUnknownExceptionValue);
    }
    return *this;
  }

  LogEvent && fieldException(std::string_view key, std::exception_ptr error) &&
  {
    static_cast<LogEvent &>(*this).fieldException(key, std::move(error));
    return std::move(*this);
  }

  std::string str() const
  {
    return message_.str();
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
    const auto interval_ms = std::chrono::duration_cast<std::chrono::milliseconds>(interval).count();
    RCLCPP_WARN_STREAM_THROTTLE(logger_, clock, interval_ms, str());
  }

private:
  rclcpp::Logger logger_;
  std::ostringstream message_;
};

}  // namespace livekit_ros2_bridge
