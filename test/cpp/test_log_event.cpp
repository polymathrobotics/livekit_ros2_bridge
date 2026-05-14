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

#include <chrono>
#include <exception>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>

#include "gtest/gtest.h"
#include "ros_test_support.hpp"
#include "utils/log_event.hpp"

namespace livekit_ros2_bridge
{

namespace
{

enum class SampleLogEnum
{
  FirstValue,
  SECOND_VALUE,
};

}  // namespace

TEST(LogEventTest, BuildsStructuredMessageWithFallbacksInFieldOrder)
{
  EXPECT_EQ(
    LogEvent(rclcpp::get_logger("log_event_test"), "sample_event")
      .field("count", 3U)
      .field("success", true)
      .field("status", "ok")
      .fieldOr("missing", std::string{}, "<unset>")
      .fieldOr("present", std::string{"value"})
      .str(),
    "event=sample_event count=3 success=true status=ok missing=<unset> present=value");
}

TEST(LogEventTest, UsesDefaultFallbacksForNullAndEmptyFieldInputs)
{
  const char * const null_value = nullptr;
  const char * const empty_cstr = "";

  EXPECT_EQ(
    LogEvent(rclcpp::get_logger("log_event_test"), "sample_event")
      .field("nullable", null_value)
      .fieldOr("empty_string", std::string{})
      .fieldOr("empty_view", std::string_view{})
      .fieldOr("empty_cstr", empty_cstr)
      .fieldOr("null_cstr", null_value)
      .str(),
    "event=sample_event nullable=<null> empty_string=<unknown> empty_view=<unknown> "
    "empty_cstr=<unknown> null_cstr=<unknown>");
}

TEST(LogEventTest, UsesCustomFallbacksForStringViewAndCStringInputs)
{
  const char * const null_value = nullptr;

  EXPECT_EQ(
    LogEvent(rclcpp::get_logger("log_event_test"), "sample_event")
      .fieldOr("empty_view", std::string_view{}, "<missing>")
      .fieldOr("null_cstr", null_value, "<missing>")
      .fieldOr("present_view", std::string_view{"ready"}, "<missing>")
      .fieldOr("present_cstr", "ok", "<missing>")
      .str(),
    "event=sample_event empty_view=<missing> null_cstr=<missing> present_view=ready "
    "present_cstr=ok");
}

TEST(LogEventTest, UsesFallbacksForOptionalFieldInputs)
{
  const std::optional<int> missing_count;
  const std::optional<int> present_count = 7;
  const std::optional<std::string> empty_name = std::string{};
  const std::optional<std::string> present_name = std::string{"ready"};

  EXPECT_EQ(
    LogEvent(rclcpp::get_logger("log_event_test"), "sample_event")
      .fieldOr("missing_count", missing_count, "<missing>")
      .fieldOr("present_count", present_count, "<missing>")
      .fieldOr("empty_name", empty_name, "<missing>")
      .fieldOr("present_name", present_name, "<missing>")
      .str(),
    "event=sample_event missing_count=<missing> present_count=7 empty_name=<missing> present_name=ready");
}

TEST(LogEventTest, UsesMagicEnumNamesForEnumFields)
{
  const std::optional<SampleLogEnum> missing_value;
  const std::optional<SampleLogEnum> present_value = SampleLogEnum::SECOND_VALUE;

  EXPECT_EQ(
    LogEvent(rclcpp::get_logger("log_event_test"), "sample_event")
      .fieldEnum("direct", SampleLogEnum::FirstValue)
      .fieldEnumOr("missing", missing_value, "<missing>")
      .fieldEnumOr("present", present_value, "<missing>")
      .str(),
    "event=sample_event direct=FirstValue missing=<missing> present=SECOND_VALUE");
}

TEST(LogEventTest, FormatsExceptionMessagesAndFallbacks)
{
  const auto exception = std::make_exception_ptr(std::runtime_error("boom"));
  const auto non_std_exception = std::make_exception_ptr(7);
  const std::exception_ptr missing_exception;

  EXPECT_EQ(
    LogEvent(rclcpp::get_logger("log_event_test"), "sample_event").fieldException("error", exception).str(),
    "event=sample_event error=boom");
  EXPECT_EQ(
    LogEvent(rclcpp::get_logger("log_event_test"), "sample_event").fieldException("error", non_std_exception).str(),
    "event=sample_event error=unknown_exception");
  EXPECT_EQ(
    LogEvent(rclcpp::get_logger("log_event_test"), "sample_event").fieldException("error", missing_exception).str(),
    "event=sample_event error=unknown_exception");
}

TEST(LogEventTest, SupportsConditionalChainableFields)
{
  const char * const null_value = nullptr;

  EXPECT_EQ(
    LogEvent(rclcpp::get_logger("log_event_test"), "sample_event")
      .field("required", true)
      .fieldIf(false, "skipped", 7)
      .fieldIf(true, "count", 3)
      .fieldIfNotEmpty("present_string", std::string{"value"})
      .fieldIfNotEmpty("empty_string", std::string{})
      .fieldIfNotEmpty("present_view", std::string_view{"ready"})
      .fieldIfNotEmpty("empty_view", std::string_view{})
      .fieldIfNotEmpty("present_cstr", "ok")
      .fieldIfNotEmpty("null_cstr", null_value)
      .str(),
    "event=sample_event required=true count=3 present_string=value present_view=ready "
    "present_cstr=ok");
}

TEST(LogEventTest, WarnThrottleUsesChronoIntervals)
{
  test_support::ScopedRclcppInit init;
  rclcpp::Clock clock(RCL_SYSTEM_TIME);
  const auto interval = std::chrono::microseconds(999);

  EXPECT_NO_THROW(LogEvent(rclcpp::get_logger("log_event_test"), "sample_event").warnThrottle(clock, interval));
}

}  // namespace livekit_ros2_bridge
