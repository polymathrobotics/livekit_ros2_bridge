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
#include <string>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "utils/log_event.hpp"

namespace livekit_ros2_bridge
{

namespace
{

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

}  // namespace

TEST(LogEventTest, BuildsStructuredMessageInFieldOrder)
{
  const std::string message = LogEvent(rclcpp::get_logger("log_event_test"), "sample_event")
                                .kv("count", 3U)
                                .kv("success", true)
                                .kv("status", "ok")
                                .str();

  EXPECT_EQ(message, "event=sample_event count=3 success=true status=ok");
}

TEST(LogEventTest, KvOrUsesFallbackForEmptyValues)
{
  const std::string message = LogEvent(rclcpp::get_logger("log_event_test"), "sample_event")
                                .kvOr("missing", std::string{}, "<unset>")
                                .kvOr("present", std::string("value"))
                                .str();

  EXPECT_EQ(message, "event=sample_event missing=<unset> present=value");
}

TEST(LogEventTest, WarnThrottleSmokeTest)
{
  ScopedRclcppInit init;
  rclcpp::Clock clock(RCL_SYSTEM_TIME);

  LogEvent(rclcpp::get_logger("log_event_test"), "sample_event")
    .kv("detail", "value")
    .warnThrottle(clock, std::chrono::milliseconds(0));

  SUCCEED();
}

}  // namespace livekit_ros2_bridge
