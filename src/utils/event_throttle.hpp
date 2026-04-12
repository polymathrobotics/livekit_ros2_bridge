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
#include <functional>
#include <utility>

namespace livekit_ros2_bridge
{

constexpr std::size_t kNoSuppressedEvents = 0U;
const auto kNoNextFireScheduled = std::chrono::steady_clock::time_point{};

class EventThrottle
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;
  using TimePointSource = std::function<TimePoint()>;

  explicit EventThrottle(std::chrono::steady_clock::duration interval)
  : EventThrottle(interval, []() { return std::chrono::steady_clock::now(); })
  {}

  explicit EventThrottle(std::chrono::steady_clock::duration interval, TimePointSource now)
  : interval_(interval)
  , now_(std::move(now))
  {}

  std::size_t recordAndTakePendingCount()
  {
    ++count_;
    const auto now = now_();
    const bool should_fire_now = next_fire_at_ == kNoNextFireScheduled || now >= next_fire_at_;
    if (should_fire_now) {
      const std::size_t fired_count = count_;
      count_ = kNoSuppressedEvents;
      next_fire_at_ = now + interval_;
      return fired_count;
    }
    return kNoSuppressedEvents;
  }

private:
  std::chrono::steady_clock::duration interval_;
  TimePointSource now_;
  std::size_t count_ = kNoSuppressedEvents;
  std::chrono::steady_clock::time_point next_fire_at_{kNoNextFireScheduled};
};

}  // namespace livekit_ros2_bridge
