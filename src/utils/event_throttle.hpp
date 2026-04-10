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

namespace livekit_ros2_bridge
{

class EventThrottle
{
public:
  explicit EventThrottle(std::chrono::steady_clock::duration interval)
  : interval_(interval)
  {}

  std::size_t recordAndCheck()
  {
    ++count_;
    const auto now = std::chrono::steady_clock::now();
    const bool should_fire_now = next_fire_at_ == std::chrono::steady_clock::time_point{} || now >= next_fire_at_;
    if (should_fire_now) {
      const std::size_t fired_count = count_;
      count_ = 0U;
      next_fire_at_ = now + interval_;
      return fired_count;
    }
    return 0U;
  }

private:
  std::chrono::steady_clock::duration interval_;
  std::size_t count_ = 0U;
  std::chrono::steady_clock::time_point next_fire_at_{};
};

}  // namespace livekit_ros2_bridge
