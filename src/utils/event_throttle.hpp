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

  // Returns the pending count on the first call and after each interval; otherwise returns 0.
  std::size_t record()
  {
    ++pending_;
    const auto now = std::chrono::steady_clock::now();
    if (next_allowed_at_ == TimePoint{} || now >= next_allowed_at_) {
      const std::size_t pending = pending_;
      pending_ = 0U;
      next_allowed_at_ = now + interval_;
      return pending;
    }
    return 0U;
  }

private:
  using TimePoint = std::chrono::steady_clock::time_point;

  std::chrono::steady_clock::duration interval_;
  std::size_t pending_ = 0U;
  TimePoint next_allowed_at_{};
};

}  // namespace livekit_ros2_bridge
