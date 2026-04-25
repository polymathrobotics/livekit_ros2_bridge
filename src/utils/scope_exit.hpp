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

#ifndef LIVEKIT_ROS2_BRIDGE__SCOPE_EXIT_HPP_
#define LIVEKIT_ROS2_BRIDGE__SCOPE_EXIT_HPP_

#include <utility>

namespace livekit_ros2_bridge
{

template <typename Callback>
class ScopeExit final
{
public:
  // Callbacks must not throw; destructor exceptions terminate.
  explicit ScopeExit(Callback callback)
  : callback_(std::move(callback))
  {}

  ~ScopeExit()
  {
    callback_();
  }

  ScopeExit(const ScopeExit &) = delete;
  ScopeExit & operator=(const ScopeExit &) = delete;
  ScopeExit(ScopeExit &&) = delete;
  ScopeExit & operator=(ScopeExit &&) = delete;

private:
  Callback callback_;
};

template <typename Callback>
ScopeExit(Callback) -> ScopeExit<Callback>;

}  // namespace livekit_ros2_bridge

#endif
