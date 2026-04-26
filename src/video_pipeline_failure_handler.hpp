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
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>

namespace livekit_ros2_bridge
{

// Coalesces repeated GStreamer EOS/error callbacks into one stream-owned action.
class VideoPipelineFailureHandler final
{
public:
  using Action = std::function<void()>;

  VideoPipelineFailureHandler(std::chrono::milliseconds delay, Action action);
  ~VideoPipelineFailureHandler();

  VideoPipelineFailureHandler(const VideoPipelineFailureHandler &) = delete;
  VideoPipelineFailureHandler & operator=(const VideoPipelineFailureHandler &) = delete;
  VideoPipelineFailureHandler(VideoPipelineFailureHandler &&) = delete;
  VideoPipelineFailureHandler & operator=(VideoPipelineFailureHandler &&) = delete;

  bool schedule();
  void clearPending();
  void close();

private:
  void run();

  std::chrono::milliseconds delay_;
  Action action_;
  std::mutex mutex_;
  std::condition_variable condition_;
  bool closed_ = false;
  bool pending_ = false;
  bool running_ = false;
  std::thread worker_;
};

}  // namespace livekit_ros2_bridge
