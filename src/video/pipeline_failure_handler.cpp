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

#include "video/pipeline_failure_handler.hpp"

#include <utility>

namespace livekit_ros2_bridge::video
{

PipelineFailureHandler::PipelineFailureHandler(std::chrono::milliseconds delay, Callback callback)
: delay_(delay)
, callback_(std::move(callback))
{}

PipelineFailureHandler::~PipelineFailureHandler()
{
  close();
}

bool PipelineFailureHandler::schedule()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (closed_ || pending_ || callback_running_) {
    return false;
  }

  if (!worker_.joinable()) {
    worker_ = std::thread([this]() { run(); });
  }
  pending_ = true;
  condition_.notify_one();
  return true;
}

void PipelineFailureHandler::cancelPending()
{
  std::lock_guard<std::mutex> lock(mutex_);
  pending_ = false;
  condition_.notify_all();
}

void PipelineFailureHandler::close()
{
  std::thread worker;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {
      return;
    }

    closed_ = true;
    condition_.notify_all();
    worker = std::move(worker_);
  }

  if (worker.joinable()) {
    worker.join();
  }
}

void PipelineFailureHandler::run()
{
  std::unique_lock<std::mutex> lock(mutex_);
  while (true) {
    condition_.wait(lock, [this]() { return closed_ || pending_; });
    if (closed_) {
      return;
    }

    if (
      delay_ > std::chrono::milliseconds::zero() &&
      condition_.wait_for(lock, delay_, [this]() { return closed_ || !pending_; }))
    {
      if (closed_) {
        return;
      }
      continue;
    }
    if (!pending_) {
      continue;
    }

    callback_running_ = true;
    lock.unlock();
    callback_();
    lock.lock();
    callback_running_ = false;
    pending_ = false;
  }
}

}  // namespace livekit_ros2_bridge::video
