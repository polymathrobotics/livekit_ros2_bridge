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

#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>

// Owns a ROS node with a fully isolated context, executor, and spin thread for
// one plugin instance. Construction starts spinning immediately, and teardown
// stops that thread before the runner-owned context is shut down.
class RosNodeRunner
{
public:
  // Starts the isolated ROS context and executor thread before returning.
  explicit RosNodeRunner(const std::string & node_name);
  ~RosNodeRunner() noexcept;

  RosNodeRunner(const RosNodeRunner &) = delete;
  RosNodeRunner & operator=(const RosNodeRunner &) = delete;

  // Returns the node bound to this runner's isolated context and executor.
  // Callers must not retain it past the runner lifetime.
  rclcpp::Node::SharedPtr node() const
  {
    return node_;
  }

private:
  // These members share one isolated ROS graph that is torn down with this
  // runner and never shared across plugin instances.
  rclcpp::Context::SharedPtr context_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;
};
