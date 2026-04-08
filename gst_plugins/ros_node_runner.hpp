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

class RosNodeRunner
{
public:
  explicit RosNodeRunner(const std::string & node_name);
  ~RosNodeRunner() noexcept;

  RosNodeRunner(const RosNodeRunner &) = delete;
  RosNodeRunner & operator=(const RosNodeRunner &) = delete;

  rclcpp::Node::SharedPtr node() const
  {
    return node_;
  }

private:
  rclcpp::Context::SharedPtr context_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;
};
