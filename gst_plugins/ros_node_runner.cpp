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

#include "ros_node_runner.hpp"

RosNodeRunner::RosNodeRunner(const std::string & node_name)
{
  context_ = std::make_shared<rclcpp::Context>();
  context_->init(0, nullptr);

  rclcpp::NodeOptions options;
  options.context(context_);
  node_ = std::make_shared<rclcpp::Node>(node_name, options);

  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context_;
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(executor_options);
  executor_->add_node(node_);

  spin_thread_ = std::thread([this]() { executor_->spin(); });
}

RosNodeRunner::~RosNodeRunner() noexcept
{
  try {
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    executor_->remove_node(node_);
    node_.reset();
    context_->shutdown("RosNodeRunner destroyed");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("RosNodeRunner"), "Exception during teardown: %s", e.what());
  }
}
