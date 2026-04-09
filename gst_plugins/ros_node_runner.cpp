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

namespace
{

rclcpp::Logger ros_node_runner_logger()
{
  return rclcpp::get_logger("RosNodeRunner");
}

}  // namespace

RosNodeRunner::RosNodeRunner(const std::string & node_name)
: node_name_(node_name)
{
  const char * step = "context_init";
  try {
    context_ = std::make_shared<rclcpp::Context>();
    context_->init(0, nullptr);

    step = "node_create";
    rclcpp::NodeOptions options;
    options.context(context_);
    node_ = std::make_shared<rclcpp::Node>(node_name, options);

    step = "executor_create";
    rclcpp::ExecutorOptions executor_options;
    executor_options.context = context_;
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(executor_options);

    step = "executor_add_node";
    executor_->add_node(node_);

    // Start spinning during construction so element setup can use the runner's
    // node immediately without relying on any process-global executor.
    step = "thread_start";
    spin_thread_ = std::thread([this]() {
      try {
        executor_->spin();
      } catch (const std::exception & e) {
        RCLCPP_ERROR(
          ros_node_runner_logger(),
          "event=ros_node_runner_spin_failed node_name=%s phase=spin step=executor_spin reason=%s",
          node_name_.c_str(),
          e.what());
      } catch (...) {
        RCLCPP_ERROR(
          ros_node_runner_logger(),
          "event=ros_node_runner_spin_failed node_name=%s phase=spin step=executor_spin reason=unknown_exception",
          node_name_.c_str());
      }
    });
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      ros_node_runner_logger(),
      "event=ros_node_runner_start_failed node_name=%s phase=startup step=%s reason=%s",
      node_name_.c_str(),
      step,
      e.what());
    throw;
  } catch (...) {
    RCLCPP_ERROR(
      ros_node_runner_logger(),
      "event=ros_node_runner_start_failed node_name=%s phase=startup step=%s reason=unknown_exception",
      node_name_.c_str(),
      step);
    throw;
  }
}

RosNodeRunner::~RosNodeRunner() noexcept
{
  const char * step = "executor_cancel";

  try {
    // Tear down in reverse execution order so the spin loop exits before the
    // node is removed and the isolated context is shut down.
    executor_->cancel();

    step = "thread_join";
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }

    step = "executor_remove_node";
    executor_->remove_node(node_);

    step = "node_reset";
    node_.reset();

    step = "context_shutdown";
    context_->shutdown("RosNodeRunner destroyed");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      ros_node_runner_logger(),
      "event=ros_node_runner_teardown_failed node_name=%s phase=teardown step=%s reason=%s",
      node_name_.c_str(),
      step,
      e.what());
  } catch (...) {
    RCLCPP_ERROR(
      ros_node_runner_logger(),
      "event=ros_node_runner_teardown_failed node_name=%s phase=teardown step=%s reason=unknown_exception",
      node_name_.c_str(),
      step);
  }
}
