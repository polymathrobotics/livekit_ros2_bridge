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

#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/node_interfaces/node_services_interface.hpp"
#include "rclcpp/node_interfaces/node_timers_interface.hpp"
#include "rclcpp/node_interfaces/node_topics_interface.hpp"
#include "rclcpp/node_interfaces/node_waitables_interface.hpp"

namespace livekit_ros2_bridge
{

struct ExecutorNodeInterfaces
{
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base;
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr waitables;
  rclcpp::Clock::SharedPtr clock;
};

struct PublisherNodeInterfaces
{
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph;
  rclcpp::Clock::SharedPtr clock;
};

struct ServiceNodeInterfaces
{
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph;
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr timers;
  rclcpp::Clock::SharedPtr clock;
};

struct SubscriptionNodeInterfaces
{
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph;
  rclcpp::Clock::SharedPtr clock;
};

struct GraphNodeInterfaces
{
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph;
  rclcpp::Clock::SharedPtr clock;
  rclcpp::Logger logger;
};

struct TimerNodeInterfaces
{
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base;
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr timers;
};

struct RosNodeInterfaces
{
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr base;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters;
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr graph;
  rclcpp::node_interfaces::NodeServicesInterface::SharedPtr services;
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr timers;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr topics;
  rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr waitables;
  rclcpp::Clock::SharedPtr clock;
  rclcpp::Logger logger;

  ExecutorNodeInterfaces executor() const
  {
    return {base, waitables, clock};
  }

  PublisherNodeInterfaces publisher() const
  {
    return {topics, graph, clock};
  }

  ServiceNodeInterfaces service() const
  {
    return {base, graph, timers, clock};
  }

  SubscriptionNodeInterfaces subscription() const
  {
    return {parameters, topics, graph, clock};
  }

  GraphNodeInterfaces graphOnly() const
  {
    return {graph, clock, logger};
  }

  TimerNodeInterfaces timer() const
  {
    return {base, timers};
  }
};

inline RosNodeInterfaces makeRosNodeInterfaces(rclcpp::Node & node)
{
  return {
    node.get_node_base_interface(),
    node.get_node_parameters_interface(),
    node.get_node_graph_interface(),
    node.get_node_services_interface(),
    node.get_node_timers_interface(),
    node.get_node_topics_interface(),
    node.get_node_waitables_interface(),
    node.get_clock(),
    node.get_logger(),
  };
}

}  // namespace livekit_ros2_bridge
