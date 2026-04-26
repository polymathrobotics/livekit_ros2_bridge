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

#include "rclcpp/serialization.hpp"
#include "rcpputils/shared_library.hpp"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_cpp/message_initialization.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

namespace livekit_ros2_bridge::ros_interfaces
{

using MessageMembers = rosidl_typesupport_introspection_cpp::MessageMembers;

class MessageBuffer
{
public:
  MessageBuffer(const MessageMembers & members, rosidl_runtime_cpp::MessageInitialization init);
  ~MessageBuffer();

  MessageBuffer(const MessageBuffer &) = delete;
  MessageBuffer & operator=(const MessageBuffer &) = delete;

  void * data();

private:
  const MessageMembers & members_;
  void * buffer_;
};

struct MessageTypeSupport
{
  // Keep the libraries alive while cached type-support handles may be used.
  explicit MessageTypeSupport(const std::string & type);

  std::shared_ptr<rcpputils::SharedLibrary> serialization_library;
  std::shared_ptr<rcpputils::SharedLibrary> introspection_library;
  const rosidl_message_type_support_t * serialization_handle;
  const rosidl_message_type_support_t * introspection_handle;
  const MessageMembers & members;
  rclcpp::SerializationBase serializer;
};

}  // namespace livekit_ros2_bridge::ros_interfaces
