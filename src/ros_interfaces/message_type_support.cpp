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

#include "ros_interfaces/message_type_support.hpp"

#include <new>
#include <stdexcept>

#include "rclcpp/typesupport_helpers.hpp"
#include "rclcpp/version.h"

// rclcpp 28+ (Jazzy) renamed get_typesupport_handle -> get_message_typesupport_handle.
#if !RCLCPP_VERSION_GTE(28, 0, 0)
namespace rclcpp
{
inline const rosidl_message_type_support_t * get_message_typesupport_handle(
  const std::string & type, const std::string & typesupport_identifier, rcpputils::SharedLibrary & library)
{
  return get_typesupport_handle(type, typesupport_identifier, library);
}
}  // namespace rclcpp
#endif

namespace livekit_ros2_bridge::ros_interfaces
{

namespace
{

constexpr char kSerializationTypeSupportIdentifier[] = "rosidl_typesupport_cpp";
constexpr char kIntrospectionTypeSupportIdentifier[] = "rosidl_typesupport_introspection_cpp";

const MessageMembers & requireMembers(const rosidl_message_type_support_t * introspection)
{
  if (introspection == nullptr || introspection->data == nullptr) {
    throw std::runtime_error("Introspection type support handle is null");
  }
  return *static_cast<const MessageMembers *>(introspection->data);
}

}  // namespace

MessageBuffer::MessageBuffer(const MessageMembers & members, rosidl_runtime_cpp::MessageInitialization init)
: members_(members)
, buffer_(::operator new(members.size_of_))
{
  members_.init_function(buffer_, init);
}

MessageBuffer::~MessageBuffer()
{
  members_.fini_function(buffer_);
  ::operator delete(buffer_);
}

void * MessageBuffer::data()
{
  return buffer_;
}

MessageTypeSupport::MessageTypeSupport(const std::string & type)
: serialization_library(rclcpp::get_typesupport_library(type, kSerializationTypeSupportIdentifier))
, introspection_library(rclcpp::get_typesupport_library(type, kIntrospectionTypeSupportIdentifier))
, serialization_handle(
    rclcpp::get_message_typesupport_handle(type, kSerializationTypeSupportIdentifier, *serialization_library))
, introspection_handle(
    rclcpp::get_message_typesupport_handle(type, kIntrospectionTypeSupportIdentifier, *introspection_library))
, members(requireMembers(introspection_handle))
, serializer(serialization_handle)
{}

}  // namespace livekit_ros2_bridge::ros_interfaces
