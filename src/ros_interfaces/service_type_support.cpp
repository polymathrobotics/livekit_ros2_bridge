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

#include "ros_interfaces/service_type_support.hpp"

#include <stdexcept>

#include "rclcpp/typesupport_helpers.hpp"
#include "rclcpp/version.h"

namespace livekit_ros2_bridge::ros_interfaces
{

namespace
{

constexpr char kSerializationTypeSupportIdentifier[] = "rosidl_typesupport_cpp";
constexpr char kServiceTypeSupportSymbolPrefix[] = "__get_service_type_support_handle__";
constexpr char kRequestMessageTypeSuffix[] = "_Request";
constexpr char kResponseMessageTypeSuffix[] = "_Response";

// rclcpp 28+ (Jazzy) provides get_service_typesupport_handle natively.
// On Humble we load the symbol manually.
#if RCLCPP_VERSION_GTE(28, 0, 0)

const rosidl_service_type_support_t * getHandle(
  const std::string & type, const std::string & typesupport_identifier, rcpputils::SharedLibrary & library)
{
  return rclcpp::get_service_typesupport_handle(type, typesupport_identifier, library);
}

#else

const rosidl_service_type_support_t * getHandle(
  const std::string & type, const std::string & typesupport_identifier, rcpputils::SharedLibrary & library)
{
  std::string symbol = typesupport_identifier + kServiceTypeSupportSymbolPrefix;
  for (const char ch : type) {
    if (ch == '/') {
      symbol += "__";
      continue;
    }
    symbol += ch;
  }

  if (!library.has_symbol(symbol)) {
    throw std::runtime_error("Service typesupport symbol not found: " + symbol);
  }

  using GetServiceTypeSupportHandleFn = const rosidl_service_type_support_t * (*)();
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  auto get_service_type_support_handle = reinterpret_cast<GetServiceTypeSupportHandleFn>(library.get_symbol(symbol));
  return get_service_type_support_handle();
}

#endif

}  // namespace

ServiceTypeSupport::ServiceTypeSupport(const std::string & type)
: library(rclcpp::get_typesupport_library(type, kSerializationTypeSupportIdentifier))
, handle(getHandle(type, kSerializationTypeSupportIdentifier, *library))
, request(type + kRequestMessageTypeSuffix)
, response(type + kResponseMessageTypeSuffix)
{}

ServiceTypeSupportCache::ServiceTypeSupportCache(std::size_t failure_capacity)
: failures_(failure_capacity)
{}

std::shared_ptr<ServiceTypeSupport> ServiceTypeSupportCache::get(const std::string & type)
{
  auto it = entries_.find(type);
  if (it != entries_.end()) {
    return it->second;
  }

  if (const auto failure = failures_.find(type); failure.has_value()) {
    std::rethrow_exception(*failure);
  }

  try {
    return entries_.emplace(type, std::make_shared<ServiceTypeSupport>(type)).first->second;
  } catch (const std::exception & exc) {
    if (
      dynamic_cast<const std::invalid_argument *>(&exc) != nullptr ||
      dynamic_cast<const std::runtime_error *>(&exc) != nullptr)
    {
      failures_.remember(type, std::current_exception());
    }
    throw;
  }
}

}  // namespace livekit_ros2_bridge::ros_interfaces
