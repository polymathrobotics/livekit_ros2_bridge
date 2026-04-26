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

#include <cstddef>
#include <exception>
#include <memory>
#include <string>
#include <unordered_map>

#include "rcpputils/shared_library.hpp"
#include "ros_interfaces/failure_cache.hpp"
#include "ros_interfaces/message_type_support.hpp"
#include "rosidl_runtime_c/service_type_support_struct.h"

namespace livekit_ros2_bridge::ros_interfaces
{

struct ServiceTypeSupport
{
  explicit ServiceTypeSupport(const std::string & type);

  std::shared_ptr<rcpputils::SharedLibrary> library;
  const rosidl_service_type_support_t * handle;
  MessageTypeSupport request;
  MessageTypeSupport response;
};

class ServiceTypeSupportCache
{
public:
  explicit ServiceTypeSupportCache(std::size_t failure_capacity = 256U);

  std::shared_ptr<ServiceTypeSupport> get(const std::string & type);

private:
  std::unordered_map<std::string, std::shared_ptr<ServiceTypeSupport>> entries_;
  FailureCache failures_;
};

}  // namespace livekit_ros2_bridge::ros_interfaces
