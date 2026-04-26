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

#include <stdexcept>

#include "gtest/gtest.h"
#include "ros_interfaces/message_type_support.hpp"
#include "ros_interfaces/service_type_support.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"
#include "rosidl_runtime_cpp/traits.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace livekit_ros2_bridge::ros_interfaces
{

namespace
{

const char * setBoolServiceType()
{
  return rosidl_generator_traits::name<std_srvs::srv::SetBool>();
}

TEST(ServiceTypeSupportTest, LoadsServiceAndRequestResponseMessageSupport)
{
  const ServiceTypeSupport support(setBoolServiceType());

  EXPECT_NE(support.handle, nullptr);
  EXPECT_NE(support.request.serialization_handle, nullptr);
  EXPECT_NE(support.request.introspection_handle, nullptr);
  EXPECT_NE(support.response.serialization_handle, nullptr);
  EXPECT_NE(support.response.introspection_handle, nullptr);

  MessageBuffer request(support.request.members, rosidl_runtime_cpp::MessageInitialization::ZERO);
  MessageBuffer response(support.response.members, rosidl_runtime_cpp::MessageInitialization::ZERO);
  EXPECT_NE(request.data(), nullptr);
  EXPECT_NE(response.data(), nullptr);
}

TEST(ServiceTypeSupportCacheTest, ReturnsCachedSupportForRepeatedType)
{
  ServiceTypeSupportCache cache;

  const auto first = cache.get(setBoolServiceType());
  const auto second = cache.get(setBoolServiceType());

  EXPECT_EQ(first, second);
}

TEST(ServiceTypeSupportCacheTest, CachesInvalidTypeFailures)
{
  ServiceTypeSupportCache cache;

  EXPECT_THROW(static_cast<void>(cache.get("missing_pkg/srv/Missing")), std::runtime_error);
  EXPECT_THROW(static_cast<void>(cache.get("missing_pkg/srv/Missing")), std::runtime_error);
}

}  // namespace

}  // namespace livekit_ros2_bridge::ros_interfaces
