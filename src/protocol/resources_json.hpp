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

#include <string>

#include "protocol/resources.hpp"

namespace livekit_ros2_bridge::protocol::resources
{

/// Parse a resource-list request body.
/// Throws `std::invalid_argument` when the payload is not a JSON object or `query` / `limit`
/// violate the RPC contract. Missing, null, and blank queries normalize to "no filter".
ResourceListRequest parseRequest(const std::string & payload);

/// Serialize services as `{ "services": [{ "service", "interface_type" }, ...] }` in ROS graph order.
/// Entries without exactly one interface type are omitted to preserve the single `interface_type`
/// protocol shape. Callers must pre-filter results and apply any limit before calling this response
/// serializer.
std::string serializeServices(const ResourceNamesAndTypes & resources_by_name);

/// Serialize topics as `{ "topics": [{ "topic", "interface_type" }, ...] }` in ROS graph order.
/// Entries without exactly one interface type are omitted to preserve the single `interface_type`
/// protocol shape. Callers must pre-filter results and apply any limit before calling this response
/// serializer.
std::string serializeTopics(const ResourceNamesAndTypes & resources_by_name);

}  // namespace livekit_ros2_bridge::protocol::resources
