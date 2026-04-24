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

#include "protocol/services.hpp"

namespace livekit_ros2_bridge::protocol::services
{

/// Parse a service-call request body into the typed DTO consumed by the ROS runtime.
/// Unknown top-level fields are ignored so the request envelope can grow without revving this
/// parser. Throws `std::invalid_argument` with caller-fixable validation text that the RPC layer
/// surfaces as an invalid-request error.
ServiceCallRequest parse(const std::string & text);

/// Serialize a successful service-call response DTO as the complete RPC response body.
/// The response DTO carries the service metadata that was actually used to execute the request so
/// serialization reflects any name normalization or late interface-type resolution performed
/// downstream.
std::string serialize(const ServiceCallResponse & response);

}  // namespace livekit_ros2_bridge::protocol::services
