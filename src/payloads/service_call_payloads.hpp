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

#include <cstdint>
#include <string>
#include <vector>

namespace livekit_ros2_bridge
{

/// Parsed form of a `ros.services.call` request body.
struct ServiceCallRequest
{
  /// Normalized absolute ROS service name from the required `service` field.
  std::string service;
  /// Optional trimmed `interface_type` hint. Blank or omitted values become empty.
  std::string interface_type;
  /// Raw request CDR bytes from the required non-empty `request` payload object.
  std::vector<std::uint8_t> request;
  /// Optional integer timeout from `timeout_ms`. Zero means the caller did not request a timeout.
  int timeout_ms = 0;
};

/// Parse a JSON object request with required `service` and `request` fields, plus optional
/// `interface_type` and `timeout_ms`. The nested `request` object uses the stable CDR payload
/// schema and must not decode to an empty byte vector.
ServiceCallRequest parseServiceCallRequest(const std::string & payload);

/// Serialize a successful service-call response as
/// `{ "ok": true, "service": { "name", "interface_type" }, "response": <cdr>, "elapsed_ms": ... }`.
std::string serializeServiceCallResponse(
  const std::string & service,
  const std::string & interface_type,
  const std::vector<std::uint8_t> & response,
  int elapsed_ms);

}  // namespace livekit_ros2_bridge
