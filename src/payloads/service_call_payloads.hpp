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
#include <exception>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace livekit_ros2_bridge
{

/// Parsed form of a `ros.services.call` request body.
struct ServiceCallRequest
{
  /// Normalized absolute ROS service name.
  std::string service;
  /// Optional `pkg/srv/Type` hint. Blank or omitted values stay empty so the caller can resolve
  /// the type from the ROS graph.
  std::string interface_type;
  /// Serialized request bytes. Empty payloads are invalid.
  std::vector<std::uint8_t> request_payload;
  /// Parsed `timeout_ms` wire value when present. Callers decide how omitted or non-positive
  /// values map to execution deadlines.
  std::optional<int> timeout_ms;
};

namespace service_call_payloads
{

/// Parse a JSON object request body with required `service` and `request` fields, plus optional
/// `interface_type` and `timeout_ms`. The nested `request` object uses the stable CDR payload
/// object format and must not decode to an empty byte vector.
/// Unknown top-level fields are ignored so the request envelope can grow without revving this
/// parser. Throws `std::invalid_argument` with caller-fixable validation text that the RPC layer
/// surfaces as an invalid-request error.
ServiceCallRequest parse(const std::string & payload);

/// Return the request boundary associated with a service-call validation error when available.
/// This lets higher layers add a precise `request_field` log field without parsing free-form
/// human-readable exception text.
std::optional<std::string_view> invalidRequestField(const std::exception & exc);

/// Serialize a successful service-call response as
/// `{ "ok": true, "service": { "name", "interface_type" }, "response": <cdr>, "elapsed_ms": ... }`.
/// Callers should pass the service metadata that was actually used to execute the request so the
/// response reflects any name normalization or late interface-type resolution performed downstream.
std::string serialize(
  const std::string & service,
  const std::string & interface_type,
  const std::vector<std::uint8_t> & response,
  int elapsed_ms);

inline ServiceCallRequest parseRequest(const std::string & payload)
{
  return parse(payload);
}

inline std::string serializeResponse(
  const std::string & service,
  const std::string & interface_type,
  const std::vector<std::uint8_t> & response,
  int elapsed_ms)
{
  return serialize(service, interface_type, response, elapsed_ms);
}

}  // namespace service_call_payloads

// TODO: Remove these compatibility wrappers once remaining external call sites migrate to
// `service_call_payloads::parse` / `service_call_payloads::serialize`.
inline ServiceCallRequest parseServiceCallRequest(const std::string & payload)
{
  return service_call_payloads::parse(payload);
}

inline std::string serializeServiceCallResponse(
  const std::string & service,
  const std::string & interface_type,
  const std::vector<std::uint8_t> & response,
  int elapsed_ms)
{
  return service_call_payloads::serialize(service, interface_type, response, elapsed_ms);
}

}  // namespace livekit_ros2_bridge
