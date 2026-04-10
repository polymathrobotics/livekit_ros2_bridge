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

#include "payloads/service_call_payloads.hpp"

#include "nlohmann/json.hpp"
#include "payloads/cdr_payload.hpp"
#include "utils/json_object_parser.hpp"
#include "utils/ros_resource_name_utils.hpp"

namespace livekit_ros2_bridge
{

namespace
{

using Json = nlohmann::json;

}  // namespace

ServiceCallRequest parseServiceCallRequest(const std::string & payload)
{
  const Json json =
    parseJsonObject(payload, "Invalid JSON in service call request", "Service call request must be a JSON object");

  ServiceCallRequest result;

  const std::string normalized_service =
    normalizeRosResourceName(parseRequiredNonEmptyTrimmedStringField(json, "service", "service is required"));
  result.service = normalized_service;

  result.request = parseCdrPayload(json, "request");
  if (result.request.empty()) {
    // Service calls always forward a concrete serialized request message; an empty payload is
    // treated as malformed rather than as a typed default instance.
    throw std::invalid_argument("request.payload_base64 must not be empty");
  }

  const auto requested_interface_type =
    parseOptionalNonEmptyTrimmedStringField(json, "interface_type", "interface_type must be a string");
  if (requested_interface_type) {
    result.interface_type = *requested_interface_type;
  }

  const auto timeout_it = json.find("timeout_ms");
  if (timeout_it != json.end()) {
    if (!timeout_it->is_number_integer()) {
      throw std::invalid_argument("timeout_ms must be an integer");
    }
    result.timeout_ms = timeout_it->get<int>();
  }

  return result;
}

std::string serializeServiceCallResponse(
  const std::string & service,
  const std::string & interface_type,
  const std::vector<std::uint8_t> & response,
  int elapsed_ms)
{
  const Json service_descriptor = {{"name", service}, {"interface_type", interface_type}};
  const Json body = {
    {"ok", true},
    {"service", service_descriptor},
    {"response", serializeCdrPayload(response)},
    {"elapsed_ms", elapsed_ms},
  };
  return body.dump();
}

}  // namespace livekit_ros2_bridge
