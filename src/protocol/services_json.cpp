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

#include "protocol/services_json.hpp"

#include <chrono>
#include <stdexcept>

#include "nlohmann/json.hpp"
#include "protocol/cdr.hpp"
#include "protocol/detail/json_fields.hpp"
#include "protocol/validation_error.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"

namespace livekit_ros2_bridge::protocol::services
{

using Json = nlohmann::json;

namespace
{

constexpr char kServiceField[] = "service";
constexpr char kInterfaceTypeField[] = "interface_type";
constexpr char kRequestField[] = "request";
constexpr char kResponseField[] = "response";
constexpr char kTimeoutMsField[] = "timeout_ms";
constexpr char kPayloadField[] = "payload";
constexpr char kProtocolNodeName[] = "livekit_ros2_bridge";
constexpr char kProtocolNamespace[] = "/";

std::string expandServiceName(const std::string & service)
{
  return rclcpp::expand_topic_or_service_name(service, kProtocolNodeName, kProtocolNamespace, true);
}

}  // namespace

ServiceCallRequest parse(const std::string & text)
{
  Json body;
  try {
    body =
      detail::parseObject(text, "Invalid JSON in service call request", "Service call request must be a JSON object");
  } catch (const std::invalid_argument & exc) {
    throw ValidationError(kPayloadField, exc.what());
  }

  ServiceCallRequest request;
  try {
    // Expand and validate through rclcpp so service-name grammar stays owned by ROS.
    // Protocol service calls are global unless the caller supplies an absolute name.
    request.service = expandServiceName(detail::requiredTrimmedStringField(body, kServiceField, "service is required"));
  } catch (const std::invalid_argument & exc) {
    throw ValidationError(kServiceField, exc.what());
  }

  try {
    // An empty or whitespace-only field means "resolve the type later from the ROS graph" rather
    // than "use an empty interface type".
    request.interface_type =
      detail::optionalTrimmedStringField(body, kInterfaceTypeField, "interface_type must be a string").value_or("");
  } catch (const std::invalid_argument & exc) {
    throw ValidationError(kInterfaceTypeField, exc.what());
  }

  try {
    // Service calls always forward a concrete serialized request message; an empty payload is
    // treated as malformed rather than as a typed default instance.
    request.payload = cdr::parseSerializedMessage(body, cdr::Field::Request);
    if (request.payload.size() == 0U) {
      throw std::invalid_argument("request.payload_base64 must not be empty");
    }
  } catch (const std::invalid_argument & exc) {
    throw ValidationError(kRequestField, exc.what());
  }

  const auto timeout_field = body.find(kTimeoutMsField);
  if (timeout_field != body.end()) {
    if (!timeout_field->is_number_integer()) {
      throw ValidationError(kTimeoutMsField, "timeout_ms must be an integer");
    }

    // Preserve the protocol value as a duration when present; the caller layer decides whether
    // non-positive timeouts fall back to its default deadline.
    request.timeout = std::chrono::milliseconds(timeout_field->get<std::chrono::milliseconds::rep>());
  }

  return request;
}

std::string serialize(const ServiceCallResponse & response)
{
  return Json{
    {kServiceField, response.service},
    {kInterfaceTypeField, response.interface_type},
    {kResponseField, cdr::serialize(response.payload)},
  }
    .dump();
}

}  // namespace livekit_ros2_bridge::protocol::services
