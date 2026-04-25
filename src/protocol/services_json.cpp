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

constexpr char kService[] = "service";
constexpr char kInterfaceType[] = "interface_type";
constexpr char kRequest[] = "request";
constexpr char kResponse[] = "response";
constexpr char kTimeoutMs[] = "timeout_ms";
constexpr char kPayload[] = "payload";

}  // namespace

ServiceCallRequest parse(const std::string & payload)
{
  Json body;
  try {
    body = detail::parseObject(
      payload, "Invalid JSON in service call request", "Service call request must be a JSON object");
  } catch (const std::invalid_argument & exc) {
    throw ValidationError(kPayload, exc.what());
  }

  ServiceCallRequest request;
  try {
    // rclcpp owns service-name grammar; the protocol namespace resolves relatives globally.
    request.name = rclcpp::expand_topic_or_service_name(
      detail::requiredString(body, kService, "service is required"), "livekit_ros2_bridge", "/", true);
  } catch (const std::invalid_argument & exc) {
    throw ValidationError(kService, exc.what());
  }

  try {
    request.interface_type =
      detail::optionalString(body, kInterfaceType, "interface_type must be a string").value_or("");
  } catch (const std::invalid_argument & exc) {
    throw ValidationError(kInterfaceType, exc.what());
  }

  try {
    // Empty requests would imply a typed default, but this boundary only forwards serialized ROS data.
    request.payload = cdr::parse(body, cdr::Field::Request);
    if (request.payload.size() == 0U) {
      throw std::invalid_argument("request.payload_base64 must not be empty");
    }
  } catch (const std::invalid_argument & exc) {
    throw ValidationError(kRequest, exc.what());
  }

  const auto timeout = body.find(kTimeoutMs);
  if (timeout != body.end()) {
    if (!timeout->is_number_integer()) {
      throw ValidationError(kTimeoutMs, "timeout_ms must be an integer");
    }

    // Preserve explicit non-positive values; the runtime owns deadline policy.
    request.timeout = std::chrono::milliseconds(timeout->get<std::chrono::milliseconds::rep>());
  }

  return request;
}

std::string serialize(const ServiceCallResponse & response)
{
  return Json{
    {kService, response.name},
    {kInterfaceType, response.interface_type},
    {kResponse, cdr::serialize(response.payload)},
  }
    .dump();
}

}  // namespace livekit_ros2_bridge::protocol::services
