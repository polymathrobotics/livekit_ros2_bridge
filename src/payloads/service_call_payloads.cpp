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

#include <exception>
#include <stdexcept>
#include <string_view>

#include "nlohmann/json.hpp"
#include "payloads/cdr_payload.hpp"
#include "payloads/json_object_parser.hpp"
#include "utils/ros_resource_name_utils.hpp"

namespace livekit_ros2_bridge
{

namespace service_call_payloads
{

namespace
{

class ServiceCallInvalidArgument final : public std::invalid_argument
{
public:
  ServiceCallInvalidArgument(std::string_view field_name, const char * message)
  : std::invalid_argument(message)
  , field_name_(field_name)
  {}

  std::string_view fieldName() const noexcept
  {
    return field_name_;
  }

private:
  std::string_view field_name_;
};

[[noreturn]] void throwInvalidRequestField(std::string_view field_name, const char * message)
{
  throw ServiceCallInvalidArgument(field_name, message);
}

}  // namespace

ServiceCallRequest parse(const std::string & payload)
{
  const nlohmann::json body = [&payload]() {
    try {
      return parseJsonObject(
        payload, "Invalid JSON in service call request", "Service call request must be a JSON object");
    } catch (const std::invalid_argument & exc) {
      throw ServiceCallInvalidArgument("payload", exc.what());
    }
  }();

  ServiceCallRequest request;
  // Canonicalize at the protocol boundary so policy checks and downstream caches do not have to
  // reason about multiple spellings of the same ROS service name.
  try {
    request.service =
      normalizeRosResourceName(parseRequiredNonEmptyTrimmedStringField(body, "service", "service is required"));
  } catch (const std::invalid_argument & exc) {
    throw ServiceCallInvalidArgument("service", exc.what());
  }

  try {
    if (
      const auto interface_type =
        parseOptionalNonEmptyTrimmedStringField(body, "interface_type", "interface_type must be a string"))
    {
      // An empty or whitespace-only field means "resolve the type later from the ROS graph" rather
      // than "use an empty interface type".
      request.interface_type = *interface_type;
    }
  } catch (const std::invalid_argument & exc) {
    throw ServiceCallInvalidArgument("interface_type", exc.what());
  }

  // Service calls always forward a concrete serialized request message; an empty payload is
  // treated as malformed rather than as a typed default instance.
  try {
    request.request_payload = cdr_payload::parse(body, "request");
  } catch (const std::invalid_argument & exc) {
    throw ServiceCallInvalidArgument("request", exc.what());
  }
  if (request.request_payload.empty()) {
    throwInvalidRequestField("request", "request.payload_base64 must not be empty");
  }

  if (const auto timeout_field = body.find("timeout_ms"); timeout_field != body.end()) {
    if (!timeout_field->is_number_integer()) {
      throwInvalidRequestField("timeout_ms", "timeout_ms must be an integer");
    }
    // Preserve the wire value as-is when present; the caller layer decides whether non-positive
    // timeouts fall back to its default deadline.
    request.timeout_ms = timeout_field->get<int>();
  }

  return request;
}

std::optional<std::string_view> invalidRequestField(const std::exception & exc)
{
  if (const auto * service_call_error = dynamic_cast<const ServiceCallInvalidArgument *>(&exc)) {
    return service_call_error->fieldName();
  }

  return std::nullopt;
}

std::string serialize(
  const std::string & service,
  const std::string & interface_type,
  const std::vector<std::uint8_t> & response,
  int elapsed_ms)
{
  return nlohmann::json{
    {"ok", true},
    {"service", nlohmann::json{{"name", service}, {"interface_type", interface_type}}},
    {"response", cdr_payload::serialize(response)},
    {"elapsed_ms", elapsed_ms},
  }
    .dump();
}

}  // namespace service_call_payloads

}  // namespace livekit_ros2_bridge
