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

#include "wire/services.hpp"

#include <exception>
#include <stdexcept>
#include <string_view>

#include "nlohmann/json.hpp"
#include "utils/ros_resource_name_utils.hpp"
#include "wire/cdr.hpp"
#include "wire/detail/json_object_parser.hpp"

namespace livekit_ros2_bridge
{

using Json = nlohmann::json;

namespace wire::services
{

namespace
{

class InvalidFieldArgument final : public std::invalid_argument
{
public:
  InvalidFieldArgument(std::string_view field, const char * message)
  : std::invalid_argument(message)
  , field_(field)
  {}

  std::string_view field() const noexcept
  {
    return field_;
  }

private:
  std::string_view field_;
};

std::string parseService(const Json & body)
{
  // Canonicalize at the protocol boundary so policy checks and downstream caches do not have to
  // reason about multiple spellings of the same ROS service name.
  try {
    return normalizeRosResourceName(
      wire::detail::parseRequiredNonEmptyTrimmedStringField(body, "service", "service is required"));
  } catch (const std::invalid_argument & exc) {
    throw InvalidFieldArgument("service", exc.what());
  }
}

std::string parseInterfaceType(const Json & body)
{
  // An empty or whitespace-only field means "resolve the type later from the ROS graph" rather
  // than "use an empty interface type".
  try {
    return wire::detail::parseOptionalNonEmptyTrimmedStringField(
             body, "interface_type", "interface_type must be a string")
      .value_or("");
  } catch (const std::invalid_argument & exc) {
    throw InvalidFieldArgument("interface_type", exc.what());
  }
}

std::vector<std::uint8_t> parseRequestPayload(const Json & body)
{
  // Service calls always forward a concrete serialized request message; an empty payload is
  // treated as malformed rather than as a typed default instance.
  try {
    auto request_payload = cdr::parse(body, "request");
    if (request_payload.empty()) {
      throw std::invalid_argument("request.payload_base64 must not be empty");
    }
    return request_payload;
  } catch (const std::invalid_argument & exc) {
    throw InvalidFieldArgument("request", exc.what());
  }
}

std::optional<int> parseTimeoutMs(const Json & body)
{
  const auto timeout_it = body.find("timeout_ms");
  if (timeout_it == body.end()) {
    return std::nullopt;
  }

  if (!timeout_it->is_number_integer()) {
    throw InvalidFieldArgument("timeout_ms", "timeout_ms must be an integer");
  }

  // Preserve the wire value as-is when present; the caller layer decides whether non-positive
  // timeouts fall back to its default deadline.
  return timeout_it->get<int>();
}

}  // namespace

ServiceCallRequest parse(const std::string & payload)
{
  Json body;
  try {
    body = wire::detail::parseJsonObject(
      payload, "Invalid JSON in service call request", "Service call request must be a JSON object");
  } catch (const std::invalid_argument & exc) {
    throw InvalidFieldArgument("payload", exc.what());
  }

  return ServiceCallRequest{
    parseService(body),
    parseInterfaceType(body),
    parseRequestPayload(body),
    parseTimeoutMs(body),
  };
}

std::optional<std::string_view> invalidRequestField(const std::exception & error)
{
  if (const auto * field_error = dynamic_cast<const InvalidFieldArgument *>(&error); field_error != nullptr) {
    return field_error->field();
  }

  return std::nullopt;
}

std::string serialize(
  const std::string & service,
  const std::string & interface_type,
  const std::vector<std::uint8_t> & response,
  int elapsed_ms)
{
  return Json{
    {"ok", true},
    {"service", Json{{"name", service}, {"interface_type", interface_type}}},
    {"response", cdr::serialize(response)},
    {"elapsed_ms", elapsed_ms},
  }
    .dump();
}

}  // namespace wire::services

}  // namespace livekit_ros2_bridge
