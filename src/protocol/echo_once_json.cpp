// Copyright 2025 Polymath Robotics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "protocol/echo_once_json.hpp"

#include <exception>
#include <stdexcept>
#include <string>

#include "nlohmann/json.hpp"
#include "protocol/detail/json_fields.hpp"
#include "protocol/validation_error.hpp"
#include "rclcpp/expand_topic_or_service_name.hpp"
#include "utils/trim.hpp"

namespace livekit_ros2_bridge::protocol::echo_once
{

using Json = nlohmann::json;

namespace
{

constexpr char kPayloadField[] = "payload";
constexpr char kKindField[] = "kind";
constexpr char kNameField[] = "name";
constexpr char kResultField[] = "result";
constexpr char kTopicExpansionNodeName[] = "livekit_ros2_bridge";
constexpr char kTopicExpansionNamespace[] = "/";

}  // namespace

EchoOnceRequest parse(const std::string & payload)
{
  Json body;
  try {
    body = detail::parseObject(payload, "Invalid JSON in echo-once request", "Echo-once request must be a JSON object");
  } catch (const std::invalid_argument & exc) {
    throw ValidationError(kPayloadField, exc.what());
  }

  EchoOnceRequest request;

  const auto kind_field = body.find(kKindField);
  if (kind_field == body.end() || !kind_field->is_string()) {
    throw ValidationError(kKindField, "echo-once 'kind' must be a string");
  }
  // Only "topic" is implemented; reject every other (including future) kind explicitly so the
  // caller learns immediately rather than receiving a silent `none`.
  if (trim(kind_field->get_ref<const std::string &>()) != "topic") {
    throw ValidationError(kKindField, "echo-once 'kind' must be 'topic'");
  }
  request.kind = SubscriptionTargetKind::Topic;

  const auto name_field = body.find(kNameField);
  if (name_field == body.end() || !name_field->is_string()) {
    throw ValidationError(kNameField, "echo-once 'name' must be a string");
  }
  try {
    request.name = rclcpp::expand_topic_or_service_name(
      trim(name_field->get_ref<const std::string &>()), kTopicExpansionNodeName, kTopicExpansionNamespace);
  } catch (const std::exception & exc) {
    throw ValidationError(kNameField, exc.what());
  }

  return request;
}

std::string serialize(EchoOnceResult result)
{
  switch (result) {
    case EchoOnceResult::Sent:
      return Json{{kResultField, "sent"}}.dump();
    case EchoOnceResult::None:
      return Json{{kResultField, "none"}}.dump();
  }
  throw std::invalid_argument("echo-once result is invalid");
}

}  // namespace livekit_ros2_bridge::protocol::echo_once
