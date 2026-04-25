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

#include "protocol/topic_publish_json.hpp"

#include <stdexcept>

#include "nlohmann/json.hpp"
#include "protocol/cdr.hpp"
#include "protocol/detail/json_fields.hpp"
#include "protocol/validation_error.hpp"

namespace livekit_ros2_bridge::protocol::topic_publish
{

RosPublishRequest parse(const std::vector<std::uint8_t> & payload)
{
  nlohmann::json body;
  try {
    body = protocol::detail::parseObject(
      payload, "Invalid JSON in publish request", "Publish request must be a JSON object");
  } catch (const std::invalid_argument & exc) {
    throw ValidationError("payload", exc.what());
  }

  RosPublishRequest request;
  try {
    request.ros_topic = protocol::detail::requiredString(
      body,
      "topic",
      "Publish request requires a string 'topic' field.",
      "Publish request requires a non-empty 'topic' field.");
  } catch (const std::invalid_argument & exc) {
    throw ValidationError("topic", exc.what());
  }

  try {
    request.interface_type = protocol::detail::requiredString(
      body, "interface_type", "Publish request requires a non-empty 'interface_type' field.");
  } catch (const std::invalid_argument & exc) {
    throw ValidationError("interface_type", exc.what());
  }

  try {
    request.message = protocol::cdr::parse(body, protocol::cdr::Field::Message);
  } catch (const std::invalid_argument & exc) {
    throw ValidationError("message", exc.what());
  }

  // Empty CDR would otherwise reach ROS as a default-constructed message.
  if (request.message.size() == 0U) {
    throw ValidationError("message", "Publish request requires a non-empty message.payload_base64 field.");
  }

  return request;
}

}  // namespace livekit_ros2_bridge::protocol::topic_publish
