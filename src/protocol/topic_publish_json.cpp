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
#include <string>
#include <utility>

#include "nlohmann/json.hpp"
#include "protocol/cdr.hpp"
#include "protocol/detail/json_fields.hpp"
#include "protocol/validation_error.hpp"

namespace livekit_ros2_bridge::protocol::topic_publish
{

namespace
{

constexpr char kPayloadField[] = "payload";
constexpr char kTopicField[] = "topic";
constexpr char kInterfaceTypeField[] = "interface_type";
constexpr char kMessageField[] = "message";

}  // namespace

TopicPublishRequest parse(const std::vector<std::uint8_t> & bytes)
{
  nlohmann::json body;
  try {
    body =
      protocol::detail::parseObject(bytes, "Invalid JSON in publish request", "Publish request must be a JSON object");
  } catch (const std::invalid_argument & exc) {
    throw ValidationError(kPayloadField, exc.what());
  }

  TopicPublishRequest request;
  try {
    const std::string topic = protocol::detail::requiredTrimmedStringField(
      body,
      "topic",
      "Publish request requires a string 'topic' field.",
      "Publish request requires a non-empty 'topic' field.");
    request.ros_topic = topic;
  } catch (const std::invalid_argument & exc) {
    throw ValidationError(kTopicField, exc.what());
  }

  try {
    request.interface_type = protocol::detail::requiredTrimmedStringField(
      body, "interface_type", "Publish request requires a non-empty 'interface_type' field.");
  } catch (const std::invalid_argument & exc) {
    throw ValidationError(kInterfaceTypeField, exc.what());
  }

  rclcpp::SerializedMessage message;
  try {
    message = protocol::cdr::parseSerializedMessage(body, protocol::cdr::Field::Message);
  } catch (const std::invalid_argument & exc) {
    throw ValidationError(kMessageField, exc.what());
  }

  // Empty CDR would otherwise reach ROS as a default-constructed message.
  if (message.size() == 0U) {
    throw ValidationError(kMessageField, "Publish request requires a non-empty message.payload_base64 field.");
  }
  request.message = std::move(message);

  return request;
}

}  // namespace livekit_ros2_bridge::protocol::topic_publish
