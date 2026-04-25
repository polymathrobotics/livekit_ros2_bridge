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
#include "rclcpp/logging.hpp"
#include "utils/log_event.hpp"

namespace livekit_ros2_bridge::protocol::topic_publish
{

namespace
{

const auto kLogger = rclcpp::get_logger("topic_publish_request");

}  // namespace

TopicPublishRequest parse(const std::vector<std::uint8_t> & bytes)
{
  nlohmann::json body;
  try {
    body = nlohmann::json::parse(bytes.begin(), bytes.end());
  } catch (const nlohmann::json::exception & exc) {
    LogEvent(kLogger, "topic_publish_request_rejected").field("reason", "invalid_json").debug();
    throw std::invalid_argument(std::string("Invalid publish request JSON: ") + exc.what());
  }

  if (!body.is_object()) {
    LogEvent(kLogger, "topic_publish_request_rejected").field("reason", "invalid_root").debug();
    throw std::invalid_argument("Publish request must be a JSON object.");
  }

  TopicPublishRequest request;
  try {
    const std::string topic = protocol::detail::requiredTrimmedStringField(
      body,
      "topic",
      "Publish request requires a string 'topic' field.",
      "Publish request requires a non-empty 'topic' field.");
    request.ros_topic = topic;
  } catch (const std::invalid_argument &) {
    LogEvent(kLogger, "topic_publish_request_rejected").field("reason", "invalid_topic").debug();
    throw;
  }

  try {
    request.interface_type = protocol::detail::requiredTrimmedStringField(
      body, "interface_type", "Publish request requires a non-empty 'interface_type' field.");
  } catch (const std::invalid_argument &) {
    LogEvent(kLogger, "topic_publish_request_rejected")
      .field("reason", "invalid_interface_type")
      .field("topic", request.ros_topic)
      .debug();
    throw;
  }

  rclcpp::SerializedMessage message;
  try {
    message = protocol::cdr::parseSerializedMessage(body, protocol::cdr::Field::Message);
  } catch (const std::invalid_argument &) {
    LogEvent(kLogger, "topic_publish_request_rejected")
      .field("reason", "invalid_message")
      .field("topic", request.ros_topic)
      .debug();
    throw;
  }

  // Empty CDR would otherwise reach ROS as a default-constructed message.
  if (message.size() == 0U) {
    LogEvent(kLogger, "topic_publish_request_rejected")
      .field("reason", "invalid_message")
      .field("topic", request.ros_topic)
      .debug();
    throw std::invalid_argument("Publish request requires a non-empty message.payload_base64 field.");
  }
  request.message = std::move(message);

  return request;
}

}  // namespace livekit_ros2_bridge::protocol::topic_publish
