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

#include "topic_publish_command.hpp"

#include <stdexcept>
#include <string_view>
#include <utility>

#include "nlohmann/json.hpp"
#include "payloads/cdr_payload.hpp"
#include "payloads/json_object_parser.hpp"
#include "rclcpp/logging.hpp"
#include "utils/log_event.hpp"
#include "utils/ros_resource_name_utils.hpp"

namespace livekit_ros2_bridge
{

namespace
{

constexpr char kTopicFieldError[] = "Publish command requires a string 'topic' field.";
constexpr char kTopicEmptyError[] = "Publish command requires a non-empty 'topic' field.";
constexpr char kInterfaceTypeError[] = "Publish command requires a non-empty 'interface_type' field.";
constexpr char kMessagePayloadError[] = "Publish command requires a non-empty message.payload_base64 field.";
const auto kLogger = rclcpp::get_logger("topic_publish_command");

void logRejectedCommand(std::string_view reason, std::string_view topic = {})
{
  auto event = LogEvent(kLogger, "topic_publish_command_rejected");
  event.field("reason", reason);
  if (!topic.empty()) {
    event.field("topic", topic);
  }
  event.debug();
}

}  // namespace

TopicPublishCommand parseTopicPublishCommand(const std::vector<std::uint8_t> & bytes)
{
  nlohmann::json body;
  try {
    body = nlohmann::json::parse(bytes.begin(), bytes.end());
  } catch (const nlohmann::json::exception & exc) {
    logRejectedCommand("invalid_json");
    throw std::invalid_argument(std::string("Invalid publish command JSON: ") + exc.what());
  }

  if (!body.is_object()) {
    logRejectedCommand("invalid_root");
    throw std::invalid_argument("Publish command must be a JSON object.");
  }

  // ControlPacketRouter already emits the operator-facing warn logs for invalid publish packets,
  // including the exact error text. Keep parser-local logs at debug level so local troubleshooting
  // can distinguish which contract boundary rejected the command without repeating that detail.
  std::string topic;
  try {
    topic = normalizeRosResourceName(
      parseRequiredNonEmptyTrimmedStringField(body, "topic", kTopicFieldError, kTopicEmptyError));
  } catch (const std::invalid_argument &) {
    logRejectedCommand("invalid_topic");
    throw;
  }

  std::string interface_type;
  try {
    interface_type = parseRequiredNonEmptyTrimmedStringField(body, "interface_type", kInterfaceTypeError);
  } catch (const std::invalid_argument &) {
    logRejectedCommand("invalid_interface_type", topic);
    throw;
  }

  std::vector<std::uint8_t> cdr;
  try {
    cdr = cdr_payload::parse(body, "message");
  } catch (const std::invalid_argument &) {
    logRejectedCommand("invalid_message", topic);
    throw;
  }

  // Reject an empty decoded CDR blob instead of treating it as an implicit
  // default-constructed message instance.
  if (cdr.empty()) {
    logRejectedCommand("invalid_message", topic);
    throw std::invalid_argument(kMessagePayloadError);
  }

  // Normalize topic names here so policy checks, publisher lookup, and logs see one resource
  // spelling. Keep `interface_type` trimmed-but-exact because publish-time validation compares it
  // against the ROS graph.
  return TopicPublishCommand{
    std::move(topic),
    std::move(interface_type),
    std::move(cdr),
  };
}

}  // namespace livekit_ros2_bridge
