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
#include <utility>

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

TopicPublishCommand parseTopicPublishCommand(const std::vector<std::uint8_t> & command_payload)
{
  Json body;
  try {
    body = Json::parse(command_payload.begin(), command_payload.end());
  } catch (const Json::exception & exc) {
    throw std::invalid_argument(std::string("Invalid publish command JSON: ") + exc.what());
  }

  if (!body.is_object()) {
    throw std::invalid_argument("Publish command must be a JSON object.");
  }

  auto cdr_payload = parseCdrPayload(body, "message");
  if (cdr_payload.empty()) {
    throw std::invalid_argument("Publish command requires a non-empty message.payload_base64 field.");
  }

  const auto topic_it = body.find("topic");
  if (topic_it == body.end() || !topic_it->is_string()) {
    throw std::invalid_argument("Publish command requires a string 'topic' field.");
  }

  std::string topic = normalizeRosResourceName(topic_it->get_ref<const std::string &>());
  if (topic.empty()) {
    throw std::invalid_argument("Publish command requires a non-empty 'topic' field.");
  }

  const std::string interface_type = parseRequiredNonEmptyTrimmedStringField(
    body, "interface_type", "Publish command requires a non-empty 'interface_type' field.");
  return TopicPublishCommand{
    std::move(topic),
    std::move(interface_type),
    std::move(cdr_payload),
  };
}

}  // namespace livekit_ros2_bridge
