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

#include "protocol/cdr.hpp"

#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "nlohmann/json.hpp"
#include "protocol/constants.hpp"
#include "protocol/detail/base64.hpp"

namespace livekit_ros2_bridge::protocol::cdr
{

namespace
{

constexpr char kContentType[] = "content_type";
constexpr char kPayloadBase64[] = "payload_base64";

const char * fieldKey(Field field)
{
  switch (field) {
    case Field::Message:
      return "message";
    case Field::Request:
      return "request";
    case Field::Response:
      return "response";
  }

  throw std::logic_error("Unhandled CDR envelope field.");
}

}  // namespace

std::vector<std::uint8_t> parse(const nlohmann::json & body, Field field)
{
  const char * key = fieldKey(field);

  const auto it = body.find(key);
  if (it == body.end() || !it->is_object()) {
    throw std::invalid_argument(std::string(key) + " must be an object.");
  }

  const auto & envelope = *it;
  const auto content_type = envelope.find(kContentType);
  if (content_type == envelope.end() || !content_type->is_string()) {
    throw std::invalid_argument(std::string(kContentType) + " must be a string.");
  }
  if (content_type->get_ref<const std::string &>() != protocol::kCdrContentType) {
    throw std::invalid_argument(std::string(key) + "." + kContentType + " must be application/x-ros-cdr.");
  }

  // The bridge treats padded standard base64 as part of the protocol contract so malformed payloads
  // fail here instead of reaching downstream ROS deserialization with ambiguous byte contents.
  const auto payload = envelope.find(kPayloadBase64);
  if (payload == envelope.end() || !payload->is_string()) {
    throw std::invalid_argument(std::string(kPayloadBase64) + " must be a string.");
  }
  const auto & encoded = payload->get_ref<const std::string &>();
  switch (auto result = detail::base64::decode(encoded); result.status) {
    case detail::base64::Status::Ok:
      return std::move(result.bytes);
    case detail::base64::Status::MissingPadding:
      throw std::invalid_argument("payload_base64 must be padded standard base64.");
    case detail::base64::Status::InvalidEncoding:
      throw std::invalid_argument("payload_base64 is not valid base64.");
  }

  throw std::logic_error("Unhandled base64 decode status.");
}

nlohmann::json serialize(const std::vector<std::uint8_t> & bytes)
{
  return {
    {kContentType, protocol::kCdrContentType},
    {kPayloadBase64, detail::base64::encode(bytes.data(), bytes.size())},
  };
}

}  // namespace livekit_ros2_bridge::protocol::cdr
