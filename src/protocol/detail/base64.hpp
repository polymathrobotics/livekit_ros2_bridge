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

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace livekit_ros2_bridge::protocol::detail::base64
{

enum class Status
{
  Ok,
  InvalidEncoding,
  MissingPadding,
};

struct Result
{
  // Failed decodes never expose partial output.
  std::vector<std::uint8_t> bytes;
  Status status = Status::Ok;
};

/// `bytes` may be null only when `size == 0`.
std::string encode(const std::uint8_t * bytes, std::size_t size);

/// Decode canonical padded standard base64; valid unpadded input reports `MissingPadding`.
Result decode(std::string_view text);

}  // namespace livekit_ros2_bridge::protocol::detail::base64
