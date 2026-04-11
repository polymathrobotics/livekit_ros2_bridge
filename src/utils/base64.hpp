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

namespace livekit_ros2_bridge
{

enum class Base64DecodeStatus
{
  kOk,
  kInvalidEncoding,
  kMissingPadding,
};

struct Base64DecodeResult
{
  std::vector<std::uint8_t> bytes;
  Base64DecodeStatus status = Base64DecodeStatus::kOk;

  explicit operator bool() const noexcept
  {
    return status == Base64DecodeStatus::kOk;
  }
};

std::string base64Encode(const std::uint8_t * data, std::size_t size);
Base64DecodeResult base64Decode(std::string_view value);

std::string base64UrlEncode(const std::uint8_t * data, std::size_t size);
Base64DecodeResult base64UrlDecode(std::string_view value);

}  // namespace livekit_ros2_bridge
