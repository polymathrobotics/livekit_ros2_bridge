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

#include "utils/percent_encode.hpp"

#include <string>

namespace livekit_ros2_bridge::utils
{

namespace
{

constexpr char kHexDigits[] = "0123456789ABCDEF";

bool isUnreservedByte(unsigned char byte)
{
  return (byte >= 'A' && byte <= 'Z') || (byte >= 'a' && byte <= 'z') || (byte >= '0' && byte <= '9') || byte == '-' ||
         byte == '.' || byte == '_' || byte == '~';
}

}  // namespace

std::string percentEncodeUnreserved(std::string_view name)
{
  std::string suffix;
  suffix.reserve(name.size() * 3U);
  for (const char ch : name) {
    const auto byte = static_cast<unsigned char>(ch);
    if (isUnreservedByte(byte)) {
      suffix.push_back(static_cast<char>(byte));
      continue;
    }
    suffix.push_back('%');
    suffix.push_back(kHexDigits[byte >> 4U]);
    suffix.push_back(kHexDigits[byte & 0x0FU]);
  }
  return suffix;
}

}  // namespace livekit_ros2_bridge::utils
