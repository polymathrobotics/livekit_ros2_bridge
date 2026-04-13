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

// These helpers sit on a protocol boundary. They intentionally reject lenient base64
// variants that some decoders normalize, because the bridge treats canonical padded
// RFC 4648 base64 as part of the wire contract.
enum class Base64Status
{
  kOk,
  // Input is not canonical padded standard base64: wrong alphabet, misplaced '=',
  // or non-zero pad bits in the final quantum.
  kInvalidEncoding,
  // Input failed quartet alignment after basic alphabet/padding placement checks, which
  // callers report as a missing-padding validation error.
  kMissingPadding,
};

struct Base64DecodeResult
{
  // Populated only when `status == kOk`; failed decodes do not expose partial output.
  std::vector<std::uint8_t> bytes;
  Base64Status status = Base64Status::kOk;

  explicit operator bool() const noexcept
  {
    return status == Base64Status::kOk;
  }
};

/// Encode raw bytes as padded standard base64. `bytes` may be null only when
/// `size == 0`.
std::string encodeBase64(const std::uint8_t * bytes, std::size_t size);

/// Decode padded standard base64 without accepting whitespace or unpadded variants.
/// Returns `kMissingPadding` separately when the input is not quartet-aligned after
/// basic validation so higher-level payload parsers can surface a more specific error.
Base64DecodeResult decodeBase64(std::string_view base64);

}  // namespace livekit_ros2_bridge
