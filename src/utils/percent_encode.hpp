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

#pragma once

#include <string>
#include <string_view>

namespace livekit_ros2_bridge::utils
{

// Percent-encodes every byte outside the RFC 3986 unreserved set so a
// configured source name can be embedded in a LiveKit track name reversibly.
// Shared by the video and audio other-source track names.
std::string percentEncodeUnreserved(std::string_view name);

}  // namespace livekit_ros2_bridge::utils
