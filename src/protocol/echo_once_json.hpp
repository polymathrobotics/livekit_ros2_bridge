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

#include "protocol/echo_once.hpp"

namespace livekit_ros2_bridge::protocol::echo_once
{

/// Parses a `{kind, name}` echo-once request. Only `kind:"topic"` is supported; any other
/// (including future) kind is a validation error so unsupported requests fail loudly rather than
/// returning a misleading `none`. Topic names are expanded with rclcpp grammar, like heartbeats.
EchoOnceRequest parse(const std::string & payload);

std::string serialize(EchoOnceResult result);

}  // namespace livekit_ros2_bridge::protocol::echo_once
