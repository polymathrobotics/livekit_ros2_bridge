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

#include <gst/gst.h>

#include <string>

namespace livekit_ros2_bridge::utils
{

struct EndpointCounts
{
  guint appsrc = 0;
  guint appsink = 0;
  guint bridge_appsrc = 0;
  guint bridge_appsink = 0;
};

struct EndpointLayout
{
  guint appsrc = 0;
  guint appsink = 0;
  guint bridge_appsrc = 0;
  guint bridge_appsink = 0;
  const char * bridge_appsrc_name = nullptr;
  const char * bridge_appsink_name = nullptr;

  bool matches(const EndpointCounts & counts) const noexcept;
  bool hasUserEndpoint(const EndpointCounts & counts) const noexcept;
};

// Parses `description` with gst_parse_launch and rejects any pipeline that
// defines appsrc/appsink endpoints beyond the bridge-owned ones named by the
// layout. Shared by the video and audio config loaders so both media kinds
// validate fragments identically at startup.
void validatePipeline(const std::string & context, const std::string & description, const EndpointLayout & layout);

// Other-source fragments must provide exactly one appsink, which the bridge
// recovers by the reserved name passed here (per media kind).
constexpr EndpointLayout makeOtherSourceLayout(const char * bridge_appsink_name)
{
  return EndpointLayout{0U, 1U, 0U, 1U, nullptr, bridge_appsink_name};
}

}  // namespace livekit_ros2_bridge::utils
