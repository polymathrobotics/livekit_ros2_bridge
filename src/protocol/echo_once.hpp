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

#include "protocol/subscriptions.hpp"

namespace livekit_ros2_bridge
{

// "Request a topic's last message" — the generic, client-triggered primitive behind the
// ros2.topic.echo.once RPC.
struct EchoOnceRequest
{
  // Reuses the subscription target-kind discriminant so the primitive can grow new kinds later
  // without a protocol break. Only Topic is implemented today; any other kind is a validation error.
  SubscriptionTargetKind kind = SubscriptionTargetKind::Topic;
  std::string name;
};

// Outcome of an echo-once request.
//
// `None` deliberately collapses two cases the bridge does NOT distinguish:
//   - the topic has no cached value *yet* — the last message may be redelivered to the bridge's
//     subscription moments later (a retry would then succeed), and
//   - the topic will *never* have a cached value — it is volatile, video, or otherwise not cached.
// The client handles both identically: retry a few times, then stop. Disambiguating them on the
// bridge would buy nothing and require per-topic bookkeeping the bridge intentionally avoids.
enum class EchoOnceResult
{
  Sent,
  None,
};

}  // namespace livekit_ros2_bridge
