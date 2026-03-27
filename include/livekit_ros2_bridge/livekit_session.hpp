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

#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace livekit_ros2_bridge
{

struct RpcInvocation
{
  std::string request_id;
  std::string caller_identity;
  std::string payload;
  double response_timeout_sec = 0.0;
};

class BridgeRpcError : public std::runtime_error
{
public:
  BridgeRpcError(std::uint32_t code, std::string message)
  : std::runtime_error(std::move(message))
  , code_(code)
  {}

  std::uint32_t code() const noexcept
  {
    return code_;
  }

private:
  std::uint32_t code_;
};

using RpcHandler = std::function<std::optional<std::string>(const RpcInvocation &)>;

class LiveKitSession
{
public:
  virtual ~LiveKitSession() = default;

  virtual bool connect(const std::string & url, const std::string & token) = 0;
  virtual bool registerRpcMethod(const std::string & method_name, RpcHandler handler) = 0;
  virtual void disconnect() = 0;
};

std::unique_ptr<LiveKitSession> makeLiveKitSession();

}  // namespace livekit_ros2_bridge
