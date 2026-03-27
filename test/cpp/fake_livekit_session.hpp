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

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "livekit_ros2_bridge/livekit_session.hpp"

namespace livekit_ros2_bridge
{

struct FakeLiveKitSessionState
{
  std::vector<std::string> registered_methods;
  std::vector<std::string> unregistered_methods;
  std::vector<std::string> events;
  std::map<std::string, RpcHandler> handlers;
};

class FakeLiveKitSession final : public LiveKitSession
{
public:
  FakeLiveKitSession()
  : state(std::make_shared<FakeLiveKitSessionState>())
  {}

  bool connect(const std::string &, const std::string &) override
  {
    return true;
  }

  bool registerRpcMethod(const std::string & method_name, RpcHandler handler) override
  {
    state->registered_methods.push_back(method_name);
    state->handlers[method_name] = std::move(handler);
    return true;
  }

  bool unregisterRpcMethod(const std::string & method_name) override
  {
    state->events.push_back("unregister:" + method_name);
    state->unregistered_methods.push_back(method_name);
    state->handlers.erase(method_name);
    return true;
  }

  void disconnect() override
  {
    state->events.push_back("disconnect");
  }

  std::shared_ptr<FakeLiveKitSessionState> state;
};

}  // namespace livekit_ros2_bridge
