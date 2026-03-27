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

#include "livekit_ros2_bridge/livekit_session.hpp"

#include <exception>
#include <memory>
#include <optional>
#include <utility>

#include "livekit/livekit.h"
#include "livekit/rpc_error.h"
#include "livekit_ros2_bridge/protocol.hpp"
#include "rclcpp/logging.hpp"

namespace livekit_ros2_bridge
{

namespace
{

livekit::LocalParticipant::RpcHandler makeLiveKitRpcHandler(
  const rclcpp::Logger & logger, const std::string & method_name, RpcHandler handler)
{
  return [logger, method_name, handler = std::move(handler)](
           const livekit::RpcInvocationData & invocation) -> std::optional<std::string> {
    const char * caller_identity =
      invocation.caller_identity.empty() ? "<unknown>" : invocation.caller_identity.c_str();
    RCLCPP_INFO(
      logger,
      "Received RPC method %s request_id=%s caller_identity=%s payload_bytes=%zu",
      method_name.c_str(),
      invocation.request_id.c_str(),
      caller_identity,
      invocation.payload.size());

    try {
      return handler(
        RpcInvocation{
          invocation.request_id,
          invocation.caller_identity,
          invocation.payload,
          invocation.response_timeout_sec,
        });
    } catch (const BridgeRpcError & exc) {
      throw livekit::RpcError(exc.code(), exc.what());
    } catch (const std::exception & exc) {
      RCLCPP_ERROR(logger, "RPC method %s failed: %s", method_name.c_str(), exc.what());
      throw livekit::RpcError(protocol::kRpcErrorInternal, "Internal error handling RPC method");
    } catch (...) {
      RCLCPP_ERROR(logger, "RPC method %s failed with unknown exception", method_name.c_str());
      throw livekit::RpcError(protocol::kRpcErrorInternal, "Internal error handling RPC method");
    }
  };
}

class LiveKitRoomSession final : public LiveKitSession
{
public:
  LiveKitRoomSession()
  : logger_(rclcpp::get_logger("livekit_ros2_bridge.livekit_session"))
  {}

  ~LiveKitRoomSession() override
  {
    disconnect();
  }

  bool connect(const std::string & url, const std::string & token) override
  {
    if (room_ != nullptr) {
      return true;
    }

    // `initialize()` returning false means LiveKit is already running elsewhere
    // in this process. Treat multiple sessions as unsupported and fail fast for now.
    if (!livekit::initialize()) {
      RCLCPP_ERROR(logger_, "Failed to initialize LiveKit");
      return false;
    }
    livekit_initialized_ = true;
    RCLCPP_INFO(logger_, "LiveKit initialized");

    auto room = std::make_unique<livekit::Room>();
    livekit::RoomOptions room_options;
    room_options.auto_subscribe = true;

    RCLCPP_INFO(logger_, "Connecting to LiveKit room at %s", url.c_str());
    try {
      if (!room->Connect(url, token, room_options)) {
        RCLCPP_ERROR(logger_, "Failed to connect to LiveKit room");
        disconnect();
        return false;
      }
    } catch (const std::exception & exc) {
      RCLCPP_ERROR(logger_, "LiveKit room connect failed: %s", exc.what());
      disconnect();
      return false;
    } catch (...) {
      RCLCPP_ERROR(logger_, "LiveKit room connect failed with unknown exception");
      disconnect();
      return false;
    }

    if (room->localParticipant() == nullptr) {
      RCLCPP_ERROR(logger_, "LiveKit local participant unavailable");
      disconnect();
      return false;
    }

    room_ = std::move(room);
    const livekit::RoomInfoData room_info = room_->room_info();
    const char * room_name = room_info.name.empty() ? "<unknown>" : room_info.name.c_str();
    const char * room_sid = room_info.sid.has_value() ? room_info.sid->c_str() : "<unknown>";
    const std::string & identity = room_->localParticipant()->identity();
    const char * participant_identity = identity.empty() ? "<unknown>" : identity.c_str();
    RCLCPP_INFO(
      logger_, "Connected to LiveKit room name=%s sid=%s identity=%s", room_name, room_sid, participant_identity);
    return true;
  }

  bool registerRpcMethod(const std::string & method_name, RpcHandler handler) override
  {
    if (room_ == nullptr) {
      return false;
    }

    auto * participant = room_->localParticipant();
    if (participant == nullptr) {
      RCLCPP_ERROR(logger_, "Cannot register RPC method %s: local participant unavailable", method_name.c_str());
      return false;
    }

    try {
      participant->registerRpcMethod(method_name, makeLiveKitRpcHandler(logger_, method_name, std::move(handler)));
      return true;
    } catch (const std::exception & exc) {
      RCLCPP_ERROR(logger_, "Failed to register RPC method %s: %s", method_name.c_str(), exc.what());
      return false;
    }
  }

  void disconnect() override
  {
    room_.reset();
    if (livekit_initialized_) {
      livekit::shutdown();
      livekit_initialized_ = false;
    }
  }

private:
  rclcpp::Logger logger_;
  std::unique_ptr<livekit::Room> room_;
  bool livekit_initialized_ = false;
};

}  // namespace

std::unique_ptr<LiveKitSession> makeLiveKitSession()
{
  return std::make_unique<LiveKitRoomSession>();
}

}  // namespace livekit_ros2_bridge
