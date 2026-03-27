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

#include "livekit_ros2_bridge/rpc_controller.hpp"

#include <algorithm>
#include <cctype>
#include <string_view>

#include "livekit_ros2_bridge/protocol.hpp"
#include "nlohmann/json.hpp"
#include "rclcpp/logging.hpp"

namespace livekit_ros2_bridge
{

namespace
{

using Json = nlohmann::json;

std::string trim(std::string_view value)
{
  const auto begin =
    std::find_if_not(value.begin(), value.end(), [](unsigned char ch) { return std::isspace(ch) != 0; });
  const auto end =
    std::find_if_not(value.rbegin(), value.rend(), [](unsigned char ch) { return std::isspace(ch) != 0; }).base();
  if (begin >= end) {
    return "";
  }
  return std::string(begin, end);
}

[[noreturn]] void throwInvalidRequest()
{
  throw BridgeRpcError(protocol::kRpcErrorInvalidRequest, "Invalid request payload");
}

Json parseRpcObject(const std::string & payload)
{
  try {
    Json json = Json::parse(payload);
    if (!json.is_object()) {
      throwInvalidRequest();
    }
    return json;
  } catch (const Json::exception &) {
    throwInvalidRequest();
  }
}

std::string parseRequiredResourceName(const Json & payload, const char * field_name)
{
  const auto it = payload.find(field_name);
  if (it == payload.end() || !it->is_string()) {
    throwInvalidRequest();
  }
  const std::string normalized = normalizeRosResourceName(trim(it->get_ref<const std::string &>()));
  if (normalized.empty()) {
    throwInvalidRequest();
  }
  return normalized;
}

void validateRequiredObject(const Json & payload, const char * field_name)
{
  const auto it = payload.find(field_name);
  if (it == payload.end() || !it->is_object()) {
    throwInvalidRequest();
  }
}

void requireCallerIdentity(const RpcInvocation & invocation)
{
  if (!invocation.caller_identity.empty()) {
    return;
  }
  throw BridgeRpcError(protocol::kRpcErrorUnauthorized, "caller_identity is required for this RPC");
}

[[noreturn]] void throwNotImplemented(const std::string & method_name)
{
  throw BridgeRpcError(protocol::kRpcErrorInternal, method_name + " is not implemented yet");
}

}  // namespace

RpcController::RpcController(rclcpp::Logger logger, const Params & params)
: logger_(std::move(logger))
, access_policy_(
    params.access.rules.subscribe.allow,
    params.access.rules.subscribe.deny,
    params.access.rules.service.allow,
    params.access.rules.service.deny)
{}

void RpcController::registerMethods(LiveKitSession & session)
{
  if (methods_registered_) {
    return;
  }

  const auto makeNotImplementedHandler = [](const std::string & method_name) -> RpcHandler {
    return [method_name](const RpcInvocation &) -> std::optional<std::string> { throwNotImplemented(method_name); };
  };

  if (!session.registerRpcMethod(
        protocol::kRpcTopicSubscribe,
        [this](const RpcInvocation & invocation) { return handleTopicSubscribe(invocation); }))
  {
    RCLCPP_ERROR(logger_, "Failed to register RPC method %s", protocol::kRpcTopicSubscribe);
  }

  if (!session.registerRpcMethod(
        protocol::kRpcTopicUnsubscribe, makeNotImplementedHandler(protocol::kRpcTopicUnsubscribe)))
  {
    RCLCPP_ERROR(logger_, "Failed to register RPC method %s", protocol::kRpcTopicUnsubscribe);
  }

  if (!session.registerRpcMethod(
        protocol::kRpcServiceCall, [this](const RpcInvocation & invocation) { return handleServiceCall(invocation); }))
  {
    RCLCPP_ERROR(logger_, "Failed to register RPC method %s", protocol::kRpcServiceCall);
  }

  methods_registered_ = true;
}

void RpcController::unregisterMethods(LiveKitSession & session)
{
  if (!methods_registered_) {
    return;
  }

  if (!session.unregisterRpcMethod(protocol::kRpcTopicSubscribe)) {
    RCLCPP_ERROR(logger_, "Failed to unregister RPC method %s", protocol::kRpcTopicSubscribe);
  }

  if (!session.unregisterRpcMethod(protocol::kRpcTopicUnsubscribe)) {
    RCLCPP_ERROR(logger_, "Failed to unregister RPC method %s", protocol::kRpcTopicUnsubscribe);
  }

  if (!session.unregisterRpcMethod(protocol::kRpcServiceCall)) {
    RCLCPP_ERROR(logger_, "Failed to unregister RPC method %s", protocol::kRpcServiceCall);
  }

  methods_registered_ = false;
}

std::optional<std::string> RpcController::handleTopicSubscribe(const RpcInvocation & invocation) const
{
  requireCallerIdentity(invocation);
  const Json payload = parseRpcObject(invocation.payload);
  const std::string topic = parseRequiredResourceName(payload, "topic");

  if (!access_policy_.authorize(AccessOperation::Subscribe, topic)) {
    throw BridgeRpcError(protocol::kRpcErrorForbidden, "ROS topic '" + topic + "' not permitted.");
  }

  throwNotImplemented(protocol::kRpcTopicSubscribe);
}

std::optional<std::string> RpcController::handleServiceCall(const RpcInvocation & invocation) const
{
  requireCallerIdentity(invocation);
  const Json payload = parseRpcObject(invocation.payload);
  const std::string service = parseRequiredResourceName(payload, "service");
  validateRequiredObject(payload, "request");

  if (!access_policy_.authorize(AccessOperation::CallService, service)) {
    throw BridgeRpcError(protocol::kRpcErrorForbidden, "ROS service '" + service + "' not permitted.");
  }

  throwNotImplemented(protocol::kRpcServiceCall);
}

}  // namespace livekit_ros2_bridge
