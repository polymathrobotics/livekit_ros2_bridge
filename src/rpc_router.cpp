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

#include "rpc_router.hpp"

#include <array>
#include <chrono>
#include <future>
#include <map>
#include <memory>
#include <set>
#include <utility>
#include <vector>

#include "interface_definition_lookup.hpp"
#include "payloads/interface_payloads.hpp"
#include "payloads/resource_list_payloads.hpp"
#include "payloads/service_call_payloads.hpp"
#include "protocol.hpp"
#include "rclcpp/logging.hpp"
#include "ros_executor_queue.hpp"
#include "ros_service_caller.hpp"

namespace livekit_ros2_bridge
{

namespace
{

const auto kRpcRouterLogger = rclcpp::get_logger("rpc_router");

using GraphResourceMap = std::map<std::string, std::vector<std::string>>;

void requireCallerIdentity(const RpcInvocation & invocation)
{
  if (!invocation.caller_identity.empty()) {
    return;
  }
  throw RpcHandlerError(protocol::kRpcErrorUnauthorized, "caller_identity is required for this RPC");
}

std::vector<ResourceListEntry> filterResourceListEntries(
  const GraphResourceMap & all_graph_resources,
  const AccessPolicy & access_policy,
  AccessOperation access_op,
  const ResourceListRequest & request)
{
  std::vector<ResourceListEntry> entries;

  for (const auto & graph_resource_entry : all_graph_resources) {
    const auto & resource_name = graph_resource_entry.first;
    const auto & interface_types = graph_resource_entry.second;

    if (interface_types.size() != 1U) {
      continue;
    }
    if (!access_policy.allows(access_op, resource_name)) {
      continue;
    }
    const auto & interface_type = interface_types.front();
    if (
      request.query.has_value() && resource_name.find(*request.query) == std::string::npos &&
      interface_type.find(*request.query) == std::string::npos)
    {
      continue;
    }

    entries.push_back({resource_name, interface_type});
    if (request.limit.has_value() && entries.size() >= *request.limit) {
      break;
    }
  }

  return entries;
}

[[noreturn]] void rethrowAsRpcHandlerError(const std::exception & exc)
{
  if (dynamic_cast<const RpcHandlerError *>(&exc) != nullptr) {
    throw;
  }
  // Payload and bounds errors are caller-fixable invalid requests; everything
  // else is reported as an internal bridge failure.
  if (
    dynamic_cast<const std::invalid_argument *>(&exc) != nullptr ||
    dynamic_cast<const std::out_of_range *>(&exc) != nullptr)
  {
    throw RpcHandlerError(protocol::kRpcErrorInvalidRequest, exc.what());
  }
  throw RpcHandlerError(protocol::kRpcErrorInternal, exc.what());
}

template <typename HandleRpcT>
std::optional<std::string> handleRpcWithCallerIdentity(const RpcInvocation & invocation, HandleRpcT handle_rpc)
{
  // Caller identity is part of the authorization model, so reject anonymous
  // invocations before parsing payloads or touching bridge state.
  requireCallerIdentity(invocation);

  try {
    return handle_rpc();
  } catch (const std::exception & exc) {
    rethrowAsRpcHandlerError(exc);
  }
}

template <typename QueryResourcesT, typename SerializeResponseT>
std::optional<std::string> handleResourceListRpc(
  const RpcInvocation & invocation,
  RosExecutorQueue & ros_executor_queue,
  const AccessPolicy & access_policy,
  QueryResourcesT query_resources,
  AccessOperation access_op,
  SerializeResponseT serialize_response)
{
  return handleRpcWithCallerIdentity(
    invocation,
    [&invocation,
     &ros_executor_queue,
     &access_policy,
     access_op,
     query_resources = std::move(query_resources),
     serialize_response = std::move(serialize_response)]() mutable {
      auto request = parseResourceListRequest(invocation.payload);
      // Query the ROS graph on the executor thread, then apply access-policy
      // filtering before serializing the response back to the RPC caller.
      auto future = ros_executor_queue.submit(
        [request, &access_policy, access_op, query_resources = std::move(query_resources)]() mutable {
          return filterResourceListEntries(query_resources(), access_policy, access_op, request);
        });
      return serialize_response(future.get());
    });
}

}  // namespace

RpcRouter::RpcRouter(
  rclcpp::Node & node,
  const AccessPolicy & access_policy,
  RosExecutorQueue & ros_executor_queue,
  RosServiceCaller & ros_service_caller)
: node_(node)
, access_policy_(access_policy)
, ros_executor_queue_(ros_executor_queue)
, ros_service_caller_(ros_service_caller)
{}

const std::array<RpcRouter::RpcMethodBinding, 4> & RpcRouter::rpcMethodBindings()
{
  static const std::array<RpcMethodBinding, 4> methods{{
    {protocol::kRpcServiceCall, &RpcRouter::handleServiceCall},
    {protocol::kRpcInterfacesGet, &RpcRouter::handleInterfacesGet},
    {protocol::kRpcServicesList, &RpcRouter::handleServiceList},
    {protocol::kRpcTopicsList, &RpcRouter::handleTopicList},
  }};
  return methods;
}

bool RpcRouter::registerRpcMethods(RoomSession & session)
{
  bool all_registered = true;
  for (const auto & method : rpcMethodBindings()) {
    // RoomSession keeps the callable after registration, so bind the member
    // function once here and remove it later with unregisterRpcMethods().
    const auto handler = [this, member_handler = method.handler](const RpcInvocation & invocation) {
      return (this->*member_handler)(invocation);
    };
    if (!session.registerRpcMethod(method.name, handler)) {
      RCLCPP_ERROR(kRpcRouterLogger, "event=rpc_method_registration_failed method=%s", method.name);
      // Registration is best-effort rather than transactional so one failure
      // does not hide other methods that can still be served on this session.
      all_registered = false;
    }
  }
  return all_registered;
}

void RpcRouter::unregisterRpcMethods(RoomSession & session)
{
  for (const auto & method : rpcMethodBindings()) {
    if (!session.unregisterRpcMethod(method.name)) {
      RCLCPP_ERROR(kRpcRouterLogger, "Failed to unregister RPC method %s", method.name);
    }
  }
}

std::optional<std::string> RpcRouter::handleServiceCall(const RpcInvocation & invocation)
{
  return handleRpcWithCallerIdentity(invocation, [this, &invocation]() {
    auto request = parseServiceCallRequest(invocation.payload);

    if (!access_policy_.allows(AccessOperation::CallService, request.service)) {
      throw RpcHandlerError(protocol::kRpcErrorForbidden, "ROS service '" + request.service + "' not permitted.");
    }

    const auto start = std::chrono::steady_clock::now();
    // First hop onto the ROS executor thread to create the client/request with
    // node-affine APIs, then wait on the returned service-call future here.
    auto queued_call_future = ros_executor_queue_.submit(
      [this, requester_identity = invocation.caller_identity, request = std::move(request)]() mutable {
        return ros_service_caller_.call(requester_identity, request);
      });
    auto result_future = queued_call_future.get();

    auto service_call_response = result_future.get();
    const int elapsed_ms = static_cast<int>(
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count());
    return serializeServiceCallResponse(
      service_call_response.service, service_call_response.interface_type, service_call_response.response, elapsed_ms);
  });
}

std::optional<std::string> RpcRouter::handleInterfacesGet(const RpcInvocation & invocation)
{
  return handleRpcWithCallerIdentity(invocation, [this, &invocation]() {
    auto types = parseRequestedInterfaceTypes(invocation.payload);
    std::set<std::string> seen;
    std::vector<InterfaceDefinition> all_interfaces;
    for (const auto & type : types) {
      auto definitions = lookupInterfaceDefinitions(type);
      if (seen.insert(definitions.requested.interface_type).second) {
        all_interfaces.push_back(std::move(definitions.requested));
      }
      for (auto & dep : definitions.dependencies) {
        if (seen.insert(dep.interface_type).second) {
          all_interfaces.push_back(std::move(dep));
        }
      }
    }
    return serializeInterfacesResponse(all_interfaces);
  });
}

std::optional<std::string> RpcRouter::handleServiceList(const RpcInvocation & invocation)
{
  return handleResourceListRpc(
    invocation,
    ros_executor_queue_,
    access_policy_,
    [this]() { return node_.get_service_names_and_types(); },
    AccessOperation::CallService,
    serializeServiceListResponse);
}

std::optional<std::string> RpcRouter::handleTopicList(const RpcInvocation & invocation)
{
  return handleResourceListRpc(
    invocation,
    ros_executor_queue_,
    access_policy_,
    [this]() { return node_.get_topic_names_and_types(); },
    AccessOperation::Subscribe,
    serializeTopicListResponse);
}

}  // namespace livekit_ros2_bridge
