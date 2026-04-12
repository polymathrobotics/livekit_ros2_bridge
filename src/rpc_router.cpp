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
#include <cstring>
#include <exception>
#include <future>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
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
#include "utils/log_event.hpp"

namespace livekit_ros2_bridge
{

namespace
{

const auto kRpcRouterLogger = rclcpp::get_logger("rpc_router");
constexpr char kUnknownLogValue[] = "<unknown>";

using GraphResourceMap = std::map<std::string, std::vector<std::string>>;

[[noreturn]] void rethrowAsRpcHandlerError(const std::exception & exc);

const char * requestIdForLog(const RpcInvocation & invocation)
{
  return invocation.request_id.empty() ? kUnknownLogValue : invocation.request_id.c_str();
}

const char * requesterIdentityForLog(const RpcInvocation & invocation)
{
  return invocation.caller_identity.empty() ? kUnknownLogValue : invocation.caller_identity.c_str();
}

const char * resourceForMethod(const char * method_name)
{
  if (std::strcmp(method_name, protocol::kRpcServiceCall) == 0) {
    return "services";
  }
  if (std::strcmp(method_name, protocol::kRpcInterfacesGet) == 0) {
    return "interfaces";
  }
  if (std::strcmp(method_name, protocol::kRpcServicesList) == 0) {
    return "services";
  }
  if (std::strcmp(method_name, protocol::kRpcTopicsList) == 0) {
    return "topics";
  }
  return nullptr;
}

std::uint32_t rpcErrorCodeForException(const std::exception & exc)
{
  if (const auto * rpc_handler_error = dynamic_cast<const RpcHandlerError *>(&exc)) {
    return rpc_handler_error->code();
  }
  if (
    dynamic_cast<const std::invalid_argument *>(&exc) != nullptr ||
    dynamic_cast<const std::out_of_range *>(&exc) != nullptr)
  {
    return protocol::kRpcErrorInvalidRequest;
  }
  return protocol::kRpcErrorInternal;
}

const char * rpcReasonForCode(std::uint32_t code)
{
  switch (code) {
    case protocol::kRpcErrorInvalidRequest:
      return "invalid_request";
    case protocol::kRpcErrorUnauthorized:
      return "unauthorized";
    case protocol::kRpcErrorForbidden:
      return "forbidden";
    case protocol::kRpcErrorInternal:
    default:
      return "internal";
  }
}

[[noreturn]] void logAndRethrowRpcHandlerError(
  const RpcInvocation & invocation,
  const char * method_name,
  const char * resource,
  const char * service,
  const std::exception & exc)
{
  const auto code = rpcErrorCodeForException(exc);
  const char * reason = rpcReasonForCode(code);
  const char * request_resource = resource != nullptr ? resource : resourceForMethod(method_name);
  LogEvent event(kRpcRouterLogger, code == protocol::kRpcErrorInternal ? "rpc_request_failed" : "rpc_request_rejected");
  event.field("reason", reason)
    .field("method", method_name)
    .field("request_id", requestIdForLog(invocation))
    .field("requester_identity", requesterIdentityForLog(invocation));

  if (service != nullptr) {
    event.field("resource", request_resource == nullptr ? kUnknownLogValue : request_resource)
      .field("service", service)
      .field("error", exc.what());
  } else if (request_resource != nullptr) {
    event.field("resource", request_resource).field("error", exc.what());
  } else {
    event.field("error", exc.what());
  }

  if (code == protocol::kRpcErrorInternal) {
    event.error();
  } else {
    event.warn();
  }

  rethrowAsRpcHandlerError(exc);
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
    const bool query_matches_resource = !request.query.has_value() ||
                                        resource_name.find(*request.query) != std::string::npos ||
                                        interface_type.find(*request.query) != std::string::npos;
    if (!query_matches_resource) {
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
std::optional<std::string> handleRpcWithCallerIdentity(
  const RpcInvocation & invocation, const char * method_name, HandleRpcT handle_rpc)
{
  // Caller identity is part of the authorization model, so reject anonymous
  // invocations before parsing payloads or touching bridge state.
  if (invocation.caller_identity.empty()) {
    const char * resource = resourceForMethod(method_name);
    if (resource != nullptr) {
      LogEvent(kRpcRouterLogger, "rpc_request_rejected")
        .field("reason", "unauthorized")
        .field("method", method_name)
        .field("request_id", requestIdForLog(invocation))
        .field("requester_identity", requesterIdentityForLog(invocation))
        .field("resource", resource)
        .field("error", "caller_identity_required")
        .warn();
    } else {
      LogEvent(kRpcRouterLogger, "rpc_request_rejected")
        .field("reason", "unauthorized")
        .field("method", method_name)
        .field("request_id", requestIdForLog(invocation))
        .field("requester_identity", requesterIdentityForLog(invocation))
        .field("error", "caller_identity_required")
        .warn();
    }
    throw RpcHandlerError(protocol::kRpcErrorUnauthorized, "caller_identity is required for this RPC");
  }

  try {
    return handle_rpc();
  } catch (const RpcHandlerError &) {
    throw;
  } catch (const std::exception & exc) {
    logAndRethrowRpcHandlerError(invocation, method_name, nullptr, nullptr, exc);
  }
}

template <typename QueryResourcesT, typename SerializeResponseT>
std::optional<std::string> handleResourceListRpc(
  const RpcInvocation & invocation,
  const char * method_name,
  const char * resource,
  RosExecutorQueue & ros_executor_queue,
  const AccessPolicy & access_policy,
  QueryResourcesT query_resources,
  AccessOperation access_op,
  SerializeResponseT serialize_response)
{
  return handleRpcWithCallerIdentity(
    invocation,
    method_name,
    [&invocation,
     method_name,
     resource,
     &ros_executor_queue,
     &access_policy,
     access_op,
     query_resources = std::move(query_resources),
     serialize_response = std::move(serialize_response)]() mutable {
      try {
        auto request = parseResourceListRequest(invocation.payload);
        // Query the ROS graph on the executor thread, then apply access-policy
        // filtering before serializing the response back to the RPC caller.
        auto future = ros_executor_queue.submit(
          [request, &access_policy, access_op, query_resources = std::move(query_resources)]() mutable {
            return filterResourceListEntries(query_resources(), access_policy, access_op, request);
          });
        return serialize_response(future.get());
      } catch (const std::exception & exc) {
        logAndRethrowRpcHandlerError(invocation, method_name, resource, nullptr, exc);
      }
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

std::array<std::pair<const char *, RpcHandler>, 4> RpcRouter::rpcMethodCatalog()
{
  return {{
    {protocol::kRpcServiceCall, [this](const RpcInvocation & invocation) { return handleServiceCall(invocation); }},
    {protocol::kRpcInterfacesGet, [this](const RpcInvocation & invocation) { return handleInterfacesGet(invocation); }},
    {protocol::kRpcServicesList, [this](const RpcInvocation & invocation) { return handleServiceList(invocation); }},
    {protocol::kRpcTopicsList, [this](const RpcInvocation & invocation) { return handleTopicList(invocation); }},
  }};
}

bool RpcRouter::registerRpcMethods(RoomSession & session)
{
  bool all_registered = true;
  for (const auto & method : rpcMethodCatalog()) {
    if (!session.registerRpcMethod(method.first, method.second)) {
      LogEvent(kRpcRouterLogger, "rpc_method_registration_failed").field("method", method.first).error();
      // Registration is best-effort rather than transactional so one failure
      // does not hide other methods that can still be served on this session.
      all_registered = false;
    }
  }
  return all_registered;
}

void RpcRouter::unregisterRpcMethods(RoomSession & session)
{
  for (const auto & method : rpcMethodCatalog()) {
    if (!session.unregisterRpcMethod(method.first)) {
      LogEvent(kRpcRouterLogger, "rpc_method_unregistration_failed").field("method", method.first).error();
    }
  }
}

std::optional<std::string> RpcRouter::handleServiceCall(const RpcInvocation & invocation)
{
  return handleRpcWithCallerIdentity(invocation, protocol::kRpcServiceCall, [this, &invocation]() {
    ServiceCallRequest request;
    try {
      request = parseServiceCallRequest(invocation.payload);
    } catch (const std::exception & exc) {
      logAndRethrowRpcHandlerError(invocation, protocol::kRpcServiceCall, nullptr, nullptr, exc);
    }

    if (!access_policy_.allows(AccessOperation::CallService, request.service)) {
      LogEvent(kRpcRouterLogger, "rpc_request_rejected")
        .field("reason", "forbidden")
        .field("method", protocol::kRpcServiceCall)
        .field("request_id", requestIdForLog(invocation))
        .field("requester_identity", requesterIdentityForLog(invocation))
        .field("resource", "services")
        .field("service", request.service)
        .field("error", "service_not_permitted")
        .warn();
      throw RpcHandlerError(protocol::kRpcErrorForbidden, "ROS service '" + request.service + "' not permitted.");
    }

    try {
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
        service_call_response.service,
        service_call_response.interface_type,
        service_call_response.response,
        elapsed_ms);
    } catch (const std::exception & exc) {
      logAndRethrowRpcHandlerError(invocation, protocol::kRpcServiceCall, nullptr, request.service.c_str(), exc);
    }
  });
}

std::optional<std::string> RpcRouter::handleInterfacesGet(const RpcInvocation & invocation)
{
  return handleRpcWithCallerIdentity(invocation, protocol::kRpcInterfacesGet, [this, &invocation]() {
    try {
      auto types = parseRequestedInterfaceTypes(invocation.payload);
      std::set<std::string> seen;
      std::vector<InterfaceDefinition> all_interfaces;
      for (const auto & type : types) {
        for (auto & definition : lookupInterfaceDefinitions(type)) {
          if (seen.insert(definition.interface_type).second) {
            all_interfaces.push_back(std::move(definition));
          }
        }
      }
      return serializeInterfacesResponse(all_interfaces);
    } catch (const std::exception & exc) {
      logAndRethrowRpcHandlerError(invocation, protocol::kRpcInterfacesGet, "interfaces", nullptr, exc);
    }
  });
}

std::optional<std::string> RpcRouter::handleServiceList(const RpcInvocation & invocation)
{
  return handleResourceListRpc(
    invocation,
    protocol::kRpcServicesList,
    "services",
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
    protocol::kRpcTopicsList,
    "topics",
    ros_executor_queue_,
    access_policy_,
    [this]() { return node_.get_topic_names_and_types(); },
    AccessOperation::Subscribe,
    serializeTopicListResponse);
}

}  // namespace livekit_ros2_bridge
